from numpy.random import random_integers
import pygstc
from uuid import UUID, uuid4
from pygstc.gstc import GstcError, GstdError, GstdClient
from pygstc.logger import CustomLogger
import time
import os
import json
from catalog import *
from datetime import datetime
import subprocess
import shutil
import exiftool
from item_utility import gen_thumbnail_factory


# Default camera settings with options/range format
DEFAULT_CAMERA_SETTINGS = {
    "resolution": {
        "options": ["640x480", "1920x1080", "2432x2048"],
        "current_selection": 1
    },
    "framerate": {
        "options": [10, 30, 60],
        "current_selection": 1
    },
    "capture_format": {
        "options": ["JPG", "PNG"],
        "current_selection": 0
    },
    "capture_quality": {
        "options": ["Low", "Mid", "High"],
        "current_selection": 2
    },
    "record_format": {
        "options": ["MP4", "AVI"],
        "current_selection": 0
    },
    "record_bitrate": {
        "range": [1, 15],
        "current_selection": 8
    },
    "preview_quality": {
        "options": ["Low", "Mid", "High"],
        "current_selection": 1
    }
}


class MultiCamApp:
    TEST_MODE = True
    SENSORNAME_MAP = {
        0: "left",
        1: "front",
        2: "right",
    }

    def __init__(self, media_path: str, catalog: Catalog):
        self.media_path = media_path
        self.settings_path = os.path.join(media_path, "camera_settings.json")
        self.settings = self._load_settings()

        # Create Janus streaming plugin config
        janus_cfg_path = f'{media_path}/janus.plugin.streaming.jcfg'
        self._create_janus_config(janus_cfg_path)

        # Start Janus WebRTC Gateway
        self.janus_proc = subprocess.Popen(
            ['/opt/janus/bin/janus', '-C', janus_cfg_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True
        )

        # Wait for Janus to start
        time.sleep(2)

        # start a daemon on a separate process
        self.gstd_proc = subprocess.Popen(
            ['gstd', '-d', f'{media_path}/gst.log', '-l', f'{media_path}/gstd.log',
             '--gst-debug=pngenc:5'],
            start_new_session=True
        )

        self.gstc = self.connect_to_gstd(self.gstd_proc)

        self.current_preview_index = 0
        self.catalog = catalog
        self.pipelines = {}

        self._gst_inited = False

    def __del__(self):
        self.gst_deinit()

        # Kill GStreamer daemon
        if hasattr(self, 'gstd_proc'):
            self.gstd_proc.kill()
            self.gstd_proc.wait()

        # Kill Janus WebRTC Gateway
        if hasattr(self, 'janus_proc'):
            self.janus_proc.kill()
            self.janus_proc.wait()

    def _load_settings(self):
        """Load settings from disk or return defaults."""
        try:
            with open(self.settings_path, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return DEFAULT_CAMERA_SETTINGS.copy()

    def _save_settings(self):
        """Save settings to disk."""
        with open(self.settings_path, 'w') as f:
            json.dump(self.settings, f, indent=2)

    def get_settings(self):
        """Return current settings."""
        return self.settings

    def save_settings(self, new_settings: dict):
        """Update and save settings."""
        self.settings = new_settings
        self._save_settings()
        return self.settings

    def _get_setting_value(self, key: str):
        """Get the actual value of a setting (resolves current_selection)."""
        setting = self.settings.get(key, {})
        if "options" in setting:
            idx = setting.get("current_selection", 0)
            return setting["options"][idx]
        elif "range" in setting:
            return setting.get("current_selection", setting["range"][0])
        return None

    def _create_janus_config(self, cfg_path):
        """Create Janus configuration files in media_path"""
        # Create janus.plugin.streaming.jcfg
        streaming_config = """# Janus Streaming Plugin Configuration
general: {
    #admin_key = "supersecret"
}

# Webcam stream - video only (VP8)
webcam: {
    type = "rtp"
    id = 1
    description = "Live Camera Stream (VP8)"
    metadata = "Multi-camera feed"
    audio = false
    video = true
    videoport = 5004
    videopt = 96
    videocodec = "vp8"
    videortpmap = "VP8/90000"
}
"""
        with open(cfg_path, 'w') as f:
            f.write(streaming_config)

    def connect_to_gstd(self, gstd_proc, timeout=3.0):
        end = time.monotonic() + timeout
        while True:
            if gstd_proc is not None and gstd_proc.poll() is not None:
                raise RuntimeError(f"gstd exited early")
            try:
                return GstdClient()
            except Exception as e:
                if time.monotonic() > end:
                    raise RuntimeError(f"Failed to connect to gstd: {e}")
                time.sleep(0.1)
                continue

    def _construct_caps(self):
        # Get resolution from settings
        resolution_str = self._get_setting_value("resolution") or "1920x1080"
        parts = resolution_str.split("x")
        self.video_width = int(parts[0])
        self.video_height = int(parts[1])

        # In test mode, scale down for performance
        if self.TEST_MODE:
            self.video_width = 320
            self.video_height = 240

        # Get framerate from settings
        video_framerate = self._get_setting_value("framerate") or 30

        video_format = "I420" if self.TEST_MODE else "NV12"
        memory_feature = "(memory:NVMM)" if not self.TEST_MODE else ""
        caps = (
            f"video/x-raw{memory_feature},"
            f"width=(int){self.video_width},height=(int){self.video_height},"
            f"framerate=(fraction){video_framerate}/1,format=(string){video_format}"
        )
        return caps

    def _construct_video_pipeline(self, camera_index: int):
        patterns = ["kx2=1 ky2=1 kt=10", "kxy=1 kt=10", "kx=1 ky=1 kt=10"]
        color = random_integers(0, 4294967295)
        if self.TEST_MODE:
            src_element = f"videotestsrc is-live=true pattern=zone-plate {patterns[camera_index]} foreground-color={color}"
        else:
            src_element = f"nvarguscamerasrc sensor-id={camera_index}"
        pipeline = f"{src_element} name=cam{camera_index} ! {self.caps} ! interpipesink name=isink_cam{camera_index} sync=false"
        return pipeline

    def _construct_composite_pipeline(self):
        cell_w = int(self.video_width/2)
        cell_h = int(self.video_height/2)
        composite_element = "compositor" if self.TEST_MODE else "nvcompositor"
        pipeline = f"{composite_element} name=comp "
        for i in range(3):
            x = 0 if i == 0 else int(cell_w/2) if i == 1 else cell_w
            y = 0 if i == 1 else cell_h
            pipeline += f"sink_{i}::xpos={x} sink_{i}::ypos={y} sink_{i}::width={cell_w} sink_{i}::height={cell_h} "
        pipeline += f"! {self.caps} ! interpipesink name=isink_comp sync=false "

        for i in range(3):
            pipeline += f"interpipesrc listen-to=isink_cam{i} format=time ! queue ! comp.sink_{i} "

        return pipeline

    def _construct_snapshot_pipeline(self, camera_index: int):
        # Get capture settings
        capture_format = (self._get_setting_value("capture_format") or "JPG").upper()
        capture_quality = self._get_setting_value("capture_quality") or "High"

        # Map quality to encoder value
        quality_map = {"Low": 60, "Mid": 80, "High": 100}
        quality_value = quality_map.get(capture_quality, 100)

        sensor_name = self.SENSORNAME_MAP[camera_index]

        if capture_format == "PNG":
            # pngenc needs RGB/RGBA, not I420 - add videoconvert
            encoder_element = "videoconvert ! pngenc"
            sink_location = f"{self.media_path}/{sensor_name}.png"
        else:  # JPG
            if self.TEST_MODE:
                encoder_element = f"jpegenc quality={quality_value}"
            else:
                encoder_element = f"nvjpegenc idct-method=float quality={quality_value}"
            sink_location = f"{self.media_path}/{sensor_name}.jpg"

        pipeline = (
            f"interpipesrc name=isrc_snapshot{camera_index} listen-to=isink_cam{camera_index} num-buffers=1 ! "
            f"{encoder_element} ! filesink location={sink_location}"
        )
        return pipeline

    def _construct_record_pipeline(self, camera_index: int):
        # Get record settings
        record_format = (self._get_setting_value("record_format") or "MP4").upper()
        record_bitrate = self._get_setting_value("record_bitrate") or 8
        bitrate_bps = record_bitrate * 1000000  # Convert Mbps to bps

        sensor_name = self.SENSORNAME_MAP[camera_index]

        # Build encoder and muxer based on format
        if record_format == "AVI":
            if self.TEST_MODE:
                encoder_element = f"x264enc bitrate={record_bitrate * 1000} ! h264parse ! avimux"
            else:
                encoder_element = f"nvv4l2h264enc bitrate={bitrate_bps} ! h264parse ! avimux"
            sink_location = f"{self.media_path}/{sensor_name}.avi"
        else:  # MP4
            if self.TEST_MODE:
                encoder_element = f"x264enc bitrate={record_bitrate * 1000} ! h264parse ! mp4mux"
            else:
                encoder_element = f"nvv4l2h264enc bitrate={bitrate_bps} ! h264parse ! mp4mux"
            sink_location = f"{self.media_path}/{sensor_name}.mp4"

        pipeline = (
            f"interpipesrc name=isrc_record{camera_index} listen-to=isink_cam{camera_index} format=time ! "
            f"queue ! {encoder_element} ! filesink location={sink_location}"
        )
        return pipeline

    def _construct_preview_pipeline(self):
        """
        Constructs RTP/VP8 pipeline for Janus WebRTC streaming.

        Reference pipeline:
        gst-launch-1.0 -v v4l2src device=/dev/video0
          ! image/jpeg,width=1280,height=720,framerate=30/1
          ! jpegdec ! videoconvert
          ! vp8enc deadline=1 target-bitrate=4000000 min-quantizer=4 max-quantizer=20 cpu-used=2
          ! rtpvp8pay ! udpsink host=127.0.0.1 port=5004
        """
        # Get preview quality setting
        preview_quality = self._get_setting_value("preview_quality") or "Mid"

        # Map quality to bitrate (in bps)
        bitrate_map = {"Low": 1000000, "Mid": 4000000, "High": 8000000}
        target_bitrate = bitrate_map.get(preview_quality, 4000000)

        # VP8 encoding parameters for low-latency WebRTC streaming
        vp8_params = f"deadline=1 target-bitrate={target_bitrate} min-quantizer=4 max-quantizer=20 cpu-used=2"

        if self.TEST_MODE:
            # Test mode: interpipesrc → videoconvert → VP8 → RTP → UDP
            pipeline = (
                f"interpipesrc name=isrc_preview0 listen-to=isink_cam0 format=time ! "
                f"videoconvert ! "
                f"vp8enc {vp8_params} ! "
                f"rtpvp8pay ! "
                f"udpsink host=127.0.0.1 port=5004"
            )
        else:
            # Production mode: interpipesrc → nvvidconv → videoconvert → VP8 → RTP → UDP
            # nvvidconv converts NVMM memory to system memory for vp8enc
            pipeline = (
                f"interpipesrc name=isrc_preview0 listen-to=isink_cam0 format=time ! "
                f"nvvidconv ! video/x-raw,format=I420 ! "
                f"videoconvert ! "
                f"vp8enc {vp8_params} ! "
                f"rtpvp8pay ! "
                f"udpsink host=127.0.0.1 port=5004"
            )
        return pipeline

    def shuffle_colors(self):
        for i in range(3):
            color = random_integers(0, 4294967295)
            self.gstc.element_set(
                f"pcam{i}", f"cam{i}", "foreground-color", f"{color}")

    def gst_init(self):
        os.system("rm -f /tmp/preview*")
        os.system(f"rm -f {self.media_path}/snapshot*")
        os.system(f"rm -f {self.media_path}/record*")

        # Build pipelines with current settings
        self.caps = self._construct_caps()
        self.pipelines = {
            **{f"pcam{i}": self._construct_video_pipeline(i) for i in range(3)},
            **{f"psnapshot{i}": self._construct_snapshot_pipeline(i) for i in range(3)},
            **{f"precord{i}": self._construct_record_pipeline(i) for i in range(3)},
            "pcomposite": self._construct_composite_pipeline(),
            "ppreview0": self._construct_preview_pipeline(),
        }

        for pipeline_name, pipeline_desc in self.pipelines.items():
            print(f"Creating {pipeline_name}: {pipeline_desc}")
            self.gstc.pipeline_create(pipeline_name, pipeline_desc)
        if self.TEST_MODE:
            self.shuffle_colors()
        for i in range(3):
            self.gstc.pipeline_play(f"pcam{i}")
            time.sleep(0.5)
        self.gstc.pipeline_play("pcomposite")
        self.gstc.pipeline_play("ppreview0")
        self._gst_inited = True

    def gst_deinit(self):
        if not self._gst_inited:
            return
        for pipeline_name in self.pipelines.keys():
            self.gstc.pipeline_stop(pipeline_name)
            self.gstc.pipeline_delete(pipeline_name)
        self._gst_inited = False

    def setPreviewIndex(self, preview_index: int):
        """
        Set the preview index to the given index.
        """
        if preview_index >= 0 and preview_index < 4:
            self.current_preview_index = preview_index
            target = "isink_comp" if preview_index == 3 else f"isink_cam{preview_index}"
            self.gstc.element_set(
                "ppreview0", "isrc_preview0", "listen-to", target)
            return preview_index
        else:
            return self.current_preview_index

    def capture(self):
        """
        Capture the current frame from all camera, return uuid of the capture.
        """
        # Get capture format from settings
        capture_format = (self._get_setting_value("capture_format") or "JPG").lower()
        ext = "png" if capture_format == "png" else "jpg"

        files = {
            sensor_name: f"{self.media_path}/{sensor_name}.{ext}" for sensor_name in self.SENSORNAME_MAP.values()}

        # Delete old files first
        for f in files.values():
            if os.path.exists(f):
                os.remove(f)

        for i in range(3):
            self.gstc.pipeline_play(f"psnapshot{i}")
            
        time.sleep(0.5)

        for i in range(3):
            self.gstc.pipeline_stop(f"psnapshot{i}")

        thumbnails = [gen_thumbnail_factory(file) for file in files.values()]
        contents = {
            **files,
            "thumbnails": thumbnails,
        }
        self.catalog.add_item("MultiViewImage", contents)

    def recordStart(self):
        """
        Record the current frame from all camera, return uuid of the record.
        """

        for i in range(3):
            self.gstc.pipeline_play(f"precord{i}")

    def recordStop(self):
        """
        Stop the recording and save the record to the media path.
        """

        for i in range(3):
            pname = f"precord{i}"
            self.gstc.event_eos(pname)
            self.gstc.bus_filter(pname, "eos")
            self.gstc.bus_read(pname)
            self.gstc.pipeline_stop(pname)

        # Get record format from settings
        record_format = (self._get_setting_value("record_format") or "MP4").lower()
        ext = "avi" if record_format == "avi" else "mp4"

        files = {
            sensor_name: f"{self.media_path}/{sensor_name}.{ext}" for sensor_name in self.SENSORNAME_MAP.values()
        }
        thumbnails = [gen_thumbnail_factory(file) for file in files.values()]
        contents = {
            **files,
            "thumbnails": thumbnails,
        }
        self.catalog.add_item("MultiViewVideo", contents)
