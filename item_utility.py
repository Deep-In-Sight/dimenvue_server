"""
Item utility functions for catalog management.
Includes thumbnail generation, metadata extraction, and item size calculations.
"""

import cv2
import numpy as np
import os
import subprocess
import json
import exiftool
import tempfile
import struct


# ==================== Thumbnail Generation ====================

def gen_thumbnail_factory(file_path: str, size: tuple[int, int] = (256, 256)) -> str:
    """
    Generate a thumbnail for a file and save to thumbnail.jpg.
    Supports: .jpg, .jpeg, .png, .mp4, .mov, .avi
    """
    ext = os.path.splitext(file_path)[1].lower()
    dst_name = os.path.dirname(file_path) + "thumbnail.jpg"
    if ext in (".jpg", ".jpeg", ".png"):
        img = gen_thumbnail_image(file_path, size)
    elif ext in (".mp4", ".mov", ".avi"):
        img = gen_thumbnail_video(file_path, size)
    else:
        raise ValueError(f"Unsupported file type: {ext}")
    cv2.imwrite(dst_name, img)
    return dst_name


def gen_thumbnail_image(file_path: str, size: tuple[int, int] = (256, 256)) -> np.ndarray:
    """Generate thumbnail from an image file."""
    return cv2.resize(cv2.imread(file_path), size)


def gen_thumbnail_video(video_file: str, size: tuple[int, int] = (256, 256)) -> np.ndarray:
    """Generate thumbnail from the middle frame of a video."""
    cap = cv2.VideoCapture(video_file)
    num_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    frame_index = int(num_frames / 2)
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
    ret, frame = cap.read()
    if not ret:
        raise ValueError(f"Failed to read frame from {video_file}")
    thumbnail = cv2.resize(frame, size)
    cap.release()
    return thumbnail


# ==================== Metadata Extraction ====================

key_maps = {
    'image': [
        ('File:FileName', 'File Name'),
        ('File:FileSize', 'File Size'),
        ('File:FileModifyDate', 'File Modify Date'),
        ('File:FileAccessDate', 'File Access Date'),
        ('File:ImageWidth', 'Image Width'),
        ('File:ImageHeight', 'Image Height'),
        ('Composite:Megapixels', 'Megapixels'),
    ],
    'video': [
        ('File:FileName', 'File Name'),
        ('File:FileSize', 'File Size'),
        ('File:FileModifyDate', 'File Modify Date'),
        ('File:FileAccessDate', 'File Access Date'),
        ('QuickTime:ImageWidth', 'Video Width'),
        ('QuickTime:ImageHeight', 'Video Height'),
        ('QuickTime:MaxBitrate', 'Video Max Bitrate'),
        ('Composite:Megapixels', 'Megapixels'),
        ('QuickTime:VideoFrameRate', 'Video Frame Rate'),
        ('QuickTime:Encoder', 'Video Encoder'),
    ]
}


def get_metadata_factory(file_path: str) -> dict:
    """Get filtered metadata for a file based on its type."""
    ext = os.path.splitext(file_path)[1].lower()
    if ext in (".jpg", ".jpeg", ".png"):
        key_map = key_maps['image']
    elif ext in (".mp4", ".mov", ".avi"):
        key_map = key_maps['video']
    else:
        raise ValueError(f"Unsupported file type: {ext}")
    return get_exiftool_metadata(file_path, key_map)


def get_exiftool_metadata(file_path: str, key_map: list) -> dict:
    """Extract metadata using exiftool and filter to requested keys."""
    with exiftool.ExifToolHelper() as et:
        metadata = et.get_metadata([file_path])[0]
    result = {}
    for old_key, new_key in key_map:
        if old_key in metadata:
            result[new_key] = metadata[old_key]
    return result


# ==================== Item Size Calculations ====================

def get_item_size(item_type: str, item_path: str) -> float | None:
    """
    Get item-specific size metric.

    Returns:
    - Video: Duration in seconds (from front.<ext> video file)
    - Scan: Area in square meters (from metadata.json)
    - Other types: None (no size metric)
    """
    if item_type == "Video":
        # Get duration from front.<ext> video file
        try:
            files = os.listdir(item_path)
            for filename in files:
                if filename.startswith("front."):
                    video_file = os.path.join(item_path, filename)
                    if os.path.exists(video_file):
                        duration = get_video_duration(video_file)
                        if duration is not None:
                            return duration
        except OSError:
            pass
        return None

    elif item_type == "Scan":
        # Get area from metadata.json file
        try:
            metadata_path = os.path.join(item_path, "map_metadata.json")
            if os.path.exists(metadata_path):
                with open(metadata_path, 'r', encoding='utf-8') as f:
                    metadata = json.load(f)
                    area = metadata.get('area')
                    if area is not None:
                        return float(area)
        except (OSError, json.JSONDecodeError, ValueError):
            pass
        return None

    return None


def get_video_duration(video_path: str) -> float | None:
    """
    Get video duration in seconds using ffprobe.
    Falls back to OpenCV if ffprobe is not available.
    """
    # Try ffprobe first (more accurate)
    try:
        result = subprocess.run(
            [
                'ffprobe', '-v', 'quiet', '-print_format', 'json',
                '-show_format', video_path
            ],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            data = json.loads(result.stdout)
            duration_str = data.get('format', {}).get('duration')
            if duration_str:
                return float(duration_str)
    except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError):
        pass

    # Fallback to OpenCV
    try:
        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        cap.release()
        if fps > 0:
            return frame_count / fps
    except Exception:
        pass

    return None


def get_folder_size(folder_path: str) -> int:
    """
    Calculate total size of all files in a folder (recursive).
    Returns size in bytes.
    """
    total_size = 0
    try:
        for dirpath, dirnames, filenames in os.walk(folder_path):
            for filename in filenames:
                file_path = os.path.join(dirpath, filename)
                if os.path.isfile(file_path):
                    total_size += os.path.getsize(file_path)
    except OSError:
        pass
    return total_size
