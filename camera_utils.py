import cv2
import numpy as np
import os
import exiftool
import tempfile


def gen_thumbnail_factory(file_path: str, size: tuple[int, int] = (256, 256)) -> np.ndarray:
    ext = os.path.splitext(file_path)[1]
    temp_file = tempfile.mkstemp(suffix='.jpg')[1]
    if ext == ".jpg" or ext == ".jpeg":
        img = gen_thumbnail_image(file_path, size)
    elif ext == ".mp4" or ext == ".mov":
        img = gen_thumbnail_video(file_path, size)
    else:
        raise ValueError(f"Unsupported file type: {ext}")
    cv2.imwrite(temp_file, img)
    return temp_file


def gen_thumbnail_image(file_path: str, size: tuple[int, int] = (256, 256)) -> np.ndarray:
    return cv2.resize(cv2.imread(file_path), size)


def gen_thumbnail_video(video_file: str, size: tuple[int, int] = (256, 256)) -> np.ndarray:
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
    ext = os.path.splitext(file_path)[1]
    if ext == ".jpg" or ext == ".jpeg":
        key_map = key_maps['image']
    elif ext == ".mp4" or ext == ".mov":
        key_map = key_maps['video']
    else:
        raise ValueError(f"Unsupported file type: {ext}")
    return get_exiftool_metadata(file_path, key_map)


def get_exiftool_metadata(file_path: str, key_map: list) -> dict:
    with exiftool.ExifToolHelper() as et:
        metadata = et.get_metadata([file_path])[0]
    print(f"metadata: {metadata}")
    result = {}
    for old_key, new_key in key_map:
        if old_key in metadata:
            result[new_key] = metadata[old_key]
    print(f"filtered metadata: {result}")
    return result
