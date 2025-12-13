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
    Generate a thumbnail for a file and return the temp file path.
    Supports: .jpg, .jpeg, .png, .mp4, .mov, .avi
    """
    ext = os.path.splitext(file_path)[1].lower()
    temp_file = tempfile.mkstemp(suffix='.jpg')[1]
    if ext in (".jpg", ".jpeg", ".png"):
        img = gen_thumbnail_image(file_path, size)
    elif ext in (".mp4", ".mov", ".avi"):
        img = gen_thumbnail_video(file_path, size)
    else:
        raise ValueError(f"Unsupported file type: {ext}")
    cv2.imwrite(temp_file, img)
    return temp_file


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

def get_item_size(item_type: str, contents: dict, item_path: str) -> float | None:
    """
    Get item-specific size metric.

    Returns:
    - MultiViewVideo: Duration in seconds (from first video file)
    - MappingScan: Area in square meters (from LAS bounds)
    - MultiViewImage: None (no size metric)
    """
    if item_type == "MultiViewVideo":
        # Get duration from first available video file
        for key in ['left', 'front', 'right']:
            if key in contents:
                video_file = os.path.join(item_path, contents[key])
                if os.path.exists(video_file):
                    duration = get_video_duration(video_file)
                    if duration is not None:
                        return duration
        return None

    elif item_type == "MappingScan":
        # Calculate area from LAS file bounds
        las_file = contents.get('point_cloud')
        if las_file:
            las_path = os.path.join(item_path, las_file)
            if os.path.exists(las_path):
                bounds = get_las_bounds(las_path)
                if bounds:
                    return (bounds['maxX'] - bounds['minX']) * (bounds['maxY'] - bounds['minY'])
        return None

    # MultiViewImage and others have no size metric
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


def get_las_bounds(las_path: str) -> dict | None:
    """
    Get bounds from a LAS file header.

    Returns dict with minX, maxX, minY, maxY, minZ, maxZ or None on error.
    """
    try:
        with open(las_path, 'rb') as f:
            # Read LAS header
            signature = f.read(4)
            if signature != b'LASF':
                return None

            # Skip to offset 179 where bounds start (LAS 1.x format)
            f.seek(179)

            # Read bounds (6 doubles: maxX, minX, maxY, minY, maxZ, minZ)
            bounds_data = f.read(48)  # 6 * 8 bytes
            bounds = struct.unpack('<6d', bounds_data)

            return {
                'maxX': bounds[0],
                'minX': bounds[1],
                'maxY': bounds[2],
                'minY': bounds[3],
                'maxZ': bounds[4],
                'minZ': bounds[5]
            }
    except Exception as e:
        print(f"Error reading LAS bounds from {las_path}: {e}")
        return None


def get_folder_size(folder_path: str) -> int:
    """
    Calculate total size of all files in a folder (non-recursive for item folders).
    Returns size in bytes.
    """
    total_size = 0
    try:
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            if os.path.isfile(file_path):
                total_size += os.path.getsize(file_path)
    except OSError:
        pass
    return total_size
