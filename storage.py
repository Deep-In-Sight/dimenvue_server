import psutil
import pyudev
from pathlib import Path
import os
import subprocess
import shutil


pyudev_ctx = pyudev.Context()


def format_internal(catalog):
    """Format internal storage by clearing the catalog."""
    catalog.clear_all_items()
    return {"status": "success", "message": "Internal storage formatted"}


def format_external(mountpoint: str):
    """Format external storage by deleting all files."""
    if not os.path.ismount(mountpoint):
        raise ValueError(f"'{mountpoint}' is not a valid mountpoint")

    # Delete all files and folders in the mountpoint
    for item in os.listdir(mountpoint):
        item_path = os.path.join(mountpoint, item)
        if os.path.isfile(item_path):
            os.remove(item_path)
        elif os.path.isdir(item_path):
            shutil.rmtree(item_path)

    return {"status": "success", "message": f"External storage '{mountpoint}' formatted"}


def get_internal_usage(catalog) -> dict:
    """
    Get internal storage usage with category breakdown.
    Returns: {total, free, photos, videos, scans, system}
    Note: 'used' should be calculated as (total - free) by client
    """
    # Get overall disk usage for the data path
    data_path = catalog.data_path
    parent_path = os.path.dirname(data_path)

    try:
        usage = psutil.disk_usage(parent_path)
    except OSError:
        usage = psutil.disk_usage("/")

    # Get category sizes from catalog stats
    stats = catalog.get_storage_stats()

    photos_bytes = stats.get("Photos", 0)
    videos_bytes = stats.get("Videos", 0)
    scans_bytes = stats.get("Scans", 0)

    # System = (total - free) - (photos + videos + scans)
    user_data = photos_bytes + videos_bytes + scans_bytes
    system_bytes = max(0, usage.total - usage.free - user_data)

    return {
        "total": usage.total,
        "free": usage.free,
        "photos": photos_bytes,
        "videos": videos_bytes,
        "scans": scans_bytes,
        "system": system_bytes
    }


def _is_usb(devnode: str) -> bool:
    device = pyudev.Devices.from_device_file(pyudev_ctx, devnode)
    return device.get('ID_BUS') == 'usb'


def _get_label(devnode: str) -> str:
    device = pyudev.Devices.from_device_file(pyudev_ctx, devnode)
    return device.get('ID_FS_LABEL', 'No label')

def _get_fs_type(devnode: str) -> str | None:
    try:
        out = subprocess.check_output(
            ["lsblk", '-no', 'fstype', devnode],
            text=True
        ).strip()
        return out or None
    except subprocess.CalledProcessError:
        return None

def get_usage(mountpoint: str) -> dict:
    usage = psutil.disk_usage(mountpoint)
    return {
        'size': usage.total,
        'used': usage.used,
        'free': usage.free,
        'percent': usage.percent
    }


def get_removable():
    allowed_fstypes = ['exfat', 'ext2', 'ext3', 'ext4', 'xfs', 'ntfs']
    partitions = psutil.disk_partitions()
    partitions = [p for p in partitions if _is_usb(p.device)]
    partitions = [p for p in partitions if _get_fs_type(p.device) in allowed_fstypes]
    partitions = [p for p in partitions if 'rw' in p.opts]
    results = {}

    for p in partitions:
        results[p.mountpoint] = {
            'mountpoint': p.mountpoint,
            'label': _get_label(p.device),
            'usage': get_usage(p.mountpoint)
        }

    return results


def get_removable_dummy():
    return {
        '/media/linh/USB': {
            'mountpoint': '/media/linh/USB',
            'label': 'USB',
            'usage': {
                'size': 1000000000,
                'used': 500000000,
                'free': 500000000,
                'percent': 50
            }
        },
        '/media/linh/USB2': {
            'mountpoint': '/media/linh/USB2',
            'label': 'USB2',
            'usage': {
                'size': 1000000000,
                'used': 700000000,
                'free': 300000000,
                'percent': 70
            }
        }
    }
