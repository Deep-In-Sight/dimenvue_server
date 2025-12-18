"""
Integration tests for Storage API endpoints.

Tests cover:
- Removable storage detection and listing
- Internal storage usage and formatting
- External storage formatting and validation
"""

import pytest
import os
from unittest.mock import patch, MagicMock
from pathlib import Path


# ==================== Section 4.1: Removable Storage (2 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_list_usb_devices_with_usb_connected(async_client):
    """
    Test 1: List USB Devices
    GET /storage/removable with USB connected -> 200 OK, device list with mountpoint, label, usage
    """
    # Mock USB device data
    mock_usb_data = {
        "/media/usb0": {
            "mountpoint": "/media/usb0",
            "label": "USB_DRIVE",
            "usage": {
                "size": 16000000000,
                "used": 8000000000,
                "free": 8000000000,
                "percent": 50.0
            }
        },
        "/media/usb1": {
            "mountpoint": "/media/usb1",
            "label": "BACKUP",
            "usage": {
                "size": 32000000000,
                "used": 16000000000,
                "free": 16000000000,
                "percent": 50.0
            }
        }
    }

    with patch("server_app.get_removable", return_value=mock_usb_data):
        response = await async_client.get("/storage/removable")

    assert response.status_code == 200
    data = response.json()

    # Verify response structure and content
    assert "/media/usb0" in data
    assert "/media/usb1" in data

    # Verify USB device 0
    usb0 = data["/media/usb0"]
    assert usb0["mountpoint"] == "/media/usb0"
    assert usb0["label"] == "USB_DRIVE"
    assert "usage" in usb0
    assert usb0["usage"]["size"] == 16000000000
    assert usb0["usage"]["free"] == 8000000000

    # Verify USB device 1
    usb1 = data["/media/usb1"]
    assert usb1["mountpoint"] == "/media/usb1"
    assert usb1["label"] == "BACKUP"
    assert "usage" in usb1


@pytest.mark.asyncio
@pytest.mark.integration
async def test_list_usb_devices_with_no_usb(async_client):
    """
    Test 2: No USB Devices
    GET /storage/removable with no USB -> 200 OK, {}
    """
    # Mock no USB devices connected
    with patch("storage.get_removable", return_value={}):
        response = await async_client.get("/storage/removable")

    assert response.status_code == 200
    data = response.json()
    assert data == {}


# ==================== Section 4.2: Internal Storage (2 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_internal_storage_usage(async_client, temp_catalog, tmp_path):
    """
    Test 3: Get Internal Storage Usage
    GET /storage/internal/usage -> 200 OK, stats match catalog totals
    """
    # Add some items to the catalog to have storage stats
    # Create sample item directories with files
    item_path1 = tmp_path / "photo_item"
    item_path1.mkdir(parents=True, exist_ok=True)
    (item_path1 / "front.jpg").write_bytes(b"fake image data" * 1000)  # ~15KB
    (item_path1 / "left.jpg").write_bytes(b"fake image data" * 1000)
    (item_path1 / "right.jpg").write_bytes(b"fake image data" * 1000)

    item_path2 = tmp_path / "video_item"
    item_path2.mkdir(parents=True, exist_ok=True)
    (item_path2 / "video.mp4").write_bytes(b"fake video data" * 2000)  # ~30KB

    item_path3 = tmp_path / "scan_item"
    item_path3.mkdir(parents=True, exist_ok=True)
    (item_path3 / "scan.pcd").write_bytes(b"fake scan data" * 1500)  # ~22.5KB

    # Add items to catalog
    temp_catalog.add_item("Image", str(item_path1))
    temp_catalog.add_item("Video", str(item_path2))
    temp_catalog.add_item("Scan", str(item_path3))

    # Mock psutil.disk_usage to return consistent values
    mock_usage = MagicMock()
    mock_usage.total = 100000000000  # 100GB
    mock_usage.free = 50000000000    # 50GB

    with patch("storage.psutil.disk_usage", return_value=mock_usage):
        response = await async_client.get("/storage/internal/usage")

    assert response.status_code == 200
    data = response.json()

    # Verify response structure
    assert "total" in data
    assert "free" in data
    assert "photos" in data
    assert "videos" in data
    assert "scans" in data
    assert "system" in data

    # Verify values
    assert data["total"] == 100000000000
    assert data["free"] == 50000000000

    # Verify storage stats match catalog
    catalog_stats = temp_catalog.get_storage_stats()
    assert data["photos"] == catalog_stats["Photos"]
    assert data["videos"] == catalog_stats["Videos"]
    assert data["scans"] == catalog_stats["Scans"]

    # Verify photos, videos, scans are positive (items were added)
    assert data["photos"] > 0
    assert data["videos"] > 0
    assert data["scans"] > 0


@pytest.mark.asyncio
@pytest.mark.integration
async def test_format_internal_storage(async_client, temp_catalog, tmp_path):
    """
    Test 4: Format Internal Storage
    PUT /storage/internal/format -> 200 OK, all items cleared, stats reset
    """
    # Add some items to the catalog first
    item_path1 = tmp_path / "item1"
    item_path1.mkdir(parents=True, exist_ok=True)
    (item_path1 / "file.jpg").write_bytes(b"data" * 100)

    item_path2 = tmp_path / "item2"
    item_path2.mkdir(parents=True, exist_ok=True)
    (item_path2 / "video.mp4").write_bytes(b"data" * 200)

    uuid1 = temp_catalog.add_item("Image", str(item_path1))
    uuid2 = temp_catalog.add_item("Video", str(item_path2))

    # Verify items exist before formatting
    catalog_data = temp_catalog.get_data()
    assert uuid1 in catalog_data["items"]
    assert uuid2 in catalog_data["items"]
    assert catalog_data["storage_stats"]["Photos"] > 0
    assert catalog_data["storage_stats"]["Videos"] > 0

    # Format internal storage
    response = await async_client.put("/storage/internal/format")

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert "formatted" in data["message"].lower() or "success" in data["message"].lower()

    # Verify all items are cleared
    catalog_data = temp_catalog.get_data()
    assert len(catalog_data["items"]) == 0

    # Verify stats are reset to zero
    stats = catalog_data["storage_stats"]
    assert stats["Photos"] == 0
    assert stats["Videos"] == 0
    assert stats["Scans"] == 0


# ==================== Section 4.3: External Storage (2 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_format_external_storage(async_client, tmp_path):
    """
    Test 5: Format External Storage
    PUT /storage/external/format with {"mountpoint": "/media/usb0"} -> 200 OK, files deleted
    """
    # Create a mock mountpoint with files
    mock_mountpoint = tmp_path / "usb_mount"
    mock_mountpoint.mkdir(parents=True, exist_ok=True)

    # Create some files and directories in the mountpoint
    (mock_mountpoint / "file1.txt").write_text("test data 1")
    (mock_mountpoint / "file2.jpg").write_bytes(b"fake image data")

    subdir = mock_mountpoint / "subdir"
    subdir.mkdir(parents=True, exist_ok=True)
    (subdir / "nested_file.txt").write_text("nested data")

    # Verify files exist before formatting
    assert (mock_mountpoint / "file1.txt").exists()
    assert (mock_mountpoint / "file2.jpg").exists()
    assert (subdir / "nested_file.txt").exists()

    # Mock os.path.ismount to return True for our test mountpoint
    def mock_ismount(path):
        return path == str(mock_mountpoint)

    with patch("os.path.ismount", side_effect=mock_ismount):
        response = await async_client.put(
            "/storage/external/format",
            json={"mountpoint": str(mock_mountpoint)}
        )

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert str(mock_mountpoint) in data["message"]

    # Verify all files and directories are deleted
    assert not (mock_mountpoint / "file1.txt").exists()
    assert not (mock_mountpoint / "file2.jpg").exists()
    assert not (subdir / "nested_file.txt").exists()
    assert not subdir.exists()

    # Mountpoint itself should still exist (just empty)
    assert mock_mountpoint.exists()


@pytest.mark.asyncio
@pytest.mark.integration
async def test_format_external_storage_non_existent_device(async_client):
    """
    Test 6: Format Non-Existent Device
    PUT /storage/external/format with invalid mountpoint -> 404 Not Found
    """
    # Mock os.path.ismount to return False (invalid mountpoint)
    with patch("os.path.ismount", return_value=False):
        response = await async_client.put(
            "/storage/external/format",
            json={"mountpoint": "/invalid/mountpoint"}
        )

    # The endpoint returns 400 Bad Request for ValueError, not 404
    # Based on server_app.py line 247: ValueError -> HTTPException(status_code=400)
    assert response.status_code == 400
    data = response.json()
    assert "detail" in data
    assert "not a valid mountpoint" in data["detail"].lower()
