"""
Pytest configuration and fixtures for DimenvuePro Server tests.

Provides fixtures for:
- FastAPI test client (sync and async)
- Temporary catalog and directories
- Mock USB devices
- Mock ROS2 processes
- Camera and mapping app instances
"""

import os
import sys
import json
import shutil
import asyncio
import tempfile
import subprocess
from pathlib import Path
from unittest.mock import MagicMock, AsyncMock, patch
from typing import Generator, AsyncGenerator

# Mock pyudev BEFORE any imports that might trigger storage.py
# This prevents the pyudev.Context() initialization error in test environment
_mock_pyudev = MagicMock()
_mock_pyudev.Context.return_value = MagicMock()
_mock_pyudev.Device = MagicMock()
_mock_pyudev.Monitor = MagicMock()
sys.modules['pyudev'] = _mock_pyudev

# Mock ROS2 launch module BEFORE ros2 module is imported
_mock_launch = MagicMock()
_mock_launch.LaunchService = MagicMock()
_mock_launch.LaunchDescription = MagicMock()
sys.modules['launch'] = _mock_launch
sys.modules['launch.actions'] = MagicMock()
sys.modules['launch_ros'] = MagicMock()
sys.modules['launch_ros.actions'] = MagicMock()

# Mock the ros2 module functions
_mock_ros2 = MagicMock()
_mock_ros2.StartEverything = AsyncMock()
_mock_ros2.StopEverything = AsyncMock()
_mock_ros2.GetInitStatus = MagicMock(return_value="UNKNOWN")
sys.modules['ros2'] = _mock_ros2

# Mock finalize_mapping module and its dependencies (laspy, open3d, etc.)
sys.modules['laspy'] = MagicMock()
sys.modules['open3d'] = MagicMock()
_mock_finalize = MagicMock()
_mock_finalize.do_finalize = MagicMock()
sys.modules['finalize_mapping'] = _mock_finalize

import pytest
import pytest_asyncio
from fastapi.testclient import TestClient
from httpx import AsyncClient, ASGITransport

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))


# ==================== App & Client Fixtures ====================

@pytest.fixture(scope="session")
def event_loop():
    """Create event loop for async tests."""
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def temp_output_dir(tmp_path: Path) -> Path:
    """Create a temporary output directory for tests."""
    output_dir = tmp_path / "dmv_data"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


@pytest.fixture
def mock_gstd_client():
    """Mock GStreamer daemon client."""
    mock = MagicMock()
    mock.pipeline_create = MagicMock()
    mock.pipeline_play = MagicMock()
    mock.pipeline_stop = MagicMock()
    mock.pipeline_delete = MagicMock()
    mock.element_set = MagicMock()
    mock.event_eos = MagicMock()
    mock.bus_filter = MagicMock()
    mock.bus_read = MagicMock()
    return mock


@pytest.fixture
def mock_janus_process():
    """Mock Janus WebRTC process."""
    mock = MagicMock()
    mock.poll = MagicMock(return_value=None)
    mock.kill = MagicMock()
    mock.wait = MagicMock()
    return mock


@pytest.fixture
def mock_gstd_process():
    """Mock GStreamer daemon process."""
    mock = MagicMock()
    mock.poll = MagicMock(return_value=None)
    mock.kill = MagicMock()
    mock.wait = MagicMock()
    return mock


@pytest.fixture
def mock_camera_app(temp_output_dir, mock_gstd_client, mock_janus_process, mock_gstd_process):
    """Create a mock camera app for testing."""
    from camera_app import MultiCamApp, CameraState

    with patch('camera_app.subprocess.Popen') as mock_popen, \
         patch('camera_app.GstdClient', return_value=mock_gstd_client), \
         patch('camera_app.time.sleep'):

        mock_popen.side_effect = [mock_janus_process, mock_gstd_process]

        # Create a mock catalog
        mock_catalog = MagicMock()
        mock_catalog.add_item = MagicMock(return_value="test-uuid")

        app = MultiCamApp(
            media_path=str(temp_output_dir),
            catalog=mock_catalog
        )
        app.state = CameraState.IDLE
        yield app

        # Cleanup
        app.state = CameraState.IDLE


@pytest.fixture
def temp_catalog(tmp_path: Path):
    """Create a temporary catalog for testing."""
    from catalog import Catalog

    library_path = tmp_path / "library"
    library_path.mkdir(parents=True, exist_ok=True)

    catalog = Catalog(library_path=str(library_path))
    yield catalog
    catalog.close()


@pytest.fixture
def temp_catalog_with_items(temp_catalog, tmp_path: Path):
    """Create a temporary catalog with sample items."""
    # Create sample items
    items = []

    for i in range(3):
        item_path = tmp_path / f"item_{i}"
        item_path.mkdir(parents=True, exist_ok=True)

        # Create dummy files
        (item_path / "front.jpg").write_bytes(b"fake image data " * 100)
        (item_path / "left.jpg").write_bytes(b"fake image data " * 100)
        (item_path / "right.jpg").write_bytes(b"fake image data " * 100)

        # Add to catalog
        uuid = temp_catalog.add_item("Image", str(item_path))
        items.append(uuid)

    return temp_catalog, items


@pytest.fixture
def mock_mapping_app(temp_output_dir, temp_catalog):
    """Create a mock mapping app for testing."""
    from mapping_app import MappingApp, MappingState

    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"), \
         patch('mapping_app.do_finalize'):

        app = MappingApp(
            data_path=str(temp_output_dir),
            catalog=temp_catalog
        )
        app.state = MappingState.IDLE
        yield app


@pytest.fixture
def test_app(temp_output_dir, mock_gstd_client, mock_janus_process, mock_gstd_process, temp_catalog):
    """Create a test FastAPI app with mocked dependencies."""
    from fastapi import FastAPI
    from camera_app import MultiCamApp, CameraState
    from mapping_app import MappingApp, MappingState
    from settings import Settings

    # Create a mock process factory that returns fresh mocks for each Popen call
    def mock_popen_factory(*args, **kwargs):
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None
        mock_proc.kill = MagicMock()
        mock_proc.wait = MagicMock()
        mock_proc.stdout = MagicMock()
        mock_proc.stderr = MagicMock()
        return mock_proc

    with patch('camera_app.subprocess.Popen', side_effect=mock_popen_factory) as mock_popen, \
         patch('camera_app.GstdClient', return_value=mock_gstd_client), \
         patch('camera_app.time.sleep'), \
         patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"), \
         patch('mapping_app.do_finalize'):

        # Import server_app module and create fresh app
        import importlib
        import server_app

        # Override the global instances
        server_app.OUTPUT_DIR = temp_output_dir
        server_app.settings = Settings(temp_output_dir / "settings.json")
        server_app.catalog = temp_catalog
        server_app.cameraApp = MultiCamApp(
            media_path=str(temp_output_dir),
            catalog=temp_catalog
        )
        server_app.mappingApp = MappingApp(
            data_path=str(temp_output_dir),
            catalog=temp_catalog
        )

        yield server_app.app


@pytest.fixture
def test_client(test_app) -> Generator[TestClient, None, None]:
    """Create a synchronous test client."""
    with TestClient(test_app) as client:
        yield client


@pytest_asyncio.fixture
async def async_client(test_app) -> AsyncGenerator[AsyncClient, None]:
    """Create an async test client."""
    transport = ASGITransport(app=test_app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client


# ==================== USB/Storage Fixtures ====================

@pytest.fixture
def mock_usb_device(tmp_path: Path):
    """Create a mock USB device for export testing."""
    class MockUSBDevice:
        def __init__(self, path: Path):
            self.mount_point = str(path / "usb_mount")
            self.label = "TEST_USB"
            self.total_size = 16 * 1024 * 1024 * 1024  # 16GB
            self.free_size = 8 * 1024 * 1024 * 1024    # 8GB

            # Create the mount point
            Path(self.mount_point).mkdir(parents=True, exist_ok=True)

        def get_usage(self):
            return {
                'size': self.total_size,
                'used': self.total_size - self.free_size,
                'free': self.free_size,
                'percent': 50.0
            }

    return MockUSBDevice(tmp_path)


@pytest.fixture
def mock_storage_functions(mock_usb_device):
    """Mock storage utility functions."""
    def mock_get_removable():
        return {
            mock_usb_device.mount_point: {
                'mountpoint': mock_usb_device.mount_point,
                'label': mock_usb_device.label,
                'usage': mock_usb_device.get_usage()
            }
        }

    with patch('storage.get_removable', side_effect=mock_get_removable), \
         patch('storage.psutil.disk_usage') as mock_disk_usage:

        # Mock disk usage for internal storage
        mock_usage = MagicMock()
        mock_usage.total = 100 * 1024 * 1024 * 1024  # 100GB
        mock_usage.free = 50 * 1024 * 1024 * 1024    # 50GB
        mock_usage.used = 50 * 1024 * 1024 * 1024    # 50GB
        mock_usage.percent = 50.0
        mock_disk_usage.return_value = mock_usage

        yield


# ==================== ROS2 Fixtures ====================

@pytest.fixture
def ros2_available():
    """Check if ROS2 workspace is available for mapping tests."""
    ros2_ws = os.getenv("ROS2_WS_PATH", "/ros2_ws")
    return os.path.exists(ros2_ws) and os.path.exists(f"{ros2_ws}/install")


@pytest.fixture
def mock_ros2_launch():
    """Mock ROS2 launch service."""
    with patch('ros2.LaunchService') as mock_ls:
        mock_service = MagicMock()
        mock_service.run = MagicMock()
        mock_service.shutdown = MagicMock()
        mock_ls.return_value = mock_service

        with patch('ros2.threading.Thread') as mock_thread:
            mock_thread_instance = MagicMock()
            mock_thread_instance.start = MagicMock()
            mock_thread_instance.join = MagicMock()
            mock_thread_instance.is_alive = MagicMock(return_value=False)
            mock_thread.return_value = mock_thread_instance

            yield mock_service


@pytest.fixture
def mock_imu_status_file(temp_output_dir):
    """Create a mock IMU status file."""
    artifact_dir = temp_output_dir / "mapping_artifact"
    artifact_dir.mkdir(parents=True, exist_ok=True)
    status_file = artifact_dir / "imu_stabilization_status.txt"

    class IMUStatusController:
        def __init__(self, file_path: Path):
            self.file_path = file_path

        def set_status(self, status: str):
            self.file_path.write_text(status)

        def get_status(self) -> str:
            if self.file_path.exists():
                return self.file_path.read_text().strip()
            return "UNKNOWN"

    return IMUStatusController(status_file)


# ==================== Export Fixtures ====================

@pytest.fixture
def clean_export_manager():
    """Create a clean export manager for each test."""
    from export_manager import ExportManager

    manager = ExportManager()
    yield manager
    manager.stop()


# ==================== Helper Fixtures ====================

@pytest.fixture
def sample_image_file(tmp_path: Path) -> Path:
    """Create a sample image file for testing."""
    import numpy as np

    try:
        import cv2
        # Create a simple test image
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        img[:, :] = (255, 128, 64)  # BGR color

        image_path = tmp_path / "test_image.jpg"
        cv2.imwrite(str(image_path), img)
        return image_path
    except ImportError:
        # Fallback if OpenCV not available
        image_path = tmp_path / "test_image.jpg"
        # Write minimal valid JPEG header
        image_path.write_bytes(
            b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00'
            + b'\xff\xdb\x00C\x00' + bytes(64)  # Quantization table
            + b'\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00'  # SOF
            + b'\xff\xda\x00\x08\x01\x01\x00\x00?\x00'  # SOS
            + b'\x00' * 100  # Image data
            + b'\xff\xd9'  # EOI
        )
        return image_path


@pytest.fixture
def sample_video_file(tmp_path: Path) -> Path:
    """Create a sample video file for testing."""
    video_path = tmp_path / "test_video.mp4"

    try:
        import cv2
        import numpy as np

        # Create a simple test video
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(video_path), fourcc, 30.0, (100, 100))

        for i in range(90):  # 3 seconds at 30fps
            frame = np.zeros((100, 100, 3), dtype=np.uint8)
            frame[:, :] = (i * 3 % 256, 128, 64)
            out.write(frame)

        out.release()
        return video_path
    except (ImportError, Exception):
        # Fallback: create a minimal file
        video_path.write_bytes(b"fake video data " * 1000)
        return video_path


@pytest.fixture
def sample_point_cloud(tmp_path: Path) -> Path:
    """Create a sample point cloud file for testing."""
    pcd_path = tmp_path / "test_cloud.pcd"

    # Create minimal PCD file
    pcd_content = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 10
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 10
DATA ascii
0.0 0.0 0.0
1.0 0.0 0.0
2.0 0.0 0.0
0.0 1.0 0.0
1.0 1.0 0.0
2.0 1.0 0.0
0.0 2.0 0.0
1.0 2.0 0.0
2.0 2.0 0.0
1.0 1.0 1.0
"""
    pcd_path.write_text(pcd_content)
    return pcd_path


# ==================== Markers ====================

def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "integration: Integration tests")
    config.addinivalue_line("markers", "e2e: End-to-end tests")
    config.addinivalue_line("markers", "hardware: Tests requiring hardware")
    config.addinivalue_line("markers", "ros2: Tests requiring ROS2")
    config.addinivalue_line("markers", "slow: Slow running tests")
    config.addinivalue_line("markers", "load: Load/performance tests")


# ==================== Cleanup Fixtures ====================

@pytest.fixture(autouse=True)
def cleanup_after_test():
    """Cleanup after each test."""
    yield
    # Add any global cleanup here if needed
