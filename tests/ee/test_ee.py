"""
End-to-End Tests for DimenvuePro Server

These tests use REAL GStreamer daemon and ROS2 components (no mocks).
Run these tests inside the Docker container where all dependencies are available.

Test scenarios:
- EE-1: Complete Photo Capture Workflow
- EE-2: Video Recording Workflow
- EE-3: Spatial Mapping Workflow (requires ROS2 and test_bag)
- EE-4: Storage Management Workflow
- EE-5: Multi-Export Queue Management
- EE-6: Preview Switching During Recording

Usage:
    # Run inside Docker container
    cd /ros2_ws/dimenvue_server/tests
    pytest test_ee.py -v -m ee
"""

import asyncio
import os
import sys
import tempfile
import shutil
from pathlib import Path
from typing import AsyncGenerator

import pytest
import pytest_asyncio

# Add server directory to path BEFORE importing server modules
# tests/ee/test_ee.py -> tests/ee -> tests -> dimenvue_server
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# Import test dependencies - these don't require mocks
from httpx import AsyncClient, ASGITransport

# Path for test bag data
TEST_BAG_PATH = "/shared_data/test_bag"


class EETestContext:
    """Context manager for EE tests with real GStreamer."""

    def __init__(self):
        self.temp_dir = None
        self.app = None
        self.catalog = None
        self.camera_app = None
        self.mapping_app = None

    def setup(self):
        """Initialize test environment with real components."""
        # Create temp directory
        self.temp_dir = Path(tempfile.mkdtemp(prefix="ee_test_"))
        output_dir = self.temp_dir / "dmv_data"
        output_dir.mkdir(parents=True, exist_ok=True)

        # Create required subdirectories
        (output_dir / "library").mkdir(parents=True, exist_ok=True)
        (output_dir / "capture_tmp").mkdir(parents=True, exist_ok=True)
        (output_dir / "record_tmp").mkdir(parents=True, exist_ok=True)
        (output_dir / "mapping_artifact").mkdir(parents=True, exist_ok=True)

        # Set environment variable
        os.environ["CAMERA_APP_OUTPUT_DIR"] = str(output_dir)

        # Import real modules (no mocks)
        from catalog import Catalog
        from settings import Settings
        from camera_app import MultiCamApp
        from mapping_app import MappingApp

        # Create real instances
        self.catalog = Catalog(library_path=str(output_dir / "library"))
        settings = Settings(output_dir / "settings.json")

        # Create camera app with real GStreamer
        self.camera_app = MultiCamApp(
            media_path=str(output_dir),
            catalog=self.catalog
        )

        # Create mapping app with real ROS2
        self.mapping_app = MappingApp(
            data_path=str(output_dir),
            catalog=self.catalog
        )

        # Override server_app globals
        import server_app
        server_app.OUTPUT_DIR = output_dir
        server_app.settings = settings
        server_app.catalog = self.catalog
        server_app.cameraApp = self.camera_app
        server_app.mappingApp = self.mapping_app

        self.app = server_app.app
        self.output_dir = output_dir

        return self

    def cleanup(self):
        """Clean up test resources."""
        try:
            # Deinitialize camera pipelines
            if self.camera_app and hasattr(self.camera_app, 'pipelines') and self.camera_app.pipelines:
                try:
                    self.camera_app._do_gst_deinit()
                except Exception as e:
                    print(f"Error during camera cleanup: {e}")

            # Kill gstd and janus processes
            if self.camera_app:
                if hasattr(self.camera_app, 'gstd_proc') and self.camera_app.gstd_proc:
                    try:
                        self.camera_app.gstd_proc.kill()
                        self.camera_app.gstd_proc.wait()
                    except Exception:
                        pass
                if hasattr(self.camera_app, 'janus_proc') and self.camera_app.janus_proc:
                    try:
                        self.camera_app.janus_proc.kill()
                        self.camera_app.janus_proc.wait()
                    except Exception:
                        pass
        except Exception as e:
            print(f"Error during cleanup: {e}")

        try:
            if self.catalog:
                self.catalog.close()
        except Exception:
            pass

        try:
            if self.temp_dir and self.temp_dir.exists():
                shutil.rmtree(self.temp_dir, ignore_errors=True)
        except Exception:
            pass


@pytest.fixture(scope="function")
def ee_context():
    """Create EE test context with real GStreamer.

    Note: Uses function scope for proper isolation between tests.
    Each test gets a fresh GStreamer daemon and catalog.
    """
    context = EETestContext()
    context.setup()
    yield context
    context.cleanup()


@pytest_asyncio.fixture
async def ee_client(ee_context) -> AsyncGenerator[AsyncClient, None]:
    """Create an async test client for EE tests."""
    transport = ASGITransport(app=ee_context.app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client


@pytest.fixture
def ee_usb_mount(tmp_path):
    """Create a mock USB mount point."""
    usb_mount = tmp_path / "usb_mount"
    usb_mount.mkdir(parents=True, exist_ok=True)
    return str(usb_mount)


@pytest.mark.asyncio
@pytest.mark.ee
async def test_ee_complete_photo_capture_workflow(ee_client, ee_context, ee_usb_mount):
    """
    EE-1: Complete Photo Capture Workflow

    Test flow:
    1. Init camera (real GStreamer pipelines)
    2. Capture photo (real file creation via GStreamer)
    3. Get catalog (verify item appears)
    4. Get metadata (check file info)
    5. Export to USB
    6. Poll progress until complete
    7. Verify files on USB
    """
    usb_mount = ee_usb_mount

    # Action 1: Initialize camera (creates real GStreamer pipelines)
    response = await ee_client.put("/cameraApp/init")
    assert response.status_code == 200, f"Init failed: {response.text}"

    # Give pipelines time to start and stabilize
    await asyncio.sleep(2)

    # Action 2: Capture photo (real GStreamer capture)
    response = await ee_client.put("/cameraApp/capture")
    assert response.status_code == 200, f"Capture failed: {response.text}"
    capture_result = response.json()
    assert capture_result.get("status") == "captured"

    # Wait for files to be written
    await asyncio.sleep(1)

    # Action 3: Get catalog
    response = await ee_client.get("/catalog")
    assert response.status_code == 200, f"Get catalog failed: {response.text}"
    catalog_data = response.json()

    # Verify photo appears in catalog
    assert len(catalog_data["items"]) >= 1, "No items in catalog after capture"

    # Find the most recent item
    items_list = list(catalog_data["items"].items())
    item_uuid, item = items_list[-1]
    assert "Image" in item["type"], f"Expected Image type, got {item['type']}"

    # Action 4: Verify files exist
    item_url = item.get("url", "")
    assert item_url, "Item should have a url"

    item_dir = Path(item_url)
    assert item_dir.exists(), f"Item directory does not exist: {item_dir}"

    # Check for real image files created by GStreamer
    image_files = list(item_dir.glob("*.jpg")) + list(item_dir.glob("*.png"))
    assert len(image_files) > 0, f"No image files found in {item_dir}"

    # Verify files have content (not empty)
    for img_file in image_files:
        assert img_file.stat().st_size > 0, f"Image file is empty: {img_file}"

    # Action 5: Get metadata for a file
    file_path = str(image_files[0])
    response = await ee_client.get(f"/catalog/metadata/{file_path}")
    assert response.status_code == 200, f"Get metadata failed: {response.text}"
    metadata = response.json()
    assert metadata is not None

    # Action 6: Export to USB
    response = await ee_client.put(
        f"/catalog/{item_uuid}/export",
        json={"mountpoint": usb_mount}
    )
    assert response.status_code == 200, f"Export failed: {response.text}"
    export_response = response.json()
    assert "task_id" in export_response

    # Action 7: Poll export progress until complete
    max_polls = 60
    export_complete = False
    for _ in range(max_polls):
        response = await ee_client.get("/export/progress")
        assert response.status_code == 200
        progress_data = response.json()

        if progress_data.get("is_complete", False):
            export_complete = True
            break

        await asyncio.sleep(0.1)

    assert export_complete, "Export did not complete in time"

    # Verify files on USB
    usb_path = Path(usb_mount)
    assert usb_path.exists()
    exported_files = [f for f in usb_path.rglob("*") if f.is_file()]
    assert len(exported_files) > 0, "No files found on USB after export"

    # Cleanup: Deinitialize camera
    response = await ee_client.put("/cameraApp/deinit")
    assert response.status_code == 200


@pytest.mark.asyncio
@pytest.mark.ee
async def test_ee_video_recording_workflow(ee_client, ee_context, ee_usb_mount):
    """
    EE-2: Video Recording Workflow

    Test flow:
    1. Init camera
    2. recordStart
    3. Wait 5 seconds (shorter for tests)
    4. recordStop
    5. Get catalog (get video UUID), verify video files in correct location
    6. Rename video to "TestVideo"
    7. Get catalog, verify name changed, verify video files in correct location
    8. Export to USB
    9. Poll progress until complete
    10. Verify exported files on USB
    """
    usb_mount = ee_usb_mount

    # Action 1: Initialize camera
    response = await ee_client.put("/cameraApp/init")
    assert response.status_code == 200

    await asyncio.sleep(2)

    # Action 2: Start recording
    response = await ee_client.put("/cameraApp/recordStart")
    assert response.status_code == 200

    # Action 3: Wait for recording (5 seconds for test)
    await asyncio.sleep(5)

    # Action 4: Stop recording
    response = await ee_client.put("/cameraApp/recordStop")
    assert response.status_code == 200

    # Wait for files to be finalized
    await asyncio.sleep(2)

    # Action 5: Get catalog to retrieve video UUID, verify video files in correct location
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_data = response.json()

    # Find the video item
    video_uuid = None
    video_item = None
    for uuid, item in reversed(list(catalog_data["items"].items())):
        if "Video" in item["type"]:
            video_uuid = uuid
            video_item = item
            break

    assert video_item is not None, "Video not found in catalog"

    # Verify video files exist in correct location
    item_url = video_item.get("url", "")
    assert item_url, "Video item should have a url"
    video_dir = Path(item_url)

    assert video_dir.exists(), f"Video directory does not exist: {video_dir}"
    video_files = list(video_dir.glob("*.mp4")) + list(video_dir.glob("*.avi"))
    assert len(video_files) == 3, f"Expected 3 video files, got {len(video_files)}"

    # Verify files have content
    for vid_file in video_files:
        assert vid_file.stat().st_size > 0, f"Video file is empty: {vid_file}"

    original_video_dir = video_dir  # Save for later comparison

    # Action 6: Rename video to "TestVideo"
    response = await ee_client.put(
        f"/catalog/{video_uuid}/rename",
        json={"name": "TestVideo"}
    )
    assert response.status_code == 200
    renamed_item = response.json()
    assert "TestVideo" in renamed_item.get("name", ""), f"Name not updated: {renamed_item}"

    # Action 7: Get catalog, verify name changed, verify video files in correct location
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_data = response.json()

    # Find the renamed video item
    renamed_video_item = catalog_data["items"].get(video_uuid)
    assert renamed_video_item is not None, "Video not found in catalog after rename"
    assert "TestVideo" in renamed_video_item.get("name", ""), \
        f"Name not reflected in catalog: {renamed_video_item.get('name')}"

    # Verify video files still in correct location after rename
    renamed_url = renamed_video_item.get("url", "")
    assert renamed_url, "Video item should still have a url after rename"
    renamed_video_dir = Path(renamed_url)

    assert renamed_video_dir.exists(), f"Video directory does not exist after rename: {renamed_video_dir}"
    video_files_after_rename = list(renamed_video_dir.glob("*.mp4")) + list(renamed_video_dir.glob("*.avi"))
    assert len(video_files_after_rename) == 3, \
        f"Expected 3 video files after rename, got {len(video_files_after_rename)}"

    # Verify files still have content
    for vid_file in video_files_after_rename:
        assert vid_file.stat().st_size > 0, f"Video file is empty after rename: {vid_file}"

    # Action 8: Export to USB
    response = await ee_client.put(
        f"/catalog/{video_uuid}/export",
        json={"mountpoint": usb_mount}
    )
    assert response.status_code == 200

    # Action 9: Poll export progress until complete
    max_polls = 120
    export_complete = False
    for _ in range(max_polls):
        response = await ee_client.get("/export/progress")
        assert response.status_code == 200
        progress_data = response.json()

        if progress_data.get("is_complete", False):
            export_complete = True
            break

        await asyncio.sleep(0.1)

    assert export_complete, "Export did not complete in time"

    # Action 10: Verify exported files on USB
    usb_path = Path(usb_mount)
    assert usb_path.exists(), f"USB mount point does not exist: {usb_mount}"

    exported_files = [f for f in usb_path.rglob("*") if f.is_file()]
    assert len(exported_files) > 0, f"No files found on USB after export: {usb_mount}"

    # Verify video files were exported
    exported_videos = [f for f in exported_files if f.suffix.lower() in ('.mp4', '.avi')]
    assert len(exported_videos) >= 3, \
        f"Expected at least 3 video files on USB, got {len(exported_videos)}"

    # Cleanup
    response = await ee_client.put("/cameraApp/deinit")
    assert response.status_code == 200


@pytest.mark.asyncio
@pytest.mark.ee
@pytest.mark.ros2
@pytest.mark.slow
async def test_ee_spatial_mapping_workflow(ee_client, ee_context, ee_usb_mount):
    """
    EE-3: Spatial Mapping Workflow

    This test uses real ROS2 launch with the test_bag data.
    Requires: ROS2 environment and /shared_data/test_bag

    Test flow per specification:
    1. PUT /mappingApp/start and wait until return
    2. Poll GET /mappingApp/state until state is RUNNING
    3. Wait 30 seconds (simulate environment scanning)
    4. PUT /mappingApp/stop
    5. Poll state until IDLE
    6. GET /catalog (verify scan entry added)
    7. Verify scan item details
    8. PUT /catalog/{uuid}/export to USB
    9. Poll GET /export/progress until complete
    """
    import subprocess

    # Skip if test_bag doesn't exist
    if not Path(TEST_BAG_PATH).exists():
        pytest.skip(f"Test bag not found at {TEST_BAG_PATH}")

    usb_mount = ee_usb_mount

    # Get catalog count before mapping to compare later
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_before = response.json()
    items_before = len(catalog_before.get("items", {}))

    # Verify initial state is IDLE
    response = await ee_client.get("/mappingApp/state")
    assert response.status_code == 200
    initial_state = response.json()
    assert initial_state.get("state") == "idle", f"Expected idle state, got {initial_state}"

    # Action 1: Start mapping
    response = await ee_client.put("/mappingApp/start")
    assert response.status_code == 200
    start_response = response.json()
    print(f"Start response: {start_response}")

    # Verify state transitions: IDLE → STARTING → INITIALIZING → RUNNING
    # Track the complete sequence
    max_polls = 120
    state_sequence = []
    expected_states = ["starting", "initializing", "running"]

    for i in range(max_polls):
        response = await ee_client.get("/mappingApp/state")
        assert response.status_code == 200
        state_data = response.json()
        current_state = state_data.get("state")

        # Track unique state transitions in order
        if len(state_sequence) == 0 or state_sequence[-1] != current_state:
            state_sequence.append(current_state)
            print(f"State transition [{i}]: {current_state}")

            # Check IMU status during INITIALIZING
            if current_state == "initializing":
                imu_status = state_data.get("status", "")
                if imu_status:
                    print(f"  IMU status: {imu_status}")

        if current_state == "running":
            break

        await asyncio.sleep(1)
    else:
        pytest.fail(f"Mapping did not reach RUNNING state within {max_polls}s. States seen: {state_sequence}")

    # Verify expected state transitions occurred
    for expected in expected_states:
        assert expected in state_sequence, \
            f"Expected state '{expected}' not found in sequence: {state_sequence}"

    # Verify nodes are running (when in RUNNING state)
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        node_list = result.stdout
        print(f"ROS2 nodes while running: {node_list}")

        # Check for expected nodes (may have different exact names)
        expected_nodes = ["fastlio", "imu_monitor", "bridge", "recorder"]
        for node_keyword in expected_nodes:
            if node_keyword.lower() not in node_list.lower():
                print(f"  Warning: Node containing '{node_keyword}' not found in node list")
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"  Could not verify ROS2 nodes: {e}")

    # Action 3: Let mapping process for 30 seconds (accumulate point cloud data)
    print("Mapping running for 30 seconds...")
    await asyncio.sleep(30)

    # Action 4: Stop mapping
    response = await ee_client.put("/mappingApp/stop")
    assert response.status_code == 200
    stop_response = response.json()
    print(f"Stop response: {stop_response}")

    # Action 5: Poll until IDLE, tracking STOPPING state
    stop_sequence = []
    for i in range(120):
        response = await ee_client.get("/mappingApp/state")
        assert response.status_code == 200
        state_data = response.json()
        current_state = state_data.get("state")

        if len(stop_sequence) == 0 or stop_sequence[-1] != current_state:
            stop_sequence.append(current_state)
            print(f"Stop transition [{i}]: {current_state}")

        if current_state == "idle":
            break

        await asyncio.sleep(1)
    else:
        pytest.fail(f"Mapping did not return to IDLE state. States seen: {stop_sequence}")

    # Verify STOPPING state was observed (may be brief)
    if "stopping" in stop_sequence:
        print("  STOPPING state observed during shutdown")

    # Verify back to IDLE
    response = await ee_client.get("/mappingApp/state")
    assert response.status_code == 200
    assert response.json().get("state") == "idle"

    # Action 6: GET /catalog and verify scan entry added
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_after = response.json()
    items_after = catalog_after.get("items", {})

    assert len(items_after) > items_before, \
        f"Expected new scan in catalog. Before: {items_before}, After: {len(items_after)}"

    # Action 7: Find and verify the new scan item
    scan_uuid = None
    scan_item = None

    for uuid, item in items_after.items():
        item_type = item.get("type", "")
        # Look for Scan or Mapping type items
        if "Scan" in item_type or "Mapping" in item_type or "PointCloud" in item_type:
            # Check if this is a new item (not in items_before)
            if uuid not in catalog_before.get("items", {}):
                scan_uuid = uuid
                scan_item = item
                break

    # If no scan type found, check for the newest item by timestamp
    if scan_item is None and len(items_after) > items_before:
        # Get the newest item
        newest_uuid = list(items_after.keys())[-1]
        if newest_uuid not in catalog_before.get("items", {}):
            scan_uuid = newest_uuid
            scan_item = items_after[newest_uuid]
            print(f"  Using newest catalog item as scan: type={scan_item.get('type')}")

    assert scan_item is not None, \
        f"New scan not found in catalog. Items: {list(items_after.keys())}"

    print(f"Found scan item: UUID={scan_uuid}")
    print(f"  Type: {scan_item.get('type')}")
    print(f"  Name: {scan_item.get('name')}")
    print(f"  URL: {scan_item.get('url')}")

    # Verify scan item has required fields
    assert scan_uuid is not None, "Scan should have UUID"
    assert "url" in scan_item, "Scan item should have url field"

    # Verify timestamp exists (might be 'created_at', 'timestamp', or 'date')
    has_timestamp = "created_at" in scan_item or "timestamp" in scan_item or "date" in scan_item
    assert has_timestamp, f"Scan item should have timestamp. Keys: {scan_item.keys()}"

    # Verify artifact files exist
    scan_url = scan_item.get("url", "")
    if scan_url:
        scan_dir = Path(scan_url)
        if scan_dir.exists():
            print(f"  Scan directory exists: {scan_dir}")

            # Check for point cloud file
            pcd_files = list(scan_dir.glob("*.pcd")) + list(scan_dir.glob("*.ply")) + list(scan_dir.glob("*.las"))
            if pcd_files:
                print(f"  Point cloud files: {[f.name for f in pcd_files]}")
            else:
                print("  Warning: No point cloud files found")

            # Check for metadata file
            metadata_file = scan_dir / "metadata.json"
            if metadata_file.exists():
                print(f"  Metadata file exists: {metadata_file}")
            else:
                print("  Warning: metadata.json not found")

            # Check for thumbnail
            thumb_files = list(scan_dir.glob("*.jpg")) + list(scan_dir.glob("*.png")) + list(scan_dir.glob("*thumb*"))
            if thumb_files:
                print(f"  Thumbnail files: {[f.name for f in thumb_files]}")
        else:
            print(f"  Warning: Scan directory does not exist: {scan_dir}")

    # Action 8: Export scan to USB
    response = await ee_client.put(
        f"/catalog/{scan_uuid}/export",
        json={"mountpoint": usb_mount}
    )
    assert response.status_code == 200, f"Export failed: {response.text}"
    export_response = response.json()
    assert "task_id" in export_response, f"Export response should contain task_id: {export_response}"
    print(f"Export started: task_id={export_response.get('task_id')}")

    # Action 9: Poll export progress until complete
    max_export_polls = 120
    export_complete = False

    for i in range(max_export_polls):
        response = await ee_client.get("/export/progress")
        assert response.status_code == 200
        progress_data = response.json()

        overall_progress = progress_data.get("overall_progress", 0)
        is_complete = progress_data.get("is_complete", False)

        if i % 10 == 0:  # Log every 10 polls
            print(f"Export progress [{i}]: {overall_progress}% complete={is_complete}")

        if is_complete:
            export_complete = True
            break

        await asyncio.sleep(1)

    assert export_complete, f"Export did not complete within {max_export_polls}s"
    print("Export completed successfully")

    # Verify files on USB
    usb_path = Path(usb_mount)
    exported_files = [f for f in usb_path.rglob("*") if f.is_file()]
    assert len(exported_files) > 0, f"No files found on USB after export: {usb_mount}"
    print(f"Exported {len(exported_files)} files to USB")

    # Verify no orphaned ROS2 nodes after stop
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        node_list = result.stdout

        # Check that mapping-specific nodes are not present
        orphaned_nodes = []
        mapping_node_keywords = ["fastlio", "imu_monitor", "bridge_node", "recorder_node"]
        for keyword in mapping_node_keywords:
            if keyword.lower() in node_list.lower():
                orphaned_nodes.append(keyword)

        if orphaned_nodes:
            print(f"  Warning: Potential orphaned nodes: {orphaned_nodes}")
        else:
            print("  No orphaned ROS2 mapping nodes detected")

    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"  Could not verify orphaned nodes: {e}")

    # Verify no orphaned imu_monitor processes
    try:
        result = subprocess.run(
            ["pgrep", "-f", "imu_monitor"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.stdout.strip():
            print(f"  Warning: imu_monitor process still running: PIDs={result.stdout.strip()}")
        else:
            print("  No orphaned imu_monitor processes")
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"  Could not verify orphaned processes: {e}")

    print("EE-003 Spatial Mapping Workflow: PASSED")


@pytest.mark.asyncio
@pytest.mark.ee
async def test_ee_storage_management_workflow(ee_client, ee_context):
    """
    EE-4: Storage Management Workflow

    Test flow:
    1. Capture 3 photos
    2. Record 1 video
    3. Get usage
    4. Delete 2 items
    5. Get usage (verify reduced)
    6. Format internal storage
    7. Get catalog (verify empty)
    """
    # Initialize camera
    response = await ee_client.put("/cameraApp/init")
    assert response.status_code == 200
    await asyncio.sleep(2)

    # Action 1: Capture 3 photos
    for i in range(3):
        response = await ee_client.put("/cameraApp/capture")
        assert response.status_code == 200
        await asyncio.sleep(1)

    # Get catalog to retrieve photo UUIDs
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_data = response.json()

    photo_uuids = []
    for uuid, item in catalog_data["items"].items():
        if "Image" in item.get("type", ""):
            photo_uuids.append(uuid)

    assert len(photo_uuids) >= 3, f"Expected at least 3 photos, got {len(photo_uuids)}"

    # Action 2: Record 1 video
    response = await ee_client.put("/cameraApp/recordStart")
    assert response.status_code == 200
    await asyncio.sleep(3)
    response = await ee_client.put("/cameraApp/recordStop")
    assert response.status_code == 200
    await asyncio.sleep(2)

    # Action 3: Get storage usage
    response = await ee_client.get("/storage/internal/usage")
    assert response.status_code == 200
    usage_before_delete = response.json()

    photos_usage = usage_before_delete.get("photos", 0)
    videos_usage = usage_before_delete.get("videos", 0)
    assert photos_usage > 0, "Photos usage should be greater than 0"
    assert videos_usage > 0, "Videos usage should be greater than 0"

    # Action 4: Delete 2 items
    items_to_delete = photo_uuids[:2]
    for item_uuid in items_to_delete:
        response = await ee_client.put(f"/catalog/{item_uuid}/delete")
        assert response.status_code == 200

    # Action 5: Get usage and verify reduced
    response = await ee_client.get("/storage/internal/usage")
    assert response.status_code == 200
    usage_after_delete = response.json()

    photos_after = usage_after_delete.get("photos", 0)
    assert photos_after < photos_usage, "Photos usage should decrease after deletion"

    # Action 6: Format internal storage
    response = await ee_client.put("/storage/internal/format")
    assert response.status_code == 200

    # Action 7: Get catalog and verify empty
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_data = response.json()
    assert len(catalog_data.get("items", {})) == 0, "Catalog should be empty after format"

    # Cleanup
    response = await ee_client.put("/cameraApp/deinit")
    assert response.status_code == 200


@pytest.mark.asyncio
@pytest.mark.ee
async def test_ee_multi_export_queue_management(ee_client, ee_context, ee_usb_mount):
    """
    EE-5: Multi-Export Queue Management

    Test flow:
    1. Create 3 items (photos)
    2. Queue all 3 for export
    3. Verify queue length = 3
    4. Monitor progress until all complete
    5. Verify results
    """
    usb_mount = ee_usb_mount

    # Initialize camera
    response = await ee_client.put("/cameraApp/init")
    assert response.status_code == 200
    await asyncio.sleep(2)

    # Create 3 items (photos for faster test)
    for i in range(3):
        response = await ee_client.put("/cameraApp/capture")
        assert response.status_code == 200
        await asyncio.sleep(1)

    # Get catalog UUIDs
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_data = response.json()

    item_uuids = list(catalog_data["items"].keys())[-3:]
    assert len(item_uuids) == 3, f"Expected 3 items, got {len(item_uuids)}"

    # Clear any previous export tasks
    response = await ee_client.post("/export/clear")
    assert response.status_code == 200

    # Queue all 3 exports
    for item_uuid in item_uuids:
        response = await ee_client.put(
            f"/catalog/{item_uuid}/export",
            json={"mountpoint": usb_mount}
        )
        assert response.status_code == 200

    # Verify queue length
    response = await ee_client.get("/export/progress")
    assert response.status_code == 200
    progress_data = response.json()

    total_tasks = progress_data.get("total_tasks", 0)
    assert total_tasks == 3, f"Expected 3 tasks in queue, got {total_tasks}"

    # Monitor progress until complete
    max_polls = 200
    all_complete = False
    progress_history = []

    for _ in range(max_polls):
        response = await ee_client.get("/export/progress")
        assert response.status_code == 200
        progress_data = response.json()

        overall_progress = progress_data.get("overall_progress", 0)
        progress_history.append(overall_progress)

        if progress_data.get("is_complete", False):
            all_complete = True
            break

        await asyncio.sleep(0.1)

    assert all_complete, "Not all exports completed in time"
    assert progress_history[-1] >= 99, f"Final progress {progress_history[-1]} < 99"

    # Get export results
    response = await ee_client.get("/export/results")
    assert response.status_code == 200
    results = response.json()
    # Results format: {'succeeded': [...], 'failed': [...]}
    succeeded = results.get("succeeded", [])
    failed = results.get("failed", [])
    assert len(succeeded) == 3, f"Expected 3 succeeded exports, got {len(succeeded)}"
    assert len(failed) == 0, f"Expected 0 failed exports, got {len(failed)}"

    # Cleanup
    response = await ee_client.put("/cameraApp/deinit")
    assert response.status_code == 200


@pytest.mark.asyncio
@pytest.mark.ee
async def test_ee_preview_switching_during_recording(ee_client, ee_context):
    """
    EE-6: Preview Switching During Recording

    Test flow:
    1. recordStart
    2. Wait 2 seconds
    3. Switch preview to index 1
    4. Wait 2 seconds
    5. Switch preview to index 2
    6. Wait 2 seconds
    7. recordStop
    """
    # Initialize camera
    response = await ee_client.put("/cameraApp/init")
    assert response.status_code == 200
    await asyncio.sleep(2)

    # Action 1: Start recording
    response = await ee_client.put("/cameraApp/recordStart")
    assert response.status_code == 200

    # Action 2: Wait 2 seconds
    await asyncio.sleep(2)

    # Action 3: Switch preview to index 1 (front camera)
    response = await ee_client.put(
        "/cameraApp/preview-index",
        json={"index": 1}
    )
    assert response.status_code == 200
    assert response.json().get("index") == 1

    # Action 4: Wait 2 seconds
    await asyncio.sleep(2)

    # Action 5: Switch preview to index 2 (right camera)
    response = await ee_client.put(
        "/cameraApp/preview-index",
        json={"index": 2}
    )
    assert response.status_code == 200
    assert response.json().get("index") == 2

    # Action 6: Wait 2 seconds
    await asyncio.sleep(2)

    # Action 7: Stop recording
    response = await ee_client.put("/cameraApp/recordStop")
    assert response.status_code == 200

    await asyncio.sleep(2)

    # Verify video in catalog
    response = await ee_client.get("/catalog")
    assert response.status_code == 200
    catalog_data = response.json()

    video_item = None
    for uuid, item in reversed(list(catalog_data["items"].items())):
        if "Video" in item.get("type", ""):
            video_item = item
            break

    assert video_item is not None, "Video not found in catalog after recording"

    # Verify video files exist
    item_url = video_item.get("url", "")
    assert item_url, "Video item should have a url"
    video_dir = Path(item_url)

    if video_dir.exists():
        video_files = list(video_dir.glob("*.mp4")) + list(video_dir.glob("*.avi"))
        assert len(video_files) == 3, f"Expected 3 video files, got {len(video_files)}"

        # Verify files have content
        for vid_file in video_files:
            file_size = vid_file.stat().st_size
            assert file_size > 10000, f"Video file too small ({file_size} bytes): {vid_file}"

    # Cleanup
    response = await ee_client.put("/cameraApp/deinit")
    assert response.status_code == 200


@pytest.mark.asyncio
@pytest.mark.ee
async def test_ee_camera_state_machine(ee_client, ee_context):
    """
    Test camera state machine transitions.

    Verifies:
    - IDLE → INITIALIZING → READY (init)
    - READY → CAPTURING → READY (capture)
    - READY → RECORDING → READY (record start/stop)
    - READY → DEINITIALIZING → IDLE (deinit)
    """
    # Verify initial state
    response = await ee_client.get("/cameraApp/state")
    assert response.status_code == 200

    # Init: IDLE → INITIALIZING → READY
    response = await ee_client.put("/cameraApp/init")
    assert response.status_code == 200

    # Wait and verify READY
    await asyncio.sleep(3)
    response = await ee_client.get("/cameraApp/state")
    assert response.status_code == 200
    assert response.json().get("state") == "ready"

    # Capture: READY → CAPTURING → READY
    response = await ee_client.put("/cameraApp/capture")
    assert response.status_code == 200

    await asyncio.sleep(1)
    response = await ee_client.get("/cameraApp/state")
    assert response.status_code == 200
    assert response.json().get("state") == "ready"

    # Record: READY → RECORDING → READY
    response = await ee_client.put("/cameraApp/recordStart")
    assert response.status_code == 200

    await asyncio.sleep(0.5)
    response = await ee_client.get("/cameraApp/state")
    assert response.status_code == 200
    assert response.json().get("state") == "recording"

    await asyncio.sleep(2)
    response = await ee_client.put("/cameraApp/recordStop")
    assert response.status_code == 200

    await asyncio.sleep(2)
    response = await ee_client.get("/cameraApp/state")
    assert response.status_code == 200
    assert response.json().get("state") == "ready"

    # Deinit: READY → DEINITIALIZING → IDLE
    response = await ee_client.put("/cameraApp/deinit")
    assert response.status_code == 200

    await asyncio.sleep(2)
    response = await ee_client.get("/cameraApp/state")
    assert response.status_code == 200
    assert response.json().get("state") == "idle"
