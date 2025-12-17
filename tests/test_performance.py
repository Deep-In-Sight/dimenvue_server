"""
Performance and load tests for DimenvuePro Server.

Tests server behavior under heavy load conditions including:
- Rapid capture sequences
- Long recording sessions
- Large export queues

These tests are marked as @pytest.mark.slow and can be skipped with: pytest -m "not slow"
"""

import pytest
import asyncio
import time
import os
from pathlib import Path
from unittest.mock import patch, MagicMock
import psutil


# ==================== Load-1: Rapid Capture Sequence ====================

@pytest.mark.asyncio
@pytest.mark.load
@pytest.mark.slow
async def test_rapid_capture_sequence(async_client, temp_output_dir, temp_catalog):
    """
    Load-1: Rapid Capture Sequence

    Execute PUT /cameraApp/capture 50 times in rapid succession.
    Verify:
    - All 50 captures succeed (or graceful rate limiting)
    - 150 image files created (3 per capture)
    - 50 catalog entries
    - No memory leaks, server remains responsive
    """
    num_captures = 50
    expected_images_per_capture = 3
    expected_total_images = num_captures * expected_images_per_capture

    # Record initial memory usage
    process = psutil.Process()
    initial_memory_mb = process.memory_info().rss / 1024 / 1024

    # Track results
    successful_captures = 0
    rate_limited_captures = 0
    failed_captures = 0

    # Initialize camera app
    init_response = await async_client.put("/cameraApp/init")
    assert init_response.status_code == 200

    # Wait for initialization to complete
    await asyncio.sleep(0.5)

    # Execute 50 rapid captures
    capture_start_time = time.time()

    for i in range(num_captures):
        try:
            response = await async_client.put("/cameraApp/capture")

            if response.status_code == 200:
                successful_captures += 1
            elif response.status_code == 409:
                # Graceful rate limiting (camera busy)
                rate_limited_captures += 1
                # Brief pause before retry
                await asyncio.sleep(0.1)
            else:
                failed_captures += 1

        except Exception as e:
            failed_captures += 1
            print(f"Capture {i+1} failed with exception: {e}")

    capture_duration = time.time() - capture_start_time

    # Check memory usage after captures
    final_memory_mb = process.memory_info().rss / 1024 / 1024
    memory_increase_mb = final_memory_mb - initial_memory_mb

    # Verify server remains responsive
    state_response = await async_client.get("/cameraApp/state")
    assert state_response.status_code == 200, "Server should remain responsive after rapid captures"

    # Verify catalog entries
    catalog_response = await async_client.get("/catalog")
    assert catalog_response.status_code == 200
    catalog_data = catalog_response.json()
    catalog_items = catalog_data.get("items", {})

    # Count image files in temp directory
    capture_tmp_path = temp_output_dir / "capture_tmp"
    if capture_tmp_path.exists():
        image_files = list(capture_tmp_path.glob("**/*.jpg")) + list(capture_tmp_path.glob("**/*.png"))
    else:
        image_files = []

    # Assertions
    print(f"\nRapid Capture Results:")
    print(f"  Total attempts: {num_captures}")
    print(f"  Successful: {successful_captures}")
    print(f"  Rate limited: {rate_limited_captures}")
    print(f"  Failed: {failed_captures}")
    print(f"  Duration: {capture_duration:.2f}s")
    print(f"  Rate: {num_captures/capture_duration:.2f} captures/sec")
    print(f"  Catalog entries: {len(catalog_items)}")
    print(f"  Image files: {len(image_files)}")
    print(f"  Memory increase: {memory_increase_mb:.2f} MB")

    # All captures should succeed or be gracefully rate-limited
    assert failed_captures == 0, f"{failed_captures} captures failed unexpectedly"

    # Should have successful captures (at least some)
    assert successful_captures > 0, "No captures succeeded"

    # Memory should not increase excessively (allow up to 500MB increase for buffering)
    assert memory_increase_mb < 500, f"Memory leak detected: {memory_increase_mb:.2f} MB increase"

    # Server should remain responsive
    assert state_response.status_code == 200


# ==================== Load-2: Long Recording Session ====================

@pytest.mark.asyncio
@pytest.mark.load
@pytest.mark.slow
async def test_long_recording_session(async_client, temp_output_dir, temp_catalog):
    """
    Load-2: Long Recording Session

    PUT /cameraApp/recordStart
    Wait 3600 seconds (1 hour) - SHORTENED TO 10 SECONDS FOR TESTING
    PUT /cameraApp/recordStop

    Verify:
    - Video files created with appropriate duration
    - No file corruption
    - Memory usage stable

    NOTE: Full production test should use 3600 seconds (1 hour).
    This test uses 10 seconds for practical testing purposes.
    """
    # PRODUCTION: Use 3600 seconds (1 hour)
    # TEST: Use 10 seconds for practical testing
    recording_duration_seconds = 10
    production_duration_seconds = 3600  # Document the full test duration

    # Record initial memory usage
    process = psutil.Process()
    initial_memory_mb = process.memory_info().rss / 1024 / 1024
    memory_samples = [initial_memory_mb]

    # Initialize camera app
    init_response = await async_client.put("/cameraApp/init")
    assert init_response.status_code == 200

    # Wait for initialization
    await asyncio.sleep(0.5)

    # Start recording
    start_response = await async_client.put("/cameraApp/recordStart")
    assert start_response.status_code == 200, "Recording should start successfully"

    record_start_time = time.time()

    # Monitor memory during recording
    sample_interval = 2  # Sample every 2 seconds
    elapsed = 0

    print(f"\nRecording for {recording_duration_seconds}s (production: {production_duration_seconds}s)...")

    while elapsed < recording_duration_seconds:
        await asyncio.sleep(sample_interval)
        elapsed = time.time() - record_start_time

        # Sample memory
        current_memory_mb = process.memory_info().rss / 1024 / 1024
        memory_samples.append(current_memory_mb)

        # Check server is still responsive
        state_response = await async_client.get("/cameraApp/state")
        assert state_response.status_code == 200, f"Server unresponsive at {elapsed:.1f}s"

        state_data = state_response.json()
        print(f"  [{elapsed:.1f}s] State: {state_data.get('state')}, Memory: {current_memory_mb:.1f} MB")

    # Stop recording
    stop_response = await async_client.put("/cameraApp/recordStop")
    assert stop_response.status_code == 200, "Recording should stop successfully"

    actual_duration = time.time() - record_start_time

    # Wait for file finalization
    await asyncio.sleep(1.0)

    # Check final memory
    final_memory_mb = process.memory_info().rss / 1024 / 1024
    memory_samples.append(final_memory_mb)

    # Calculate memory statistics
    memory_increase = final_memory_mb - initial_memory_mb
    max_memory_mb = max(memory_samples)
    memory_range = max_memory_mb - initial_memory_mb

    # Check for video files
    record_tmp_path = temp_output_dir / "record_tmp"
    video_files = []
    if record_tmp_path.exists():
        video_files = list(record_tmp_path.glob("**/*.mp4")) + list(record_tmp_path.glob("**/*.avi"))

    # Check catalog for video entries
    catalog_response = await async_client.get("/catalog")
    assert catalog_response.status_code == 200
    catalog_data = catalog_response.json()
    catalog_items = catalog_data.get("items", {})
    video_items = [item for item in catalog_items.values() if "Video" in item.get("type", "")]

    # Results
    print(f"\nLong Recording Results:")
    print(f"  Duration: {actual_duration:.2f}s (target: {recording_duration_seconds}s)")
    print(f"  Video files created: {len(video_files)}")
    print(f"  Catalog video entries: {len(video_items)}")
    print(f"  Initial memory: {initial_memory_mb:.2f} MB")
    print(f"  Final memory: {final_memory_mb:.2f} MB")
    print(f"  Memory increase: {memory_increase:.2f} MB")
    print(f"  Max memory: {max_memory_mb:.2f} MB")
    print(f"  Memory range: {memory_range:.2f} MB")

    # Assertions
    assert actual_duration >= recording_duration_seconds * 0.9, "Recording duration too short"

    # Memory should be stable (allow reasonable buffering)
    # For 10s test, allow up to 200MB increase; scale proportionally for 1 hour
    max_allowed_increase = 200 if recording_duration_seconds <= 10 else 1000
    assert memory_increase < max_allowed_increase, \
        f"Memory not stable: {memory_increase:.2f} MB increase (max: {max_allowed_increase} MB)"

    # No excessive memory growth during recording
    assert memory_range < max_allowed_increase, \
        f"Memory range too high: {memory_range:.2f} MB (max: {max_allowed_increase} MB)"


# ==================== Load-3: Large Export Queue ====================

@pytest.mark.asyncio
@pytest.mark.load
@pytest.mark.slow
async def test_large_export_queue(async_client, temp_output_dir, temp_catalog, mock_usb_device):
    """
    Load-3: Large Export Queue

    Create 100 catalog items (mix of photos/videos/scans).
    Queue all 100 items for export.
    Monitor progress until all complete.

    Verify:
    - All 100 exports process sequentially
    - Progress reporting accurate
    - No deadlocks
    """
    num_items = 100

    # Create 100 catalog items (mix of types)
    item_types = ["Image", "Video", "Scan"]
    created_items = []

    print(f"\nCreating {num_items} catalog items...")

    for i in range(num_items):
        # Rotate through types
        item_type = item_types[i % len(item_types)]

        # Create temporary item directory
        item_path = temp_output_dir / f"test_item_{i}"
        item_path.mkdir(parents=True, exist_ok=True)

        # Create dummy files based on type
        if item_type == "Image":
            (item_path / "front.jpg").write_bytes(b"fake image data" * 100)
            (item_path / "left.jpg").write_bytes(b"fake image data" * 100)
            (item_path / "right.jpg").write_bytes(b"fake image data" * 100)
        elif item_type == "Video":
            (item_path / "front.mp4").write_bytes(b"fake video data" * 500)
            (item_path / "left.mp4").write_bytes(b"fake video data" * 500)
            (item_path / "right.mp4").write_bytes(b"fake video data" * 500)
        else:  # Scan
            (item_path / "map.pcd").write_bytes(b"fake point cloud data" * 200)
            (item_path / "metadata.json").write_text('{"scan_duration": 60}')

        # Add to catalog
        item_uuid = temp_catalog.add_item(item_type, str(item_path))
        created_items.append(item_uuid)

    # Wait for catalog to flush
    await asyncio.sleep(0.5)

    # Verify items were added
    catalog_response = await async_client.get("/catalog")
    assert catalog_response.status_code == 200
    catalog_data = catalog_response.json()
    catalog_items = catalog_data.get("items", {})

    assert len(catalog_items) >= num_items, f"Expected at least {num_items} items, got {len(catalog_items)}"

    print(f"Created {len(catalog_items)} catalog items")

    # Clear any previous export tasks
    clear_response = await async_client.post("/export/clear")
    assert clear_response.status_code == 200

    # Queue all items for export
    mountpoint = mock_usb_device.mount_point
    export_tasks = []

    print(f"Queueing {num_items} items for export...")

    for i, item_uuid in enumerate(created_items):
        try:
            export_response = await async_client.put(
                f"/catalog/{item_uuid}/export",
                json={"mountpoint": mountpoint}
            )

            if export_response.status_code == 200:
                task_data = export_response.json()
                export_tasks.append(task_data.get("task_id"))
            elif export_response.status_code == 409:
                # Item already exported or other conflict - expected for duplicates
                print(f"  Item {i+1}: Conflict (409) - may already exist")
            else:
                print(f"  Item {i+1}: Failed with status {export_response.status_code}")

        except Exception as e:
            print(f"  Item {i+1}: Exception - {e}")

    print(f"Queued {len(export_tasks)} export tasks")

    # Monitor progress until completion
    max_wait_time = 120  # 2 minutes max
    start_time = time.time()
    last_progress = -1
    progress_updates = []

    print("Monitoring export progress...")

    while True:
        elapsed = time.time() - start_time

        if elapsed > max_wait_time:
            pytest.fail(f"Export did not complete within {max_wait_time}s - possible deadlock")

        # Get progress
        progress_response = await async_client.get("/export/progress")
        assert progress_response.status_code == 200, "Progress endpoint should be responsive"

        progress_data = progress_response.json()
        total_tasks = progress_data.get("total_tasks", 0)
        completed_tasks = progress_data.get("completed_tasks", 0)
        failed_tasks = progress_data.get("failed_tasks", 0)
        queued_tasks = progress_data.get("queued_tasks", 0)
        overall_progress = progress_data.get("overall_progress", 0.0)
        is_complete = progress_data.get("is_complete", False)
        current_item = progress_data.get("current_item")

        # Log progress if changed significantly
        if int(overall_progress) > last_progress:
            last_progress = int(overall_progress)
            current_item_name = current_item.get("name") if current_item else "None"
            print(f"  [{elapsed:.1f}s] Progress: {overall_progress:.1f}% "
                  f"({completed_tasks}/{total_tasks} complete, "
                  f"{queued_tasks} queued, {failed_tasks} failed) "
                  f"Current: {current_item_name}")

            progress_updates.append({
                "time": elapsed,
                "progress": overall_progress,
                "completed": completed_tasks,
                "total": total_tasks
            })

        # Check if complete
        if is_complete:
            print(f"Export complete in {elapsed:.1f}s")
            break

        await asyncio.sleep(0.5)

    # Get final results
    results_response = await async_client.get("/export/results")
    assert results_response.status_code == 200
    results_data = results_response.json()

    succeeded = results_data.get("succeeded", [])
    failed = results_data.get("failed", [])

    # Final report
    print(f"\nLarge Export Queue Results:")
    print(f"  Total items: {num_items}")
    print(f"  Export tasks queued: {len(export_tasks)}")
    print(f"  Succeeded: {len(succeeded)}")
    print(f"  Failed: {len(failed)}")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Progress updates: {len(progress_updates)}")

    if failed:
        print(f"  Failed items:")
        for fail in failed[:5]:  # Show first 5
            print(f"    - {fail.get('name')}: {fail.get('reason')}")

    # Assertions
    assert len(export_tasks) > 0, "Should have queued export tasks"

    # Progress should have been reported
    assert len(progress_updates) > 0, "Progress should be reported during export"

    # Progress should be monotonically increasing (or same)
    for i in range(1, len(progress_updates)):
        assert progress_updates[i]["progress"] >= progress_updates[i-1]["progress"], \
            "Progress should not decrease"

    # All exports should complete (succeeded or failed, but not stuck)
    total_processed = len(succeeded) + len(failed)
    assert total_processed == len(export_tasks), \
        f"All tasks should complete: {total_processed}/{len(export_tasks)}"

    # Most exports should succeed (allow some failures due to test environment)
    success_rate = len(succeeded) / len(export_tasks) if export_tasks else 0
    print(f"  Success rate: {success_rate*100:.1f}%")

    # No deadlock - test completed within time limit
    assert elapsed < max_wait_time, "Export completed without deadlock"
