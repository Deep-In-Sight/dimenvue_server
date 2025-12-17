"""
Integration tests for Camera API endpoints.

Tests camera initialization, capture, and video recording functionality
with proper state management and error handling.
"""

import asyncio
from pathlib import Path

import pytest
from unittest.mock import patch, MagicMock
from camera_app import CameraState


@pytest.mark.asyncio
@pytest.mark.integration
class TestCameraInitialization:
    """Test suite for camera initialization and deinitialization (12 tests)."""

    async def test_camera_init_success(self, async_client, test_app):
        """1. Camera Init Success - PUT /cameraApp/init when not initialized."""
        # Verify initial state
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.status_code == 200
        assert state_resp.json()["state"] == "idle"

        # Initialize camera
        response = await async_client.put("/cameraApp/init")
        assert response.status_code == 200

        # Verify camera is now ready
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.status_code == 200
        assert state_resp.json()["state"] == "ready"

    async def test_camera_init_idempotency(self, async_client, test_app):
        """2. Camera Init Idempotency - PUT /cameraApp/init when already initialized."""
        # First init
        response1 = await async_client.put("/cameraApp/init")
        assert response1.status_code == 200

        # Second init (should be idempotent)
        response2 = await async_client.put("/cameraApp/init")
        assert response2.status_code == 200

        # Verify state is still ready
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.status_code == 200
        assert state_resp.json()["state"] == "ready"

    async def test_camera_deinit(self, async_client, test_app):
        """3. Camera Deinit - PUT /cameraApp/deinit when initialized."""
        # Initialize first
        await async_client.put("/cameraApp/init")

        # Deinitialize
        response = await async_client.put("/cameraApp/deinit")
        assert response.status_code == 200

        # Verify state is idle
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.status_code == 200
        assert state_resp.json()["state"] == "idle"

    async def test_deinit_idempotency(self, async_client, test_app):
        """4. Deinit Idempotency - PUT /cameraApp/deinit when already idle."""
        # Ensure camera is idle
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.json()["state"] == "idle"

        # Deinit when already idle
        response = await async_client.put("/cameraApp/deinit")
        assert response.status_code == 200

        # Verify response indicates already deinitialized
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.status_code == 200
        data = state_resp.json()
        assert data["state"] == "idle"

    async def test_get_camera_state(self, async_client, test_app):
        """5. Get Camera State - GET /cameraApp/state returns state."""
        response = await async_client.get("/cameraApp/state")
        assert response.status_code == 200

        data = response.json()
        assert "state" in data
        assert data["state"] in ["idle", "initializing", "ready", "capturing",
                                  "record_starting", "recording", "record_stopping",
                                  "deinitializing", "error"]

    async def test_concurrent_init_requests(self, async_client, test_app):
        """6. Concurrent Init Requests - 3 concurrent init requests."""
        # Send 3 concurrent init requests
        tasks = [
            async_client.put("/cameraApp/init"),
            async_client.put("/cameraApp/init"),
            async_client.put("/cameraApp/init")
        ]
        responses = await asyncio.gather(*tasks, return_exceptions=True)

        # At least one should succeed (200)
        status_codes = [r.status_code for r in responses if not isinstance(r, Exception)]
        assert 200 in status_codes

        # Others should be 200 (already_initialized) or 409 (conflict)
        for code in status_codes:
            assert code in [200, 409]

        # Final state should be ready
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.json()["state"] == "ready"

    async def test_init_during_deinit(self, async_client, test_app):
        """7. Init During Deinit - Init while deinit in progress."""
        import server_app

        # Initialize first
        await async_client.put("/cameraApp/init")

        # Mock deinit to be slow
        original_deinit = server_app.cameraApp._do_gst_deinit

        async def slow_deinit():
            server_app.cameraApp.state = CameraState.DEINITIALIZING
            await asyncio.sleep(0.5)
            original_deinit()
            server_app.cameraApp.state = CameraState.IDLE

        with patch.object(server_app.cameraApp, '_do_gst_deinit', side_effect=lambda: None):
            # Start deinit
            deinit_task = asyncio.create_task(slow_deinit())

            # Wait a bit to ensure deinit is in progress
            await asyncio.sleep(0.1)

            # Try to init during deinit
            response = await async_client.put("/cameraApp/init")
            assert response.status_code == 409

            # Wait for deinit to complete
            await deinit_task

    async def test_deinit_during_init(self, async_client, test_app):
        """8. Deinit During Init - Deinit while init in progress."""
        import server_app

        # Mock init to be slow
        original_init = server_app.cameraApp._do_gst_init

        async def slow_init():
            server_app.cameraApp.state = CameraState.INITIALIZING
            await asyncio.sleep(0.5)
            original_init()
            server_app.cameraApp.state = CameraState.READY

        with patch.object(server_app.cameraApp, '_do_gst_init', side_effect=lambda: None):
            # Start init
            init_task = asyncio.create_task(slow_init())

            # Wait a bit to ensure init is in progress
            await asyncio.sleep(0.1)

            # Try to deinit during init
            response = await async_client.put("/cameraApp/deinit")
            assert response.status_code == 409

            # Wait for init to complete
            await init_task

    async def test_multiple_concurrent_deinit_requests(self, async_client, test_app):
        """9. Multiple Concurrent Deinit Requests - 3 concurrent deinit."""
        # Initialize first
        await async_client.put("/cameraApp/init")

        # Send 3 concurrent deinit requests
        tasks = [
            async_client.put("/cameraApp/deinit"),
            async_client.put("/cameraApp/deinit"),
            async_client.put("/cameraApp/deinit")
        ]
        responses = await asyncio.gather(*tasks, return_exceptions=True)

        # At least one should succeed (200)
        status_codes = [r.status_code for r in responses if not isinstance(r, Exception)]
        assert 200 in status_codes

        # Others should be 200 (already_deinitialized) or 409 (conflict)
        for code in status_codes:
            assert code in [200, 409]

        # Final state should be idle
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.json()["state"] == "idle"

    async def test_get_state_during_init(self, async_client, test_app):
        """10. Get State During Init - State during init shows initializing."""
        import server_app

        # Mock init to be slow
        async def slow_init():
            server_app.cameraApp.state = CameraState.INITIALIZING
            await asyncio.sleep(0.5)
            server_app.cameraApp._do_gst_init()
            server_app.cameraApp.state = CameraState.READY

        with patch.object(server_app.cameraApp, '_do_gst_init', side_effect=lambda: None):
            # Start init
            init_task = asyncio.create_task(slow_init())

            # Wait a bit to ensure init is in progress
            await asyncio.sleep(0.1)

            # Get state during init
            response = await async_client.get("/cameraApp/state")
            assert response.status_code == 200
            data = response.json()
            assert data["state"] == "initializing"

            # Wait for init to complete
            await init_task

    async def test_get_state_during_deinit(self, async_client, test_app):
        """11. Get State During Deinit - State during deinit shows deinitializing."""
        import server_app

        # Initialize first
        await async_client.put("/cameraApp/init")

        # Mock deinit to be slow
        async def slow_deinit():
            server_app.cameraApp.state = CameraState.DEINITIALIZING
            await asyncio.sleep(0.5)
            server_app.cameraApp._do_gst_deinit()
            server_app.cameraApp.state = CameraState.IDLE

        with patch.object(server_app.cameraApp, '_do_gst_deinit', side_effect=lambda: None):
            # Start deinit
            deinit_task = asyncio.create_task(slow_deinit())

            # Wait a bit to ensure deinit is in progress
            await asyncio.sleep(0.1)

            # Get state during deinit
            response = await async_client.get("/cameraApp/state")
            assert response.status_code == 200
            data = response.json()
            assert data["state"] == "deinitializing"

            # Wait for deinit to complete
            await deinit_task

    async def test_rapid_init_deinit_cycle(self, async_client, test_app):
        """12. Rapid Init/Deinit Cycle - 5 cycles of init/deinit."""
        for i in range(5):
            # Init
            init_resp = await async_client.put("/cameraApp/init")
            assert init_resp.status_code == 200

            # Verify ready
            state_resp = await async_client.get("/cameraApp/state")
            assert state_resp.json()["state"] == "ready"

            # Deinit
            deinit_resp = await async_client.put("/cameraApp/deinit")
            assert deinit_resp.status_code == 200

            # Verify idle
            state_resp = await async_client.get("/cameraApp/state")
            assert state_resp.json()["state"] == "idle"


@pytest.mark.asyncio
@pytest.mark.integration
class TestCameraCapture:
    """Test suite for camera capture functionality (3 tests)."""

    async def test_single_frame_capture(self, async_client, test_app, tmp_path):
        """13. Single Frame Capture - PUT /cameraApp/capture when initialized."""
        import server_app

        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Create capture_tmp directory with dummy files (simulating GStreamer output)
        capture_tmp = Path(server_app.cameraApp.capture_tmp_path)
        capture_tmp.mkdir(parents=True, exist_ok=True)
        (capture_tmp / "left.jpg").write_bytes(b"fake jpg data")
        (capture_tmp / "front.jpg").write_bytes(b"fake jpg data")
        (capture_tmp / "right.jpg").write_bytes(b"fake jpg data")

        # Mock catalog.add_item and thumbnail generation
        mock_add_item = MagicMock(return_value="test-uuid")
        with patch.object(server_app.cameraApp.catalog, 'add_item', mock_add_item), \
             patch('camera_app.gen_thumbnail_factory'):

            # Capture
            response = await async_client.put("/cameraApp/capture")
            assert response.status_code == 200
            assert "status" in response.json()

            # Verify catalog was updated
            assert mock_add_item.called

    async def test_capture_without_init(self, async_client, test_app):
        """14. Capture Without Init - PUT /cameraApp/capture when not initialized."""
        # Ensure camera is not initialized
        state_resp = await async_client.get("/cameraApp/state")
        if state_resp.json()["state"] != "idle":
            await async_client.put("/cameraApp/deinit")

        # Try to capture without init
        response = await async_client.put("/cameraApp/capture")
        assert response.status_code == 409

    async def test_multiple_sequential_captures(self, async_client, test_app):
        """15. Multiple Sequential Captures - 3 sequential captures."""
        import server_app

        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Mock catalog.add_item and thumbnail generation
        mock_add_item = MagicMock(return_value="test-uuid")
        with patch.object(server_app.cameraApp.catalog, 'add_item', mock_add_item), \
             patch('camera_app.gen_thumbnail_factory'):

            # Perform 3 sequential captures
            for i in range(3):
                # Create capture_tmp directory with dummy files before each capture
                capture_tmp = Path(server_app.cameraApp.capture_tmp_path)
                capture_tmp.mkdir(parents=True, exist_ok=True)
                (capture_tmp / "left.jpg").write_bytes(b"fake jpg data")
                (capture_tmp / "front.jpg").write_bytes(b"fake jpg data")
                (capture_tmp / "right.jpg").write_bytes(b"fake jpg data")

                response = await async_client.put("/cameraApp/capture")
                assert response.status_code == 200

                # Verify state returns to ready after each capture
                state_resp = await async_client.get("/cameraApp/state")
                assert state_resp.json()["state"] == "ready"

            # Verify catalog was updated 3 times
            assert mock_add_item.call_count >= 3


@pytest.mark.asyncio
@pytest.mark.integration
class TestVideoRecording:
    """Test suite for video recording functionality (5 tests)."""

    async def test_start_recording(self, async_client, test_app):
        """16. Start Recording - PUT /cameraApp/recordStart when initialized."""
        import server_app

        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Mock record start
        with patch('camera_app.os.makedirs'), \
             patch('camera_app.shutil.rmtree'):

            # Start recording
            response = await async_client.put("/cameraApp/recordStart")
            assert response.status_code == 200
            assert "status" in response.json()

            # Verify state is recording
            state_resp = await async_client.get("/cameraApp/state")
            assert state_resp.json()["state"] == "recording"

    async def test_stop_recording(self, async_client, test_app):
        """17. Stop Recording - PUT /cameraApp/recordStop after recording."""
        import server_app

        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Create record_tmp directory with dummy files (simulating GStreamer output)
        record_tmp = Path(server_app.cameraApp.record_tmp_path)
        record_tmp.mkdir(parents=True, exist_ok=True)
        (record_tmp / "left.mp4").write_bytes(b"fake mp4 data")
        (record_tmp / "front.mp4").write_bytes(b"fake mp4 data")
        (record_tmp / "right.mp4").write_bytes(b"fake mp4 data")

        # Mock catalog.add_item and thumbnail generation
        mock_add_item = MagicMock(return_value="test-uuid")
        with patch.object(server_app.cameraApp.catalog, 'add_item', mock_add_item), \
             patch('camera_app.gen_thumbnail_factory'):

            # Start recording
            await async_client.put("/cameraApp/recordStart")

            # Wait a bit (simulating 3 seconds)
            await asyncio.sleep(0.2)

            # Stop recording
            response = await async_client.put("/cameraApp/recordStop")
            assert response.status_code == 200
            assert "status" in response.json()

            # Verify state returns to ready
            state_resp = await async_client.get("/cameraApp/state")
            assert state_resp.json()["state"] == "ready"

            # Verify catalog was updated
            assert mock_add_item.called

    async def test_record_start_without_init(self, async_client, test_app):
        """18. Record Start Without Init - PUT /cameraApp/recordStart when not initialized."""
        # Ensure camera is not initialized
        state_resp = await async_client.get("/cameraApp/state")
        if state_resp.json()["state"] != "idle":
            await async_client.put("/cameraApp/deinit")

        # Try to start recording without init
        response = await async_client.put("/cameraApp/recordStart")
        assert response.status_code == 409

    async def test_duplicate_record_start(self, async_client, test_app):
        """19. Duplicate Record Start - PUT /cameraApp/recordStart when already recording."""
        import server_app

        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Mock recording operations
        with patch('camera_app.os.makedirs'), \
             patch('camera_app.shutil.rmtree'):

            # Start recording
            response1 = await async_client.put("/cameraApp/recordStart")
            assert response1.status_code == 200

            # Try to start recording again
            response2 = await async_client.put("/cameraApp/recordStart")
            assert response2.status_code == 409

    async def test_record_stop_without_start(self, async_client, test_app):
        """20. Record Stop Without Start - PUT /cameraApp/recordStop when not recording."""
        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Try to stop recording without starting
        response = await async_client.put("/cameraApp/recordStop")
        assert response.status_code == 409


@pytest.mark.asyncio
@pytest.mark.integration
class TestPreviewSwitching:
    """Test suite for preview switching functionality (3 tests)."""

    async def test_switch_preview_index(self, async_client, test_app):
        """21. Switch Preview Index - PUT /cameraApp/preview-index with valid index."""
        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Switch preview to index 1 (front camera)
        response = await async_client.put(
            "/cameraApp/preview-index",
            json={"index": 1}
        )

        assert response.status_code == 200
        data = response.json()
        assert data["index"] == 1

        # Verify the index is actually changed
        get_response = await async_client.get("/cameraApp/preview-index")
        assert get_response.status_code == 200
        assert get_response.json()["index"] == 1

    async def test_get_preview_index(self, async_client, test_app):
        """22. Get Preview Index - GET /cameraApp/preview-index returns current index."""
        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Get current preview index
        response = await async_client.get("/cameraApp/preview-index")

        assert response.status_code == 200
        data = response.json()
        assert "index" in data
        # Default index should be 0 (left camera)
        assert isinstance(data["index"], int)
        assert data["index"] >= 0

    async def test_invalid_preview_index(self, async_client, test_app):
        """23. Invalid Preview Index - PUT /cameraApp/preview-index with invalid index."""
        # Initialize camera
        await async_client.put("/cameraApp/init")

        # Get current index before attempting invalid change
        initial_response = await async_client.get("/cameraApp/preview-index")
        initial_index = initial_response.json()["index"]

        # Try to switch to invalid index (99)
        response = await async_client.put(
            "/cameraApp/preview-index",
            json={"index": 99}
        )

        assert response.status_code == 400
        data = response.json()
        assert "detail" in data

        # Verify current preview index unchanged
        get_response = await async_client.get("/cameraApp/preview-index")
        assert get_response.json()["index"] == initial_index


@pytest.mark.asyncio
@pytest.mark.integration
class TestCameraSettings:
    """Test suite for camera settings functionality (3 tests)."""

    async def test_get_camera_settings(self, async_client, test_app):
        """24. Get Camera Settings - GET /cameraApp/settings returns settings structure."""
        response = await async_client.get("/cameraApp/settings")

        assert response.status_code == 200
        settings = response.json()

        # Verify settings structure matches DEFAULT_CAMERA_SETTINGS
        # Each setting should have either "options" or "range", and "current_selection"
        expected_keys = ["resolution", "framerate", "capture_format",
                         "capture_quality", "record_format", "record_bitrate",
                         "preview_quality"]

        for key in expected_keys:
            if key in settings:
                assert "current_selection" in settings[key], f"Setting {key} missing 'current_selection'"
                assert isinstance(settings[key]["current_selection"], int)

                # Settings can have either "options" (list) or "range" (min/max)
                if "options" in settings[key]:
                    assert isinstance(settings[key]["options"], list)
                    # current_selection should be a valid index
                    assert 0 <= settings[key]["current_selection"] < len(settings[key]["options"])
                elif "range" in settings[key]:
                    assert isinstance(settings[key]["range"], list)
                    assert len(settings[key]["range"]) == 2
                    # current_selection should be within range
                    min_val, max_val = settings[key]["range"]
                    assert min_val <= settings[key]["current_selection"] <= max_val
                else:
                    pytest.fail(f"Setting {key} missing both 'options' and 'range'")

    async def test_update_camera_settings(self, async_client, test_app):
        """25. Update Camera Settings - PUT /cameraApp/settings updates and persists settings."""
        # Get current settings
        get_response = await async_client.get("/cameraApp/settings")
        assert get_response.status_code == 200
        current_settings = get_response.json()

        # Prepare updated settings - change current_selection for available keys
        new_settings = current_settings.copy()

        # Find a setting we can change
        changed_key = None
        for key in new_settings:
            if isinstance(new_settings[key], dict) and "options" in new_settings[key]:
                options = new_settings[key]["options"]
                current = new_settings[key]["current_selection"]
                if len(options) > 1:
                    # Change to a different valid selection
                    new_selection = (current + 1) % len(options)
                    new_settings[key]["current_selection"] = new_selection
                    changed_key = key
                    break

        if changed_key is None:
            pytest.skip("No settings available to change")

        # Update settings
        update_response = await async_client.put(
            "/cameraApp/settings",
            json={"settings": new_settings}
        )

        assert update_response.status_code == 200
        updated_settings = update_response.json()

        # Verify the setting was updated
        assert updated_settings[changed_key]["current_selection"] == new_settings[changed_key]["current_selection"]

        # Verify settings persist (get again)
        verify_response = await async_client.get("/cameraApp/settings")
        assert verify_response.status_code == 200
        persisted_settings = verify_response.json()
        assert persisted_settings[changed_key]["current_selection"] == new_settings[changed_key]["current_selection"]

    async def test_update_camera_settings_invalid_keys(self, async_client, test_app):
        """26. Update Camera Settings With Invalid Keys - Invalid keys are rejected."""
        # Get current settings
        get_response = await async_client.get("/cameraApp/settings")
        assert get_response.status_code == 200
        current_settings = get_response.json()

        # Create settings with invalid keys
        invalid_settings = current_settings.copy()
        invalid_settings["wrong_key"] = {"options": ["a", "b"], "current_selection": 0}
        invalid_settings["frame_rate"] = {"options": [30, 60], "current_selection": 0}  # typo

        # Try to update with invalid settings
        response = await async_client.put(
            "/cameraApp/settings",
            json={"settings": invalid_settings}
        )

        # Should return 400 Bad Request for invalid keys
        assert response.status_code == 400
        data = response.json()
        assert "detail" in data
