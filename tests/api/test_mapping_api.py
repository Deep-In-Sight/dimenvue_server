"""
Integration tests for Mapping API endpoints.

Tests the mapping lifecycle, state management, concurrency, and settings.
Uses mocked ROS2 functions (StartEverything, StopEverything, GetInitStatus)
and mocked finalize_mapping.do_finalize.

Total tests: 27
- 2.1 Mapping Lifecycle: 13 tests (Tests 1-13)
- 2.2 Mapping State: 5 tests (Tests 14-18)
- 2.3 Mapping Concurrency: 3 tests (Tests 19-21)
- 2.4 Mapping Settings: 4 tests (Tests 22-25)
- 2.5 Scan Name Prefix: 2 tests (Tests 26-27)
"""

import asyncio
import pytest
from unittest.mock import patch, AsyncMock, MagicMock
from httpx import AsyncClient


# ==================== 2.1 Mapping Lifecycle (13 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_start_mapping_success(async_client: AsyncClient):
    """
    Test 1: Start Mapping Success
    PUT /mappingApp/start when idle → 200 OK, state "starting"
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Action: Start mapping
        response = await async_client.put("/mappingApp/start")

        # Expects: 200 OK, state "starting"
        assert response.status_code == 200
        data = response.json()
        assert data["state"] == "starting"
        assert data["details"] == "started"

        # Verify ROS2 start was called
        mock_start.assert_called_once()


@pytest.mark.asyncio
@pytest.mark.integration
async def test_verify_node_launch_after_start(async_client: AsyncClient):
    """
    Test 2: Verify Node Launch After Start
    After start, ROS2 nodes should be launched
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Action: Start mapping
        response = await async_client.put("/mappingApp/start")
        assert response.status_code == 200

        # Expects: StartEverything was called with correct parameters
        mock_start.assert_called_once()
        args = mock_start.call_args[0]

        # Should be called with file_format and artifact_dir
        assert len(args) == 2
        file_format, artifact_dir = args
        assert file_format in ["PLY", "PCD", "LAS", "LAZ"]
        assert "mapping_artifact" in artifact_dir


@pytest.mark.asyncio
@pytest.mark.integration
async def test_automatic_transition_starting_to_initializing(async_client: AsyncClient):
    """
    Test 3: Automatic State Transition STARTING → INITIALIZING
    IMU status "TRACKING" triggers transition
    """
    # Mock GetInitStatus to return TRACKING
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Initially UNKNOWN, then TRACKING
        mock_status.return_value = "UNKNOWN"

        # Start mapping
        response = await async_client.put("/mappingApp/start")
        assert response.status_code == 200

        # Simulate IMU starting to track
        mock_status.return_value = "TRACKING"

        # Wait for polling task to update state
        await asyncio.sleep(1.5)

        # Check state has transitioned to INITIALIZING
        state_response = await async_client.get("/mappingApp/state")
        assert state_response.status_code == 200
        state_data = state_response.json()
        assert state_data["state"] == "initializing"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_automatic_transition_initializing_to_running(async_client: AsyncClient):
    """
    Test 4: Automatic State Transition INITIALIZING → RUNNING
    IMU status "STABILIZED" triggers transition
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Start with UNKNOWN
        mock_status.return_value = "UNKNOWN"

        # Start mapping
        response = await async_client.put("/mappingApp/start")
        assert response.status_code == 200

        # Simulate IMU tracking
        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        # Simulate IMU stabilized
        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Check state has transitioned to RUNNING
        state_response = await async_client.get("/mappingApp/state")
        assert state_response.status_code == 200
        state_data = state_response.json()
        assert state_data["state"] == "running"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_duplicate_mapping_start_running(async_client: AsyncClient):
    """
    Test 5: Duplicate Mapping Start (Running)
    Start when running → 200 OK, "already_running"
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Start mapping and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Try to start again while running
        response = await async_client.put("/mappingApp/start")

        # Expects: 200 OK with already_running
        assert response.status_code == 200
        data = response.json()
        assert data["details"] == "already_running"
        assert data["state"] == "running"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_duplicate_mapping_start_initializing(async_client: AsyncClient):
    """
    Test 6: Duplicate Mapping Start (Initializing)
    Start when initializing → 200 OK, "already_running"
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Start mapping and transition to INITIALIZING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        # Try to start again while initializing
        response = await async_client.put("/mappingApp/start")

        # Expects: 200 OK with already_running
        assert response.status_code == 200
        data = response.json()
        assert data["details"] == "already_running"
        assert data["state"] == "initializing"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_start_during_stopping(async_client: AsyncClient):
    """
    Test 7: Start During Stopping
    Start while stopping → 409 Conflict
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock) as mock_stop, \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Start and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Make stop slow so we can catch the stopping state
        async def slow_stop():
            await asyncio.sleep(0.5)

        mock_stop.side_effect = slow_stop

        # Start stop in background
        stop_task = asyncio.create_task(async_client.put("/mappingApp/stop"))
        await asyncio.sleep(0.1)

        # Try to start while stopping
        start_response = await async_client.put("/mappingApp/start")

        # Expects: 409 Conflict
        assert start_response.status_code == 409
        assert "stopping" in start_response.json()["detail"].lower()

        # Wait for stop to complete
        await stop_task


@pytest.mark.asyncio
@pytest.mark.integration
async def test_start_during_starting_race(async_client: AsyncClient):
    """
    Test 8: Start During Starting (Race)
    Second start during first → 409 Conflict
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Make start slow
        async def slow_start(*args):
            await asyncio.sleep(0.3)

        mock_start.side_effect = slow_start

        # Send two concurrent start requests
        task1 = asyncio.create_task(async_client.put("/mappingApp/start"))
        await asyncio.sleep(0.05)  # Let first one enter
        task2 = asyncio.create_task(async_client.put("/mappingApp/start"))

        responses = await asyncio.gather(task1, task2)

        # First should succeed, second should fail
        status_codes = [r.status_code for r in responses]
        assert 200 in status_codes
        assert 409 in status_codes

        # The 409 response should mention "starting"
        for r in responses:
            if r.status_code == 409:
                assert "starting" in r.json()["detail"].lower()


@pytest.mark.asyncio
@pytest.mark.integration
async def test_stop_mapping_from_running(async_client: AsyncClient):
    """
    Test 9: Stop Mapping from RUNNING
    PUT /mappingApp/stop when running → 200 OK, finalize runs, catalog updated
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock) as mock_stop, \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize') as mock_finalize:

        # Start and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Stop mapping
        response = await async_client.put("/mappingApp/stop")

        # Expects: 200 OK
        assert response.status_code == 200
        data = response.json()
        assert data["details"] == "stopped"
        assert data["state"] == "idle"

        # Verify stop and finalize were called
        mock_stop.assert_called_once()
        mock_finalize.assert_called_once()

        # Verify catalog was updated (check via catalog endpoint)
        catalog_response = await async_client.get("/catalog")
        assert catalog_response.status_code == 200


@pytest.mark.asyncio
@pytest.mark.integration
async def test_stop_mapping_during_initializing(async_client: AsyncClient):
    """
    Test 10: Stop Mapping During INITIALIZING
    Stop when initializing → 409 Conflict
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Start mapping and transition to INITIALIZING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        # Try to stop while initializing
        response = await async_client.put("/mappingApp/stop")

        # Expects: 409 Conflict
        assert response.status_code == 409
        assert "initializing" in response.json()["detail"].lower()


@pytest.mark.asyncio
@pytest.mark.integration
async def test_stop_mapping_during_starting(async_client: AsyncClient):
    """
    Test 11: Stop Mapping During STARTING
    Stop when starting → 409 Conflict
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Make start slow
        async def slow_start(*args):
            await asyncio.sleep(0.5)

        mock_start.side_effect = slow_start

        # Start mapping
        start_task = asyncio.create_task(async_client.put("/mappingApp/start"))
        await asyncio.sleep(0.1)

        # Try to stop while starting
        stop_response = await async_client.put("/mappingApp/stop")

        # Expects: 409 Conflict
        assert stop_response.status_code == 409
        assert "starting" in stop_response.json()["detail"].lower()

        # Wait for start to complete
        await start_task


@pytest.mark.asyncio
@pytest.mark.integration
async def test_stop_mapping_during_stopping(async_client: AsyncClient):
    """
    Test 12: Stop Mapping during STOPPING
    Stop twice rapidly → First 200, second 409
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock) as mock_stop, \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Start and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Make stop slow
        async def slow_stop():
            await asyncio.sleep(0.5)

        mock_stop.side_effect = slow_stop

        # Send two stop requests
        task1 = asyncio.create_task(async_client.put("/mappingApp/stop"))
        await asyncio.sleep(0.05)
        task2 = asyncio.create_task(async_client.put("/mappingApp/stop"))

        responses = await asyncio.gather(task1, task2)

        # First should succeed, second should fail
        assert responses[0].status_code == 200
        assert responses[1].status_code == 409
        assert "stopping" in responses[1].json()["detail"].lower()


@pytest.mark.asyncio
@pytest.mark.integration
async def test_stop_mapping_idempotency(async_client: AsyncClient):
    """
    Test 13: Stop Mapping Idempotency
    Stop when idle → 200 OK, "already_stopped"
    """
    with patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Try to stop when already idle
        response = await async_client.put("/mappingApp/stop")

        # Expects: 200 OK with already_stopped
        assert response.status_code == 200
        data = response.json()
        assert data["details"] == "already_stopped"
        assert data["state"] == "idle"


# ==================== 2.2 Mapping State (5 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_mapping_state_idle(async_client: AsyncClient):
    """
    Test 14: Get Mapping State - IDLE
    GET /mappingApp/state when idle → {"state": "idle"}
    """
    with patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Get state when idle
        response = await async_client.get("/mappingApp/state")

        # Expects: 200 OK, state "idle"
        assert response.status_code == 200
        data = response.json()
        assert data["state"] == "idle"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_mapping_state_starting(async_client: AsyncClient):
    """
    Test 15: Get Mapping State - STARTING
    State immediately after start → {"state": "starting"}
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Make start slow so we can catch starting state
        async def slow_start(*args):
            await asyncio.sleep(0.5)

        mock_start.side_effect = slow_start

        # Start mapping
        start_task = asyncio.create_task(async_client.put("/mappingApp/start"))
        await asyncio.sleep(0.1)

        # Check state during start
        state_response = await async_client.get("/mappingApp/state")
        assert state_response.status_code == 200
        state_data = state_response.json()
        assert state_data["state"] == "starting"

        # Wait for start to complete
        await start_task


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_mapping_state_initializing(async_client: AsyncClient):
    """
    Test 16: Get Mapping State - INITIALIZING
    State when IMU tracking → {"state": "initializing"}
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Start mapping
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        # Transition to INITIALIZING
        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        # Check state
        response = await async_client.get("/mappingApp/state")
        assert response.status_code == 200
        data = response.json()
        assert data["state"] == "initializing"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_mapping_state_running(async_client: AsyncClient):
    """
    Test 17: Get Mapping State - RUNNING
    State when IMU stabilized → {"state": "running"}
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Start mapping and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Check state
        response = await async_client.get("/mappingApp/state")
        assert response.status_code == 200
        data = response.json()
        assert data["state"] == "running"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_mapping_state_stopping(async_client: AsyncClient):
    """
    Test 18: Get Mapping State - STOPPING
    State during stop → {"state": "stopping"}
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock) as mock_stop, \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Start and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Make stop slow so we can catch stopping state
        async def slow_stop():
            await asyncio.sleep(0.5)

        mock_stop.side_effect = slow_stop

        # Start stop in background
        stop_task = asyncio.create_task(async_client.put("/mappingApp/stop"))
        await asyncio.sleep(0.1)

        # Check state during stop
        state_response = await async_client.get("/mappingApp/state")
        assert state_response.status_code == 200
        state_data = state_response.json()
        assert state_data["state"] == "stopping"

        # Wait for stop to complete
        await stop_task


# ==================== 2.3 Mapping Concurrency (3 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_concurrent_start_requests(async_client: AsyncClient):
    """
    Test 19: Concurrent Start Requests
    3 concurrent starts → First succeeds, others 409
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus') as mock_status:

        mock_status.return_value = "UNKNOWN"

        # Send 3 concurrent start requests
        tasks = [
            async_client.put("/mappingApp/start"),
            async_client.put("/mappingApp/start"),
            async_client.put("/mappingApp/start")
        ]
        responses = await asyncio.gather(*tasks)

        # Count successes and failures
        success_count = sum(1 for r in responses if r.status_code == 200)
        conflict_count = sum(1 for r in responses if r.status_code == 409)

        # Expects: Exactly one success
        assert success_count == 1, f"Expected 1 success, got {success_count}"
        assert conflict_count == 2, f"Expected 2 conflicts, got {conflict_count}"

        # Verify only one StartEverything was called
        assert mock_start.call_count == 1

        # Final state should be starting or initializing
        state_response = await async_client.get("/mappingApp/state")
        state_data = state_response.json()
        assert state_data["state"] in ["starting", "initializing"]


@pytest.mark.asyncio
@pytest.mark.integration
async def test_concurrent_stop_requests(async_client: AsyncClient):
    """
    Test 20: Concurrent Stop Requests
    3 concurrent stops → First succeeds, others 409
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock) as mock_stop, \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Start and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Send 3 concurrent stop requests
        tasks = [
            async_client.put("/mappingApp/stop"),
            async_client.put("/mappingApp/stop"),
            async_client.put("/mappingApp/stop")
        ]
        responses = await asyncio.gather(*tasks)

        # Count successes and failures
        success_count = sum(1 for r in responses if r.status_code == 200)
        conflict_count = sum(1 for r in responses if r.status_code == 409)

        # Expects: Exactly one success
        assert success_count == 1, f"Expected 1 success, got {success_count}"
        assert conflict_count == 2, f"Expected 2 conflicts, got {conflict_count}"

        # Verify only one StopEverything was called
        assert mock_stop.call_count == 1

        # Final state should be idle
        state_response = await async_client.get("/mappingApp/state")
        state_data = state_response.json()
        assert state_data["state"] == "idle"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_rapid_start_stop_cycle(async_client: AsyncClient):
    """
    Test 21: Rapid Start/Stop Cycle
    3 cycles → All succeed, no state corruption, 3 scans in catalog
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Get initial catalog count
        initial_catalog = await async_client.get("/catalog")
        initial_count = len(initial_catalog.json()["items"])

        # Perform 3 start/stop cycles
        for cycle in range(3):
            # Start mapping
            mock_status.return_value = "UNKNOWN"
            start_response = await async_client.put("/mappingApp/start")
            assert start_response.status_code == 200, f"Cycle {cycle}: Start failed"

            # Transition to RUNNING
            mock_status.return_value = "TRACKING"
            await asyncio.sleep(1.5)

            mock_status.return_value = "STABILIZED"
            await asyncio.sleep(1.5)

            # Verify running state
            state = await async_client.get("/mappingApp/state")
            assert state.json()["state"] == "running", f"Cycle {cycle}: Not running"

            # Stop mapping
            stop_response = await async_client.put("/mappingApp/stop")
            assert stop_response.status_code == 200, f"Cycle {cycle}: Stop failed"

            # Verify idle state
            state = await async_client.get("/mappingApp/state")
            assert state.json()["state"] == "idle", f"Cycle {cycle}: Not idle after stop"

        # Verify 3 new scans added to catalog
        final_catalog = await async_client.get("/catalog")
        final_count = len(final_catalog.json()["items"])
        assert final_count == initial_count + 3, \
            f"Expected {initial_count + 3} items, got {final_count}"


# ==================== 2.4 Mapping Settings (4 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_mapping_settings(async_client: AsyncClient):
    """
    Test 22: Get Mapping Settings
    GET /mappingApp/settings → Returns DEFAULT_MAPPING_SETTINGS structure
    """
    with patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Get settings
        response = await async_client.get("/mappingApp/settings")

        # Expects: 200 OK with correct structure
        assert response.status_code == 200
        settings = response.json()

        # Verify structure matches DEFAULT_MAPPING_SETTINGS
        assert "preview_voxel_size" in settings
        assert "file_format" in settings
        assert "map_quality" in settings

        # Verify each setting has options and current_selection
        for key in ["preview_voxel_size", "file_format", "map_quality"]:
            assert "options" in settings[key]
            assert "current_selection" in settings[key]
            assert isinstance(settings[key]["options"], list)
            assert isinstance(settings[key]["current_selection"], int)


@pytest.mark.asyncio
@pytest.mark.integration
async def test_update_mapping_settings_idle(async_client: AsyncClient):
    """
    Test 23: Update Mapping Settings (Idle)
    PUT /mappingApp/settings when idle → 200 OK, persisted
    """
    with patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Get current settings
        get_response = await async_client.get("/mappingApp/settings")
        current_settings = get_response.json()

        # Update settings
        new_settings = current_settings.copy()
        new_settings["preview_voxel_size"]["current_selection"] = 0
        new_settings["file_format"]["current_selection"] = 0

        update_response = await async_client.put(
            "/mappingApp/settings",
            json={"settings": new_settings}
        )

        # Expects: 200 OK
        assert update_response.status_code == 200
        updated_settings = update_response.json()

        # Verify settings were updated
        assert updated_settings["preview_voxel_size"]["current_selection"] == 0
        assert updated_settings["file_format"]["current_selection"] == 0

        # Verify settings persist (get again)
        verify_response = await async_client.get("/mappingApp/settings")
        persisted_settings = verify_response.json()
        assert persisted_settings["preview_voxel_size"]["current_selection"] == 0
        assert persisted_settings["file_format"]["current_selection"] == 0


@pytest.mark.asyncio
@pytest.mark.integration
async def test_update_settings_during_mapping(async_client: AsyncClient):
    """
    Test 24: Update Settings During Mapping
    Update while running → Settings change after stop
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Get initial settings
        initial_settings = await async_client.get("/mappingApp/settings")
        initial_voxel_selection = initial_settings.json()["preview_voxel_size"]["current_selection"]

        # Start and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Get current settings (active settings during mapping)
        running_settings = await async_client.get("/mappingApp/settings")
        running_voxel = running_settings.json()["preview_voxel_size"]["current_selection"]

        # Update settings while running
        new_settings = running_settings.json().copy()
        new_voxel_selection = (initial_voxel_selection + 1) % 3
        new_settings["preview_voxel_size"]["current_selection"] = new_voxel_selection

        update_response = await async_client.put(
            "/mappingApp/settings",
            json={"settings": new_settings}
        )
        assert update_response.status_code == 200

        # Settings should remain unchanged before stop (active settings)
        before_stop = await async_client.get("/mappingApp/settings")
        assert before_stop.json()["preview_voxel_size"]["current_selection"] == running_voxel

        # Stop mapping
        await async_client.put("/mappingApp/stop")

        # Settings should be updated after stop
        after_stop = await async_client.get("/mappingApp/settings")
        assert after_stop.json()["preview_voxel_size"]["current_selection"] == new_voxel_selection


@pytest.mark.asyncio
@pytest.mark.integration
async def test_invalid_setting(async_client: AsyncClient):
    """
    Test 25: Invalid Setting (MAP-SET-004)
    PUT /mappingApp/settings with invalid file_format and valid preview_voxel_size
    → 400 Bad Request, invalid values ignored, valid value updated
    """
    with patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Get current settings to compare after
        get_response = await async_client.get("/mappingApp/settings")
        initial_settings = get_response.json()
        initial_voxel_selection = initial_settings["preview_voxel_size"]["current_selection"]

        # Send flat format with invalid file_format and valid preview_voxel_size
        # "OBJ" is not in the valid options ["PLY", "PCD", "LAS", "LAZ"]
        # 5 is a valid preview_voxel_size (in options [5, 10, 15])
        invalid_request = {
            "file_format": "OBJ",       # Invalid: not in options
            "preview_voxel_size": 5     # Valid: in options [5, 10, 15]
        }

        # Try to update with mixed valid/invalid settings
        update_response = await async_client.put(
            "/mappingApp/settings",
            json=invalid_request
        )

        # Expects: 400 Bad Request indicating invalid values
        assert update_response.status_code == 400

        # Verify invalid value was ignored but valid value was updated
        verify_response = await async_client.get("/mappingApp/settings")
        settings = verify_response.json()

        # file_format should remain unchanged (invalid "OBJ" was ignored)
        assert settings["file_format"]["current_selection"] == initial_settings["file_format"]["current_selection"]

        # preview_voxel_size should be updated to index 0 (value 5)
        # Options are [5, 10, 15], so value 5 = index 0
        assert settings["preview_voxel_size"]["current_selection"] == 0


# ==================== 2.5 Scan Name Prefix (2 tests) ====================


@pytest.mark.asyncio
@pytest.mark.integration
async def test_start_mapping_with_name_prefix(async_client: AsyncClient):
    """
    Test 26: Start Mapping with Name Prefix
    PUT /mappingApp/start with {"name_prefix": "MyProject"} → 200 OK
    Catalog item should be named "MyProject_1"
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Start mapping with name prefix
        mock_status.return_value = "UNKNOWN"
        response = await async_client.put(
            "/mappingApp/start",
            json={"name_prefix": "TestProject"}
        )

        # Expects: 200 OK
        assert response.status_code == 200
        data = response.json()
        assert data["state"] == "starting"

        # Transition to RUNNING
        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Stop mapping to trigger catalog add
        stop_response = await async_client.put("/mappingApp/stop")
        assert stop_response.status_code == 200

        # Verify catalog item was created with prefix
        catalog_response = await async_client.get("/catalog")
        assert catalog_response.status_code == 200
        catalog_data = catalog_response.json()

        # Find the item with prefix "TestProject_"
        items = catalog_data.get("items", {})
        prefixed_items = [
            item for item in items.values()
            if item["name"].startswith("TestProject_")
        ]
        assert len(prefixed_items) == 1
        assert prefixed_items[0]["name"] == "TestProject_1"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_start_mapping_incremental_naming(async_client: AsyncClient):
    """
    Test 27: Incremental Naming with Same Prefix
    Multiple scans with same prefix → "Scan_1", "Scan_2", etc.
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.StopEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status, \
         patch('mapping_app.do_finalize'):

        # Get initial catalog count
        initial_catalog = await async_client.get("/catalog")
        initial_items = initial_catalog.json().get("items", {})

        # Perform 2 mapping cycles with same prefix
        for i in range(2):
            # Start mapping with prefix
            mock_status.return_value = "UNKNOWN"
            start_response = await async_client.put(
                "/mappingApp/start",
                json={"name_prefix": "IncrementalTest"}
            )
            assert start_response.status_code == 200

            # Transition to RUNNING
            mock_status.return_value = "TRACKING"
            await asyncio.sleep(1.5)

            mock_status.return_value = "STABILIZED"
            await asyncio.sleep(1.5)

            # Stop mapping
            stop_response = await async_client.put("/mappingApp/stop")
            assert stop_response.status_code == 200

        # Verify catalog has items with incremental names
        final_catalog = await async_client.get("/catalog")
        items = final_catalog.json().get("items", {})

        # Find items with prefix
        prefixed_items = [
            item["name"] for item in items.values()
            if item["name"].startswith("IncrementalTest_")
        ]

        assert "IncrementalTest_1" in prefixed_items
        assert "IncrementalTest_2" in prefixed_items
