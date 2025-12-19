# DimenvuePro Server - Integration & EE Test Specifications

## Overview
This document specifies integration tests and end-to-end tests for the DimenvuePro spatial scanner server. Each test follows the **Given-Action-Expects** format.

**Scope:** Integration tests verify API endpoints and service layer interactions. EE tests validate complete workflows from user action to final result.

---

## 1. Camera API Integration Tests

### 1.1 Camera Initialization

#### Test: Camera Init Success
- **Test ID:** `CAM-INIT-001`
- **Implementation:** [test_camera_init_success](api/test_camera_api.py#L21)
- **Given:** Camera app is not initialized
- **Action:** `PUT /cameraApp/init`
- **Expects:**
  - Status: 200 OK
  - Response contains initialization confirmation

  - GStreamer pipelines are running
  - Janus WebRTC server is started
  - Preview stream is available

#### Test: Camera Init Idempotency
- **Test ID:** `CAM-INIT-002`
- **Implementation:** [test_camera_init_idempotency](api/test_camera_api.py#L37)
- **Given:** Camera app is already initialized
- **Action:** `PUT /cameraApp/init`
- **Expects:**
  - Status: 200 OK
  - No duplicate processes spawned
  - Existing pipelines remain functional

#### Test: Camera Deinit
- **Test ID:** `CAM-INIT-003`
- **Implementation:** [test_camera_deinit](api/test_camera_api.py#L52)
- **Given:** Camera app is initialized
- **Action:** `PUT /cameraApp/deinit`
- **Expects:**
  - Status: 200 OK
  - GStreamer pipelines stopped
  - Janus processes terminated
  - Resources cleaned up

#### Test: Deinit Idempotency
- **Test ID:** `CAM-INIT-004`
- **Implementation:** [test_deinit_idempotency](api/test_camera_api.py#L66)
- **Given:** Camera app is already deinitialized (idle state)
- **Action:** `PUT /cameraApp/deinit`
- **Expects:**
  - Status: 200 OK
  - Response: `{"status": "already_deinitialized", "state": "idle"}`
  - No errors or side effects

#### Test: Get Camera State
- **Test ID:** `CAM-INIT-005`
- **Implementation:** [test_get_camera_state](api/test_camera_api.py#L82)
- **Given:** Camera is in various states (idle, ready, initializing, error)
- **Action:** `GET /cameraApp/state`
- **Expects:**
  - Status: 200 OK
  - Response contains:
    - `state`: Current state value ("idle", "ready", "initializing", "deinitializing", "error")
    - `initialized`: Boolean flag
    - `error`: Error message (null if no error)

#### Test: Concurrent Init Requests
- **Test ID:** `CAM-INIT-006`
- **Implementation:** [test_concurrent_init_requests](api/test_camera_api.py#L93)
- **Given:** Camera is idle
- **Action:** Send 3 concurrent `PUT /cameraApp/init` requests
- **Expects:**
  - First request: 200 OK, `{"status": "initialized", "state": "ready"}`
  - Other requests: Either
    - 409 Conflict with "Cannot init: camera is initializing"
    - OR 200 OK with "already_initialized" (if they arrive after init completes)
  - Only one initialization actually runs
  - Final state: `GET /cameraApp/state` returns `"state": "ready"`
  - No duplicate GStreamer pipelines spawned

#### Test: Init During Deinit
- **Test ID:** `CAM-INIT-007`
- **Implementation:** [test_init_during_deinit](api/test_camera_api.py#L115)
- **Given:**
  - Camera is initialized (ready state)
  - Deinit operation is in progress
- **Action:** `PUT /cameraApp/init` while deinit is running
- **Expects:**
  - Status: 409 Conflict
  - Error message: "Cannot init: camera is deinitializing"
  - Deinit completes successfully
  - State transitions: ready → deinitializing → idle (unaffected by init attempt)

#### Test: Deinit During Init
- **Test ID:** `CAM-INIT-008`
- **Implementation:** [test_deinit_during_init](api/test_camera_api.py#L145)
- **Given:**
  - Camera is idle
  - Init operation is in progress
- **Action:** `PUT /cameraApp/deinit` while init is running
- **Expects:**
  - Status: 409 Conflict
  - Error message: "Cannot deinit: camera is initializing"
  - Init completes successfully
  - State transitions: idle → initializing → ready (unaffected by deinit attempt)

#### Test: Multiple Concurrent Deinit Requests
- **Test ID:** `CAM-INIT-009`
- **Implementation:** [test_multiple_concurrent_deinit_requests](api/test_camera_api.py#L172)
- **Given:** Camera is ready (initialized)
- **Action:** Send 3 concurrent `PUT /cameraApp/deinit` requests
- **Expects:**
  - First request: 200 OK, `{"status": "deinitialized", "state": "idle"}`
  - Other requests: Either
    - 409 Conflict with "Cannot deinit: camera is deinitializing"
    - OR 200 OK with "already_deinitialized" (if they arrive after deinit completes)
  - Only one deinitialization actually runs
  - Final state: idle
  - All pipelines properly cleaned up

#### Test: Get State During Init
- **Test ID:** `CAM-INIT-010`
- **Implementation:** [test_get_state_during_init](api/test_camera_api.py#L197)
- **Given:** Init in progress (between request start and completion)
- **Action:** `GET /cameraApp/state`
- **Expects:**
  - Status: 200 OK
  - Response: `{"state": "initializing", "initialized": false, "error": null}`

#### Test: Get State During Deinit
- **Test ID:** `CAM-INIT-011`
- **Implementation:** [test_get_state_during_deinit](api/test_camera_api.py#L224)
- **Given:** Deinit in progress
- **Action:** `GET /cameraApp/state`
- **Expects:**
  - Status: 200 OK
  - Response: `{"state": "deinitializing", "initialized": true, "error": null}`

#### Test: Rapid Init/Deinit Cycle
- **Test ID:** `CAM-INIT-012`
- **Implementation:** [test_rapid_init_deinit_cycle](api/test_camera_api.py#L254)
- **Given:** Camera is idle
- **Action:**
  1. `PUT /cameraApp/init` (wait for completion)
  2. `PUT /cameraApp/deinit` (wait for completion)
  3. Repeat steps 1-2 five times rapidly
- **Expects:**
  - All 10 operations (5 init, 5 deinit) succeed with 200 OK
  - No state corruption
  - No resource leaks (check with `ps`, `lsof`)
  - Final state: idle
  - GStreamer pipelines fully cleaned up after each cycle

---

### 1.2 Camera Capture

#### Test: Single Frame Capture
- **Test ID:** `CAM-CAP-001`
- **Implementation:** [test_single_frame_capture](api/test_camera_api.py#L279)
- **Given:**
  - Camera app is initialized
  - Output directory exists
- **Action:** `PUT /cameraApp/capture`
- **Expects:**
  - Status: 200 OK
  - 3 image files created (left, front, right cameras)
  - Files are valid JPG/PNG format
  - Catalog contains new `Photo` entry
  - Entry has correct UUID, timestamp, and file paths

#### Test: Capture Without Init
- **Test ID:** `CAM-CAP-002`
- **Implementation:** [test_capture_without_init](api/test_camera_api.py#L306)
- **Given:** Camera app is NOT initialized
- **Action:** `PUT /cameraApp/capture`
- **Expects:**
  - Status: 400 Bad Request
  - Error message indicates camera not initialized
  - No files created

#### Test: Multiple Sequential Captures (back to back, when previous FINISH)
- **Test ID:** `CAM-CAP-003`
- **Implementation:** [test_multiple_sequential_captures](api/test_camera_api.py#L317)
- **Given:** Camera app is initialized
- **Action:**
  1. `PUT /cameraApp/capture`, then
  2. `PUT /cameraApp/capture`, then
  3. `PUT /cameraApp/capture`
- **Expects:**
  - All 3 captures succeed with 200 OK
  - 9 total image files created (3 per capture)
  - 3 separate catalog entries with unique UUIDs
  - No file name collisions

---

### 1.3 Video Recording

#### Test: Start Recording
- **Test ID:** `CAM-REC-001`
- **Implementation:** [test_start_recording](api/test_camera_api.py#L354)
- **Given:**
  - Camera app is initialized
  - No recording in progress
- **Action:** `PUT /cameraApp/recordStart`
- **Expects:**
  - Status: 200 OK
  - Recording state set to active
  - GStreamer recording pipelines start writing to disk

#### Test: Stop Recording
- **Test ID:** `CAM-REC-002`
- **Implementation:** [test_stop_recording](api/test_camera_api.py#L374)
- **Given:** Recording is in progress for at least 3 seconds
- **Action:** `PUT /cameraApp/recordStop`
- **Expects:**
  - Status: 200 OK
  - 3 video files created (left, front, right)
  - Files are valid MP4/AVI format
  - Catalog contains new `Video` entry
  - Video duration matches recording time (±1 second tolerance)

#### Test: Record Start Without Init
- **Test ID:** `CAM-REC-003`
- **Implementation:** [test_record_start_without_init](api/test_camera_api.py#L411)
- **Given:** Camera app is NOT initialized
- **Action:** `PUT /cameraApp/recordStart`
- **Expects:**
  - Status: 400 Bad Request
  - No recording starts
  - No files created

#### Test: Duplicate Record Start
- **Test ID:** `CAM-REC-004`
- **Implementation:** [test_duplicate_record_start](api/test_camera_api.py#L422)
- **Given:** Recording is already in progress
- **Action:** `PUT /cameraApp/recordStart`
- **Expects:**
  - Status: 409 Conflict
  - Existing recording continues unaffected
  - No duplicate recording started

#### Test: Record Stop Without Start
- **Test ID:** `CAM-REC-005`
- **Implementation:** [test_record_stop_without_start](api/test_camera_api.py#L441)
- **Given:** No recording in progress
- **Action:** `PUT /cameraApp/recordStop`
- **Expects:**
  - Status: 400 Bad Request
  - Error message indicates no recording in progress

---

### 1.4 Preview Switching

#### Test: Switch Preview Index
- **Test ID:** `CAM-PREV-001`
- **Implementation:** [test_switch_preview_index](api/test_camera_api.py#L456)
- **Given:**
  - Camera app is initialized
  - Current preview index is 0 (left camera)
- **Action:** `PUT /cameraApp/preview-index` with body `{"index": 1}`
- **Expects:**
  - Status: 200 OK
  - Preview switches to front camera (index 1)

#### Test: Get Preview Index
- **Test ID:** `CAM-PREV-002`
- **Implementation:** [test_get_preview_index](api/test_camera_api.py#L476)
- **Given:**
  - Camera app is initialized
  - Current preview index is 0 (left camera)
- **Action:** `GET /cameraApp/preview-index`
- **Expects:**
  - Status: 200 OK
  - return `{"index": 0}`

#### Test: Invalid Preview Index
- **Test ID:** `CAM-PREV-003`
- **Implementation:** [test_invalid_preview_index](api/test_camera_api.py#L491)
- **Given:** Camera app is initialized
- **Action:** `PUT /cameraApp/preview-index` with body `{"index": 99}`
- **Expects:**
  - Status: 400 Bad Request
  - Error message indicates invalid index
  - Current preview unchanged

---

### 1.5 Camera Settings

#### Test: Get Camera Settings
- **Test ID:** `CAM-SET-001`
- **Implementation:** [test_get_camera_settings](api/test_camera_api.py#L520)
- **Given:** Server is running
- **Action:** `GET /cameraApp/settings`
- **Expects:**
  - Status: 200 OK
  - Response contains: resolution, framerate, capture_format, capture_quality, record_format, record_bitrate, preview_quality
  - The current settings must have the same structure as the DEFAULT_CAMERA_SETTINGS, except the "current_selection" values

#### Test: Update Camera Settings
- **Test ID:** `CAM-SET-002`
- **Implementation:** [test_update_camera_settings](api/test_camera_api.py#L552)
- **Given:** Camera app is NOT recording/capturing
- **Action:** `PUT /cameraApp/settings` with body:
  ```json
  {
    "resolution": "1920x1080",
    "framerate": 30
  }
  ```
- **Expects:**
  - Status: 200 OK
  - New settings dict has the "current_selection" updated for the requested keys.
  - The updated settings must have the same structure as the DEFAULT_CAMERA_SETTINGS, except the "current_selection" values
  - setting persisted to disk

#### Test: Update Camera Settings With Invalid keys
- **Test ID:** `CAM-SET-003`
- **Implementation:** [test_update_camera_settings_invalid_keys](api/test_camera_api.py#L596)
- **Given:** Camera app is NOT recording/capturing
- **Action:** `PUT /cameraApp/settings` with body:
  ```json
  {
    "wrong_key": "value",
    "frame_rate": 30
  }
  ```
  The invalid value is either out of range or not in options list.
- **Expects:**
  - Status: 400 Bad request
  - The invalid keys are ignored, the valid keys are updated.

---

## 2. Mapping API Integration Tests

**Note:** The mapping service uses ROS2 LaunchService via Python API to manage all mapping nodes within the container. State transitions are managed by an async state machine with automatic IMU stabilization monitoring.

**Architecture:**
- **ROS2 Launch System:** Uses `launch.LaunchService` to orchestrate all nodes from `ros2/toplevel.launch.py`
- **Nodes Launched:**
  - `fast_lio/fastlio_mapping` - SLAM/odometry
  - `python3 imu_monitor.py` - IMU stabilization tracker
  - `point_cloud_bridge/bridge_node` - Real-time streaming
  - `point_cloud_bridge/recorder_node` - Data recording
  - `ros2 bag record` - Raw sensor data
  - **Development mode:** `ros2 bag play` from `/shared_data/office_sim_bag/`
  - **Production mode:** `ros2_ouster/ouster_driver` for live sensor
- **State Management:** Async lock-protected state machine with automatic polling

**Lifecycle:**
```
IDLE → STARTING → INITIALIZING → RUNNING → STOPPING → IDLE
      (start)   (nodes launch) (IMU stable) (stop)
```

### 2.1 Mapping Lifecycle

#### Test: Start Mapping Success
- **Test ID:** `MAP-LIFE-001`
- **Implementation:** [test_start_mapping_success](api/test_mapping_api.py#L26)
- **Given:**
  - No mapping running (state: idle)
  - ROS2 is available (launch module can be imported)
  - ROS2 workspace exists at `/ros2_ws` with fast_lio and point_cloud_bridge installed
  - Artifact directory is writable
- **Action:** `PUT /mappingApp/start`
- **Expects:**
  - Status: 200 OK
  - Response: `{"details": "started", "state": "starting"}` (or similar structure)
  - State immediately transitions to STARTING
  - All nodes started
  - Artifact directory created: `{data_path}/mapping_artifact/`
  - IMU status file created: `{artifact_dir}/imu_stabilization_status.txt`

#### Test: Verify Node Launch After Start
- **Test ID:** `MAP-LIFE-002`
- **Implementation:** [test_verify_node_launch_after_start](api/test_mapping_api.py#L49)
- **Given:** Mapping just started (state: starting)
- **Action:**
  1. Wait 2-3 seconds for nodes to initialize
  2. Check running ROS2 nodes: `ros2 node list`
- **Expects:**
  - Nodes visible in ROS2 graph:
    - `/fastlio_mapping`
    - `/imu_monitor`
    - `/bridge_node`
    - `/recorder_node`

#### Test: Automatic State Transition STARTING INITIALIZING
- **Test ID:** `MAP-LIFE-003`
- **Implementation:** [test_automatic_transition_starting_to_initializing](api/test_mapping_api.py#L74)
- **Given:**
  - Mapping started successfully (state: starting)
- **Action:**
  1. Poll `GET /mappingApp/state` every 0.5 seconds
  2. Check the IMU status file content
- **Expects:**
  - IMU status file contains "TRACKING"
  - State transitions from STARTING → INITIALIZING automatically

#### Test: Automatic State Transition INITIALIZING RUNNING
- **Test ID:** `MAP-LIFE-004`
- **Implementation:** [test_automatic_transition_initializing_to_running](api/test_mapping_api.py#L105)
- **Given:**
  - Mapping is initializing (state: initializing)
  - IMU monitor is tracking stability
- **Action:**
  1. Poll `GET /mappingApp/state`
  2. Check the IMU status file content
  3. timeout 20secs
- **Expects:**
  - status file read "STABILIZED"
  - `GET /mappingApp/state` returns: `{"state": "running"}`

#### Test: Duplicate Mapping Start (Idempotency - Running)
- **Test ID:** `MAP-LIFE-005`
- **Implementation:** [test_duplicate_mapping_start_running](api/test_mapping_api.py#L137)
- **Given:** Mapping process is already running (state: running)
- **Action:** `PUT /mappingApp/start`
- **Expects:**
  - Status: 200 OK
  - Response: `{"details": "already_running", "state": "running"}`

#### Test: Duplicate Mapping Start (Idempotency - Initializing)
- **Test ID:** `MAP-LIFE-006`
- **Implementation:** [test_duplicate_mapping_start_initializing](api/test_mapping_api.py#L167)
- **Given:** Mapping is in INITIALIZING state (waiting for IMU)
- **Action:** `PUT /mappingApp/start`
- **Expects:**
  - Status: 200 OK
  - Response: `{"details": "already_running", "state": "initializing"}`

#### Test: Start During Stopping
- **Test ID:** `MAP-LIFE-007`
- **Implementation:** [test_start_during_stopping](api/test_mapping_api.py#L194)
- **Given:** Mapping is being stopped (state: stopping)
- **Action:** `PUT /mappingApp/start` while stop is in progress
- **Expects:**
  - Status: 409 conflict
  - RuntimeError raised with message: "Cannot start: mapping is stopping"
  - Stop operation completes unaffected
  - Final state after stop completes: IDLE

#### Test: Start During Starting (Race Condition)
- **Test ID:** `MAP-LIFE-008`
- **Implementation:** [test_start_during_starting_race](api/test_mapping_api.py#L237)
- **Given:** Mapping is during startup (state: starting)
- **Action:** Second `PUT /mappingApp/start` arrives before first completes
- **Expects:**
  - First request: 200 OK, state → starting
  - Second request: 409 conflict error with "Cannot start: mapping is starting"
  - Only one LaunchService created
  - Final state: initializing (first start succeeds)

#### Test: Stop Mapping from RUNNING State
- **Test ID:** `MAP-LIFE-009`
- **Implementation:** [test_stop_mapping_from_running](api/test_mapping_api.py#L271)
- **Given:** Mapping has been running for at least 15 seconds (IMU stabilized)
- **Action:** `PUT /mappingApp/stop`
- **Expects:**
  - Status: 200 OK
  - Response: `{"details": "stopped", "state": "idle"}` (or similar)
  - State transitions: RUNNING → STOPPING → IDLE
  - Finalization script executes: `finalize_mapping.py`
  - Point cloud file created in artifact_dir (format based on settings)
  - Catalog updated with new `Scan` entry
  - Entry includes:
    - UUID
    - Name (auto-generated or timestamped)
    - File paths (point cloud, thumbnail if generated)
    - Metadata (point count, bounding box, etc.)

#### Test: Stop Mapping During INITIALIZING
- **Test ID:** `MAP-LIFE-010`
- **Implementation:** [test_stop_mapping_during_initializing](api/test_mapping_api.py#L311)
- **Given:** Mapping is in INITIALIZING state
- **Action:** `PUT /mappingApp/stop`
- **Expects:**
  - Status: 409 conflict
  - initialization continues

#### Test: Stop Mapping During STARTING
- **Test ID:** `MAP-LIFE-011`
- **Implementation:** [test_stop_mapping_during_starting](api/test_mapping_api.py#L336)
- **Given:** Mapping just started (state: starting, nodes still launching)
- **Action:** `PUT /mappingApp/stop` (within 1-2 seconds of start)
- **Expects:**
  - Status: 409 conflict
  - startup continues

#### Test: Stop Mapping during STOPPING
- **Test ID:** `MAP-LIFE-012`
- **Implementation:** [test_stop_mapping_during_stopping](api/test_mapping_api.py#L367)
- **Given:** Mapping is running
- **Action:** Send `PUT /mappingApp/stop` twice in rapid succession
- **Expects:**
  - First request: 200 OK, state → stopping
  - Second request: 409 Conflict
  - Only one shutdown sequence executes
  - Final state: idle

#### Test: Stop Mapping Idempotency
- **Test ID:** `MAP-LIFE-013`
- **Implementation:** [test_stop_mapping_idempotency](api/test_mapping_api.py#L408)
- **Given:** No mapping process running (state: idle)
- **Action:** `PUT /mappingApp/stop`
- **Expects:**
  - Status: 200 OK
  - Response: `{"details": "already_stopped", "state": "idle"}`
  - No errors or side effects
  - No shutdown operations performed

---

### 2.2 Mapping State

#### Test: Get Mapping State - IDLE
- **Test ID:** `MAP-STATE-001`
- **Implementation:** [test_get_mapping_state_idle](api/test_mapping_api.py#L430)
- **Given:** No mapping process running
- **Action:** `GET /mappingApp/state`
- **Expects:**
  - Status: 200 OK
  - Response: `{"state": "idle"}`

#### Test: Get Mapping State - STARTING
- **Test ID:** `MAP-STATE-002`
- **Implementation:** [test_get_mapping_state_starting](api/test_mapping_api.py#L448)
- **Given:** Mapping just started, LaunchService launching nodes
- **Action:** `GET /mappingApp/state` immediately after start
- **Expects:**
  - Status: 200 OK
  - Response: `{"state": "starting"}`

#### Test: Get Mapping State - INITIALIZING
- **Test ID:** `MAP-STATE-003`
- **Implementation:** [test_get_mapping_state_initializing](api/test_mapping_api.py#L478)
- **Given:** Nodes launched, IMU not yet stabilized
- **Action:** `GET /mappingApp/state`
- **Expects:**
  - Status: 200 OK
  - Response: `{"state": "initializing"}`

#### Test: Get Mapping State - RUNNING
- **Test ID:** `MAP-STATE-004`
- **Implementation:** [test_get_mapping_state_running](api/test_mapping_api.py#L503)
- **Given:** IMU stabilized, actively mapping
- **Action:** `GET /mappingApp/state`
- **Expects:**
  - Status: 200 OK
  - Response: `{"state": "running"}`

#### Test: Get Mapping State - STOPPING
- **Test ID:** `MAP-STATE-005`
- **Implementation:** [test_get_mapping_state_stopping](api/test_mapping_api.py#L530)
- **Given:** Stop requested, shutdown in progress
- **Action:** `GET /mappingApp/state` during stop
- **Expects:**
  - Status: 200 OK
  - Response: `{"state": "stopping"}`
  - (Note: This state may be brief and hard to catch)

---

### 2.3 Mapping Concurrency & State Machine

#### Test: Concurrent Start Requests
- **Test ID:** `MAP-CONC-001`
- **Implementation:** [test_concurrent_start_requests](api/test_mapping_api.py#L575)
- **Given:** Mapping is idle
- **Action:** Send 3 concurrent `PUT /mappingApp/start` requests
- **Expects:**
  - First request 200 OK, after finish getState returns "initializing"
  - Second/third requests: 409 Conflict
  - Only one LaunchService created
  - Only one polling task started

#### Test: Concurrent Stop Requests
- **Test ID:** `MAP-CONC-002`
- **Implementation:** [test_concurrent_stop_requests](api/test_mapping_api.py#L612)
- **Given:**
  - Mapping state is RUNNING
- **Action:** Send 3 concurrent `PUT /mappingApp/stop` requests
- **Expects:**
  - First request 200 OK, after finish getState returns "idle"
  - Second/third requests: 409 conflict
  - Shutdown proceeds cleanly

#### Test: Rapid Start/Stop Cycle
- **Test ID:** `MAP-CONC-003`
- **Implementation:** [test_rapid_start_stop_cycle](api/test_mapping_api.py#L659)
- **Given:** Mapping is idle
- **Action:**
  1. `PUT /mappingApp/start` (wait for state → initializing or running)
  2. `PUT /mappingApp/stop` (wait for state → idle)
  3. Repeat steps 1-2 three times
- **Expects:**
  - All 6 operations succeed (200 OK)
  - No state corruption
  - No orphaned ROS2 nodes (verify with `ros2 node list` after each cycle)
  - No thread leaks (verify with process inspection)
  - Final state: idle
  - All LaunchServices properly shut down
  - 3 new scans added to catalog.

---

### 2.4 Mapping Settings

#### Test: Get Mapping Settings
- **Test ID:** `MAP-SET-001`
- **Implementation:** [test_get_mapping_settings](api/test_mapping_api.py#L711)
- **Given:** Server is running
- **Action:** `GET /mappingApp/settings`
- **Expects:**
  - Status: 200 OK
  - Response structure matches `DEFAULT_MAPPING_SETTINGS`:
    ```json
    {
      "preview_voxel_size": {
        "options": [5, 10, 15],
        "current_selection": 1
      },
      "file_format": {
        "options": ["PLY", "PCD", "LAS", "LAZ"],
        "current_selection": 1
      },
      "map_quality": {
        "options": ["Low", "Mid", "High"],
        "current_selection": 1
      }
    }
    ```

#### Test: Update Mapping Settings (Idle State)
- **Test ID:** `MAP-SET-002`
- **Implementation:** [test_update_mapping_settings_idle](api/test_mapping_api.py#L740)
- **Given:** No mapping in progress (state: idle)
- **Action:** `PUT /mappingApp/settings` with body:
  ```json
  {
    "preview_voxel_size": 5,
    "file_format": "PLY"
  }
  ```
- **Expects:**
  - Status: 200 OK
  - Settings persisted to `{data_path}/mapping_settings.json`
  - New settings dict has the "current_selection" updated for the requested keys.
  - The updated settings must have the same structure as the DEFAULT_MAPPING_SETTINGS, except the "current_selection" values

#### Test: Update Settings During Mapping
- **Test ID:** `MAP-SET-003`
- **Implementation:** [test_update_settings_during_mapping](api/test_mapping_api.py#L778)
- **Given:** Mapping is running (state: running)
- **Action:**
  - `PUT /mappingApp/settings` with new values
  - check settings before and after stop
- **Expects:**
  - Status: 200 OK
  - settings remain unchanged before stop
  - settings are updated after stop

#### Test: Invalid Setting
- **Test ID:** `MAP-SET-004`
- **Implementation:** [test_invalid_setting](api/test_mapping_api.py#L831)
- **Given:** Mapping is idle
- **Action:** `PUT /mappingApp/settings` with body:
  ```json
  {
    "file_format": "OBJ",
    "preview_voxel_size": 5
  }
  ```
- **Expects:**
  - Status: 400 Bad Request
  - invalid values ignored, valid value updated.

---

### 2.5 Scan Name Prefix

#### Test: Start Mapping with Name Prefix
- **Test ID:** `MAP-PREFIX-001`
- **Implementation:** [test_start_mapping_with_name_prefix](api/test_mapping_api.py#L879)
- **Given:** Mapping is idle, no items with prefix "TestProject" exist
- **Action:** `PUT /mappingApp/start` with body:
  ```json
  {
    "name_prefix": "TestProject"
  }
  ```
  Then complete the mapping cycle (wait for RUNNING, then stop)
- **Expects:**
  - Status: 200 OK
  - Mapping starts successfully
  - After stop, catalog contains item named "TestProject_1"
  - Item type is "Scan"

#### Test: Incremental Naming with Same Prefix
- **Test ID:** `MAP-PREFIX-002`
- **Implementation:** [test_start_mapping_incremental_naming](api/test_mapping_api.py#L930)
- **Given:** Mapping is idle
- **Action:**
  1. Start and stop mapping with `{"name_prefix": "IncrementalTest"}`
  2. Start and stop mapping again with same prefix
- **Expects:**
  - First scan named "IncrementalTest_1"
  - Second scan named "IncrementalTest_2"
  - Incremental numbering continues correctly

---

## 3. Catalog API Integration Tests

### 3.1 Catalog Retrieval

#### Test: Get Full Catalog
- **Test ID:** `CAT-RET-001`
- **Implementation:** [test_get_full_catalog](api/test_catalog_api.py#L24)
- **Given:** Catalog contains 5 items (2 images, 2 videos, 1 scan)
- **Action:** `GET /catalog`
- **Expects:**
  - Status: 200 OK
  - Response contains all 5 items
  - Each item has: uuid, name, type, created_at, file_paths
  - Storage stats show correct totals by category

#### Test: Get Empty Catalog
- **Test ID:** `CAT-RET-002`
- **Implementation:** [test_get_empty_catalog](api/test_catalog_api.py#L101)
- **Given:** Catalog is empty (no items)
- **Action:** `GET /catalog`
- **Expects:**
  - Status: 200 OK
  - Response: `{"items": [], "storage_stats": {...}}`

---

### 3.2 Item Metadata

#### Test: Get Image Metadata
- **Test ID:** `CAT-META-001`
- **Implementation:** [test_get_image_metadata](api/test_catalog_api.py#L134)
- **Given:** Catalog contains an image item with UUID `abc-123`
- **Action:** `GET /catalog/metadata/{file_path}`
- **Expects:**
  - Status: 200 OK
  - Response contains EXIF data: resolution, camera model, timestamp, GPS (if available)

#### Test: Get Metadata for Non-Existent File
- **Test ID:** `CAT-META-002`
- **Implementation:** [test_get_metadata_nonexistent_file](api/test_catalog_api.py#L164)
- **Given:** Catalog does not contain file at path
- **Action:** `GET /catalog/metadata/invalid/path.jpg`
- **Expects:**
  - Status: 404 Not Found

---

### 3.3 Item Deletion

#### Test: Delete Item by UUID
- **Test ID:** `CAT-DEL-001`
- **Implementation:** [test_delete_item_by_uuid](api/test_catalog_api.py#L183)
- **Given:** Catalog contains item with UUID `abc-123`
- **Action:** `PUT /catalog/abc-123/delete`
- **Expects:**
  - Status: 200 OK
  - Item removed from catalog JSON
  - Associated files deleted from disk
  - Storage stats updated

#### Test: Delete Non-Existent Item
- **Test ID:** `CAT-DEL-002`
- **Implementation:** [test_delete_nonexistent_item](api/test_catalog_api.py#L227)
- **Given:** Catalog does not contain UUID `invalid-uuid`
- **Action:** `PUT /catalog/invalid-uuid/delete`
- **Expects:**
  - Status: 404 Not Found

---

### 3.4 Item Renaming

#### Test: Rename Item
- **Test ID:** `CAT-REN-001`
- **Implementation:** [test_rename_item](api/test_catalog_api.py#L247)
- **Given:** Catalog contains item with UUID `abc-123` and name "Scan001"
- **Action:** `PUT /catalog/abc-123/rename` with body:
  ```json
  {
    "new_name": "Office_Floor_Plan"
  }
  ```
- **Expects:**
  - Status: 200 OK
  - Catalog entry updated with new name
  - folder name changed to the new name

#### Test: Rename with Duplicate Name
- **Test ID:** `CAT-REN-002`
- **Implementation:** [test_rename_with_duplicate_name](api/test_catalog_api.py#L296)
- **Given:**
  - Item `abc-123` named "Scan001"
  - Item `def-456` already named "Scan002"
- **Action:** `PUT /catalog/abc-123/rename` with `new_name: "Scan002"`
- **Expects:**
  - Status: 200 OK
  - item `abc-123` folder name change to "Scan002_1"

---

### 3.5 Item Export

#### Test: Export Item to USB
- **Test ID:** `CAT-EXP-001`
- **Implementation:** [test_export_item_to_usb](api/test_catalog_api.py#L360)
- **Given:**
  - Catalog contains item `abc-123`
  - USB device mounted at `/media/usb0`
- **Action:** `PUT /catalog/abc-123/export` with body:
  ```json
  {
    "mountpoint": "/media/usb0"
  }
  ```
- **Expects:**
  - Status: 200 OK
  - Export task added to queue
  - Response contains task ID

#### Test: Export Invalid Item
- **Test ID:** `CAT-EXP-002`
- **Implementation:** [test_export_invalid_item](api/test_catalog_api.py#L399)
- **Given:** Catalog doesn't contain item `abc-123`
- **Action:** `PUT /catalog/abc-123/export`
- **Expects:**
  - Status: 404 not found

---

### 3.6 Item Files

#### Test: Get Item Files
- **Test ID:** `CAT-FILE-001`
- **Implementation:** [test_get_item_files](api/test_catalog_api.py#L424)
- **Given:** Catalog contains item with UUID `abc-123` containing files (front.jpg, left.jpg, right.jpg, thumbnail.jpg)
- **Action:** `GET /catalog/abc-123/files`
- **Expects:**
  - Status: 200 OK
  - Response: `{"files": ["front.jpg", "left.jpg", "right.jpg", "thumbnail.jpg"]}`
  - Files list is sorted alphabetically

#### Test: Get Files for Non-Existent Item
- **Test ID:** `CAT-FILE-002`
- **Implementation:** [test_get_files_nonexistent_item](api/test_catalog_api.py#L461)
- **Given:** Catalog does not contain UUID `invalid-uuid`
- **Action:** `GET /catalog/invalid-uuid/files`
- **Expects:**
  - Status: 404 Not Found

---

## 4. Storage API Integration Tests

### 4.1 Removable Storage

#### Test: List USB Devices
- **Test ID:** `STO-REM-001`
- **Implementation:** [test_list_usb_devices_with_usb_connected](api/test_storage_api.py#L21)
- **Given:**
  - 1 USB device connected at `/media/usb0`
  - Device has 16GB total, 8GB free
- **Action:** `GET /storage/removable`
- **Expects:**
  - Status: 200 OK
  - Response contains device list, in the following format:
  ```json
  {
    mountpoint1: {
      "mountpoint": mountpoint1,
      "label": labal,
      "usage": {
        "size": total_size,
        "used": used_size,
        "free": free_size,
        "percent": used_percentage
      }
    },
    ...
  }
  ```


#### Test: No USB Devices
- **Test ID:** `STO-REM-002`
- **Implementation:** [test_list_usb_devices_with_no_usb](api/test_storage_api.py#L77)
- **Given:** No removable storage connected
- **Action:** `GET /storage/removable`
- **Expects:**
  - Status: 200 OK
  - Response: `{}`

---

### 4.2 Internal Storage

#### Test: Get Internal Storage Usage
- **Test ID:** `STO-INT-001`
- **Implementation:** [test_get_internal_storage_usage](api/test_storage_api.py#L96)
- **Given:** catalog with following stats:
```json
{
  ...,
  "storage_stats" : {
    "Photos": 500_000_000,
    "Videos": 1_000_000_000,
    "Scans": 2_000_000_000
  }
}
```
- **Action:** `GET /storage/internal/usage`
- **Expects:**
  - Status: 200 OK
  - stats summation matches catalog total filesystem size (within 1% error)

#### Test: Format Internal Storage
- **Test ID:** `STO-INT-002`
- **Implementation:** [test_format_internal_storage](api/test_storage_api.py#L159)
- **Given:** Internal storage has items
- **Action:** `PUT /storage/internal/format`
- **Expects:**
  - Status: 200 OK
  - All catalog items removed
  - All media files deleted
  - Catalog JSON cleared
  - Storage stats reset to zero

---

### 4.3 External Storage

#### Test: Format External Storage
- **Test ID:** `STO-EXT-001`
- **Implementation:** [test_format_external_storage](api/test_storage_api.py#L207)
- **Given:** USB device at `/media/usb0` with files
- **Action:** `PUT /storage/external/format` with body:
  ```json
  {
    "device": "/media/usb0"
  }
  ```
- **Expects:**
  - Status: 200 OK
  - all files deleted from the usb mounted filesystem

#### Test: Format Non-Existent Device
- **Test ID:** `STO-EXT-002`
- **Implementation:** [test_format_external_storage_non_existent_device](api/test_storage_api.py#L256)
- **Given:** No device at `/media/invalid`
- **Action:** `PUT /storage/external/format` with device `/media/invalid`
- **Expects:**
  - Status: 404 Not Found

---

## 5. Export System Integration Tests

### 5.1 Export Queue

#### Test: Get Export Progress - Empty Queue
- **Test ID:** `EXP-QUE-001`
- **Implementation:** [test_get_export_progress_empty_queue](api/test_export_api.py#L16)
- **Given:** No exports in progress
- **Action:** `GET /export/progress`
- **Expects:**
  - Status: 200 OK
  - Response: `{'total_tasks': 0,'completed_tasks': 0,'failed_tasks': 0,'queued_tasks': 0,'overall_progress': 0.0,'current_item': None,'is_complete': True}`

#### Test: Get Export Progress - Active
- **Test ID:** `EXP-QUE-002`
- **Implementation:** [test_get_export_progress_active](api/test_export_api.py#L53)
- **Given:**
  - 3 items in export queue
  - 1st item is 50% complete
- **Action:** `GET /export/progress`
- **Expects:**
  - Status: 200 OK
  - Response contains:
    - `overall_progress: 16-17` (50% of 1/3)
    - `current_item: {item_uuid, progress: 50}`
    - `queued_tasks: 2`

---

## 7. End-to-End Test Scenarios

### EE-1: Complete Photo Capture Workflow

#### Test: Capture, View, Export Photo
- **Test ID:** `EE-001`
- **Implementation:** [test_ee_complete_photo_capture_workflow](ee/test_ee.py#L175)
- **Given:**
  - Clean system state
  - USB device connected
- **Action:**
  1. `PUT /cameraApp/init`
  2. `PUT /cameraApp/capture`
  3. `GET /catalog` (verify item appears)
  4. `GET /catalog/metadata/{file_path}` (check EXIF)
  5. `PUT /catalog/{uuid}/export` with USB destination
  6. Poll `GET /export/progress` until complete
  7. Verify files on USB device
- **Expects:**
  - All steps return 200 OK
  - Photo appears in catalog within 2 seconds of capture
  - Export completes successfully
  - Files exist on USB with correct size/hash

---

### EE-2: Video Recording Workflow

#### Test: Record, Rename, Export Video
- **Test ID:** `EE-002`
- **Implementation:** [test_ee_video_recording_workflow](ee/test_ee.py#L279)
- **Given:**
  - Camera initialized
  - Internal storage has space
- **Action:**
  1. `PUT /cameraApp/init`
  2. `PUT /cameraApp/recordStart`
  3. Wait 5 seconds
  4. `PUT /cameraApp/recordStop`
  5. `GET /catalog` - get video UUID, verify video files in correct location
  6. `PUT /catalog/{uuid}/rename` to "TestVideo"
  7. `GET /catalog` - verify name changed, verify video files in correct location
  8. `PUT /catalog/{uuid}/export` to USB
  9. Poll `GET /export/progress` until complete
  10. Verify exported files on USB
- **Expects:**
  - Video ~5 seconds long
  - 3 video files created (left/front/right) in item directory
  - After rename: name reflects in catalog, files still in correct location
  - Export succeeds with files copied to USB

---

### EE-3: Spatial Mapping Workflow

#### Test: Map Environment, Save Scan, Export
- **Test ID:** `EE-003`
- **Implementation:** [test_ee_spatial_mapping_workflow](ee/test_ee.py#L426)
- **Given:**
  - ROS2 workspace available at `/ros2_ws` with `fast_lio` and `point_cloud_bridge` packages installed
  - Development mode enabled (rosbag data available at `/shared_data/office_sim_bag/`)
  - API server running
  - Mapping is idle
- **Action:**
  1. `PUT /mappingApp/start` and wait until return
  2. Poll `GET /mappingApp/state` until state is RUNNING
  3. Wait 30 seconds (simulate environment scanning - accumulate sensor data)
  4.  `PUT /mappingApp/stop`
  5. Poll state until IDLE
  6.  `GET /catalog` (verify scan entry added)
  7.  Verify scan item
  8.  `PUT /catalog/{uuid}/export` to USB
  9. Poll `GET /export/progress` until complete
- **Expects:**
  - Start succeeds: state transitions IDLE → STARTING
  - ROS2 LaunchService starts in background thread
  - All nodes launch successfully within 3 seconds:
    - `/fastlio_mapping`
    - `/imu_monitor`
    - `/bridge_node`
    - `/recorder_node`
    - Bag playback process (development mode)
  - State automatically transitions: STARTING → INITIALIZING (when IMU data arrives)
  - IMU stabilization takes ~10 seconds, status "TRACKING" → "STABILIZED"
  - State automatically transitions: INITIALIZING → RUNNING (when IMU stabilized)
  - Mapping runs for 30 seconds accumulating point cloud data
  - Stop succeeds: state transitions RUNNING → STOPPING
  - Finalization script success: metadata and thumbnail generated in artifact directory
  - Mapping ready: state transitions STOPPING -> IDLE
  - Scan appears in catalog with:
    - UUID
    - Timestamp
    - File paths (point cloud, metadata, sensor_raw)
    - Thumbnail (if generated)
  - Export to USB succeeds
  - No orphaned ROS2 nodes after stop (verify with `ros2 node list`)
  - No orphaned Python processes (verify with `ps aux | grep imu_monitor`)
  - Launch thread properly terminated

---

### EE-4: Storage Management Workflow

#### Test: Fill Storage, Delete Items, Format
- **Test ID:** `EE-004`
- **Implementation:** [test_ee_storage_management_workflow](ee/test_ee.py#L723)
- **Given:** Server is running
- **Action:**
  1. Capture 10 photos
  2. Record 3 videos
  3. `GET /storage/internal/usage` (check usage)
  4. `PUT /catalog/{uuid}/delete` for 5 items
  5. `GET /storage/internal/usage` (verify reduced)
  6. `PUT /storage/internal/format`
  7. `GET /catalog` (verify empty)
  8. `GET /storage/internal/usage` (verify zeroed)
- **Expects:**
  - Storage usage increases with each capture/record
  - Deletion reduces usage proportionally
  - Format clears all items and resets usage to near-zero

---

### EE-5: Multi-Export Queue Management

#### Test: Queue Multiple Exports, Monitor Progress
- **Test ID:** `EE-005`
- **Implementation:** [test_ee_multi_export_queue_management](ee/test_ee.py#L808)
- **Given:**
  - Catalog has 5 items
  - USB device connected
- **Action:**
  1. Export item 1 (large video, 2GB)
  2. Export item 2 (image, 5MB)
  3. Export item 3 (scan, 500MB)
  4. `GET /export/progress` (verify queue length = 3)
  5. Poll progress every 1 second until all complete
  6. `GET /export/results` when complete
- **Expects:**
  - Exports process sequentially (not parallel)
  - Overall progress increases monotonically 0→100%
  - All 3 exports succeed
  - Files on USB match source files (size/checksum)

---

### EE-6: Preview Switching During Recording

#### Test: Switch Preview While Recording
- **Test ID:** `EE-006`
- **Implementation:** [test_ee_preview_switching_during_recording](ee/test_ee.py#L899)
- **Given:** Camera initialized and recording
- **Action:**
  1. `PUT /cameraApp/recordStart`
  2. Wait 2 seconds
  3. `PUT /cameraApp/preview-index` (switch to index 1)
  4. Wait 2 seconds
  5. `PUT /cameraApp/preview-index` (switch to index 3 - composite)
  6. Wait 2 seconds
  7. `PUT /cameraApp/recordStop`
- **Expects:**
  - Recording continues uninterrupted during preview switches
  - Final video is 6+ seconds long
  - No dropped frames or corruption
  - All 3 camera files valid

---

## 8. Performance & Load Tests

### Load-1: Rapid Capture Sequence

#### Test: 50 Consecutive Captures
- **Test ID:** `LOAD-001`
- **Implementation:** [test_rapid_capture_sequence](test_performance.py#L26)
- **Given:** Camera initialized
- **Action:** Execute `PUT /cameraApp/capture` 50 times in rapid succession
- **Expects:**
  - All 50 captures succeed (or graceful rate limiting)
  - 150 image files created
  - 50 catalog entries
  - No memory leaks
  - Server remains responsive

---

### Load-2: Long Recording Session

#### Test: 1-Hour Video Recording
- **Test ID:** `LOAD-002`
- **Implementation:** [test_long_recording_session](test_performance.py#L131)
- **Given:**
  - Camera initialized
  - Sufficient storage (>50GB free)
- **Action:**
  1. `PUT /cameraApp/recordStart`
  2. Wait 3600 seconds
  3. `PUT /cameraApp/recordStop`
- **Expects:**
  - Recording completes successfully
  - Video files are ~1 hour duration
  - No file corruption
  - Memory usage stable throughout

---

### Load-3: Large Export Queue

#### Test: Queue 100 Items for Export
- **Test ID:** `LOAD-003`
- **Implementation:** [test_large_export_queue](test_performance.py#L252)
- **Given:** Catalog has 100 items (mix of photos/videos/scans)
- **Action:**
  1. Queue all 100 items for export
  2. Monitor progress
  3. Verify all complete
- **Expects:**
  - All 100 exports process sequentially
  - Progress reporting accurate
  - No queue corruption or deadlocks
  - All files transferred successfully

---

## 10. Test Environment Requirements

### Hardware/Software Prerequisites
- **Container Runtime:** Docker or Podman (for running the containerized server)
  - **GStreamer:** 1.22.9
  - **Janus WebRTC gateway**
  - **ROS2 Humble:** Installed in container with Fast-LIO workspace
  - **Environment Variables:**
    - `ROS2_WS_PATH`: Path to ROS2 workspace (default: `/ros2_ws`)
    - `SHARED_DATA_PATH`: Path for shared data (default: `/shared_data`)
    - `CAMERA_APP_OUTPUT_DIR`: Output directory for media/artifacts (default: `/dmv_data`)
  - **Test Storage:**
    - 50GB free for internal storage tests
    - USB device (16GB+) for export tests
  - **Cameras:** 3x CSI cameras or test mode enabled
  - **LiDAR:** Mock sensor or actual hardware, or use rosbag playback for mapping tests

### Mock/Test Mode Considerations
- Enable `MultiCamApp.TEST_MODE = True` for synthetic video (zone-plate patterns)
- Use rosbag files for simulated LiDAR/IMU data in mapping tests
- Use tmpfs for fast catalog tests
- Mock USB devices with pyudev fixtures
- For mapping tests without ROS2: mock the process start/stop and status file generation

### Test Data Setup
- Pre-generate sample catalog with known items
- Prepare large files (1GB+) for export performance tests
- Create point cloud samples for mapping validation

---

## 11. Test Execution Priorities

### Priority 1 (Critical Path)
- EE-1: Photo capture workflow
- EE-2: Video recording workflow
- EE-3: Spatial mapping workflow
- Camera init/deinit
- Catalog CRUD operations

### Priority 2 (Core Features)
- Export queue management
- Storage operations
- Settings persistence
- Preview switching

### Priority 3 (Robustness)
- Error recovery scenarios
- Concurrent operations
- Load tests
- Edge cases

---

## 12. Test Automation Notes

### Recommended Test Framework
- **pytest** with async support (pytest-asyncio)
- **httpx** or **TestClient** from FastAPI
- **pytest-docker** for container orchestration
- **pytest-timeout** for long-running tests

### Test Fixtures
```python
@pytest.fixture
def test_client():
    return TestClient(app)

@pytest.fixture
def temp_catalog(tmp_path):
    # Setup temp catalog path
    pass

@pytest.fixture
def mock_usb_device():
    # Mock pyudev device
    pass

@pytest.fixture(scope="session")
def ros2_available():
    """Check if ROS2 workspace is available for mapping tests"""
    ros2_ws = os.getenv("ROS2_WS_PATH", "/ros2_ws")
    return os.path.exists(ros2_ws) and os.path.exists(f"{ros2_ws}/install")

@pytest.fixture
def mock_mapping_process():
    """Mock ROS2 process for mapping tests without actual ROS2"""
    # Returns a mock Popen object for testing
    pass
```

### CI/CD Considerations
- Run Priority 1 tests on every commit
- Run Priority 2 tests on PR
- Run Priority 3 tests nightly
- Skip hardware-dependent tests in CI (tag with `@pytest.mark.hardware`)
- Skip ROS2-dependent mapping tests if workspace unavailable (tag with `@pytest.mark.ros2`)
- Use containerized test environment matching production container
- Mount test volumes for artifact directories and storage tests

---

## 9. Unit Tests

### 9.1 Mapping Finalization

#### Test: do_finalize with Valid PCD File
- **Test ID:** `UNIT-FIN-001`
- **Implementation:** [test_do_finalize_valid_pcd](units/test_finalize_mapping.py#L120)
- **Given:**
  - Temp artifact directory with:
    - `map_result.PCD` file containing valid point cloud data (random 1000 points)
    - `sensor_raw/` directory with mock rosbag metadata
- **Action:** Call `do_finalize(artifact_dir, "PCD")`
- **Expects:**
  - Returns without error
  - `thumbnail.png` generated in artifact_dir
  - `map_metadata.json` generated with valid structure containing:
    - `point_count`: number of points
    - `bounds`: min/max x,y,z
    - `created_at`: ISO timestamp
    - `file_format`: "PCD"

#### Test: do_finalize with Missing Map File
- **Test ID:** `UNIT-FIN-002`
- **Implementation:** [test_do_finalize_missing_map](units/test_finalize_mapping.py#L168)
- **Given:** Empty artifact directory (no map_result file)
- **Action:** Call `do_finalize(artifact_dir, "PCD")`
- **Expects:**
  - Raises appropriate error (FileNotFoundError or similar)
  - Does NOT call sys.exit()

#### Test: do_finalize PLY Format
- **Test ID:** `UNIT-FIN-003`
- **Implementation:** [test_do_finalize_ply_format](units/test_finalize_mapping.py#L202)
- **Given:** Artifact directory with `map_result.PLY` file (500 points)
- **Action:** Call `do_finalize(artifact_dir, "PLY")`
- **Expects:**
  - Returns without error
  - `map_metadata.json` generated with num_points=500

#### Test: do_finalize PCD Format
- **Test ID:** `UNIT-FIN-004`
- **Implementation:** [test_do_finalize_pcd_format](units/test_finalize_mapping.py#L218)
- **Given:** Artifact directory with `map_result.PCD` file (500 points)
- **Action:** Call `do_finalize(artifact_dir, "PCD")`
- **Expects:**
  - Returns without error
  - `map_metadata.json` generated with num_points=500

#### Test: load_pointcloud PCD File
- **Test ID:** `UNIT-FIN-005`
- **Implementation:** [test_load_pcd_file](units/test_finalize_mapping.py#L238)
- **Given:** Valid PCD file with 100 points
- **Action:** Call `load_pointcloud(filepath)`
- **Expects:**
  - Returns numpy array with shape (100, 3)
  - Points loaded correctly

#### Test: load_pointcloud Unsupported Format
- **Test ID:** `UNIT-FIN-006`
- **Implementation:** [test_load_unsupported_format](units/test_finalize_mapping.py#L253)
- **Given:** File with unsupported extension (.xyz)
- **Action:** Call `load_pointcloud(filepath)`
- **Expects:**
  - Raises `ValueError` with "Unsupported file format"

#### Test: calculate_area_volume Box
- **Test ID:** `UNIT-FIN-007`
- **Implementation:** [test_calculate_area_volume_box](units/test_finalize_mapping.py#L269)
- **Given:** Point cloud forming a 10x10x5 box
- **Action:** Call `calculate_area_volume(points)`
- **Expects:**
  - Area approximately 100 (10 * 10)
  - Volume approximately 500 (10 * 10 * 5)

#### Test: calculate_area_volume Empty
- **Test ID:** `UNIT-FIN-008`
- **Implementation:** [test_calculate_area_volume_empty](units/test_finalize_mapping.py#L287)
- **Given:** Empty point cloud (0 points)
- **Action:** Call `calculate_area_volume(points)`
- **Expects:**
  - Returns 0.0 or (0.0, 0.0)
  - No exception raised

---

### 9.2 ROS2 Lifecycle Management

#### Test: StartEverything Node Creation
- **Test ID:** `UNIT-ROS-001`
- **Implementation:** [test_start_everything_nodes](units/test_ros2_lifecycle.py#L126)
- **Given:** ROS2 environment available, no mapping nodes running
- **Action:** Call `await StartEverything(file_format="PCD", artifact_dir="/tmp/test")`
- **Expects:**
  - Function returns without error
  - ROS2 launch process started (subprocess running)
  - After 5s delay, `ros2 node list` contains expected nodes:
    - `/fastlio_mapping`
    - `/imu_monitor`
    - `/bridge_node`
    - `/recorder_node`
  - `ros2 topic list` contains expected topics:
    - `/ouster/imu`
    - `/ouster/points`
    - `/cloud_registered_body`
    - `/Odometry`

#### Test: StartEverything Creates Artifact Directory
- **Test ID:** `UNIT-ROS-002`
- **Implementation:** [test_start_everything_artifact_dir](units/test_ros2_lifecycle.py#L188)
- **Given:** artifact_dir does not exist
- **Action:** Call `await StartEverything(file_format="PCD", artifact_dir="/tmp/new_artifact_dir")`
- **Expects:**
  - artifact_dir created automatically
  - Directory has proper permissions

#### Test: StopEverything Process Shutdown
- **Test ID:** `UNIT-ROS-003`
- **Implementation:** [test_stop_everything_shutdown](units/test_ros2_lifecycle.py#L221)
- **Given:** StartEverything was called and nodes are running
- **Action:** Call `await StopEverything()`
- **Expects:**
  - Function returns without error
  - Launch subprocess terminated (SIGINT sent)
  - After 5s, `ros2 node list` returns empty or no mapping nodes
  - No orphaned processes (`pgrep -f fastlio` returns nothing)

#### Test: StopEverything Idempotency
- **Test ID:** `UNIT-ROS-004`
- **Implementation:** [test_stop_everything_idempotent](units/test_ros2_lifecycle.py#L280)
- **Given:** No mapping nodes running (already stopped or never started)
- **Action:** Call `await StopEverything()`
- **Expects:**
  - Returns without error
  - No exceptions raised

#### Test: Start/Stop Cycle
- **Test ID:** `UNIT-ROS-005`
- **Implementation:** [test_start_stop_cycle](units/test_ros2_lifecycle.py#L302)
- **Given:** ROS2 environment available
- **Action:** Start and stop mapping twice in sequence
- **Expects:**
  - Each cycle starts and stops without error
  - No orphaned processes after cycles complete
  - System can handle repeated start/stop operations

### 9.3 IMU Monitor

Tests for `ros2/imu_monitor.py` - IMU stabilization detection using moving standard deviation.

**Detection Method:** Moving standard deviation (mstd) over 1-second window
- Accelerometer mstd threshold: 0.3 m/s²
- Gyroscope mstd threshold: 0.05 rad/s
- Stabilization confirmation: Stable for `track_duration` seconds

#### Test: IMU Monitor Detects Stabilization
- **Test ID:** `UNIT-IMU-001`
- **Implementation:** [test_imu_monitor_detects_stabilization](units/test_imu_monitor.py#L42)
- **Given:**
  - ROS2 environment available
  - Test bag exists at `/shared_data/test_bag` with `/ouster/imu` topic
- **Action:**
  1. Start IMU monitor with `--imu-topic /ouster/imu --track-duration 3.0`
  2. Play test_bag at 2x speed
  3. Wait for status file to contain "STABILIZED"
- **Expects:**
  - IMU monitor starts successfully
  - Status file created with "TRACKING" initially
  - Stabilization detected within 30 seconds
  - Status file contains "STABILIZED"

#### Test: Status File Created on Start
- **Test ID:** `UNIT-IMU-003`
- **Implementation:** [test_status_file_created_on_start](units/test_imu_monitor.py#L140)
- **Given:**
  - ROS2 environment available
  - IMU monitor configured with custom topic `/test_imu`
- **Action:**
  1. Start IMU monitor with `--imu-topic /test_imu`
  2. Publish single IMU message to topic
  3. Check status file
- **Expects:**
  - Status file created after first message received
  - Initial status is "TRACKING"

#### Test: MSTD Calculation Basic
- **Test ID:** `UNIT-IMU-004`
- **Implementation:** [test_calculate_std_basic](units/test_imu_monitor.py#L200)
- **Given:** Buffer with values [1, 2, 3, 4, 5]
- **Action:** Calculate standard deviation
- **Expects:**
  - Standard deviation equals sqrt(2) ≈ 1.414
  - Mean equals 3, variance equals 2

#### Test: MSTD Calculation Constant Values
- **Test ID:** `UNIT-IMU-005`
- **Implementation:** [test_calculate_std_constant_values](units/test_imu_monitor.py#L231)
- **Given:** Buffer with all same values [5, 5, 5, 5, 5]
- **Action:** Calculate standard deviation
- **Expects:**
  - Variance equals 0.0
  - Standard deviation equals 0.0

#### Test: MSTD Calculation High Variance
- **Test ID:** `UNIT-IMU-006`
- **Implementation:** [test_calculate_std_high_variance](units/test_imu_monitor.py#L251)
- **Given:** Buffer with oscillating values [-10, 10, -10, 10, -10]
- **Action:** Calculate standard deviation
- **Expects:**
  - Standard deviation > 5.0
  - High variance detected correctly

---

## Appendix: Test ID Reference Table

| # | Test ID | Section | Test Name | Implementation |
|---|---------|---------|-----------|----------------|
| 1 | CAM-INIT-001 | 1.1 | [Camera Init Success](#test-camera-init-success) | [test_camera_init_success](api/test_camera_api.py#L21) |
| 2 | CAM-INIT-002 | 1.1 | [Camera Init Idempotency](#test-camera-init-idempotency) | [test_camera_init_idempotency](api/test_camera_api.py#L37) |
| 3 | CAM-INIT-003 | 1.1 | [Camera Deinit](#test-camera-deinit) | [test_camera_deinit](api/test_camera_api.py#L52) |
| 4 | CAM-INIT-004 | 1.1 | [Deinit Idempotency](#test-deinit-idempotency) | [test_deinit_idempotency](api/test_camera_api.py#L66) |
| 5 | CAM-INIT-005 | 1.1 | [Get Camera State](#test-get-camera-state) | [test_get_camera_state](api/test_camera_api.py#L82) |
| 6 | CAM-INIT-006 | 1.1 | [Concurrent Init Requests](#test-concurrent-init-requests) | [test_concurrent_init_requests](api/test_camera_api.py#L93) |
| 7 | CAM-INIT-007 | 1.1 | [Init During Deinit](#test-init-during-deinit) | [test_init_during_deinit](api/test_camera_api.py#L115) |
| 8 | CAM-INIT-008 | 1.1 | [Deinit During Init](#test-deinit-during-init) | [test_deinit_during_init](api/test_camera_api.py#L145) |
| 9 | CAM-INIT-009 | 1.1 | [Multiple Concurrent Deinit Requests](#test-multiple-concurrent-deinit-requests) | [test_multiple_concurrent_deinit_requests](api/test_camera_api.py#L172) |
| 10 | CAM-INIT-010 | 1.1 | [Get State During Init](#test-get-state-during-init) | [test_get_state_during_init](api/test_camera_api.py#L197) |
| 11 | CAM-INIT-011 | 1.1 | [Get State During Deinit](#test-get-state-during-deinit) | [test_get_state_during_deinit](api/test_camera_api.py#L224) |
| 12 | CAM-INIT-012 | 1.1 | [Rapid Init/Deinit Cycle](#test-rapid-initdeinit-cycle) | [test_rapid_init_deinit_cycle](api/test_camera_api.py#L254) |
| 13 | CAM-CAP-001 | 1.2 | [Single Frame Capture](#test-single-frame-capture) | [test_single_frame_capture](api/test_camera_api.py#L279) |
| 14 | CAM-CAP-002 | 1.2 | [Capture Without Init](#test-capture-without-init) | [test_capture_without_init](api/test_camera_api.py#L306) |
| 15 | CAM-CAP-003 | 1.2 | [Multiple Sequential Captures](#test-multiple-sequential-captures-back-to-back-when-previous-finish) | [test_multiple_sequential_captures](api/test_camera_api.py#L317) |
| 16 | CAM-REC-001 | 1.3 | [Start Recording](#test-start-recording) | [test_start_recording](api/test_camera_api.py#L354) |
| 17 | CAM-REC-002 | 1.3 | [Stop Recording](#test-stop-recording) | [test_stop_recording](api/test_camera_api.py#L374) |
| 18 | CAM-REC-003 | 1.3 | [Record Start Without Init](#test-record-start-without-init) | [test_record_start_without_init](api/test_camera_api.py#L411) |
| 19 | CAM-REC-004 | 1.3 | [Duplicate Record Start](#test-duplicate-record-start) | [test_duplicate_record_start](api/test_camera_api.py#L422) |
| 20 | CAM-REC-005 | 1.3 | [Record Stop Without Start](#test-record-stop-without-start) | [test_record_stop_without_start](api/test_camera_api.py#L441) |
| 21 | CAM-PREV-001 | 1.4 | [Switch Preview Index](#test-switch-preview-index) | [test_switch_preview_index](api/test_camera_api.py#L456) |
| 22 | CAM-PREV-002 | 1.4 | [Get Preview Index](#test-get-preview-index) | [test_get_preview_index](api/test_camera_api.py#L476) |
| 23 | CAM-PREV-003 | 1.4 | [Invalid Preview Index](#test-invalid-preview-index) | [test_invalid_preview_index](api/test_camera_api.py#L491) |
| 24 | CAM-SET-001 | 1.5 | [Get Camera Settings](#test-get-camera-settings) | [test_get_camera_settings](api/test_camera_api.py#L520) |
| 25 | CAM-SET-002 | 1.5 | [Update Camera Settings](#test-update-camera-settings) | [test_update_camera_settings](api/test_camera_api.py#L552) |
| 26 | CAM-SET-003 | 1.5 | [Update Camera Settings With Invalid Keys](#test-update-camera-settings-with-invalid-keys) | [test_update_camera_settings_invalid_keys](api/test_camera_api.py#L596) |
| 27 | MAP-LIFE-001 | 2.1 | [Start Mapping Success](#test-start-mapping-success) | [test_start_mapping_success](api/test_mapping_api.py#L26) |
| 28 | MAP-LIFE-002 | 2.1 | [Verify Node Launch After Start](#test-verify-node-launch-after-start) | [test_verify_node_launch_after_start](api/test_mapping_api.py#L49) |
| 29 | MAP-LIFE-003 | 2.1 | [Automatic Transition STARTING→INITIALIZING](#test-automatic-state-transition-starting-initializing) | [test_automatic_transition_starting_to_initializing](api/test_mapping_api.py#L74) |
| 30 | MAP-LIFE-004 | 2.1 | [Automatic Transition INITIALIZING→RUNNING](#test-automatic-state-transition-initializing-running) | [test_automatic_transition_initializing_to_running](api/test_mapping_api.py#L105) |
| 31 | MAP-LIFE-005 | 2.1 | [Duplicate Mapping Start (Running)](#test-duplicate-mapping-start-idempotency---running) | [test_duplicate_mapping_start_running](api/test_mapping_api.py#L137) |
| 32 | MAP-LIFE-006 | 2.1 | [Duplicate Mapping Start (Initializing)](#test-duplicate-mapping-start-idempotency---initializing) | [test_duplicate_mapping_start_initializing](api/test_mapping_api.py#L167) |
| 33 | MAP-LIFE-007 | 2.1 | [Start During Stopping](#test-start-during-stopping) | [test_start_during_stopping](api/test_mapping_api.py#L194) |
| 34 | MAP-LIFE-008 | 2.1 | [Start During Starting (Race)](#test-start-during-starting-race-condition) | [test_start_during_starting_race](api/test_mapping_api.py#L237) |
| 35 | MAP-LIFE-009 | 2.1 | [Stop Mapping from RUNNING](#test-stop-mapping-from-running-state) | [test_stop_mapping_from_running](api/test_mapping_api.py#L271) |
| 36 | MAP-LIFE-010 | 2.1 | [Stop Mapping During INITIALIZING](#test-stop-mapping-during-initializing) | [test_stop_mapping_during_initializing](api/test_mapping_api.py#L311) |
| 37 | MAP-LIFE-011 | 2.1 | [Stop Mapping During STARTING](#test-stop-mapping-during-starting) | [test_stop_mapping_during_starting](api/test_mapping_api.py#L336) |
| 38 | MAP-LIFE-012 | 2.1 | [Stop Mapping During STOPPING](#test-stop-mapping-during-stopping) | [test_stop_mapping_during_stopping](api/test_mapping_api.py#L367) |
| 39 | MAP-LIFE-013 | 2.1 | [Stop Mapping Idempotency](#test-stop-mapping-idempotency) | [test_stop_mapping_idempotency](api/test_mapping_api.py#L408) |
| 40 | MAP-STATE-001 | 2.2 | [Get Mapping State - IDLE](#test-get-mapping-state---idle) | [test_get_mapping_state_idle](api/test_mapping_api.py#L430) |
| 41 | MAP-STATE-002 | 2.2 | [Get Mapping State - STARTING](#test-get-mapping-state---starting) | [test_get_mapping_state_starting](api/test_mapping_api.py#L448) |
| 42 | MAP-STATE-003 | 2.2 | [Get Mapping State - INITIALIZING](#test-get-mapping-state---initializing) | [test_get_mapping_state_initializing](api/test_mapping_api.py#L478) |
| 43 | MAP-STATE-004 | 2.2 | [Get Mapping State - RUNNING](#test-get-mapping-state---running) | [test_get_mapping_state_running](api/test_mapping_api.py#L503) |
| 44 | MAP-STATE-005 | 2.2 | [Get Mapping State - STOPPING](#test-get-mapping-state---stopping) | [test_get_mapping_state_stopping](api/test_mapping_api.py#L530) |
| 45 | MAP-CONC-001 | 2.3 | [Concurrent Start Requests](#test-concurrent-start-requests) | [test_concurrent_start_requests](api/test_mapping_api.py#L575) |
| 46 | MAP-CONC-002 | 2.3 | [Concurrent Stop Requests](#test-concurrent-stop-requests) | [test_concurrent_stop_requests](api/test_mapping_api.py#L612) |
| 47 | MAP-CONC-003 | 2.3 | [Rapid Start/Stop Cycle](#test-rapid-startstop-cycle) | [test_rapid_start_stop_cycle](api/test_mapping_api.py#L659) |
| 48 | MAP-SET-001 | 2.4 | [Get Mapping Settings](#test-get-mapping-settings) | [test_get_mapping_settings](api/test_mapping_api.py#L711) |
| 49 | MAP-SET-002 | 2.4 | [Update Mapping Settings (Idle)](#test-update-mapping-settings-idle-state) | [test_update_mapping_settings_idle](api/test_mapping_api.py#L740) |
| 50 | MAP-SET-003 | 2.4 | [Update Settings During Mapping](#test-update-settings-during-mapping) | [test_update_settings_during_mapping](api/test_mapping_api.py#L778) |
| 51 | MAP-SET-004 | 2.4 | [Invalid Setting](#test-invalid-setting) | [test_invalid_setting](api/test_mapping_api.py#L831) |
| 52 | MAP-PREFIX-001 | 2.5 | [Start Mapping with Name Prefix](#test-start-mapping-with-name-prefix) | [test_start_mapping_with_name_prefix](api/test_mapping_api.py#L879) |
| 53 | MAP-PREFIX-002 | 2.5 | [Incremental Naming with Same Prefix](#test-incremental-naming-with-same-prefix) | [test_start_mapping_incremental_naming](api/test_mapping_api.py#L930) |
| 54 | CAT-RET-001 | 3.1 | [Get Full Catalog](#test-get-full-catalog) | [test_get_full_catalog](api/test_catalog_api.py#L24) |
| 55 | CAT-RET-002 | 3.1 | [Get Empty Catalog](#test-get-empty-catalog) | [test_get_empty_catalog](api/test_catalog_api.py#L101) |
| 56 | CAT-META-001 | 3.2 | [Get Image Metadata](#test-get-image-metadata) | [test_get_image_metadata](api/test_catalog_api.py#L134) |
| 57 | CAT-META-002 | 3.2 | [Get Metadata for Non-Existent File](#test-get-metadata-for-non-existent-file) | [test_get_metadata_nonexistent_file](api/test_catalog_api.py#L164) |
| 58 | CAT-DEL-001 | 3.3 | [Delete Item by UUID](#test-delete-item-by-uuid) | [test_delete_item_by_uuid](api/test_catalog_api.py#L183) |
| 59 | CAT-DEL-002 | 3.3 | [Delete Non-Existent Item](#test-delete-non-existent-item) | [test_delete_nonexistent_item](api/test_catalog_api.py#L227) |
| 60 | CAT-REN-001 | 3.4 | [Rename Item](#test-rename-item) | [test_rename_item](api/test_catalog_api.py#L247) |
| 61 | CAT-REN-002 | 3.4 | [Rename with Duplicate Name](#test-rename-with-duplicate-name) | [test_rename_with_duplicate_name](api/test_catalog_api.py#L296) |
| 62 | CAT-EXP-001 | 3.5 | [Export Item to USB](#test-export-item-to-usb) | [test_export_item_to_usb](api/test_catalog_api.py#L360) |
| 63 | CAT-EXP-002 | 3.5 | [Export Invalid Item](#test-export-invalid-item) | [test_export_invalid_item](api/test_catalog_api.py#L399) |
| 64 | CAT-FILE-001 | 3.6 | [Get Item Files](#test-get-item-files) | [test_get_item_files](api/test_catalog_api.py#L424) |
| 65 | CAT-FILE-002 | 3.6 | [Get Files Non-Existent Item](#test-get-files-for-non-existent-item) | [test_get_files_nonexistent_item](api/test_catalog_api.py#L461) |
| 66 | STO-REM-001 | 4.1 | [List USB Devices](#test-list-usb-devices) | [test_list_usb_devices_with_usb_connected](api/test_storage_api.py#L21) |
| 67 | STO-REM-002 | 4.1 | [No USB Devices](#test-no-usb-devices) | [test_list_usb_devices_with_no_usb](api/test_storage_api.py#L77) |
| 68 | STO-INT-001 | 4.2 | [Get Internal Storage Usage](#test-get-internal-storage-usage) | [test_get_internal_storage_usage](api/test_storage_api.py#L96) |
| 69 | STO-INT-002 | 4.2 | [Format Internal Storage](#test-format-internal-storage) | [test_format_internal_storage](api/test_storage_api.py#L159) |
| 70 | STO-EXT-001 | 4.3 | [Format External Storage](#test-format-external-storage) | [test_format_external_storage](api/test_storage_api.py#L207) |
| 71 | STO-EXT-002 | 4.3 | [Format Non-Existent Device](#test-format-non-existent-device) | [test_format_external_storage_non_existent_device](api/test_storage_api.py#L256) |
| 72 | EXP-QUE-001 | 5.1 | [Get Export Progress - Empty Queue](#test-get-export-progress---empty-queue) | [test_get_export_progress_empty_queue](api/test_export_api.py#L16) |
| 73 | EXP-QUE-002 | 5.1 | [Get Export Progress - Active](#test-get-export-progress---active) | [test_get_export_progress_active](api/test_export_api.py#L53) |
| 74 | EE-001 | 7 | [Complete Photo Capture Workflow](#ee-1-complete-photo-capture-workflow) | [test_ee_complete_photo_capture_workflow](ee/test_ee.py#L175) |
| 75 | EE-002 | 7 | [Video Recording Workflow](#ee-2-video-recording-workflow) | [test_ee_video_recording_workflow](ee/test_ee.py#L279) |
| 76 | EE-003 | 7 | [Spatial Mapping Workflow](#ee-3-spatial-mapping-workflow) | [test_ee_spatial_mapping_workflow](ee/test_ee.py#L426) |
| 77 | EE-004 | 7 | [Storage Management Workflow](#ee-4-storage-management-workflow) | [test_ee_storage_management_workflow](ee/test_ee.py#L723) |
| 78 | EE-005 | 7 | [Multi-Export Queue Management](#ee-5-multi-export-queue-management) | [test_ee_multi_export_queue_management](ee/test_ee.py#L808) |
| 79 | EE-006 | 7 | [Preview Switching During Recording](#ee-6-preview-switching-during-recording) | [test_ee_preview_switching_during_recording](ee/test_ee.py#L899) |
| 80 | LOAD-001 | 8 | [Rapid Capture Sequence](#load-1-rapid-capture-sequence) | [test_rapid_capture_sequence](test_performance.py#L26) |
| 81 | LOAD-002 | 8 | [Long Recording Session](#load-2-long-recording-session) | [test_long_recording_session](test_performance.py#L131) |
| 82 | LOAD-003 | 8 | [Large Export Queue](#load-3-large-export-queue) | [test_large_export_queue](test_performance.py#L252) |
| 83 | UNIT-FIN-001 | 9.1 | [do_finalize Valid PCD](#test-do_finalize-with-valid-pcd-file) | [test_do_finalize_valid_pcd](units/test_finalize_mapping.py#L120) |
| 84 | UNIT-FIN-002 | 9.1 | [do_finalize Missing Map](#test-do_finalize-with-missing-map-file) | [test_do_finalize_missing_map](units/test_finalize_mapping.py#L168) |
| 85 | UNIT-FIN-003 | 9.1 | [do_finalize PLY Format](#test-do_finalize-ply-format) | [test_do_finalize_ply_format](units/test_finalize_mapping.py#L202) |
| 86 | UNIT-FIN-004 | 9.1 | [do_finalize PCD Format](#test-do_finalize-pcd-format) | [test_do_finalize_pcd_format](units/test_finalize_mapping.py#L218) |
| 87 | UNIT-FIN-005 | 9.1 | [load_pointcloud PCD File](#test-load_pointcloud-pcd-file) | [test_load_pcd_file](units/test_finalize_mapping.py#L238) |
| 88 | UNIT-FIN-006 | 9.1 | [load_pointcloud Unsupported Format](#test-load_pointcloud-unsupported-format) | [test_load_unsupported_format](units/test_finalize_mapping.py#L253) |
| 89 | UNIT-FIN-007 | 9.1 | [calculate_area_volume Box](#test-calculate_area_volume-box) | [test_calculate_area_volume_box](units/test_finalize_mapping.py#L269) |
| 90 | UNIT-FIN-008 | 9.1 | [calculate_area_volume Empty](#test-calculate_area_volume-empty) | [test_calculate_area_volume_empty](units/test_finalize_mapping.py#L287) |
| 91 | UNIT-ROS-001 | 9.2 | [StartEverything Node Creation](#test-starteverything-node-creation) | [test_start_everything_nodes](units/test_ros2_lifecycle.py#L126) |
| 92 | UNIT-ROS-002 | 9.2 | [StartEverything Creates Dir](#test-starteverything-creates-artifact-directory) | [test_start_everything_artifact_dir](units/test_ros2_lifecycle.py#L188) |
| 93 | UNIT-ROS-003 | 9.2 | [StopEverything Shutdown](#test-stopeverything-process-shutdown) | [test_stop_everything_shutdown](units/test_ros2_lifecycle.py#L221) |
| 94 | UNIT-ROS-004 | 9.2 | [StopEverything Idempotent](#test-stopeverything-idempotency) | [test_stop_everything_idempotent](units/test_ros2_lifecycle.py#L280) |
| 95 | UNIT-ROS-005 | 9.2 | [Start/Stop Cycle](#test-startstop-cycle) | [test_start_stop_cycle](units/test_ros2_lifecycle.py#L302) |
| 96 | UNIT-IMU-001 | 9.3 | [Test Bag Available](#test-test-bag-available) | [test_bag_available](units/test_imu_monitor.py#L31) |
| 97 | UNIT-IMU-002 | 9.3 | [IMU Monitor Stabilization](#test-imu-monitor-detects-stabilization) | [test_imu_monitor_detects_stabilization](units/test_imu_monitor.py#L42) |
| 98 | UNIT-IMU-003 | 9.3 | [Status File Created on Start](#test-status-file-created-on-start) | [test_status_file_created_on_start](units/test_imu_monitor.py#L140) |
| 99 | UNIT-IMU-004 | 9.3 | [MSTD Calculation Basic](#test-mstd-calculation-basic) | [test_calculate_std_basic](units/test_imu_monitor.py#L200) |
| 100 | UNIT-IMU-005 | 9.3 | [MSTD Calculation Constant](#test-mstd-calculation-constant-values) | [test_calculate_std_constant_values](units/test_imu_monitor.py#L231) |
| 101 | UNIT-IMU-006 | 9.3 | [MSTD Calculation High Variance](#test-mstd-calculation-high-variance) | [test_calculate_std_high_variance](units/test_imu_monitor.py#L251) |

---

**Document Version:** 1.6
**Last Updated:** 2025-12-19
**Author:** Test Specification Document (Auto-generated from API analysis)
**Changelog:**
- v1.6: Added Scan Name Prefix tests (Section 2.5, MAP-PREFIX-001/002)
- v1.5: Added missing test specs (UNIT-FIN-004 to 007, UNIT-ROS-005, UNIT-IMU-002 to 005), renamed e2e to ee
- v1.4: Added IMU monitor unit test with mstd detection (Section 9.3)
- v1.3: Added unit tests for do_finalize, StartEverything, StopEverything (Section 9)
- v1.2: Added test IDs and implementation links for all implemented tests
- v1.1: Added camera state machine and concurrency tests with async/await implementation
