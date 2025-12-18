# Test Audit Report: CAM-REC-001

## Test Information
- **Test ID:** CAM-REC-001
- **Test Name:** Start Recording
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_camera_api.py` (lines 354-372)
- **Audit Date:** 2025-12-18

## Compliance Status
**PARTIAL**

## Specification Requirements

### Given:
- Camera app is initialized
- No recording in progress

### Action:
- `PUT /cameraApp/recordStart`

### Expects:
1. Status: 200 OK
2. Recording state set to active
3. GStreamer recording pipelines start writing to disk

## Implementation Analysis

### What is Implemented:

1. **Pre-condition Setup** (Lines 358-359)
   - Camera initialization: `await async_client.put("/cameraApp/init")` ✓
   - Ensures no prior recording in progress (fresh state after init) ✓

2. **Action** (Line 366)
   - Correct endpoint: `PUT /cameraApp/recordStart` ✓

3. **Response Validation** (Lines 367-368)
   - Checks 200 OK status code ✓
   - Validates response contains "status" field ✓

4. **State Verification** (Lines 371-372)
   - Verifies state is "recording" ✓
   - Uses separate GET request to confirm state transition ✓

### What is Missing:

1. **GStreamer Pipeline Verification**
   - The test does NOT verify that GStreamer recording pipelines actually start writing to disk
   - The test mocks `os.makedirs` and `shutil.rmtree` (lines 362-363), which prevents actual disk operations
   - No verification that the three recording pipelines (`precord0`, `precord1`, `precord2`) are started
   - No verification that files are being written to `record_tmp_path`

## Gap Analysis

### Critical Gap:
The specification explicitly requires verification that "GStreamer recording pipelines start writing to disk". The current implementation:
- Uses mocks that prevent actual file system operations
- Does not verify pipeline state via GStreamer APIs
- Does not check for the existence or creation of output files in `record_tmp_path`

### Implementation Details from Source Code:
According to `/home/linh/ros2_ws/dimenvue_server/camera_app.py` (lines 573-579), the `_do_record_start()` method:
1. Clears and creates `record_tmp_path` directory
2. Starts three GStreamer pipelines: `precord0`, `precord1`, `precord2`
3. Each pipeline writes to a file like `{record_tmp_path}/{sensor_name}.{format}`

The test should verify at least one of these behaviors.

## Recommendations

### Option 1: Verify Pipeline State (Integration Test)
```python
# After recordStart, verify pipelines are playing
for i in range(3):
    pipeline_state = server_app.cameraApp.gstc.pipeline_get_state(f"precord{i}")
    assert pipeline_state == "playing"
```

### Option 2: Verify File Creation (EE Test)
```python
# Remove mocks and verify actual file operations
response = await async_client.put("/cameraApp/recordStart")
assert response.status_code == 200

# Verify record_tmp directory was created
record_tmp = Path(server_app.cameraApp.record_tmp_path)
assert record_tmp.exists()
assert record_tmp.is_dir()

# Wait briefly for pipelines to start writing
await asyncio.sleep(0.5)

# Verify output files are being created
for sensor in ["left", "front", "right"]:
    record_format = server_app.cameraApp._get_setting_value("record_format").lower()
    file_path = record_tmp / f"{sensor}.{record_format}"
    assert file_path.exists()
```

### Option 3: Mock Verification (Unit Test)
```python
# Keep mocks but verify they were called correctly
with patch('camera_app.os.makedirs') as mock_makedirs, \
     patch('camera_app.shutil.rmtree') as mock_rmtree:

    response = await async_client.put("/cameraApp/recordStart")

    # Verify directory operations
    mock_rmtree.assert_called_once()
    mock_makedirs.assert_called_once()

    # Verify pipeline start was called (requires mocking gstc.pipeline_play)
```

## Conclusion

The test successfully validates HTTP response and state transitions but fails to verify the core requirement that GStreamer pipelines actually write to disk. To achieve full compliance, the test should either:
1. Verify GStreamer pipeline state transitions
2. Verify actual file creation in integration/EE mode
3. Add mock verification for pipeline operations

Given the test is marked as `@pytest.mark.integration`, Option 1 or 2 would be most appropriate.
