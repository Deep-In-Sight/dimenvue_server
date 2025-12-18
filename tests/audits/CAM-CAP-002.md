# Test Audit Report: CAM-CAP-002

## Test Information
- **Test ID:** CAM-CAP-002
- **Test Name:** Capture Without Init
- **Implementation Location:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_camera_api.py#L306`
- **Audit Date:** 2025-12-18

## Compliance Status
**NON-COMPLIANT**

## Specification Requirements

### Given
Camera app is NOT initialized

### Action
`PUT /cameraApp/capture`

### Expected Behavior
1. Status: **400 Bad Request**
2. Error message indicates camera not initialized
3. No files created

## Actual Implementation

### Test Code (Lines 306-315)
```python
async def test_capture_without_init(self, async_client, test_app):
    """14. Capture Without Init - PUT /cameraApp/capture when not initialized."""
    # Ensure camera is not initialized
    state_resp = await async_client.get("/cameraApp/state")
    if state_resp.json()["state"] != "idle":
        await async_client.put("/cameraApp/deinit")

    # Try to capture without init
    response = await async_client.put("/cameraApp/capture")
    assert response.status_code == 409
```

### Actual Behavior
1. Status: **409 Conflict** (not 400 Bad Request)
2. Error message: RuntimeError from camera_app.py indicating invalid state
3. No file verification performed

## Discrepancies

### 1. Incorrect Status Code (Critical)
- **Specified:** 400 Bad Request
- **Implemented:** 409 Conflict
- **Root Cause:** The endpoint `/cameraApp/capture` in `server_app.py` (line 163) catches `RuntimeError` and returns 409 instead of 400

### 2. Missing Error Message Validation
- **Specified:** Error message indicates camera not initialized
- **Implemented:** No assertion on error message content
- **Gap:** Test does not verify the error message mentions camera not being initialized

### 3. Missing File Creation Check
- **Specified:** No files created
- **Implemented:** No verification that files were not created
- **Gap:** Test does not check that no files exist in capture directories

## API Implementation Analysis

From `server_app.py` lines 146-163:
```python
@app.put("/cameraApp/capture")
async def capture():
    try:
        await cameraApp.capture()
        # ...
    except RuntimeError as e:
        # Camera is in invalid state for capture
        raise HTTPException(status_code=409, detail=str(e))
```

From `camera_app.py` lines 558-559:
```python
if self.state != CameraState.READY:
    raise RuntimeError(f"Cannot capture: camera is {self.state.value}")
```

When camera is not initialized (state is IDLE), the RuntimeError is raised with message "Cannot capture: camera is idle", which gets converted to a 409 Conflict response.

## Recommendations

### Option 1: Update Specification (Recommended)
The API implementation uses 409 Conflict for state-based errors, which is semantically appropriate per RFC 7231. The specification should be updated to match:
- Change expected status from 400 to 409
- Update error message expectation to "Cannot capture: camera is idle"

### Option 2: Update Implementation
If 400 Bad Request is the required standard:
- Modify `server_app.py` to distinguish between initialization state errors (400) and other state errors (409)
- Update exception handling logic in the capture endpoint

### Option 3: Enhance Test Coverage
Regardless of which option is chosen, the test should be enhanced to:
1. Verify the error message content explicitly
2. Check that no files were created in the capture directories
3. Verify the camera state remains idle after the failed capture attempt

## Conclusion

The test implementation does not comply with the specification due to an incorrect expected status code (409 vs 400). Additionally, the test lacks assertions for error message validation and file creation verification. The discrepancy likely stems from a semantic decision in the API design to use 409 for state-related conflicts, which should be reconciled either by updating the specification or modifying the implementation.
