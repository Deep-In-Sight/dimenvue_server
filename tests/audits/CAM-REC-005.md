# Test Audit Report: CAM-REC-005

## Test Information
- **Test ID:** CAM-REC-005
- **Test Name:** Record Stop Without Start
- **Implementation:** [test_record_stop_without_start](../api/test_camera_api.py#L441)
- **Audit Date:** 2025-12-18

## Compliance Status
**NON-COMPLIANT**

## Specification Requirements

### Given
- No recording in progress

### Action
- `PUT /cameraApp/recordStop`

### Expected Results
- **Status:** 400 Bad Request
- **Error message:** Indicates no recording in progress

## Actual Implementation

### Test Code (lines 441-448)
```python
async def test_record_stop_without_start(self, async_client, test_app):
    """20. Record Stop Without Start - PUT /cameraApp/recordStop when not recording."""
    # Initialize camera
    await async_client.put("/cameraApp/init")

    # Try to stop recording without starting
    response = await async_client.put("/cameraApp/recordStop")
    assert response.status_code == 409
```

### What Is Tested
- Camera is initialized (READY state)
- Attempts to stop recording without starting
- **Asserts status code:** 409 Conflict

## Discrepancies

### 1. Wrong HTTP Status Code
- **Specification requires:** 400 Bad Request
- **Implementation checks for:** 409 Conflict
- **Impact:** CRITICAL - The test validates the wrong status code

### 2. Missing Error Message Validation
- **Specification requires:** Validation that error message indicates no recording in progress
- **Implementation:** Does not validate the error message content
- **Impact:** MODERATE - Test does not verify the error response body

## API Behavior Analysis

The actual API implementation (`server_app.py` lines 194-211) shows:
- Returns `409 Conflict` when `RuntimeError` is raised
- The error occurs when camera is not in RECORDING state
- Error message from `camera_app.py:640`: `"Cannot stop recording: camera is {state.value}"`

**Finding:** The API implementation returns 409, not 400. This means either:
1. The specification is incorrect and should specify 409, OR
2. The API implementation is incorrect and should return 400

## HTTP Status Code Semantics

- **400 Bad Request:** Client sent a malformed or invalid request
- **409 Conflict:** Request conflicts with current state of the server

**Analysis:** Stopping a recording when not recording is a **state conflict**, making 409 semantically more appropriate than 400.

## Recommendations

### Option 1: Update Specification (Recommended)
Update TEST_SPECS.md to specify:
- Status: 409 Conflict (matches API behavior)
- Add error message validation to the spec

### Option 2: Update API Implementation
Change API to return 400 instead of 409 (less semantically correct)

### Option 3: Update Test Implementation
Regardless of which status code is chosen:
1. Add error message validation:
   ```python
   assert response.status_code == 409  # or 400
   assert "not" in response.json()["detail"].lower()
   assert "recording" in response.json()["detail"].lower()
   ```

## Conclusion

The test is **NON-COMPLIANT** with its specification due to the status code mismatch (409 vs 400). However, the test appears to be testing the **actual API behavior** correctly. This suggests the specification may need updating rather than the test itself. Additionally, the test should validate the error message content to be fully compliant with the spec's requirement for "error message indicates no recording in progress."
