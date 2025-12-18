# Test Audit Report: STO-EXT-002

## Test Information
- **Test ID:** STO-EXT-002
- **Test Name:** Format Non-Existent Device
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_storage_api.py#L256`
- **Audit Date:** 2025-12-18

## Compliance Status
**NON-COMPLIANT**

## Specification Requirements

### Given
- No device at `/media/invalid`

### Action
- `PUT /storage/external/format` with device `/media/invalid`

### Expected Result
- **Status:** 404 Not Found

## Implementation Analysis

### What is Implemented
The test implementation at lines 256-274:

1. **Mocks:** Uses `patch("os.path.ismount", return_value=False)` to simulate a non-existent mountpoint
2. **Request:** Sends `PUT /storage/external/format` with `{"mountpoint": "/invalid/mountpoint"}`
3. **Assertion:** Expects status code **400 Bad Request** (not 404)
4. **Validation:** Checks that response contains `"detail"` field with message containing `"not a valid mountpoint"`

### Key Code (lines 268-273):
```python
# The endpoint returns 400 Bad Request for ValueError, not 404
# Based on server_app.py line 247: ValueError -> HTTPException(status_code=400)
assert response.status_code == 400
data = response.json()
assert "detail" in data
assert "not a valid mountpoint" in data["detail"].lower()
```

## Discrepancies

### 1. HTTP Status Code Mismatch
- **Specification requires:** 404 Not Found
- **Implementation expects:** 400 Bad Request
- **Rationale in code:** Comment references `server_app.py line 247` where `ValueError` raises `HTTPException(status_code=400)`

### 2. Request Path Difference
- **Specification uses:** `/media/invalid` as the device path
- **Implementation uses:** `/invalid/mountpoint` as the mountpoint path

Both represent non-existent locations, so this is semantically equivalent but syntactically different.

## Root Cause Analysis

The implementation correctly tests the non-existent device scenario but asserts the **actual server behavior** (400 Bad Request) rather than the **specified behavior** (404 Not Found).

This indicates either:
1. The specification is incorrect and should be updated to require 400 Bad Request, OR
2. The server implementation (`server_app.py` line 247) is incorrect and should return 404 for non-existent devices

## Recommendations

### Option 1: Update Specification (Preferred)
If 400 Bad Request is the correct semantic response for an invalid mountpoint:
- Update TEST_SPECS.md to reflect "Status: 400 Bad Request"
- Rationale: 400 indicates client error (invalid parameter), which is semantically appropriate for a malformed/invalid mountpoint

### Option 2: Fix Server Implementation
If 404 Not Found is the desired behavior:
- Modify `server_app.py` line 247 to raise `HTTPException(status_code=404)` instead of 400
- Update test to expect 404
- Rationale: 404 indicates resource not found, which could be appropriate for a non-existent device

### Option 3: Distinguish Between Cases
Implement different status codes based on the error type:
- 400 Bad Request: Malformed mountpoint parameter (e.g., invalid format)
- 404 Not Found: Valid format but device doesn't exist
- This would require more sophisticated validation logic

## Conclusion

The test implementation is **NON-COMPLIANT** with the specification due to the HTTP status code mismatch (400 vs 404). However, the test accurately reflects the current server behavior. This is a specification-implementation alignment issue that requires either specification update or server code modification to achieve compliance.
