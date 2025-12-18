# Test Audit Report: STO-EXT-001

## Test Information
- **Test ID:** STO-EXT-001
- **Test Name:** Format External Storage
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_storage_api.py:207`
- **Audit Date:** 2025-12-18

## Compliance Status
**PARTIAL COMPLIANCE**

## Specification Requirements

### Given
- USB device at `/media/usb0` with files

### Action
- `PUT /storage/external/format` with body:
  ```json
  {
    "device": "/media/usb0"
  }
  ```

### Expected Behavior
- Status: 200 OK
- All files deleted from the USB mounted filesystem

## Implementation Analysis

### What is Implemented
The test implementation (`test_format_external_storage` at line 207) correctly verifies:

1. **Status Code:** Asserts `response.status_code == 200` (line 239)
2. **File Deletion:** Verifies that all files and directories are deleted from the mountpoint:
   - Root-level files deleted (lines 245-246)
   - Nested files in subdirectories deleted (line 247)
   - Subdirectories themselves deleted (line 248)
3. **Mountpoint Preservation:** Confirms the mountpoint directory itself still exists (line 251)
4. **Success Response:** Validates response contains success status and mountpoint in message (lines 241-242)

### Discrepancies

1. **Request Body Key Mismatch:**
   - **Spec requires:** `{"device": "/media/usb0"}`
   - **Implementation uses:** `{"mountpoint": str(mock_mountpoint)}` (line 236)
   - The JSON key is `"mountpoint"` instead of `"device"`

2. **Device Path Mismatch:**
   - **Spec specifies:** `/media/usb0` (a typical USB mount path)
   - **Implementation uses:** A temporary path created by `tmp_path / "usb_mount"` (line 213)
   - While functionally equivalent for testing, it doesn't use the exact path from the spec

## Impact Assessment

### Critical Issues
- **Request Body Schema:** The test uses `"mountpoint"` as the JSON key, but the specification requires `"device"`. This is a contract violation that could cause integration issues if the API expects `"device"`.

### Minor Issues
- **Path Convention:** Using `tmp_path` instead of `/media/usb0` is acceptable for testing isolation, but the discrepancy should be noted.

## Recommendations

1. **Immediate Action Required:**
   - Verify the actual API implementation in `server_app.py` to determine which JSON key is correct (`"device"` or `"mountpoint"`)
   - Update either the test specification or the test implementation to align with the actual API contract
   - If API uses `"mountpoint"`, update TEST_SPECS.md line referencing this test
   - If API uses `"device"`, update test implementation line 236

2. **Documentation:**
   - Add a code comment explaining why `tmp_path` is used instead of `/media/usb0` (for test isolation)
   - Consider adding a constant or variable named `SPEC_DEVICE_PATH = "/media/usb0"` to maintain traceability to the spec

3. **Test Enhancement:**
   - Consider adding a test that uses a path closer to the real-world scenario (e.g., creating a mock at `/tmp/media/usb0`)
   - Verify the test covers edge cases like empty directories, read-only files, and symlinks

## Conclusion

The test implementation correctly validates the core functionality (200 OK response and file deletion), but has a critical discrepancy in the request body schema. This must be resolved to ensure the test accurately reflects the API specification.
