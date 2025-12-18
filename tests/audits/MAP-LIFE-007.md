# Test Audit Report: MAP-LIFE-007

## Test Information
- **Test ID:** MAP-LIFE-007
- **Test Name:** Start During Stopping
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py:194`
- **Audit Date:** 2025-12-18

## Compliance Status
**PARTIAL**

## Specification Requirements

### Given
- Mapping is being stopped (state: stopping)

### Action
- `PUT /mappingApp/start` while stop is in progress

### Expected Behavior
1. Status: 409 conflict
2. RuntimeError raised with message: "Cannot start: mapping is stopping"
3. Stop operation completes unaffected
4. Final state after stop completes: IDLE

## Implementation Analysis

### What is Implemented
The test implementation at lines 194-233 (`test_start_during_stopping`) covers:

1. **Setup Phase (lines 204-212):**
   - Starts mapping and transitions to RUNNING state
   - Uses proper mocking for StartEverything, StopEverything, GetInitStatus, and do_finalize

2. **Stopping State Creation (lines 215-222):**
   - Makes stop operation slow (0.5s delay) to create a window for testing
   - Initiates stop in background task
   - Waits 0.1s to ensure stop operation is in progress

3. **Concurrent Start Attempt (lines 224-229):**
   - Attempts to start while stopping
   - Verifies 409 Conflict status code
   - Checks that error message contains "stopping"

4. **Stop Completion (lines 231-232):**
   - Waits for stop operation to complete
   - Ensures stop operation is not interrupted

### What is Missing

1. **RuntimeError Exception Verification:**
   - The spec requires "RuntimeError raised with message: 'Cannot start: mapping is stopping'"
   - The implementation only checks the HTTP response status (409) and message content
   - Does NOT verify that a RuntimeError was actually raised internally
   - Note: This may be acceptable since the HTTP layer catches and converts the RuntimeError to a 409 response

2. **Exact Error Message:**
   - Spec requires: "Cannot start: mapping is stopping"
   - Implementation checks: `"stopping" in start_response.json()["detail"].lower()`
   - This is a loose check - it only verifies "stopping" appears somewhere in the message (case-insensitive)
   - Does not verify the exact message format

3. **Final State Verification:**
   - The spec requires verifying "Final state after stop completes: IDLE"
   - The implementation waits for stop to complete but does NOT verify the final state is IDLE
   - Missing assertion to check state after stop completes

## Gaps and Discrepancies

| Requirement | Status | Details |
|------------|--------|---------|
| 409 Conflict Status | ✓ Implemented | Line 228: `assert start_response.status_code == 409` |
| RuntimeError Raised | ✗ Not Verified | No verification of internal exception type |
| Exact Error Message | ⚠ Partial | Only checks "stopping" substring, not exact message |
| Stop Completes Unaffected | ✓ Implemented | Line 232: `await stop_task` ensures completion |
| Final State is IDLE | ✗ Missing | No assertion verifying final state after stop |

## Recommendations

1. **Add Final State Verification:**
   ```python
   # After line 232
   await stop_task

   # Verify final state is IDLE
   final_state_response = await async_client.get("/mappingApp/state")
   assert final_state_response.status_code == 200
   assert final_state_response.json()["state"] == "idle"
   ```

2. **Strengthen Error Message Verification (Optional):**
   ```python
   # Replace line 229 with more specific check
   detail = start_response.json()["detail"]
   assert "cannot start" in detail.lower() and "stopping" in detail.lower()
   ```

3. **Consider Internal Exception Testing (Optional):**
   - If testing internal implementation details is desired, add a test at the service layer to verify RuntimeError is raised
   - The current HTTP-level test is valid for integration testing

## Summary

The test provides good coverage of the core behavior (409 conflict during stop) but lacks verification of the final state after stop completes, which is explicitly required by the specification. The error message verification is also weaker than specified. These are relatively minor gaps that should be addressed for full compliance.

The test correctly validates the most critical aspect: preventing concurrent start operations during shutdown and ensuring the stop operation completes successfully.
