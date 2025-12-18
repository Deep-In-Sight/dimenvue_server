# Test Audit Report: MAP-CONC-001

## Test Information
- **Test ID:** MAP-CONC-001
- **Test Name:** Concurrent Start Requests
- **Implementation Location:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py:575`
- **Audit Date:** 2025-12-18

## Compliance Status
**PARTIAL**

## Specification Requirements vs Implementation

### Requirements from Spec

| Requirement | Implementation Status | Notes |
|-------------|----------------------|-------|
| **Given:** Mapping is idle | ✅ IMPLEMENTED | Test uses default idle state from async_client fixture |
| **Action:** Send 3 concurrent PUT /mappingApp/start requests | ✅ IMPLEMENTED | Lines 586-591: Creates 3 concurrent tasks using asyncio.gather |
| **Expect:** First request 200 OK | ✅ IMPLEMENTED | Lines 594-598: Asserts exactly 1 success with status 200 |
| **Expect:** After finish getState returns "initializing" | ⚠️ PARTIALLY IMPLEMENTED | Lines 605-607: Checks state but accepts both "starting" OR "initializing", not specifically "initializing" |
| **Expect:** Second/third requests 409 Conflict | ✅ IMPLEMENTED | Lines 595-599: Asserts exactly 2 conflicts with status 409 |
| **Expect:** Only one LaunchService created | ❌ NOT IMPLEMENTED | LaunchService is not directly mocked or verified |
| **Expect:** Only one polling task started | ❌ NOT IMPLEMENTED | No verification of _polling_task creation count |

### Implementation Details

**What is tested:**
1. **Concurrent request handling** (lines 586-591): Uses `asyncio.gather()` to send 3 simultaneous PUT requests
2. **Response code distribution** (lines 594-599): Verifies exactly 1 success (200) and 2 conflicts (409)
3. **StartEverything call count** (line 602): Verifies the underlying ROS2 start function is called only once
4. **Final state check** (lines 605-607): Confirms system is in "starting" or "initializing" state

**What is NOT tested:**
1. **LaunchService instantiation**: The spec mentions "Only one LaunchService created" but the test doesn't directly verify this. LaunchService is not mocked in the test.
2. **Polling task creation**: The spec requires "Only one polling task started". The test doesn't verify `_polling_task` creation or count.
3. **Strict state requirement**: Spec says state should be "initializing" after finish, but implementation accepts either "starting" or "initializing".

## Gaps and Discrepancies

### Gap 1: LaunchService Verification
- **Spec Requirement:** Only one LaunchService created
- **Current Implementation:** Not verified
- **Impact:** Medium - Cannot confirm that the underlying ROS2 launch mechanism isn't being duplicated
- **Root Cause:** LaunchService is not directly used in mapping_app.py (based on grep results). This requirement may be outdated or implementation-specific.

### Gap 2: Polling Task Verification
- **Spec Requirement:** Only one polling task started
- **Current Implementation:** Not verified
- **Impact:** High - The polling task (`_polling_task`) drives state transitions. Multiple tasks could cause race conditions.
- **Evidence:** mapping_app.py line 213 creates polling task in `_do_start()`: `self._polling_task = asyncio.create_task(self._poll_init_status())`

### Gap 3: State Assertion Precision
- **Spec Requirement:** State should be "initializing" after finish
- **Current Implementation:** Accepts "starting" OR "initializing"
- **Impact:** Low - Both states are valid transitional states, but spec is more specific
- **Rationale:** The looser assertion may account for timing variations in async execution

## Recommendations

### Priority 1: Add Polling Task Verification
Add verification that only one `_polling_task` is created:
```python
# After line 602, add:
# Verify polling task state (would require accessing app._polling_task)
# Or mock asyncio.create_task to count invocations
```

### Priority 2: Clarify LaunchService Requirement
- If LaunchService is still part of the implementation, add mock verification
- If LaunchService has been replaced/removed, update the spec to reflect current architecture
- Consider whether `StartEverything` call count (already tested) is the equivalent verification

### Priority 3: Tighten State Assertion (Optional)
Consider making the state check more specific:
```python
# Instead of: assert state_data["state"] in ["starting", "initializing"]
# Use: assert state_data["state"] == "initializing"
```
However, this may introduce flakiness if timing varies. Current implementation may be pragmatic.

## Conclusion

The test provides **good coverage** of the core concurrency requirements: verifying that concurrent requests result in exactly one success, two conflicts, and one invocation of the start function. However, it lacks verification of internal implementation details (polling task creation) specified in the requirements, and has a minor discrepancy in state assertion precision.

**Recommendation:** Upgrade to COMPLIANT by adding polling task verification and clarifying the LaunchService requirement in the spec.
