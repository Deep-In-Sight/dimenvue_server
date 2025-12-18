# Test Audit Report: MAP-LIFE-008

## Test Information
- **Test ID:** MAP-LIFE-008
- **Test Name:** Start During Starting (Race Condition)
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py#L237`
- **Date Audited:** 2025-12-18

## Compliance Status
**PARTIAL** - Test implementation covers core requirements but has gaps in verification completeness.

## Specification Requirements vs Implementation

### Requirement 1: Given State
- **Spec:** "Mapping is during startup (state: starting)"
- **Implementation:** Test creates race condition by making first start slow (0.3s) and sending second request after 0.05s delay
- **Status:** ✓ COMPLIANT - Effectively simulates the "starting" state condition

### Requirement 2: Action
- **Spec:** "Second `PUT /mappingApp/start` arrives before first completes"
- **Implementation:**
  ```python
  task1 = asyncio.create_task(async_client.put("/mappingApp/start"))
  await asyncio.sleep(0.05)  # Let first one enter
  task2 = asyncio.create_task(async_client.put("/mappingApp/start"))
  ```
- **Status:** ✓ COMPLIANT - Race condition properly simulated

### Requirement 3: First Request Response
- **Spec:** "First request: 200 OK, state → starting"
- **Implementation:**
  ```python
  assert 200 in status_codes
  ```
- **Status:** ⚠ PARTIAL - Verifies 200 status but does NOT verify state transition to "starting" or which request was first

### Requirement 4: Second Request Response
- **Spec:** "Second request: 409 conflict error with 'Cannot start: mapping is starting'"
- **Implementation:**
  ```python
  assert 409 in status_codes
  for r in responses:
      if r.status_code == 409:
          assert "starting" in r.json()["detail"].lower()
  ```
- **Status:** ✓ COMPLIANT - Verifies 409 status and checks for "starting" in error message

### Requirement 5: Only One LaunchService Created
- **Spec:** "Only one LaunchService created"
- **Implementation:** Mock verifies StartEverything is called, but does not explicitly verify it's called only once
- **Status:** ✗ NON-COMPLIANT - No assertion on `mock_start.call_count == 1`

### Requirement 6: Final State
- **Spec:** "Final state: initializing (first start succeeds)"
- **Implementation:** No verification of final state
- **Status:** ✗ NON-COMPLIANT - Missing assertion on final state

## Discrepancies and Gaps

1. **Missing Call Count Verification**: Test does not verify that `StartEverything` (LaunchService) is called exactly once, which is critical for preventing resource duplication in race conditions.

2. **Missing Final State Assertion**: Test does not verify that the final state is "initializing" after the first start completes successfully.

3. **Non-Deterministic Response Ordering**: Test accepts either request as first/second (200 or 409 in any order). While this is realistic for race conditions, spec implies first request should get 200 and second should get 409. Test should either:
   - Verify task1 gets 200 and task2 gets 409 (deterministic)
   - Or document that order is non-deterministic due to race nature

4. **No State Verification in Response**: Test does not verify the "state" field in the 200 OK response body.

## Recommendations

### High Priority
1. Add assertion to verify LaunchService is created only once:
   ```python
   mock_start.assert_called_once()
   ```

2. Add final state verification after both requests complete:
   ```python
   final_state = await async_client.get("/mappingApp/state")
   assert final_state.json()["state"] == "initializing"
   ```

### Medium Priority
3. Verify response body structure for the 200 OK response:
   ```python
   success_response = next(r for r in responses if r.status_code == 200)
   assert success_response.json()["state"] in ["starting", "initializing"]
   ```

4. Consider documenting the non-deterministic nature of which request "wins" the race, or make it deterministic by checking task1 specifically.

### Low Priority
5. Add explicit verification that the exact error message matches spec:
   ```python
   assert "cannot start: mapping is starting" in r.json()["detail"].lower()
   ```

## Code References

**Application Logic** (`/home/linh/ros2_ws/dimenvue_server/mapping_app.py:233-234`):
```python
if self.state in [MappingState.STARTING, MappingState.STOPPING]:
    raise RuntimeError(f"Cannot start: mapping is {self.state.value}")
```

**API Endpoint** (`/home/linh/ros2_ws/dimenvue_server/server_app.py:322-324`):
```python
except RuntimeError as e:
    # Mapping is in invalid state for start
    raise HTTPException(status_code=409, detail=str(e))
```

## Summary

The test successfully validates the core race condition behavior and prevents concurrent starts, which is the primary safety concern. However, it lacks verification of:
1. Resource creation count (critical for preventing resource leaks)
2. Final state after race resolution
3. Complete response body structure

These gaps should be addressed to ensure full compliance with the specification and comprehensive test coverage.
