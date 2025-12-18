# Test Audit Report: MAP-LIFE-010

**Test ID:** MAP-LIFE-010
**Test Name:** Stop Mapping During INITIALIZING
**Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py#L311`
**Date:** 2025-12-18

---

## Compliance Status: PARTIAL

---

## Specification Requirements

### Given
- Mapping is in INITIALIZING state

### Action
- `PUT /mappingApp/stop`

### Expected Outcomes
1. Status: 409 conflict
2. Initialization continues

---

## Implementation Analysis

### What is Implemented

The test at lines 311-331 implements the following:

```python
async def test_stop_mapping_during_initializing(async_client: AsyncClient):
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
```

**Verification Steps:**
1. Sets up mock to return "UNKNOWN" status
2. Calls start mapping (transitions to STARTING)
3. Changes mock to return "TRACKING" status
4. Waits 1.5 seconds (allows background task to transition to INITIALIZING)
5. Calls stop mapping
6. Asserts 409 status code
7. Asserts error message contains "initializing"

---

## Gaps and Discrepancies

### Gap 1: Missing Verification of "Initialization Continues"

**Specification Requirement:**
> initialization continues

**Current Implementation:**
The test does NOT verify that initialization continues after the 409 response. It only checks:
- The stop request returns 409
- The error message mentions "initializing"

**Missing Verifications:**
- No assertion that the state remains INITIALIZING after the failed stop attempt
- No verification that the background initialization task continues running
- No check that the system can eventually transition to RUNNING state

### Gap 2: Timing Assumption

**Issue:**
The test relies on `await asyncio.sleep(1.5)` to allow the state transition from STARTING to INITIALIZING. This is fragile because:
- The background monitor task runs every 1.0 second (see `mapping_app.py` line 206)
- There's no explicit verification that INITIALIZING state was actually reached before the stop attempt
- If the background task is delayed, the test might attempt to stop during STARTING instead

**Risk:**
The test could pass while testing the wrong state (STARTING instead of INITIALIZING), since both states return 409 on stop attempts.

---

## Recommendations

### Critical: Add Verification of Continued Initialization

Add explicit verification that initialization continues:

```python
# After the 409 response, verify state remains INITIALIZING
state_response = await async_client.get("/mappingApp/state")
assert state_response.json()["state"] == "initializing"

# Verify initialization can complete
mock_status.return_value = "STABILIZED"
await asyncio.sleep(1.5)

final_state = await async_client.get("/mappingApp/state")
assert final_state.json()["state"] == "running"
```

### Important: Replace Timing Assumption with State Verification

Replace the sleep-based approach with explicit state polling:

```python
# Wait for INITIALIZING state explicitly
for _ in range(5):
    state = await async_client.get("/mappingApp/state")
    if state.json()["state"] == "initializing":
        break
    await asyncio.sleep(0.5)
else:
    pytest.fail("State did not transition to INITIALIZING")

# Now attempt stop
response = await async_client.put("/mappingApp/stop")
```

---

## Summary

The test correctly verifies the **409 status code requirement** but fails to verify the **"initialization continues"** requirement. This represents a partial implementation of the specification. The test should be enhanced to confirm that the initialization process is not disrupted by the rejected stop request and can complete successfully.

**Compliance Breakdown:**
- Status 409 conflict: ✓ IMPLEMENTED
- Initialization continues: ✗ NOT VERIFIED
