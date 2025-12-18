# Test Audit Report: MAP-LIFE-005

## Test Information
- **Test ID:** MAP-LIFE-005
- **Test Name:** Duplicate Mapping Start (Idempotency - Running)
- **Implementation Location:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py` (lines 137-163)
- **Function Name:** `test_duplicate_mapping_start_running`

## Compliance Status
**COMPLIANT**

## Specification Requirements

### Given
- Mapping process is already running (state: running)

### Action
- `PUT /mappingApp/start`

### Expected Response
- Status: 200 OK
- Response: `{"details": "already_running", "state": "running"}`

## Implementation Analysis

### What the Test Does

The test implementation correctly follows the specification:

1. **Setup Phase (lines 145-153):**
   - Mocks ROS2 dependencies (`StartEverything`, `GetInitStatus`)
   - Starts mapping with initial state UNKNOWN
   - Transitions through TRACKING (after 1.5s sleep)
   - Transitions to STABILIZED (after another 1.5s sleep)
   - This ensures the mapping process reaches the "running" state

2. **Action Phase (line 156):**
   - Executes `PUT /mappingApp/start` while system is in running state
   - This is the duplicate/idempotent start attempt

3. **Assertion Phase (lines 159-162):**
   - Verifies status code is 200 OK
   - Verifies response contains `"details": "already_running"`
   - Verifies response contains `"state": "running"`

### Code Snippet
```python
async def test_duplicate_mapping_start_running(async_client: AsyncClient):
    """
    Test 5: Duplicate Mapping Start (Running)
    Start when running → 200 OK, "already_running"
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock), \
         patch('mapping_app.GetInitStatus') as mock_status:

        # Start mapping and transition to RUNNING
        mock_status.return_value = "UNKNOWN"
        await async_client.put("/mappingApp/start")

        mock_status.return_value = "TRACKING"
        await asyncio.sleep(1.5)

        mock_status.return_value = "STABILIZED"
        await asyncio.sleep(1.5)

        # Try to start again while running
        response = await async_client.put("/mappingApp/start")

        # Expects: 200 OK with already_running
        assert response.status_code == 200
        data = response.json()
        assert data["details"] == "already_running"
        assert data["state"] == "running"
```

## Compliance Details

| Requirement | Specification | Implementation | Status |
|-------------|---------------|----------------|--------|
| Initial State | State: running | Transitions to running via UNKNOWN → TRACKING → STABILIZED | ✓ Pass |
| HTTP Method | PUT | `async_client.put("/mappingApp/start")` | ✓ Pass |
| Endpoint | /mappingApp/start | `/mappingApp/start` | ✓ Pass |
| Response Status | 200 OK | `assert response.status_code == 200` | ✓ Pass |
| Response Details | "already_running" | `assert data["details"] == "already_running"` | ✓ Pass |
| Response State | "running" | `assert data["state"] == "running"` | ✓ Pass |

## Gaps or Discrepancies
None identified. The implementation fully matches the specification.

## Recommendations
None required. The test is well-implemented and thoroughly validates the idempotency behavior when attempting to start an already-running mapping process.

## Additional Notes
- The test properly uses async/await patterns for integration testing
- Uses appropriate mocking of ROS2 dependencies
- Includes realistic state transition timing (1.5s sleeps for state machine polling)
- Test documentation clearly describes the purpose
- Part of comprehensive mapping lifecycle test suite (13 tests total in this section)
