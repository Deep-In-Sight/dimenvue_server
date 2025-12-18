# Test Audit Report: MAP-LIFE-001

## Test Information
- **Test ID:** MAP-LIFE-001
- **Test Name:** Start Mapping Success
- **Implementation:** [test_start_mapping_success](../api/test_mapping_api.py#L26)
- **Audit Date:** 2025-12-18

## Compliance Status
**PARTIAL COMPLIANCE**

## Specification Requirements vs Implementation

### What the Spec Requires

From TEST_SPECS.md (lines 365-380):

**Given:**
- No mapping running (state: idle)
- ROS2 is available (launch module can be imported)
- ROS2 workspace exists at `/ros2_ws` with fast_lio and point_cloud_bridge installed
- Artifact directory is writable

**Action:**
- `PUT /mappingApp/start`

**Expects:**
- Status: 200 OK
- Response: `{"details": "started", "state": "starting"}` (or similar structure)
- State immediately transitions to STARTING
- All nodes started
- Artifact directory created: `{data_path}/mapping_artifact/`
- IMU status file created: `{artifact_dir}/imu_stabilization_status.txt`

### What is Actually Implemented

From test_mapping_api.py (lines 26-45):

```python
@pytest.mark.asyncio
@pytest.mark.integration
async def test_start_mapping_success(async_client: AsyncClient):
    """
    Test 1: Start Mapping Success
    PUT /mappingApp/start when idle → 200 OK, state "starting"
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Action: Start mapping
        response = await async_client.put("/mappingApp/start")

        # Expects: 200 OK, state "starting"
        assert response.status_code == 200
        data = response.json()
        assert data["state"] == "starting"
        assert data["details"] == "started"

        # Verify ROS2 start was called
        mock_start.assert_called_once()
```

## Gap Analysis

### ✅ COMPLIANT Items

1. **HTTP Status Code (200 OK)** - Verified at line 38
2. **Response Structure** - Both "state" and "details" fields verified (lines 40-41)
3. **Response Values** - Correctly expects `{"state": "starting", "details": "started"}`
4. **StartEverything Called** - Verifies ROS2 start function is called once (line 44)

### ⚠️ GAPS and Missing Verifications

1. **Precondition: Initial State Not Verified**
   - Spec requires: "No mapping running (state: idle)"
   - Implementation: Does NOT verify the initial state is idle before the action
   - Impact: Test could pass even if system was in wrong initial state

2. **Precondition: ROS2 Availability Not Checked**
   - Spec requires: "ROS2 is available (launch module can be imported)"
   - Implementation: Uses mocks, doesn't verify actual ROS2 availability
   - Note: This is acceptable for integration tests with mocked dependencies

3. **Artifact Directory Creation Not Verified**
   - Spec requires: "Artifact directory created: `{data_path}/mapping_artifact/`"
   - Implementation: Does NOT verify directory creation
   - Impact: Critical postcondition missing

4. **IMU Status File Creation Not Verified**
   - Spec requires: "IMU status file created: `{artifact_dir}/imu_stabilization_status.txt`"
   - Implementation: Does NOT verify file creation
   - Impact: Critical postcondition missing

5. **State Transition Not Explicitly Verified**
   - Spec requires: "State immediately transitions to STARTING"
   - Implementation: Only checks response, doesn't verify actual state machine transition
   - Note: Response check may be sufficient, but could be more explicit

6. **Nodes Actually Started Not Verified**
   - Spec requires: "All nodes started"
   - Implementation: Only verifies `StartEverything` was called, not that nodes actually launched
   - Note: Test MAP-LIFE-002 handles this verification separately

## Recommendations

### Priority 1 (Critical)

1. **Add Artifact Directory Verification**
   ```python
   # After the start call succeeds
   # Get the artifact directory path from the app configuration
   artifact_dir = f"{data_path}/mapping_artifact/"
   assert os.path.exists(artifact_dir), "Artifact directory not created"
   ```

2. **Add IMU Status File Verification**
   ```python
   imu_status_file = f"{artifact_dir}/imu_stabilization_status.txt"
   assert os.path.exists(imu_status_file), "IMU status file not created"
   ```

### Priority 2 (Recommended)

3. **Verify Initial State is Idle**
   ```python
   # Before the action
   state_response = await async_client.get("/mappingApp/state")
   assert state_response.json()["state"] == "idle", "System not in idle state"
   ```

4. **Add State Machine Transition Verification**
   ```python
   # After the start call
   state_response = await async_client.get("/mappingApp/state")
   assert state_response.json()["state"] == "starting", "State did not transition to starting"
   ```

### Priority 3 (Optional)

5. **Add Comment About Node Launch Testing**
   - The test relies on MAP-LIFE-002 (test_verify_node_launch_after_start) to verify actual node launch
   - Add a comment clarifying this test division of responsibilities

## Notes

- The test uses mocks (`patch('mapping_app.StartEverything')`) which is appropriate for unit/integration testing
- The actual ROS2 node verification is delegated to MAP-LIFE-002, which is a reasonable test design
- The core API contract (endpoint, response format) is well-tested
- The filesystem side effects (directories, files) are not verified, which is the main gap

## Conclusion

The test successfully verifies the API contract and basic functionality but **lacks verification of important side effects** specified in the requirements (artifact directory and IMU status file creation). These should be added to achieve full compliance with the specification.

**Recommended Action:** Enhance the test to verify filesystem side effects before marking as fully compliant.
