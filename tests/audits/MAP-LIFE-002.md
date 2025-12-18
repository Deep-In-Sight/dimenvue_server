# Test Audit Report: MAP-LIFE-002

## Test Information
- **Test ID:** MAP-LIFE-002
- **Test Name:** Verify Node Launch After Start
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py` (lines 49-70)
- **Audit Date:** 2025-12-18

## Compliance Status
**NON-COMPLIANT**

## Specification Requirements

The test specification requires:

1. **Precondition:** Mapping just started (state: starting)
2. **Action:**
   - Wait 2-3 seconds for nodes to initialize
   - Check running ROS2 nodes using `ros2 node list`
3. **Expectations:**
   - Verify nodes are visible in ROS2 graph:
     - `/fastlio_mapping`
     - `/imu_monitor`
     - `/bridge_node`
     - `/recorder_node`

## Implementation Analysis

The current implementation (lines 49-70):

```python
async def test_verify_node_launch_after_start(async_client: AsyncClient):
    """
    Test 2: Verify Node Launch After Start
    After start, ROS2 nodes should be launched
    """
    with patch('mapping_app.StartEverything', new_callable=AsyncMock) as mock_start, \
         patch('mapping_app.GetInitStatus', return_value="UNKNOWN"):

        # Action: Start mapping
        response = await async_client.put("/mappingApp/start")
        assert response.status_code == 200

        # Expects: StartEverything was called with correct parameters
        mock_start.assert_called_once()
        args = mock_start.call_args[0]

        # Should be called with file_format and artifact_dir
        assert len(args) == 2
        file_format, artifact_dir = args
        assert file_format in ["PLY", "PCD", "LAS", "LAZ"]
        assert "mapping_artifact" in artifact_dir
```

## Gaps and Discrepancies

### Critical Gaps

1. **No Wait Period**
   - **Required:** Wait 2-3 seconds for nodes to initialize
   - **Implemented:** No wait/sleep period
   - **Impact:** Cannot verify node launch timing behavior

2. **No ROS2 Node List Check**
   - **Required:** Execute `ros2 node list` command to check running nodes
   - **Implemented:** No subprocess call or ROS2 interaction
   - **Impact:** Core requirement completely missing

3. **No Node Verification**
   - **Required:** Verify 4 specific nodes are running:
     - `/fastlio_mapping`
     - `/imu_monitor`
     - `/bridge_node`
     - `/recorder_node`
   - **Implemented:** No node checking logic
   - **Impact:** Primary test objective not achieved

### What Is Actually Tested

The implementation only tests:
- The API returns 200 status code
- `StartEverything()` function is called once
- `StartEverything()` receives correct parameters (file_format and artifact_dir)

This is essentially a unit test of the API endpoint's parameter passing, not an integration test of node launch behavior.

## Recommendations

### Option 1: Full Integration Test (Preferred)
Implement the test as specified:

```python
async def test_verify_node_launch_after_start(async_client: AsyncClient):
    """
    Test 2: Verify Node Launch After Start
    After start, ROS2 nodes should be launched
    """
    # Given: Start mapping
    response = await async_client.put("/mappingApp/start")
    assert response.status_code == 200
    assert response.json()["state"] == "starting"

    # Action: Wait for nodes to initialize
    await asyncio.sleep(2.5)

    # Check running ROS2 nodes
    result = subprocess.run(
        ["ros2", "node", "list"],
        capture_output=True,
        text=True,
        timeout=5
    )

    # Expects: All required nodes are visible
    nodes = result.stdout.strip().split('\n')
    required_nodes = [
        '/fastlio_mapping',
        '/imu_monitor',
        '/bridge_node',
        '/recorder_node'
    ]

    for node in required_nodes:
        assert node in nodes, f"Node {node} not found in ROS2 graph"
```

### Option 2: Mock-Based Alternative
If real ROS2 integration is not desired in this test suite, update the specification to match the mock-based approach, or mark this as a unit test rather than an integration test.

### Option 3: Update Test ID
If the current implementation is intentional, rename the test to something like `test_start_mapping_parameters` and create a separate test for MAP-LIFE-002 that actually verifies node launch.

## Conclusion

The test implementation does not fulfill the specification requirements. It tests API parameter passing but does not verify that ROS2 nodes are actually launched and visible in the ROS2 graph. To achieve compliance, the test needs to:
1. Add a 2-3 second wait period
2. Execute `ros2 node list` command
3. Verify all 4 required nodes are present in the output
