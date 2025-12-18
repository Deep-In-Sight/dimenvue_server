# Test Audit Report: MAP-LIFE-009

**Test ID:** MAP-LIFE-009
**Test Name:** Stop Mapping from RUNNING State
**Implementation:** [test_stop_mapping_from_running](../api/test_mapping_api.py#L271)
**Audit Date:** 2025-12-18
**Status:** PARTIAL

---

## Specification Requirements

According to TEST_SPECS.md, the test should verify:

1. **Given:** Mapping has been running for at least 15 seconds (IMU stabilized)
2. **Action:** `PUT /mappingApp/stop`
3. **Expects:**
   - Status: 200 OK
   - Response: `{"details": "stopped", "state": "idle"}` (or similar)
   - State transitions: RUNNING → STOPPING → IDLE
   - Finalization script executes: `finalize_mapping.py`
   - Point cloud file created in artifact_dir (format based on settings)
   - Catalog updated with new `Scan` entry
   - Entry includes:
     - UUID
     - Name (auto-generated or timestamped)
     - File paths (point cloud, thumbnail if generated)
     - Metadata (point count, bounding box, etc.)

---

## Implementation Analysis

### What is Implemented

The test at lines 271-307 implements the following:

**Setup:**
- Mocks ROS2 functions: `StartEverything`, `StopEverything`, `GetInitStatus`, and `do_finalize`
- Starts mapping and transitions through states: STARTING → INITIALIZING → RUNNING
- Uses async sleep to allow state transitions (1.5 seconds each)

**Test Actions:**
- Calls `PUT /mappingApp/stop` when in RUNNING state
- Verifies HTTP response status and payload

**Assertions:**
- ✅ Status: 200 OK
- ✅ Response structure: `{"details": "stopped", "state": "idle"}`
- ✅ `StopEverything` was called once
- ✅ `do_finalize` was called once
- ✅ Catalog endpoint returns 200 OK (line 305-306)

### What is Missing

1. **Pre-condition verification:**
   - Does NOT verify mapping has been running for "at least 15 seconds"
   - Only waits 3 seconds total (1.5s for INITIALIZING + 1.5s for RUNNING transition)
   - Specification suggests 15 seconds to ensure IMU stability and data accumulation

2. **State transition verification:**
   - Does NOT explicitly verify RUNNING → STOPPING → IDLE state transitions
   - Only checks final state after stop completes
   - The STOPPING state is transient and not verified

3. **Finalization verification:**
   - ✅ Verifies `do_finalize` was called (mocked)
   - ❌ Does NOT verify point cloud file was created in artifact_dir
   - ❌ Does NOT verify file format matches settings
   - ❌ Does NOT verify finalization script parameters

4. **Catalog verification:**
   - ✅ Checks catalog endpoint responds with 200 OK
   - ❌ Does NOT verify catalog was actually updated with a new Scan entry
   - ❌ Does NOT verify entry contains required fields:
     - UUID
     - Name (auto-generated or timestamped)
     - File paths (point cloud, thumbnail)
     - Metadata (point count, bounding box, etc.)

5. **Artifact directory verification:**
   - ❌ Does NOT verify artifact directory structure
   - ❌ Does NOT verify point cloud file exists
   - ❌ Does NOT verify thumbnail generation (if applicable)

---

## Gaps and Discrepancies

### Critical Gaps

1. **Catalog Content Verification:**
   - The test only checks if `/catalog` returns 200 OK
   - It should parse the response and verify:
     - A new Scan item was added
     - The item has all required fields (UUID, name, file_paths, metadata)
     - The metadata contains expected fields (point count, bounding box)

2. **File Creation Verification:**
   - No verification that finalization actually created output files
   - Should check artifact directory for:
     - Point cloud file in the correct format (PLY/PCD/LAS/LAZ based on settings)
     - Metadata file (if generated)
     - Thumbnail (if generated)

3. **Timing/Stabilization:**
   - Spec requires "at least 15 seconds (IMU stabilized)"
   - Implementation only waits ~3 seconds total
   - This may not adequately simulate real-world conditions

### Non-Critical Gaps

1. **State Transition Observability:**
   - The STOPPING state is not verified
   - While this is a transient state and hard to catch, the spec explicitly mentions it
   - Could use a slower mock to observe this state

2. **Settings Integration:**
   - Does not verify the point cloud format matches the current mapping settings
   - Should check that file_format setting is respected

---

## Recommendations

### High Priority

1. **Add Catalog Entry Verification:**
   ```python
   # After catalog_response
   catalog_data = catalog_response.json()
   assert len(catalog_data["items"]) > 0, "Expected scan entry in catalog"

   # Find the newly added scan
   scan_items = [item for item in catalog_data["items"] if item["type"] == "Scan"]
   assert len(scan_items) > 0, "Expected at least one Scan entry"

   latest_scan = scan_items[-1]
   assert "uuid" in latest_scan
   assert "name" in latest_scan
   assert "file_paths" in latest_scan
   assert "metadata" in latest_scan
   ```

2. **Verify Finalization Execution:**
   ```python
   # Verify do_finalize was called with correct artifact directory
   mock_finalize.assert_called_once()
   finalize_args = mock_finalize.call_args[0]
   assert len(finalize_args) > 0
   artifact_dir = finalize_args[0]
   assert "mapping_artifact" in artifact_dir
   ```

3. **Extend Running Duration:**
   ```python
   # After reaching RUNNING state
   await asyncio.sleep(15)  # Wait 15 seconds as spec requires
   ```

### Medium Priority

4. **Add File Creation Verification:**
   - If testing with actual finalization (not just mocked), verify files exist:
   ```python
   # After stop completes
   assert os.path.exists(os.path.join(artifact_dir, "*.ply"))  # or other formats
   ```

5. **Verify Settings Respect:**
   ```python
   # Before stop, get current settings
   settings_response = await async_client.get("/mappingApp/settings")
   file_format = settings_response.json()["file_format"]["current_selection"]

   # After finalization, verify file format matches
   ```

### Low Priority

6. **Add State Transition Verification:**
   - Use a slow mock to observe STOPPING state
   - Poll state endpoint during stop operation

---

## Conclusion

**Compliance Status:** PARTIAL

The test implementation covers the basic happy-path scenario but lacks comprehensive verification of the most critical requirement: catalog entry creation and validation. The test successfully verifies that:
- The API endpoint returns correct HTTP response
- Stop and finalize functions are called
- Final state is idle

However, it fails to verify:
- Catalog was updated with proper Scan entry structure
- Entry contains all required metadata fields
- Point cloud files were created
- File format matches settings
- Adequate stabilization time (15 seconds)

**Risk Level:** Medium - The test may pass even if catalog integration or file creation fails.

**Recommended Action:** Implement catalog content verification (High Priority recommendations) to ensure the test actually validates the complete stop-and-finalize workflow.
