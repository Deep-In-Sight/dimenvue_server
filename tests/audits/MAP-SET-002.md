# Test Audit Report: MAP-SET-002

**Test ID:** MAP-SET-002
**Test Name:** Update Mapping Settings (Idle State)
**Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py#L740`
**Audit Date:** 2025-12-18

---

## Compliance Status: NON-COMPLIANT

---

## Specification Requirements

The test specification requires:

1. **Given:** No mapping in progress (state: idle)
2. **Action:** `PUT /mappingApp/settings` with body:
   ```json
   {
     "preview_voxel_size": 5,
     "file_format": "PLY"
   }
   ```
3. **Expects:**
   - Status: 200 OK
   - Settings persisted to `{data_path}/mapping_settings.json`
   - New settings dict has the "current_selection" updated for the requested keys
   - The updated settings must have the same structure as DEFAULT_MAPPING_SETTINGS, except the "current_selection" values

---

## Implementation Analysis

### What the Specification Requires

The spec shows a **flat format** request body where keys map directly to desired values:
```json
{
  "preview_voxel_size": 5,
  "file_format": "PLY"
}
```

This is the simplified API format where:
- `preview_voxel_size: 5` means "set preview voxel size to 5cm"
- `file_format: "PLY"` means "set file format to PLY"

The API should handle this flat format and convert it to the full structure internally, updating the appropriate `current_selection` indices.

### What the Implementation Does

The test implementation (lines 740-774) uses a **full format** request body:

```python
# Get current settings (full structure)
get_response = await async_client.get("/mappingApp/settings")
current_settings = get_response.json()

# Update settings by modifying current_selection indices
new_settings = current_settings.copy()
new_settings["preview_voxel_size"]["current_selection"] = 0
new_settings["file_format"]["current_selection"] = 0

# Send full structure
update_response = await async_client.put(
    "/mappingApp/settings",
    json={"settings": new_settings}
)
```

The implementation:
1. Fetches the entire settings structure
2. Modifies `current_selection` indices directly
3. Sends the full structure back to the API

### Key Discrepancies

1. **Request Body Format Mismatch:**
   - **Spec:** Uses flat format `{"preview_voxel_size": 5, "file_format": "PLY"}`
   - **Implementation:** Uses full format with explicit structure modification

2. **Value Mismatch:**
   - **Spec:** Requests `preview_voxel_size: 5` (should map to index 0 in `[5, 10, 15]`)
   - **Implementation:** Sets `current_selection: 0` directly (which correctly points to value 5)
   - The end result is the same, but the API format differs

3. **Extra Wrapper Object:**
   - **Implementation:** Wraps payload in `{"settings": new_settings}`
   - **Spec:** Does not show this wrapper (unclear if required)

---

## Backend Validation

The backend API endpoint at `/home/linh/ros2_ws/dimenvue_server/server_app.py#L237-243`:

```python
@app.put("/mappingApp/settings")
def put_mapping_settings(req: dict[str, Any]):
    new_settings = req.get("settings", req)
    try:
        return mappingApp.save_settings(new_settings)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
```

The `save_settings` method in `mapping_app.py` (lines 163-184) calls `_validate_settings` which **supports both formats**:

1. **Full format:** `{"key": {"options": [...], "current_selection": N}}`
2. **Flat format:** `{"key": value}` - value is matched against current options and converted to index

This means the API **does support** the flat format shown in the spec. The implementation should be testing the flat format, but instead tests the full format.

---

## Gaps and Issues

### Critical Issues

1. **Wrong Request Format:** The test does not validate that the API accepts the flat format shown in the specification
2. **Spec Format Not Tested:** The actual user-facing API format (flat format) is never exercised by this test

### Minor Issues

1. **Persistence Not Verified:** While the test re-fetches settings (lines 769-773), it doesn't explicitly verify that the `mapping_settings.json` file was created/updated on disk
2. **State Verification:** The test mocks `GetInitStatus` to return "UNKNOWN" but doesn't verify the mapping state is actually idle

---

## Recommendations

### 1. Fix Request Body Format (Critical)

Update the test to use the flat format as specified:

```python
# CURRENT (Wrong):
new_settings = current_settings.copy()
new_settings["preview_voxel_size"]["current_selection"] = 0
new_settings["file_format"]["current_selection"] = 0
update_response = await async_client.put(
    "/mappingApp/settings",
    json={"settings": new_settings}
)

# SHOULD BE (Correct):
update_response = await async_client.put(
    "/mappingApp/settings",
    json={
        "preview_voxel_size": 5,
        "file_format": "PLY"
    }
)
```

### 2. Verify File Persistence (Recommended)

Add explicit file system verification:

```python
# Verify settings persisted to disk
settings_file = Path(data_path) / "mapping_settings.json"
assert settings_file.exists()
with open(settings_file) as f:
    persisted = json.load(f)
    assert persisted["preview_voxel_size"]["current_selection"] == 0  # Index for value 5
    assert persisted["file_format"]["current_selection"] == 0  # Index for value "PLY"
```

### 3. Verify State Precondition (Recommended)

Verify the mapping state is actually idle before the test:

```python
# Verify initial state is idle
state_response = await async_client.get("/mappingApp/state")
assert state_response.json()["state"] == "idle"
```

### 4. Test Structure Validation (Recommended)

Add explicit validation that the response structure matches DEFAULT_MAPPING_SETTINGS:

```python
# Verify structure matches DEFAULT_MAPPING_SETTINGS
for key in ["preview_voxel_size", "file_format", "map_quality"]:
    assert key in updated_settings
    assert "options" in updated_settings[key]
    assert "current_selection" in updated_settings[key]
```

---

## Conclusion

The test is **NON-COMPLIANT** because it tests the wrong API format. The specification clearly shows a flat format request body (`{"preview_voxel_size": 5, "file_format": "PLY"}`), but the implementation tests a full structure format with explicit `current_selection` manipulation.

While the backend supports both formats, the test should validate the format shown in the specification - the flat format that represents the actual user-facing API. This is a critical gap because it means the simpler, documented API format is not being tested.

The test should be updated to use the flat format as specified, and additional verification should be added for file persistence and state preconditions.
