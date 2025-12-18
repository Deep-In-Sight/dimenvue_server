# Test Audit Report: CAM-SET-002

## Test Information
- **Test ID:** CAM-SET-002
- **Test Name:** Update Camera Settings
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_camera_api.py#L552`
- **Audit Date:** 2025-12-18

## Compliance Status
**NON-COMPLIANT**

## Specification Requirements

### Given
Camera app is NOT recording/capturing

### Action
`PUT /cameraApp/settings` with body:
```json
{
  "resolution": "1920x1080",
  "framerate": 30
}
```

### Expected Behavior
1. Status: 200 OK
2. New settings dict has the "current_selection" updated for the requested keys
3. The updated settings must have the same structure as the DEFAULT_CAMERA_SETTINGS, except the "current_selection" values
4. Setting persisted to disk

## Implementation Analysis

### What the Test Does

1. **Gets current settings** via `GET /cameraApp/settings`
2. **Copies entire settings object** and modifies `current_selection` indices
3. **Sends full settings object** with `json={"settings": new_settings}` containing the complete settings structure
4. **Verifies 200 OK status**
5. **Verifies the changed setting** has updated `current_selection`
6. **Verifies persistence** by fetching settings again

### Critical Discrepancies

#### 1. Request Format Mismatch
- **Spec requires:** Partial update with simple key-value pairs
  ```json
  {
    "resolution": "1920x1080",
    "framerate": 30
  }
  ```
- **Test sends:** Full settings object wrapped in a `settings` key
  ```json
  {
    "settings": {
      "resolution": {"options": [...], "current_selection": 1},
      "framerate": {"options": [...], "current_selection": 2},
      ...entire settings structure...
    }
  }
  ```

#### 2. Value Format Mismatch
- **Spec requires:** Direct values (`"1920x1080"`, `30`)
- **Test sends:** Index values (`"current_selection": 1`)

The specification expects the API to accept simple key-value pairs and translate them into the appropriate `current_selection` indices, but the test bypasses this by sending the full internal structure.

#### 3. Partial Update Not Tested
- **Spec requires:** Updating only specific keys (`resolution` and `framerate`)
- **Test sends:** Complete settings object with all keys

The specification implies a partial update mechanism where only specified keys are modified, but the test sends the entire settings structure.

#### 4. Missing Precondition Check
- **Spec requires:** Camera app is NOT recording/capturing
- **Test implementation:** Does not verify or enforce this precondition

### What Works

1. Verifies 200 OK status code
2. Verifies settings are persisted (re-fetches and confirms changes)
3. Verifies structure integrity (all keys remain valid)
4. Tests with valid settings values

## API Implementation Notes

From `/home/linh/ros2_ws/dimenvue_server/server_app.py#L223-229`:
```python
@app.put("/cameraApp/settings")
def put_camera_settings(req: dict[str, Any]):
    new_settings = req.get("settings", req)
    try:
        return cameraApp.save_settings(new_settings)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
```

The API accepts either:
- `{"settings": {...}}` (what the test uses)
- Direct settings object `{...}` (fallback)

However, it expects the **full settings structure** with `current_selection` indices, not the simple key-value format specified in the test specification.

## Recommendations

### Critical Changes Required

1. **Fix Request Format:** Test should send partial updates with direct values as per spec:
   ```python
   update_response = await async_client.put(
       "/cameraApp/settings",
       json={
           "resolution": "1920x1080",
           "framerate": 30
       }
   )
   ```

2. **Update API Implementation:** The API endpoint needs to be modified to:
   - Accept simple key-value pairs
   - Map values to appropriate `current_selection` indices
   - Support partial updates (merge with existing settings)
   - Validate that values exist in the options list

3. **Add Precondition Check:** Test should verify camera is not recording/capturing:
   ```python
   status_response = await async_client.get("/cameraApp/state")
   assert status_response.json()["state"] not in ["recording", "capturing"]
   ```

4. **Test Partial Updates:** Verify that only specified keys are updated and others remain unchanged

### Alternative: Update Specification

If the current API behavior (accepting full settings structure) is intentional, then the test specification should be updated to match the actual API contract. However, this would be poor API design as it:
- Requires clients to fetch full settings before updating
- Exposes internal structure (`current_selection` indices)
- Makes updates more error-prone
- Violates REST best practices for partial updates

## Conclusion

The test is **NON-COMPLIANT** with its specification. There is a fundamental mismatch between the intended API contract (simple key-value partial updates) and what is actually implemented/tested (full settings structure with indices). This needs to be resolved by either fixing the implementation to match the specification or updating the specification to match the implementation.
