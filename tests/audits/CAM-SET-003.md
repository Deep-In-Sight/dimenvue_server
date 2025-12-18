# Test Audit Report: CAM-SET-003

## Test Information
- **Test ID:** CAM-SET-003
- **Test Name:** Update Camera Settings With Invalid Keys
- **Implementation Location:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_camera_api.py#L596`
- **Audit Date:** 2025-12-18

## Compliance Status
**NON-COMPLIANT**

## Specification Requirements

### Preconditions
- Camera app is NOT recording/capturing

### Action
- Send `PUT /cameraApp/settings` with body containing:
  - Invalid key: `"wrong_key": "value"`
  - Valid key: `"frame_rate": 30`

### Expected Behavior
1. Status: 400 Bad Request
2. **Invalid keys are ignored, the valid keys are updated**

## Actual Implementation

### Test Implementation (lines 596-617)
```python
async def test_update_camera_settings_invalid_keys(self, async_client, test_app):
    """26. Update Camera Settings With Invalid Keys - Invalid keys are rejected."""
    # Get current settings
    get_response = await async_client.get("/cameraApp/settings")
    assert get_response.status_code == 200
    current_settings = get_response.json()

    # Create settings with invalid keys
    invalid_settings = current_settings.copy()
    invalid_settings["wrong_key"] = {"options": ["a", "b"], "current_selection": 0}
    invalid_settings["frame_rate"] = {"options": [30, 60], "current_selection": 0}  # typo

    # Try to update with invalid settings
    response = await async_client.put(
        "/cameraApp/settings",
        json={"settings": invalid_settings}
    )

    # Should return 400 Bad Request for invalid keys
    assert response.status_code == 400
    data = response.json()
    assert "detail" in data
```

### Endpoint Implementation (camera_app.py, lines 160-180)
```python
def save_settings(self, new_settings: dict):
    """Update and save settings.

    Raises:
        ValueError: If new_settings contains invalid keys
    """
    # Validate keys - only allow keys that exist in DEFAULT_CAMERA_SETTINGS
    valid_keys = set(DEFAULT_CAMERA_SETTINGS.keys())
    invalid_keys = set(new_settings.keys()) - valid_keys
    if invalid_keys:
        raise ValueError(f"Invalid settings keys: {', '.join(sorted(invalid_keys))}")

    self.settings = new_settings
    self._save_settings()
    return self.settings
```

## Discrepancies

### Critical Issues

1. **Conflicting Expected Behavior**
   - **Specification states:** "The invalid keys are ignored, the valid keys are updated"
   - **Actual implementation:** Invalid keys cause complete rejection (400 error) - NO settings are updated
   - **Test validates:** The 400 error response (matches implementation, NOT specification)

2. **Request Body Format Mismatch**
   - **Specification example:** Direct JSON values `{"wrong_key": "value", "frame_rate": 30}`
   - **Test implementation:** Nested settings structure `{"settings": {"wrong_key": {...}, "frame_rate": {...}}}`
   - The test uses the settings dictionary format instead of simple key-value pairs

3. **Invalid Key Name Issue**
   - **Test uses:** `"frame_rate"` (described as "typo" in comment)
   - **Issue:** The test comment says "# typo" but doesn't clarify if `frame_rate` is the intended invalid key or if the actual camera setting uses a different name (e.g., `framerate`, `frame_rate_fps`)
   - This ambiguity makes it unclear which key is supposed to be invalid

4. **Missing Precondition Validation**
   - **Specification requires:** Camera app is NOT recording/capturing
   - **Test implementation:** Does NOT verify this precondition

5. **Missing Post-Validation**
   - **Specification requires:** "valid keys are updated"
   - **Test implementation:** Does NOT verify whether the valid key (`frame_rate`) was actually updated
   - Since the endpoint rejects the entire request, this validation would fail if added

## Root Cause Analysis

The fundamental issue is a **specification-implementation mismatch**:

- The **specification** describes a "lenient" behavior: filter out invalid keys, apply valid ones
- The **implementation** follows a "strict" behavior: reject entire request if any invalid key exists
- The **test** validates the strict behavior, contradicting the specification

## Recommendations

### Option 1: Update Specification (Recommended)
If the strict validation is the desired behavior:

1. Update TEST_SPECS.md CAM-SET-003 to state: "Status: 400 Bad Request. The entire request is rejected when invalid keys are present. No settings are updated."
2. Remove the statement "The invalid keys are ignored, the valid keys are updated"
3. Current test implementation would then be compliant

### Option 2: Update Implementation
If lenient behavior is desired:

1. Modify `camera_app.py::save_settings()` to filter invalid keys instead of rejecting:
   ```python
   # Filter to only valid keys
   valid_keys = set(DEFAULT_CAMERA_SETTINGS.keys())
   filtered_settings = {k: v for k, v in new_settings.items() if k in valid_keys}
   ```
2. Update test to verify:
   - Status: 200 OK (or possibly 207 Multi-Status with warnings)
   - Valid keys are updated
   - Invalid keys are ignored
   - Perhaps return a warning in response about ignored keys

### Option 3: Clarify Test Data
Regardless of which option is chosen:

1. Clarify the comment about `frame_rate` being a "typo"
2. Use an unambiguously invalid key name
3. Add precondition check for non-recording state
4. Match the request body format shown in specification

## Impact Assessment
- **Severity:** High - fundamental behavior mismatch
- **User Impact:** Users expecting lenient behavior will see complete rejection instead
- **Test Quality:** Test correctly validates implementation but contradicts specification
