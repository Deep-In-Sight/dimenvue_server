# Test Audit Report: MAP-SET-004

**Test ID:** MAP-SET-004
**Test Name:** Invalid Setting
**Implementation Location:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_mapping_api.py#L831`
**Audit Date:** 2025-12-18

---

## Compliance Status: **COMPLIANT**

---

## Specification Requirements

### Preconditions
- **Given:** Mapping is idle

### Action
- **Endpoint:** `PUT /mappingApp/settings`
- **Request Body:**
  ```json
  {
    "file_format": "OBJ",
    "preview_voxel_size": 5
  }
  ```

### Expected Behavior
- **Status Code:** 400 Bad Request
- **Behavior:** Invalid values ignored, valid value updated

---

## Implementation Analysis

### What the Implementation Does

1. **Precondition Setup:**
   - Patches `mapping_app.GetInitStatus` to return "UNKNOWN" (mapping is idle)
   - Retrieves initial settings via `GET /mappingApp/settings`

2. **Request Execution:**
   - Sends `PUT /mappingApp/settings` with exactly the specified payload:
     ```json
     {
       "file_format": "OBJ",
       "preview_voxel_size": 5
     }
     ```
   - "OBJ" is invalid (not in valid options: ["PLY", "PCD", "LAS", "LAZ"])
   - 5 is valid (in valid options: [5, 10, 15])

3. **Assertions:**
   - ✓ Verifies status code is 400 Bad Request
   - ✓ Verifies `file_format` remains unchanged (invalid value ignored)
   - ✓ Verifies `preview_voxel_size` is updated to index 0 (value 5 applied)

---

## Gaps and Discrepancies

**None identified.** The implementation fully matches the specification.

---

## Detailed Comparison

| Aspect | Specification | Implementation | Match |
|--------|--------------|----------------|-------|
| Precondition | Mapping is idle | Mocks `GetInitStatus` → "UNKNOWN" | ✓ |
| HTTP Method | PUT | PUT | ✓ |
| Endpoint | `/mappingApp/settings` | `/mappingApp/settings` | ✓ |
| Request - `file_format` | "OBJ" (invalid) | "OBJ" (invalid) | ✓ |
| Request - `preview_voxel_size` | 5 (valid) | 5 (valid) | ✓ |
| Expected Status | 400 Bad Request | Asserts 400 | ✓ |
| Invalid Value Behavior | Ignored | Asserts `file_format` unchanged | ✓ |
| Valid Value Behavior | Updated | Asserts `preview_voxel_size` = 0 (value 5) | ✓ |

---

## Recommendations

None. The test implementation is thorough and correctly validates:
- Mixed valid/invalid input handling
- Partial update semantics (valid values applied, invalid values rejected)
- Proper error response (400 status code)
- State verification after attempted update

The test includes clear comments explaining the validation logic and expected behavior.
