# Test Audit Report: CAM-REC-002

## Test Information
- **Test ID:** CAM-REC-002
- **Test Name:** Stop Recording
- **Implementation:** `/home/linh/ros2_ws/dimenvue_server/tests/api/test_camera_api.py` (lines 374-410)
- **Audit Date:** 2025-12-18

## Compliance Status
**NON-COMPLIANT**

## Specification Requirements

The test specification requires verification of:

1. **Given:** Recording is in progress for at least 3 seconds
2. **Action:** `PUT /cameraApp/recordStop`
3. **Expects:**
   - Status: 200 OK
   - 3 video files created (left, front, right)
   - Files are valid MP4/AVI format
   - Catalog contains new `Video` entry
   - Video duration matches recording time (±1 second tolerance)

## Implementation Analysis

### What the Test Implements

The test implementation (`test_stop_recording`) performs the following:

```python
async def test_stop_recording(self, async_client, test_app):
    import server_app

    # Initialize camera
    await async_client.put("/cameraApp/init")

    # Create record_tmp directory with dummy files (simulating GStreamer output)
    record_tmp = Path(server_app.cameraApp.record_tmp_path)
    record_tmp.mkdir(parents=True, exist_ok=True)
    (record_tmp / "left.mp4").write_bytes(b"fake mp4 data")
    (record_tmp / "front.mp4").write_bytes(b"fake mp4 data")
    (record_tmp / "right.mp4").write_bytes(b"fake mp4 data")

    # Mock catalog.add_item and thumbnail generation
    mock_add_item = MagicMock(return_value="test-uuid")
    with patch.object(server_app.cameraApp.catalog, 'add_item', mock_add_item), \
         patch('camera_app.gen_thumbnail_factory'):

        # Start recording
        await async_client.put("/cameraApp/recordStart")

        # Wait a bit (simulating 3 seconds)
        await asyncio.sleep(0.2)

        # Stop recording
        response = await async_client.put("/cameraApp/recordStop")
        assert response.status_code == 200
        assert "status" in response.json()

        # Verify state returns to ready
        state_resp = await async_client.get("/cameraApp/state")
        assert state_resp.json()["state"] == "ready"

        # Verify catalog was updated
        assert mock_add_item.called
```

### Verification Coverage

| Requirement | Status | Notes |
|------------|--------|-------|
| Given: Recording for at least 3 seconds | ❌ NOT VERIFIED | Only waits 0.2 seconds, not 3 seconds |
| Action: PUT /cameraApp/recordStop | ✅ VERIFIED | Calls the endpoint |
| Expects: Status 200 OK | ✅ VERIFIED | Asserts status_code == 200 |
| Expects: 3 video files created | ❌ NOT VERIFIED | Pre-creates fake files, doesn't verify actual creation |
| Expects: Files are valid MP4/AVI format | ❌ NOT VERIFIED | Uses fake data, no format validation |
| Expects: Catalog contains new Video entry | ⚠️ PARTIAL | Only checks mock was called, not actual entry content |
| Expects: Video duration matches recording time | ❌ NOT VERIFIED | No duration validation whatsoever |

## Gaps and Discrepancies

### 1. Recording Duration Requirement Not Met
**Severity:** Critical

The specification explicitly states "Recording is in progress for at least 3 seconds" as a precondition, but the test only waits 0.2 seconds before stopping.

**Current Behavior:**
```python
await asyncio.sleep(0.2)  # Only 200ms, not 3000ms
```

**Expected Behavior:**
```python
await asyncio.sleep(3.0)  # Should wait at least 3 seconds
```

**Gap:** The test does not verify that the recording duration requirement is met. This is critical because video duration validation (specified requirement #5) requires actual recording time.

### 2. Video Files Not Actually Created or Verified
**Severity:** Critical

The test pre-creates fake MP4 files before starting the recording, defeating the purpose of verifying that the `recordStop` endpoint actually creates the video files.

**Current Behavior:**
- Test manually creates `left.mp4`, `front.mp4`, `right.mp4` with fake data (`b"fake mp4 data"`)
- Files exist before `recordStop` is called
- No verification that `recordStop` actually creates or processes these files

**Expected Behavior:**
- Let GStreamer create the actual video files during recording
- After `recordStop`, verify that 3 files exist at expected paths
- Verify files were created/modified after the stop call

**Gap:** The test does not verify the actual file creation behavior of `recordStop`. The implementation calls `_do_record_stop()` which:
1. Sends EOS to 3 recording pipelines (precord0, precord1, precord2)
2. Stops the pipelines
3. References files at `{record_tmp_path}/{left|front|right}.{format}`

The test should verify these files are created by the actual recording process, not pre-created as fixtures.

### 3. Video Format Validation Missing
**Severity:** Critical

The specification requires verification that "Files are valid MP4/AVI format", but the test uses fake data that is not valid MP4/AVI.

**Current Behavior:**
- Files contain `b"fake mp4 data"` which is not a valid video format
- No format validation is performed

**Expected Behavior:**
- Files should be actual valid video files created by GStreamer
- Test should verify file format using one of:
  - File magic number/header inspection
  - `ffprobe` or similar tool to validate container format
  - Attempt to read video metadata (codec, duration, resolution)

**Gap:** No video format validation. The implementation uses the `record_format` setting which defaults to MP4 but can be AVI. Test should verify the actual format matches the setting.

### 4. Catalog Entry Not Properly Verified
**Severity:** High

The test mocks `catalog.add_item` and only checks that it was called, without verifying:
- The correct parameters were passed
- The entry type is "Video"
- The entry was actually added to the catalog

**Current Behavior:**
```python
mock_add_item = MagicMock(return_value="test-uuid")
# ...
assert mock_add_item.called
```

**Expected Behavior:**
```python
# Verify add_item was called with correct parameters
mock_add_item.assert_called_once_with("Video", record_tmp_path)
# Or without mocking, verify the catalog actually contains the entry
```

**Gap:** The test should verify:
1. `add_item` was called with `item_type="Video"`
2. `add_item` was called with the correct `content_path` (record_tmp_path)
3. The catalog entry has the expected structure and metadata

### 5. Video Duration Not Validated
**Severity:** Critical

The specification explicitly requires "Video duration matches recording time (±1 second tolerance)" but the test performs no duration validation whatsoever.

**Current Behavior:**
- No duration checking at all
- Fake files have no duration metadata

**Expected Behavior:**
- After `recordStop`, inspect the created video files
- Extract video duration using `ffprobe` or similar
- Verify duration is approximately 3 seconds (±1 second: 2-4 seconds acceptable)

**Gap:** Complete absence of duration validation. This requires:
1. Actual video files created by GStreamer
2. Video metadata extraction
3. Duration comparison against expected recording time

### 6. Response Body Validation Incorrect
**Severity:** Medium

The test asserts `"status" in response.json()` but the actual endpoint implementation does not return a status field in the response body.

**Current Behavior:**
```python
assert "status" in response.json()
```

**Actual Endpoint Behavior:**
According to `server_app.py`, the `/cameraApp/recordStop` endpoint likely returns an empty 200 response or a simple confirmation, not a JSON with "status" field.

**Gap:** The test assertion may be incorrect. Need to verify what the endpoint actually returns and update the assertion accordingly, or update the endpoint to match the test expectation.

### 7. Mock Patterns Reduce Test Value
**Severity:** High

The heavy use of mocking (pre-created fake files, mocked catalog, mocked thumbnail generation) means the test is not verifying the actual integration behavior of the recordStop functionality.

**Current Behavior:**
- Fake video files instead of real GStreamer output
- Mocked catalog instead of actual catalog integration
- Mocked thumbnail generation

**Expected Behavior:**
For an integration test, the test should:
- Use real (or more realistic) video files from actual GStreamer recording
- Verify actual catalog integration (or use a test catalog)
- Allow thumbnail generation to run (or verify it's called with correct parameters)

**Gap:** The test behaves more like a unit test than an integration test, missing the opportunity to verify the actual end-to-end flow of stopping a recording.

## Recommendations

### Critical Priority (Must Fix for Compliance)

1. **Record for Required Duration**
   ```python
   # Wait at least 3 seconds as per specification
   await asyncio.sleep(3.0)
   ```

2. **Verify Actual Video File Creation**
   - Remove pre-creation of fake video files
   - Let GStreamer actually record for 3 seconds
   - After `recordStop`, verify files exist and were created/modified recently
   ```python
   # After recordStop
   for sensor in ["left", "front", "right"]:
       file_path = record_tmp / f"{sensor}.mp4"
       assert file_path.exists(), f"{sensor}.mp4 not created"
       assert file_path.stat().st_size > 0, f"{sensor}.mp4 is empty"
   ```

3. **Add Video Format Validation**
   - Use `ffprobe` or Python video libraries to verify format
   ```python
   import subprocess
   result = subprocess.run(
       ["ffprobe", "-v", "error", "-show_format", str(file_path)],
       capture_output=True, text=True
   )
   assert result.returncode == 0, f"Invalid video format for {file_path}"
   ```

4. **Add Video Duration Validation**
   - Extract duration from video metadata
   - Verify it matches recording time ±1 second
   ```python
   result = subprocess.run(
       ["ffprobe", "-v", "error", "-show_entries",
        "format=duration", "-of", "default=noprint_wrappers=1:nokey=1",
        str(file_path)],
       capture_output=True, text=True
   )
   duration = float(result.stdout.strip())
   assert 2.0 <= duration <= 4.0, f"Duration {duration}s not in range [2,4]s"
   ```

### High Priority

5. **Verify Catalog Entry Parameters**
   ```python
   mock_add_item.assert_called_once_with("Video", str(record_tmp))
   ```

6. **Fix Response Body Assertion**
   - Verify what the endpoint actually returns
   - Update assertion to match actual behavior
   - Or update endpoint to return expected response structure

### Medium Priority

7. **Consider Reducing Mocking for Integration Value**
   - Use actual catalog with test data path
   - Allow thumbnail generation to run (or verify parameters)
   - This better tests the actual integration behavior

8. **Add File Count Verification**
   ```python
   # Verify exactly 3 video files exist
   video_files = list(record_tmp.glob("*.mp4"))
   assert len(video_files) == 3, f"Expected 3 files, found {len(video_files)}"
   ```

## Conclusion

The test is **NON-COMPLIANT** with the specification. It fails to verify multiple critical requirements:

1. **Does not record for the required 3 seconds** (only 0.2s)
2. **Does not verify actual video file creation** (uses pre-created fakes)
3. **Does not validate video format** (fake data is not valid MP4/AVI)
4. **Does not verify video duration** (no duration checking at all)
5. **Does not properly verify catalog integration** (only checks mock was called)

The test appears to be a preliminary unit test with heavy mocking rather than a true integration test that verifies the complete recordStop flow. To achieve compliance, the test needs to:
- Record for at least 3 seconds
- Let GStreamer create actual video files
- Validate video format and duration
- Properly verify catalog integration

These changes would transform it from a mocked unit test into a proper integration test that verifies the specification requirements.
