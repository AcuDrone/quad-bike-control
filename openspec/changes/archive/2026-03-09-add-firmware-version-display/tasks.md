# Tasks: add-firmware-version-display

## Phase 1: Backend - Add Version Constant

- [x] **Add firmware version constant to Constants.h** (after line 336, before `#endif`)
  - Add `#define FIRMWARE_VERSION "1.0.0"` (semantic version format)
  - Add comment explaining version format and update procedure
  - Verify: Constant compiles without errors

## Phase 2: Backend - Extend Telemetry Data

- [x] **Update WebPortal::Telemetry struct** (include/WebPortal.h, after line 72)
  - Add `String firmware_version;` field to Telemetry struct
  - Add comment documenting the field
  - Verify: Struct compiles, size increase acceptable

- [x] **Update TelemetryManager::collectTelemetry()** (src/TelemetryManager.cpp, after line 98)
  - Add `telemetry.firmware_version = FIRMWARE_VERSION;`
  - Include Constants.h if not already included
  - Verify: Version constant accessible, telemetry populates correctly

- [x] **Update WebPortal::createTelemetryJSON()** (src/WebPortal.cpp, after line 446)
  - Add `doc["firmware_version"] = telemetry.firmware_version;`
  - Verify: JSON serialization includes version field
  - Verify: WebSocket message size still under limits

## Phase 3: Frontend - Display Version in UI

- [x] **Add version status item to HTML** (data/index.html, after line 317 in status-bar)
  - Add HTML:
    ```html
    <div class="status-item" id="fw-version">
        <strong>Firmware:</strong> <span id="fw-version-text">Loading...</span>
    </div>
    ```
  - Verify: Status bar displays new item, layout not broken

- [x] **Update JavaScript to display version** (data/index.html, in updateTelemetry function after line 652)
  - Add code:
    ```javascript
    // Update firmware version
    const fwVersionElem = document.getElementById('fw-version-text');
    if (fwVersionElem && data.firmware_version) {
        fwVersionElem.textContent = data.firmware_version;
    }
    ```
  - Verify: Version updates when WebSocket connects
  - Verify: Handles missing version gracefully (shows "Loading...")

## Phase 4: Testing

- [x] **Test firmware upload and display**
  - Upload firmware to ESP32
  - Upload SPIFFS filesystem (data/ folder)
  - Connect to web portal
  - Verify: Firmware version displays correctly in status bar

- [x] **Test version constant change**
  - Change FIRMWARE_VERSION to "1.0.1"
  - Rebuild and upload
  - Verify: New version displays in portal

- [x] **Test UI responsiveness**
  - Check status bar layout on desktop browser
  - Check status bar layout on mobile (responsive design)
  - Verify: Version text doesn't overflow or break layout

- [x] **Test WebSocket reconnection**
  - Disconnect and reconnect WebSocket
  - Verify: Version persists and updates correctly

## Dependencies

- All tasks are sequential - complete Phase 1 before Phase 2, etc.
- Phase 3 (frontend) can be started in parallel with Phase 2 if desired
- Phase 4 (testing) requires all previous phases complete

## Success Criteria

- ✅ Firmware version displays in web portal status bar
- ✅ Version updates automatically on WebSocket connection
- ✅ UI layout remains clean and responsive
- ✅ Version constant is easy to find and update in Constants.h
- ✅ Implementation follows existing telemetry patterns
