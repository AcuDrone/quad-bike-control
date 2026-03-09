# Tasks: enhance-webportal-telemetry-and-controls

## Phase 1: Backend - Extend Telemetry Data Collection

- [x] **Extend WebPortal::Telemetry struct** (include/WebPortal.h)
  - Add SBUS channel array: `uint16_t sbus_channels[16]`
  - Add SBUS signal quality fields: `float sbus_frame_rate`, `float sbus_error_rate`, `uint32_t sbus_signal_age`
  - Add gear switching state: `bool gear_switching`
  - Verify: Compile succeeds, struct size reasonable

- [x] **Update TelemetryManager to collect SBUS channels** (src/TelemetryManager.cpp)
  - Call `sbusInput_.getRawChannels()` to get all 16 channel values
  - Call `sbusInput_.getSignalQuality()` to get quality metrics
  - Populate SBUS fields in telemetry struct
  - Verify: Serial print telemetry shows SBUS data when signal present

- [x] **Update TelemetryManager to collect gear switching state** (src/TelemetryManager.cpp)
  - Check `vehicleController_.getTransmission().isPositionControlActive()` to detect switching
  - Set `gear_switching` boolean in telemetry
  - Verify: Serial print shows switching=true during gear changes, false when stable

- [x] **Update WebPortal JSON serialization** (src/WebPortal.cpp)
  - Add SBUS channels array to JSON: `"sbus_channels": [...]`
  - Add SBUS quality metrics: `"sbus_frame_rate"`, `"sbus_error_rate"`, `"sbus_signal_age"`
  - Add gear switching: `"gear_switching": true/false`
  - Verify: Check JSON output in browser dev console, all fields present

## Phase 2: Backend - Add Brake Control Command

- [x] **Add brake control handler to VehicleController** (src/VehicleController.cpp)
  - Add `processBrakeCommand(float value, WebPortal& webPortal)` method
  - Validate value range 0-100
  - Call brake actuator with percentage (convert to position if needed)
  - Send success/error response via WebSocket
  - Verify: Send `{"cmd": "set_brake", "value": 50}` via WebSocket, brake actuator responds

- [x] **Wire up brake command in processWebCommand()** (src/VehicleController.cpp)
  - Add case for "set_brake" command
  - Parse value from JSON
  - Call `processBrakeCommand()`
  - Enforce input source check (reject if SBUS active)
  - Verify: Brake command accepted when WEB active, rejected when SBUS active

## Phase 3: Frontend - Add SBUS Channel Display

- [x] **Add SBUS telemetry card to HTML** (data/index.html)
  - Create new card "📡 S-bus Channels" in main-grid
  - Add table or grid showing all 16 channels
  - Display raw value (µs) and normalized percentage for key channels (Ch1-4)
  - Display signal quality: frame rate, error rate, signal age
  - Style: match existing card design, compact layout
  - Verify: Card renders correctly, empty/zero values initially

- [x] **Add JavaScript to update SBUS display** (data/index.html)
  - In `updateTelemetry()`, parse `sbus_channels` array
  - Update channel value displays
  - Update signal quality metrics
  - Highlight mapped channels (Ch1: steering, Ch2: throttle, Ch3: gear, Ch4: brake)
  - Verify: SBUS values update in real-time when signal present

## Phase 4: Frontend - Add CAN Bus Data Display

- [x] **Add CAN telemetry card to HTML** (data/index.html)
  - Create new card "🚗 Vehicle Data (CAN)" in main-grid
  - Add telemetry items for: Engine RPM, Speed, Coolant Temp, Oil Temp, Throttle Position
  - Add CAN status indicator (connected/disconnected)
  - Add data age display (time since last update)
  - Style: match existing telemetry-grid layout
  - Verify: Card renders correctly, shows placeholder values

- [x] **Add JavaScript to update CAN display** (data/index.html)
  - In `updateTelemetry()`, update CAN data fields from telemetry
  - Color-code temperature warnings:
    - Coolant: green <90°C, yellow 90-105°C, red >105°C
    - Oil: green <110°C, yellow 110-130°C, red >130°C
  - Show "disconnected" state when `can_status !== "connected"`
  - Format data age as human-readable (e.g., "2s ago")
  - Verify: CAN values update correctly, color coding works, disconnected state shows

## Phase 5: Frontend - Add Brake Control Slider

- [x] **Add brake slider to Manual Control card** (data/index.html)
  - Add slider-container for brake (after throttle slider)
  - Range: 0-100, default 0
  - Label: "Brake" with percentage display
  - Style: match existing steering/throttle sliders
  - Verify: Slider renders correctly, disabled by default

- [x] **Add JavaScript for brake slider** (data/index.html)
  - Add event listeners for brake slider (mousedown/up, touchstart/end)
  - Add input event handler with rate limiting (100ms debounce)
  - Send `{"cmd": "set_brake", "value": <0-100>}` via WebSocket
  - Update brake-value display text
  - Enable/disable slider based on input source (same as steering/throttle)
  - Verify: Brake commands sent on slider change, rate limited, disabled when SBUS active

## Phase 6: Frontend - Add Gear Transition Indicator

- [x] **Add gear switching indicator to HTML** (data/index.html)
  - Add `<div id="gear-switching-indicator">` in gear display area
  - Style: overlay or inline indicator with animation (spinner or loading dots)
  - Show target gear: "⚙️ Switching to H..."
  - Hide by default (`display: none`)
  - Verify: Indicator element present but hidden

- [x] **Add JavaScript to show gear transition state** (data/index.html)
  - In `updateTelemetry()`, check `gear_switching` boolean
  - If true: show indicator with target gear name
  - If false: hide indicator
  - Prevent gear display from flashing during transition (keep showing last stable or target gear)
  - Verify: Indicator shows during gear changes, hides when stable

## Phase 7: Testing and Validation

- [ ] **Test SBUS display with live signal**
  - Connect SBUS receiver with ArduPilot
  - Verify all 16 channels update in real-time
  - Verify signal quality metrics are accurate
  - Verify mapped channels highlighted correctly

- [ ] **Test CAN display with vehicle data**
  - Connect CAN bus to vehicle
  - Verify RPM, speed, temps display correctly
  - Verify color coding for temperature warnings
  - Test disconnected state (unplug CAN)

- [ ] **Test brake control from web**
  - Move brake slider, verify actuator responds
  - Verify rate limiting works (no command flooding)
  - Verify disabled when SBUS active
  - Verify WebSocket response shown to user

- [ ] **Test gear transition indicator**
  - Send gear change command
  - Verify "switching" indicator appears during transition
  - Verify indicator disappears when gear reached
  - Verify no neutral flashing during transition

- [ ] **Integration test - All features together**
  - Load web portal with all new cards visible
  - Verify telemetry updates at 5 Hz
  - Verify WebSocket message size <1KB
  - Test with 3-5 simultaneous clients
  - Verify no performance degradation in control loop

## Dependencies
- Phase 2 can start in parallel with Phase 1
- Phase 3, 4, 5, 6 depend on Phase 1 completion (telemetry struct extended)
- Phase 7 requires all phases complete
