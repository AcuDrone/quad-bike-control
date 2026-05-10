## 1. NVS Default Position Helpers (TransmissionController)
- [x] 1.1 Add `int32_t defaultPositions_[4]` private field, initialized from compile-time `#define` values in constructor
- [x] 1.2 Add private `loadDefaultPositions()` — reads `def_reverse/neutral/low/high` from NVS `transmission` namespace; leaves array at `#define` values if keys absent
- [x] 1.3 Add private `saveDefaultPosition(Gear gear, int32_t position)` — writes single key to NVS and updates `defaultPositions_[gear]` in RAM
- [x] 1.4 Add public `setDefaultPosition(Gear gear, int32_t position)` — validates range, calls `saveDefaultPosition()`; returns false on invalid gear or unreasonable value
- [x] 1.5 Update `getGearPosition()` — when `isCalibrated_` is false, return `defaultPositions_[gear]` instead of the inline `switch` over compile-time `#define` values

## 2. Startup Loading
- [x] 2.1 In `main.cpp`, call `transmissionActuator.loadDefaultPositions()` after NVS init, alongside the existing `loadCalibration()` call

## 3. Web Command: set_gear_default
- [x] 3.1 In `VehicleController::processWebCommand()`, add handling for `cmd == "set_gear_default"` — always available (not restricted to WEB input source, same as calibration commands)
- [x] 3.2 Add `processSetGearDefaultCommand(const String& gear, int32_t position, WebPortal& webPortal)` — maps gear string to enum, calls `transmission_.setDefaultPosition()`, sends JSON response `{"ok": true, "gear": "X", "position": N}`

## 4. Telemetry: gear_defaults field
- [x] 4.1 Add `gear_defaults` struct/fields to `TelemetryData` in `TelemetryManager` — four int32 fields (R, N, L, H)
- [x] 4.2 In `TelemetryManager::update()`, populate from `vehicleController_.getTransmission().getDefaultPosition(gear)` for each gear
- [x] 4.3 In `WebPortal::broadcastTelemetry()`, serialize `gear_defaults` as `{"R": N, "N": N, "L": N, "H": N}` in the JSON document

## 5. Web UI
- [x] 5.1 Add live encoder count display next to gear buttons in Manual Control section (reads `data.hall_position` already available in telemetry)
- [x] 5.2 Add collapsible "Gear Position Defaults" panel with four number inputs (R, N, L, H) and a Save button per gear
- [x] 5.3 On telemetry update, populate the input fields with `data.gear_defaults.R/N/L/H` if the user has not started editing
- [x] 5.4 On Save, send `{"cmd": "set_gear_default", "strValue": "R", "floatValue": 725}` via WebSocket

## 6. Validation
- [ ] 6.1 Confirm defaults load correctly on first boot (no NVS keys) — factory values used
- [ ] 6.2 Set a default from the web portal, power-cycle, confirm the new value is used
- [ ] 6.3 Run auto-calibration — confirm calibrated positions override the defaults without changing `def_*` NVS keys
- [ ] 6.4 Clear calibration — confirm system falls back to user-configured NVS defaults (not factory values)
