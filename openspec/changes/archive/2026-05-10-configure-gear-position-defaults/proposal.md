# Change: Configure Gear Position Defaults

## Why
The four default gear encoder positions (`TRANS_POSITION_REVERSE/NEUTRAL/LOW/HIGH`) are compile-time `#define` constants. Adjusting them requires a firmware recompile and reflash. They should instead be stored in NVS so they survive reflashing with default values and can be tuned from the web portal without code changes. The web portal also needs to display the live encoder count near the gear controls so the user can drive to a position and read the exact count to enter as a default.

## What Changes
- Add NVS-backed default positions (`def_reverse`, `def_neutral`, `def_low`, `def_high`) in the existing `transmission` namespace. Compile-time `#define` values remain as the factory fallback if NVS keys are absent.
- `TransmissionController::getGearPosition()` returns the NVS default when not auto-calibrated, maintaining the existing three-level priority: auto-calibrated → user-configured NVS default → compile-time factory default.
- New `set_gear_default` web command saves a single gear's default position to NVS and updates it immediately in RAM.
- Telemetry gains a `gear_defaults` object (`{R, N, L, H}` encoder counts) so the web UI can show current values without a separate request.
- The Manual Control section of the web portal shows the live encoder count next to the gear buttons, and a collapsible "Gear Position Defaults" panel lets the user view and edit all four values.

## Impact
- Affected specs: `vehicle-systems` (default position loading), `web-control` (new command), `web-telemetry` (new telemetry field + UI)
- Affected code:
  - `include/TransmissionController.h` — new private fields `defaultPositions_[4]`, new private methods `loadDefaultPositions()`, `saveDefaultPosition(Gear, int32_t)`, new public `setDefaultPosition(Gear, int32_t)`
  - `src/TransmissionController.cpp` — implement the above; update `getGearPosition()` to use `defaultPositions_` when uncalibrated
  - `src/main.cpp` — call `loadDefaultPositions()` after NVS init, alongside `loadCalibration()`
  - `include/VehicleController.h` / `src/VehicleController.cpp` — handle `set_gear_default` command
  - `src/TelemetryManager.cpp` / `src/WebPortal.cpp` — add `gear_defaults` to telemetry JSON
  - `data/index.html` — show encoder count near gear buttons; add gear defaults editor panel
- NVS keys added: `def_reverse`, `def_neutral`, `def_low`, `def_high` in `transmission` namespace
- No changes to existing calibration keys (`pos_*`, `calibrated`, `state_*`)
