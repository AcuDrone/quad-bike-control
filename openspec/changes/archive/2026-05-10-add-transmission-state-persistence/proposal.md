# Change: Add Transmission State Persistence

## Why
After a power cycle the transmission controller has no memory of where the actuator physically sits, forcing a full autohome sequence on every boot. If the actuator was known to be at a valid gear when power was lost, that expensive and noisy homing run can be skipped entirely.

## What Changes
- On every confirmed gear arrival, save `{gear, encoder_position, valid=true}` to NVS (namespace `transmission`).
- At the start of every gear change, overwrite `valid=false` in NVS so a mid-move power loss is detectable.
- On startup, read the saved state:
  - If `valid=true` **and** the physical gear switch confirms the saved gear → restore encoder to saved position, skip autohome.
  - Otherwise (valid=false, or physical switch disagrees) → run autohome as today.

## Impact
- Affected specs: `vehicle-systems` (Transmission Control System requirement)
- Affected code:
  - `src/TransmissionController.cpp` — `setGear()`, `update()`, new `saveState()` / `loadState()` helpers
  - `include/TransmissionController.h` — new private NVS keys, new methods
  - `src/main.cpp` — startup block that decides autohome vs restore
- NVS keys added to existing `transmission` namespace: `state_valid`, `state_gear`, `state_pos`
- No changes to existing calibration keys (`pos_*`, `calibrated`)
