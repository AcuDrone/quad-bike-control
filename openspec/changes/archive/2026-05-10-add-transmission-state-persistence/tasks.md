## 1. NVS State Helpers
- [x] 1.1 Add private `saveState(Gear gear, int32_t position, bool valid)` to `TransmissionController` — writes `state_valid`, `state_gear`, `state_pos` to NVS namespace `transmission`
- [x] 1.2 Add private `loadState(Gear& gear, int32_t& position, bool& valid)` — reads same keys, returns false if keys absent

## 2. Mark In-Transit on Gear Change Start
- [x] 2.1 In `setGear()`, call `saveState(gear, getPosition(), false)` before `moveToPosition()`

## 3. Mark Arrived on Gear Confirmation
- [x] 3.1 In `update()`, at the point where the physical switch confirms `targetGear_` is reached, call `saveState(targetGear_, getPosition(), true)`

## 4. Startup Restore Logic
- [x] 4.1 Add public `bool restoreStateIfValid()` to `TransmissionController`:
  - Load state via `loadState()`
  - If `valid=false` → return false (caller runs autohome)
  - Read physical gear switch
  - If physical gear matches saved gear → call `transmissionEncoder.setPosition(savedPosition)`, return true
  - Else → return false (caller runs autohome)
- [x] 4.2 In `main.cpp` startup block, replace the "TEMPORARY: Skip calibration/homing" comment with:
  - Call `transmissionActuator.restoreStateIfValid()`
  - If returns true → log "Restored transmission state, skipping autohome"
  - If returns false → run `autoHome()` as before

## 5. Validation
- [ ] 5.1 Bench test: power-cycle after reaching a gear — confirm autohome is skipped and gear is correct
- [ ] 5.2 Bench test: power-cycle mid-gear-change (state_valid=false) — confirm autohome runs
- [ ] 5.3 Bench test: manually move actuator while off, confirm physical-switch mismatch triggers autohome
