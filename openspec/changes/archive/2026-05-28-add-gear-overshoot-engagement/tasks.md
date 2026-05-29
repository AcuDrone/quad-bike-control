---
id: add-gear-overshoot-engagement
title: Add Gear Overshoot Engagement — Tasks
---

## Tasks

- [x] 1. Add `TRANS_GEAR_OVERSHOOT` constant to `include/Constants.h` (default 40 encoder counts) in the Transmission Movement Parameters section
- [x] 2. Add `GearChangePhase` private enum (`NONE`, `OVERSHOOT`, `RETURN`) and member `gearChangePhase_` to `TransmissionController` private section in `include/TransmissionController.h`
- [x] 3. Add `finalGearPosition_` (`int32_t`) private member to hold the true target while Phase 1 is active
- [x] 4. Initialise `gearChangePhase_(GearChangePhase::NONE)` and `finalGearPosition_(0)` in `TransmissionController` constructor
- [x] 5. Update `setGear()` in `src/TransmissionController.cpp`:
  - Compute `direction = sign(targetPosition − getPosition())`
  - If already within `TRANS_POSITION_TOLERANCE`: skip overshoot (phase stays `NONE`, call `moveToPosition(targetPosition)` as before)
  - Otherwise: store `finalGearPosition_ = targetPosition`, set `gearChangePhase_ = OVERSHOOT`, call `moveToPosition(targetPosition + direction × TRANS_GEAR_OVERSHOOT)`
- [x] 6. Update `update()` in `src/TransmissionController.cpp` to detect Phase 1 completion:
  - When `gearChangePhase_ == OVERSHOOT` and `!isPositionControlActive()`: log overshoot reached, set `gearChangePhase_ = RETURN`, call `moveToPosition(finalGearPosition_)`
- [x] 7. Ensure physical-switch arrival confirmation (`physicalGear == targetGear_`) is only acted on when `gearChangePhase_ == NONE` or `gearChangePhase_ == RETURN` (not during `OVERSHOOT`)
- [x] 8. Reset `gearChangePhase_ = NONE` on gear-change completion (physical switch confirms or position control stops)
- [x] 9. Build and flash; verify via serial log that overshoot and return phases appear for each gear change
- [x] 10. Bench-test: confirm each gear (R, N, L, H) mechanically clicks into full engagement; adjust `TRANS_GEAR_OVERSHOOT` if needed
