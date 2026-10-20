## 1. Implementation
- [x] 1.1 `ThrottleController`: `isValidCalibration()` (both endpoints in servo range, `|full - idle| >= THROTTLE_MIN_SPAN_US`) used by `saveCalibration()` and `loadCalibration()`; new error text "Idle and full must differ by >= 100us"
- [x] 1.2 `usToPercent()`: normalise span sign for inverted pairs, result 0..100 between endpoints, clamped outside
- [x] 1.3 Boot and save logs append "(inverted)" when `full < idle`
- [x] 1.4 Audit every idle/full user (`percentToUs`, MAVLink/web throttle, gear-boost PID, sliders, telemetry) for an `idle < full` assumption
- [x] 1.5 `data/index.html`: calibration help text (uk + en + inline fallback) mentions inverted mounting
- [x] 1.6 `include/Constants.h` / `include/ThrottleController.h` comments
- [x] 1.7 `include/Changelog.h`: one line in 1.2.0 (uk + en)

## 2. Verification
- [x] 2.1 `pio run` builds with no new warnings in project code
- [x] 2.2 `openspec validate update-throttle-inverted-calibration --strict` passes
- [ ] 2.3 Bench: calibrate idle=2200/full=800 on the buggy, verify 0/50/100 % positions and that RC throttle moves the right way
