## 1. Pure logic and tests
- [x] 1.1 `include/TransmissionLaw.h` (header-only, no Arduino / `Constants.h`): gear enum mirroring `TransmissionController::Gear` values (static_assert in `TransmissionController.cpp`); `chooseTransmissionInit(sensorGear, gearPositions[4], nvsFound, nvsValid, nvsGear, nvsPct)` → `{source SENSOR|NVS|STEP_TO_NEUTRAL, gear, pct, direction}`; `TransmissionResync { candidate, count; bool step(sensorGear, targetGear, needed); }`.
- [x] 1.2 `test/test_transmission_sync/test_main.cpp`: sensor wins over valid NVS; sensor UNKNOWN + valid NVS → NVS; invalid / missing / UNKNOWN-gear / out-of-range-pct NVS → STEP_TO_NEUTRAL; step direction: hint from the R side (`state_pct` < N) → +1, from the H/L side → −1, no NVS record → +1, bogus `state_pct` (NaN / < 0 / > 100) → +1, `state_pct` == N → sign(N − position of `state_gear`); resync fires on the 2nd consecutive identical mismatch, not on the 1st; resets on a match, on UNKNOWN and on a different sensor gear; no resync when sensor == target.
- [x] 1.3 `platformio.ini`: add `test_transmission_sync` on its own indented `test_filter` line and to the `[env:native]` comment block; `pio test -e native` passes.

## 2. Firmware
- [x] 2.1 `Constants.h`: `TRANS_RESYNC_CONFIRM_CHECKS` (2), `TRANS_INIT_SETTLE_MS` (1500, comment: max 20 % + 1 % overshoot ≈ 0.75 s, doubled); bump `FIRMWARE_VERSION` to "1.5.14".
- [x] 2.2 `TransmissionController`: `targetGear_` / `fromGear_` start `GEAR_UNKNOWN`; `isInitialised()`; `initialise()` replaces `restoreStateIfValid()`: SENSOR → direct move + save valid + "Initialised from sensor: X"; NVS → direct move to `state_pct` + "Initialised from NVS: X at P%".
- [x] 2.3 STEP_TO_NEUTRAL: log "Init: sensor unknown, stepping to NEUTRAL"; start a normal NEUTRAL step with the law's overshoot direction, settle `TRANS_INIT_SETTLE_MS` for the overshoot move, `fromGear_` UNKNOWN, mark the state invalid without overwriting `state_pct`; `initStepActive_` flag. On RETURN completion: sensor N → normal; sensor UNKNOWN → "Init: sensor silent, assuming NEUTRAL"; save valid in both cases; clear the flag.
- [x] 2.4 `setGear()` returns false while not initialised (replaces routing through N for an unknown current gear); during the init step it queues as for any active step.
- [x] 2.5 Idle mismatch check: feed `TransmissionResync`; on fire set `targetGear_`, `saveState(gear, getCurrentServoPct(), true)`, log "Resynced from sensor: target=X -> Y"; keep the first-detection MISMATCH log; reset the counter when a change starts.
- [x] 2.6 `VehicleController::update()`: call `initialise()` once when `!isInitialised() && isEngineRunning()`; remove `transmissionInitialized_` and the `setGear(NEUTRAL)` fallback.
- [x] 2.7 `processGearCommand()`: if not initialised call `initialise()` first, then `setGear()` as today (replies unchanged). MAVLink path unchanged.
- [x] 2.8 `shouldClipThrottle()` true for a `GEAR_UNKNOWN` target; `getTargetGearString()` / `getFromGearString()` return "?" for UNKNOWN.

## 3. Docs, bench
- [x] 3.1 `Changelog.h` 1.5.14 bullet (uk + en).
- [x] 3.2 Build `esp32-s3-devkitc-1` clean.
- [ ] 3.3 Bench: lever in L, sensor working, power off mid-shift, power on, start engine → log "Initialised from sensor: L", web shows L, no servo jump to N.
- [ ] 3.4 Bench: sensor connector unplugged, power off mid-shift (state_valid=false), power on, start engine → log "Init: sensor unknown, stepping to NEUTRAL", overshoot in the direction of the hint, lever ends in N (sensor shows N once plugged back, or "sensor silent, assuming NEUTRAL" without it), total ≈ 1.5 + 2 + 0.3 s; afterwards a web and a MAVLink gear command both shift normally.
- [ ] 3.5 Bench: engine off, sensor unplugged, NVS invalid, web `set_gear` L → init step to N first, reply "Moving to LOW gear", then N→H→L runs from the queue.
- [ ] 3.6 Bench: engine off, web `set_gear` N from L with the sensor working → log "Initialised from sensor: L" first, then the shift L→H→N starts from the L position.
- [ ] 3.7 Bench: move the servo to L with `move_to_position` while target is N → after ≈1 s "Resynced from sensor: target=NEUTRAL -> LOW", servo does not move.
