## 1. Pure law
- [x] 1.1 `include/SteerPositionLaw.h`: add `int16_t boostDecay` to `SteerPosParams` (after `boostStep`); boost update per check — stalled → `+boostStep` (cap unchanged), moving → `−boostDecay`, never below 0; duty = `clamp(max(|err| × kp, floor) + boost, floor, maxDuty)` signed toward the target; update the header comment (steps 5 and 7, change id).
- [x] 1.2 Boost hold band (design D7): `SteerPosParams.boostHoldErr`; on a moving check the boost is held while `|err| > boostHoldErr` and decays by `boostDecay` inside it; a stalled check adds anywhere; `STEER_BOOST_HOLD_ERR` 20 (2.0°) in `Constants.h`, wired into `POS_PARAMS`; `STEER_BOOST_DECAY` 10 → 30 (fast in-band shed, bench D7); native tests (held far, decays inside, stalled inside adds; the parameter block mirrors decay 30, the gentle-decay test uses a decay-10 variant); changelog 1.5.20 mentions the held boost and floors 85.

## 2. Firmware
- [x] 2.1 `include/Constants.h` (quad values on `master`): `STEER_PWM_MIN_DUTY` 80 → 90, `STEER_PWM_TRACK_MIN_DUTY` 70 → 90 (bench-tune), new `STEER_BOOST_DECAY` 10, `STEER_POSITION_TOLERANCE` 4 → 2, new `STEER_PARK_TRIGGER_TOL` 4; comments with the 2026-11-05 measurements (plateaus at 70, broke free at 89–107, parked 0.3–0.4° short) and the new formula; `static_assert(STEER_PWM_MIN_DUTY >= STEER_PWM_TRACK_MIN_DUTY)`; `STEER_RESTART_TOLERANCE` stays 10.
- [x] 2.2 `src/SteeringController.cpp`: `POS_PARAMS` gets `STEER_BOOST_DECAY`; `PARK_DEFAULTS.triggerTol` = `STEER_PARK_TRIGGER_TOL` (not `STEER_POSITION_TOLERANCE`). Range guard, stall latch, park correction and jog paths unchanged.
- [x] 2.3 `include/SteeringController.h`: doc comments for the new formula, the 0.2° window and the 0.4° park-correction trigger.
- [x] 2.4 `include/Changelog.h` 1.5.20: uk "Кермо: прибрано ривки на повороті (буст додається поверх мінімальної потужності й спадає плавно, мінімальна потужність 90) і недоїзд до центру на 0,3–0,4° (зупинка в межах 0,2°)"; en "Steering: no stop-go jerks on a turn (the anti-stall boost adds on top of the duty floor and decays gently, floor 90) and no 0.3–0.4° undershoot of centre (parks within 0.2°)". `FIRMWARE_VERSION` "1.5.20".

## 3. Tests
- [x] 3.1 `test/test_steer_position/test_main.cpp`: mirror the new constants in the parameter block (posTol 2, restartTol 10, floors 85 / 85, boostDecay 30, boostHoldErr 20) and adjust the existing expectations that depend on the old values.
- [x] 3.2 New: stalled at the tracking floor with P < floor → the first stalled check gives duty `floor + 30` (not hidden).
- [x] 3.3 New: after break-away the boost decays 30 → 20 → 10 → 0 over three moving checks and never goes negative.
- [x] 3.4 New: driving parks at |err| = 2 (0.2°) and does NOT park at |err| = 3 (0.3°), from both sides.
- [x] 3.5 New: a parked loop stays parked at |err| = 10 and restarts at |err| = 11 (1.0° restart unchanged).
- [x] 3.6 Far from the target (P > floor) the duty equals `P + boost` as before.
- [x] 3.7 `pio test -e native` green; `pio run -e esp32-s3-devkitc-1` builds.

## 4. Bench / vehicle (quad)
- [x] 4.1 USB `goto +10` / `goto -10` from centre: no plateaus in the diag log (no standing intervals at constant duty), move time noted (was 2.0–2.5 s), current while moving noted (was 17–27 A on plateaus).
- [x] 4.2 `goto 0` from +10 and from −10: residual ≤ 0.2° from both sides; no overshoot / hunting around centre. If it overshoots, lower `STEER_PWM_TRACK_MIN_DUTY` 90 → 85 → 80, then consider window 0.3°; record the final value in `Constants.h`. Bench 2026-11-06: floors 85 / 85, hold 2.0°, decay 30 — no mid-move plateaus, residual ≤ 0.1° from both sides, no overshoot.
- [ ] 4.3 Park correction: no pulses after a 0.1–0.2° park (trigger still 0.4°); a 0.5° residual still gets corrected.
- [ ] 4.4 Web jog / nudge still work (now clamped to ≥ 90 duty).
- [ ] 4.5 RC drive test: turns and returns to centre without jerks, the vehicle does not pull.

## 5. Aztec merge note (do at the next master → aztec merge, not in this change)
- [ ] 5.1 Merge takes the law change, `STEER_BOOST_DECAY` and `STEER_PARK_TRIGGER_TOL`; keep aztec `STEER_PWM_MIN_DUTY` 30, `STEER_PWM_TRACK_MIN_DUTY` 30 and `STEER_POSITION_TOLERANCE` 4 (0.4°); bench-check a 10° move on the buggy.
