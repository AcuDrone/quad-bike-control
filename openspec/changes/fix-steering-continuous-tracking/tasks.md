## 0. Preconditions
- [x] 0.1 The proposal is approved. Rate mode stays out of scope; no motor-driver change (the self-locking worm holds the shaft).
- [x] 0.2 Version slot: `master` is at 1.5.4. Take **1.5.5** (patch per `openspec/project.md`: behaviour fix, no recalibration or wiring change). Do not take 1.6.0 or 1.7.0, which `move-can-rx-to-interrupt` and `move-aztec-vesc-to-can` claim. If either has landed first, take the next free patch on top of it.
- [ ] 0.3 Branch: implement on `master` (quad), then merge `master` into `aztec`.

## 1. Pure law (host-testable)
- [x] 1.1 Create `include/SteerPositionLaw.h` (header-only, no Arduino / `Constants.h`) per design D1–D7: `SteerPosState { IDLE, DRIVING, PARKED }`, `SteerPosParams` (with `breakawayMinDuty` and `trackMinDuty`), `SteerPosTimers` with `startFromRest()`, `steerPosStep()` returning `NONE / PARKED / RESTARTED / STALL / TIMEOUT` and the signed duty for `DRIVING`. Duty law: `clamp(|err| × kp + boost, floor, maxDuty)`, `floor` = `breakawayMinDuty` while break-away is armed, else `trackMinDuty` (design D6).
- [x] 1.2 Add `test/test_steer_position/test_main.cpp` (Unity). Cases:
  - `DRIVING` → `PARKED` at `|err| ≤ posTol`; `PARKED` stays at `posTol < |err| ≤ restartTol` (moved target); `PARKED` → `DRIVING` at `|err| > restartTol` with all timers reset and break-away armed (`RESTARTED`);
  - a target update in `DRIVING` changes the duty sign/size but never resets `moveStartMs`, the stall baseline, the progress baseline, the boost or the break-away state;
  - duty floors: at 2° error from rest the duty is 140; after the wheel moves ≥ 0.3° from the drive's start it is 100 at 2° error; a target update does not re-arm 140; a direction flip and a restart from `PARKED` re-arm 140; at 30° error the duty is 255 either way;
  - track floor too low: no progress at 100 → boost raises the duty by 30 per 100 ms;
  - stall: target stream beyond a fixed position (jam) at 25 Hz → `STALL` after 500 ms ± one step, both with break-away armed (never moved) and after break-away (moved, then jammed at the track floor); a slowly moving wheel (≥ 0.5° per 500 ms) never stalls;
  - direction flip in `DRIVING` restarts the stall window and zeroes the boost; a reversal through zero speed does not trip a stall;
  - move timeout: error held > `restartTol` for 7500 ms → `TIMEOUT`; a target ramp the wheel follows within `restartTol` for 30 s → no timeout; a single lock-to-lock step that takes > 7500 ms → `TIMEOUT` (today's behaviour).
- [x] 1.3 Add `test_steer_position` to `[env:native]` `test_filter` in `platformio.ini`, one suite per indented line (the single-line form silently skips suites).

## 2. Constants (`include/Constants.h`)
- [x] 2.1 Remove `STEER_RETARGET_TOLERANCE` and rewrite its comment block ("Re-command guard + stall latch") to describe continuous tracking and the timer-reset rules (design D2–D4). Keep `STEER_STALL_COOLDOWN_MS 700` unchanged.
- [x] 2.2 Add, each with a one-line rationale comment:
  - `STEER_RESTART_TOLERANCE 30` — 0.1° (3.0°), drive restarts from parked only beyond this — bench-tune 2–5° (design D5);
  - `STEER_PWM_TRACK_MIN_DUTY 100` — duty floor while driving after break-away; must exceed the worm's running friction — bench-tune 90–120 (design D6).
- [x] 2.3 Update comments: `STEER_PWM_MIN_DUTY` (140) is the break-away floor (worm static friction, "no movement at 70"), used from rest until 0.3° of progress, and still the rate-mode floor; `STEER_POSITION_TOLERANCE` is the park window; `STEER_MOVE_TIMEOUT` measures time continuously behind by more than `STEER_RESTART_TOLERANCE` (design D4).

## 3. Steering controller (`SteeringController.h/.cpp`)
- [x] 3.1 Replace the closed-loop bookkeeping with `SteerPosState posState_` and `SteerPosTimers` (design D1, D7). Keep `isMoving_` semantics for the rest of the class as "the closed loop owns the target" (`DRIVING` or `PARKED`), or replace every use of it.
- [x] 3.2 `setSteeringPercent()` (design D2): keep the gates, clamping and boot-centering cancel; remove the 5° guard. In `DRIVING` / `PARKED` only update `targetRel_`. In `IDLE` check the stall latch, clear it on acceptance, cancel any jog, set `targetRel_`, then `startFromRest()` and `DRIVING`, or `PARKED` (no drive) if `|error| ≤ STEER_POSITION_TOLERANCE`.
- [x] 3.3 `update()`: the stall check moves into the law for `DRIVING` (the jog stall check stays as it is). Keep the order: sensor/zero/driver/boot-centre, jog handling, `!sensorOk_` → `IDLE` + coast, range guard (`DRIVING` only) → `IDLE` + coast, then `steerPosStep()`. Act on its event: `PARKED` → `motor_.stop()` once, log; `RESTARTED` → log; `STALL` → `triggerStallStop()` with the direction of the remaining error; `TIMEOUT` → "Movement timeout" log, `IDLE` + coast. In `DRIVING` `motor_.setSpeed(duty)` and the existing 100 ms log line with the added `floor=` field. In `PARKED` send nothing.
- [x] 3.4 Stop paths: every place that sets `isMoving_ = false` today (`stop()`, `refreshSensor()` on sensor loss, `driveJog()`, `triggerStallStop()`, `serviceDriver()` on comm loss and on a VESC fault, captures and zero-set through `stop()`) sets `IDLE` and coasts with `motor_.stop()`. A VESC fault while `PARKED` coasts without a stall latch (no drive was in progress) — keep today's `else motor_.stop()` branch and make sure `PARKED` takes it. If `calibrated_` becomes false while parked (limits erased), go to `IDLE` on the next `update()`.
- [x] 3.5 Logging per design D8: one line on entering `PARKED`, one on leaving it to drive. No change to the jog/rate/nudge logs.
- [x] 3.6 Doxygen in `SteeringController.h`: `setSteeringPercent()` (continuous tracking, latch only from rest), `update()` (states, hysteresis, two duty floors), and the removed re-command guard.

## 4. Docs
- [x] 4.1 Check `MAVLINK_SETUP.md` and `WEB_PORTAL_SETUP.md` for statements about the 5° re-command guard or the steering minimum duty; update them to "tracks continuously, parks within 1°, restarts beyond 3°; 140 break-away, 100 while tracking" if present. No web UI change.

## 5. Version and changelog
- [x] 5.1 `include/Constants.h`: `FIRMWARE_VERSION` "1.5.4" → **"1.5.5"** (or the version chosen in task 0.2).
- [x] 5.2 `include/Changelog.h`: a new first entry `{"version":"1.5.5","date":"<implementation date>"}`:
  - uk: "Кермо від автопілота тепер плавно стежить за стіком, без ривків", "Під час стеження мотор керма працює на нижчій потужності, повна потужність лише для зрушення з місця", "Повторне калібрування не потрібне"
  - en: "Autopilot steering now follows the stick smoothly, without jerks", "While tracking the steering motor runs at a lower minimum power; full break-away power only when starting from rest", "No recalibration required"

## 6. Build and test
- [x] 6.1 `pio run` builds without new warnings on `master`.
- [x] 6.2 `pio test -e native` passes every listed suite, including `test_steer_position`, and all of them are reported as run.
- [x] 6.3 `openspec validate fix-steering-continuous-tracking --strict`.
- [ ] 6.4 Merge `master` into `aztec` and check that `pio run` builds there without new warnings. Resolve only branch-specific context (Changelog / version line).

## 7. Bench verification [USER/MANUAL]
Enable the SERVO debug feature and watch the `[STEER] ... target= err= duty= boost= floor=` lines and the parked lines.
- [ ] 7.1 [USER/MANUAL] Wheels lifted, MAVLink steering, stick moved slowly from centre to half lock and back over ~5 s: the wheel follows continuously; after the first line of each direction `floor=100` and `duty=` stays at or near the track floor; there is no 0 ↔ 140 chatter and no parked/drive cycling while the stick moves; `target=` changes by small steps, not 5° jumps. If the wheel hesitates at 100 (boost climbing every 100 ms), raise `STEER_PWM_TRACK_MIN_DUTY`.
- [ ] 7.2 [USER/MANUAL] Wheels on the ground (vehicle stationary, then rolling slowly), same slow-stick test: same result as 7.1, duty at the track floor, no visible stop-go.
- [ ] 7.3 [USER/MANUAL] Stick held still after a move (wheels lifted and on the ground): one `parked` line, then no drive lines, and the LWS reading (`steer_angle`) does not drift for 30 s. Drift would mean compliance between the worm output and the sensor (design Context).
- [ ] 7.4 [USER/MANUAL] Stall, both floors: (a) from rest, command a target into a lock the wheel already sits against → `Stall-stop (position)` within about 0.5 s at the 140 floor; (b) track slowly into a lock with the stick moving in the jam direction → `Stall-stop (position)` within about 0.5 s after the wheel stops, the boost climbing from the 100 floor. In both cases the motor coasts, same-direction refusal for 700 ms, opposite stick accepted at once.
- [ ] 7.5 [USER/MANUAL] Move timeout: with a heavy drag (wheels on the ground, tyre on a high-friction surface) or a temporarily lowered `STEER_MOVE_TIMEOUT`, hold the error above 3° without a stall: `Movement timeout — stopping` appears after the timeout, and the motor coasts. A 30 s session of continuous stick movement with the wheel keeping up does not time out.
- [ ] 7.6 [USER/MANUAL] Web `set_steering` (slider): a large drag moves as before and ends in one `parked` line. A web step under 3° from a parked wheel does not drive (expected, design D5); a step over 3° drives as before.
- [ ] 7.7 [USER/MANUAL] Stop paths: unplug the LWS while parked or driving → coast, sensor fault logged; jog and nudge from the web while parked → the jog works; failsafe → centres and parks; VESC link unplugged → driver fault, coast. Boot centering still runs once and ends in one `parked` line.
- [ ] 7.8 [USER/MANUAL] The web "What's new" and the sidebar show version 1.5.5.
