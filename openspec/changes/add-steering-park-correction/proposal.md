# Change: Steering park correction — finish a centring with small pulses, tunable from the web

## Why
After a turn the RC stick returns to centre and the target is exactly 0° (the 2 %
`RC_STEERING_DEADBAND` in `MavlinkInterface` snaps a centred stick to 0). The wheel, however,
parks 0.5–1.0° off and the vehicle pulls left or right; the side varies from turn to turn.

The numbers measured with the USB steering diagnostics (change `add-steering-serial-diagnostics`,
quad bike, 2026-11-05) explain it:
- sustained break-away duty: L 46–50, R 42–44; coast after the duty is cut ≈ 0 (the 1:40
  self-locking worm stops the shaft at once);
- short pulses at standstill: nothing at ≤ 100 duty × 70 ms; 100 duty × 100 ms moves the wheel
  0.4–0.6°;
- a closed-loop `goto` stops SHORT of the target by 0.2–0.4°, inside the park window.

The closed loop (`include/SteerPositionLaw.h`) parks as soon as `|error| ≤ STEER_POSITION_TOLERANCE`
(4 = 0.4°) and drives again only beyond `STEER_RESTART_TOLERANCE` (10 = 1.0°). The P duty near the
target sits at the tracking floor and the wheel decelerates against the worm friction before the
window, so it often parks at the far edge of the window; any residual between 0.4° and 1.0° then
stays for good, because nothing inside the hysteresis band ever drives. Shrinking the window or
the band brings back the stop-go chatter fixed by `fix-steering-continuous-tracking`. The
remaining error is smaller than one standstill nudge (0.4–0.6°), so a single fixed pulse would
overshoot; a ladder of pulses that starts below the measured standstill threshold (rolling tyres
need less) and escalates only when the wheel does not move fits the measurements.

## What Changes
- **Park correction stage** in `SteeringController`, a sub-mode of the closed loop's PARKED state
  (no new `SteerPosState`, the `park` token is unchanged). It runs only when:
  - the steering is in position mode (healthy LWS, calibrated, VESC link up, no fault), the loop is
    PARKED, the setting is enabled and the USB diagnostics lock is off;
  - the target has been stable — moved by no more than `STEER_PARK_TARGET_STABLE` (3 = 0.3°) from
    the reference target — and the loop has been parked, for at least `STEER_PARK_SETTLE_MS`
    (1000 ms); the settle clock restarts on a target move beyond 0.3° and on entering PARKED;
  - `|target − position| > max(STEER_POSITION_TOLERANCE, done tolerance)`.
  It works relative to the CURRENT target, not only 0°; centre is the main case.
- **Pulse ladder** (duty × ms, per-vehicle setting, defaults rung 1 = 70 × 80, rung 2 = 85 × 90,
  rung 3 = 100 × 100). The first pulse is rung 1, toward the target (sign of the error). After each
  pulse the stage waits `STEER_PARK_PULSE_GAP_MS` (500 ms, from the pulse start + its length), then
  compares the position with the one before the pulse (`moved` = progress toward the target):
  1. `|error| ≤ done tolerance` → **done**;
  2. the error changed sign and `|error| >` done tolerance (overshoot) → log, next pulse rung 1 in
     the new direction of the error;
  3. moved away from the target by ≥ `STEER_PARK_MOVE_MIN` (1 = 0.1°) → log, next pulse rung 1
     toward the target;
  4. moved toward the target by ≥ 0.1° → same rung;
  5. moved < 0.1° → next rung (stays at rung 3).
  Every pulse counts toward the max. Once a correction has started it continues until the done
  tolerance, not the 0.4° trigger.
- **Give up** when the max pulses (setting, default 5) have been issued and the evaluation after the
  last one is not done: log `park correction gave up at X°`. Max pulses is a budget per parking:
  "done" does not refill it, so a wheel that keeps creeping cannot cause buzzing. The budget and the
  ladder re-arm only on a target move beyond 0.3°, on a new park entry (a main-loop restart beyond
  1.0°, or a command from idle) — i.e. the settle clock reset.
- **Pulses** are sent straight to `IMotorDriver::setSpeed(dir × duty)` and ended with
  `IMotorDriver::stop()` (coast) after the rung's ms — not through `driveJog()`, which would clamp
  the duty up to `STEER_PWM_MIN_DUTY` (80). They are not jogs (`jogDir_` stays 0). A pulse in a
  direction blocked by the stall latch is not sent; the stage waits for the cooldown and does not
  count it. `lastDriveDir_` is updated so a VESC fault during a pulse is handled as today.
- **Cancel immediately** (coast if a pulse is running, stage reset): target moved > 0.3°; the loop
  leaves PARKED for any reason — main-loop restart beyond `STEER_RESTART_TOLERANCE` (the main loop
  keeps priority), stop(), sensor fault, VESC fault / link loss, stall-stop; jog, nudge, rate mode,
  limit capture, sensor zero-set; USB diagnostics lock set (a `diagGoto()` move never gets the
  correction); the setting switched off or any park setting changed. Steering speed scaling changes
  nothing here (it scales the percent before the target is formed; a centred stick is 0 either
  way).
- **Debug log** (feature SERVO): `[STEER] park nudge 85x90 -> moved 0.15 deg (err 0.35)` after each
  evaluation; `[STEER] park overshoot -> rung 1, reversing (err -0.30)`;
  `[STEER] park nudge moved the wrong way -> rung 1 (err 0.70)`;
  `[STEER] park correction done at 0.10 deg (3 pulses)`;
  `[STEER] park correction gave up at 0.60 deg (5 pulses)`.
- **Per-vehicle web settings** in the EXISTING NVS namespace `"steering"` (beside `lim_l`, `lim_r`,
  `inv`, `cal_ver` and `steer_sca_base`; never `clear()` it; the v4 migration and the zero-set's
  `eraseLimits()` do not touch the new keys), owned by `SteeringController`:
  | Setting | NVS key | Type | Range | Default |
  |---|---|---|---|---|
  | enabled | `park_en` | bool | — | true |
  | rung 1/2/3 duty | `park_d1`, `park_d2`, `park_d3` | uint8 | 40–150 | 70 / 85 / 100 |
  | rung 1/2/3 length | `park_t1`, `park_t2`, `park_t3` | uint16 ms | 40–200 | 80 / 90 / 100 |
  | done tolerance | `park_tol` | int32, 0.1° (repo unit) | 1–5 (0.1–0.5°) | 2 (0.2°) |
  | max pulses | `park_max` | uint8 | 1–10 | 5 |
  The ladder must be non-decreasing: each rung's duty and ms ≥ the previous rung's and ≤ the next
  rung's; a rung command that would break this is refused (to raise the whole ladder, save rung 3
  first). Settings are persisted first, then applied. At boot a missing key reads as the default; an
  out-of-range value or a non-monotonic stored ladder is logged and ignored (the defaults are used
  for that setting / the whole ladder, NVS is not rewritten). With `park_en` off the steering
  behaves exactly as before this change.
- **Constants** (`include/Constants.h`, steering block): the defaults `STEER_PARK_ENABLED_DEFAULT`,
  `STEER_PARK_DUTY{1,2,3}_DEFAULT`, `STEER_PARK_MS{1,2,3}_DEFAULT`, `STEER_PARK_DONE_TOL_DEFAULT`,
  `STEER_PARK_MAX_PULSES_DEFAULT`; the bounds `STEER_PARK_DUTY_MIN`/`_MAX` (40/150),
  `STEER_PARK_MS_MIN`/`_MAX` (40/200), `STEER_PARK_DONE_TOL_MIN`/`_MAX` (1/5),
  `STEER_PARK_MAX_PULSES_MIN`/`_MAX` (1/10); the NVS key names. Compile-time only (not on the web):
  `STEER_PARK_SETTLE_MS` 1000, `STEER_PARK_TARGET_STABLE` 3 (0.3°), `STEER_PARK_PULSE_GAP_MS` 500,
  `STEER_PARK_MOVE_MIN` 1 (0.1°). These four decide when the firmware may move the wheel on its own
  and how a move is judged; a wrong value (short settle, small gap, tiny move threshold below the
  sensor's 0.1° resolution) produces pulses during driving or a buzzing wheel, which can only be
  seen with the USB diagnostics — so they change only with a reflash after a diagnostics run.
  Positions and tolerances stay in 0.1° integer units like every other steering constant (hence no
  `_DEG` suffix).
- **Pure law, host-tested**: header-only `include/SteerParkLaw.h` (no Arduino), in the style of
  `SteerPositionLaw.h`: `SteerParkParams` (trigger tolerance, done tolerance, move-min, settle ms,
  gap ms, max pulses, 3 × {duty, ms}), `SteerParkState` (phase ARMED / WAIT / DONE / GAVE_UP, rung,
  pulse count, pulse start ms and length, position before the pulse, last direction, last
  duty / ms, `reset()`), and `steerParkStep(state, params, now, err, pos, settledMs)` →
  `SteerParkResult { action NONE | PULSE | DONE | GIVE_UP, duty, ms, dir, moved, note
  NONE | SAME | ESCALATE | OVERSHOOT | WRONG_WAY }`. A step with `settledMs < settleMs` resets the
  cycle and returns NONE. Plus `steerParkLadderValid()` for the non-decreasing check. Native suite
  `test/test_steer_park` with the default ladder and a custom one.
- **Web**: commands `set_steer_park_enabled` (bool), `set_steer_park_rung` (value = rung 1–3,
  `strValue` = `"D,T"`), `set_steer_park_tol` (degrees), `set_steer_park_max` (count), routed through
  `VehicleController::processWebCommand`, accepted from any input source. Replies: "Park correction
  enabled" / "Park correction disabled", "Park rung N set to D×T", "Park tolerance set to X°", "Park
  max pulses set to N", "Park rung out of range (duty 40-150, 40-200 ms)", "Park ladder must not
  decrease (rung N)", "Park tolerance out of range (0.1-0.5°)", "Park max pulses out of range
  (1-10)", "Failed to save the park correction setting".
- **New card** "Доводка центру керма" / "Steering park correction" in `data/index.html`, right after
  the "Steering Calibration" card: an enable toggle button (pattern of the gear-boost toggle), three
  rung rows (duty + ms inputs, Save), a done-tolerance row and a max-pulses row (Save each, pattern of
  the speed-limit card rows), local validation, a status line for replies and a last-result line:
  "Останнє паркування: 3 імпульси, залишок 0.15°, останній 85×90" / "Last parking: 3 pulses,
  residual 0.15°, last 85×90"; "Здалось на 0.6° після 5 імпульсів" / "Gave up at 0.6° after 5
  pulses"; "Скасовано рухом стіка" / "Cancelled by stick movement"; "ще не було" / "none yet". Help
  text: lower the first rung if a pulse overshoots, raise the rungs if it gives up. uk + en.
- **Telemetry**: `steer_park_enabled`, `steer_park_d1..d3`, `steer_park_t1..t3`, `steer_park_tol`
  (deg, 1 decimal), `steer_park_max`, and a nested `steer_park_last` object
  `{outcome: "none"|"done"|"gave_up"|"cancelled", pulses, residual (deg, 2 decimals, null when
  none), duty, ms}` (nested objects are already used: `gearDefaults`, `probe`). The last result
  records "done" / "gave_up" at the end of a cycle and "cancelled" when a cycle that had issued at
  least one pulse is cancelled by a target move or a main-loop restart; other cancels (stop, faults,
  jog, diag) leave it unchanged. RAM only (not persisted).
- **Firmware 1.5.18** (patch: behaviour fix, no recalibration, no wiring change; ships after the 1.5.17 web fix), changelog bullet
  uk + en naming the new card.

## Impact
- Affected specs:
  - `vehicle-actuators`: ADDED "Steering Park Correction", ADDED "Steering Park Correction
    Settings".
  - `web-control`: ADDED "Steering Park Correction Configuration via Web Interface".
  - `web-telemetry`: ADDED "Steering Park Correction Telemetry".
- Affected code: new `include/SteerParkLaw.h`, `include/SteeringController.h`,
  `src/SteeringController.cpp`, `include/Constants.h`, `include/VehicleController.h`,
  `src/VehicleController.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp` (telemetry fields and
  the capacity note; `parseWebCommand()` already carries `value` + `strValue`),
  `src/TelemetryManager.cpp`, `data/index.html`, `include/Changelog.h`, `platformio.ini`
  (`test_filter` + header comment), new `test/test_steer_park/test_main.cpp`.
- Relation to pending changes: "Steering Stop Hysteresis" (in `fix-steering-continuous-tracking`,
  not yet archived) says the parked loop sends no motor commands and issues no drive inside the
  band. The park correction is the one, bounded exception (at most max-pulses short pulses per
  parking, only after the settle time); the new requirement states this explicitly instead of
  MODIFYing a requirement that is not in `specs/` yet. Archive this change after
  `fix-steering-continuous-tracking` and `add-steering-serial-diagnostics` (the diagnostics lock
  it refers to). The hysteresis values are those of 1.5.12 (0.4° / 1.0°), not the 1.0° / 3.0° in
  the pending spec text. No other pending change touches the steering loop.
- Behaviour change: a parked wheel 0.4–1.0° off target now gets up to 5 short pulses ~1 s after the
  stick settles; the wheel should end within 0.2° and the vehicle should stop pulling. With the
  card switch OFF the behaviour is exactly as before this change (1.5.17).
- Safety: pulses are ≤ 150 duty × 200 ms by the setting bounds (≤ 0.6°-class moves at the
  defaults), spaced ≥ 500 ms, at most 10 per parking, only in position mode with a healthy sensor
  and VESC; any stick movement, fault, jog, diagnostics or stop cancels at once; an overshoot beyond
  1.0° hands control back to the normal loop.
- Aztec (rack steering): gets the same code and default ladder through the `master` → `aztec`
  merge; the rack's friction differs from the quad's, so it may need its own ladder (now a web
  setting) after a bench run.
