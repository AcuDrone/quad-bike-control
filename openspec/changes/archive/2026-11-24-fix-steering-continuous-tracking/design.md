# Design: Steering — continuous target tracking, stop hysteresis and a tracking duty floor

## Context
- **Drive train.** Brushed motor → **1:40 self-locking worm gear** → steering shaft. The worm holds
  the shaft whenever the motor is not driven; the tyre cannot back-drive it. The Bosch LWS is
  assumed to sit on the steering shaft downstream of the worm, so a stopped motor means a constant
  LWS reading. A tyre spring-back hypothesis is ruled out by the worm under that assumption; one
  bench check (task 7.3) confirms it: with the stick still, the LWS reading must not drift after
  `stop()`. Drift would mean compliance between the worm output and the sensor.
- Command paths into the position loop (`SteeringController::setSteeringPercent()`):
  - MAVLink: `VehicleController::processMavlinkCommands()` calls it on **every control-loop
    iteration** with the latest decoded, speed-scaled `SERVO_OUTPUT_RAW` value. The value itself
    changes at the autopilot's rate (about 25 Hz), a few degrees per frame while the stick moves;
  - web `set_steering`: the slider is debounced 100 ms, so a drag usually arrives as one large
    target;
  - boot centering and the failsafe centre: one call with 0 %.
- Today's loop (`update()`), per iteration while `isMoving_`:
  1. stall check: position change < `STEER_STALL_THRESHOLD` (0.5°) over `STEER_STALL_TIMEOUT`
     (500 ms) → stall-stop + latch;
  2. `|error| ≤ STEER_POSITION_TOLERANCE` (1°) → `stop()` = zero duty = coast, `isMoving_` false;
  3. range guard, then `now − moveStartTime_ ≥ STEER_MOVE_TIMEOUT` (7500 ms) → `stop()`;
  4. anti-stall boost: no progress of `STEER_BOOST_MIN_PROGRESS` (0.3°) per 100 ms → `+30` duty
     (and `−30` while moving);
  5. `duty = clamp(|err| × 1.15 + boost, 140, 255)`. `STEER_PWM_MIN_DUTY` 140 was set for
     break-away against the worm's static friction (bench: no movement at 70).
- Today's `setSteeringPercent()`: while `isMoving_` and `|target − targetRel_| ≤ 50` (5°) → no-op.
  Otherwise a fresh move: target, `moveStartTime_`, stall baseline, progress baseline and
  `dutyBoost_` are all reset. When not moving (target reached, coasting) every call outside 1° is a
  fresh move.
- **Diagnosis.** Under a slowly moving stick the duty log alternates 0 ↔ 140. The motion is a
  sequence of discrete moves: an accepted target (outside the 5° guard) starts a move at the 140
  floor; the loop reaches the ±1° window; `stop()`; the worm stops the shaft instantly; the next
  target more than 5° away starts the next 140 kick. Stop-go stepping, no spring-back involved.
- Rate mode (`SteerRate.h`, `JogOwner::RATE`) never uses the position loop. Out of scope.

## Goals / Non-Goals
- Goals:
  - the wheel follows a continuously moving target without stop-go;
  - no 0 ↔ 140 duty chatter when the stick moves slowly or holds still;
  - stall detection, the move timeout and the stall latch keep working under the continuous stream;
  - every stop is still a coast.
- Non-goals:
  - a new duty law (PID, velocity loop, feed-forward). The P law stays; only its floor is split
    (D6);
  - any motor-driver change: no brake or hold command (the worm holds);
  - any change to the sensorless rate mode, the jog or the nudge;
  - any change to the web slider debounce or to MAVLink decoding and speed scaling;
  - new telemetry keys or web controls for the new constants (compile-time, bench-tuned).

## Decisions

### D1 — Three position-loop states
The `isMoving_` flag becomes a state (`SteerPosState` in `SteerPositionLaw.h`):

| State | Motor | Entered | Left |
|---|---|---|---|
| `IDLE` | coast (zero duty) | boot, any stop / fault / stall / timeout / guard / jog / rate / capture / zero-set | accepted percent command |
| `DRIVING` | P duty toward the target | from `IDLE` on an accepted command with `\|error\| > POS_TOL`; from `PARKED` when `\|error\| > RESTART_TOL` | `\|error\| ≤ POS_TOL` → `PARKED`; stall / timeout / guard / fault → `IDLE` |
| `PARKED` | coast (`stop()` once on entry), the worm holds | from `DRIVING` at `\|error\| ≤ POS_TOL`; from `IDLE` on an accepted command with `\|error\| ≤ POS_TOL` | `\|error\| > RESTART_TOL` → `DRIVING`; any stop / fault / jog / rate / capture / zero-set / sensor loss → `IDLE` |

- `PARKED` sends no motor commands after the single `stop()` on entry. The difference from `IDLE`
  is only that the loop still owns the target and applies the 3° restart band (D5).
- `isMoving_` keeps its meaning for the rest of the class ("the closed loop owns the target"), true
  in `DRIVING` and `PARKED`. Everything that today does `isMoving_ = false; motor_.stop()` now sets
  `IDLE` and coasts.

### D2 — Continuous tracking: a target update is not a new move
`setSteeringPercent()` keeps its gates (calibrated, sensor healthy, driver healthy), its clamping to
the limits and to `STEER_LWS_SAFE_RANGE`, and its boot-centering cancel. Then:
- **state `DRIVING` or `PARKED`:** `targetRel_ = target`, return true. Nothing else changes:
  `moveStartTime_`, `lastStallPosition_` / `lastStallCheckTime_`, `lastProgressPos_` /
  `lastProgressTime_`, `dutyBoost_` and the break-away state (D6) keep their values. Any delta is
  accepted, including 0.1°. Whether a parked loop drives is decided in `update()` by D5;
- **state `IDLE`:** the stall latch is checked exactly as today (`latchBlocks(d)`, `d` = sign of
  `target − position`). If accepted, the latch is cleared, any jog is cancelled, `targetRel_` is set,
  and the state becomes `DRIVING` (with the "start from rest" reset) or `PARKED` if already within
  `STEER_POSITION_TOLERANCE`.
- `STEER_RETARGET_TOLERANCE` is removed. Its only job was to stop the stream from resetting the
  timers; D2 removes the reset itself, so the guard has nothing left to protect.
- Why the latch is not checked on a target update while tracking: the latch is set only by a stall,
  and a stall always ends in `IDLE` (D3). A tracking loop therefore never carries a latch.

**Start from rest** (`IDLE → DRIVING` or `PARKED → DRIVING`) is the only event that resets:
`moveStartTime_ = now`; stall baseline = current position, `now`; progress baseline = current
position, `now`; `dutyBoost_ = 0`; break-away armed with `breakawayStartPos` = current position
(D6).

### D3 — Stall detector under a moving target
- The detector runs only in `DRIVING` (as today it runs only while `isMoving_`). In `PARKED` the
  motor is not driven, so "no progress" is expected and not a stall.
- Its window restarts on a start from rest (D2) and **when the drive direction flips** while
  driving (sign of `error` changes because the stick reversed or the wheel overshot). A reversal
  passes through zero speed, and a window spanning it can see less than 0.5° of net travel without
  any jam. The direction flip also resets `dutyBoost_` to 0 (the boost was earned against the load
  in the old direction) and re-arms break-away (D6).
- It never restarts on a target update. A jammed wheel under a stream of targets that all lie beyond
  the jam keeps the same error sign, so the window elapses and the stall trips after 500 ms, exactly
  as with a single command today.
- A stall-stop is unchanged: coast, `IDLE`, latch the direction of the remaining error for
  `STEER_STALL_COOLDOWN_MS`. While latched, the stream's same-direction targets are refused in
  `IDLE` (D2) and the motor stays coasting; an opposite-direction target is accepted at once; after
  the cooldown a same-direction target is accepted and drives into the jam again, which trips again
  after 500 ms. This is today's behaviour.

### D4 — Move timeout under a moving target
- **Rule:** `moveStartTime_` is set on a start from rest and **re-armed on every `DRIVING` iteration
  where `|error| ≤ STEER_RESTART_TOLERANCE`**. The timeout trips when
  `now − moveStartTime_ ≥ STEER_MOVE_TIMEOUT` (7500 ms, unchanged).
- Meaning: the timeout measures how long the loop has been continuously more than 3° behind its
  target. Measuring from the first command (today) would make any long tracking session time out
  after 7.5 s even when the wheel follows perfectly. Measuring from the last target update would never
  trip under a 25 Hz stream.
- A single lock-to-lock command (web or autopilot step) behaves as today: the error stays above 3°
  for the whole travel, so the clock runs from the start, and the existing bench check of
  `STEER_MOVE_TIMEOUT` against a full lock-to-lock move still applies.
- A stick moving faster than the wheel for 7.5 s in one direction is not physical (the target is
  clamped to the limits, so the wheel catches up at the lock or stalls there).
- On timeout: coast, `IDLE`, no latch (as today). The next command from the stream starts a fresh
  drive (as today).
- Why not "time since progress": progress is the anti-stall boost's and the stall detector's job.
  The timeout is the backstop for "progressing too slowly to ever arrive", for example a dragging
  wheel that moves 0.6° every 500 ms.

### D5 — Stop hysteresis
- `DRIVING → PARKED` at `|error| ≤ STEER_POSITION_TOLERANCE` (10 = 1°, unchanged): `stop()` once
  (coast), the worm holds the shaft. `PARKED → DRIVING` only at `|error| > STEER_RESTART_TOLERANCE`
  (**30 = 3°**, new, bench-tune), as a start from rest.
- The rule is evaluated in `update()` against the current target and position. With the worm the
  position does not move while parked, so in practice the error grows only because the target moved.
- **3°** of steering-shaft angle: three times the stop window, so a 1° overshoot plus a small target
  creep does not restart; about 0.6 % of a ~500° half-travel, far below what the autopilot's steering
  controller notices at the wheels. Bench-tune between 2° and 5°.
- A slow stick: the target creeps, the wheel stays parked until the target is 3° away, then drives
  and tracks continuously (D2, D6) until it catches up within 1°. At 25 Hz a stick that keeps moving
  keeps the loop in `DRIVING`; it only parks when the stick pauses.
- Consequence for single commands: from `PARKED`, a new target within 3° of the wheel is accepted
  without a drive. This applies to the web slider too (the minimum effective web step from a parked
  wheel becomes 3° of shaft angle). From `IDLE` the threshold stays 1°: a first command after a
  stop, a jog or boot is driven exactly as today.

### D6 — Two duty floors: break-away and tracking
- **Law:** `duty = clamp(|err| × STEER_KP + boost, floor, STEER_PWM_MAX_DUTY)`, signed by the error,
  with
  - `floor = STEER_PWM_MIN_DUTY` (**140**, unchanged value, now the *break-away* floor) while
    break-away is armed;
  - `floor = STEER_PWM_TRACK_MIN_DUTY` (**100**, new, bench-tune) otherwise.
- **Break-away is armed** on a start from rest (`IDLE → DRIVING`, `PARKED → DRIVING`) and on a drive
  direction flip (D3: the motor passes through zero speed, the worm is at rest again), storing
  `breakawayStartPos` = current position. It is **disarmed** on the first `DRIVING` iteration where
  `|position − breakawayStartPos| ≥ STEER_BOOST_MIN_PROGRESS` (3 = 0.3°), and stays disarmed until
  the next start from rest or direction flip.
- Rule chosen: the simple distance rule, not the 100 ms progress-check tick. It reacts on the first
  iteration the wheel has moved instead of waiting up to 100 ms, needs one stored position, and
  reuses the existing 0.3° "moving" threshold so there is no new constant for it. A target update
  never re-arms break-away (D2).
- Why 140 only at break-away: 140 (55 %) was chosen because the worm's *static* friction needs it
  ("no movement at 70"). Once the shaft turns, the worm's *running* friction is lower, so a lower
  floor keeps it moving. 100 is a first guess between the two bench points; it must exceed running
  friction. Bench-tune 90–120.
- **If 100 is too low** the motor slows or stops mid-track: the anti-stall boost already adds +30
  per 100 ms of no progress (100 → 130 → 160 …), and a true jam still trips the 500 ms stall-stop,
  as today. A too-low track floor costs some lag, never a missed stall.
- **Why the stick now feels continuous:** with the target moving a few degrees per frame (D2) the
  error stays small; the loop never reaches the park window while the stick moves, so it never
  leaves `DRIVING`; after the first 0.3° the floor is 100, and `|err| × 1.15` at small errors is
  below it, so the motor runs steadily near the track floor instead of kicking at 140, reaching 1°
  and stopping. The only 140 kicks left are true starts: from idle, from parked beyond 3°, and on a
  reversal.
- Large steps (web slider, lock-to-lock) are unaffected in practice: the P term saturates at 255
  above ~22° of error, far above either floor.

### D7 — Pure law and tests
- `include/SteerPositionLaw.h`, header-only, no Arduino and no `Constants.h` (callers pass
  constants), like `SteerRate.h`:
  - `enum class SteerPosState : uint8_t { IDLE, DRIVING, PARKED }`;
  - `struct SteerPosParams { posTol, restartTol, kp, breakawayMinDuty, trackMinDuty, maxDuty,
    stallThreshold, stallWindowMs, moveTimeoutMs, boostCheckMs, boostMinProgress, boostStep }`;
  - `struct SteerPosTimers { moveStartMs, stallPos, stallMs, progressPos, progressMs, boost,
    lastDir, breakaway, breakawayStartPos }` with `startFromRest(pos, now)`;
  - `SteerPosEvent steerPosStep(SteerPosState&, SteerPosTimers&, const SteerPosParams&,
    int32_t target, int32_t pos, uint32_t now, int16_t* dutyOut)` returning
    `NONE / PARKED / RESTARTED / STALL / TIMEOUT`. The range guard, sensor, driver and latch stay in
    `SteeringController` (they need hardware state).
- `SteeringController::update()` and `setSteeringPercent()` call the law; side effects (motor
  commands, logs, latch) stay in the controller.

### D8 — Logging
- The existing 100 ms `[STEER] ... target= err= duty= boost=` line keeps its format while
  `DRIVING`, with one added field `floor=<140|100>` so the bench can see which floor is active.
- One line on entering `PARKED` (`[STEER] parked err=<0.1°>`) and one on leaving it
  (`[STEER] parked -> drive err=<0.1°>`), SERVO feature only. With the stream this is a few lines
  per second at most, and it is exactly what the bench needs to see the chatter is gone.

## Risks / Trade-offs
- **Track floor below running friction.** The motor stalls briefly at each tracking start until the
  boost lifts it; visible as lag, not as a fault. Bench-tune `STEER_PWM_TRACK_MIN_DUTY` upward.
- **Track floor too high.** The wheel overshoots the 1° window on small errors and parks early or
  reverses; visible as `floor=` 100 with alternating `err=` signs. Bench-tune downward.
- **Compliance between worm and LWS.** If the LWS reading drifts after `stop()` (task 7.3), the
  sensor is not rigidly downstream of the worm; the 3° band absorbs small drift, larger drift needs a
  mechanical fix, not a firmware one.
- **3° band on the web slider.** Very fine web adjustments under 3° from a parked wheel no longer
  move it. Acceptable for a bench control; the band is a compile-time bench-tune.
- **Move timeout semantics change.** A tracking session no longer times out after 7.5 s of tracking.
  This is intended; a jam is still caught by the stall detector in 500 ms.

## Migration Plan
- Compile-time only. No NVS change, no recalibration, no wiring change.
- Rollback: flash 1.5.4.

## Open Questions
- Final values of `STEER_RESTART_TOLERANCE` (3°) and `STEER_PWM_TRACK_MIN_DUTY` (100) after the
  bench.
