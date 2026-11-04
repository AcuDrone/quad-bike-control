# Change: Steering — continuous target tracking, stop hysteresis and a tracking duty floor

## Why
Steering commanded by the autopilot moves in visible stop-go jerks. ArduPilot streams
`SERVO_OUTPUT_RAW` at about 25 Hz, and a moving stick changes the target by a few degrees per
frame. Steering from the web slider is smooth: the slider is debounced 100 ms and sends one large
target. Both paths end in `SteeringController::setSteeringPercent()`; the difference is the shape of
the command stream. The diagnosis below is from the code (`src/SteeringController.cpp`); the bench
test is pending.

The steering actuator (aztec buggy, and the quad on `master`, which uses the same drive) is a
brushed motor through a **1:40 self-locking worm gear**. The worm holds the shaft by itself as soon
as the motor stops; the tyre cannot back-drive it. A "tyre winds up and springs back when the motor
coasts" explanation is therefore ruled out, provided the LWS sits on the steering shaft downstream
of the worm (one bench check confirms it: with the stick still, the LWS reading must not drift after
`stop()`).

The jerk is a sequence of discrete moves, stop-go stepping:
- **The re-command guard quantises the stick.** While a move is in progress, a target within
  `STEER_RETARGET_TOLERANCE` (50 = 5°) of the current target is a no-op. The guard was added on
  purpose: without it the 25 Hz stream reset the 500 ms stall window and the 7500 ms move timeout on
  every frame, and neither could ever elapse. Its side effect is that a smoothly moving stick
  becomes 5° steps. Each accepted step also resets `moveStartTime_`, the stall timers and
  `dutyBoost_`.
- **Every step is a break-away kick.** `duty = clamp(|err| × STEER_KP + boost, 140, 255)` with
  `STEER_KP` 1.15 per 0.1°. `STEER_PWM_MIN_DUTY` 140 (55 %) was chosen for break-away against the
  worm's static friction ("no movement at 70"). Below about 12° of error the duty is always 140, so
  every small step starts as a full break-away kick.
- **Stop at the window, then start again.** The loop reaches the ±1° window
  (`STEER_POSITION_TOLERANCE` 10) and calls `stop()` (zero duty, coast). The worm stops the shaft
  instantly. The next target more than 5° away (or, once stopped, any target outside 1°) starts the
  next move with the next 140 kick.

The result under a slowly moving stick is a 0 ↔ 140 duty chatter: kick, reach the window, stop,
kick. No spring-back is involved.

## What Changes
- **Part 1 — continuous target tracking.** While the position loop is driving or parked, every
  `setSteeringPercent()` call updates the target, whatever the delta. Updating the target SHALL NOT
  reset `moveStartTime_`, the stall-check timers, the progress baseline or `dutyBoost_`. Those are
  reset only when a drive starts from rest (design D2). `STEER_RETARGET_TOLERANCE` is removed: it no
  longer rejects anything. Timers that still elapse under a moving target (design D3, D4):
  - the stall detector runs only while the loop is actually driving. Its 500 ms window restarts when
    a drive starts from rest and when the drive direction flips (the stick reversed), never on a
    target update;
  - the move timeout measures how long the loop has been continuously behind the target, not the
    time since the first command. Its clock is re-armed whenever `|error| ≤ STEER_RESTART_TOLERANCE`
    (the loop is keeping up). It trips only if the wheel stays more than 3° behind for
    `STEER_MOVE_TIMEOUT` (7500 ms);
  - the stall latch and cooldown (`STEER_STALL_COOLDOWN_MS`, same-direction refusal after a stall)
    are unchanged.
- **Part 2 — stop hysteresis** (design D5). The loop stops driving when
  `|error| ≤ STEER_POSITION_TOLERANCE` (1°) and does not drive again until
  `|error| > STEER_RESTART_TOLERANCE`, a new constant, **30 = 3°** (bench-tune). Three loop states:
  **idle / driving / parked**. Parked = target reached, motor coasting through `stop()`, the worm
  holds the shaft; no motor commands are sent while parked. Between the two bands no new kick is
  issued.
- **Part 3 — two duty floors** (design D6). `STEER_PWM_MIN_DUTY` **140** stays the **break-away**
  floor, applied only when a drive starts from rest (idle or parked → driving, and on a drive
  direction flip) until the wheel has moved `STEER_BOOST_MIN_PROGRESS` (0.3°) from where that drive
  started. After that, while driving continuously, the floor is a new `STEER_PWM_TRACK_MIN_DUTY`,
  **100** (bench-tune: must exceed the worm's running friction, which is lower than break-away). If
  the motor stalls at 100, the existing anti-stall boost escalates by +30 per 100 ms, and a real stall
  ends the move as today. The duty law is otherwise unchanged:
  `clamp(|err| × STEER_KP + boost, floor, 255)`. With the target moving a few degrees per frame the
  error stays small, the loop stays in driving, and the motor runs near the track floor instead of
  kicking at 140 and stopping.
- **Pure law, host-tested.** The tracking state machine (idle / driving / parked, hysteresis,
  timer rules, both duty floors) moves into a header-only `include/SteerPositionLaw.h` with a native
  test suite `test/test_steer_position`, in the style of `SteerRate.h` and `SpeedLimiterLaw.h`.
- **Unchanged:**
  - `IMotorDriver`, `VescMotorDriver`, `VescProtocol`: no new command. `stop()` stays zero duty =
    coast; with the self-locking worm the load side is locked anyway;
  - the web slider path: same 100 ms debounce, same `set_steering` command, same
    `setSteeringPercent()` call. A single large web target is driven exactly as before. The one
    visible difference is that a web target within 3° of a parked wheel is accepted without a drive
    (design D5);
  - `STEER_KP`, `STEER_PWM_MIN_DUTY` (still 140, now the break-away floor), `STEER_PWM_MAX_DUTY` and
    the anti-stall boost constants;
  - the stall threshold and window, the move-timeout value, the range guard, boot centering and the
    failsafe centre;
  - MAVLink decoding and steering speed scaling;
  - jog, nudge and the emergency stop.
- **Out of scope:** the sensorless rate mode (`SteerRate.h`, `JogOwner::RATE`). It never uses the
  position loop and keeps `STEER_PWM_MIN_DUTY` as its floor.
- **Version.** `FIRMWARE_VERSION` 1.5.4 → **1.5.5** (patch per `openspec/project.md`: behaviour fix,
  no recalibration, no wiring change), with a `Changelog.h` entry (uk/en). 1.6.0 and 1.7.0 stay
  reserved for `move-can-rx-to-interrupt` and `move-aztec-vesc-to-can` (task 0.2).
- **Branch.** Implemented on `master`, then `master` is merged into `aztec`. Both vehicles run the
  same `SteeringController`.

## Impact
- Affected specs:
  - `vehicle-actuators`:
    - REMOVED "Steering Re-command Guard and Stall Latch" and ADDED its replacement "Steering
      Continuous Target Tracking and Stall Latch" (continuous tracking replaces the 5° no-op guard;
      timer-reset rules; the stall latch, cooldown and escape scenarios are carried over unchanged).
      A MODIFIED delta is not possible here: the validator refuses a MODIFIED block that drops the
      two guard scenarios ("Re-command within tolerance is a no-op", "Target change beyond tolerance
      resets the move"), and keeping them under those names would contradict the new behaviour;
    - ADDED "Steering Stop Hysteresis";
    - ADDED "Steering Break-away and Tracking Duty Floors".
  - `vehicle-systems`: no delta. "Steering Control System" still holds as written: movement stops
    when the target is reached within `STEER_POSITION_TOLERANCE`. The rate-mode duty mapping keeps
    `STEER_PWM_MIN_DUTY`.
  - `mavlink-interface`: no delta. Decoding, rate and scaling are untouched.
  - `web-telemetry`, `web-control`: no delta. No new keys, no new commands.
- Affected code:
  - `include/Constants.h` (new `STEER_RESTART_TOLERANCE`, `STEER_PWM_TRACK_MIN_DUTY`;
    `STEER_RETARGET_TOLERANCE` removed; version), `include/Changelog.h`;
  - `include/SteeringController.h`, `src/SteeringController.cpp`;
  - new `include/SteerPositionLaw.h`, new `test/test_steer_position/`, `platformio.ini`
    (`test_filter`).
- Pending changes: neither `move-aztec-vesc-to-can` (1.7.0) nor `move-can-rx-to-interrupt` (1.6.0)
  touches "Steering Re-command Guard and Stall Latch"; this change no longer touches the driver
  requirements. No archive-order dependency.
- Safety:
  - every stop path is a coast, exactly as today, and the worm locks the shaft after it;
  - stall detection and the move timeout keep working under the continuous stream (design D3, D4);
    the stall latch is unchanged;
  - the lower tracking floor cannot hide a jam: a stalled motor gets the +30 per 100 ms boost and
    then the 500 ms stall-stop, as today.
- Behaviour change for operators: autopilot steering follows the stick smoothly instead of in steps.
  No recalibration, no wiring change.
