# Design: Steering RATE mode without the LWS sensor

## Context
- Steering on aztec: a VESC (UART0) drives the steering motor open-loop by duty
  (`IMotorDriver::setSpeed(−255..+255)`). Position feedback comes from the Bosch LWS on CAN.
  `SteeringController` closes the loop.
- Sensor health already exists. `LwsSensor::evaluate()` returns a fault token that takes effect
  at once (on the first bad frame, or after `STEER_LWS_TIMEOUT_MS` = 100 ms with no frame).
  Recovery needs `STEER_LWS_RECOVER_FRAMES` = 5 consecutive good frames (≈ 50 ms).
  `isSensorOk()` is the debounced flag.
- Without the sensor today:
  - `setSteeringPercent()` refuses.
  - `jog()` / `nudge()` run open-loop at a fixed duty and stop after `STEER_JOG_TIMEOUT_MS`
    (500 ms) without a refresh, or when the `STEER_NUDGE_MS` pulse ends.
  - Position stall detection runs only `if (sensorOk_ && …)`.
  - A VESC fault code takes `triggerStallStop()` (stop + latch in `lastDriveDir_`).
- MAVLink steering:
  - `MavlinkInterface::getSteering()` returns −100..+100 with `RC_STEERING_DEADBAND` (2 %)
    applied.
  - `VehicleController::processMavlinkCommands()` passes it through `applySteeringScale()` to
    `setSteeringPercent()`. It runs every loop while the source is MAVLink and the signal is
    valid. It re-reads the LAST decoded channel even when no new frame arrived.
- Autopilot mode: `mavlink_.hasAutopilotMode()` / `getAutopilotCustomMode()`. Rover MANUAL =
  `ROVER_CUSTOM_MODE_MANUAL` (0). The speed-scaling code already treats an unknown or stale mode
  as MANUAL.

## Goals / Non-Goals
- Goal: the RC operator can still steer (open-loop, by rate) in MANUAL when the sensor is dead.
- Goal: nothing new moves the wheels while the sensor is healthy, in autonomous modes, in failsafe,
  or at boot.
- Non-goal: dead-reckoning a position from rate or VESC tachometer, software end stops without a
  sensor, a configurable mode switch, rate mode from the web portal.

## Decisions

### D1 — Three steering modes, derived every loop (no stored state machine beyond the token)
`steer_mode` ∈ { `position`, `rate`, `blocked` } is computed in `VehicleController::update()`
before the MAVLink commands are processed. It uses this pure function (in `include/SteerRate.h`,
no Arduino dependency):

```
SteerMode steerModeSelect(bool sensorOk, bool sourceIsMavlink,
                          bool apModeKnown, bool apModeIsManual);
  sensorOk                                                    -> POSITION
  !sensorOk && sourceIsMavlink && (!apModeKnown || apModeIsManual)
                                                              -> RATE
  otherwise                                                   -> BLOCKED
```

Notes:
- `sensorOk` = `SteeringController::isSensorOk()`, the existing debounced flag.
- `sourceIsMavlink` = `currentInputSource_ == InputSource::MAVLINK`. Web control engaged means
  WEB; failsafe means FAILSAFE.
- `apModeKnown` = `mavlink_.hasAutopilotMode()`; `apModeIsManual` =
  `getAutopilotCustomMode() == ROVER_CUSTOM_MODE_MANUAL` (passed as a bool so the header needs no
  `Constants.h` and stays host-testable). Unknown/stale counts as MANUAL, the same rule as
  the speed scaling ("Speed-Scaled Steering Authority").
- Limits missing with a healthy sensor is still `position`. Percent commands are then refused as
  today (not calibrated). Rate mode NEVER activates while the sensor is healthy.
- VESC link down does not change the mode. Every command is refused anyway, and
  `steer_driver_ok` reports it separately.

### D2 — Transitions and their actions

| From → To | Trigger | Action |
|---|---|---|
| POSITION → RATE / BLOCKED | sensor fault (first bad frame or 100 ms timeout) | existing: closed-loop move stopped, a web jog/nudge continues. Rate is **disarmed** (D5). |
| RATE → BLOCKED | source leaves MAVLink (web takeover, link loss → FAILSAFE) or autopilot reports a known non-MANUAL mode | `SteeringController::stopRate()`: a rate-owned drive stops at once (duty 0). It does not wait for the 500 ms timeout. A web jog/nudge is not touched. |
| BLOCKED → RATE | source back to MAVLink, or autopilot back to MANUAL/unknown | rate **disarmed**. No motion until the stick is seen centred (D5). |
| RATE → POSITION | sensor healthy again (5 good frames) | `stopRate()`: the motor stops and the wheel **holds where it is**. No target is set, no move is started, boot centring is not re-requested. The next percent command is a fresh, normal closed-loop move. The stall baseline reset already in `refreshSensor()` applies. |
| BLOCKED → POSITION | sensor healthy again | nothing beyond today's recovery |

Every change of `steer_mode` produces one serial line, `[STEER] mode: rate -> position (sensor OK)`
(feature SERVO, not rate-limited because transitions are debounced by D7). It also produces a
MAVLink `STATUSTEXT` (D9).

### D3 — Rate mapping (design decision, user proposal)
Pure function in `include/SteerRate.h`:

```
int16_t steerRateDuty(float pct, float deadband, uint8_t minDuty, uint8_t maxDuty);
  a = |pct| clamped to 100
  a <  deadband                → 0
  d = (a − deadband) / (100 − deadband)          // 0 < d ≤ 1
  duty = round(minDuty + (maxDuty − minDuty) · d²)
  return sign(pct) · duty                          // + = right
```

It is called with `RC_STEERING_DEADBAND` (2.0), `STEER_PWM_MIN_DUTY` (140) and
`STEER_PWM_MAX_DUTY` (255).

- Reference points: 2 %→140, 25 %→146, 50 %→168, 75 %→204, 100 %→255.
- The quadratic curve gives fine control near centre.
- The jump from 0 to the minimum working duty at the deadband edge is deliberate. Below ~140 the
  actuator does not move (bench, `Constants.h`).
- The deadband check is repeated here, although `getSteering()` already applies it, so the
  function stays correct on its own and in the native test.
- Input is the value `processMavlinkCommands()` already computes: `applySteeringScale(getSteering())`.
  So speed scaling applies to the rate as well (D12).
- Sign convention: the same as `setSteeringPercent()`, + = right = positive motor speed.

### D4 — Rate drive = continuous jog with variable duty, owned by RATE
This is the minimal implementation, reusing jog. New method
`bool SteeringController::setRateCommand(float pct)`:
1. Refuse (return false, no motion) if `sensorOk_` (defence in depth: rate is never allowed with a
   healthy sensor), if `!steerDriverOk_`, or while not armed (D5).
2. `duty = steerRateDuty(...)`. If 0, stop the drive only if the active jog is RATE-owned, then
   return true.
3. If a WEB-owned jog/nudge is active (refreshed within `STEER_JOG_TIMEOUT_MS` / pulse not ended),
   ignore the command (return true). Web jog has priority (D6).
4. Otherwise it is the `jog(dir, |duty|)` path with `jogOwner_ = RATE`:
   - The stall-latch check `latchBlocks()` refuses the latched direction for
     `STEER_STALL_COOLDOWN_MS`, and an accepted command clears the latch.
   - `lastDriveDir_` is set.
   - `lastJogRefresh_` is refreshed.
   - `motor_.setSpeed(±duty)`.

   `jog()` gains an internal owner parameter. The existing web `jog()`/`nudge()` set
   `jogOwner_ = WEB`.

Refresh and timeout:
- `VehicleController` calls `setRateCommand()` only when a NEW `SERVO_OUTPUT_RAW` frame has
  arrived since the last call. This uses the frame counter of "Servo Command Frame Counter".
- Re-reading the last decoded channel every loop would otherwise refresh the jog indefinitely on
  stale data.
- With no new frame for `STEER_JOG_TIMEOUT_MS` (500 ms), the existing jog timeout in `update()`
  stops the motor.
- The autopilot must therefore stream `SERVO_OUTPUT_RAW` at ≥ 5 Hz. The normal rate is about
  25 Hz: the firmware already requests `MAVLINK_SERVO_OUTPUT_RATE_HZ` = 25 Hz and re-requests it
  below 10 Hz (`SRx_RC_CHAN` per `MAVLINK_SETUP.md`). Below about 2 Hz the motion
  stutters.

### D5 — Centre-to-arm
- On every entry into RATE, the rate path is disarmed. It arms when one fresh frame shows the
  stick inside the deadband (`steerRateDuty(...) == 0`).
- Until then `setRateCommand()` refuses non-zero commands.
- The disarmed state is visible as the serial line `[STEER] rate mode: centre the stick to arm`,
  logged once per entry.
- Why: a sensor that dies while the operator holds 50 % stick would otherwise turn a held
  position into a continuous rotation toward the lock.
- Confirmed by the user. It costs one stick-centre gesture.

### D6 — Web jog/nudge keep working and win over rate commands
- Today a web jog/nudge works in every input source, including while MAVLink is in charge,
  because MAVLink percent commands are refused without a sensor.
- In rate mode a centred stick sends duty 0 at ~25 Hz. If that stopped any jog, web jog would be
  unusable whenever MAVLink is active.
- So the rate path never stops or overrides a WEB-owned jog/nudge. It resumes when that jog ends
  (release, 500 ms timeout, or the pulse ends).
- The web jog stop command (`jog(0)`) stops whatever jog is active, as today.

### D7 — No extra hysteresis; reuse the sensor debounce
- POSITION→RATE uses the existing immediate fault. RATE→POSITION uses the existing 5-good-frames
  streak.
- A sensor that flaps produces stop-start motion (each RATE→POSITION stops the rate drive, and
  the next fault re-enters RATE disarmed).
- That is safe, because each transition stops the motor and re-entry needs a re-centre. It is
  reported, not hidden: one serial line per transition, and `STATUSTEXT` is rate-limited by the
  existing `MAVLINK_STATUSTEXT_MIN_MS`.
- No new timer or parameter is added.

### D8 — What is bypassed in rate mode, explicitly
- Position stall detection (`STEER_STALL_THRESHOLD`/`STEER_STALL_TIMEOUT`): it runs only while
  `sensorOk_`, and rate requires `!sensorOk_`. It cannot false-trigger.
- Anti-stall boost, move timeout `STEER_MOVE_TIMEOUT`, position tolerance: closed-loop only.
- Software limits and the ±550° range guard: no position is available.
- **Still active**:
  - VESC link failsafe: no telemetry for `STEER_VESC_COMM_TIMEOUT_MS` → stop and reject.
  - VESC fault code → `triggerStallStop(lastDriveDir_)` + 700 ms latch.
  - The stall latch refusal of the latched direction.
  - The 500 ms refresh timeout.
- End stops are left to the VESC's own *Motor Current Max* clamp and *Absolute Max Current*
  fault. No firmware current-based stop is added (unchanged policy: "the firmware SHALL NOT run
  its own sustained-over-current timer").

### D9 — Indication
- Web telemetry: `steer_mode` string, always present.
- "Кермо: стан": new tile "Режим" / "Mode" with localized labels:
  - `position` "Позиція" / "Position";
  - `rate` "Швидкість (без датчика)" / "Rate (no sensor)" in the danger colour;
  - `blocked` "Заблоковано" / "Blocked" in the warning colour.
- "Увага": a new line "Кермо без датчика: режим швидкості" / "Steering without sensor: rate mode"
  (danger colour) while `steer_mode == "rate"`. It is in addition to the existing sensorless line.
- MAVLink:
  - `NAMED_VALUE_INT` (id 252) named `STEER_MODE`, value 0 = position, 1 = rate, 2 = blocked.
    It is sent on the existing 1 Hz slow steering timer and once immediately on every change.
  - It is a different message id from the six `NAMED_VALUE_FLOAT`, so the "exactly six
    NAMED_VALUE_FLOAT" exception is untouched. Mission Planner decodes `NAMED_VALUE_INT` from
    `common.xml` natively.
  - `STATUSTEXT` on every change: `Steer: RATE mode (no sensor)` with severity WARNING;
    `Steer: position mode` with severity INFO; `Steer: blocked (no sensor)` with severity
    WARNING.
- Serial: D2.

### D10 — Web slider lock
- While `steer_mode !== 'position'`, `updateControlsAvailability()` additionally disables
  `#steering-slider` and the three `steer-test-*` buttons.
- It shows a hint under the slider:
  - rate: "Без датчика кута кермо керується стіком як швидкістю (тільки MANUAL). Повзунок
    вимкнено, Jog/Nudge працюють."
  - blocked: "Немає датчика кута: повзунок вимкнено, використовуйте Jog/Nudge у Калібруванні."
- The firmware `set_steering` handler also answers
  `"Steering rejected: no steering sensor — use Jog/Nudge"` when the sensor is unhealthy
  (defence in depth). The command was already refused by `setSteeringPercent()`.
- Throttle/brake/gear and "take control" are unchanged.

### D11 — Failsafe and boot centring
- `applyFailsafe()` still calls `setSteeringPercent(0)`, which is refused without a sensor.
- Entering FAILSAFE makes the mode BLOCKED (source ≠ MAVLink), so the D2 action `stopRate()`
  stops any rate drive in the same loop. Failsafe never centres without a sensor.
- The MAVLink signal loss that causes FAILSAFE also stops new frames, so the 500 ms jog timeout
  is a second backstop.
- Boot centring keeps its existing gate (`sensorOk_`).

### D12 — Speed scaling applies to the rate (user decision)
- The rate path takes the speed-scaled stick value (`applySteeringScale(getSteering())`), the same
  value position mode uses.
- So full stick gives duty 255 only at or below the scaling base speed. Above it, the deflection,
  and therefore the rate, shrinks by `min(1, base / speed)`. That makes it safer at speed.
- `STEER_SCA` / `steer_scale` keep reporting the real applied scale; their semantics are unchanged.
  The existing fault rules still apply: an invalid speed, no base, or a non-MANUAL mode means
  scale 1.
- No change to "Speed-Scaled Steering Authority" or "Steering Scale Telemetry via
  NAMED_VALUE_FLOAT" is needed: rate mode is the autopilot command path in MANUAL, which those
  requirements already scale.

## Risks / Trade-offs
- **Open loop, no position limits.**
  - Nothing stops the wheels at the lock except the VESC current clamp/fault. The operator must
    watch the wheels.
  - Full stick at duty 255 into a lock stresses the rack until the VESC limits current.
  - Mitigations: MANUAL-only, centre-to-arm, 500 ms dead-man on frame refresh, help text and the
    attention line.
- **The sensor's ±600° latch cannot be protected against.** Rate mode only exists while the
  sensor is already unhealthy. If the cause was the latch, the sensor stays dead until a power
  cycle. If the cause was a timeout, rotating far in rate mode may additionally latch it.
  Accepted.
- **Direction reversal at full duty.** Stick hard-left to hard-right reverses the VESC duty
  directly, as web jog can today. The VESC's own duty ramp and current limits soften it.
  A bench check is in tasks.md.
- **Flapping sensor** (D7): stop-start motion and a burst of STATUSTEXT. Visible and safe.
- **Low SERVO_OUTPUT_RAW rate**: below ~2 Hz the rate drive stutters (500 ms timeout). This is
  documented as a commissioning requirement (≥ 5 Hz).
- **Unknown autopilot mode counts as MANUAL.** If the heartbeat is stale but `SERVO_OUTPUT_RAW`
  still arrives, rate mode is allowed. This matches the existing scaling rule. An autonomous mode
  with a lost heartbeat while `SERVO_OUTPUT_RAW` continues is considered implausible on one
  serial link.

## Migration
None. No NVS, wiring or calibration change. Firmware 1.5.0.

## Open Questions
None.
