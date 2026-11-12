# Design: add-aztec-parking-brake

## Context
- **Hardware (owner).** Hand-brake lever with a cable to the rear caliper. A DC linear actuator that
  pulls the cable is installed and wired through a BTS7960 on X6:
  - X6 pin 2 = L_PWM = GPIO4, X6 pin 3 = R_PWM = GPIO5, X6 pins 4/5 = L_IS / R_IS → CD4051 mux
    CH3 / CH0;
  - the BTS7960 enables are hard-wired, as on X7;
  - the actuator has internal limit switches at both ends and is self-locking;
  - its motor supply is on the permanent battery feed (not ACC, not the ignition contactor), so it
    can move in any ignition state;
  - the owner estimates the full travel at about 2 s; travel time and polarity are not measured
    yet.
- **Code today (verified).**
  - `BTS7960Controller` (`src/BTS7960Controller.cpp`) configures `LEDC_TIMER_1` once, shared by every
    instance through a static flag, at `MOTOR_PWM_FREQ` (10 kHz), 8-bit, low-speed mode.
  - `setSpeed(+n)` drives RPWM, `setSpeed(-n)` drives LPWM, and `stop()` sets both to 0. A
    `setRPWM` / `setLPWM` clears the opposite channel first.
  - LEDC channels in use: 1 (throttle, `LEDC_TIMER_0`, 50 Hz), 2 (transmission, `LEDC_TIMER_0`), and
    4 / 5 (X7 brake RPWM / LPWM, `LEDC_TIMER_1`). Nothing in `src/` or `include/` uses channels 0, 3,
    6 or 7. `Constants.h` and the pinout doc call 6 / 7 "free for future use".
  - Gear: `transmission_.getTargetGear()` is the commanded gear. `shouldClipThrottle()` caps the
    operator's throttle at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (8.5 %) while a gear change is active,
    or the target is N or UNKNOWN. `isThrottleLockedForShift()` (boost PID or shift idle hold) skips
    the operator throttle entirely.
  - Throttle paths:
    - MAVLink: `processMavlinkCommands()` → `throttle_.setThrottlePercent(applySpeedLimit(clipped))`.
    - Web: `update()` re-applies `webThrottleDemandPct_` the same way every loop.
    - `processThrottleCommand()` stores the demand and applies it.
    - `ThrottleController::setThrottlePercent()` applies the opening slew; `idle()` bypasses it.
  - Speed: `SpeedSensor::getSpeedMs()` decays to 0 after 2 s of silence. `isValid()` is false until
    the first pulse since boot, and while a mid-motion pulse loss is latched (`isSuspicious()`). The
    gear interlock (`canChangeGear`) trusts the reading when it is valid or suspicious.
  - GCS: `src/main.cpp` assembles `report.digitalFlags` from `EFI_DIGITAL_FLAG_WHEEL_LOCK` (0x01) and
    `EFI_DIGITAL_FLAG_FRONT_LIGHT` (0x02). `MavlinkInterface` sends it as a float in
    `EFI_STATUS.pt_compensation`, and bits 2–7 are documented as unassigned.
  - Web telemetry: `StaticJsonDocument<9216>`. The comment in `src/WebPortal.cpp` puts the aztec
    worst case at ≈ 8.7 KB, about 0.5 KB under the limit.

## Goals / Non-Goals
- Goals:
  - operator-commanded apply / release;
  - no automatic apply;
  - a refusal to apply while moving;
  - a drive-away release that keeps the engine at idle until the brake is off;
  - a state that survives reboot and never claims certainty after an interrupted run;
  - web control now, and a command entry point the GCS path can call later.
- Non-Goals:
  - position or current feedback;
  - a GCS command path;
  - an RC channel;
  - changes to the X7 service brake;
  - anything on `master`.

## Decisions

### D1. Own class, pure law, VehicleController owns the policy
- `ParkBrakeController` (`include/ParkBrakeController.h`, `src/ParkBrakeController.cpp`) owns:
  - a reference to a second `BTS7960Controller` (`parkBrakeActuator`, created in `main.cpp` next to
    `brakeActuator`);
  - the run timer, the reverse dwell, the state and the NVS namespace `"pbrake"`.
- `VehicleController` owns `ParkBrakeController`, calls its `update()` once per loop (next to
  `updateBrakeControl()`), and owns the cross-cutting policy: the speed interlock, the drive-away
  trigger and the throttle hold.
- The arithmetic and decisions live in `include/ParkBrakeLaw.h` (header-only, no Arduino), with
  native tests:
  - stored-state decode;
  - command transitions;
  - run duration;
  - travel validation;
  - the apply speed check;
  - the drive-away edge and throttle-hold predicates.

This keeps `VehicleController.cpp` (2335 lines) from growing a second actuator loop.

### D2. Pins, LEDC channels, PWM
- `PIN_PARK_BRAKE_LPWM` = GPIO4 (X6 pin 2), `PIN_PARK_BRAKE_RPWM` = GPIO5 (X6 pin 3).
- `LEDC_CH_PARK_BRAKE_RPWM` = 6 and `LEDC_CH_PARK_BRAKE_LPWM` = 7. These are the former steering
  channels, free in code and documented as "free for future use". Channel 3 stays free, and so
  does 0, which is unlisted in the doc.
- Both channels use `LEDC_TIMER_1`. `BTS7960Controller::begin()` reuses the already configured timer
  (static flag), so the X6 bridge runs at the same 10 kHz, 8-bit as X7. No new timer is used.
- Drive duty `PARK_BRAKE_DRIVE_DUTY` = 255 (full).
- Direction mapping:
  - `invert = 0`: APPLY = `setSpeed(+255)` (RPWM, GPIO5) and RELEASE = `setSpeed(-255)` (LPWM, GPIO4);
  - `invert = 1` swaps them.
  - The mapping lives in one function, so the bench result only flips a stored bit.

### D3. End of travel by time; no hold
- A run lasts `travel_ms + PARK_BRAKE_TRAVEL_MARGIN_MS` (500 ms) at full duty. Then `stop()` (both
  PWM 0) and the state becomes `APPLIED` / `RELEASED`.
- Overdriving is harmless: the internal limit switch opens the motor circuit, so the bridge drives
  an open load for the remainder.
- There is no hold PWM, because the actuator is self-locking. Unlike X7, no stall current flows at
  rest.
- `travel_ms` range is 1000–5000 ms (owner decision, 2026-11-12), default 2000 (owner's
  estimate), so a default run lasts 2500 ms and the longest possible run 5500 ms. The 5000 ms cap
  leaves 2.5× headroom over the estimate for a slower-than-expected actuator. It also bounds the
  drive-away idle hold and the time the bridge can drive into an open limit switch. An over-long
  value only lengthens the release run (and, at a 100 % release point, the drive-away idle hold). A too-short one would
  leave the cable half-pulled and is the dangerous direction, so the 500 ms margin also covers an
  estimate that is slightly low. The bench task measures both directions, and the longer one,
  rounded up, becomes the setting.
- The run timer uses `millis()` deltas, so wrap is safe. It is advanced only in `update()`. A loop
  stall lengthens the run, which is harmless for the same reason.

### D4. State machine and commands

| State now | APPLY (speed OK) | APPLY (moving) | RELEASE |
|---|---|---|---|
| UNKNOWN | start apply run | refused | start release run |
| RELEASED | start apply run | refused | re-run release (re-assert) |
| APPLIED | re-run apply (re-assert) | refused | start release run |
| APPLYING | ignored ("already applying") | ignored | stop → 200 ms dwell → release run |
| RELEASING | stop → 200 ms dwell → apply run | refused (release continues) | ignored ("already releasing") |

- A repeated same-direction command during a run does not restart the timer, so a held button or a
  chatty future GCS cannot stretch a run forever.
- A reversal always runs the full travel plus margin. The position at the turn is unknown, and the
  full run is guaranteed to reach the end.
- The dwell `PARK_BRAKE_REVERSE_DWELL_MS` = 200 ms at zero duty spares the motor and bridge a
  full-voltage plug reversal. `BTS7960Controller` would already prevent shoot-through.
- Entry point: `ParkBrakeResult VehicleController::commandParkBrake(ParkBrakeRequest req,
  ParkBrakeSource src)` with `ParkBrakeSource` = `WEB` today (`GCS` reserved). Results:
  - `STARTED`, `STARTED_SPEED_UNKNOWN`, `ALREADY_RUNNING`;
  - `REFUSED_MOVING` (carries the speed);
  - `REFUSED_DRIVER`, `REFUSED_NVS` (apply only), `REVERSING`.
  - The web layer maps each result to a reply text. A future MAV_CMD_USER_2 handler maps them to
    `MAV_RESULT_ACCEPTED` / `TEMPORARILY_REJECTED` / `DENIED` / `FAILED`.
- Not gated by input source, ignition state, web control or fail-safe. Same as the lock: the park
  brake is a parking function, not a driving command. Accepting commands with the ignition OFF is
  correct because the actuator is on permanent battery power (D11).
- Refused only when the X6 driver failed to initialise (`REFUSED_DRIVER`, "Park brake driver not
  available").

### D5. Drive-away release (rising edge) and throttle hold
- The operator demand is the value each throttle path already computes after the existing gear clip
  (`shouldClipThrottle()` → ≤ 8.5 %) and before the speed limiter, the slew and this hold:
  - MAVLink: `mavlink_.getThrottle()` clipped — everything arriving from the Pixhawk, manual or
    autopilot (D11);
  - web: `webThrottleDemandPct_` clipped.
  - In FAILSAFE there is no operator demand, so the drive-away does not trigger.
- **Trigger.** All of these must hold:
  - state ∈ {APPLIED, UNKNOWN};
  - ignition on: `relayController_.getIgnitionState() != OFF` (D12-B);
  - target gear ∈ {L, H, R};
  - no gear change is active and the throttle is not locked for a shift;
  - demand > `PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT` (10 %);
  - the drive-away latch is armed.

  When they hold, a RELEASE starts with source "drive-away", the same as an operator RELEASE.
- **Latch.**
  - Armed whenever the brake is APPLIED / UNKNOWN, the ignition is on and the demand is ≤ 10 %.
    Disarmed when a release starts and whenever the ignition is OFF (D12-B).
  - It stops an apply → release ping-pong when the demand is already above 10 % as the apply
    completes, for example an autopilot holding throttle, or an operator who applies with a finger
    on the slider. In that case the brake stays applied and the throttle goes to idle; the operator
    must drop to 10 % or below and press again (D11).
  - In N the demand is clipped to ≤ 8.5 %, so the latch arms there. Shifting into L with the stick
    held up then counts as a press: drive-away after the shift completes.
- **Hold.** While target gear ∈ {L, H, R} and the state is APPLIED, APPLYING or UNKNOWN, or
  RELEASING before the throttle release point (D12-A), the operator path calls `throttle_.idle()`
  instead of `setThrottlePercent(applySpeedLimit(demand))`.
  - `applySpeedLimit()` is still evaluated on the demand, so the limiter state does not freeze.
  - Gear-change ownership (boost PID, shift idle hold) and throttle calibration are untouched: they
    already bypass the operator path.
  - `pbrake_thr_hold` reports the hold.
  - The web reply to `set_throttle` stays a success but reads "Throttle set — held at idle until the
    park brake is released".
- **From the throttle release point** (D12-A; at 100 % from the end of the run), the next loop
  passes the demand normally. `setThrottlePercent()` rises from idle at the opening slew (default
  60 %/s), so there is no snap.
- **N.** No trigger and no hold. Revving in place keeps today's 8.5 % clip.
- **R.** Treated like L/H. A target gear UNKNOWN (transmission not initialised) is neither: no
  trigger and no hold, and the existing 8.5 % clip applies.
- **UNKNOWN is treated as possibly applied.** It holds the throttle and is eligible for drive-away.
  Driving against a half-applied brake is the failure we must not allow. A release is never unsafe
  under decision 1, which bans only automatic apply. This is what "unknown until the first operator
  command" means in practice: the first release, whether by button or by throttle, settles it.

### D6. Apply speed interlock
- Threshold `PARK_BRAKE_APPLY_MAX_SPEED_MS` = 0.833 m/s (3 km/h).
- Speed source: `speedSensor_.getSpeedMs()` when `isValid() || isSuspicious()`. This is the same
  trust rule as the gear interlock, and a suspicious reading is a decaying upper bound.
- If the sensor has never pulsed since boot (not valid, not suspicious), there is no evidence of
  motion. The usual case is a buggy standing since power-up. APPLY is allowed and the reply notes
  "(speed unknown)" (D11). The CAN speed field is dead (PID 0x0D
  disabled), so it is not used.
- Reporting:
  - web reply `false`, "Park brake apply refused: moving at N.N km/h (max 3.0)", shown in the
    card's status line in the danger colour;
  - a `DebugFeature::BRAKE` log line;
  - the state is unchanged, and a running release continues.

### D7. Persistence (NVS `"pbrake"`)
- Keys:
  - `state` (uint8): 0 unknown, 1 released, 2 applied, 3 applying, 4 releasing;
  - `travel_ms` (uint16);
  - `thr_pass` (uint8, throttle release point, % of `travel_ms`; D12-A);
  - `invert` (uint8).
- A run start writes 3 or 4 before the bridge is driven. A run end writes 1 or 2 after `stop()`.
  That is two small writes per operation, which is negligible wear.
- Boot: 1 → RELEASED and 2 → APPLIED. Anything else (3, 4, missing, out of range) → UNKNOWN, logged,
  and not rewritten. A power loss, OTA reboot or watchdog reset mid-run therefore comes back
  UNKNOWN. The bridge is initialised to zero duty (`begin()` → `stop()`), and nothing is driven
  until a command or a drive-away.
- Write failure at run start:
  - APPLY is refused (`REFUSED_NVS`, "Failed to save the park brake state — apply refused"). An apply
    whose start was not recorded could come back after a reboot as a confident RELEASED while the
    brake is on.
  - RELEASE proceeds with the reply suffix "(state not saved)". A stale APPLIED / UNKNOWN after a
    reboot only holds the throttle and is cleared by the next release.
- Write failure at run end: logged. RAM state is correct, and the stored moving value reads as
  UNKNOWN after a reboot (safe).
- `travel_ms` / `thr_pass` / `invert`:
  - persist first, then apply;
  - a stored out-of-range travel or release point is logged and ignored (default used, not rewritten), the same rule
    as the brake card.
- `invert` change:
  - refused while a run is in progress ("Park brake direction change refused: actuator moving");
  - otherwise persists `invert`, then sets and persists state UNKNOWN, because the old state was
    measured with the old polarity.

### D8. Interactions
- **Service brake (X7).** Fully independent: separate driver, timer channel pair, state and
  commands. `brake_pct` and the "🛑 Гальмо" card are unchanged.
- **Fail-safe** (`applyFailsafe()`):
  - does not touch the park brake: no apply and no stop;
  - a run in progress completes to its end so that the state stays determinate (D11);
  - the throttle is already idle in fail-safe;
  - `relayController_.allOff()` is unrelated.
- **Ignition / relays / lock.** Independent. Commands work with the ignition OFF; the actuator's
  motor supply is on permanent battery power, so a timed run with the ignition OFF really moves it
  (D11).
- **Speed limiter.** Unchanged; it keeps evaluating the demand under the hold (D5).
- **Throttle open slew.** Unchanged; it shapes the ramp after the release.
- **Gear boost / shift idle hold.** These own the throttle during a shift exactly as today. The hold
  acts on the operator path only, and a shift with the brake applied is allowed.
- **Transmission interlock.** Unchanged. The park brake does not block gear changes.
- **Web control / input-source arbitration.** Unchanged. The park-brake commands bypass the
  `currentInputSource_ != WEB` gate, like `set_wheel_lock`.

### D9. Telemetry
- Web: seven scalar keys, about 112 B of document memory and ≤ 190 B of JSON. The aztec worst case
  becomes about 8.9 KB, still about 0.3 KB under 9216. The size is unchanged and the capacity
  comment is updated. `pbrake_state` and `pbrake_da_wait` are `const char*` literals (no `String` copy).
- GCS:
  - bit2 `EFI_DIGITAL_FLAG_PARK_BRAKE` (0x04) is set when the state is APPLIED;
  - bit3 `EFI_DIGITAL_FLAG_PARK_BRAKE_UNSETTLED` (0x08) is set when the state is APPLYING,
    RELEASING or UNKNOWN.
  - Both 0 means RELEASED. Two bits are needed because a single bit cannot tell "released" from
    "unknown / moving", and the GCS operator needs that difference before driving.
  - The value on the wire is now 0–15. Decoders that test bits 0/1 with a mask are unaffected
    (verified by reading the `gcs` sources: the MP plugin uses `FLAG_* & flags`, and QGC rounds the
    float to an int).

### D10. Web card
- Placed in the Tests section after "Vehicle Ignition & Lighting", where the lock and light live. It
  holds both the operator buttons and the two tuning controls, so the bench procedure is on one
  card.
- Apply and Release use `confirmTwice()`:
  - Apply warns "Стоянкове гальмо буде затягнуто. Газ на передачі — лише холостий." / "The parking
    brake will be applied. Throttle in gear stays at idle.";
  - Release warns "Стоянкове гальмо буде відпущено — на схилі багі може покотитися." / "The parking
    brake will be released — on a slope the buggy may roll.";
  - armed labels "Підтвердити затягування" / "Confirm apply" and "Підтвердити відпускання" /
    "Confirm release".
- The direction toggle also uses `confirmTwice()`, with the warning "Напрямок зміниться, стан стане
  «невідомо»." / "The direction will change and the state becomes unknown."
- A throttle-release-point slider (20–100 %, step 5, default 50) shows "N %" and the matching time
  "≈ M ms" for the travel time in use, and sends once on release, like the travel slider (D12-A).
- Notes from `pbrake_da_wait` explain a throttle press that did nothing (ignition off / drop the
  throttle and press again), and "Throttle passing, brake still releasing" covers the second part
  of a release run (D12-B, D12-A).
- Without a WebSocket the card shows the existing `steer_no_connection` text and sends nothing.
- The buttons stay enabled during a run: an opposite press reverses, and a same press is answered
  "already …".

### D11. Owner answers (2026-11-12)
The open questions of the first draft were answered by the owner. They are decisions now, and the
rationale is kept.
1. **Actuator supply: permanent battery** (not ACC, not the ignition contactor). Commands are
   accepted in any ignition state, and a run with the ignition OFF really moves the actuator, so the
   timed state stays truthful. No ignition gate is added.
2. **Apply with a speed sensor that has not pulsed since boot: allowed**, with "(speed unknown)" in
   the reply. Without any pulse there is no evidence of motion, and the usual case is a buggy that
   has stood still since power-up. Refusing would block parking after every boot.
3. **Fail-safe during a run: the run completes.** Stopping mid-run would leave the state UNKNOWN.
   Re-applying after a drive-away release would be an automatic apply, which decision 1 excludes.
   The throttle is already idle in fail-safe.
4. **Drive-away needs a rising edge.** Applying with the throttle already above 10 % in gear keeps
   the brake applied and the throttle at idle. The operator must drop to 10 % or below and press
   again. This prevents an apply → release ping-pong with a held throttle or autopilot output.
5. **UNKNOWN is treated as possibly applied.** The throttle in gear is held at idle and the
   drive-away can release it. Driving against a half-applied brake is the failure to avoid, and a
   release is never an automatic apply.
6. **Any throttle source triggers the drive-away**: the web, and everything arriving from the
   Pixhawk over MAVLink, manual or autopilot. The rule is about the throttle reaching the engine,
   whoever commands it, and the rising edge (4) keeps it deliberate.
7. **Default travel time 2000 ms** (owner's estimate), still web-tunable over 1000–5000 ms (range set by the owner). The
   bench measurement of both directions remains (tasks 6.3).

### D12. Owner decisions on the drive-away logic (2026-11-12)
**A. The throttle passes part-way through the release run (anti roll-back on a slope).**
- A RELEASING run holds the operator throttle at idle (in gear) only until the **throttle release
  point**. That point is `thr_pass_pct` % of `travel_ms`, measured from the start of the release
  drive, so a 200 ms reverse dwell is not counted.
- From that point the throttle passes through the speed limiter and the opening slew, from idle,
  while the actuator finishes its run.
- Parameter:
  - NVS `"pbrake"` key `thr_pass` (uint8);
  - constants `PARK_BRAKE_THR_PASS_PCT_DEFAULT` 50, `PARK_BRAKE_THR_PASS_PCT_MIN` 20,
    `PARK_BRAKE_THR_PASS_PCT_MAX` 100;
  - web command `set_park_brake_thr_pass_pct`;
  - telemetry `pbrake_thr_pass_pct`;
  - a slider in the card, 20–100 %, step 5.
- 100 % keeps the original behaviour: the hold lasts until the run ends, margin included, not just
  until 100 % of the travel. Below 20 % is not offered, because the owner rejected "no hold at all".
- The rule applies to every RELEASING run: drive-away or operator Release, from APPLIED or from
  UNKNOWN. APPLIED, APPLYING and UNKNOWN (not releasing) keep holding idle in gear.
- Default numbers: travel 2000 ms → the throttle starts ramping at 1000 ms into the release drive,
  and the run ends at 2500 ms.
- In the second half of the run:
  - `pbrake_thr_hold` is false;
  - `pbrake_state` stays `"releasing"` and `pbrake_remaining_ms` keeps counting down;
  - the GCS "unsettled" bit3 stays 1 (bit2 0) until the run ends, because the actuator is still
    moving.
- An APPLY accepted after the throttle was passed reverses the run. The state becomes APPLYING,
  which is a hold state, so the operator throttle goes back to idle in the same loop (`idle()`
  bypasses the slew). The APPLY still passes the 3 km/h interlock. If the buggy is already moving
  faster, the APPLY is refused and the release continues.
- Rationale: on a slope, a full idle hold for travel + margin lets the buggy roll back once the
  brake lets go. Overlapping engine pull with the end of the release prevents that.

**B. The drive-away needs the ignition ON.**
- "Ignition on" = `RelayController::getIgnitionState() != IgnitionState::OFF`
  (`include/RelayController.h:222-224`, reached through `VehicleController::getIgnitionState()`,
  `include/VehicleController.h:396`).
- This is the firmware's own commanded state: `computeShadow()` energizes `RELAY_SET_IGNITION`
  exactly when that state is not OFF (`src/RelayController.cpp:517`), so ACC, IGNITION and CRANKING
  all count as ON. It also reads OFF while the relay board is unavailable. It is not engine RPM.
- Owner decision (2026-11-12): all three states in which the ignition relay is energized — ACC,
  IGNITION and CRANKING — count as "ignition on" for the drive-away release. Only OFF blocks it.
- With the ignition OFF:
  - throttle in gear above 10 % does not start a release;
  - the brake stays applied and the throttle stays at idle (the hold is unchanged);
  - the operator's explicit Release still works in any ignition state, because the actuator is on
    permanent battery power.
- Rising-edge interaction: the drive-away latch arms only while the ignition is ON (and the brake is
  APPLIED / UNKNOWN with demand ≤ 10 %), and it is disarmed whenever the ignition is OFF. A throttle
  already above 10 % when the ignition is switched on therefore needs a new rising edge: drop to
  ≤ 10 % and press again.
- How the operator sees it:
  - telemetry `pbrake_da_wait` = `"ign_off"` or `"rearm"` (else `"none"`);
  - card notes "Запалювання вимкнено — газ не відпускає стоянкове гальмо" / "Ignition off — throttle
    does not release the parking brake" and "Відпустіть газ нижче 10 % і натисніть знову" / "Drop
    the throttle below 10 % and press again";
  - web `set_throttle` reply with the ignition off: "Throttle set — held at idle: ignition off, the
    park brake stays applied";
  - a `DebugFeature::BRAKE` log line, rate-limited to one per blocked press.

**C. Unchanged and confirmed.**
- No dwell or debounce on the 10 % threshold: the release starts in the loop in which the demand
  crosses it.
- If the throttle drops during a release run, the run completes and the brake stays released. There
  is no automatic re-apply (decision 1).

## Risks / Trade-offs
- **Wrong travel time (too short).** The cable stops part-way and the firmware reports APPLIED.
  Mitigation: the 2000 ms default is the owner's estimate, not a measurement, so measuring travel
  both ways (bench task 6.3) comes before field use. The 500 ms margin covers a slightly low
  estimate. Current-based end detection is the real fix (follow-up).
- **Unknown polarity.** The first Release on the bench may tighten the cable. Mitigation: the
  bench step does Release from UNKNOWN with the wheels free, then the invert toggle.
- **Boot glitch on GPIO4/5.** The pins float between reset and `BTS7960Controller::begin()`.
  Mitigation: a bench task watches power-up, reset and OTA reboot. The actuator is self-locking, so
  a short twitch would not change the cable much. The same exposure already exists on X7.
- **Pulling against a partly released rear brake.** With the throttle passed at 50 % of travel,
  the engine pulls for about the second half of the travel (≈ 1 s at 2000 ms, plus the 500 ms
  margin in which the actuator is normally already at its stop) while the rear caliper is still
  partly clamped. This is the deliberate price of not rolling back on a slope. The opening slew
  keeps the pull gentle at first. Mitigation: the release point is tunable 20–100 % (100 % = no
  overlap), and a bench task tunes it on a slope.
- **Ignition relay state is the commanded state, not a measured one.** If Relay_1 is commanded on
  but the ECU is not actually powered, the drive-away can still release. This is harmless: a
  release is never an automatic apply, and the throttle is useless without a running engine.

## Migration Plan
- No migration. The `"pbrake"` namespace is new, and missing keys read as UNKNOWN / 2000 ms /
  normal direction.
- First drive after flashing: in gear, throttle is held at idle until a release has started and
  reached its throttle release point (1000 ms at the defaults), or until a Release has completed.
- Rollback: flash 1.5.28. The `"pbrake"` namespace is ignored, and the actuator stays where it is.

## Follow-ups (out of scope)
- End of travel by motor current through the X6 IS lines (CD4051 CH3 / CH0; mux control PCA9557 U2
  @ 0x1C, not driven today): stall detection, travel measurement and fault reporting.
- GCS → board park-brake command via `MAV_CMD_USER_2`, calling `commandParkBrake(…, GCS)`, plus the
  GCS button. Decode of bits 2–3 in the MP plugin / QGC (`gcs` repository).
- Event-journal entries for park-brake runs and refusals.
- Spec clean-up: the old `vehicle-systems` "Brake Control System" scenario "Brake hold on startup"
  (service brake to 30 % at power-on) and "Vehicle State Coordination" ("parking brake is applied
  (30%)") describe an automatic parking function that was never implemented. They contradict
  decision 1 in spirit. They concern the X7 service brake and are left untouched here.

## Open Questions
None left. All of the owner's questions were answered on 2026-11-12 (D11, D12). The real travel
time, the tighten polarity and the release point on a slope are bench measurements (tasks 6.2, 6.3,
6.5b), not open decisions.
