## 0. Preconditions
- [x] 0.1 `replace-aztec-steering-sensor-with-bosch-lws` and `redesign-web-ui-service-console` are implemented on `aztec` (they are; firmware 1.4.0). This change is archived AFTER both, because its MODIFIED blocks quote their deltas.

## 1. Pure logic (host-testable)
- [x] 1.1 Create `include/SteerRate.h` (no Arduino / `Constants.h` dependency):
  - `enum class SteerMode : uint8_t { POSITION = 0, RATE = 1, BLOCKED = 2 }`
  - `steerModeToken()` → `position` / `rate` / `blocked`
  - `steerModeSelect(sensorOk, sourceIsMavlink, apModeKnown, apModeIsManual)` (design D1)
  - `steerRateDuty(pct, deadband, minDuty, maxDuty)` (design D3: deadband → 0, `min + (max−min)·d²`, rounded, signed, |pct| clamped to 100)
- [x] 1.2 Add `test/test_steer_rate/test_main.cpp` (Unity). It covers:
  - the deadband edge (1.99 → 0, 2.0 → 140);
  - the reference points 25/50/75/100 % → 146/168/204/255;
  - sign symmetry and clamping of ±150 %;
  - all `steerModeSelect` combinations, including "healthy sensor → POSITION regardless of source/mode" and "unknown mode → RATE".
- [x] 1.3 Extend `[env:native]` in `platformio.ini` with the multi-line `test_filter` form (one suite name per indented line; the single-line space-separated form did not select both suites), and update its comment:
  ```ini
  test_filter =
      test_lws305
      test_steer_rate
  ```

## 2. Steering controller (`SteeringController.h/.cpp`)
- [x] 2.1 Add a jog owner (`WEB` / `RATE`). `jog()` and `nudge()` from the web set `WEB`; the internal rate path sets `RATE`. `stop()` clears it.
- [x] 2.2 Add `bool setRateCommand(float pct)`. It:
  - refuses while `sensorOk_`, while `!steerDriverOk_`, or while disarmed with nonzero duty;
  - on duty 0, arms the path and stops only a RATE-owned drive;
  - ignores the command while a WEB-owned jog/nudge is active;
  - otherwise goes through the jog path (stall-latch check, `lastDriveDir_`, `lastJogRefresh_`, `setSpeed(±duty)`) with duty from `steerRateDuty(pct, RC_STEERING_DEADBAND, STEER_PWM_MIN_DUTY, STEER_PWM_MAX_DUTY)`.
  - It cancels a pending boot centering like any accepted command.
- [x] 2.3 Add `void stopRate()` (stops only a RATE-owned drive) and `void disarmRate()`. Add a `bool isRateArmed()` accessor for logging.
- [x] 2.4 In `update()`, the jog log suffix shows `" rate"` for a RATE-owned drive. Confirm by code reading that the position stall check (`sensorOk_ && …`), the boost, the move timeout and the range guard never run for a RATE-owned drive. Add a comment stating that stall detection is bypassed in rate mode by design.
- [x] 2.5 Update the class Doxygen header (mode summary, rate drive, ownership).

## 3. Vehicle layer (`VehicleController.h/.cpp`)
- [x] 3.1 Compute `steerMode_` every `update()` before `processMavlinkCommands()` with `steerModeSelect(steering_.isSensorOk(), currentInputSource_ == MAVLINK, mavlink_.hasAutopilotMode(), mavlink_.getAutopilotCustomMode() == ROVER_CUSTOM_MODE_MANUAL)`. Expose `getSteerMode()`.
- [x] 3.2 On a mode change:
  - log one line `[STEER] mode: <old> -> <new> (<reason>)`, with reason = sensor token / sensor OK / source / autopilot mode;
  - call `steering_.stopRate()` when leaving RATE;
  - call `steering_.disarmRate()` and log `centre the stick to arm` when entering RATE.
- [x] 3.3 In `processMavlinkCommands()`:
  - in RATE mode, call `steering_.setRateCommand(applySteeringScale(mavlink_.getSteering()))` only when `mavlink_.getCommandCount()` differs from the last value seen;
  - in POSITION and BLOCKED mode, keep `setSteeringPercent()` as today.
- [x] 3.4 Failsafe: verify that entering FAILSAFE switches the mode to BLOCKED and calls `stopRate()` in the same iteration. `applyFailsafe()` keeps `setSteeringPercent(0)`, which is refused without a sensor.
- [x] 3.5 `processSteeringCommand()`: when `!steering_.isSensorOk()`, respond `"Steering rejected: no steering sensor — use Jog/Nudge"`. Keep the existing text for the uncalibrated case.

## 4. MAVLink (`MavlinkInterface.h/.cpp`, `src/main.cpp`)
- [x] 4.1 Add `uint32_t getCommandCount() const` returning `totalCommands_`. Check that it increments only on accepted `SERVO_OUTPUT_RAW` command frames.
- [x] 4.2 Add `uint8_t steerMode` to the vehicle-state report struct, filled in `main.cpp` from `VehicleController::getSteerMode()`.
- [x] 4.3 Send `NAMED_VALUE_INT` `STEER_MODE` (compile-time literal, ≤ 10 chars, zero-padded) on the 1 Hz slow steering timer, and once immediately when the value changes.
- [x] 4.4 Send a `STATUSTEXT` on a mode change: `Steer: RATE mode (no sensor)` (WARNING), `Steer: blocked (no sensor)` (WARNING), `Steer: position mode` (INFO). It goes through the existing change detection and `MAVLINK_STATUSTEXT_MIN_MS` rate limit.
- [x] 4.5 Update the comment block at the named-value packing: `STEER_MODE` is a `NAMED_VALUE_INT`, and the six `NAMED_VALUE_FLOAT` stay the only floats.

## 5. Web telemetry and UI
- [x] 5.1 `TelemetryManager`: add `steer_mode` (static token). `WebPortal.cpp`: emit `doc["steer_mode"]`. Re-check the JSON document capacity comment and the size.
- [x] 5.2 `data/index.html` "Кермо: стан":
  - add the "Режим" / "Mode" tile after "Датчик" / "Sensor", with localized labels and colours (design D9);
  - show `--` for a missing or unknown value.
- [x] 5.3 `data/index.html` "Увага": add the `att_steer_rate` line (danger colour) while `steer_mode === 'rate'`, with en/uk dictionary entries.
- [x] 5.4 `data/index.html` Тести → Ручне керування:
  - in `updateControlsAvailability()`, also disable `#steering-slider` and `steer-test-left/center/right` while the last `steer_mode !== 'position'`;
  - add a hint element under the slider with the rate / blocked texts (en/uk);
  - re-run the availability check on every telemetry frame that carries `steer_mode`.
- [x] 5.5 Help (ⓘ) texts en/uk:
  - "Кермо: стан": rate mode explanation;
  - Manual control: why the slider can be locked.

## 6. Docs
- [x] 6.1 `MAVLINK_SETUP.md`:
  - add `STEER_MODE` (`NAMED_VALUE_INT` 252, 1 Hz + on change, 0/1/2) to the outbound message table and the steering telemetry section;
  - add the three mode `STATUSTEXT` lines to the `STATUSTEXT` section;
  - add a short "Steering without the angle sensor (rate mode)" operator section. It covers: MANUAL only, centre to arm, release = stop within 500 ms, no end-stop protection except the VESC current limit, `SERVO_OUTPUT_RAW` ≥ 5 Hz.
- [x] 6.2 `include/Constants.h`: a comment in the steering section noting that the rate mode reuses `RC_STEERING_DEADBAND`, `STEER_PWM_MIN_DUTY`, `STEER_PWM_MAX_DUTY` and `STEER_JOG_TIMEOUT_MS`. No new tunable unless the bench requires one.

## 7. Version and changelog
- [x] 7.1 `include/Constants.h`: `FIRMWARE_VERSION` "1.4.0" → "1.5.0".
- [x] 7.2 `include/Changelog.h`: new first entry `{"version":"1.5.0","date":"<implementation date>"}`:
  - uk: "Без датчика кута кермо керується стіком як швидкістю: відхилення = обертання, центр = стоп; тільки в MANUAL", "Перед початком руху стік потрібно один раз поставити в центр", "Повзунок керма в веб-панелі блокується без датчика; Jog/Nudge працюють", "Повторне калібрування не потрібне"
  - en: "Without the angle sensor the stick steers by rate: deflection = rotation, centre = stop; MANUAL only", "Centre the stick once before the wheels will move", "The web steering slider is locked without the sensor; Jog/Nudge still work", "No recalibration required"

## 8. Build and test
- [x] 8.1 `pio run` builds without new warnings.
- [x] 8.2 `pio test -e native` passes (`test_lws305` and `test_steer_rate`).
- [x] 8.3 `pio run -t buildfs` (LittleFS image with the updated `index.html`).
- [x] 8.4 `openspec validate add-aztec-steering-rate-mode --strict`.

## 9. Bench verification [USER/MANUAL]
- [ ] 9.1 [USER/MANUAL] Sensor healthy, MANUAL: the stick steers by position exactly as before. `steer_mode` = position, `STEER_MODE` = 0.
- [ ] 9.2 [USER/MANUAL] Unplug the LWS (or its CAN) while in MANUAL with the stick centred. Expected:
  - `steer_mode` = rate, the "Увага" rate line appears, the Mode tile shows "Rate (no sensor)";
  - a `STATUSTEXT` "Steer: RATE mode (no sensor)" and `STEER_MODE` = 1 appear in Mission Planner;
  - one serial line is logged.
- [ ] 9.3 [USER/MANUAL] Centre-to-arm:
  - unplug the LWS while holding the stick deflected → the wheels do NOT rotate, and the serial log shows "centre the stick to arm";
  - centre the stick once, then deflect → rotation starts;
  - repeat after a mode round-trip (MANUAL → HOLD → MANUAL with the stick held deflected) → again no motion until centred.
- [ ] 9.4 [USER/MANUAL] Stick slightly off centre → slow rotation in the stick direction (right = right). Centre → stops. Full stick → full speed (duty 255; check `steer_motor_current` and the VESC log).
- [ ] 9.5 [USER/MANUAL] Hold the stick, then stop `SERVO_OUTPUT_RAW` (unplug TELEM). The wheels stop within 500 ms, then FAILSAFE: they stay stopped and do not centre.
- [ ] 9.6 [USER/MANUAL] Hold the stick into a lock. Only the VESC current limit holds it, and no firmware stall-stop is logged. Hard reversal left↔right at full stick is acceptable.
- [ ] 9.7 [USER/MANUAL] Switch the autopilot to HOLD/AUTO with the sensor unplugged. `steer_mode` = blocked, no motion on any steering output, and a running rate drive stops at once.
- [ ] 9.8 [USER/MANUAL] Back to MANUAL → rate again, after centring the stick.
- [ ] 9.9 [USER/MANUAL] Reconnect the LWS during a rate drive. The motor stops and the wheels hold. `steer_mode` = position. The next stick movement is a normal position move.
- [ ] 9.10 [USER/MANUAL] Web, sensor unplugged:
  - the slider and test buttons in Тести are locked with the hint, in both rate (MAVLink active) and blocked (web control taken);
  - Calibration Jog/Nudge work and are not cancelled by a centred stick;
  - take control → throttle/brake/gear still work.
- [ ] 9.11 [USER/MANUAL] Speed scaling in rate mode:
  - with the sensor unplugged, set a bench test speed above the scaling base (`set_test_speed`, or roll the wheel);
  - full stick → duty below 255, matching `min(1, base/speed)` applied to the deflection;
  - `steer_scale` / `STEER_SCA` show the same scale;
  - clear the override → full stick gives 255 again.
- [ ] 9.12 [USER/MANUAL] The web "What's new" shows 1.5.0, and the sidebar shows firmware 1.5.0.
