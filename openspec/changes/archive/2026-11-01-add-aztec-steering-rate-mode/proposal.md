# Change: Steering RATE mode when the Bosch LWS sensor is unhealthy (aztec)

## Why
Since `replace-aztec-steering-sensor-with-bosch-lws` (1.3.0), every percent steering command
(MAVLink stick, web slider and test buttons, boot centring, failsafe centring) is refused while the
LWS steering-angle sensor is unhealthy. Only web jog/nudge still move the wheels. So if the sensor
fails in the field (CAN unplugged, frame timeout, the ±600° latched fault, `not_calibrated`), the
operator on the RC transmitter loses steering completely. The vehicle cannot be driven home or
even off the track. There is no position feedback in that state. The stick can still steer the
VESC directly, though: deflection sets the direction and speed of rotation, and centre means stop.
The operator then closes the loop by watching the wheels, as with a sensorless jog.

## What Changes
- **New steering mode `rate`, selected automatically.** It needs no switch and no parameter. The
  firmware enters it when ALL of these hold:
  - the LWS sensor is unhealthy (`isSensorOk()` false, the existing fault/recovery debounce);
  - the active input source is MAVLink;
  - the autopilot is in Rover MANUAL, or its mode is unknown or stale. The firmware already treats
    an unknown or stale mode as MANUAL.

  In rate mode the steering channel (`SERVO_OUTPUT_RAW`, after the existing deadband and speed
  scale) is a **rate** command:
  - Inside `RC_STEERING_DEADBAND` (±2 %) the duty is 0, so the wheel stops.
  - Outside it, duty = `STEER_PWM_MIN_DUTY` + (255 − `STEER_PWM_MIN_DUTY`) × d². Here d (0..1] is
    the deflection beyond the deadband. Full stick gives duty 255 at or below the steering
    speed-scaling base speed. Above it, the existing speed scale shrinks the deflection and so the
    rate; `STEER_SCA` / `steer_scale` keep reporting the real scale. The sign is the stick
    direction (+ = right).
  - The rate drive is a continuous jog with variable duty. Each new `SERVO_OUTPUT_RAW` frame
    refreshes it. If no frame arrives for `STEER_JOG_TIMEOUT_MS` (500 ms), it stops.
- **Centre-to-arm.** After entering rate mode, the stick must be seen inside the deadband once
  before any rate motion. This stops a held stick from becoming a sudden continuous rotation when
  the sensor dies.
- **Other sensorless states stay blocked** (`blocked` mode):
  - Autonomous autopilot modes (not MANUAL) get no motion.
  - Web as the input source: the slider is locked, jog/nudge still work.
  - Failsafe: the motor stops (duty 0) and never centres.
  - Boot centring stays blocked without a sensor.
- **Return to `position` mode** happens automatically once the sensor is healthy again (existing
  `STEER_LWS_RECOVER_FRAMES` streak). At that moment the rate drive stops and the wheel holds where
  it is. No target is set. The next percent command is accepted normally.
- **No firmware stall stop in rate mode.** Position stall detection, the anti-stall boost, the move
  timeout and the range guard need a position, so they are bypassed. End stops are left to the
  VESC current limit. The existing VESC-fault stop and stall latch still apply.
- **Web jog/nudge keep priority.** While a web jog or nudge is active, rate commands are ignored.
- **Indication**:
  - Telemetry key `steer_mode` (`position` | `rate` | `blocked`).
  - A "Режим" / "Mode" tile in "Кермо: стан".
  - A new "Увага" line: "Кермо без датчика: режим швидкості".
  - MAVLink `NAMED_VALUE_INT` `STEER_MODE` (0/1/2) at 1 Hz and immediately on change, plus a
    `STATUSTEXT` on each transition.
  - A serial console line on each transition.
- **Web**: while `steer_mode` is not `position`, the steering slider and the Left/Center/Right
  test buttons in Тести → Ручне керування are locked and a hint explains why. Jog/Nudge and the
  web "take control" for throttle/brake/gear are unchanged.
- **Version**: `FIRMWARE_VERSION` 1.4.0 → **1.5.0** (new behaviour) with a `Changelog.h` entry
  (uk/en). No recalibration required.

## Impact
- Affected specs:
  - `vehicle-systems`: MODIFIED "Steering Control System"; ADDED "Sensorless Steering Rate Mode".
  - `vehicle-actuators`: MODIFIED "Sensorless Manual Steering"; ADDED "Steering Rate Drive".
  - `mavlink-interface`: ADDED "Steering Control Mode Reporting", "Servo Command Frame Counter".
  - `web-telemetry`: ADDED "Steering Control Mode Telemetry"; MODIFIED "Steering Angle Sensor
    Telemetry" and "Diagnostics Attention Summary".
  - `web-control`: ADDED "Steering Slider Lock Without Sensor".
- Affected code:
  - `include/SteerRate.h` (new, pure: mapping and mode selection), `include/SteeringController.h`,
    `src/SteeringController.cpp`, `src/VehicleController.cpp` / `.h`.
  - `include/MavlinkInterface.h`, `src/MavlinkInterface.cpp`, `src/main.cpp` (MAVLink report
    struct), `include/TelemetryManager.h`, `src/TelemetryManager.cpp`, `src/WebPortal.cpp`.
  - `data/index.html`, `include/Constants.h`, `include/Changelog.h`, `test/` (new native test).
  - `MAVLINK_SETUP.md`.
- **Archive order**: this change's deltas are written against the specs as they will be after
  `replace-aztec-steering-sensor-with-bosch-lws` and then `redesign-web-ui-service-console` are
  archived. The MODIFIED blocks quote those pending deltas. It SHALL be archived after both.
- Safety: open-loop steering with no position limits, enabled only in MANUAL, only from the
  autopilot stick, and only while the sensor is already unhealthy. See `design.md`
  ("Risks").
- Not breaking: no wiring, calibration, NVS or parameter change.
