## ADDED Requirements
### Requirement: Exact Steering Angle via Web Interface
The web portal SHALL provide, in the steering calibration card next to Jog / Nudge, a numeric "Target angle, °" input (step 0.1°, bounded by the captured Left / Right limits) with a "Go" button and a "0°" button. They SHALL send the WebSocket command `steer_angle` with `value` = the target angle in degrees (float, + = right); the browser SHALL NOT convert it to percent. The firmware SHALL convert the angle to 0.1° units, clamp it to the captured limits and the ±550° safe range, and move through the same closed-loop command path as `set_steering`. The steering jog speed slider SHALL span the firmware's jog duty range (80–255, default 120 = `STEER_JOG_DUTY`).

#### Scenario: Angle command accepted
- **WHEN** web control is active, `steer_mode` is `position`, the limits are captured, the VESC link is up and the USB diagnostics lock is off
- **AND** the operator enters 12.3 and presses Go
- **THEN** the firmware SHALL drive the wheels to 12.3° (clamped to the captured limits) and reply with a success response
- **AND** the `steering_pct` telemetry SHALL keep reporting the measured position as before

#### Scenario: Angle command refused
- **WHEN** `steer_angle` arrives while MAVLink has control, the USB diagnostics console holds the steering, the sensor is unhealthy, the steering is not calibrated or the value is not a finite number
- **THEN** the firmware SHALL NOT move the steering and SHALL reply with an error response in the same style as `set_steering` ("MAVLink control active", "held by the USB diagnostics console", "no steering sensor — use Jog/Nudge", "not calibrated or sensor fault", "Invalid steering angle")

#### Scenario: Angle controls locked like the percent slider
- **WHEN** web control is not active, or `steer_mode` is not `position`
- **THEN** the angle input, Go and 0° SHALL be disabled
- **AND** while `steer_mode` is not `position` the same lock hint as the percent slider SHALL be shown under them
