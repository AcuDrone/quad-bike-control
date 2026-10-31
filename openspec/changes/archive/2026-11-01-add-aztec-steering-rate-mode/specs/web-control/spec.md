## ADDED Requirements
### Requirement: Steering Slider Lock Without Sensor
The web portal SHALL lock the percent steering controls in Тести → Ручне керування / Tests → Manual control (the steering slider and the Left / Center / Right test buttons) whenever `steer_mode` is not `position`, and SHALL explain why. The steering jog and nudge controls and the other manual controls SHALL be unaffected.

#### Scenario: Slider locked in rate mode
- **WHEN** a telemetry frame has `steer_mode` equal to `rate`
- **THEN** the steering slider and the three steering test buttons SHALL be disabled, in addition to the existing input-source gating
- **AND** a hint SHALL be shown under the slider: "Без датчика кута кермо керується стіком як швидкістю (тільки MANUAL). Повзунок вимкнено, Jog/Nudge працюють." / "No angle sensor: the stick steers by rate (MANUAL only). Slider disabled; Jog/Nudge still work."

#### Scenario: Slider locked while blocked
- **WHEN** a telemetry frame has `steer_mode` equal to `blocked`, including after the operator has taken web control with the sensor unhealthy
- **THEN** the steering slider and the three steering test buttons SHALL be disabled
- **AND** a hint SHALL be shown: "Немає датчика кута: повзунок вимкнено, використовуйте Jog/Nudge у Калібруванні." / "No angle sensor: slider disabled, use Jog/Nudge in Calibration."

#### Scenario: Slider unlocked again
- **WHEN** a later frame has `steer_mode` equal to `position`
- **THEN** the lock and the hint SHALL be removed on that frame, and the controls SHALL follow the existing input-source gating only

#### Scenario: Other controls unaffected
- **WHEN** the steering slider is locked by the steering mode
- **THEN** the steering jog and nudge controls SHALL remain enabled exactly as before (see "Steering Calibration and Sensor Zero via Web Interface")
- **AND** "take control" and the throttle, brake and gear controls SHALL keep their existing behaviour

#### Scenario: Firmware rejects a percent command without a sensor
- **WHEN** a `set_steering` command reaches the firmware while the steering angle sensor is unhealthy
- **THEN** it SHALL be rejected without driving the motor, and the response SHALL say that there is no steering sensor and that Jog/Nudge should be used
