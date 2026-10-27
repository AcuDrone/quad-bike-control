## MODIFIED Requirements
### Requirement: Steering Control System
The system SHALL provide steering control through the steering motor driver (`IMotorDriver`, a VESC on the `aztec` branch) with proportional speed control and absolute multi-turn position feedback from a Bosch LWS steering-angle sensor (0 265 005 486) mounted directly on the steering shaft and read over the shared CAN bus (see "Bosch LWS Steering Angle Sensor"). Software position limits SHALL apply. All positions SHALL be expressed in 0.1° of steering-shaft angle (int32) relative to the sensor's own stored zero, with positive = right (the sensor's angle negated). While the sensor is not healthy, the autopilot steering channel SHALL be interpreted as a rate command under the conditions of "Sensorless Steering Rate Mode", and every other percent command SHALL be rejected.

#### Scenario: Set steering position by percentage
- **WHEN** SteeringController.setSteeringPercent() is called with value from -100 to +100
- **THEN** the steering actuator moves toward the corresponding angle with proportional speed (duty scales with remaining error, clamped between minimum and maximum duty)
- **AND** -100% maps to the calibrated left limit angle
- **AND** 0% maps to the sensor zero (angle 0.0)
- **AND** +100% maps to the calibrated right limit angle (left and right ranges may be asymmetric)
- **AND** movement stops when the target angle is reached (within `STEER_POSITION_TOLERANCE`)
- **AND** the command is rejected if either limit is missing or the steering angle sensor is not healthy
- **AND** a rejected autopilot percent command SHALL NOT itself drive the motor; only the rate path of "Sensorless Steering Rate Mode" MAY then use the same channel

#### Scenario: Absolute position on startup
- **WHEN** the system initializes
- **THEN** no I2C steering sensor SHALL be probed and no homing procedure SHALL be performed (the angle is absolute)
- **AND** the stored limits SHALL be loaded whether or not sensor frames are already arriving
- **AND** if both limits are stored, a boot centering SHALL be requested
- **AND** the requested centering SHALL be issued once, from the control loop, as soon as the sensor is healthy and the steering motor driver link is healthy, and SHALL be logged when issued
- **AND** if it cannot be issued within `STEER_BOOT_CENTER_TIMEOUT_MS`, the request SHALL be dropped and the actuator SHALL NOT move by itself afterwards
- **AND** an accepted steering, jog or nudge command, a limit capture or a sensor zero-set made before it is issued SHALL cancel the request
- **AND** if the limits are missing, the actuator SHALL remain stopped
- **AND** boot centering SHALL NOT be attempted in rate mode or in any other way without a healthy sensor

#### Scenario: Enforce software steering limits
- **WHEN** a steering command would move beyond the calibrated left or right limit angle
- **THEN** the target SHALL be clamped to the limit
- **AND** the actuator SHALL NOT move past the software limit under closed-loop control

#### Scenario: Closed-loop target clamped to the safe range
- **WHEN** a percent command (MAVLink, web slider or test buttons, boot centering or failsafe centre) maps to a target beyond `±STEER_LWS_SAFE_RANGE` (550.0°), because a stored limit lies beyond it
- **THEN** the command SHALL be accepted and its target clamped to ±`STEER_LWS_SAFE_RANGE`
- **AND** a closed-loop move that would drive further outward while the absolute angle is at or beyond the safe range SHALL be stopped

#### Scenario: Calibrate steering positions
- **WHEN** steering calibration is performed via the web portal
- **THEN** the system SHALL allow jogging and nudging the actuator left/right with or without a healthy sensor
- **AND** the center SHALL be set by the sensor zero-set sequence, not by a capture
- **AND** the left and right limits SHALL be captured separately, each validated and saved to NVS immediately
- **AND** calibration SHALL persist across reboots

#### Scenario: Steering position wrap handling
- **WHEN** the steering travels through any number of shaft turns within the sensor range
- **THEN** the position SHALL be taken directly from the sensor's absolute multi-turn angle, with no wrap carry and no turn counter in the firmware
- **AND** position control and limit enforcement SHALL behave identically across the whole travel

#### Scenario: Steering sensor failure
- **WHEN** the steering angle sensor is not healthy (frame timeout, invalid angle, sensor fault bits, sensor not calibrated, latched range fault, CAN controller down, or a zero-set in progress)
- **THEN** a closed-loop position move SHALL be stopped immediately
- **AND** steering percent commands SHALL be rejected while the sensor is not healthy
- **AND** the autopilot steering channel SHALL drive the motor as a rate command only while the conditions of "Sensorless Steering Rate Mode" hold (input source MAVLink and the autopilot in MANUAL or its mode unknown)
- **AND** jog and nudge commands SHALL remain available open-loop (see "Sensorless Manual Steering")
- **AND** the fault and its reason SHALL be logged and reported in telemetry
- **AND** normal operation SHALL resume automatically once the sensor is healthy again, except that a latched range fault clears only when the sensor is power-cycled by restarting the vehicle

#### Scenario: Steering movement backstops
- **WHEN** a movement with a healthy sensor exceeds the move timeout, or the sensed angle changes less than `STEER_STALL_THRESHOLD` over the stall window while driving
- **THEN** the steering motor SHALL be stopped
- **AND** the event SHALL be logged
- **AND** these position backstops SHALL NOT apply to a rate-mode drive, which has no position

#### Scenario: Steering failsafe
- **WHEN** signal loss is detected or failsafe is activated
- **THEN** the steering actuator SHALL move to the sensor zero (center)
- **AND** if the limits are missing or the sensor is not healthy, the actuator SHALL stop immediately
- **AND** a rate-mode drive in progress SHALL be stopped (duty 0) in the same control-loop iteration, and the steering SHALL NOT be centred

## ADDED Requirements
### Requirement: Sensorless Steering Rate Mode
The system SHALL select one steering mode on every control-loop iteration from the steering sensor health, the active input source and the autopilot flight mode, and SHALL report it as `position`, `rate` or `blocked`. The selection SHALL be automatic, with no operator switch and no parameter. In `rate` mode the autopilot steering channel SHALL command the direction and speed of steering rotation rather than a position.

#### Scenario: Mode selection
- **WHEN** the steering mode is evaluated
- **THEN** it SHALL be `position` while the steering angle sensor is healthy (the existing debounced health flag), whatever the input source or autopilot mode
- **AND** it SHALL be `rate` while the sensor is not healthy AND the active input source is MAVLink AND the autopilot's flight mode is Rover MANUAL (`custom_mode` = `ROVER_CUSTOM_MODE_MANUAL`) or unavailable (never received or stale)
- **AND** it SHALL be `blocked` in every other case (sensor not healthy with the web or failsafe as input source, or with a known autopilot mode other than MANUAL)
- **AND** a healthy sensor with missing limits SHALL be `position` (percent commands are then rejected as uncalibrated), so that `rate` never activates while the sensor is healthy

#### Scenario: Rate command from the autopilot stick
- **WHEN** the mode is `rate` and a new `SERVO_OUTPUT_RAW` frame has arrived since the last rate command
- **THEN** the steering percentage the autopilot path already computes (decoded channel, `RC_STEERING_DEADBAND` applied, speed scale applied) SHALL be passed to the steering controller's rate command
- **AND** inside the deadband the commanded duty SHALL be zero and the motor SHALL stop
- **AND** outside the deadband the duty SHALL be `STEER_PWM_MIN_DUTY + (STEER_PWM_MAX_DUTY − STEER_PWM_MIN_DUTY) × d²`, where `d = (|pct| − RC_STEERING_DEADBAND) / (100 − RC_STEERING_DEADBAND)` clamped to at most 1, so that full deflection gives duty 255
- **AND** a positive percentage SHALL turn the wheels right and a negative one left
- **AND** full deflection SHALL give duty 255 only while the applied steering speed scale is 1 (at or below the scaling base speed, or scaling inactive); above the base speed the scaled deflection, and therefore the rate, SHALL be reduced by the same factor as in position mode
- **AND** the reported steering scale (`STEER_SCA`, `steer_scale`) SHALL keep reporting the scale actually applied, with its existing meaning
- **AND** when no new frame has arrived, no rate command SHALL be issued, so that a stale decoded channel never keeps the motor running

#### Scenario: Release or frame loss stops the motor
- **WHEN** the mode is `rate` and no new `SERVO_OUTPUT_RAW` frame has refreshed the rate command for `STEER_JOG_TIMEOUT_MS` (500 ms)
- **THEN** the motor SHALL be stopped
- **AND** a centred stick SHALL stop the motor on the next frame

#### Scenario: Centre the stick to arm
- **WHEN** the mode changes to `rate` from `position` or from `blocked`
- **THEN** rate commands outside the deadband SHALL be refused until one fresh frame has shown the stick inside the deadband
- **AND** the waiting state SHALL be logged once on the serial console

#### Scenario: Autonomous modes stay blocked
- **WHEN** the sensor is not healthy and the autopilot reports a known flight mode other than MANUAL
- **THEN** the mode SHALL be `blocked` and the autopilot steering channel SHALL NOT drive the motor
- **AND** a rate drive in progress SHALL be stopped in the same control-loop iteration

#### Scenario: Leaving rate mode for blocked
- **WHEN** the mode changes from `rate` to `blocked` (input source changes away from MAVLink, including web takeover and failsafe, or the autopilot leaves MANUAL)
- **THEN** a drive started by a rate command SHALL be stopped immediately rather than after the refresh timeout
- **AND** a web jog or nudge in progress SHALL NOT be stopped by this transition

#### Scenario: Sensor recovers during rate mode
- **WHEN** the mode changes from `rate` to `position` because the sensor became healthy again (the existing `STEER_LWS_RECOVER_FRAMES` streak)
- **THEN** a drive started by a rate command SHALL be stopped and the wheels SHALL hold where they are
- **AND** no position target SHALL be set and no move SHALL be started by the transition itself, and boot centering SHALL NOT be requested again
- **AND** the next percent command SHALL be accepted and executed as a normal closed-loop move

#### Scenario: No debounce beyond the sensor's own
- **WHEN** the sensor health flag changes
- **THEN** the mode SHALL follow it with the existing fault timing (immediate fault, `STEER_LWS_TIMEOUT_MS` timeout, `STEER_LWS_RECOVER_FRAMES` recovery) and no additional timer
- **AND** every mode change SHALL stop a rate-owned drive and require the stick to be centred again before rate motion resumes

#### Scenario: Mode transitions are logged
- **WHEN** the steering mode changes
- **THEN** one serial console line SHALL name the old mode, the new mode and the reason (sensor fault token, sensor OK, input source or autopilot mode)
