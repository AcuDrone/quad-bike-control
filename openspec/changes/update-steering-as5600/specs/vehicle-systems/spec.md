# vehicle-systems Spec Delta

## MODIFIED Requirements

### Requirement: Steering Control System
The system SHALL provide steering control through a BTS7960 H-bridge motor driver with proportional PWM speed control and absolute position feedback from an AS5600 magnetic angle sensor (I2C, magnet on the steering shaft, travel < 360° lock-to-lock), with software position limits.

#### Scenario: Set steering position by percentage
- **WHEN** SteeringController.setSteeringPercent() is called with value from -100 to +100
- **THEN** the steering actuator moves toward the corresponding AS5600 angle with proportional speed (duty scales with remaining error, clamped between minimum and maximum duty)
- **AND** -100% maps to the calibrated left limit angle
- **AND** 0% maps to the calibrated center angle
- **AND** +100% maps to the calibrated right limit angle (left and right ranges may be asymmetric)
- **AND** movement stops when the target angle is reached (within tolerance)
- **AND** the command is rejected if calibration is missing or the sensor is invalid

#### Scenario: Absolute position on startup
- **WHEN** the system initializes
- **THEN** the AS5600 sensor SHALL be initialized over I2C and magnet presence verified
- **AND** no homing procedure SHALL be performed (position is absolute)
- **AND** if calibration exists and the magnet is detected, the actuator SHALL move to the calibrated center angle
- **AND** if calibration is missing or the magnet is not detected, the actuator SHALL remain stopped

#### Scenario: Enforce software steering limits
- **WHEN** a steering command would move beyond the calibrated left or right limit angle
- **THEN** the target SHALL be clamped to the limit
- **AND** the actuator SHALL NOT move past the software limit

#### Scenario: Calibrate steering positions
- **WHEN** steering calibration is triggered via web portal
- **THEN** the system SHALL allow jogging the actuator left/right and capturing the current AS5600 angle as center, left limit, or right limit
- **AND** captured angles SHALL be validated (left and right on opposite sides of center, minimum span)
- **AND** calibrated angles SHALL be saved to NVS
- **AND** calibration SHALL persist across reboots

#### Scenario: Steering position wrap handling
- **WHEN** the AS5600 raw angle range crosses the 0/4095 wrap point within the steering travel
- **THEN** relative position SHALL be computed as the signed shortest delta from the calibrated center
- **AND** position control and limit enforcement SHALL behave identically to non-wrapping travel

#### Scenario: Steering sensor failure
- **WHEN** an AS5600 I2C read fails or the magnet is no longer detected
- **THEN** the steering motor SHALL be stopped immediately
- **AND** steering position commands SHALL be ignored while the sensor is invalid
- **AND** the fault SHALL be logged and reported in telemetry
- **AND** normal operation SHALL resume automatically when valid readings return

#### Scenario: Steering movement backstops
- **WHEN** a movement exceeds the move timeout, or the sensed position changes less than the stall threshold over the stall window while driving
- **THEN** the steering motor SHALL be stopped
- **AND** the event SHALL be logged

#### Scenario: Steering failsafe
- **WHEN** signal loss is detected or failsafe is activated
- **THEN** the steering actuator SHALL move to the calibrated center position
- **AND** if not calibrated, the actuator SHALL stop immediately
