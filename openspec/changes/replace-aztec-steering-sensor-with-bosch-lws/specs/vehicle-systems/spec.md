## MODIFIED Requirements
### Requirement: Steering Control System
The system SHALL provide steering control through the steering motor driver (`IMotorDriver`, a VESC on the `aztec` branch) with proportional speed control and absolute multi-turn position feedback from a Bosch LWS steering-angle sensor (0 265 005 486) mounted directly on the steering shaft and read over the shared CAN bus (see "Bosch LWS Steering Angle Sensor"). Software position limits SHALL apply. All positions SHALL be expressed in 0.1° of steering-shaft angle (int32) relative to the sensor's own stored zero, with positive = right (the sensor's angle negated).

#### Scenario: Set steering position by percentage
- **WHEN** SteeringController.setSteeringPercent() is called with value from -100 to +100
- **THEN** the steering actuator moves toward the corresponding angle with proportional speed (duty scales with remaining error, clamped between minimum and maximum duty)
- **AND** -100% maps to the calibrated left limit angle
- **AND** 0% maps to the sensor zero (angle 0.0)
- **AND** +100% maps to the calibrated right limit angle (left and right ranges may be asymmetric)
- **AND** movement stops when the target angle is reached (within `STEER_POSITION_TOLERANCE`)
- **AND** the command is rejected if either limit is missing or the steering angle sensor is not healthy

#### Scenario: Absolute position on startup
- **WHEN** the system initializes
- **THEN** no I2C steering sensor SHALL be probed and no homing procedure SHALL be performed (the angle is absolute)
- **AND** the stored limits SHALL be loaded whether or not sensor frames are already arriving
- **AND** if both limits are stored, a boot centering SHALL be requested
- **AND** the requested centering SHALL be issued once, from the control loop, as soon as the sensor is healthy and the steering motor driver link is healthy, and SHALL be logged when issued
- **AND** if it cannot be issued within `STEER_BOOT_CENTER_TIMEOUT_MS`, the request SHALL be dropped and the actuator SHALL NOT move by itself afterwards
- **AND** an accepted steering, jog or nudge command, a limit capture or a sensor zero-set made before it is issued SHALL cancel the request
- **AND** if the limits are missing, the actuator SHALL remain stopped

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
- **AND** jog and nudge commands SHALL remain available open-loop (see "Sensorless Manual Steering")
- **AND** the fault and its reason SHALL be logged and reported in telemetry
- **AND** normal operation SHALL resume automatically once the sensor is healthy again, except that a latched range fault clears only when the sensor is power-cycled by restarting the vehicle

#### Scenario: Steering movement backstops
- **WHEN** a movement with a healthy sensor exceeds the move timeout, or the sensed angle changes less than `STEER_STALL_THRESHOLD` over the stall window while driving
- **THEN** the steering motor SHALL be stopped
- **AND** the event SHALL be logged

#### Scenario: Steering failsafe
- **WHEN** signal loss is detected or failsafe is activated
- **THEN** the steering actuator SHALL move to the sensor zero (center)
- **AND** if the limits are missing or the sensor is not healthy, the actuator SHALL stop immediately

## ADDED Requirements
### Requirement: Steering Limit Calibration Around the Sensor Zero
The steering controller SHALL use the Bosch LWS sensor's stored zero as the steering center and SHALL persist only the left and right limits, in 0.1° with positive = right, in NVS namespace `"steering"` under a new layout version, migrating away from every earlier steering layout.

#### Scenario: Capture a limit
- **WHEN** the left or right limit is captured while the sensor is healthy
- **THEN** the current angle SHALL be stored as that limit only, and saved to NVS immediately
- **AND** the steering motor SHALL be stopped
- **AND** the steering SHALL be calibrated whenever both limits are stored and valid

#### Scenario: Limit validation
- **WHEN** a limit capture is attempted
- **THEN** it SHALL be refused with "sensor fault" while the sensor is not healthy
- **AND** a left limit SHALL be refused unless it is at most `−STEER_CAL_MIN_SPAN`, and a right limit unless it is at least `+STEER_CAL_MIN_SPAN`, naming the failed check (wrong side of zero, or too close to zero)
- **AND** a limit beyond `±STEER_LWS_SAFE_RANGE` SHALL NOT be refused for that reason
- **AND** a refused capture SHALL leave the stored limits unchanged

#### Scenario: Limit captured beyond the safe range
- **WHEN** a valid left or right limit is captured with an absolute angle beyond `STEER_LWS_SAFE_RANGE` (550.0°)
- **THEN** it SHALL be stored and saved like any other limit
- **AND** the capture response SHALL carry a warning that the sensor's latched-fault threshold (~600°) is near
- **AND** telemetry `steer_range_warning` SHALL be `true` while either stored limit is beyond the safe range

#### Scenario: Mirrored sensor caught at calibration
- **WHEN** the sensor is mounted so that its angle increases toward the right
- **THEN** a capture at the left lock SHALL read a positive angle and be refused as on the wrong side of zero
- **AND** the closed loop SHALL therefore never run with an inverted sign

#### Scenario: Zero-set invalidates the limits
- **WHEN** the sensor zero-set sequence has sent its reset frame, whatever its outcome
- **THEN** both stored limits SHALL be erased from NVS and the steering SHALL be not calibrated
- **AND** the limits SHALL have to be captured again against the new zero

#### Scenario: NVS layout and migration
- **WHEN** calibration is saved
- **THEN** it SHALL be stored under `cal_ver` = 3, `lim_l` and `lim_r` in namespace `"steering"`
- **WHEN** calibration is loaded and `cal_ver` is not 3
- **THEN** the obsolete keys `fc_raw`, `cc_raw`, `fl_pos`, `cl_raw`, `fr_pos`, `cr_raw`, `c_ang`, `l_ang` and `r_ang` SHALL be removed individually where present
- **AND** `cal_ver` SHALL be written as 3 with no limits, and the steering SHALL be not calibrated and SHALL require recalibration
- **AND** other keys in the namespace (e.g. `steer_sca_base`) SHALL be preserved, and the namespace SHALL never be cleared as a whole

## REMOVED Requirements
### Requirement: Dual-Sensor Multi-Turn Steering Position
**Reason**: The two AS5600 sensors, incremental turn tracking, turn recovery from the coarse sensor and the fine/coarse consistency check are replaced by a single absolute multi-turn Bosch LWS sensor on CAN.
**Migration**: Steering position comes from the LWS angle (see "Bosch LWS Steering Angle Sensor" in vehicle-actuators and "Steering Control System"). Remove both AS5600s and the geared-down shaft.

### Requirement: Dual-Sensor Steering Calibration
**Reason**: There is no coarse sensor and no derived ratio any more, and the center is the sensor's own stored zero, so the three-point session calibration is obsolete.
**Migration**: Use "Steering Limit Calibration Around the Sensor Zero": set the sensor zero once at straight-ahead, then capture Left and Right. The `cal_ver` = 2 keys are removed on first boot and recalibration is required.
