## MODIFIED Requirements
### Requirement: Steering Control System
The system SHALL provide steering control through the steering motor driver (`IMotorDriver`, a VESC on the `aztec` branch) with proportional speed control and absolute multi-turn position feedback from a Bosch LWS steering-angle sensor (0 265 005 486) mounted directly on the steering shaft and read over the shared CAN bus (see "Bosch LWS Steering Angle Sensor"). Software position limits SHALL apply. All positions SHALL be expressed in 0.1° of steering-shaft angle (int32) relative to the sensor's own stored zero, with positive = right: the sensor's angle negated for a normal mount, or taken unchanged when the stored sensor-direction flag says the sensor is mounted mirrored (see "Steering Sensor Direction Inference"). While the sensor is not healthy, the autopilot steering channel SHALL be interpreted as a rate command under the conditions of "Sensorless Steering Rate Mode", and every other percent command SHALL be rejected.

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
- **AND** the sensor direction (normal or mirrored) SHALL be inferred from the limit captures, never configured by hand
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

### Requirement: Steering Limit Calibration Around the Sensor Zero
The steering controller SHALL use the Bosch LWS sensor's stored zero as the steering center and SHALL persist only the left and right limits, in 0.1° with positive = right, together with the sensor-direction flag, in NVS namespace `"steering"` under layout version 4, migrating away from every earlier steering layout.

#### Scenario: Capture a limit
- **WHEN** the left or right limit is captured while the sensor is healthy
- **THEN** the current position, in firmware sign under the sensor-direction flag that results from the capture (see "Steering Sensor Direction Inference"), SHALL be stored as that limit only, and saved to NVS immediately
- **AND** the steering motor SHALL be stopped
- **AND** the steering SHALL be calibrated whenever both limits are stored and valid

#### Scenario: Limit validation
- **WHEN** a limit capture is attempted
- **THEN** it SHALL be refused with "sensor fault" while the sensor is not healthy
- **AND** a capture whose absolute position is below `STEER_CAL_MIN_SPAN` SHALL be refused as too close to zero, whatever its sign
- **AND** a capture on the wrong side of zero for its button SHALL either flip the sensor-direction flag or be refused, as defined in "Steering Sensor Direction Inference"
- **AND** every stored left limit SHALL be at most `−STEER_CAL_MIN_SPAN` and every stored right limit at least `+STEER_CAL_MIN_SPAN`
- **AND** a limit beyond `±STEER_LWS_SAFE_RANGE` SHALL NOT be refused for that reason
- **AND** a refused capture SHALL leave the stored limits unchanged

#### Scenario: Limit captured beyond the safe range
- **WHEN** a valid left or right limit is captured with an absolute angle beyond `STEER_LWS_SAFE_RANGE` (550.0°)
- **THEN** it SHALL be stored and saved like any other limit
- **AND** the capture response SHALL carry a warning that the sensor's latched-fault threshold (~600°) is near
- **AND** telemetry `steer_range_warning` SHALL be `true` while either stored limit is beyond the safe range

#### Scenario: Mirrored sensor handled at calibration
- **WHEN** the sensor is mounted so that its angle increases toward the right
- **THEN** the first limit capture SHALL detect it and set the sensor-direction flag to mirrored, as defined in "Steering Sensor Direction Inference"
- **AND** the steering SHALL be calibrated only after both limits are captured under the same flag, so the closed loop SHALL never run with an inverted sign

#### Scenario: Zero-set invalidates the limits
- **WHEN** the sensor zero-set sequence has sent its reset frame, whatever its outcome
- **THEN** both stored limits SHALL be erased from NVS and the steering SHALL be not calibrated
- **AND** the limits SHALL have to be captured again against the new zero

#### Scenario: NVS layout and migration
- **WHEN** calibration or the sensor-direction flag is saved
- **THEN** it SHALL be stored under `cal_ver` = 4, `lim_l`, `lim_r` and `inv` (bool, `true` = mirrored) in namespace `"steering"`
- **WHEN** calibration is loaded and `cal_ver` is 3
- **THEN** `lim_l` and `lim_r` SHALL be kept, `inv` SHALL be written as `false` (v3 limits were captured with the fixed negation), and `cal_ver` SHALL be written as 4
- **AND** a steering that was calibrated under v3 SHALL stay calibrated with no recalibration
- **WHEN** calibration is loaded and `cal_ver` is neither 3 nor 4
- **THEN** the obsolete keys `fc_raw`, `cc_raw`, `fl_pos`, `cl_raw`, `fr_pos`, `cr_raw`, `c_ang`, `l_ang` and `r_ang`, and any `lim_l`, `lim_r` and `inv`, SHALL be removed individually where present
- **AND** `cal_ver` SHALL be written as 4 with no limits and `inv` = `false`, and the steering SHALL be not calibrated and SHALL require recalibration
- **AND** other keys in the namespace (e.g. `steer_sca_base`) SHALL be preserved, and the namespace SHALL never be cleared as a whole
- **WHEN** `inv` is missing under `cal_ver` = 4
- **THEN** it SHALL be read as `false`

#### Scenario: Firmware downgrade
- **WHEN** a firmware that only knows layout 3 boots with `cal_ver` = 4
- **THEN** its own migration SHALL erase the limits, so a mirrored calibration SHALL NOT be used by a firmware that would ignore the flag

## ADDED Requirements
### Requirement: Steering Sensor Direction Inference
The steering controller SHALL keep a persistent sensor-direction flag (`inv`: `false` = normal mount, sensor angle positive = left; `true` = mirrored mount, sensor angle positive = right) and SHALL derive the firmware position as `−angle` when the flag is `false` and `+angle` when it is `true`, and the signed rotation rate likewise. The flag SHALL be inferred from the Left/Right limit captures, so that a mirrored mount needs no re-fitting and no manual setting. All consumers of the position and rate (closed-loop control, limits, percent mapping, telemetry `steer_angle` / `steer_rate`, MAVLink `STEER_POS`) SHALL use the firmware sign and SHALL be unaware of the flag.

#### Scenario: Capture on the expected side
- **WHEN** "Set Left" is pressed and the position under the current flag is at most `−STEER_CAL_MIN_SPAN`, or "Set Right" is pressed and it is at least `+STEER_CAL_MIN_SPAN`
- **THEN** the limit SHALL be stored and the flag SHALL be unchanged

#### Scenario: Wrong side with the opposite limit unset
- **WHEN** "Set Left" is pressed and the position under the current flag is at least `+STEER_CAL_MIN_SPAN`, or "Set Right" is pressed and it is at most `−STEER_CAL_MIN_SPAN`
- **AND** the opposite limit is not stored
- **THEN** the flag SHALL be toggled and saved to NVS, the position SHALL be recomputed under the new flag, and the limit SHALL be stored
- **AND** the opposite limit SHALL be cleared, so the steering SHALL be not calibrated until the other limit is captured under the new flag
- **AND** the stall and progress baselines SHALL be reset to the recomputed position
- **AND** the serial console SHALL log `[STEER] sensor direction: mirrored` or `[STEER] sensor direction: normal` (feature SERVO), naming the capture that decided it
- **AND** the capture response SHALL say that the sensor direction was detected as mirrored (or normal) and that the other limit must be captured

#### Scenario: Wrong side with the opposite limit stored
- **WHEN** a capture is on the wrong side of zero by at least `STEER_CAL_MIN_SPAN` under the current flag
- **AND** the opposite limit is stored
- **THEN** the capture SHALL be refused, the flag SHALL NOT be toggled and the stored limits SHALL be unchanged
- **AND** the refusal SHALL say that the reading contradicts the stored opposite limit, that the sensor zero is probably off-centre or the sensor was re-mounted, and that the fix is to jog straight ahead, press "Set sensor zero" (which erases the limits) and capture both limits again

#### Scenario: No ping-pong between captures
- **WHEN** a capture has toggled the flag and the other limit is then captured on the wrong side again
- **THEN** that capture SHALL be refused as contradicting the limit just stored, and the flag SHALL NOT be toggled back

#### Scenario: Too close to zero
- **WHEN** a capture's absolute position is below `STEER_CAL_MIN_SPAN`
- **THEN** it SHALL be refused as too close to zero and the flag SHALL NOT be toggled, whatever the sign

#### Scenario: Commands that do not depend on the direction
- **WHEN** the flag is wrong or not yet inferred (before the first capture after a re-mount)
- **THEN** jog, nudge, rate mode and "Set sensor zero" SHALL work unchanged, because they do not use the position sign
- **AND** percent commands SHALL stay refused until both limits are stored, because the calibration is incomplete

#### Scenario: Flag survives a zero-set
- **WHEN** the sensor zero-set sequence runs
- **THEN** it SHALL erase the limits as before but SHALL NOT change the flag; the next capture re-confirms or toggles it

#### Scenario: Flag reported at boot
- **WHEN** the calibration is loaded at boot
- **THEN** the calibration log line SHALL include the sensor direction (normal or mirrored)
- **AND** the first-frame log line SHALL still show the raw sensor angle in the sensor's own sign, together with the resulting firmware position and the flag
