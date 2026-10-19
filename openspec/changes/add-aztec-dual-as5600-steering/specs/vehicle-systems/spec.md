## MODIFIED Requirements
### Requirement: Steering Control System
The system SHALL provide steering control through the steering motor driver (`IMotorDriver`, a VESC on the `aztec` branch) with proportional speed control and absolute multi-turn position feedback from two AS5600 magnetic angle sensors. The fine sensor SHALL be on the steering shaft, which turns ±1.7 turns from centre. The coarse sensor SHALL be on a geared-down shaft sweeping less than 360° lock-to-lock. Software position limits SHALL apply. All positions SHALL be expressed as unwrapped fine-sensor counts relative to the calibrated centre (int32, about 0.088° of steering shaft per count).

#### Scenario: Set steering position by percentage
- **WHEN** SteeringController.setSteeringPercent() is called with value from -100 to +100
- **THEN** the steering actuator moves toward the corresponding unwrapped fine-sensor position with proportional speed (duty scales with remaining error, clamped between minimum and maximum duty)
- **AND** -100% maps to the calibrated left limit position
- **AND** 0% maps to the calibrated center position
- **AND** +100% maps to the calibrated right limit position (left and right ranges may be asymmetric and may each span more than one shaft turn)
- **AND** movement stops when the target position is reached (within tolerance)
- **AND** the command is rejected if calibration is missing or the combined sensor state is invalid

#### Scenario: Absolute position on startup
- **WHEN** the system initializes
- **THEN** both AS5600 sensors SHALL be probed and magnet presence verified on each
- **AND** the stored calibration SHALL be loaded whether or not the sensors answered
- **AND** no homing procedure SHALL be performed: the absolute shaft turn SHALL be recovered from the coarse sensor
- **AND** if calibration exists, both sensors are healthy and turn recovery succeeds during initialization, a boot centering SHALL be requested
- **AND** the requested centering SHALL be issued once, from the control loop, as soon as the steering motor driver link is also healthy, and SHALL be logged when issued
- **AND** if it cannot be issued within `STEER_BOOT_CENTER_TIMEOUT_MS`, the request SHALL be dropped and the actuator SHALL NOT move by itself afterwards
- **AND** an accepted steering, jog or nudge command or a calibration capture made before it is issued SHALL cancel the request
- **AND** if calibration is missing, either sensor is faulted, or the turn is not recovered, the actuator SHALL remain stopped

#### Scenario: Enforce software steering limits
- **WHEN** a steering command would move beyond the calibrated left or right limit position
- **THEN** the target SHALL be clamped to the limit
- **AND** the actuator SHALL NOT move past the software limit

#### Scenario: Calibrate steering positions
- **WHEN** steering calibration is triggered via web portal
- **THEN** the system SHALL allow jogging and nudging the actuator left/right and capturing center, left limit and right limit with the existing three capture commands
- **AND** each capture SHALL record both the fine-sensor position and the coarse-sensor raw angle
- **AND** captured values SHALL be validated (left and right on opposite sides of center, minimum span, and the derived coarse-to-fine ratio checks)
- **AND** calibration values SHALL be saved to NVS
- **AND** calibration SHALL persist across reboots

#### Scenario: Steering position wrap handling
- **WHEN** the fine sensor crosses its 0/4095 wrap point, one or more times, within the steering travel
- **THEN** the position SHALL continue as an unwrapped count (the turn counter carries the wrap)
- **AND** position control and limit enforcement SHALL behave identically on every turn of the travel

#### Scenario: Steering sensor failure
- **WHEN** an I2C read of either AS5600 fails, either magnet is no longer detected, or the two sensors disagree by more than the mismatch threshold
- **AND** the condition persists for the fault debounce
- **THEN** the steering motor SHALL be stopped immediately
- **AND** steering position, jog and nudge commands SHALL be ignored while the combined sensor state is invalid
- **AND** no single-sensor degraded mode SHALL exist
- **AND** the fault and its reason SHALL be logged and reported in telemetry
- **AND** after an I2C or magnet fault, normal operation SHALL resume automatically once both sensors read validly again and, when calibrated, the absolute turn has been recovered
- **AND** after a runtime mismatch fault, normal operation SHALL NOT resume automatically (see Continuous consistency check)

#### Scenario: Steering movement backstops
- **WHEN** a movement exceeds the move timeout, or the sensed position changes less than the stall threshold over the stall window while driving
- **THEN** the steering motor SHALL be stopped
- **AND** the event SHALL be logged

#### Scenario: Steering failsafe
- **WHEN** signal loss is detected or failsafe is activated
- **THEN** the steering actuator SHALL move to the calibrated center position
- **AND** if not calibrated, the actuator SHALL stop immediately

## ADDED Requirements
### Requirement: Dual-Sensor Multi-Turn Steering Position
The steering controller SHALL derive the steering position from a fine AS5600 on the steering shaft and a coarse AS5600 on a geared-down shaft. It SHALL track fine-sensor turns incrementally, recover the absolute turn from the coarse sensor, and continuously cross-check the two sensors.

#### Scenario: Incremental turn tracking
- **WHEN** a good fine-sensor sample is read
- **AND** it differs from the previous good sample by more than half a turn (2048 counts) in raw terms
- **THEN** the turn counter SHALL be incremented or decremented so that the unwrapped position changes by the signed shortest delta
- **AND** the position SHALL be `fineRaw + 4096 × turn − fineCenterRaw`

#### Scenario: Turn recovery from the coarse sensor
- **WHEN** the steering is calibrated
- **AND** both sensors deliver good samples at boot, or after the combined sensor state was faulted
- **THEN** the expected fine position SHALL be computed from the coarse sample using the calibrated, seam-linearised coarse-to-fine line
- **AND** for each coarse poll the turn number SHALL be chosen so that `fineRaw − fineCenterRaw + 4096 × turn` is closest to that expected position
- **AND** recovery SHALL succeed only after `STEER_TURN_RECOVER_SAMPLES` consecutive coarse polls each yield the same turn (relative to the incremental turn counter) with a residual within `STEER_SENSOR_MISMATCH_COUNTS` (a quarter turn)
- **AND** a poll that yields a different turn, a residual beyond the threshold, or no good sample pair SHALL restart the count
- **AND** if the residual exceeds the threshold on `STEER_SENSOR_FAULT_DEBOUNCE` consecutive polls, the fault reason SHALL be `mismatch` until polls agree again
- **AND** until recovery succeeds, the combined sensor state SHALL be invalid, the motor SHALL stay stopped, commands SHALL be rejected, and the fault reason SHALL be `recovering` when no other reason applies

#### Scenario: Continuous consistency check
- **WHEN** the steering is calibrated and a coarse sample is read, at least every `STEER_COARSE_POLL_MS`
- **THEN** the residual between the tracked fine position and the coarse-derived expected position SHALL be computed
- **AND** a residual beyond `STEER_SENSOR_MISMATCH_COUNTS` on `STEER_SENSOR_FAULT_DEBOUNCE` consecutive coarse samples SHALL raise a mismatch fault (motor stopped, commands rejected)
- **AND** the mismatch fault SHALL be latched: the combined sensor state SHALL stay invalid and the fault reason SHALL stay `mismatch` even if later samples agree
- **AND** the latch SHALL be cleared only by a successful center capture or a reboot

#### Scenario: Per-sensor fault debounce
- **WHEN** a sample from either sensor has a failed I2C read or a cleared magnet-detect bit
- **THEN** that sample SHALL NOT update the position or the turn counter
- **AND** a fault SHALL be declared for that sensor after `STEER_SENSOR_FAULT_DEBOUNCE` consecutive bad samples
- **AND** the fault reason SHALL be one of `fine_i2c`, `fine_magnet`, `coarse_i2c`, `coarse_magnet`, `mismatch` or `recovering`
- **AND** the fault reason SHALL be `none` only while the combined sensor state is valid

#### Scenario: Late sensor connection
- **WHEN** either sensor did not answer at boot
- **THEN** it SHALL be re-probed every `STEER_SENSOR_REPROBE_MS`
- **AND** once it answers, steering SHALL become available without a reboot, after turn recovery when calibrated

### Requirement: Dual-Sensor Steering Calibration
The steering controller SHALL capture both sensors at each calibration point, derive the coarse-to-fine ratio itself, validate the geometry, and persist the calibration in NVS namespace `"steering"` under keys distinct from the single-sensor layout.

#### Scenario: Capture center starts a calibration session
- **WHEN** the center is captured while both sensors are healthy
- **THEN** the turn counter SHALL be set to zero
- **AND** the fine raw angle and the coarse raw angle SHALL be stored as the center pair
- **AND** both limits SHALL be cleared, leaving the steering not calibrated
- **AND** a latched mismatch fault SHALL be cleared
- **AND** a calibration session SHALL be opened

#### Scenario: Capture a limit
- **WHEN** a left or right limit is captured while both sensors are healthy
- **AND** a calibration session is open or the steering is already fully calibrated
- **THEN** the current unwrapped fine position (relative to center) and the coarse raw angle SHALL be stored as that limit pair
- **AND** the calibration SHALL be re-validated and persisted
- **AND** the capture SHALL be refused when neither condition holds, because the absolute turn is unknown

#### Scenario: Calibration session ends on fault or reboot
- **WHEN** a sensor fault is declared or the system reboots during an incomplete calibration
- **THEN** the calibration session SHALL end
- **AND** a later limit capture SHALL be refused until the center is captured again

#### Scenario: Derived ratio and geometry validation
- **WHEN** center, left and right pairs are all present
- **THEN** the coarse travel arc and its unused gap SHALL be determined from the three coarse values, and a seam SHALL be placed mid-gap
- **AND** the calibration SHALL be accepted only if:
  - the fine limits lie on opposite sides of center, each at least `STEER_CAL_MIN_SPAN` away
  - the seam-linearised coarse values are strictly ordered left, center, right (or the reverse)
  - the coarse span per fine-shaft turn is at least `STEER_COARSE_MIN_PER_TURN`
  - the unused coarse gap is at least `STEER_COARSE_MIN_GAP`
  - the center pair lies within `STEER_CAL_LINEARITY_COUNTS` of the line through the two limit pairs
- **AND** the ratio, seam and direction SHALL be derived values recomputed on load and SHALL NOT be stored
- **AND** a rejected calibration SHALL leave the steering not calibrated and SHALL report the failed check

#### Scenario: New NVS layout replaces the single-sensor keys
- **WHEN** calibration is saved
- **THEN** it SHALL be stored under `cal_ver`, `fc_raw`, `cc_raw`, `fl_pos`, `cl_raw`, `fr_pos` and `cr_raw` in namespace `"steering"`
- **AND** the obsolete `c_ang`, `l_ang` and `r_ang` keys SHALL be removed individually
- **AND** other keys in the namespace (e.g. `steer_sca_base`) SHALL be preserved
- **WHEN** calibration is loaded and `cal_ver` is not the dual-sensor version
- **THEN** the steering SHALL be not calibrated and SHALL require recalibration
