# vehicle-systems Specification

## Purpose
TBD - created by archiving change add-vehicle-control-system. Update Purpose after archive.
## Requirements
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

### Requirement: Throttle Control System
The system SHALL provide throttle control through a servo-driven acceleration mechanism. Throttle
percent SHALL map linearly from the calibrated idle pulse width (0 %) to the calibrated full pulse
width (100 %); the two endpoints MAY be in either order so that a mirrored servo mount (full below
idle) is supported. Every percentage command SHALL pass through the throttle opening rate limit
(see "Configurable Throttle Opening Rate") at the servo input, before the percent-to-µs mapping:
opening SHALL be rate-limited, closing SHALL never be slowed.

#### Scenario: Set throttle position by percentage
- **WHEN** ThrottleSystem.setPosition() is called with value from 0 to 100
- **THEN** throttle servo moves to corresponding position, rising no faster than the configured
  opening rate
- **AND** 0% maps to idle/closed throttle
- **AND** 100% maps to full throttle

#### Scenario: Return to idle
- **WHEN** ThrottleSystem.idle() is called
- **THEN** throttle servo returns to 0% position immediately, without any rate limit
- **AND** throttle remains at idle until new command
- **AND** the opening rate limit's last commanded percentage SHALL be reset to 0, so the next
  opening ramps from idle

#### Scenario: Throttle safety override
- **WHEN** brake system is engaged above threshold (e.g., >50%)
- **THEN** throttle is automatically reduced to idle
- **AND** throttle commands are ignored until brakes release

#### Scenario: Inverted servo calibration accepted
- **WHEN** a throttle calibration is saved with idle = 2200 µs and full = 800 µs
- **THEN** the calibration SHALL be accepted and persisted to NVS
- **AND** 0 % SHALL command 2200 µs, 50 % 1500 µs and 100 % 800 µs
- **AND** the reported throttle percent SHALL be 0..100 for pulse widths between the endpoints and
  clamped to 0 / 100 outside them
- **AND** the save log SHALL mark the pair as inverted
- **AND** after a reboot the stored inverted pair SHALL be loaded, not replaced by the defaults

#### Scenario: Span check applies to absolute difference
- **WHEN** a throttle calibration is saved or loaded from NVS
- **THEN** it SHALL be accepted only if both endpoints lie within `THROTTLE_SERVO_MIN_US` ..
  `THROTTLE_SERVO_MAX_US` and `|full - idle| >= THROTTLE_MIN_SPAN_US`, regardless of which endpoint
  is larger
- **AND** a rejected save SHALL report "Idle and full must differ by >= 100us"
- **AND** an invalid stored pair SHALL fall back to the default endpoints

#### Scenario: Opening is rate-limited at the servo input
- **WHEN** the opening rate is 60 %/s, the last commanded throttle is 0 % and percentage commands
  of 100 % arrive every control-loop iteration
- **THEN** the commanded throttle SHALL rise by at most 60 %/s × the elapsed time per iteration,
  reaching 100 % after ≈1.67 s
- **AND** this SHALL hold identically for the MAVLink throttle, the web `set_throttle` command, the
  per-loop web re-limit and the speed-limiter output, because all of them reach the servo through
  the same percentage command

#### Scenario: Closing is immediate
- **WHEN** a percentage command is lower than or equal to the last commanded percentage
- **THEN** it SHALL be applied in the same call, whatever the opening rate

#### Scenario: Elapsed time is bounded
- **WHEN** a percentage command arrives after a gap since the previous throttle write
- **THEN** the elapsed time used by the opening rate limit SHALL be capped at
  `THROTTLE_OPEN_SLEW_MAX_DT_S` (0.1 s), and the first command after start-up SHALL use zero
- **AND** a stalled loop or a path that was silent for seconds SHALL therefore never open the
  throttle in one step

#### Scenario: Direct pulse-width writes bypass the opening limit
- **WHEN** the gear-change boost PID writes the throttle in microseconds
- **THEN** the opening rate limit SHALL NOT be applied, the PID keeping its own
  `TRANS_GEAR_BOOST_SLEW_RATE_US` slew
- **AND** the opening limit's last commanded percentage SHALL be re-seeded from the written pulse
  width, so a later percentage command ramps from the servo's real position
- **AND** the throttle calibration jog SHALL also drive the servo directly, and saving or
  cancelling the calibration SHALL reset the last commanded percentage to 0

### Requirement: Transmission Control System
The transmission control system SHALL enforce the physical gear sequence `[R, N, H, L]` and SHALL govern overshoot and rollback dwell timing via constants in `Constants.h`.

#### Scenario: Gear change routes through sequence
- **WHEN** a gear change command is received (S-bus or web)
- **AND** the target gear is not adjacent to the current gear in sequence `[R, N, H, L]`
- **THEN** the vehicle controller SHALL pass the final target gear to `TransmissionController::setGear()`
- **AND** intermediate gear changes SHALL be handled automatically by `TransmissionController`
- **AND** the telemetry SHALL report the current physical gear at each step

#### Scenario: Constants govern dwell timing
- **WHEN** `TRANS_OVERSHOOT_DWELL_MS` or `TRANS_ROLLBACK_DWELL_MS` are modified in `Constants.h`
- **THEN** the corresponding dwell durations in the transmission state machine SHALL reflect the updated values without additional code changes

#### Scenario: Physical gear is decoded from the 3-bit gear code
- **WHEN** `TransmissionController::getPhysicalGear()` reads a valid opto-input snapshot
- **THEN** it SHALL decode the gear code `[bit0 bit1 bit6]` only through
  `TransmissionController::decodeGearCode()`, using the runtime code table
- **AND** the runtime table SHALL default to `GEAR_CODE_LOW` = `101`, `GEAR_CODE_HIGH` = `001`,
  `GEAR_CODE_NEUTRAL` = `011`, `GEAR_CODE_REVERSE` = `010`, overridden by codes captured through the
  web gear code calibration (NVS `"transmission"` `code_r/n/h/l`, loaded at `begin()`)
- **AND** the four default codes SHALL be checked distinct at compile time
- **AND** in the runtime table every assigned code SHALL be distinct and never `000` or `111`; a gear
  may be UNSET (after another gear took its code over) and SHALL then never be decoded

#### Scenario: Captured code takes effect immediately
- **WHEN** a gear code is captured or the table is reset to defaults
- **THEN** the gear read cache SHALL be invalidated so the next `getPhysicalGear()` decodes with the
  new table
- **AND** the next gear code SHALL be logged again

#### Scenario: Code 101 is LOW
- **WHEN** the gear code reads `101` with the default table
- **THEN** the physical gear SHALL be `GEAR_LOW`

#### Scenario: Code 001 is HIGH
- **WHEN** the gear code reads `001` with the default table
- **THEN** the physical gear SHALL be `GEAR_HIGH`

#### Scenario: Code 011 is NEUTRAL
- **WHEN** the gear code reads `011` with the default table
- **THEN** the physical gear SHALL be `GEAR_NEUTRAL`

#### Scenario: Code 010 is REVERSE
- **WHEN** the gear code reads `010` with the default table
- **THEN** the physical gear SHALL be `GEAR_REVERSE`

#### Scenario: All lines open reads UNKNOWN
- **WHEN** the gear code reads `000` (e.g. mid-shift, every line open)
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`
- **AND** the unknown-gear behavior SHALL apply (throttle capped at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX`)

#### Scenario: Lost opto feed reads UNKNOWN
- **WHEN** the gear code reads `111` (opto feed lost, every line floats HIGH)
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`

#### Scenario: Unassigned codes read UNKNOWN
- **WHEN** the gear code reads `100` or `110` with the default table (in general: any code not in
  the runtime table)
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`

#### Scenario: Bit 7 does not affect the gear
- **WHEN** port bit 7 changes level while bits 0, 1 and 6 are unchanged
- **THEN** the physical gear SHALL be unchanged

#### Scenario: UNKNOWN code while idle is re-read from later snapshots
- **WHEN** the gear code decodes to `GEAR_UNKNOWN` and no gear change is in progress
- **THEN** `getPhysicalGear()` SHALL keep the previous cached gear and consume the next expander
  poll instead, at most `TRANS_GEAR_READ_RETRY_COUNT` times, without issuing I2C reads of its own
- **AND** once the retries are spent it SHALL report `GEAR_UNKNOWN`

#### Scenario: Gear code changes are logged
- **WHEN** the gear code differs from the last logged code
- **THEN** a TRANSMISSION debug line `[TRANS] gear code=0bXYZ -> <L|H|N|R|UNKNOWN>` SHALL be logged

### Requirement: Brake Control System
The system SHALL control braking force through a linear actuator-driven brake mechanism. The
"fully released" endstop SHALL be read from the opto-isolated brake limit sensor (In5) through the
input expander rather than from a dedicated GPIO.

#### Scenario: Set brake position by percentage
- **WHEN** BrakeSystem.setPosition() is called with value from 0 to 100
- **THEN** brake actuator extends proportionally
- **AND** 0% maps to fully released brakes
- **AND** 100% maps to maximum braking force

#### Scenario: Release brakes
- **WHEN** BrakeSystem.release() is called
- **THEN** brake actuator retracts to 0% position
- **AND** brakes are confirmed released

#### Scenario: Detect the released endstop via the input expander
- **WHEN** `VehicleController::isBrakeReleased()` is called
- **THEN** it SHALL read the brake limit sensor state (In5) from the `BoardInputs` snapshot
- **AND** it SHALL NOT perform a `digitalRead()` or issue its own I2C transaction
- **AND** the added latency (up to `BOARD_INPUT_POLL_MS`, 25 ms) SHALL be acceptable against the
  `BRAKE_SENSOR_OVERRUN_TIME` (1000 ms) overrun window

#### Scenario: Emergency braking
- **WHEN** BrakeSystem.emergencyStop() is called
- **THEN** brakes are immediately applied to 100%
- **AND** throttle is forced to idle
- **AND** transmission remains in current gear
- **AND** emergency braking SHALL NOT depend on the input expander being healthy

#### Scenario: Brake hold on startup
- **WHEN** system powers on or resets
- **THEN** brakes are automatically applied to 30% (parking brake)
- **AND** brakes remain engaged until explicitly released by user command

### Requirement: Vehicle State Coordination
The system SHALL coordinate vehicle systems to ensure safe operation.

#### Scenario: Initialize all systems
- **WHEN** VehicleController.begin() is called
- **THEN** all subsystems are initialized in sequence
- **AND** parking brake is applied (30%)
- **AND** transmission is set to NEUTRAL
- **AND** throttle is set to idle
- **AND** steering is centered
- **AND** system enters READY state

#### Scenario: Query vehicle state
- **WHEN** VehicleController.getState() is called
- **THEN** complete vehicle state is returned including:
  - Steering position and limits
  - Throttle position
  - Current gear
  - Brake position
  - System health status

#### Scenario: Emergency stop all systems
- **WHEN** VehicleController.emergencyStop() is called
- **THEN** emergency braking is applied
- **AND** throttle returns to idle
- **AND** all actuators enter safe state
- **AND** system requires explicit reset to resume operation

### Requirement: Safety Interlocks
The system SHALL enforce safety interlocks between vehicle systems.

#### Scenario: Prevent gear shift under load
- **WHEN** transmission gear change is commanded
- **AND** throttle position is above idle threshold (e.g., >5%)
- **THEN** gear change is rejected
- **AND** error message is logged

#### Scenario: Brake priority over throttle
- **WHEN** both brake and throttle commands are active
- **THEN** brake command takes priority
- **AND** throttle is limited based on brake position

#### Scenario: Steering limits in reverse
- **WHEN** transmission is in REVERSE gear
- **THEN** steering range may be limited (configurable)
- **AND** steering rate may be reduced for safety

### Requirement: System Health Monitoring
The system SHALL monitor health of all vehicle systems and report faults.

#### Scenario: Detect actuator timeout
- **WHEN** actuator command is sent
- **AND** expected movement is not completed within timeout period
- **THEN** fault is logged with system and timestamp
- **AND** affected system enters safe/degraded mode
- **AND** operator is notified

#### Scenario: Watchdog monitoring
- **WHEN** main control loop is running
- **THEN** watchdog timer is refreshed every cycle
- **AND** if loop hangs, watchdog triggers system reset
- **AND** safe state is restored on reset (brakes on, neutral, idle)

#### Scenario: Report system diagnostics
- **WHEN** VehicleController.getDiagnostics() is called
- **THEN** diagnostic data is returned including:
  - Uptime and reset count
  - Calibration status and timestamps
  - Fault history (last N faults)
  - Actuator health status
  - Loop timing statistics

### Requirement: Configuration Management
The system SHALL provide centralized configuration for all vehicle system parameters.

#### Scenario: Load configuration from non-volatile storage
- **WHEN** system starts
- **THEN** configuration parameters are loaded from NVS including:
  - Servo min/max pulse widths
  - Steering rate limits
  - Throttle curve mapping
  - Brake force calibration
  - Safety threshold values

#### Scenario: Update configuration at runtime
- **WHEN** VehicleController.setConfig() is called with new parameters
- **THEN** parameters are validated
- **AND** valid parameters are applied immediately
- **AND** updated configuration is saved to NVS
- **AND** confirmation is returned

#### Scenario: Reset to factory defaults
- **WHEN** VehicleController.resetConfig() is called
- **THEN** all configuration is reset to hardcoded defaults
- **AND** calibration data is preserved (optional flag to clear)
- **AND** system restarts with default configuration

### Requirement: Multi-Source Command Input Support
The system SHALL accept vehicle control commands from multiple input sources (MAVLink, web
interface) with priority management.

#### Scenario: Apply commands from active input source
- **WHEN** control loop executes
- **THEN** input source priority is evaluated (MAVLINK > WEB > FAILSAFE)
- **AND** commands are read from highest priority active source
- **AND** commands are applied to vehicle systems

#### Scenario: Apply MAVLink commands when link active
- **WHEN** the MAVLink command stream is valid (fresh `SERVO_OUTPUT_RAW` within timeout)
- **THEN** MAVLink commands control steering, throttle, transmission, and brakes
- **AND** web control commands are ignored
- **AND** fail-safe is not active

#### Scenario: Apply web commands when MAVLink inactive
- **WHEN** the MAVLink command stream is invalid or timed out
- **AND** web control commands are available
- **THEN** web commands control steering, throttle, transmission, and brakes
- **AND** commands are validated before application
- **AND** fail-safe is not active

#### Scenario: Apply fail-safe when all sources inactive
- **WHEN** the MAVLink command stream is invalid
- **AND** no web control commands received for >1 second
- **THEN** fail-safe commands are applied to all systems
- **AND** vehicle enters safe state (center steering, idle throttle, NEUTRAL, parking brake)

### Requirement: Command Validation and Sanitization
The system SHALL validate all commands regardless of source before applying to vehicle systems.

#### Scenario: Validate steering command range
- **WHEN** steering command is received from any source
- **THEN** value is checked against valid range (-100% to +100%)
- **AND** out-of-range values are clamped to limits
- **AND** validation error is logged with source identifier

#### Scenario: Validate throttle command range
- **WHEN** throttle command is received from any source
- **THEN** value is checked against valid range (0% to 100%)
- **AND** out-of-range values are clamped to limits
- **AND** validation error is logged

#### Scenario: Validate gear selection command
- **WHEN** gear change command is received from any source
- **THEN** gear value is checked (must be R/N/L/H)
- **AND** invalid gear values are rejected
- **AND** gear change safety interlocks are enforced (idle throttle required)

#### Scenario: Validate brake command range
- **WHEN** brake command is received from any source
- **THEN** value is checked against valid range (0% to 100%)
- **AND** out-of-range values are clamped to limits

### Requirement: Input Source Telemetry
The system SHALL provide telemetry about active input source for monitoring.

#### Scenario: Report active input source
- **WHEN** getInputSource() is called
- **THEN** current active input source is returned (MAVLINK/WEB/FAILSAFE)
- **AND** source is updated each control loop cycle

#### Scenario: Include input source in system diagnostics
- **WHEN** VehicleController.getDiagnostics() is called
- **THEN** diagnostic data includes:
  - Current input source
  - Time in current source (seconds)
  - Source switch count (number of source transitions)
  - Last source switch timestamp

### Requirement: Ignition State Control Integration
The system SHALL integrate ignition state control with vehicle systems coordination.

#### Scenario: Set ignition state via vehicle controller
- **WHEN** VehicleController.setIgnitionState() is called with state (OFF/ACC/IGNITION/START)
- **THEN** ignition state is passed to RelayController.setIgnitionState()
- **AND** ignition state change is logged
- **AND** vehicle state telemetry includes new ignition state

#### Scenario: Query current ignition state
- **WHEN** VehicleController.getIgnitionState() is called
- **THEN** current ignition state is returned from RelayController (OFF/ACC/IGNITION/CRANKING)
- **AND** state reflects actual relay configuration

#### Scenario: Monitor engine cranking completion
- **WHEN** VehicleController.update() is called each control loop
- **THEN** RelayController.update() is called with current engine RPM from CAN
- **AND** cranking automatically stops when engine starts (RPM > threshold)
- **AND** cranking automatically stops after 5-second timeout
- **AND** ignition state transitions from CRANKING to IGNITION after cranking completes

### Requirement: Front Light Control Integration
The system SHALL integrate front light control with vehicle systems.

#### Scenario: Set front light state via vehicle controller
- **WHEN** VehicleController.setFrontLight() is called with on/off boolean
- **THEN** light state is passed to RelayController.setFrontLight()
- **AND** light state change is logged
- **AND** vehicle state telemetry includes light state

#### Scenario: Query current light state
- **WHEN** VehicleController.getFrontLight() is called
- **THEN** current light state is returned from RelayController (true/false)
- **AND** state reflects the actual `RELAY_MASK_FRONT_LIGHT` output (Relay_3, IO6, on the aztec buggy)

### Requirement: Ignition Safety Interlocks
The system SHALL enforce safety interlocks for ignition state changes to prevent unsafe operations.

#### Scenario: Require brake applied before ignition state change
- **WHEN** ignition state change is requested (to ACC, IGNITION, or START)
- **AND** current state is OFF
- **THEN** brake position is checked (must be >= 20%)
- **AND** if brake insufficient, ignition change is rejected
- **AND** error message is logged: "Apply brake before ignition"
- **AND** ignition remains in OFF state

#### Scenario: Allow ignition OFF without brake requirement
- **WHEN** ignition state change to OFF is requested
- **THEN** change is allowed regardless of brake position
- **AND** ignition state changes to OFF immediately

#### Scenario: Prevent cranking if engine already running
- **WHEN** START ignition state is requested
- **AND** current engine RPM >= 1100 (ENGINE_RUNNING_RPM_THRESHOLD)
- **THEN** START command is rejected
- **AND** error message is logged: "Engine already running"
- **AND** ignition remains in current state

#### Scenario: Allow cranking when engine not running
- **WHEN** START ignition state is requested
- **AND** current engine RPM < 1100
- **AND** brake is applied (>= 20%)
- **THEN** ignition state changes to CRANKING (same relay config as IGNITION)
- **AND** cranking timer starts (5-second timeout)
- **AND** cranking monitors RPM for engine start detection

#### Scenario: Allow ignition transitions between ACC and IGNITION freely
- **WHEN** ignition state change is requested between ACC and IGNITION
- **THEN** change is allowed without brake or RPM checks
- **AND** only initial power-on from OFF requires safety checks

### Requirement: Ignition and Light System Diagnostics
The system SHALL provide diagnostic information about ignition and lighting systems.

#### Scenario: Include ignition state in vehicle diagnostics
- **WHEN** VehicleController.getDiagnostics() is called
- **THEN** diagnostic data includes:
  - Current ignition state (OFF/ACC/IGNITION/CRANKING)
  - Cranking status (active/inactive)
  - Cranking elapsed time (if active)
  - Ignition state change count

#### Scenario: Include light state in vehicle diagnostics
- **WHEN** VehicleController.getDiagnostics() is called
- **THEN** diagnostic data includes:
  - Front light state (ON/OFF)
  - Light toggle count
  - Relay3 output state

### Requirement: Transmission State Persistence
The system SHALL persist the last confirmed transmission state (gear, servo position as float percent, validity flag) to NVS so that the servo can be restored to its last known position after a clean power cycle.

#### Scenario: Mark state invalid at start of gear change
- **WHEN** `TransmissionController::setGear()` is called
- **THEN** `state_valid=false` SHALL be written to NVS before the servo moves
- **AND** a mid-move power loss will therefore result in `state_valid=false` on next boot

#### Scenario: Save confirmed state on gear arrival
- **WHEN** the physical gear switch confirms the target gear is reached (RETURN phase complete)
- **THEN** `state_valid=true`, `state_gear`, and `state_pct` (float percent) SHALL be written to NVS
- **AND** the saved state reflects the servo position at that moment

#### Scenario: Restore servo position and skip autohome on valid state
- **WHEN** the system starts
- **AND** NVS `state_valid=true`
- **THEN** the servo SHALL be commanded to `state_pct` immediately
- **AND** the normal autohome path SHALL be skipped
- **AND** the system SHALL log "Restored transmission state, skipping autohome"

#### Scenario: Fall back to neutral default on invalid or missing state
- **WHEN** the system starts
- **AND** NVS `state_valid=false`, or no state is saved
- **THEN** the servo SHALL be commanded to the neutral default position (`TRANS_GEAR_DEFAULT_NEUTRAL_PCT`)
- **AND** the system SHALL log the reason (invalid flag / no data)

### Requirement: Throttle Boost During Gear Changes
The system SHALL use a PID controller to regulate engine RPM to a configurable target value (`TRANS_GEAR_BOOST_TARGET_RPM`) for the duration of a gear change, overriding SBUS/web throttle commands while active.

#### Scenario: Activate PID boost when gear change starts
**Given** a gear change is initiated (transmission actuator starts moving toward a non-NEUTRAL gear)
**When** `transmission_.needsThrottleBoost()` returns true
**Then** the gear boost PID SHALL be activated
**And** CAN RPM polling SHALL switch to fast mode (`CAN_POLL_INTERVAL_RPM_BOOST`)
**And** PID integral and derivative state SHALL be reset to zero

#### Scenario: PID holds target RPM during gear change
**Given** the gear boost PID is active
**And** CAN engine RPM data is valid
**When** `updateGearBoostPID()` is called each loop iteration
**Then** the PID error SHALL be `TRANS_GEAR_BOOST_TARGET_RPM - currentRPM`
**And** PID output SHALL be mapped to a throttle servo angle
**And** the throttle servo SHALL be set to the PID-computed angle, overriding SBUS/web commands
**And** the integral term SHALL be clamped to prevent wind-up

#### Scenario: PID output clamped to safe throttle range
**Given** the PID is active and computing output
**When** the computed angle exceeds `THROTTLE_MAX_ANGLE` or falls below `THROTTLE_MIN_ANGLE`
**Then** the output SHALL be clamped to `[THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE]`
**And** the integral term SHALL not accumulate further past the clamp boundary (anti-windup)

#### Scenario: Freeze throttle output when CAN data is stale
**Given** the gear boost PID is active
**And** CAN data is invalid or stale
**When** `updateGearBoostPID()` is called
**Then** the throttle SHALL hold its last computed angle
**And** the integral term SHALL NOT accumulate
**And** a warning SHALL be logged (rate-limited)

#### Scenario: Deactivate PID when gear change completes
**Given** the gear boost PID is active
**When** `transmission_.needsThrottleBoost()` returns false (actuator stopped at target gear)
**Then** the PID SHALL be deactivated
**And** CAN RPM polling SHALL revert to normal rate (`CAN_POLL_INTERVAL_RPM`)
**And** throttle control SHALL return to the current SBUS/web command on the next update cycle

#### Scenario: Safety timeout releases PID boost
**Given** the gear boost PID is active
**When** `TRANS_GEAR_BOOST_TIMEOUT` milliseconds elapse since activation
**Then** the PID SHALL be forcibly deactivated
**And** throttle SHALL return to SBUS/web command
**And** a warning SHALL be logged: "[BOOST] Timeout, releasing gear boost PID"

#### Scenario: PID deactivated if engine is not running
**Given** a gear change is initiated
**And** CAN engine RPM is below `ENGINE_RUNNING_RPM_THRESHOLD`
**When** `updateGearBoostPID()` is called
**Then** the PID SHALL NOT activate (engine not running — boosting throttle is unsafe)
**And** the throttle SHALL remain at idle

### Requirement: 24V Boost Rail Control and Monitoring
The system SHALL enable the external 12V→24V boost converter at boot and SHALL monitor the resulting
rail voltage, because that rail powers the steering VESC, the Jetson, Starlink and the cameras.

#### Scenario: Enable the boost rail at boot
- **WHEN** `setup()` begins
- **THEN** `PIN_BOOST_EN` (GPIO46) SHALL be configured as an output and driven HIGH as the first
  hardware action, before any subsystem initialization
- **AND** the enable SHALL occur before the steering VESC UART is initialized, since the VESC is
  powered from this rail
- **AND** the board's 100K pulldown on GPIO46 SHALL keep the rail off through reset, so no enable
  glitch occurs during boot

#### Scenario: Sample the rail voltage
- **WHEN** `RAIL_SAMPLE_INTERVAL_MS` (500 ms) elapses
- **THEN** `PIN_ADC_24V` (GPIO9, ADC1) SHALL be sampled and averaged over 8 readings
- **AND** the rail voltage SHALL be computed as `Vadc × RAIL_24V_DIVIDER_RATIO` (9.33, from the
  board's 200K/24K divider)
- **AND** the result SHALL be published to telemetry
- **AND** sampling SHALL NOT block the control loop

#### Scenario: Warn on a low rail
- **WHEN** the computed rail voltage is below `RAIL_24V_LOW_THRESHOLD` (21.0 V)
- **AND** more than `BOOST_STARTUP_GRACE_MS` (1000 ms) has elapsed since the boost was enabled
- **THEN** a low-rail warning flag SHALL be raised and published in telemetry
- **AND** a rate-limited warning SHALL be logged
- **AND** the system SHALL NOT automatically disable any subsystem — dropping the rail would remove
  steering authority

#### Scenario: Suppress warnings during startup ramp
- **WHEN** the boost has been enabled for less than `BOOST_STARTUP_GRACE_MS`
- **THEN** the low-rail warning SHALL be suppressed while the converter ramps
- **AND** the measured voltage SHALL still be reported in telemetry

#### Scenario: Report boost enable state
- **WHEN** telemetry is collected
- **THEN** the commanded boost enable state SHALL be reported alongside the measured rail voltage
- **AND** an enabled boost reporting a rail well below threshold SHALL be distinguishable from a
  deliberately disabled boost

### Requirement: Degraded Operation on Board I/O Fault
The system SHALL define safe, observable degraded behavior when the opto-input expander or the
relay expander becomes unavailable, since the gear interlock and the ignition/starter path now
depend on I2C.

#### Scenario: Gear inputs unavailable
- **WHEN** the opto-input snapshot is invalid (stale beyond `BOARD_INPUT_STALE_MS`)
- **THEN** `TransmissionController::getPhysicalGear()` SHALL report `GEAR_UNKNOWN`
- **AND** the existing unknown-gear behavior SHALL apply: throttle capped at
  `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (5%) and gear-change confirmation blocked
- **AND** no additional interlock SHALL be introduced — the existing unknown-gear path is the
  intended degraded mode
- **AND** the fault SHALL be visible in telemetry

#### Scenario: Brake limit sensor unavailable
- **WHEN** the opto-input snapshot is invalid
- **THEN** `VehicleController::isBrakeReleased()` SHALL return `false` ("not confirmed released")
- **AND** brake retraction SHALL remain bounded by `BRAKE_SENSOR_OVERRUN_TIME` and
  `BRAKE_FULL_TRAVEL_TIME` exactly as when the endstop has not yet been reached
- **AND** the system SHALL NOT report the brake as released on the basis of a failed read

#### Scenario: Relay expander unavailable
- **WHEN** the relay expander failed to initialize or is faulted
- **THEN** ignition, starter, front light and wheel lock commands SHALL become logging no-ops
- **AND** the reported ignition state SHALL be `OFF`
- **AND** steering, throttle, brake, MAVLink and web telemetry SHALL continue to operate
- **AND** the fault SHALL be visible in telemetry

#### Scenario: Faults recover without a reboot
- **WHEN** a faulted expander begins responding again
- **THEN** the affected subsystem SHALL resume normal operation automatically
- **AND** the recovery SHALL be logged
- **AND** the telemetry fault flag SHALL clear

### Requirement: Autopilot-Sourced Maximum-Speed Limit
The maximum-speed limiter's operator-set ceiling SHALL have exactly one source: the autopilot's
`SPEED_MAX` parameter, in metres per second. There SHALL be no locally stored ceiling, no local
enable toggle and no arbitration between operator-set sources. The value SHALL be held in RAM
only and SHALL NOT be persisted. The effective limit SHALL be the smaller of the usable
`SPEED_MAX` and the fixed ceiling of the engaged gear (see "Fixed Per-Gear Maximum-Speed
Ceiling"). Whichever side is absent or zero SHALL be ignored. The limiter SHALL always be armed,
and SHALL not limit only when neither side provides a ceiling. The only persisted limiter
parameters SHALL be the throttle floor and the ceiling rate. They shape how the limit is enforced
but SHALL never themselves impose a limit.

#### Scenario: A usable autopilot value is the limit
- **WHEN** the MAVLink interface reports a usable `SPEED_MAX`
- **AND** the engaged gear has no gear ceiling, or a gear ceiling at or above that value
- **THEN** the limit SHALL be that value in metres per second, with source `SPEED_MAX`

#### Scenario: No usable value means no limiting
- **WHEN** the MAVLink interface reports no usable `SPEED_MAX` — never received, zero, rejected,
  stale, or the link is down
- **THEN** `SPEED_MAX` SHALL contribute no ceiling
- **AND** the system SHALL NOT substitute any stored, default or last-known `SPEED_MAX` value
- **AND** the effective limit SHALL be the fixed gear ceiling when the engaged gear has one, and
  otherwise zero
- **AND** a zero effective limit SHALL mean NO LIMITING: the throttle demand SHALL pass through
  untouched

#### Scenario: Zero means "no limit", as the autopilot reads it
- **WHEN** the autopilot's `SPEED_MAX` is zero
- **THEN** `SPEED_MAX` SHALL impose no limit, matching ArduPilot's own reading of a zero
  `SPEED_MAX` and the value a ground station writes to clear a speed limit
- **AND** the fixed gear ceiling SHALL still apply in a gear that has one

#### Scenario: No persisted limiter configuration
- **WHEN** any `SPEED_MAX` value is received, accepted or applied
- **THEN** no non-volatile write SHALL be performed on this path
- **AND** the system SHALL NOT expose any command, key or web control for setting a local ceiling
  or for enabling and disabling the limiter or the gear ceiling
- **AND** the retired non-volatile limiter keys (`lim_on`, `lim_kmh`) SHALL be deleted on
  initialisation so a downgraded unit cannot resurrect a ceiling in the old units
- **AND** the persisted throttle floor SHALL NOT be stored under either retired key name

#### Scenario: The floor alone never limits
- **WHEN** a throttle floor is stored and the effective limit is zero
- **THEN** the demand SHALL pass through untouched, exactly as with no floor stored

#### Scenario: Log a limit change once
- **WHEN** the effective limit changes by more than `SPEED_LIMIT_LOG_EPSILON_MS`, changes between
  "a limit" and "no limit", or its source changes
- **THEN** the new limit SHALL be logged once, in m/s with km/h in parentheses and its source
  (`SPEED_MAX` or the gear ceiling with the gear letter L, H or R), or as "none"
- **AND** a further log SHALL be suppressed until at least `SPEED_LIMIT_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered limit and source SHALL NOT be updated, so the
  settled value is logged on the next opportunity rather than lost

### Requirement: Proportional Speed-Limiter Throttle Taper
The maximum-speed limiter SHALL withdraw throttle authority proportionally as the vehicle
approaches its limit, instead of applying a single reduced cap once the limit is crossed. The
width of the taper band SHALL be proportional to the limit, within an absolute minimum and
maximum. The applied ceiling SHALL be the sum of this proportional taper and the integral trim
(see "Speed-Limiter Integral Trim"), clamped to the range from the configured floor to 100 %.
All comparisons SHALL be made in metres per second. The limiter SHALL be evaluated on every
control-loop iteration for both the autopilot and the web throttle paths. It SHALL continue to
fail open on an invalid speed reading, and SHALL continue to leave the gear-change throttle boost
untouched.

#### Scenario: No limit means no taper
- **WHEN** the limit is zero
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 % and the integral trim to zero, so a limit
  that arrives later never resumes from a stale ceiling or a stale trim

#### Scenario: The taper band scales with the limit
- **WHEN** a limit `L` is in force
- **THEN** the taper band SHALL be `SPEED_LIMIT_TAPER_FRACTION × L` (0.20), but not less than
  `SPEED_LIMIT_TAPER_MIN_MS` (0.5 m/s) and not more than `SPEED_LIMIT_TAPER_MAX_MS` (1.4 m/s)
- **AND** for a limit of 2.78 m/s (10 km/h) the band SHALL be 0.56 m/s, for 5.56 m/s (20 km/h)
  1.11 m/s, and for 11.1 m/s (40 km/h) 1.4 m/s
- **AND** the band SHALL be recomputed on every evaluation from the current limit

#### Scenario: Below the taper band the demand passes through
- **WHEN** a limit is in force and the speed reading is valid
- **AND** the measured speed is at or below `limit − band`
- **THEN** the throttle ceiling SHALL be 100 %
- **AND** the commanded throttle SHALL equal the arbitrated demand

#### Scenario: Inside the taper band the ceiling falls linearly
- **WHEN** the measured speed is between `limit − band` and the limit
- **THEN** the proportional term SHALL fall linearly from 100 % to the configured floor across
  that band
- **AND** the target ceiling SHALL be that proportional term plus the integral trim, clamped to
  the floor and 100 %
- **AND** the commanded throttle SHALL be the lesser of the arbitrated demand and the applied
  ceiling

#### Scenario: At or above the limit the floor holds
- **WHEN** the measured speed is at or above the limit
- **THEN** the proportional term SHALL be the configured floor
- **AND** the target ceiling SHALL be the floor plus the integral trim, which falls while the
  speed stays above the limit
- **AND** the ceiling SHALL never step down to the floor; it SHALL reach it only through the
  taper, the trim and the ceiling rate limit

#### Scenario: The ceiling is rate-limited in both directions
- **WHEN** the computed target ceiling differs from the ceiling currently applied
- **THEN** the applied ceiling SHALL move toward the target by at most
  `SPEED_LIMIT_CEILING_SLEW_PCT_S` per second, whether rising or falling
- **AND** the rate limit SHALL be applied to the ceiling and not to the operator's demand, so the
  driver's own throttle movements are never slowed by the limiter (the separate throttle opening
  rate limit at the servo input may still slow an opening, see "Configurable Throttle Opening
  Rate")
- **AND** the elapsed time used SHALL be capped at one second, and the first evaluation after the
  limiter becomes active SHALL adopt the proportional target directly with a zero trim, so a
  stalled or restarted loop cannot produce a step change through an unbounded interval

#### Scenario: Invalid speed reading fails open
- **WHEN** a limit is in force but the speed reading is invalid or stale
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 % and the integral trim to zero
- **AND** while the source of the effective limit is `SPEED_MAX` (including a tie with the gear
  ceiling), a warning SHALL be logged no more often than `SPEED_LIMIT_WARN_MS`
- **AND** while the source is the fixed gear ceiling — whether `SPEED_MAX` is unusable or a usable
  `SPEED_MAX` lies above the gear ceiling — no warning SHALL be logged

#### Scenario: The web throttle demand is re-limited every loop
- **WHEN** a web throttle command is received
- **THEN** the requested percentage SHALL be recorded as the standing web throttle demand
- **AND** while the web input source is active, no gear boost is running and the throttle is not
  being calibrated, the limiter SHALL be re-applied to that standing demand on every control-loop
  iteration
- **AND** a vehicle that accelerates past the limit on an unchanged web command SHALL therefore be
  limited without the operator sending a new command

#### Scenario: The standing web demand is cleared whenever the throttle is idled
- **WHEN** the throttle is forced to idle — web control is engaged, fail-safe is entered, or the
  gear-change boost releases the throttle
- **THEN** the standing web throttle demand SHALL be reset to zero
- **AND** it SHALL NOT be re-applied on the next iteration, because a demand that outlives an idle
  command would silently reopen the throttle

#### Scenario: Gear-change boost is outside the limiter
- **WHEN** the gear-change throttle boost is active
- **THEN** it SHALL continue to command the throttle servo directly in microseconds
- **AND** the limiter SHALL NOT be applied to it, because the boost holds engine speed while the
  drivetrain is disengaged and no road speed is being produced

#### Scenario: The ceiling slew and the opening rate limit stack
- **WHEN** a limit is in force and the ceiling is rising at the ceiling rate while the demand is
  above it
- **THEN** the limiter output SHALL then pass through the throttle opening rate limit before the
  servo, so on opening the stricter of the two rates SHALL govern the throttle
- **AND** on closing only the ceiling rate limit SHALL slow the fall, the opening rate limit
  passing any lower value through immediately

### Requirement: Engine Hour Meter
The system SHALL accumulate the wall-clock time the ENGINE HAS ACTUALLY BEEN TURNING into a single
total counter — an engine hour meter — held in a dedicated `EngineHourMeter` owned by the vehicle
layer. The counter SHALL be a whole-second `uint64_t`, with the sub-second remainder carried
between updates so that no time is lost to truncation, and SHALL be exposed in hours as a float
(`seconds / 3600`) for presentation only.

The total SHALL be a vehicle-lifetime counter on the same terms as the odometer: it SHALL only ever
increase, and there SHALL be NO command, parameter, magic value, web control or constant that
zeroes or decreases it.

Time SHALL accrue from elapsed-time deltas measured against the millisecond clock, never from a
fixed per-iteration tick, so that the total tracks real time regardless of the control loop's
actual rate. A single delta that is implausibly large SHALL be discarded rather than accumulated,
so that a stalled main loop, a long blocking operation or a `millis()` wrap cannot inject time the
engine did not run.

#### Scenario: Count only while the engine is turning
- **WHEN** the vehicle layer updates the meter and CAN `VehicleData` is valid AND the reported
  engine RPM is at or above `ENGINE_HOURS_MIN_RPM`
- **THEN** the elapsed milliseconds since the previous update SHALL be added to the meter
- **AND** the threshold SHALL be low enough to count a NORMAL IDLE — an hour meter that only counts
  above a driving RPM would under-report the wear that idling causes — and SHALL be a constant
  SEPARATE from `ENGINE_RUNNING_RPM_THRESHOLD`, which is a gear-change safety threshold chosen for
  a different purpose and may sit above idle
- **AND** whole seconds SHALL be carried into the counter while the sub-second remainder is
  retained, so that a sequence of short updates accumulates exactly as one long one would

#### Scenario: Do not count while the engine is stopped or the CAN data is invalid
- **WHEN** CAN `VehicleData` is invalid or stale, or the reported engine RPM is below
  `ENGINE_HOURS_MIN_RPM`
- **THEN** NO time SHALL be added to the meter for that interval
- **AND** the elapsed time SHALL be discarded rather than banked, so that resuming valid CAN data
  does not credit the meter with the silent interval
- **AND** the meter SHALL NOT fall back to any assumed or inferred engine state: an unknown engine
  state SHALL NOT invent running time, on the same principle by which the odometer ignores the
  decayed speed estimate

#### Scenario: Discard an implausibly long update interval
- **WHEN** the measured interval between two updates exceeds `ENGINE_HOURS_MAX_DELTA_MS`
- **THEN** that interval SHALL be discarded entirely rather than accumulated
- **AND** the meter SHALL resume counting from the current instant, so a stalled loop, a blocking
  operation or a `millis()` wrap costs at most the discarded interval and can never add time

#### Scenario: Survive a power cycle
- **WHEN** the firmware starts
- **THEN** the meter SHALL be loaded from its own NVS namespace, a key that has never been written
  reading as 0 rather than as an error
- **AND** the restored value SHALL be logged, so the operator can confirm it survived
- **AND** the meter SHALL be written back whenever it has grown by
  `ENGINE_HOURS_NVS_WRITE_INTERVAL_S` seconds since the last write, and immediately on an ignition
  OFF transition or on entry into fail-safe, so a normal shutdown loses nothing and an unexpected
  power cut loses at most that interval
- **AND** the write interval SHALL be a hardcoded constant: no new config key, no web-settable
  parameter and no runtime tuning SHALL be introduced

#### Scenario: A failed NVS write never blocks the control loop
- **WHEN** the NVS namespace cannot be opened for writing, or the write fails
- **THEN** the failure SHALL be logged and the call SHALL return
- **AND** the in-RAM counter SHALL remain correct and SHALL continue accumulating
- **AND** the firmware SHALL NOT retry in a loop, block, or reset

#### Scenario: The total cannot be reset
- **WHEN** any MAVLink command, web command or configuration write is processed
- **THEN** there SHALL be NO code path that zeroes or decreases the TOTAL engine-seconds counter,
  the only assignments to it being the NVS load at startup and the accumulation itself
- **AND** the only way back to zero SHALL be an erase of the NVS partition
- **AND** the trip reset SHALL be the one operation that zeroes anything, and it SHALL leave the
  total untouched

#### Scenario: Expose the meter to the telemetry and MAVLink layers
- **WHEN** the telemetry layer or the MAVLink layer asks for the accumulated running time
- **THEN** the vehicle controller SHALL expose it both in exact seconds and as hours in float
- **AND** the float form SHALL be understood as a presentation of the counter, never as the counter
  itself — it SHALL NOT be read back into, re-accumulated from, or used to reconstruct the seconds
- **AND** the existing `isEngineRunning()` predicate, the odometer, the trip meter and the speed
  sensor SHALL be left untouched by the meter

### Requirement: Trip Engine Hours
The system SHALL maintain, beside the total engine hour meter and in the same owning class, a
RESETTABLE TRIP engine-hours counter ("мотогодини місії") — the hour-meter analogue of the
odometer's resettable TRIP distance. It SHALL be a whole-second `uint64_t` exposed in hours as a
float for presentation only, on the same terms as the total.

The trip counter SHALL be cleared ONLY by the SAME operation that clears the TRIP DISTANCE, because
the two describe the same "since the operator last reset" interval in different units and SHALL NOT
be allowed to diverge. There SHALL be NO separate command, NO additional magic parameter value, NO
web control and NO new constant for it.

#### Scenario: Count in lockstep with the total
- **WHEN** the meter accrues time under the counting rule of the Engine Hour Meter requirement
- **THEN** the SAME whole-second amount SHALL be added to the trip counter as to the total, at the
  same point and from the SAME single sub-second remainder carry
- **AND** a SECOND remainder SHALL NOT be maintained for the trip, so that the two counters cannot
  drift apart by repeated independent rounding of the same elapsed time
- **AND** every rule that stops the total accruing — invalid CAN, sub-threshold RPM, an
  implausibly long update interval — SHALL stop the trip accruing identically

#### Scenario: Persist the trip counter with the total
- **WHEN** the meter is written to non-volatile storage
- **THEN** the trip counter SHALL be written alongside the total, in the same namespace, under the
  same dirty flag, on the same triggers, by the same write call — no new trigger SHALL be added
- **AND** the write SHALL be treated as successful only when BOTH values were written, so that a
  partial write leaves the pair dirty for a later retry rather than marking it clean
- **AND** a trip key that has never been written SHALL read as 0, so a vehicle upgrading from a
  total-only build keeps its total and starts the trip at zero

#### Scenario: Zero the trip hours on the trip reset
- **WHEN** the vehicle layer performs the latched TRIP reset that zeroes the trip DISTANCE
- **THEN** the trip engine-hours counter SHALL be zeroed in the same operation
- **AND** the reset SHALL be persisted IMMEDIATELY rather than waiting for the next interval or
  ignition-OFF write, so a deliberate operator gesture survives a power cut
- **AND** the reset SHALL be logged, naming the unchanged total so the operator can see it survived
- **AND** the trip-hours reset SHALL have EXACTLY ONE call site, that one, and the inbound command
  handler SHALL NOT be changed to accommodate it

#### Scenario: The reset leaves the total untouched
- **WHEN** the trip engine-hours counter is reset
- **THEN** the TOTAL engine-seconds counter SHALL be unchanged, and no code path SHALL exist by
  which resetting the trip can decrease the total
- **AND** a zero trip reading afterwards SHALL be a GENUINE zero, not an "unknown" or a sentinel

#### Scenario: The trip counter survives a power cycle
- **WHEN** the firmware starts
- **THEN** the trip counter SHALL be restored from non-volatile storage alongside the total
- **AND** both restored values SHALL be logged together in one line
- **AND** a power cycle, an ignition cycle or a fail-safe SHALL NOT be treated as a reset: only the
  explicit trip reset zeroes it

### Requirement: Speed-Scaled Steering Authority
The system SHALL reduce the steering authority of autopilot-sourced steering commands as road
speed rises, using ArduPilot's own formula and the locally measured, locally validated wheel
speed. The scaling SHALL be applied on the ESP32 rather than on the autopilot, because the
autopilot's speed source silently falls back to raw GPS ground speed when its velocity estimate is
unavailable — which is precisely when this vehicle's speed source has failed. The scaling SHALL be
treated as a comfort and assist feature: **every fault condition SHALL result in no scaling at
all**, never in a reduced, held or substituted authority.

#### Scenario: Scale the deviation from centre
- **WHEN** the scaling is active and a steering command is received from the autopilot
- **THEN** the scale SHALL be computed as `min(1, base / speed)`, where `speed` is the measured
  road speed in metres per second and `base` is the speed-scaling base value in metres per second
- **AND** the commanded steering percentage SHALL be multiplied by that scale
- **AND** because the steering percentage is already signed about a centre of zero, this SHALL
  scale the deviation from centre and SHALL leave a centred command centred

#### Scenario: At or below the base speed there is no reduction
- **WHEN** the measured speed is at or below the base value
- **THEN** the scale SHALL be 1
- **AND** the steering command SHALL pass through unmodified
- **AND** a stationary vehicle and a reading that has decayed to zero SHALL therefore need no
  special case, because both satisfy this condition

#### Scenario: A non-positive base disables the scaling
- **WHEN** the base value is zero or negative
- **THEN** the scale SHALL be 1
- **AND** this SHALL match ArduPilot's own reading of a non-positive `MOT_SPD_SCA_BASE`

#### Scenario: An invalid speed reading disables the scaling entirely
- **WHEN** the speed sensor reports that its reading is not valid — for any reason, including
  never having pulsed since boot, a latched implausible-loss flag, or the sensor not being
  initialised
- **THEN** the scale SHALL be 1 and the steering command SHALL pass through unmodified
- **AND** the system SHALL NOT hold the last computed scale, SHALL NOT substitute a conservative
  or last-known speed, and SHALL NOT apply any minimum-scale floor
- **AND** a warning SHALL be logged no more often than `STEER_SCALE_WARN_MS`
- **AND** this SHALL be the case even while the vehicle is moving fast, because a fault must never
  be able to leave the driver with restricted steering

#### Scenario: Scaling applies only in the autopilot's MANUAL mode
- **WHEN** the learned autopilot reports a Rover flight mode other than MANUAL
- **THEN** the scale SHALL be 1
- **AND** the steering command SHALL pass through unmodified, because in those modes the autopilot
  computes steering from speed itself and a second, invisible reduction on top of it would be
  double limiting

#### Scenario: An unknown or stale mode is treated as MANUAL
- **WHEN** no flight mode has been received from the autopilot, or the heartbeat carrying it is
  stale
- **THEN** the mode SHALL be treated as MANUAL and the scaling SHALL apply
- **AND** this SHALL be the single place where an unavailable input does not disable the scaling,
  because MANUAL is the mode this vehicle is driven in and the cost of guessing wrong is only that
  a mode which would have been scaled by the autopilot is scaled here instead

#### Scenario: The web steering path is never scaled
- **WHEN** a steering command arrives from the web portal rather than from the autopilot
- **THEN** it SHALL be passed to the steering controller unscaled
- **AND** the scaling SHALL remain a property of the autopilot command path only, because the web
  path is a bench and maintenance control with no road speed behind it

#### Scenario: The applied scale is rate-limited in both directions
- **WHEN** the computed target scale differs from the scale currently applied
- **THEN** the applied scale SHALL move toward the target by at most `STEER_SCALE_SLEW_PER_S` per
  second, whether rising or falling
- **AND** the rate limit SHALL be applied to the scale and not to the steering command, so the
  driver's own steering movements are never slowed by the assist
- **AND** the elapsed time used SHALL be capped at one second, and the first evaluation after the
  scaling becomes active SHALL adopt the target directly, so a stalled or restarted loop cannot
  produce a step change through an unbounded interval

#### Scenario: Log a scale change once
- **WHEN** the applied scale changes by more than `STEER_SCALE_LOG_EPSILON`, or changes between
  "scaling" and "not scaling"
- **THEN** the new scale SHALL be logged once, together with the speed and the base that produced
  it and the source of that base
- **AND** a further log SHALL be suppressed until at least `STEER_SCALE_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered scale SHALL NOT be updated, so the settled value
  is logged on the next opportunity rather than lost

#### Scenario: The autopilot's own scaling must be switched off by the operator
- **WHEN** this firmware is deployed
- **THEN** the documentation SHALL require `MANUAL_OPTIONS = 0` on the autopilot as a commissioning
  step, so the steering is not scaled twice
- **AND** the firmware SHALL NOT write that parameter, or any other autopilot parameter, because
  the firmware never sends `PARAM_SET`

### Requirement: Steering Speed-Scaling Base Value
The speed-scaling base SHALL be taken from a locally stored value first and from the autopilot's
`MOT_SPD_SCA_BASE` second, and SHALL be absent — meaning no scaling — when neither is available.
The local value SHALL be persisted in non-volatile storage and SHALL be settable from the web
portal without a reflash.

#### Scenario: The locally stored value takes priority
- **WHEN** a base value is needed and the non-volatile key `steer_sca_base` holds a positive value
  in metres per second
- **THEN** that value SHALL be used
- **AND** the autopilot's `MOT_SPD_SCA_BASE` SHALL NOT be consulted, so a vehicle can be
  commissioned and driven with the scaling working before its autopilot has been touched

#### Scenario: Fall back to the autopilot parameter
- **WHEN** the locally stored value is absent or zero
- **AND** the MAVLink interface reports a usable `MOT_SPD_SCA_BASE`
- **THEN** that value SHALL be used as the base
- **AND** it SHALL be used in metres per second, unconverted, because it is already the firmware's
  internal speed unit

#### Scenario: Neither source means no scaling
- **WHEN** the locally stored value is absent or zero
- **AND** the MAVLink interface reports no usable `MOT_SPD_SCA_BASE` — never received, zero,
  rejected, stale, or the link is down
- **THEN** the base SHALL be reported as zero and the scale SHALL be 1
- **AND** a warning SHALL be logged no more often than `STEER_SCALE_WARN_MS`
- **AND** the system SHALL NOT substitute a compile-time default, because a base nobody chose
  would silently restrict the steering of an unconfigured vehicle

#### Scenario: Load and validate the stored base on startup
- **WHEN** the vehicle controller initialises
- **THEN** `steer_sca_base` SHALL be read from non-volatile storage
- **AND** a stored value that is not-a-number, negative, or greater than
  `STEER_SCALE_BASE_MAX_MS` SHALL be ignored and treated as absent
- **AND** an absent key SHALL be treated as zero, that is, as "not set"

#### Scenario: Set the base at runtime
- **WHEN** a `set_steer_sca_base` web command is received with a value between zero and
  `STEER_SCALE_BASE_MAX_MS` inclusive
- **THEN** the value SHALL be applied immediately and persisted to non-volatile storage so it
  survives a reboot
- **AND** a value of zero SHALL be accepted and SHALL mean "use the autopilot's value, or none"
- **AND** the command SHALL succeed regardless of the active input source, at the same privilege
  level as the existing calibration setters

#### Scenario: Reject an out-of-range base
- **WHEN** a `set_steer_sca_base` command carries a value that is not-a-number, negative, or
  greater than `STEER_SCALE_BASE_MAX_MS`
- **THEN** the command SHALL be rejected with an error response
- **AND** the stored and applied base SHALL be left unchanged

### Requirement: Steering Speed-Scaling Bench Override
The system SHALL provide a bench test override that substitutes a speed for the steering
speed-scaling calculation ONLY, so the feature can be exercised on a stationary, disarmed vehicle.
The override SHALL be volatile, SHALL expire on its own, and SHALL be visible while it is live.

#### Scenario: Substitute a speed for the scaling calculation
- **WHEN** a `set_test_speed` web command is received with a value between zero and
  `STEER_SCALE_TEST_SPEED_MAX_MS`
- **THEN** the steering speed-scaling calculation SHALL use that value in place of the measured
  speed
- **AND** the speed-sensor validity gate SHALL still apply, so the override cannot be used to scale
  steering while the sensor is unhealthy

#### Scenario: The override reaches nothing but the steering scale
- **WHEN** a test speed override is active
- **THEN** the wheel odometry reported to the autopilot, the reported ground speed, the
  maximum-speed throttle limiter, the transmission speed interlock and the odometer SHALL all
  continue to use the real measured speed
- **AND** the bench configuration SHALL therefore be structurally incapable of becoming a driving
  configuration

#### Scenario: The override is volatile and expires
- **WHEN** a test speed override is set
- **THEN** it SHALL be held in RAM only and SHALL NOT be persisted
- **AND** it SHALL clear itself automatically after `STEER_SCALE_TEST_SPEED_MS`
- **AND** it SHALL be cleared by a reboot
- **AND** setting it to zero SHALL clear it immediately

#### Scenario: The override is announced while it is live
- **WHEN** a test speed override is active
- **THEN** the steering scale log line SHALL mark the speed as `TEST`
- **AND** the telemetry stream SHALL report that an override is in force and what its value is

### Requirement: Front-Wheel Lock Not Fitted
The aztec buggy SHALL have no relay behind the front-wheel lock function (`RELAY_WHEEL_LOCK_FITTED =
0`). The wheel-lock plumbing (MAVLink servo channel 7, web `set_wheel_lock` command, telemetry fields)
SHALL be kept so a later change can replace it with the buggy's three locks, but a lock request SHALL
never be reported as applied.

#### Scenario: Lock request is rejected
- **WHEN** `VehicleController::setWheelLock(true)` is called from the MAVLink or the web path
- **THEN** `RelayController::setWheelLock()` SHALL return `false` without writing the relay port
- **AND** the first rejection after boot SHALL be logged; later ones SHALL NOT flood the log
- **AND** `getWheelLock()` SHALL return `false`

#### Scenario: Release request is a no-op
- **WHEN** `setWheelLock(false)` is called (every control loop while MAVLink channel 7 is low)
- **THEN** it SHALL return `true` without writing the relay port and without logging

#### Scenario: Fitted state is queryable
- **WHEN** `VehicleController::isWheelLockFitted()` or `RelayController::isWheelLockFitted()` is called
- **THEN** it SHALL return `false`

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

#### Scenario: Mirrored sensor caught at calibration
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

### Requirement: Quad Bike Relay Assignment With Fitted Front-Wheel Lock
On the quad bike (`master`) build the relay board SHALL use the quad assignment, defined in
`Constants.h` as relay-number sets and mapped at `begin()` through the detected board's Relay_N → bit
table (masks shown for the PCA9557 / XL9535 board):
- ignition/ECU line = Relay_1 + Relay_2, paralleled (`RELAY_SET_IGNITION`; mask `0b00110000` /
  `0b00000011`);
- starter = Relay_3 (`0b01000000` / `0b00000100`);
- front light = Relay_4 (`0b10000000` / `0b00001000`);
- front-wheel lock = Relay_5 + Relay_6, paralleled (`RELAY_SET_WHEEL_LOCK`; `0b00001100` /
  `0b00110000`);
- Relay_7 / Relay_8 spare (`RELAY_SET_RESERVED`; `0b00000011` / `0b11000000`).

The front-wheel lock SHALL be fitted (`RELAY_WHEEL_LOCK_FITTED` = 1). It replaces, for this build
only, the aztec buggy's "Front-Wheel Lock Not Fitted" behaviour.

#### Scenario: Wheel lock from MAVLink channel 7
- **WHEN** MAVLink servo channel 7 is above 1520 µs (e.g. 2000)
- **THEN** Relay_5 and Relay_6 SHALL both be energized, `getWheelLock()` SHALL return `true`,
  telemetry `wheel_lock_on` SHALL be `true` and `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL be set
- **AND WHEN** channel 7 returns to 1000 µs
- **THEN** both relays SHALL be released and the lock SHALL report unlocked

#### Scenario: Same relays on either board
- **WHEN** the quad build runs with the PCA9557 board or with the XL9535 board
- **THEN** the same relay numbers SHALL carry the same functions, and only the port masks SHALL differ

#### Scenario: Wheel lock reported as fitted
- **WHEN** telemetry is formatted on the quad build
- **THEN** `wheel_lock_fitted` SHALL be `true` and the web wheel-lock toggle SHALL be usable

#### Scenario: Fail-safe all-off keeps the wheel lock
- **WHEN** `RelayController::allOff()` runs
- **THEN** ignition, starter and light SHALL be switched off
- **AND** the wheel lock SHALL hold its last commanded state

#### Scenario: Relay test respects the wheel lock
- **WHEN** a web relay test asks to energize any relay while the wheel lock is engaged
- **THEN** the request SHALL be refused with a reason naming the wheel lock
- **AND WHEN** a lock command arrives while test relays are on
- **THEN** all test relays SHALL be switched off before the lock is applied
- **AND** the 2 s starter test cap SHALL apply to Relay_3

### Requirement: Quad Bike One-Line-Per-Gear Sensing
On the quad bike (`master`) build each gear SHALL be sensed on its own opto input line on the
input expander: NEUTRAL = port bit 0, REVERSE = bit 1, LOW = bit 6, HIGH = bit 7, engaged = HIGH.
Exactly one engaged line SHALL decode to its gear. No engaged line, or more than one, SHALL decode
to `GEAR_UNKNOWN`. The aztec 3-bit gear code table SHALL NOT be built on this build.

#### Scenario: Single line engaged
- **WHEN** only port bit 6 reads HIGH among bits 0, 1, 6 and 7
- **THEN** the physical gear SHALL be LOW

#### Scenario: Ambiguous or empty reading
- **WHEN** none of bits 0, 1, 6, 7 reads HIGH, or two or more do
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`

#### Scenario: No gear-code calibration on the quad
- **WHEN** telemetry is formatted on the quad build
- **THEN** `gear_code`, `gear_sensed`, `gear_codes` and `gear_codes_custom` SHALL be absent
- **AND** the web "Gear Sensor Codes" card SHALL be hidden
- **AND** a `set_gear_code` or `reset_gear_codes` command SHALL be answered with failure
  "Gear codes not used on this vehicle"

### Requirement: Quad Bike Steering VESC on UART2
On the quad bike (`master`) build the steering VESC SHALL be driven over UART2 on connector X6
(GPIO4 TX → VESC RX, GPIO5 RX ← VESC TX, via the BSS138 level shifters, 115200 baud). UART0
(GPIO43/44) SHALL stay connected to the CH9121T and SHALL carry no console and no runtime log
output.

#### Scenario: VESC link on X6
- **WHEN** the firmware starts on the quad build
- **THEN** the VESC driver SHALL open `UART_NUM_2` on GPIO5 (RX) / GPIO4 (TX)
- **AND** a valid `COMM_GET_VALUES` reply SHALL set the steering driver healthy

#### Scenario: No log output on UART0
- **WHEN** an ESP-IDF component or the Arduino core logs at runtime
- **THEN** the output SHALL go to the USB-CDC console only, never to UART0

### Requirement: Speed-Limiter Integral Trim
The maximum-speed limiter SHALL carry an integral trim, in percent, that is added to the
proportional taper so the steady-state speed converges to the limit whatever throttle is needed
to hold it, as long as that throttle lies between the configured floor and 100 %. The trim SHALL
only remove authority while the vehicle is above the limit, and SHALL only return authority while
the vehicle is below the limit and the limiter is actually restricting the demand. The trim SHALL
be bounded so the applied ceiling always stays between the floor and 100 %, and it SHALL never
move faster than the ceiling rate limit.

#### Scenario: Over the limit the trim falls in proportion to the excess
- **WHEN** a limit is in force, the speed reading is valid, and the measured speed exceeds the
  limit by `e` m/s
- **THEN** the trim SHALL decrease by `SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS × e × dt` (30 %/s per
  m/s), but by no more than `SPEED_LIMIT_CEILING_SLEW_PCT_S × dt`
- **AND** an excess of 1.4 m/s SHALL therefore take the trim from 100 % to zero in about 2.4 s
- **AND** this SHALL apply whether or not the demand is below the ceiling, because lowering the
  ceiling while over the limit is always the conservative direction

#### Scenario: Under the limit and binding the trim recovers gradually
- **WHEN** the measured speed is below the limit but above `limit − band`
- **AND** the demand is at or above the applied ceiling minus `SPEED_LIMIT_BIND_MARGIN_PCT` (1 %)
- **THEN** the trim SHALL increase by `SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS × (limit − speed) × dt`,
  but by no more than `SPEED_LIMIT_TRIM_RISE_PCT_S × dt` (10 %/s)
- **AND** the resulting ceiling rise SHALL never exceed the ceiling rate limit

#### Scenario: No accumulation while the driver is below the ceiling
- **WHEN** the measured speed is below the limit and the demand is below the applied ceiling by
  more than `SPEED_LIMIT_BIND_MARGIN_PCT`
- **THEN** the trim SHALL be held unchanged
- **AND** a driver who lifts off and re-applies the throttle SHALL meet the proportional taper
  for the current speed plus a trim that has not grown in the meantime

#### Scenario: The trim is bounded
- **WHEN** the trim is updated
- **THEN** it SHALL be clamped to the range from zero to `100 − floor`
- **AND** the target ceiling SHALL be clamped to the range from the floor to 100 %

#### Scenario: The trim is reset outside its working range
- **WHEN** the measured speed is at or below `limit − band`, the limit is zero, the speed reading
  is invalid, or the limiter evaluates for the first time after any of these
- **THEN** the trim SHALL be zero
- **AND** the next approach to the limit SHALL meet the full proportional taper

#### Scenario: Steady state converges to the limit
- **WHEN** the vehicle is held against the limit on a constant grade and the throttle `u` that
  holds the limit lies between the floor and 100 %
- **THEN** the applied ceiling SHALL settle at `u` and the measured speed SHALL settle at the limit
  within the speed-sensor resolution, rather than at a point inside the taper band below it

#### Scenario: The law is host-testable
- **WHEN** the limiter law is built
- **THEN** the band, taper, trim and slew arithmetic SHALL live in a pure header with no Arduino or
  `Constants.h` dependency
- **AND** it SHALL be covered by a native Unity test suite

### Requirement: Configurable Speed-Limiter Throttle Floor
The lowest throttle ceiling the maximum-speed limiter may apply SHALL be a persisted, operator-set
value in percent of the calibrated throttle span. It SHALL default to
`SPEED_LIMIT_FLOOR_DEFAULT_PCT` (0 %, the calibrated idle position) and SHALL be accepted in the
range `SPEED_LIMIT_FLOOR_MIN_PCT`–`SPEED_LIMIT_FLOOR_MAX_PCT` (0–20 %). It SHALL be stored in NVS
namespace `"speed"` under the key `thr_floor` (float, percent), loaded at start-up and written
immediately when changed. It SHALL apply to both vehicles alike.

#### Scenario: Default floor on a unit without a stored value
- **WHEN** the system starts and `thr_floor` has never been written
- **THEN** the floor SHALL be 0 %
- **AND** at and above the limit the limiter SHALL therefore be able to bring the throttle to the
  calibrated idle position

#### Scenario: Stored floor is restored
- **WHEN** the system starts and `thr_floor` holds a value within 0–20 %
- **THEN** that value SHALL be the floor

#### Scenario: Invalid stored floor is ignored
- **WHEN** the system starts and `thr_floor` holds NaN or a value outside 0–20 %
- **THEN** the stored value SHALL be ignored and logged, and the default SHALL be used
- **AND** the stored value SHALL NOT be repaired or rewritten

#### Scenario: Floor changed at runtime
- **WHEN** a valid new floor is set
- **THEN** it SHALL be persisted to `thr_floor` immediately
- **AND** it SHALL take effect on the next limiter evaluation, with the trim and the ceiling
  re-clamped to the new range and the ceiling still moving no faster than its rate limit

#### Scenario: The floor key does not collide with retired or sensor keys
- **WHEN** the speed sensor removes the retired limiter keys at start-up
- **THEN** `thr_floor` SHALL be left untouched
- **AND** the `"speed"` namespace SHALL never be cleared as a whole

### Requirement: Configurable Throttle Opening Rate
The rate at which the throttle may OPEN SHALL be a persisted, operator-set, per-vehicle value in
percent per second, applied by `ThrottleController::setThrottlePercent()` through the pure law
`throttleSlewStep(targetPct, lastPct, dtS, ratePctS)` (`include/ThrottleSlewLaw.h`). A value of 0
SHALL turn the limit off; any other accepted value SHALL lie in
`THROTTLE_OPEN_SLEW_MIN_PCT_S`–`THROTTLE_OPEN_SLEW_MAX_PCT_S` (30–100 %/s). The default SHALL be
`THROTTLE_OPEN_SLEW_DEFAULT_PCT_S` = 0 (off) on every vehicle; a non-zero rate is optional and
set by the operator (for example 60 %/s, idle to full in ≈1.67 s).
It SHALL be stored in the throttle calibration NVS namespace `"throttle"` under the key
`open_slew` (float, %/s), loaded at start-up and written only when the operator sets a new value;
the firmware SHALL NOT write it at start-up, on first run or when the default is used. Closing the
throttle SHALL never be rate-limited.

#### Scenario: The law limits rising and passes falling
- **WHEN** `throttleSlewStep` is called with a rate above 0
- **THEN** a target above `lastPct` SHALL return `min(target, lastPct + rate × dt)`
- **AND** a target at or below `lastPct` SHALL return the target unchanged
- **AND** the target SHALL be clamped to 0–100 % first

#### Scenario: Rate 0 is off
- **WHEN** the opening rate is 0
- **THEN** every percentage command SHALL reach the servo unchanged, exactly as before this
  setting existed

#### Scenario: Default rate on a unit without a stored value
- **WHEN** the system starts and `open_slew` has never been written
- **THEN** the opening rate SHALL be 0 (off) and the throttle SHALL open without a ramp
- **AND** `open_slew` SHALL still be absent afterwards

#### Scenario: Stored rate is restored
- **WHEN** the system starts and `open_slew` holds 0 or a value within 30–100 %/s
- **THEN** that value SHALL be the opening rate
- **AND** this SHALL hold after a firmware update that changed the default, so a unit on which
  60 %/s was saved keeps 60 %/s until the operator sets another value

#### Scenario: Invalid stored rate is ignored
- **WHEN** the system starts and `open_slew` holds NaN, a negative value, a value between 0 and
  30 %/s exclusive, or a value above 100 %/s
- **THEN** the stored value SHALL be ignored and logged, and the default (0, off) SHALL be used
- **AND** the stored value SHALL NOT be repaired or rewritten

#### Scenario: Rate changed at runtime
- **WHEN** a valid new rate is set
- **THEN** it SHALL be persisted to `open_slew` immediately, and the value in use SHALL change only
  after the write succeeded
- **AND** it SHALL take effect on the next percentage command

#### Scenario: Snap from idle on the bench
- **WHEN** the rate is 60 %/s and the throttle is commanded from idle to 100 % in one step
- **THEN** the servo SHALL take ≈1.7 s to reach full throttle
- **AND** with the rate at 0 (the default) the servo SHALL snap open with no ramp

### Requirement: Fixed Per-Gear Maximum-Speed Ceiling
On the buggy (aztec), the maximum-speed limiter SHALL also enforce a fixed ceiling for the
engaged gear. The ceilings SHALL be firmware constants in metres per second:
`GEAR_SPEED_CEILING_LOW_MS` (45 km/h) for L, `GEAR_SPEED_CEILING_HIGH_MS` (75 km/h) for H and
`GEAR_SPEED_CEILING_REVERSE_MS` (20 km/h) for R. N and an uninitialised (UNKNOWN) expected gear
SHALL have no gear ceiling. The engaged
gear SHALL be the transmission's expected (target) gear, the same gear the N/UNKNOWN throttle clip
uses, not the opto gear-sensor reading. The ceilings SHALL NOT be configurable at runtime: no web
control, no non-volatile key and no MAVLink parameter.

#### Scenario: Gear ceiling applies alone when SPEED_MAX is "no limit"
- **WHEN** no usable `SPEED_MAX` is available (zero, never received, rejected, stale or link down)
- **AND** the expected gear is L
- **THEN** the effective limit SHALL be `GEAR_SPEED_CEILING_LOW_MS` (12.5 m/s, 45 km/h)
- **AND** in H it SHALL be `GEAR_SPEED_CEILING_HIGH_MS` (≈20.83 m/s, 75 km/h)
- **AND** in R it SHALL be `GEAR_SPEED_CEILING_REVERSE_MS` (≈5.56 m/s, 20 km/h)

#### Scenario: The lower of SPEED_MAX and the gear ceiling wins
- **WHEN** a usable `SPEED_MAX` and a gear ceiling are both in force
- **THEN** the effective limit SHALL be the smaller of the two
- **AND** when they are equal the source SHALL be reported as `SPEED_MAX`
- **AND** a `SPEED_MAX` above the gear ceiling SHALL NOT raise the limit above the gear ceiling

#### Scenario: Reverse is capped at 20 km/h
- **WHEN** the expected gear is R
- **THEN** the effective limit SHALL be the smaller of the usable `SPEED_MAX` and
  `GEAR_SPEED_CEILING_REVERSE_MS` (20 km/h), or 20 km/h when there is no usable `SPEED_MAX`

#### Scenario: No gear ceiling in N or an unknown gear
- **WHEN** the expected gear is N or UNKNOWN
- **THEN** the gear SHALL contribute no ceiling, and the effective limit SHALL be the usable
  `SPEED_MAX`, or no limit
- **AND** the throttle there SHALL remain clipped to `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` as before

#### Scenario: Enforced by the existing limiter law
- **WHEN** the gear ceiling is the effective limit
- **THEN** it SHALL be enforced exactly as a `SPEED_MAX` limit: the same taper band, floor, ceiling
  rate limit, integral trim and saturation indicator
- **AND** an invalid speed reading SHALL fail open in the same way
- **AND** while the gear ceiling is the source of the effective limit, the "speed reading invalid"
  warning SHALL NOT be logged, even when a usable `SPEED_MAX` above the gear ceiling is in force

#### Scenario: Changing gear causes no throttle step
- **WHEN** the expected gear changes and the effective limit changes with it
- **THEN** the limiter state SHALL NOT be reset, and the next evaluation SHALL use the new limit
- **AND** because gear changes are only permitted at or below `TRANS_SPEED_INTERLOCK_THRESHOLD_MS`,
  below every taper band, the applied ceiling SHALL already be 100 % at that moment

