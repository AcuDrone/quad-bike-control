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
- **WHEN** `TRANS_OVERSHOOT_DWELL_MS`, `TRANS_SEQUENCE_STEP_DWELL_MS` or `TRANS_DWELL_CONFIRM_MS` are modified in `Constants.h`
- **THEN** the overshoot-dwell timeout, the inter-step pause timeout and the sensor confirmation window SHALL reflect the updated values without additional code changes

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
- **WHEN** the gear code decodes to `GEAR_UNKNOWN`, no gear change is in progress and no sequence step is queued
- **THEN** `getPhysicalGear()` SHALL keep the previous cached gear and consume the next expander
  poll instead, at most `TRANS_GEAR_READ_RETRY_COUNT` times, without issuing I2C reads of its own
- **AND** once the retries are spent it SHALL report `GEAR_UNKNOWN`
- **AND** while a gear change or a queued sequence pause is in progress the reading SHALL be decoded
  immediately (no deferral), so a stale cached gear can never count as a confirmation

#### Scenario: Gear code changes are logged
- **WHEN** the gear code differs from the last logged code
- **THEN** a TRANSMISSION debug line `[TRANS] gear code=0bXYZ -> <L|H|N|R|UNKNOWN>` SHALL be logged

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
- **THEN** RelayController.update() SHALL be called with the validated engine RPM (see "Validated
  Engine RPM for Crank Control"): the CAN engine RPM when the CAN data is valid and 0 otherwise on
  the quad bike, the effective RPM (tacho first, 0 when no source is valid) on the aztec buggy
- **AND** cranking automatically stops when the engine starts (validated RPM ≥
  `ENGINE_RUNNING_RPM_THRESHOLD`)
- **AND** cranking automatically stops once the starter has run for the starter time limit of the
  attempt (see "Configurable Starter Time Limit"; 2–10 s, default 5 s = `CRANKING_TIMEOUT`)
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
- **AND** the EFFECTIVE engine RPM (tacho when fitted, CAN otherwise) is valid and >=
  `ENGINE_RUNNING_RPM_THRESHOLD` (800 RPM on aztec)
- **THEN** START command is rejected
- **AND** error message is logged: "Engine already running"
- **AND** ignition remains in current state

#### Scenario: Allow cranking when engine not running
- **WHEN** START ignition state is requested
- **AND** the effective engine RPM is below `ENGINE_RUNNING_RPM_THRESHOLD` (800 RPM), or no RPM
  source is valid
- **AND** brake is applied (>= 20%)
- **THEN** ignition state changes to CRANKING (same relay config as IGNITION)
- **AND** cranking timer starts (5-second timeout)
- **AND** cranking monitors the effective RPM for engine start detection

#### Scenario: Allow ignition transitions between ACC and IGNITION freely
- **WHEN** ignition state change is requested between ACC and IGNITION
- **THEN** change is allowed without brake or RPM checks
- **AND** only initial power-on from OFF requires safety checks

#### Scenario: Stop the starter when the engine catches
- **WHEN** the starter is engaged (CRANKING) and the effective engine RPM reaches
  `ENGINE_RUNNING_RPM_THRESHOLD` (800 RPM)
- **THEN** the relay controller SHALL release the starter and settle into IGNITION
- **AND** the threshold SHALL sit above the starter's cranking speed (≈250–400 RPM) and below the
  engine's warm idle (≈1500 RPM), so that a started engine is recognised before it reaches idle and
  a cranking-only engine is never mistaken for a running one

#### Scenario: Suppress an armed crank on a running engine
- **WHEN** a crank is armed and waiting for the pre-crank dwell
- **AND** the effective engine RPM is at or above `ENGINE_RUNNING_RPM_THRESHOLD`
- **THEN** the crank SHALL be suppressed and the ignition SHALL settle into IGNITION

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
The system SHALL persist the last confirmed transmission state (gear, servo position as float percent, validity flag) to NVS so that, when the physical gear sensor cannot tell the gear at initialisation, the servo can be restored to its last known position after a clean power cycle. The saved state SHALL be a fallback behind the sensor, never a reason to override it.

#### Scenario: Mark state invalid at start of gear change
- **WHEN** `TransmissionController::setGear()` starts a gear change
- **THEN** `state_valid=false` SHALL be written to NVS before the servo moves
- **AND** a mid-move power loss will therefore result in `state_valid=false` on next boot

#### Scenario: Save confirmed state on gear arrival
- **WHEN** a gear change RETURN phase completes
- **THEN** `state_valid=true`, `state_gear`, and `state_pct` (float percent) SHALL be written to NVS
- **AND** the saved state reflects the servo position at that moment

#### Scenario: Restore servo position and skip autohome on valid state
- **WHEN** the transmission is initialised, the physical gear reads `GEAR_UNKNOWN`
- **AND** NVS `state_valid=true`, `state_gear` is R, N, H or L and `state_pct` is within 0.0–100.0
- **THEN** the servo SHALL be commanded to `state_pct` directly (no overshoot)
- **AND** the expected gear SHALL become `state_gear`
- **AND** the system SHALL log `[TRANS] Initialised from NVS: <gear> at <pct>%`
- **AND** when the physical gear reads exactly one gear, NVS SHALL NOT be used (the sensor wins)

#### Scenario: Fall back to neutral default on invalid or missing state
- **WHEN** the transmission is initialised, the physical gear reads `GEAR_UNKNOWN`
- **AND** NVS `state_valid=false`, no state is saved, or the saved gear / position is implausible
- **THEN** the servo SHALL NOT jump directly to the neutral default position
- **AND** the transmission SHALL instead perform the initialisation step into NEUTRAL (see
  Transmission Gear Sync With the Physical Sensor), using the invalid record only as an
  overshoot-direction hint
- **AND** the initialisation step SHALL mark the state invalid without overwriting the stored
  `state_pct`, so a power loss during it leaves the same hint

#### Scenario: Resync and sensor initialisation save a valid state
- **WHEN** the expected gear is set from the sensor (initialisation or resync)
- **THEN** `state_valid=true`, `state_gear` = the sensor gear and `state_pct` = the current servo
  position SHALL be written to NVS

### Requirement: Throttle Boost During Gear Changes
The system SHALL use a PID controller to regulate the EFFECTIVE engine RPM (tacho when fitted, CAN
otherwise) to a configurable target value (`TRANS_GEAR_BOOST_TARGET_RPM`, overridden by NVS namespace `"boost"` key `target_rpm`) for the duration of a gear change, overriding SBUS/web throttle commands while active. Whether a gear change runs the PID or only holds the throttle at idle SHALL be governed by a persisted per-vehicle switch, NVS namespace `"boost"` key `enabled` (bool), default true, loaded at start-up and written immediately when changed. The manual boost test (`test_boost`) SHALL NOT be governed by the switch.

#### Scenario: Activate PID boost when gear change starts
**Given** a gear change is initiated (transmission actuator starts moving toward a non-NEUTRAL gear)
**And** the gear-change boost switch is enabled
**When** `transmission_.needsThrottleBoost()` returns true
**Then** the gear boost PID SHALL be activated
**And** CAN RPM polling SHALL switch to fast mode (`CAN_POLL_INTERVAL_RPM_BOOST`)
**And** PID integral and derivative state SHALL be reset to zero

#### Scenario: PID holds target RPM during gear change
**Given** the gear boost PID is active
**And** the effective engine RPM is valid
**When** `updateGearBoostPID()` is called each loop iteration
**Then** the PID error SHALL be `TRANS_GEAR_BOOST_TARGET_RPM - currentRPM`, where `currentRPM` is the
effective engine RPM
**And** a new PID step SHALL be computed only when the effective RPM carries a new sample timestamp
(a new CAN reply, or a new accepted tacho edge)
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
**And** no effective RPM source is valid (CAN invalid or stale and no live tacho)
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
**And** the effective engine RPM is invalid or below `ENGINE_RUNNING_RPM_THRESHOLD`
**When** `updateGearBoostPID()` is called
**Then** the PID SHALL NOT activate (engine not running — boosting throttle is unsafe)
**And** the throttle SHALL remain at idle

#### Scenario: Default switch state on a unit without a stored value
- **WHEN** the system starts and `boost/enabled` has never been written
- **THEN** the gear-change boost SHALL be enabled, exactly as before this switch existed

#### Scenario: Stored switch state is restored
- **WHEN** the system starts and `boost/enabled` holds false
- **THEN** the gear-change boost SHALL be disabled until it is switched on again

#### Scenario: Idle hold instead of boost on a gear change while disabled
- **WHEN** the gear-change boost switch is disabled
- **AND** `transmission_.needsThrottleBoost()` returns true
- **THEN** the gear boost PID SHALL NOT be activated and the CAN RPM poll rate SHALL NOT change
- **AND** the throttle SHALL be held at idle for as long as `needsThrottleBoost()` stays true,
  whether or not CAN data is valid and whatever the engine RPM
- **AND** MAVLink throttle SHALL NOT be applied, the web throttle demand SHALL NOT re-open the
  throttle, and a web `throttle` command SHALL be rejected with "Gear change active", exactly as
  while the PID is active (one shared shift lock: PID active or idle hold)
- **AND** the gear-change sequence, its interlocks and the overshoot dwell SHALL be unchanged

#### Scenario: Idle hold released when the gear change completes
- **WHEN** the idle hold is active and `transmission_.needsThrottleBoost()` returns false
- **THEN** the hold SHALL end with the throttle set to idle and the stored web throttle demand zeroed
- **AND** MAVLink throttle SHALL be applied again from the next update cycle

#### Scenario: Disabling during an active gear-change boost
- **WHEN** the switch is turned off while the gear boost PID is active for a gear change
- **THEN** the PID SHALL be released as on gear-change completion: CAN RPM polling SHALL revert to
  `CAN_POLL_INTERVAL_RPM`, the throttle SHALL be set to idle, `gearBoostActive_` SHALL become false
  and the stored web throttle demand SHALL be zeroed
- **AND** if `needsThrottleBoost()` is still true, the idle hold SHALL continue until it goes false

#### Scenario: Enabling during a gear change
- **WHEN** the switch is turned on while the idle hold is active
- **THEN** the idle hold SHALL continue until `needsThrottleBoost()` goes false
- **AND** the gear boost PID SHALL NOT be started mid-dwell; it SHALL run from the next gear change

#### Scenario: Manual boost test ignores the switch
- **WHEN** the switch is disabled and the manual boost test (`test_boost` true) is started
- **THEN** the gear boost PID SHALL run toward the target RPM exactly as when the switch is enabled

### Requirement: Degraded Operation on Board I/O Fault
The system SHALL define safe, observable degraded behavior when the opto-input expander or the
relay expander becomes unavailable, since the gear interlock and the ignition/starter path depend on
I2C. The service brake SHALL NOT depend on the opto-input expander.

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
- **THEN** nothing about the service brake SHALL change: there is no brake limit sensor any more and
  the iBooster uses no opto input
- **AND** the system SHALL NOT report any brake state on the basis of the input expander

#### Scenario: Relay expander unavailable
- **WHEN** the relay expander failed to initialize or is faulted
- **THEN** ignition, starter, front light and wheel lock commands SHALL become logging no-ops
- **AND** the reported ignition state SHALL be `OFF`
- **AND** steering, throttle, MAVLink and web telemetry SHALL continue to operate
- **AND** the service brake SHALL keep sending its frames for as long as the unit is still heard on
  the bus, so a relay that physically stays on does not make the firmware drop an active brake
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
- **WHEN** the vehicle layer updates the meter and the EFFECTIVE engine RPM is valid (a live tacho
  when fitted, or valid CAN `VehicleData`) AND it is at or above `ENGINE_HOURS_MIN_RPM`
- **THEN** the elapsed milliseconds since the previous update SHALL be added to the meter
- **AND** the threshold SHALL be low enough to count a NORMAL IDLE — an hour meter that only counts
  above a driving RPM would under-report the wear that idling causes — and SHALL be a constant
  SEPARATE from `ENGINE_RUNNING_RPM_THRESHOLD`, which is a gear-change safety threshold chosen for
  a different purpose and may sit above idle
- **AND** whole seconds SHALL be carried into the counter while the sub-second remainder is
  retained, so that a sequence of short updates accumulates exactly as one long one would

#### Scenario: Do not count while the engine is stopped or the CAN data is invalid
- **WHEN** no effective RPM source is valid (CAN `VehicleData` invalid or stale and no fitted
  tacho), or the effective engine RPM is below `ENGINE_HOURS_MIN_RPM` (a fitted tacho that has
  timed out reads a valid 0 RPM)
- **THEN** NO time SHALL be added to the meter for that interval
- **AND** the elapsed time SHALL be discarded rather than banked, so that resuming valid RPM data
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

The front-wheel lock SHALL be fitted (`RELAY_WHEEL_LOCK_FITTED` = 1). It SHALL be driven by the web
`set_wheel_lock` command and the GCS lock command only, never by an RC / MAVLink servo channel (see
"Lock Not Driven by an RC Channel").

#### Scenario: Wheel lock from the web or the GCS
- **WHEN** a web `set_wheel_lock` true command, or a GCS lock ON command accepted under MAVLink
  control, arrives
- **THEN** Relay_5 and Relay_6 SHALL both be energized, `getWheelLock()` SHALL return `true`,
  telemetry `wheel_lock_on` SHALL be `true` and `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL be set
- **AND WHEN** a web `set_wheel_lock` false command or an accepted GCS lock OFF command arrives
- **THEN** both relays SHALL be released and the lock SHALL report unlocked

#### Scenario: Wheel lock from MAVLink channel 7
- **WHEN** MAVLink servo channel 7 is above 1520 µs (e.g. 2000), or returns to 1000 µs
- **THEN** Relay_5 and Relay_6 SHALL keep their commanded state, `getWheelLock()`,
  `wheel_lock_on` and `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL NOT change (channel 7 no longer drives
  the lock)

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

### Requirement: GCS Vehicle Command Interlocks
The vehicle layer SHALL execute the GCS vehicle commands (gear, ignition, headlight, diff lock)
through the same controllers the RC and web paths use and SHALL be the arbiter of every interlock.
"Not under MAVLink control" SHALL mean `currentInputSource_ != InputSource::MAVLINK` OR
`!isSignalValid()`; it therefore covers the fail-safe source, the web portal as the source after
an RC stream loss with a browser connected, and the web portal with web control latched (owner
decision D-B). Reason 3 SHALL be given for exactly this condition, and fail-safe behaviour itself
SHALL NOT change.

- **Gear**, checked in this order:
  - not under MAVLink control → `TEMPORARILY_REJECTED` reason 3;
  - engine not running (`isEngineRunning()`), or the transmission not yet initialised → reason 1,
    and the command SHALL NOT be remembered;
  - the target gear already equals the requested gear → `ACCEPTED`, nothing moves;
  - the transmission's speed interlock (`canChangeGear()`) refuses → reason 2; this interlock
    SHALL be used unchanged (N is never speed-refused; the CAN fallback applies while the hall
    sensor has not pulsed);
  - otherwise `setGear()` → `ACCEPTED`.
  - H SHALL be accepted over MAVLink when capability bit1 is set; the handbrake state SHALL NOT
    block a gear command.
- **Ignition**:
  - OFF SHALL always be `ACCEPTED` (engine kill), including in fail-safe, under web control,
    during the pre-crank dwell and while cranking; it SHALL set the relay state OFF and persist the
    distance and engine-hour counters on a transition into OFF;
  - ACC or ON not under MAVLink control → reason 3;
  - ACC → relay state ACC, cancelling a pending or active crank;
  - ON with the engine running (`isEngineRunning()`, which reads the engine RPM cleared at
    ignition OFF — see "Validated Engine RPM for Crank Control") → `ACCEPTED` without a crank;
  - ON otherwise → `requestCrank()` exactly like a fresh RC transition into IGNITION (the
    configured pre-crank dwell — see "Configurable Pre-Crank Dwell", default 3000 ms — starter at
    most `CRANKING_TIMEOUT`).
- **Headlight**: always accepted, in every input source.
- **Diff lock**: not under MAVLink control → reason 3; otherwise the same `setWheelLock()` path the
  web `set_wheel_lock` command uses (quad bike: Relay_5 + Relay_6; buggy: Relay_4 + Relay_5 +
  Relay_6), reported in `EFI_STATUS.pt_compensation` bit0. No RC channel drives the lock (see "Lock
  Not Driven by an RC Channel"), so an accepted diff-lock command holds until the next web or GCS
  lock command.
- With the relay board unavailable, an ignition, headlight or diff-lock command SHALL be answered
  `MAV_RESULT_FAILED` and SHALL change nothing.

#### Scenario: Gear with the engine stopped
- **WHEN** a GCS gear L command arrives under MAVLink control with the engine not running
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 1
- **AND** the gear SHALL NOT be applied when the engine starts later

#### Scenario: Gear while moving
- **WHEN** a GCS gear R command arrives with the engine running at 2.0 m/s
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 2 and the gear SHALL not change

#### Scenario: Gear already selected
- **WHEN** a GCS gear N command arrives with the engine running and the target gear already N
- **THEN** the ACK SHALL be `ACCEPTED` and the transmission SHALL not move

#### Scenario: Gear in fail-safe
- **WHEN** a GCS gear command arrives while the input source is `FAILSAFE`
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 3

#### Scenario: Web portal in control
- **WHEN** the input source is the web portal
- **THEN** gear, ignition ACC / ON and diff lock commands SHALL be answered `TEMPORARILY_REJECTED`
  with reason 3
- **AND** headlight and ignition OFF commands SHALL be accepted and executed

#### Scenario: Engine kill always works
- **WHEN** a GCS ignition OFF command arrives while cranking, in fail-safe or under web control
- **THEN** the ACK SHALL be `ACCEPTED` and the ignition and starter relays SHALL be switched off

#### Scenario: Ignition ON with the engine running
- **WHEN** a GCS ignition ON command arrives with the engine running
- **THEN** the ACK SHALL be `ACCEPTED` and the starter SHALL NOT engage

#### Scenario: Auto-crank from the GCS
- **WHEN** a GCS ignition ON command arrives under MAVLink control with the ignition OFF and the
  engine stopped
- **THEN** the ACK SHALL be `ACCEPTED`
- **AND** the ignition line SHALL come up, the starter SHALL engage after the configured pre-crank
  dwell and SHALL stop on engine start or after `CRANKING_TIMEOUT`

#### Scenario: Ignition ON right after an engine kill
- **WHEN** the engine was running at or above `ENGINE_RUNNING_RPM_THRESHOLD`, a GCS ignition OFF
  stopped it, and a GCS ignition ON arrives less than `CAN_DATA_STALE_TIMEOUT` later
- **THEN** the command SHALL NOT be treated as "ON with the engine running"
- **AND** the auto-crank SHALL start as in "Auto-crank from the GCS"

#### Scenario: Diff lock holds against channel 7
- **WHEN** a GCS diff-lock ON command is accepted under MAVLink control and servo channel 7 then
  moves from 1900 µs to 1100 µs
- **THEN** the lock SHALL stay engaged

#### Scenario: Relay board unavailable
- **WHEN** a GCS headlight command arrives while the relay expander is faulted
- **THEN** the ACK SHALL be `MAV_RESULT_FAILED`

### Requirement: Last-Change-Wins Arbitration of RC Channels and GCS Commands
RC channels (CH3 gear, CH4 ignition, CH6 headlight) and GCS commands SHALL be equal sources, and
the most recent change SHALL win. The every-loop re-apply of these three functions from the RC
channels SHALL be removed; steering, throttle and brake SHALL still follow the RC channels every
loop. The diff lock is not an RC function: CH7 SHALL NOT be read and the lock SHALL be set only by
the web and GCS commands (see "Lock Not Driven by an RC Channel").

- Per function the vehicle layer SHALL keep the last in-band decoded RC value. A value outside
  880–2160 µs (including 0) SHALL be treated as absent.
- An RC value SHALL be applied only when it differs from the stored value, or when no value is
  stored yet. An absent channel, a missing frame or a fail-safe SHALL NOT be a change and SHALL keep
  the stored value and the current vehicle state, which a GCS command may have set.
- Every time RC comes back in control — at boot, on every exit from the fail-safe source (owner
  decision D-A) and on every return from the web portal to MAVLink control, whether web control was
  released or the RC stream returned while a browser was connected (owner decision D-H) — the stored
  values and the pending RC gear SHALL have been cleared, so the first valid RC frame sets gear,
  ignition and headlight again; the lock SHALL NOT be touched. They SHALL be cleared in every loop in
  which the input source is not MAVLink. RC ignition ON SHALL then call `requestCrank()` (suppressed
  when the engine runs). A GCS or web command given before RC came back in control SHALL NOT be
  restored.
- An RC gear change requested while the engine is not running, or refused by the speed interlock,
  SHALL stay pending and SHALL be applied once the engine runs and the transmission accepts it
  (today's retry, owner decision D-E). A GCS gear command that is accepted later SHALL cancel the
  pending RC gear; a rejected GCS gear command SHALL NOT. A later RC gear change SHALL replace it.
- RC SHALL still not select H.

#### Scenario: Command holds against a constant RC channel
- **WHEN** RC CH6 stays at 1100 µs (headlight OFF) and a GCS headlight ON command is accepted
- **THEN** the headlight SHALL stay on in every following loop while CH6 does not change

#### Scenario: RC edge wins over an earlier command
- **WHEN** a GCS headlight ON was accepted and CH6 then moves from 1100 µs to 1900 µs and back to
  1100 µs
- **THEN** the headlight SHALL follow the RC channel: on, then off

#### Scenario: CH7 is not an arbitrated channel
- **WHEN** a GCS diff-lock ON was accepted and CH7 then moves from 1100 µs to 1900 µs and back to
  1100 µs
- **THEN** the lock SHALL stay on

#### Scenario: Channel loss is not a change
- **WHEN** a GCS ignition ON command was accepted and CH4 reads 0 µs in the following frames
- **THEN** the ignition SHALL stay ON
- **AND** when CH4 returns to the same in-band OFF value it had before, the ignition SHALL stay ON

#### Scenario: Fail-safe exit re-sync
- **WHEN** the RC stream is lost with no browser connected (fail-safe, all relays off except the
  lock) and returns with CH4 in the ON band
- **THEN** the first valid frame SHALL call `requestCrank()` and set gear and headlight from their
  channels, exactly as at boot
- **AND** the lock SHALL keep the state it had before the loss

#### Scenario: Return from web control re-sync
- **WHEN** the web portal was the input source and switched the headlight on and the ignition off,
  and MAVLink control returns (web control released, or the RC stream returns while a browser is
  connected) with CH6 in the OFF band and CH4 in the ON band
- **THEN** the first valid frame SHALL switch the headlight off and SHALL call `requestCrank()`
- **AND** gear SHALL be set from CH3 in the same frame
- **AND** the lock SHALL keep the state the web portal left it in, whatever CH7 reads

#### Scenario: RC-only vehicle unchanged
- **WHEN** the ground station never sends a vehicle command
- **THEN** gear, ignition (including the auto-crank on a fresh ON) and headlight SHALL follow the RC
  channels as before this change, for every in-band value
- **AND** the lock SHALL change only on a web command

#### Scenario: Pending RC gear superseded by the GCS
- **WHEN** CH3 moves to L with the engine stopped, the engine starts, and before the transmission
  accepts L a GCS gear N command is accepted
- **THEN** the pending RC gear L SHALL be cancelled and the target SHALL be N

### Requirement: GCS Handbrake Command
On builds with the parking brake (`aztec`, X6 driver initialised at boot, capability bit5 set), a
GCS handbrake command (`param1` 5) SHALL call `commandParkBrake(APPLY` for `param2` 1 or `RELEASE`
for `param2` 0`, ParkBrakeSource::GCS)` and SHALL add no interlock of its own: it SHALL be accepted
in every input source, ignition state and fail-safe (never reason 3). The result SHALL be mapped:
`STARTED`, `STARTED_SPEED_UNKNOWN`, `REVERSING`, `ALREADY_RUNNING` → `MAV_RESULT_ACCEPTED`;
`REFUSED_MOVING` → `MAV_RESULT_TEMPORARILY_REJECTED` reason 4; `REFUSED_DRIVER`, `REFUSED_NVS` →
`MAV_RESULT_FAILED` (owner decision D-C). The existing parking-brake behaviour — apply speed limit,
drive-away release, throttle hold, `EFI_STATUS.pt_compensation` bits 2 and 3 — SHALL apply to a
GCS command exactly as to a web command and SHALL NOT change. The command SHALL be logged by the
existing `[PBRAKE] … accepted (gcs)` line.

On builds without the parking brake (`master`, where X6 is the steering VESC) and on the buggy when
the X6 driver did not initialise at boot, capability bit5 SHALL be clear and a handbrake command
SHALL be answered `MAV_RESULT_UNSUPPORTED`.

#### Scenario: Apply at standstill
- **WHEN** the buggy is stopped and a GCS handbrake ON command arrives in fail-safe
- **THEN** the apply run SHALL start, the ACK SHALL be `ACCEPTED` and the log SHALL read
  `[PBRAKE] Apply accepted (gcs)`

#### Scenario: Apply while moving
- **WHEN** a GCS handbrake ON command arrives at 5 km/h
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 4 and the actuator SHALL not move

#### Scenario: Save failure
- **WHEN** the moving state cannot be written to NVS for an apply
- **THEN** the ACK SHALL be `MAV_RESULT_FAILED`

#### Scenario: Quad bike
- **WHEN** the quad bike receives a handbrake command
- **THEN** the ACK SHALL be `MAV_RESULT_UNSUPPORTED` and nothing on X6 SHALL change

### Requirement: Main Loop Timing Instrumentation
The main loop SHALL measure the duration of every iteration and of each of its phases (board inputs,
speed sensor, MAVLink, web portal, input source and web command, vehicle update, steering
diagnostics, MAVLink report, telemetry, journal) with `micros()` stamps, keep the maximum iteration
time and the maximum journal-flush time per snapshot window, and report overruns to the event
journal. The instrumentation SHALL NOT change the order or behavior of the phases.

#### Scenario: Overrun
- **WHEN** an iteration takes longer than `JOURNAL_LOOP_OVERRUN_MS` (50 ms)
- **AND** the slowest phase was not the journal flush
- **THEN** `SYS W loop_overrun ms=<total> phase=<slowest> phase_ms=<its time>` SHALL be journaled

#### Scenario: Snapshot window
- **WHEN** the periodic snapshot is written
- **THEN** it SHALL report `loop_max` and `jflush_max` for the elapsed window and both SHALL restart
  from zero

#### Scenario: Low heap
- **WHEN** free heap falls below `JOURNAL_HEAP_WARN_BYTES` (40 KB)
- **THEN** `SYS W heap_low free=<bytes> min=<bytes>` SHALL be journaled once
- **AND** `SYS I heap_ok` SHALL be journaled when free heap rises 8 KB above the threshold

### Requirement: 12 V Battery Sense on X10
On aztec (no Buck-Boost board fitted) the system SHALL measure the 12 V lead-acid battery wired to
X10 pin 3 through the on-board 200K/24K divider to GPIO9 (`PIN_ADC_24V`, ADC1), apply a user
calibration factor, and use the result as the vehicle's 12 V / "ECU voltage" measurement. The math
SHALL live in a pure header (`include/BatteryVoltage.h`) covered by native unit tests.

#### Scenario: GPIO46 unchanged
- **WHEN** `setup()` begins
- **THEN** `PIN_BOOST_EN` (GPIO46) SHALL still be configured as an output and driven HIGH as the
  first hardware action, exactly as before (harmless with nothing on X10 pin 1)

#### Scenario: Sample the battery
- **WHEN** `BAT12_SAMPLE_INTERVAL_MS` (500 ms) elapses
- **THEN** GPIO9 SHALL be sampled with 11 dB attenuation and averaged over 8 readings
- **AND** the raw voltage SHALL be `Vadc × BAT12_DIVIDER_RATIO` (9.33) and the reported voltage
  `raw × cal`
- **AND** sampling SHALL NOT block the control loop

#### Scenario: Nothing connected
- **WHEN** the raw voltage is at or below `BAT12_PRESENT_MIN_V` (5 V)
- **THEN** the battery sense SHALL be reported invalid and no low warning SHALL be raised

#### Scenario: Low battery with hysteresis
- **WHEN** the calibrated voltage falls below `BAT12_LOW_V` (11.8 V)
- **THEN** the low flag SHALL be set and a rate-limited warning logged
- **AND** it SHALL clear only once the voltage rises above `BAT12_LOW_CLEAR_V` (12.1 V)
- **AND** no subsystem SHALL be disabled automatically (warning only)

#### Scenario: Cranking sag suppressed
- **WHEN** the starter relay is energized (crank or relay test), and for `BAT12_CRANK_HOLDOFF_MS`
  (2 s) after it releases
- **THEN** the low flag SHALL be suppressed while the voltage is still measured and reported

### Requirement: Aztec Parking Brake Actuator
The aztec buggy SHALL drive its parking-brake actuator through a second `BTS7960Controller` on
connector X6: `PIN_PARK_BRAKE_LPWM` = GPIO4 (X6 pin 2) and `PIN_PARK_BRAKE_RPWM` = GPIO5 (X6 pin 3).
The actuator is self-locking and has built-in limit switches. The firmware has no position
feedback and SHALL end each movement by time.

A movement ("run") SHALL drive `PARK_BRAKE_DRIVE_DUTY` (255) in the commanded direction for the
configured travel time plus `PARK_BRAKE_TRAVEL_MARGIN_MS` (500 ms). It SHALL then stop the bridge
(both PWM 0) with no hold drive.

The park-brake state SHALL be one of `UNKNOWN`, `RELEASED`, `APPLIED`, `APPLYING` and `RELEASING`.
Commands SHALL enter through one source-agnostic function,
`VehicleController::commandParkBrake(request, source)`, which returns a result code. The web is
its only caller in this change, and a later GCS path SHALL be able to call it unchanged.

The park brake SHALL be applied only by an explicit operator command. It SHALL NOT be applied
automatically on RC or link loss, on fail-safe, on ignition off, at boot, or for any other reason.
The actuator's motor supply is on the permanent battery feed, so commands SHALL NOT be gated by
input source, ignition state or web control. They SHALL be refused
with "Park brake driver not available" only when the X6 driver failed to initialise.

The park brake SHALL be independent of the X7 service brake: separate driver, state and commands.

#### Scenario: Apply run
- **WHEN** an APPLY command is accepted while the state is `RELEASED`, `UNKNOWN` or `APPLIED`
- **THEN** the state SHALL become `APPLYING` and the bridge SHALL drive the apply direction at duty
  255
- **AND** after the travel time plus 500 ms the bridge SHALL be stopped and the state SHALL become
  `APPLIED`
- **AND** no PWM SHALL be applied to X6 while the state is `APPLIED`

#### Scenario: Release run
- **WHEN** a RELEASE command is accepted while the state is `APPLIED`, `UNKNOWN` or `RELEASED`
- **THEN** the state SHALL become `RELEASING`, the bridge SHALL drive the release direction at duty
  255 for the travel time plus 500 ms, then stop
- **AND** the state SHALL become `RELEASED`

#### Scenario: Release is always allowed
- **WHEN** a RELEASE command arrives at any vehicle speed, from any input source, with any ignition
  state, or during fail-safe
- **THEN** it SHALL be accepted, unless the X6 driver failed to initialise

#### Scenario: Repeated command during a run
- **WHEN** an APPLY arrives while `APPLYING`, or a RELEASE arrives while `RELEASING`
- **THEN** the run SHALL continue unchanged and its timer SHALL NOT be restarted
- **AND** the web reply SHALL be "Park brake already applying" or "Park brake already releasing"

#### Scenario: Opposite command during a run
- **WHEN** a RELEASE arrives while `APPLYING`, or an accepted APPLY arrives while `RELEASING`
- **THEN** the bridge SHALL be stopped and held at zero duty for `PARK_BRAKE_REVERSE_DWELL_MS`
  (200 ms)
- **AND** a full run (travel time plus 500 ms) SHALL then be driven in the new direction
- **AND** the state SHALL become the new moving state as soon as the command is accepted

#### Scenario: No automatic apply
- **WHEN** the MAVLink link is lost, the input source becomes FAILSAFE, the ignition is switched
  OFF, or the firmware boots
- **THEN** the park brake SHALL NOT be applied and the X6 bridge SHALL NOT be driven by that event

#### Scenario: Fail-safe does not interrupt a run
- **WHEN** the input source becomes FAILSAFE while a run is in progress
- **THEN** the run SHALL continue to its end and the state SHALL settle as for any run

#### Scenario: Service brake independent
- **WHEN** the park brake is commanded, running or settled
- **THEN** the iBooster service brake target, rod control, unit state and telemetry SHALL be unaffected
- **AND** a service brake command SHALL NOT change the park-brake state, except through the
  requirement Parking Brake Release by Service-Brake Press

#### Scenario: Driver unavailable
- **WHEN** the X6 `BTS7960Controller::begin()` failed at boot and any park-brake command arrives
- **THEN** it SHALL be refused with "Park brake driver not available" and the state SHALL NOT change

### Requirement: Parking Brake Apply Speed Interlock
An APPLY SHALL be refused while the vehicle moves faster than `PARK_BRAKE_APPLY_MAX_SPEED_MS`
(0.833 m/s = 3 km/h). The speed SHALL be read from the hall speed sensor when its reading is valid
or latched suspicious, the same trust rule as the gear-change speed interlock. The CAN speed field
SHALL NOT be used.

#### Scenario: Apply refused while moving
- **WHEN** an APPLY command arrives and the trusted speed is above 0.833 m/s
- **THEN** it SHALL be refused, the state SHALL NOT change and a running release SHALL continue
- **AND** the web reply SHALL be a failure "Park brake apply refused: moving at N.N km/h (max 3.0)"
- **AND** the refusal SHALL be logged under the `BRAKE` debug feature

#### Scenario: Apply accepted at standstill
- **WHEN** an APPLY command arrives and the trusted speed is at or below 0.833 m/s
- **THEN** it SHALL be accepted and the reply SHALL be "Park brake applying"

#### Scenario: Speed sensor not yet pulsed
- **WHEN** an APPLY command arrives and the speed sensor has not pulsed since boot (not valid and not
  suspicious)
- **THEN** the APPLY SHALL be accepted and the reply SHALL be "Park brake applying (speed unknown)"

### Requirement: Parking Brake Drive-Away Release and Throttle Hold
The operator throttle is the web demand or the MAVLink throttle channel (manual or autopilot)
after the existing gear clip. "Ignition on" SHALL mean that the firmware's commanded ignition-relay
state, `RelayController::getIgnitionState()`, is not `OFF` (ACC, IGNITION or CRANKING: the state
in which `RELAY_SET_IGNITION` is energized). It SHALL NOT be derived from engine RPM.

**Throttle hold.** While the transmission target gear is L, H or R, the operator throttle SHALL be
held at idle when the park brake is:
- `APPLIED`, `APPLYING` or `UNKNOWN`; or
- `RELEASING`, until the throttle release point has elapsed.

The throttle release point is `thr_pass_pct` % of `travel_ms`, counted from the start of the
release drive (after any reverse dwell). With 100 % the hold SHALL last until the run ends, margin
included. From the release point on, the operator throttle SHALL pass through the speed limiter and
the opening slew from idle while the actuator finishes its run. This SHALL apply to every RELEASING
run, whatever started it and whatever the starting state.

**Drive-away.** The firmware SHALL start a RELEASE ("drive-away") when all of these hold:
- the park brake is `APPLIED` or `UNKNOWN`;
- the ignition is on;
- the target gear is L, H or R and no gear change is in progress;
- the throttle demand rises above `PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT` (10 %), with no dwell or
  debounce.

The demand SHALL have been at or below 10 % while the brake was `APPLIED` or `UNKNOWN` and the
ignition was on, since the last release started (rising edge).

The drive-away release SHALL NOT depend on engine start or engine RPM. In N, and while the target
gear is UNKNOWN, throttle SHALL neither be held by the park brake nor release it. A release run
SHALL complete once started, even if the throttle drops. There SHALL be no automatic re-apply.

#### Scenario: Drive away in gear with the default release point
- **WHEN** the park brake is `APPLIED`, the ignition is on, the target gear is L, the travel time is
  2000 ms, the release point is 50 %, and the operator raises the throttle from 0 % to 30 %
- **THEN** a release run SHALL start at once
- **AND** the throttle servo SHALL stay at idle for the first 1000 ms of the release drive
- **AND** from 1000 ms the 30 % demand SHALL pass through the speed limiter and the opening slew,
  rising from idle, while the actuator keeps releasing
- **AND** the run SHALL end and the state SHALL become `RELEASED` at 2500 ms

#### Scenario: Release point 100 %
- **WHEN** the release point is 100 % and a release run is in progress in gear
- **THEN** the throttle SHALL stay at idle until the run ends (travel time plus 500 ms) and the
  state is `RELEASED`

#### Scenario: Operator Release uses the same release point
- **WHEN** the operator presses Release while the brake is `APPLIED` or `UNKNOWN` and the target
  gear is H
- **THEN** the throttle SHALL be held at idle until the release point and pass from then on, exactly
  as for a drive-away release

#### Scenario: Apply after the throttle was passed
- **WHEN** a release run has passed its release point (throttle passing) and an APPLY is accepted
- **THEN** the run SHALL reverse (state `APPLYING`)
- **AND** the operator throttle SHALL be put back to idle at once

#### Scenario: Throttle dropped during a release
- **WHEN** the operator drops the throttle to 0 % during a release run
- **THEN** the run SHALL complete and the state SHALL become `RELEASED`
- **AND** the park brake SHALL NOT be re-applied

#### Scenario: Throttle held at or below the threshold
- **WHEN** the park brake is `APPLIED`, the target gear is H and the demand is 8 %
- **THEN** the throttle servo SHALL stay at idle and no release SHALL start

#### Scenario: Ignition off
- **WHEN** the park brake is `APPLIED`, the commanded ignition state is `OFF`, the target gear is L
  and the demand rises from 0 % to 30 %
- **THEN** no release SHALL start, the brake SHALL stay applied and the throttle SHALL stay at idle
- **AND** telemetry `pbrake_da_wait` SHALL be `"ign_off"`
- **AND** an explicit operator Release SHALL still be accepted

#### Scenario: ACC, IGNITION and CRANKING all count as ignition on
- **WHEN** the park brake is `APPLIED`, the target gear is L, the drive-away latch is armed, and the
  commanded ignition state is `ACC`, `IGNITION` or `CRANKING`
- **AND** the demand rises from 0 % to 30 %
- **THEN** a release run SHALL start in each of the three states
- **AND** only the state `OFF` (including `OFF` reported because the relay board is unavailable)
  SHALL block the drive-away release

#### Scenario: Ignition switched on with the throttle already up
- **WHEN** the park brake is `APPLIED`, the target gear is L, the demand is held at 30 % and the
  ignition is switched from `OFF` to `ACC` or `IGNITION`
- **THEN** no release SHALL start
- **AND** `pbrake_da_wait` SHALL be `"rearm"`
- **AND** a release SHALL start only after the demand has dropped to 10 % or below and risen above
  10 % again with the ignition on

#### Scenario: Revving in neutral
- **WHEN** the park brake is `APPLIED` and the target gear is N
- **THEN** throttle SHALL NOT start a release and SHALL NOT be held by the park brake
- **AND** the existing neutral clip (`TRANS_UNKNOWN_GEAR_THROTTLE_MAX`) SHALL still apply

#### Scenario: Unknown state behaves as possibly applied
- **WHEN** the state is `UNKNOWN`, the ignition is on and the target gear is R
- **THEN** the throttle SHALL be held at idle
- **AND** a rising demand above 10 % SHALL start a release with the same release point, which
  settles the state as `RELEASED`

#### Scenario: Held throttle does not re-release after an apply
- **WHEN** the operator applies the park brake while the demand is already above 10 % in gear and
  stays there
- **THEN** no drive-away release SHALL start when the apply completes, and the throttle SHALL stay
  at idle
- **AND** `pbrake_da_wait` SHALL be `"rearm"`
- **AND** a release SHALL start only after the demand has dropped to 10 % or below and risen above
  10 % again

#### Scenario: Gear change keeps its own throttle ownership
- **WHEN** a gear change runs (boost PID or shift idle hold) while the park brake is applied
- **THEN** the gear change SHALL own the throttle exactly as without the park brake
- **AND** the park-brake hold SHALL act on the operator throttle path only

#### Scenario: Web throttle reply while held
- **WHEN** a web `set_throttle` command is accepted while the park-brake hold is active
- **THEN** the reply SHALL be a success "Throttle set — held at idle until the park brake is released"
- **AND** with the ignition off it SHALL be "Throttle set — held at idle: ignition off, the park
  brake stays applied"

### Requirement: Parking Brake State Persistence
The park-brake state SHALL be persisted in the NVS namespace `"pbrake"`, key `state` (uint8:
0 unknown, 1 released, 2 applied, 3 applying, 4 releasing). It SHALL be written as the moving value
before a run drives the bridge, and as the settled value after the bridge stops.

At boot a stored 1 or 2 SHALL restore `RELEASED` or `APPLIED`. Any other stored value, including a
missing key, SHALL give `UNKNOWN`; it SHALL be logged and not rewritten. The firmware SHALL NOT
drive the actuator at boot.

#### Scenario: State survives a reboot
- **WHEN** the park brake was `APPLIED` and the board reboots
- **THEN** the state SHALL be `APPLIED` after boot and the X6 bridge SHALL stay at zero duty

#### Scenario: Power loss mid-run
- **WHEN** power is lost, the board resets or an OTA reboot happens during a run
- **THEN** the state after boot SHALL be `UNKNOWN`, not `APPLIED` or `RELEASED`

#### Scenario: First boot
- **WHEN** the namespace `"pbrake"` has no `state` key
- **THEN** the state SHALL be `UNKNOWN` until a run completes

#### Scenario: Save failure at run start
- **WHEN** writing the moving state fails as an APPLY starts
- **THEN** the APPLY SHALL be refused with "Failed to save the park brake state — apply refused" and
  the bridge SHALL NOT be driven
- **WHEN** writing the moving state fails as a RELEASE starts
- **THEN** the release SHALL run and the reply SHALL end with "(state not saved)"

### Requirement: Configurable Parking Brake Travel Time, Throttle Release Point and Direction
The parking-brake travel time, throttle release point and drive direction SHALL be persisted in the
NVS namespace `"pbrake"`:
- `travel_ms`: uint16, 1000–5000 ms, default 2000 ms;
- `thr_pass`: uint8, the throttle release point in % of `travel_ms`, 20–100, default 50;
- `invert`: uint8 0/1, default 0.

With `invert` = 0, APPLY SHALL drive RPWM (GPIO5) and RELEASE SHALL drive LPWM (GPIO4); `invert` =
1 SHALL swap them. A missing key SHALL read as the default. A stored out-of-range travel time
or release point SHALL be logged and ignored, and not rewritten. Changes SHALL be persisted first,
then applied. A new travel time SHALL take effect from the next run. A new release point SHALL take
effect from the next release run.

#### Scenario: Set the travel time
- **WHEN** a travel time of 3200 ms is set
- **THEN** it SHALL be persisted and the next run SHALL last 3700 ms
- **AND** a run already in progress SHALL keep its duration

#### Scenario: Travel time out of range
- **WHEN** a travel time below 1000 ms or above 5000 ms is set
- **THEN** it SHALL be refused and the value in use SHALL NOT change

#### Scenario: Set the throttle release point
- **WHEN** a release point of 70 % is set with a travel time of 2000 ms
- **THEN** it SHALL be persisted, and in the next release run the throttle SHALL be held for 1400 ms
  of the release drive

#### Scenario: Throttle release point out of range
- **WHEN** a release point below 20 % or above 100 % is set
- **THEN** it SHALL be refused and the value in use SHALL NOT change

#### Scenario: Invert the direction
- **WHEN** the direction setting is changed while no run is in progress
- **THEN** `invert` SHALL be persisted
- **AND** the state SHALL be set to and persisted as `UNKNOWN`

#### Scenario: Invert refused during a run
- **WHEN** the direction setting is changed while `APPLYING` or `RELEASING`
- **THEN** it SHALL be refused with "Park brake direction change refused: actuator moving"

### Requirement: Parking Brake Release by Service-Brake Press
On the aztec buggy, a new operator service-brake press SHALL start a parking-brake RELEASE run while
the park brake is `APPLIED` or `UNKNOWN`: while the service brake is available, once it reports that
it has reached the pressed position; while it is not available, at once. For this one trigger, this requirement takes precedence over the rule that a
service brake command does not change the park-brake state.

By owner decision of 2026-11-17 this trigger has **no protection** against an automatic brake: a
brake commanded on servo channel 2 by any sender (RC handset, GCS joystick, the BlueOS Copilot's
automatic fail-safe brake, an autopilot mode) and a web brake command all count.

**Input.** The trigger SHALL use the commanded service-brake target as `applyBrake()` sets it (below
5 % it is 0, above 100 % it is 100 %), the service brake's `available` indication (see "Service Brake
Abstraction") and its `reached` indication (see "iBooster Rod Control"). A **press** is that target changing from 0 to a value above 0 while the input source is
`MAVLINK` or `WEB`.

**Rising edge.** The firmware SHALL sample the target on every control loop in every park-brake
state, so that:
- a target that is already non-zero when the park brake becomes `APPLIED` is not a press;
- a change from one non-zero value to another is not a press;
- under the `FAILSAFE` input source no press SHALL be detected, and the target frozen there SHALL NOT
  count as a press when a live source returns;
- on any input-source change the sample SHALL be re-seeded with the current target, so a takeover
  (MAVLINK ↔ WEB, or from or to `FAILSAFE`) never produces a press by itself.

**Hand-over.** After a press the release SHALL start on the first loop in which the service brake
reports `reached` or is not `available`. While the service brake is not available (ignition OFF,
unit asleep, starting, lost or in fault) the release therefore starts on the loop of the press. The
wait SHALL be cancelled, with no release, when the target returns to 0, the input source changes,
the park-brake state is no longer `APPLIED` or `UNKNOWN`, or the service brake is available but
reports that its rod does not move ("no move", see "iBooster Rod Control"). A further release then
needs a new press.

By owner decision of 2026-11-23 a press releases the parking brake also while the service brake is
not acting; the buggy can then be held by neither brake.

**Release semantics.**
- The release SHALL be the same RELEASE request as the drive-away release.
- Every existing RELEASING rule SHALL apply unchanged: the throttle hold in L/H/R up to the throttle
  release point, completion of the run if the service brake is let go, and no automatic re-apply.
- A service-brake press SHALL NOT apply the parking brake, and SHALL have no park-brake effect while
  the state is `RELEASED`, `RELEASING` or `APPLYING`.
- The trigger SHALL NOT depend on the gear, the ignition state, the flight mode or any autopilot
  status.
- The firmware SHALL NOT move or hold the service brake for this feature.

The drive-away release, the explicit web and GCS commands, the apply speed interlock and the speed
governor SHALL behave exactly as before.

#### Scenario: RC press releases after the hand-over
- **WHEN** the park brake is `APPLIED`, the source is `MAVLINK`, the ignition is on, the service
  brake is `READY` and released
- **AND** the operator presses the service brake to 50 % and holds it
- **THEN** the release run SHALL start on the first loop in which the service brake reports
  `reached`
- **AND** the state SHALL become `RELEASING`

#### Scenario: Release from UNKNOWN
- **WHEN** the park-brake state is `UNKNOWN` and a press completes its hand-over
- **THEN** a release run SHALL start and settle the state as `RELEASED`

#### Scenario: Web press releases
- **WHEN** the park brake is `APPLIED`, the source is `WEB`, the target is 0 and the operator sends a
  web brake of 80 %
- **THEN** a release run SHALL start once the service brake reports `reached`

#### Scenario: Automatic fail-safe brake releases (accepted)
- **WHEN** the park brake is `APPLIED`, the source is `MAVLINK`, and channel 2 goes from 1500 µs to
  1400 µs (25 %) because the Copilot brakes after an RC loss
- **THEN** a release run SHALL start once the service brake reports `reached`

#### Scenario: Brake held when the parking brake is applied
- **WHEN** the service brake target is 60 %, the park brake is applied from the web and becomes
  `APPLIED`, and the target stays at 60 %
- **THEN** no release SHALL start
- **AND** after the target goes to 0 and then to a non-zero value, a release SHALL start after the
  hand-over

#### Scenario: Frozen target under the firmware fail-safe
- **WHEN** the input source is `FAILSAFE` with a non-zero brake target left from before, and later
  `MAVLINK` returns with the same value
- **THEN** no release SHALL start

#### Scenario: Takeover with an inherited target
- **WHEN** web control is engaged while the MAVLink brake target is 80 %, and the web then sends 80 %
- **THEN** no release SHALL start
- **AND** a web brake of 0 followed by a non-zero web brake SHALL start a release after the hand-over

#### Scenario: Press dropped before the hand-over
- **WHEN** a press is pending and the target returns to 0 before the service brake reports `reached`
- **THEN** no release SHALL start
- **AND** the next press SHALL start a new hand-over

#### Scenario: Service brake let go mid-release
- **WHEN** a release started by the service brake is running and the operator lets go of the service
  brake
- **THEN** the run SHALL complete and the state SHALL become `RELEASED`
- **AND** the park brake SHALL NOT be re-applied

#### Scenario: No apply and no effect when not applied
- **WHEN** the park brake is `RELEASED`, `RELEASING` or `APPLYING` and the operator presses the
  service brake
- **THEN** the park-brake state and run SHALL NOT change

#### Scenario: Press releases while the service brake is unavailable (accepted)
- **WHEN** the park brake is `APPLIED`, the iBooster is `ASLEEP`, `STARTING`, `LOST` or in `FAULT`,
  and the operator presses the service brake
- **THEN** a release run SHALL start on the loop of the press
- **AND** the service brake SHALL NOT act, so the buggy is held by neither brake (owner decision)

#### Scenario: Service brake ready but its rod does not move
- **WHEN** a press is pending, the service brake is available, and it reports "no move" (accepted,
  pushing, rod not moved 0.3 mm for 3 s)
- **THEN** the wait SHALL be cancelled with no release, `BRAKE W pbrake_handover_cancel why=no_move`
  SHALL be journaled, and the parking brake SHALL stay applied until the Release command or a new
  press

#### Scenario: Unit becomes unavailable during the hand-over
- **WHEN** a press is pending, waiting for `reached`, and the service brake becomes unavailable
- **THEN** the release run SHALL start on that loop

#### Scenario: Any gear, any ignition
- **WHEN** the park brake is `APPLIED`, the target gear is N, the ignition relay is `OFF`, and a press
  occurs
- **THEN** a release run SHALL start (the service brake is asleep, so there is no hand-over wait)

#### Scenario: Throttle hold unchanged
- **WHEN** a release started by the service brake runs with the target gear L
- **THEN** the operator throttle SHALL be held at idle up to the throttle release point, exactly as
  for any release run

### Requirement: Service Brake Control
On the aztec buggy the service brake SHALL be a Bosch iBooster Gen2 commanded over the shared CAN
bus (see "Bosch iBooster Gen2 Service Brake Unit", "iBooster Unit State Machine" and "iBooster Rod
Control"). There SHALL be no other service-brake actuator: no BTS7960 brake drive, no open-loop
position estimate, no hold drive and no "released" endstop input. The brake command SHALL be a
percentage from 0 to 100 set through `applyBrake()`, which SHALL keep its clamp: below 5 % becomes
0, above 100 % becomes 100 %. The applied brake SHALL be the rod position reported by the unit.

#### Scenario: Set brake by percentage
- **WHEN** `applyBrake()` is called with a value from 5 to 100
- **THEN** the service brake target SHALL become that percentage of the configured maximum stroke
- **AND** 0 % SHALL mean released and 100 % the configured maximum stroke

#### Scenario: Dead zone kept
- **WHEN** `applyBrake()` is called with 3 %
- **THEN** the target SHALL be 0 and no brake request SHALL be sent

#### Scenario: Release
- **WHEN** the target goes from a non-zero value to 0
- **THEN** the rod SHALL be returned under control and the brake request SHALL then be cleared, as
  "iBooster Rod Control" describes

#### Scenario: Applied brake is measured
- **WHEN** the unit reports its rod position
- **THEN** the applied brake percentage SHALL be that position as a percentage of the configured
  maximum stroke, 0–100
- **AND** while the unit is not available the applied percentage SHALL be reported as 0 together with
  the brake state

#### Scenario: No brake action at power-on
- **WHEN** the system powers on or resets
- **THEN** no brake request SHALL be sent until a brake target above 0 is commanded and the unit is
  ready
- **AND** no parking or start-up brake SHALL be applied by the service brake

### Requirement: Service Brake Abstraction
The firmware SHALL talk to the service brake only through one interface (`IServiceBrake`), used from
the main loop: set a percent target; read the target, the applied percentage, `reached`,
`available`, `requestActive` and the brake state. The iBooster implementation SHALL sit behind it.
`VehicleController` SHALL NOT hold a motor driver for the service brake.

#### Scenario: Commanded and applied are separate
- **WHEN** a target of 50 % is set and the rod has not moved yet
- **THEN** the target SHALL read 50 % and the applied percentage SHALL read the measured position

#### Scenario: Throttle paths keep using the command
- **WHEN** the speed governor, the speed limiter or the throttle opening slew asks whether a brake is
  commanded
- **THEN** the answer SHALL be "commanded target above 0", whatever the unit's state
- **AND** a brake commanded while the unit is unavailable SHALL still close the throttle as before

#### Scenario: Availability
- **WHEN** the brake state is `READY`, `BRAKING` or `RELEASING`
- **THEN** `available` SHALL be true, and false in every other state
- **AND** `requestActive` SHALL be true exactly in `BRAKING` and `RELEASING`

### Requirement: iBooster Unit State Machine
The service brake SHALL track the unit in the states `ASLEEP`, `STARTING`, `READY`, `BRAKING`,
`RELEASING`, `LOST` and `FAULT`, stepped once per 10 ms slot in the CAN task by a pure law
(`include/IboosterLaw.h`), using the newest valid `0x39D`, `0x38E` and `0x38F` frames. No brake
request SHALL be sent in `ASLEEP`, `STARTING`, `LOST` or `FAULT`.

#### Scenario: Asleep with the ignition off
- **WHEN** the ignition state is `OFF` and no valid unit frame has arrived for `IB_SILENT_MS`
  (500 ms)
- **THEN** the state SHALL be `ASLEEP` and no frame SHALL be sent to the unit

#### Scenario: Start-up
- **WHEN** the ignition leaves `OFF`, or a valid unit frame arrives while `ASLEEP`
- **THEN** the state SHALL be `STARTING` and idle frames SHALL be sent every 10 ms
- **AND** the state SHALL become `READY` when a `0x39D` no older than `IB_STATUS_STALE_MS` (250 ms)
  shows status 5 or 6 with driver_brake_apply other than 3, and `0x38F` byte 4 is `0x40`
- **AND** if `READY` is not reached within `IB_START_TIMEOUT_MS` (20 s) the event SHALL be journaled
  once and the state SHALL stay `STARTING`

#### Scenario: The unit's own actuation cycle is not a fault
- **WHEN** the unit reports status 6 with internal state 5 (its periodic self-cycle, or `to_idle`
  after a release) and no other fault sign
- **THEN** the state SHALL stay `READY`

#### Scenario: Unit lost
- **WHEN** frames are being sent, the ignition is reported on, the state is `READY`, `BRAKING` or
  `RELEASING`, and the newest valid `0x39D` is older than 250 ms
- **THEN** the state SHALL become `LOST`, the request SHALL be cleared and idle frames SHALL continue
- **AND** when valid frames return the state SHALL go through `STARTING` to `READY` again

#### Scenario: Unit going to sleep with the ignition reported OFF
- **WHEN** the ignition is reported `OFF` and the unit, heard until now, falls silent
- **THEN** the state SHALL NOT become `LOST`; frames SHALL continue until 500 ms of silence and the
  state SHALL then become `ASLEEP`
- **AND** while the newest `0x39D` is older than 250 ms no new request SHALL start

#### Scenario: Unit no longer ready
- **WHEN** a fresh `0x39D` shows no fault sign but the `READY` condition is gone (status other than 5
  or 6, or `0x38F` byte 4 not `0x40`) while `READY`, or a status other than 5 or 6 while `BRAKING` or
  `RELEASING`
- **THEN** the state SHALL become `STARTING` and the request SHALL be cleared

#### Scenario: Unit fault
- **WHEN** a fresh `0x39D` shows status 2 (failure) or driver_brake_apply 3 (fault)
- **THEN** the state SHALL become `FAULT`, the request SHALL be cleared at once and idle frames SHALL
  continue
- **AND** the state SHALL leave `FAULT` only when the unit itself reports the `READY` condition again
- **AND** a `FAULT` lasting `IB_FAULT_PERSIST_MS` (20 s) SHALL be journaled once as latched

#### Scenario: Cranking dip
- **WHEN** a `LOST` or `FAULT` starts while the ignition is `CRANKING` or within `IB_CRANK_GRACE_MS`
  (3 s) after it
- **THEN** the state machine SHALL behave as at any other time
- **AND** the event SHALL be journaled at level `W` with `crank=1`, not at level `E`

#### Scenario: Brake commanded while not available
- **WHEN** the target is above 0 and the state is `STARTING`, `LOST` or `FAULT`
- **THEN** no request SHALL be sent and the target SHALL be kept
- **AND** when the state becomes `READY` with the target still above 0, the state SHALL become
  `BRAKING`

#### Scenario: Ignition switched off and on again
- **WHEN** the ignition goes from `OFF` back to on while the state is `READY`, `BRAKING`,
  `RELEASING` or `LOST` (the unit lost its Wake meanwhile, however briefly)
- **THEN** the state SHALL become `STARTING` (silent, no `LOST`), any request SHALL be cleared
- **AND** `READY` SHALL then need `0x39D` and `0x38F` frames that arrived after the ignition edge

#### Scenario: Request not accepted
- **WHEN** the request has been set for `IB_ACCEPT_TIMEOUT_MS` (1000 ms) and `0x39D` internal state
  is not 3 (external request)
- **THEN** a unit drop SHALL be counted and journaled, and the request SHALL be kept

### Requirement: iBooster Rod Control
In `BRAKING` and `RELEASING` the CAN task SHALL close a proportional loop on the `0x39D` rod position
every 10 ms slot and send the result as the `0x38C` flow, with the external-request bits set.

#### Scenario: Rod target
- **WHEN** the brake target is P % and the configured maximum stroke is M mm
- **THEN** the rod target SHALL be P / 100 × M mm, never above `IB_ROD_CEIL_MM` (16.0 mm)

#### Scenario: Flow from the rod error
- **WHEN** the rod error (target − rod) is E mm
- **THEN** the flow SHALL be `IB_ROD_KP` (250 flow units per mm) × E, 0 inside the deadband
  `IB_ROD_DEADBAND_MM` (0.05 mm), limited to + apply rate and − release rate (ml/s × 128 units)
- **AND** the flow word SHALL be `0x7E00` + flow and SHALL always stay within `0x6A00…0x9200`

#### Scenario: Old rod sample
- **WHEN** a request is set and the newest valid `0x39D` is older than `IB_ROD_HOLD_MS` (100 ms) but
  not yet stale (250 ms)
- **THEN** the flow SHALL be 0 (the rod holds) until a newer `0x39D` arrives; the loop SHALL never
  drive the rod from an old position

#### Scenario: Controlled release
- **WHEN** the target becomes 0 while `BRAKING`
- **THEN** the state SHALL become `RELEASING`, the request SHALL stay set and the rod SHALL be
  returned at no more than the release rate
- **AND** when the rod is at or below `IB_RELEASE_HANDOFF_MM` (1.0 mm) the request SHALL be cleared
  and the state SHALL become `READY`
- **AND** the release time budget SHALL be 1 s + twice the nominal return time from the rod position
  at `RELEASING` entry to 1.0 mm at the release rate (≈ 1.9 mm/s per ml/s), at least
  `IB_RELEASE_TIMEOUT_MS` (3 s) and at most `IB_RELEASE_MAX_MS` (30 s; also when the entry rod is
  unknown)
- **AND** past the budget the request SHALL be cleared, and `release_slow` journaled, only once the
  rod has stopped coming down (less than 0.1 mm in the last 1 s), or at the 30 s cap; while it still
  comes down the request SHALL be kept

#### Scenario: Re-press during release
- **WHEN** a target above 0 is commanded while `RELEASING` and the newest `0x39D` is fresh
- **THEN** the state SHALL become `BRAKING` with the new target
- **AND** with a stale `0x39D` (ignition reported OFF, unit going quiet) the state SHALL stay
  `RELEASING`

#### Scenario: Re-command after a gap
- **WHEN** the unit released by itself after a frame gap and accepted the still-set request again
- **THEN** the loop SHALL drive the rod back to the target with no operator action

#### Scenario: Reached
- **WHEN** the state is `BRAKING`, the unit reports internal state 3, and either the rod is within
  `IB_REACHED_TOL_MM` (0.2 mm) of the target, or — only after the rod has advanced `IB_ARM_MM`
  (0.3 mm) from its first position in this `BRAKING` episode, because the unit first takes up
  ~2 mm of free travel with the output rod standing still — the flow is positive and the rod has
  advanced less than 0.05 mm in `IB_STALL_MS` (300 ms), or `0x38F` byte 3 is 0
- **THEN** `reached` SHALL be true
- **AND** in every other state `reached` SHALL be false
- **AND** `reached` SHALL refer to the target currently commanded: a result computed for an earlier
  target SHALL NOT be reported

#### Scenario: Rod does not move
- **WHEN** the state is `BRAKING`, the unit reports internal state 3, the flow is positive and the
  rod has not advanced 0.3 mm for `IB_NO_MOVE_MS` (3 s)
- **THEN** "no move" SHALL be reported (journal `BRAKE W no_move`, once per episode), `reached` SHALL
  stay false and the request SHALL be kept
- **AND** "no move" SHALL clear as soon as the rod moves 0.3 mm

#### Scenario: Vehicle speed to the unit
- **WHEN** the frames are built
- **THEN** `0x38B` byte 2 SHALL be round(speed km/h / 0.4), clamped to 0–255, from the wheel speed
  floored as required by "iBooster Vehicle Speed Floor"
- **AND** the "faulty" speed qualifier (`0x80`) SHALL never be sent

### Requirement: Service Brake Settings
The maximum stroke, the apply rate and the release rate SHALL be persisted, operator-set, per-vehicle
values in the NVS namespace `brake`: `max_mm10` (0.1 mm, 2.0–16.0 mm, default 16.0 — the full
external-request stroke), `apply_mlps` (1–40 ml/s, default 8) and `release_mlps` (1–40 ml/s,
default 8). The former keys `travel_ms` and
`hold_pwm` SHALL be removed once at boot and never read.

#### Scenario: Defaults
- **WHEN** the system starts without stored values
- **THEN** the maximum stroke SHALL be 16.0 mm and both rates 8 ml/s
- **AND** a 100 % brake SHALL then command a rod target of 16.0 mm

#### Scenario: Invalid stored value
- **WHEN** a stored value is outside its range
- **THEN** it SHALL be ignored and logged, the default SHALL be used and the stored value SHALL NOT be
  rewritten

#### Scenario: Change applied
- **WHEN** a valid value is set and no brake request is active
- **THEN** it SHALL be persisted first and, once the write succeeded, used from the next slot
- **AND** the maximum stroke SHALL be accepted only on the 0.5 mm grid (2.0, 2.5, … 16.0 mm); a value
  off the grid is answered as out of range
- **AND** if the write fails the value in use SHALL stay unchanged

#### Scenario: Refused while braking
- **WHEN** a setting is sent while a brake request is active
- **THEN** it SHALL be refused, and neither the stored nor the in-use value SHALL change

#### Scenario: Old keys removed
- **WHEN** the system starts and `brake`/`travel_ms` or `brake`/`hold_pwm` exist
- **THEN** they SHALL be removed before the CAN bus is started

### Requirement: Service Brake With the Ignition OFF and Under Fail-Safe
The iBooster SHALL be woken only by the ACC / ignition line (Relay_1). This change SHALL NOT alter
the fail-safe. Both consequences are accepted by the owner and SHALL be stated in the operator
documentation: with the ignition `OFF` there is no service brake, and on fail-safe the brake drops
when `allOff()` opens the ignition relay.

#### Scenario: Fail-safe with a held brake
- **WHEN** the input source becomes `FAILSAFE` while the brake target is 40 %
- **THEN** the fail-safe SHALL do exactly what it did before this change (steering centre, throttle
  idle, transmission stop, `allOff()`, odometer and hours persisted) and SHALL add no brake action
- **AND** the brake target SHALL stay frozen at 40 %
- **AND** once the unit has gone silent the state SHALL become `ASLEEP` and no frames SHALL be sent

#### Scenario: Ignition off
- **WHEN** the ignition is switched `OFF` by any path
- **THEN** the service brake SHALL be reported unavailable (`ASLEEP`) once the unit is silent
- **AND** no brake request SHALL be attempted

#### Scenario: Main loop stalls
- **WHEN** the CAN task has received no brake command from the main loop for `IB_CMD_STALE_MS`
  (500 ms)
- **THEN** the task SHALL keep the last rod target and vehicle-speed byte and keep sending frames
- **AND** the episode and its length SHALL be journaled when the main loop runs again

### Requirement: Flash Writes During a Service-Brake Request
While a brake request is active (`BRAKING` or `RELEASING`) the firmware SHALL keep flash writes away
from the 10 ms frame schedule where it can, because a flash erase stalls the CAN task.

#### Scenario: Periodic saves deferred
- **WHEN** the odometer (every 1 km) or the engine-hour meter (every 600 s) is due to save while a
  request is active
- **THEN** the save SHALL be postponed and SHALL run on the first update after the request ends,
  moving or not (the due check runs on every update)
- **AND** the postponement SHALL last no longer than the request; a request ends with the brake
  release, with the ignition OFF (the unit sleeps; the ignition-OFF save runs) or on fail-safe
  (whose save runs at once, during the request)
- **AND** the fail-safe and explicit saves SHALL be unchanged

#### Scenario: Operator writes not deferred
- **WHEN** a gear change, a park-brake run or a setting on another card writes NVS while a request
  is active
- **THEN** the write SHALL happen as before

#### Scenario: Brake settings refused
- **WHEN** a brake setting is sent while a request is active
- **THEN** it SHALL be refused without a flash write

### Requirement: iBooster Vehicle Speed Floor
On aztec, the vehicle-speed byte sent to the iBooster (`0x38B` byte 2) SHALL be round(v / 0.4),
clamped to 0–255, where v = max(true wheel speed in km/h, `IB_LAW_SPEED_FLOOR_KMH` = 2.0 km/h). An
invalid, NaN or non-positive wheel speed SHALL give the floor (5 counts), never 0. The floor SHALL be
applied in `ibSpeedByte()` (`include/IboosterLaw.h`), the single place the byte is computed, and SHALL
apply to every `0x38B` frame the firmware sends, with or without a brake request. The floored value
SHALL NOT be used anywhere else: the speed limiter, the speed governor, telemetry, MAVLink and the web
page SHALL keep the true speed. `0x38B` byte 1 (the request / speed-qualifier bits, `0x40` only with
a request), byte 3 (`0x05`), `0x38C` and `0x38D` SHALL stay unchanged.

#### Scenario: Standing still
- **WHEN** the wheel speed is valid and 0, or valid and below 2 km/h (e.g. 1 km/h)
- **THEN** `0x38B` byte 2 SHALL be 5

#### Scenario: Speed sensor invalid
- **WHEN** the wheel speed is invalid, NaN or negative, at any true vehicle speed
- **THEN** `0x38B` byte 2 SHALL be 5

#### Scenario: At and above the floor
- **WHEN** the wheel speed is valid and exactly 2 km/h, 3 km/h or 40 km/h
- **THEN** `0x38B` byte 2 SHALL be 5, 8 and 100 respectively

#### Scenario: Clamp
- **WHEN** the wheel speed is valid and 102 km/h or more, or infinite
- **THEN** `0x38B` byte 2 SHALL be 255

#### Scenario: Rest of the firmware unaffected
- **WHEN** the buggy stands still with a valid speed of 0, or the speed sensor is invalid
- **THEN** the speed limiter, the governor, telemetry, MAVLink and the web page SHALL see the true
  speed (0, or invalid), not 2 km/h

#### Scenario: Qualifier bits unchanged
- **WHEN** frames are sent without a brake request
- **THEN** `0x38B` byte 1 SHALL carry no `0x40` (speed qualifier 0) as in firmware 1.6.0, the speed
  byte still floored; whether the unit uses the speed byte from such frames is not known

### Requirement: Transmission Gear Sync With the Physical Sensor
The expected gear (`TransmissionController::targetGear_`) SHALL be initialised from, and kept
reconciled with, the physical gear sensor. Until it is initialised the expected gear SHALL be
`GEAR_UNKNOWN`, which SHALL be the only meaning of an UNKNOWN expected gear. Initialisation SHALL
use, in order: the physical gear when it reads exactly one gear, else a valid NVS state, else a
real shift step into NEUTRAL; it SHALL never leave the transmission waiting for the sensor. The
choice of source, the initialisation-step overshoot direction and the resync decision SHALL live
in the header-only
`include/TransmissionLaw.h`, covered by the native suite `test_transmission_sync`.

#### Scenario: Not initialised at power-up
- **WHEN** the system starts
- **THEN** the expected gear and the outgoing gear SHALL be `GEAR_UNKNOWN`
- **AND** the transmission servo SHALL keep no pulse until initialisation commands it

#### Scenario: Initialisation on first engine running
- **WHEN** the transmission is not initialised and the engine is detected running
- **THEN** `VehicleController::update()` SHALL run the initialisation once

#### Scenario: Initialisation step into NEUTRAL
- **WHEN** initialisation runs, the physical gear reads `GEAR_UNKNOWN` and NVS holds no valid state
- **THEN** the system SHALL log `[TRANS] Init: sensor unknown, stepping to NEUTRAL`
- **AND** it SHALL perform a normal shift step to NEUTRAL (overshoot, overshoot dwell, return)
  through the gear-change state machine, so the gear-change boost or shift idle hold applies as for
  any step
- **AND** the expected gear SHALL be NEUTRAL from the start of the step, the outgoing gear
  `GEAR_UNKNOWN`
- **AND** the overshoot move SHALL use the fixed settle time `TRANS_INIT_SETTLE_MS` (1500 ms)
  instead of a travel time computed from the unknown servo position

#### Scenario: Initialisation step overshoot direction
- **WHEN** the initialisation step starts
- **THEN** with an NVS record whose `state_pct` is within 0.0–100.0, the overshoot direction SHALL be
  the sign of (NEUTRAL position − `state_pct`), so a lever on the R side overshoots toward H and a
  lever on the H/L side overshoots toward R
- **AND** when `state_pct` equals the NEUTRAL position, the sign of (NEUTRAL position − position of
  `state_gear`) SHALL be used
- **AND** with no record, a `state_pct` outside 0.0–100.0 or not a number, or no usable sign, the
  direction SHALL be +1 (toward H)

#### Scenario: Initialisation step completes
- **WHEN** the initialisation step's RETURN phase completes
- **THEN** with the physical gear reading NEUTRAL it SHALL complete like any step (state saved valid)
- **AND** with the physical gear still `GEAR_UNKNOWN` NEUTRAL SHALL be accepted as the working
  assumption, the state saved valid, and the system SHALL log
  `[TRANS] Init: sensor silent, assuming NEUTRAL`; the idle resync SHALL correct the expected gear
  if the sensor later reports another gear

#### Scenario: Gear requests during the initialisation step
- **WHEN** `setGear()` is called while the initialisation step runs
- **THEN** it SHALL behave as during any active step: the destination is queued and shifted to
  after the step completes

#### Scenario: Initialise from the sensor
- **WHEN** initialisation is tried and the physical gear reads exactly one gear
- **THEN** the expected gear SHALL become that gear, whatever NVS holds
- **AND** the servo SHALL be commanded directly (no overshoot) to that gear's configured position
- **AND** the system SHALL log `[TRANS] Initialised from sensor: <gear>`

#### Scenario: Throttle capped while not initialised
- **WHEN** the expected gear is `GEAR_UNKNOWN`
- **THEN** throttle from every source SHALL be capped at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (8.5 %),
  as for NEUTRAL or an active gear change

#### Scenario: Gear requests refused while not initialised
- **WHEN** `setGear()` is called while the expected gear is `GEAR_UNKNOWN`
- **THEN** it SHALL return false without commanding the servo, writing NVS or changing the queue
- **AND** a MAVLink gear request SHALL be retried by the existing retry loop until it is accepted

#### Scenario: Assumed gear reported as unknown before initialisation
- **WHEN** the expected gear is `GEAR_UNKNOWN`
- **THEN** `getTargetGearString()` and `getFromGearString()` SHALL return `"?"`, never `"N"`

#### Scenario: Resync after a confirmed mismatch
- **WHEN** the transmission is initialised, no gear change is active and no sequence is queued
- **AND** the idle check (every `TRANS_GEAR_CHECK_INTERVAL`, 500 ms) sees the physical gear read the
  same single gear, different from the expected gear, on `TRANS_RESYNC_CONFIRM_CHECKS` (2)
  consecutive checks
- **THEN** the expected gear SHALL become the physical gear and the state SHALL be saved valid with
  the current servo position
- **AND** the servo SHALL NOT be moved
- **AND** the system SHALL log `[TRANS] Resynced from sensor: target=<old> -> <new>`
- **AND** the first detection SHALL still log `[TRANS] MISMATCH: target=<old>, physical=<new>`

#### Scenario: No resync on a transient or unknown reading
- **WHEN** a check reads `GEAR_UNKNOWN`, the expected gear, or a different single gear than the
  previous mismatching check
- **THEN** the consecutive-mismatch count SHALL restart and no resync SHALL happen on that check
- **AND** starting a gear change SHALL reset the count

### Requirement: Sensor-Confirmed Gear-Change Waits
Both waits of a gear change — the overshoot dwell and the pause between sequence steps — SHALL
end as soon as the physical gear sensor confirms the expected gear, and SHALL keep their fixed
durations only as timeouts, so a shift is never slower than before and is faster whenever the
sensor works. The confirmation decision SHALL live in the header-only `include/TransmissionLaw.h`,
covered by the native suite `test_transmission_sync`.

#### Scenario: Continuous confirmation
- **WHEN** a wait is in progress and the physical gear reads the expected gear
- **THEN** the confirmation window SHALL start at that first matching read
- **AND** the wait SHALL end when the gear has read the expected gear on every read for
  `TRANS_DWELL_CONFIRM_MS` (200 ms)
- **AND** any other reading (another gear or `GEAR_UNKNOWN`) SHALL restart the window

#### Scenario: Timeout keeps today's behaviour
- **WHEN** the sensor never confirms (disconnected, stuck or reading another gear)
- **THEN** the overshoot dwell SHALL end after `TRANS_OVERSHOOT_DWELL_MS` (2000 ms) and the pause
  after `TRANS_SEQUENCE_STEP_DWELL_MS` (1000 ms), exactly as before this change

#### Scenario: Boost and idle hold follow the shorter dwell
- **WHEN** the overshoot dwell ends early on confirmation
- **THEN** `needsThrottleBoost()` SHALL become false at the same moment (it stays true exactly
  while the phase is `OVERSHOOT_DWELL`), releasing the boost PID or the shift idle hold

#### Scenario: Wait outcome is logged
- **WHEN** a wait ends
- **THEN** a TRANSMISSION debug line SHALL say whether it was confirmed (with the elapsed ms) or
  timed out
- **AND** when a single step or a whole sequence completes, its total duration SHALL be logged;
  telemetry and the web UI SHALL be unchanged

### Requirement: Configurable Speed-Limiter Ceiling Rate
The rate limit on the speed-limiter throttle ceiling SHALL be a persisted, operator-set value in
percent per second. It SHALL default to `SPEED_LIMIT_SLEW_DEFAULT_PCT_S` (80 %/s) and SHALL be
accepted in the range `SPEED_LIMIT_SLEW_MIN_PCT_S`–`SPEED_LIMIT_SLEW_MAX_PCT_S` (10–200 %/s). It
SHALL be stored in NVS namespace `"speed"` under the key `thr_slew` (float, %/s), loaded at
start-up and written immediately when changed. Wherever the speed-limiter requirements name
`SPEED_LIMIT_CEILING_SLEW_PCT_S` — the ceiling rate limit in both directions and the cap on the
trim's fall — the configured rate SHALL be used.

#### Scenario: Default rate on a unit without a stored value
- **WHEN** the system starts and `thr_slew` has never been written
- **THEN** the ceiling rate limit SHALL be 80 %/s, exactly as before this setting existed

#### Scenario: Stored rate is restored
- **WHEN** the system starts and `thr_slew` holds a value within 10–200 %/s
- **THEN** that value SHALL be the ceiling rate limit

#### Scenario: Invalid stored rate is ignored
- **WHEN** the system starts and `thr_slew` holds NaN or a value outside 10–200 %/s
- **THEN** the stored value SHALL be ignored and logged, and the default SHALL be used
- **AND** the stored value SHALL NOT be repaired or rewritten

#### Scenario: Rate changed at runtime
- **WHEN** a valid new rate is set
- **THEN** it SHALL be persisted to `thr_slew` immediately, and the value in use SHALL change
  only after the write succeeded
- **AND** it SHALL take effect on the next limiter evaluation, for the ceiling movement in both
  directions and for the cap on the trim's fall

#### Scenario: A lower rate closes the throttle more gently
- **WHEN** the speed is far above the limit and the rate is 20 %/s instead of 80 %/s
- **THEN** over the same 0.5 s the ceiling SHALL fall by 10 % instead of 40 %

### Requirement: Speed Governor
The system SHALL provide an opt-in speed governor. While the governor switch is ON, an effective
speed limit `L` > 0 exists (the same limit the maximum-speed limiter enforces: the usable
`SPEED_MAX`, and on the buggy the lower of it and the fixed gear ceiling), the speed reading is
valid and no bypass condition holds, the governor SHALL be ACTIVE: the throttle input — the RC
throttle (channel 2 above centre) or the standing web throttle demand — SHALL be a speed setpoint,
and the throttle SHALL be computed by a closed loop on the measured wheel speed. The law SHALL live
in a pure header (`include/SpeedGovernorLaw.h`) without Arduino dependencies. The governor SHALL
apply regardless of the autopilot's flight mode. It SHALL NOT command the brake: its lowest output
SHALL be the calibrated idle (0 %).

#### Scenario: Stick maps to a speed up to the limit
- **WHEN** the governor is ACTIVE with limit `L` and the throttle input is `s` %
- **THEN** for `s` ≤ `GOV_STICK_DEADBAND_PCT` (5 %) the target speed SHALL be 0
- **AND** above it the target SHALL be `vMin + (s − 5)/(100 − 5) × (L − vMin)`, with
  `vMin = min(GOV_MIN_SETPOINT_MS (1.4 m/s), 0.5 × L)`
- **AND** at `s` = 100 % the target SHALL equal `L`, so no speed above the limit can be commanded

#### Scenario: Setpoint is ramped
- **WHEN** the target speed changes
- **THEN** the setpoint used by the loop SHALL move toward it by at most the configured
  acceleration (m/s²) per second upward and the configured deceleration per second downward
- **AND** a lowered limit SHALL lower the target and the setpoint SHALL follow at the deceleration

#### Scenario: PI on the speed error with anti-windup
- **WHEN** the governor is ACTIVE with setpoint `vr` and measured speed `v`
- **THEN** the output SHALL be `clamp(Kp × (vr − v) + I, 0, uMax)`, where `I` integrates
  `Ki × (vr − v)` over time and is clamped to 0–`uMax`
- **AND** `v` SHALL be the speed reading as reported by the speed sensor, without additional
  filtering and without an error dead-band
- **AND** `I` SHALL NOT integrate further in the direction of a saturated output (output at `uMax`
  with a positive error, or at 0 with a negative error)
- **AND** the output SHALL move toward that value by at most the throttle rate `thr_slew` per second
- **AND** the elapsed time used SHALL be capped at one second

#### Scenario: Default gains at a 10 km/h limit
- **WHEN** the governor runs with the build's default settings (see "Speed Governor Settings"),
  limit 10 km/h, stick at 100 %, integrator 0, and the speed is 0 with the setpoint already ramped
  to 10 km/h (2.78 m/s)
- **THEN** the proportional part SHALL ask for `Kp` × 2.78: about 2.8 % on the buggy (`aztec`,
  Kp 1.0) and about 8.3 % on the quad bike (`master`, `old-board`, Kp 3.0)
- **AND** the output SHALL be bounded by the build's default start throttle limit (4 % on the
  buggy, 10 % on the quad bike) until the wheels turn

#### Scenario: Start throttle limit while the wheels do not turn
- **WHEN** the governor is ACTIVE and the measured speed is below `GOV_STANDSTILL_MS`
- **THEN** `uMax` SHALL be the configured start throttle limit instead of 100 %
- **AND** the integrator SHALL therefore not wind up beyond the start throttle limit while the
  vehicle stands (engine off, clutch not engaged, stuck)

#### Scenario: Stick released means stop
- **WHEN** the throttle input is at or below the 5 % dead-band
- **THEN** the throttle SHALL go to idle at once, without the output rate limit
- **AND** the setpoint and the integrator SHALL be reset to 0

#### Scenario: Downhill, the governor cannot hold the speed
- **WHEN** the output is at idle and the speed stays above the setpoint
- **THEN** the governor SHALL leave the throttle at idle and SHALL NOT act on the brake
- **AND** the web-only saturation indicator (see "Speed Governor Telemetry") SHALL report it

#### Scenario: Autopilot mode is not consulted
- **WHEN** the autopilot reports any flight mode
- **THEN** the governor's behaviour SHALL be the same as in MANUAL

### Requirement: Speed Governor Settings
The governor SHALL be configured by persisted per-vehicle settings in NVS namespace `"speed"`
(owned by `VehicleController`, shared with the speed sensor and the limiter, never cleared):

| Setting | Key | Type | Range | Default, buggy (`aztec`) | Default, quad bike (`master`, `old-board`) |
|---|---|---|---|---|---|
| switch | `gov_en` | bool | — | off | off |
| proportional gain | `gov_kp` | float, % per m/s | 0.2–10 | 1.0 | 3.0 |
| integral gain | `gov_ki` | float, %/s per m/s | 0–5 | 0.35 | 1.0 |
| setpoint acceleration | `gov_acc` | float, m/s² | 0.3–3.0 | 1.0 | 1.0 |
| setpoint deceleration | `gov_dec` | float, m/s² | 0.5–5.0 | 2.5 | 2.5 |
| start throttle limit | `gov_launch` | float, % | 1–40 | 4 | 10 |

The ranges SHALL be the same on every build; the defaults SHALL be those of the vehicle the build
is for (the column matching the build's vehicle type: 2 = buggy on `aztec`, 1 = quad bike on
`master` and `old-board`), and a build whose compiled governor defaults are tagged for another
vehicle type SHALL fail to compile. The same values SHALL apply at every effective limit (no
scheduling by the limit). They SHALL be loaded at start-up; an absent key SHALL read as the
build's default; a stored NaN or out-of-range value SHALL be logged and ignored (default used, not
rewritten). A default SHALL apply only while the key is absent or invalid: a valid stored value
SHALL never be replaced by a default, including after a firmware update that changes the
defaults. A new value SHALL be
validated, persisted first and applied only after the write succeeded, and SHALL take effect on the
next governor evaluation.

#### Scenario: Default on a unit without stored values
- **WHEN** the system starts and no `gov_*` key has been written
- **THEN** the governor switch SHALL be off and the throttle path SHALL be exactly as before this
  change
- **AND** when the switch is turned on the governor SHALL use acceleration 1.0 m/s² and
  deceleration 2.5 m/s², and on the buggy Kp 1.0, Ki 0.35 and a start throttle limit of 4 %, on
  the quad bike Kp 3.0, Ki 1.0 and a start throttle limit of 10 %

#### Scenario: Defaults belong to the build's vehicle
- **WHEN** the firmware is built for the buggy (vehicle type 2) while the compiled governor
  defaults are tagged as the quad bike's (vehicle type 1), or the other way round
- **THEN** the build SHALL fail with a message naming the governor defaults

#### Scenario: Stored values survive a change of defaults
- **WHEN** valid `gov_*` values are stored and a firmware with different compiled defaults is
  flashed
- **THEN** the stored values SHALL stay in use and SHALL NOT be rewritten

#### Scenario: Start throttle limit down to low single digits
- **WHEN** the operator sets the start throttle limit to 1 %
- **THEN** the value SHALL be accepted
- **AND** 0.5 % or 41 % SHALL be rejected

#### Scenario: Stored settings are restored
- **WHEN** the system starts with valid stored values
- **THEN** those values SHALL be in use, per vehicle

#### Scenario: Invalid stored value
- **WHEN** a stored `gov_*` value is NaN or outside its range
- **THEN** it SHALL be ignored and logged, the default SHALL be used, and the key SHALL NOT be
  rewritten

#### Scenario: Setting rejected
- **WHEN** a new value is outside its range or the NVS write fails
- **THEN** the value in use SHALL stay unchanged

### Requirement: Speed Governor Hand-Over and Speed-Sensor Fail-Open
Whenever the governor is not ACTIVE the throttle SHALL follow the path that existed before this
change (direct throttle, with the maximum-speed limiter where a limit exists). A change between
the two SHALL NOT produce a throttle step, except a closing to idle commanded by the operator or a
safety clip. An invalid speed reading SHALL make the governor stop governing (fail-open): the
throttle input SHALL become a direct throttle again without any governor restriction.

#### Scenario: Switch off is today's behaviour
- **WHEN** the governor switch is off
- **THEN** the commanded throttle SHALL be identical to the firmware without the governor, including
  the maximum-speed limiter

#### Scenario: No effective limit means direct throttle
- **WHEN** the switch is on and the effective limit is zero
- **THEN** the throttle input SHALL pass through as today

#### Scenario: Speed signal lost while governing
- **WHEN** the governor is ACTIVE and the speed reading becomes invalid
- **THEN** the governor SHALL stop governing and the throttle SHALL move from the governor's last
  output toward the direct (today's) value by at most `thr_slew` per second, in either direction
- **AND** once it has reached that value the throttle input SHALL be a direct throttle, so a stick
  at 100 % SHALL mean full throttle
- **AND** a stick moved into the dead-band or a brake command during the blend SHALL close the
  throttle to idle at once

#### Scenario: No pulse since boot counts as invalid
- **WHEN** the system has not seen a speed pulse since boot
- **THEN** the speed reading SHALL count as invalid for the governor and the throttle SHALL be direct
  until the first pulse, with the same no-speed warnings as any invalid speed
- **AND** with the first pulse the governor SHALL engage bumplessly from the throttle then
  commanded

#### Scenario: Engagement is bumpless
- **WHEN** the governor becomes ACTIVE — switch turned on, a limit appears, the speed becomes valid,
  a bypass ends, or the first evaluation after a gap longer than `GOV_MAX_GAP_MS`
- **THEN** the setpoint SHALL start at the measured speed, the output at the throttle percent `u0`
  last commanded to the throttle controller, and the integrator at `u0` but not above the start
  throttle limit
- **AND** the first governor output SHALL equal `u0`

#### Scenario: Take-over from a high direct throttle
- **WHEN** the governor becomes ACTIVE while the last commanded throttle is above the start
  throttle limit (for example 100 % on the first launch after boot, or 40 % when a 10 km/h limit
  appears at 25 km/h)
- **THEN** the first output SHALL equal that throttle, the integrator SHALL start at the start
  throttle limit
- **AND** the output SHALL then move toward the governor's value by at most `thr_slew` per second

#### Scenario: Turning the switch off while driving
- **WHEN** the switch is turned off while the governor is ACTIVE
- **THEN** the throttle SHALL blend to today's value at `thr_slew` as in the speed-loss case

### Requirement: Speed Governor Yields to Brake, Gear Change and Fail-Safe
The governor SHALL never fight the operator's brake, the gear-change throttle handling or the
fail-safe. The brake command path SHALL be unchanged by this change.

#### Scenario: RC brake on the combined channel
- **WHEN** RC channel 2 is below centre (brake)
- **THEN** the throttle input SHALL be 0 and the governor SHALL stop: idle at once, setpoint and
  integrator reset
- **AND** the brake SHALL be applied exactly as before this change

#### Scenario: Web brake while the governor is on
- **WHEN** the governor is ACTIVE and a brake target of 5 % or more is commanded from the web
- **THEN** the governor SHALL hold the throttle at idle and keep the setpoint and integrator reset
  while the brake target stays at 5 % or more
- **AND** with the governor switch off a web brake SHALL NOT change the web throttle, as before

#### Scenario: Neutral, unknown gear and gear change bypass the governor
- **WHEN** the expected gear is N or UNKNOWN, a gear change is active, the gear-change boost PID or
  idle hold owns the throttle, or the throttle is being calibrated
- **THEN** the governor SHALL NOT be evaluated and SHALL be reset
- **AND** the throttle SHALL be produced exactly as before this change, including the clip to
  `TRANS_UNKNOWN_GEAR_THROTTLE_MAX`, applied without a blend
- **AND** when the bypass ends the governor SHALL engage bumplessly

#### Scenario: Fail-safe
- **WHEN** fail-safe is entered
- **THEN** the throttle SHALL go to idle as before and the governor SHALL re-engage bumplessly from
  idle after the fail-safe ends

#### Scenario: Opening rate limit stays at the servo input
- **WHEN** the governor output rises and the throttle opening rate limit is configured
- **THEN** the output SHALL still pass through the opening rate limit before the servo

### Requirement: Ceiling Limiter Stands Down While the Speed Governor Is Active
While the governor is ACTIVE the maximum-speed ceiling limiter SHALL NOT be evaluated on the
throttle. This takes precedence over the statement in "Proportional Speed-Limiter Throttle Taper"
that the limiter is evaluated on every control-loop iteration for the autopilot and web throttle
paths (and, on the buggy, over "Fixed Per-Gear Maximum-Speed Ceiling" enforcing the gear ceiling
through the limiter law: the governor enforces it instead). In every other governor state the
limiter SHALL behave as specified in the limiter requirements ("Proportional Speed-Limiter Throttle
Taper", "Speed-Limiter Integral Trim", "Configurable Speed-Limiter Throttle Floor"), unchanged.

#### Scenario: No double limiting
- **WHEN** the governor is ACTIVE
- **THEN** the limiter state SHALL be reset on every evaluation (ceiling 100 %, trim 0, first-call
  sentinel) and its saturation indicator SHALL be cleared
- **AND** `speed_limit_ceil` SHALL report 100

#### Scenario: Limiter resumes when the governor stops
- **WHEN** the governor leaves the ACTIVE state while a limit is in force and the speed is valid
- **THEN** the limiter SHALL resume with its first-evaluation rule (taper adopted directly, zero
  trim), and the hand-over blend SHALL smooth the difference to the governor's last output

### Requirement: Configurable Pre-Crank Dwell
The pre-crank dwell — the time the ignition / ECU line SHALL have been powered before the starter
engages on an auto-crank — SHALL be a persisted, operator-set, per-vehicle value in whole
milliseconds, range `CRANK_DWELL_MS_MIN` (1000) to `CRANK_DWELL_MS_MAX` (8000), default
`ACC_PRECRANK_DWELL_MS` (3000). It SHALL be stored in the NVS namespace `"crank"`, key `dwell_ms`
(uint16), owned by `VehicleController`, loaded at boot and handed to `RelayController` through a
RAM-only setter. It SHALL apply to every crank armed through `requestCrank()`: a fresh RC ignition
ON, the re-crank when RC control returns after fail-safe or web control, and the GCS
`MAV_CMD_USER_2` ignition ON. The dwell SHALL be counted from the moment the ignition line last
went from OFF to powered. The web START command SHALL NOT use the dwell (it sets CRANKING
directly, unchanged). The armed-crank decision SHALL be the pure function `crankArmedDecide()` in
`include/CrankLaw.h`, covered by native tests.

#### Scenario: Default on a unit without a stored value
- **WHEN** the unit boots with no `crank`/`dwell_ms` key
- **THEN** the pre-crank dwell SHALL be 3000 ms

#### Scenario: Auto-crank waits for the configured dwell
- **WHEN** the dwell is 4500 ms and a fresh RC ignition ON arrives with the ignition OFF and the
  engine stopped
- **THEN** the ignition line SHALL come up at once and the reported state SHALL be ACC
- **AND** the starter SHALL engage on the first control loop at or after 4500 ms
- **AND** the cranking SHALL then end on engine start or after `CRANKING_TIMEOUT`, as before

#### Scenario: Ignition line already powered long enough
- **WHEN** the ignition has been in ACC for longer than the dwell and a crank is requested
- **THEN** the starter SHALL engage on the next control loop

#### Scenario: Stored value out of range
- **WHEN** the stored `dwell_ms` is outside 1000–8000
- **THEN** it SHALL be logged and ignored, the default SHALL be used, and the stored value SHALL NOT
  be rewritten

#### Scenario: New value applies at once
- **WHEN** a new dwell is saved while a crank is armed
- **THEN** the armed crank SHALL use the new value from the next control loop, without restarting
  the dwell clock

#### Scenario: Engine already running
- **WHEN** a crank is armed and the validated engine RPM is at or above
  `ENGINE_RUNNING_RPM_THRESHOLD`
- **THEN** the crank SHALL be suppressed and the state SHALL settle in IGNITION, whatever the dwell

### Requirement: Validated Engine RPM for Crank Control
The crank logic SHALL act only on an engine RPM the ECU (or, on the aztec buggy, the tacho) has
actually reported. `RelayController::update()` SHALL receive `crankRpm(dataValid, engineRPM)` —
the CAN engine RPM when the CAN data is valid, 0 otherwise — on the quad bike and the old board,
and the effective RPM `getEngineRpm()` on the aztec buggy. When the applied relay ignition state
goes from any other state to OFF, by any path (RC, GCS, web, fail-safe `allOff()`, starter
escalation), `VehicleController` SHALL clear the stored CAN engine RPM to 0
(`CANController::clearEngineRpm()`), leaving the CAN validity flag, its timestamp and every other
ECU field unchanged; the next ECU reply SHALL overwrite it.

#### Scenario: Crank right after switching the engine off at high RPM
- **WHEN** the engine runs at 1800 RPM, the RC ignition switch goes OFF and back to ON 1 s later
- **THEN** the stored engine RPM SHALL be 0 from the OFF transition
- **AND** the crank SHALL NOT be suppressed as "engine already running"
- **AND** the starter SHALL engage after the configured dwell

#### Scenario: CAN data invalid with a stale value
- **WHEN** the CAN data is invalid and the stored engine RPM is 1600
- **THEN** RelayController.update() SHALL receive 0
- **AND** an armed crank SHALL proceed to the starter after the dwell, and a running crank SHALL end
  only on a fresh reply at or above the threshold or on `CRANKING_TIMEOUT`

#### Scenario: Web START after an engine kill
- **WHEN** the engine was switched off at 1800 RPM and the web START is pressed within 5 s
- **THEN** it SHALL NOT be refused with "Engine already running"
- **AND** the starter SHALL NOT be released in the same loop as "Engine started!"

#### Scenario: Other ECU fields are untouched by the clear
- **WHEN** the ignition goes OFF
- **THEN** coolant temperature, MAP, module voltage, intake temperature and engine load SHALL keep
  their values and validity exactly as before this change

#### Scenario: Aztec effective RPM after OFF
- **WHEN** on the aztec buggy the ignition goes OFF with the engine running and the tacho times out
- **THEN** the effective RPM SHALL NOT fall back to a stale CAN value above 0

### Requirement: Lock Not Driven by an RC Channel
The lock function (`setWheelLock()`, "wheel lock" / "diff lock" in the API) SHALL NOT be driven by
any RC / MAVLink servo channel. MAVLink servo channel 7 SHALL NOT be read by the vehicle layer, and
its value — in band or out of band, changing or constant, on any frame including the first valid
frame after a re-sync — SHALL have no effect on the lock or on any other vehicle function. The
lock SHALL be changed only by the web `set_wheel_lock` command and by the GCS `MAV_CMD_USER_2` lock
function (`param1` 4), each with its existing checks. A lock state set by either SHALL be held until
the next web or GCS lock command; the fail-safe all-off SHALL keep it, as before. At boot the lock
SHALL be released (`RelayController::begin()` writes the port all-off with the lock state false; the
lock state is not persisted), and it SHALL stay released until a web or GCS lock command.

#### Scenario: Channel 7 switch moved
- **WHEN** MAVLink is in control with the lock released and servo channel 7 moves from 1100 µs to
  1900 µs and back
- **THEN** the lock SHALL stay released in every loop, `wheel_lock_on` SHALL stay `false` and
  `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL stay clear
- **AND** no lock relay write SHALL be made

#### Scenario: Engaged lock survives channel 7 at the opposite position
- **WHEN** the lock was engaged from the web or by an accepted GCS lock ON, and servo channel 7
  reads 1100 µs, then 1900 µs, then 1100 µs
- **THEN** the lock SHALL stay engaged

#### Scenario: Gaining MAVLink control does not touch the lock
- **WHEN** the lock was engaged from the web while the web portal was in control, and MAVLink
  control returns (web control released, fail-safe exit, or the RC stream returns while a browser
  is connected) with servo channel 7 at 1100 µs
- **THEN** the first valid frame SHALL set gear, ignition and headlight from CH3, CH4 and CH6 as
  before
- **AND** the lock SHALL stay engaged

#### Scenario: Boot with channel 7 high
- **WHEN** the board boots and the first valid frames carry servo channel 7 at 1900 µs
- **THEN** the lock SHALL be released and SHALL stay released until a web or GCS lock command

#### Scenario: Web and GCS still drive the lock
- **WHEN** a web `set_wheel_lock` command, or a GCS lock command accepted under MAVLink control,
  arrives on a build where the lock is fitted
- **THEN** the lock SHALL be set as commanded and reported in `wheel_lock_on` and
  `EFI_STATUS.pt_compensation` bit0, as before this change

### Requirement: Configurable Starter Time Limit
The starter time limit — the longest the starter SHALL run in one start attempt when the engine is
not detected as running — SHALL be a persisted, operator-set, per-vehicle value in whole
milliseconds, range `CRANK_MAX_MS_MIN` (2000) to `CRANK_MAX_MS_MAX` (10000), default
`CRANKING_TIMEOUT` (5000). It SHALL be stored in the NVS namespace `"crank"`, key `max_ms`
(uint16), owned by `VehicleController`, loaded at boot and handed to `RelayController` through a
RAM-only setter. It SHALL bound every start attempt, whatever started it: the RC ignition ON
auto-crank (including the re-crank when RC control returns), the GCS `MAV_CMD_USER_2` ignition ON
and the web START command. `RelayController` SHALL latch the value in use when it enters CRANKING;
that attempt SHALL end on engine start (validated RPM ≥ `ENGINE_RUNNING_RPM_THRESHOLD`, checked
first, unchanged) or once the latched limit has elapsed, whichever comes first. Wherever another
requirement names `CRANKING_TIMEOUT` as the bound of a start attempt, the bound SHALL be the limit
in use; `CRANKING_TIMEOUT` is its default. The timeout decision SHALL be the pure function
`crankTimedOut()` in `include/CrankLaw.h` (wrap-safe), covered by native tests. The web relay-test
cap of the starter relay (`RELAY_TEST_STARTER_MAX_MS`, 2000 ms, ignition OFF only) is not a start
attempt and SHALL stay fixed and separate.

#### Scenario: Default on a unit without a stored value
- **WHEN** the unit boots with no `crank`/`max_ms` key
- **THEN** the starter time limit SHALL be 5000 ms, as before this change

#### Scenario: Attempt ends at the configured limit
- **WHEN** the limit is 7000 ms and a start attempt (RC ON, GCS ignition ON or web START) engages
  the starter and the validated RPM stays below `ENGINE_RUNNING_RPM_THRESHOLD`
- **THEN** the starter SHALL be released on the first control loop at or after 7000 ms of cranking
- **AND** the state SHALL settle in IGNITION and "Cranking timeout" SHALL be logged
- **AND** no new attempt SHALL start without a new request (no automatic retry)

#### Scenario: Engine start still ends the attempt early
- **WHEN** the validated RPM reaches `ENGINE_RUNNING_RPM_THRESHOLD` before the limit
- **THEN** the starter SHALL be released at once, whatever the limit

#### Scenario: A change during cranking applies to the next attempt
- **WHEN** a new limit is saved while the starter is cranking
- **THEN** it SHALL be persisted and become the value in use at once
- **AND** the running attempt SHALL keep the limit it latched when it entered CRANKING
- **AND** the next attempt SHALL use the new value

#### Scenario: Stored value out of range
- **WHEN** the stored `max_ms` is outside 2000–10000
- **THEN** it SHALL be logged and ignored, the default SHALL be used, and the stored value SHALL NOT
  be rewritten

#### Scenario: Relay test cap unchanged
- **WHEN** the starter relay is energized from the web relay test
- **THEN** it SHALL still be released after `RELAY_TEST_STARTER_MAX_MS` (2000 ms), whatever the
  starter time limit

### Requirement: Effective Engine RPM Source Selection
The vehicle layer SHALL compose one EFFECTIVE engine RPM from the tacho input and the CAN
`VehicleData`, together with a source `rpm_source` ∈ {TACHO, CAN, NONE}, a validity flag and the
timestamp of the sample it came from. Every engine-RPM consumer — `isEngineRunning()` (MAVLink gear
gate, transmission init, throttle-calibration and gear-code refusals), the relay controller's starter
cut-off and crank interlock, the web START interlock, the gear-change boost PID, the engine hour
meter, web telemetry and MAVLink `EFI_STATUS` — SHALL read the effective RPM and SHALL NOT read
`VehicleData.engineRPM` directly. The selection SHALL be a pure function in `include/TachoLaw.h`.

#### Scenario: Live tacho wins
- **WHEN** the tacho is fitted and an accepted edge arrived within `TACHO_TIMEOUT_MS`
- **THEN** the effective RPM SHALL be the tacho RPM, source TACHO, valid
- **AND** this SHALL hold whether or not CAN `VehicleData` is valid, and whatever RPM CAN reports

#### Scenario: Silent tacho with a CAN RPM reading falls back to CAN
- **WHEN** the tacho is fitted but timed out
- **AND** CAN `VehicleData` is valid with `engineRPM` > 0
- **THEN** the effective RPM SHALL be the CAN RPM, source CAN, so that a broken tacho wire on an
  ECU that does report RPM can never make a running engine look stopped

#### Scenario: Silent fitted tacho reads a stopped engine
- **WHEN** the tacho is fitted and timed out
- **AND** CAN is invalid or reports 0 RPM
- **THEN** the effective RPM SHALL be 0, source TACHO, valid

#### Scenario: Tacho not fitted
- **WHEN** the tacho is not fitted
- **THEN** the effective RPM SHALL be the CAN RPM with source CAN while CAN `VehicleData` is valid
- **AND** SHALL be invalid with source NONE while CAN `VehicleData` is invalid, exactly as the
  behaviour before this change

#### Scenario: Engine-running predicate
- **WHEN** `isEngineRunning()` is evaluated
- **THEN** it SHALL return true only when the effective RPM is valid and at or above
  `ENGINE_RUNNING_RPM_THRESHOLD`

### Requirement: Engine-Running RPM Threshold
`ENGINE_RUNNING_RPM_THRESHOLD` SHALL be 800 RPM on the aztec buggy, whose warm idle is ≈1500 RPM:
above the starter cranking speed and safely below idle, so that every engine-running decision
recognises an idling engine.

#### Scenario: Idle counts as running
- **WHEN** the engine idles at ≈1500 RPM, or a cold or loaded idle sags to 1000 RPM
- **THEN** `isEngineRunning()` SHALL return true, so MAVLink gear changes are accepted and the
  crank interlock refuses the starter

#### Scenario: Cranking does not count as running
- **WHEN** the starter turns the engine at cranking speed (≈250–400 RPM) without it firing
- **THEN** the effective RPM SHALL stay below the threshold and the starter SHALL keep cranking until
  the engine fires or `CRANKING_TIMEOUT` elapses

#### Scenario: Engine stop is seen promptly
- **WHEN** the engine stops with the tacho as source
- **THEN** the effective RPM SHALL read 0 within `TACHO_TIMEOUT_MS` (0.5 s) of the last pulse, and
  `isEngineRunning()` SHALL become false

### Requirement: Aztec Lock on Relay_4 + Relay_5 + Relay_6
The aztec buggy SHALL drive its lock ("блокування") through the existing wheel-lock function with
Relay_4, Relay_5 and Relay_6 switched together as one relay-number set
(`RELAY_SET_WHEEL_LOCK = RELAY_N(4) | RELAY_N(5) | RELAY_N(6)`, `RELAY_WHEEL_LOCK_FITTED = 1`;
PCA9557 mask `0x8C` = IO7|IO3|IO2 on relay-board connectors X7/X8/X9, XL9535 mask `0x38` =
P0.3..P0.5). The lock SHALL be changed only by the web `set_wheel_lock` command, which is not restricted by the
input source, and by the GCS `MAV_CMD_USER_2` lock function (`param1` 4), which is accepted only
under MAVLink control (see "GCS Vehicle Command Interlocks"); no RC / MAVLink servo channel SHALL
drive it (see "Lock Not Driven by an RC Channel"). Neither path has an ignition interlock.
`setWheelLock()` SHALL keep a "not fitted" branch (reject a LOCK, log once per boot, report released)
selected only when `RELAY_WHEEL_LOCK_FITTED` is `0`.

#### Scenario: Lock from the web
- **WHEN** `VehicleController::processWheelLockCommand(true)` runs for a web `set_wheel_lock` command
- **THEN** `RelayController::setWheelLock(true)` SHALL energize Relay_4, Relay_5 and Relay_6 in one
  verified whole-port write and return `true`
- **AND** the web response SHALL be success ("Lock LOCKED")
- **AND** `getWheelLock()` SHALL return `true`

#### Scenario: Unlock
- **WHEN** `setWheelLock(false)` is called while the lock is engaged
- **THEN** all three lock relays SHALL be released in one verified write and `getWheelLock()` SHALL
  return `false`
- **AND** a `RelayController::setWheelLock(false)` while already released SHALL return `true`
  without a port write and without logging

#### Scenario: Lock refused while the relay board is unavailable
- **WHEN** a LOCK is requested while the relay expander failed to initialize or is faulted
- **THEN** `setWheelLock()` SHALL return `false`, log the refusal and leave the lock state unchanged
- **AND** the web response SHALL be an error ("Wheel lock command rejected")

#### Scenario: Lock independent of the ignition
- **WHEN** a LOCK or UNLOCK command arrives with the ignition in any state, including `CRANKING`
- **THEN** it SHALL be applied without changing the ignition, starter or light relays

#### Scenario: Fail-safe all-off keeps the lock
- **WHEN** `RelayController::allOff()` runs
- **THEN** ignition, starter and light SHALL be switched off
- **AND** the lock SHALL hold its last commanded state

#### Scenario: Fitted state is queryable
- **WHEN** `VehicleController::isWheelLockFitted()` or `RelayController::isWheelLockFitted()` is called
- **THEN** it SHALL return `true`

