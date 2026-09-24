# vehicle-systems Specification

## Purpose
TBD - created by archiving change add-vehicle-control-system. Update Purpose after archive.
## Requirements
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

### Requirement: Throttle Control System
The system SHALL provide throttle control through a servo-driven acceleration mechanism.

#### Scenario: Set throttle position by percentage
- **WHEN** ThrottleSystem.setPosition() is called with value from 0 to 100
- **THEN** throttle servo moves to corresponding position
- **AND** 0% maps to idle/closed throttle
- **AND** 100% maps to full throttle

#### Scenario: Return to idle
- **WHEN** ThrottleSystem.idle() is called
- **THEN** throttle servo returns to 0% position
- **AND** throttle remains at idle until new command

#### Scenario: Throttle safety override
- **WHEN** brake system is engaged above threshold (e.g., >50%)
- **THEN** throttle is automatically reduced to idle
- **AND** throttle commands are ignored until brakes release

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
- **AND** state reflects actual RELAY3 output

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
The maximum-speed limiter's ceiling SHALL have exactly one source: the autopilot's `SPEED_MAX`
parameter, in metres per second. There SHALL be no locally stored ceiling, no local enable toggle
and no arbitration. The limiter SHALL always be armed, and SHALL simply not limit whenever no
usable `SPEED_MAX` is available. The value SHALL be held in RAM only and SHALL NOT be persisted.

#### Scenario: A usable autopilot value is the limit
- **WHEN** the MAVLink interface reports a usable `SPEED_MAX`
- **THEN** the limit SHALL be that value in metres per second
- **AND** no other source SHALL be consulted, because there is no other source

#### Scenario: No usable value means no limiting
- **WHEN** the MAVLink interface reports no usable `SPEED_MAX` — never received, zero, rejected,
  stale, or the link is down
- **THEN** the limit SHALL be reported as zero
- **AND** a zero limit SHALL mean NO LIMITING: the throttle demand SHALL pass through untouched
- **AND** the system SHALL NOT substitute any stored, default or last-known ceiling, because an
  invisible ceiling the ground station cannot see or change is the more dangerous outcome

#### Scenario: Zero means "no limit", as the autopilot reads it
- **WHEN** the autopilot's `SPEED_MAX` is zero
- **THEN** the limiter SHALL not limit
- **AND** this SHALL match ArduPilot's own reading of a zero `SPEED_MAX` and the value a ground
  station writes to clear a speed limit

#### Scenario: No persisted limiter configuration
- **WHEN** any `SPEED_MAX` value is received, accepted or applied
- **THEN** no non-volatile write SHALL be performed on this path
- **AND** the system SHALL NOT expose any command, key or web control for setting a local ceiling
  or for enabling and disabling the limiter
- **AND** the retired non-volatile limiter keys SHALL be deleted on initialisation so a downgraded
  unit cannot resurrect a ceiling in the old units

#### Scenario: Log a limit change once
- **WHEN** the limit changes by more than `SPEED_LIMIT_LOG_EPSILON_MS`, or changes between "a
  limit" and "no limit"
- **THEN** the new limit SHALL be logged once, in m/s with km/h in parentheses, or as "none"
- **AND** a further log SHALL be suppressed until at least `SPEED_LIMIT_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered limit SHALL NOT be updated, so the settled
  value is logged on the next opportunity rather than lost

### Requirement: Proportional Speed-Limiter Throttle Taper
The maximum-speed limiter SHALL withdraw throttle authority proportionally as the vehicle
approaches its limit, instead of applying a single reduced cap once the limit is crossed. All
comparisons SHALL be made in metres per second. The limiter SHALL be evaluated on every
control-loop iteration for both the autopilot and the web throttle paths. It SHALL continue to
fail open on an invalid speed reading, and SHALL continue to leave the gear-change throttle boost
untouched.

#### Scenario: No limit means no taper
- **WHEN** the limit is zero
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 %, so a limit that arrives later never
  resumes from a stale low ceiling

#### Scenario: Below the taper band the demand passes through
- **WHEN** a limit is in force and the speed reading is valid
- **AND** the measured speed is at or below `limit − SPEED_LIMIT_TAPER_BAND_MS`
- **THEN** the throttle ceiling SHALL be 100 %
- **AND** the commanded throttle SHALL equal the arbitrated demand

#### Scenario: Inside the taper band the ceiling falls linearly
- **WHEN** the measured speed is between `limit − SPEED_LIMIT_TAPER_BAND_MS` and the limit
- **THEN** the throttle ceiling SHALL fall linearly from 100 % to `SPEED_LIMIT_FLOOR_PCT` across
  that band
- **AND** the commanded throttle SHALL be the lesser of the arbitrated demand and that ceiling

#### Scenario: At or above the limit the floor holds
- **WHEN** the measured speed is at or above the limit
- **THEN** the throttle ceiling SHALL be `SPEED_LIMIT_FLOOR_PCT`
- **AND** the throttle SHALL NOT be cut to zero, because removing all drive mid-corner is a
  stability event rather than a safety measure

#### Scenario: The ceiling is rate-limited in both directions
- **WHEN** the computed target ceiling differs from the ceiling currently applied
- **THEN** the applied ceiling SHALL move toward the target by at most
  `SPEED_LIMIT_CEILING_SLEW_PCT_S` per second, whether rising or falling
- **AND** the rate limit SHALL be applied to the ceiling and not to the operator's demand, so the
  driver's own throttle movements are never slowed by the limiter
- **AND** the elapsed time used SHALL be capped at one second, and the first evaluation after the
  limiter becomes active SHALL adopt the target directly, so a stalled or restarted loop cannot
  produce a step change through an unbounded interval

#### Scenario: Invalid speed reading fails open
- **WHEN** a limit is in force but the speed reading is invalid or stale
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 %
- **AND** a warning SHALL be logged no more often than `SPEED_LIMIT_WARN_MS`

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

