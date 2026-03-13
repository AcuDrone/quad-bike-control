# vehicle-systems Specification

## Purpose
TBD - created by archiving change add-vehicle-control-system. Update Purpose after archive.
## Requirements
### Requirement: Steering Control System
The system SHALL provide steering control through a servo-driven steering wheel actuator.

#### Scenario: Set steering position by percentage
- **WHEN** SteeringSystem.setPosition() is called with value from -100 to +100
- **THEN** steering servo moves to corresponding angle
- **AND** -100% maps to full left (e.g., 0 degrees)
- **AND** 0% maps to center (e.g., 90 degrees)
- **AND** +100% maps to full right (e.g., 180 degrees)

#### Scenario: Center steering wheel
- **WHEN** SteeringSystem.center() is called
- **THEN** steering servo returns to center position (0%)
- **AND** position is held until new command

#### Scenario: Smooth steering transitions
- **WHEN** rapid steering changes are commanded
- **THEN** servo movement is rate-limited to prevent jerking
- **AND** maximum steering rate is configurable (default 200 deg/sec)

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
The system SHALL control gear selection through a linear actuator with 4 discrete positions.

#### Scenario: Select transmission gear
- **WHEN** TransmissionSystem.setGear() is called with valid gear (REVERSE, NEUTRAL, HIGH, LOW)
- **THEN** linear actuator moves to calibrated position for that gear
- **AND** system waits for position confirmation or timeout
- **AND** current gear state is updated

#### Scenario: Calibrate transmission positions
- **WHEN** TransmissionSystem.calibrate() is called
- **THEN** user is prompted to manually select each gear
- **AND** actuator position for each gear is recorded
- **AND** calibration data is stored in non-volatile memory (NVS)
- **AND** calibration timestamp is saved

#### Scenario: Load calibration on startup
- **WHEN** TransmissionSystem.begin() is called
- **THEN** calibration data is loaded from NVS
- **AND** if no valid calibration exists, system prompts for calibration
- **AND** transmission moves to NEUTRAL position

#### Scenario: Query current gear
- **WHEN** TransmissionSystem.getCurrentGear() is called
- **THEN** last commanded gear is returned
- **AND** position uncertainty is indicated if calibration is stale

#### Scenario: Safe gear transitions
- **WHEN** changing from REVERSE to HIGH or LOW (or vice versa)
- **THEN** system requires passing through NEUTRAL first
- **AND** throttle must be at idle during transition
- **AND** minimum dwell time in NEUTRAL is enforced (e.g., 500ms)

### Requirement: Brake Control System
The system SHALL control braking force through a linear actuator-driven brake mechanism.

#### Scenario: Set brake position by percentage
- **WHEN** BrakeSystem.setPosition() is called with value from 0 to 100
- **THEN** brake actuator extends proportionally
- **AND** 0% maps to fully released brakes
- **AND** 100% maps to maximum braking force

#### Scenario: Release brakes
- **WHEN** BrakeSystem.release() is called
- **THEN** brake actuator retracts to 0% position
- **AND** brakes are confirmed released

#### Scenario: Emergency braking
- **WHEN** BrakeSystem.emergencyStop() is called
- **THEN** brakes are immediately applied to 100%
- **AND** throttle is forced to idle
- **AND** transmission remains in current gear

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
The system SHALL accept vehicle control commands from multiple input sources (S-bus, web interface) with priority management.

#### Scenario: Apply commands from active input source
- **WHEN** control loop executes
- **THEN** input source priority is evaluated (SBUS > WEB > FAILSAFE)
- **AND** commands are read from highest priority active source
- **AND** commands are applied to vehicle systems

#### Scenario: Apply S-bus commands when S-bus active
- **WHEN** S-bus signal is valid
- **THEN** S-bus commands control steering, throttle, transmission, and brakes
- **AND** web control commands are ignored
- **AND** fail-safe is not active

#### Scenario: Apply web commands when S-bus inactive
- **WHEN** S-bus signal is invalid or timed out
- **AND** web control commands are available
- **THEN** web commands control steering, throttle, transmission, and brakes
- **AND** commands are validated before application
- **AND** fail-safe is not active

#### Scenario: Apply fail-safe when all sources inactive
- **WHEN** S-bus signal is invalid
- **AND** no web control commands received for >1 second
- **THEN** fail-safe commands are applied to all systems
- **AND** vehicle enters safe state

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
- **THEN** current active input source is returned (SBUS/WEB/FAILSAFE)
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

