# vehicle-systems Capability Delta

## ADDED Requirements

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
