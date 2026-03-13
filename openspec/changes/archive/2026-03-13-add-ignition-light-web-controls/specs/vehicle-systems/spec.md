## ADDED Requirements

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
