## ADDED Requirements

### Requirement: ECU Capability Probe Command via Web Interface
The system SHALL accept a `can_probe` command from the web interface to trigger a one-shot ECU
capability probe, and SHALL surface the probe's progress and outcome back to the web client.

#### Scenario: Trigger probe via WebSocket
- **WHEN** a `can_probe` command is received via WebSocket
- **AND** CAN data is valid and no gear change is active
- **THEN** the command SHALL be routed through `VehicleController::processWebCommand` to
  `CANController::startProbe()`
- **AND** the controller SHALL enter probe mode and begin the capability sweep
- **AND** the command SHALL be accepted regardless of the active input source (diagnostic
  privilege level, same as calibration commands)

#### Scenario: Reject or defer probe while busy or unavailable
- **WHEN** a `can_probe` command is received while a probe is already running, or while a gear
  change is active
- **THEN** the command SHALL be rejected/deferred without disrupting the in-progress probe or gear
  change
- **AND** the response SHALL indicate a busy/deferred status
- **WHEN** a `can_probe` command is received while CAN data is invalid (ECU disconnected)
- **THEN** the probe SHALL short-circuit with a "no ECU / disconnected" result and no bus traffic

#### Scenario: Deliver probe results to the web client
- **WHEN** the probe completes
- **THEN** the probe results (supported-PID bitmaps, per-PID supported/answered/raw/decoded, and
  the DTC summary) SHALL be delivered to connected web clients via the existing telemetry
  WebSocket broadcast while the results are fresh
- **AND** the results SHALL also be mirrored to the serial log via the CAN debug feature

#### Scenario: Display probe control and results in web UI
- **WHEN** the web page loads
- **THEN** a "Probe ECU" button SHALL be available to send the `can_probe` command
- **AND** the button SHALL show an in-progress state and reject further clicks while a probe is
  running
- **AND** a results panel SHALL display the returned bitmaps, per-PID results, and DTC summary
  when results are received
