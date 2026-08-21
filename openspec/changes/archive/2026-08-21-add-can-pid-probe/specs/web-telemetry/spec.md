## ADDED Requirements

### Requirement: Manifold Absolute Pressure Telemetry
The system SHALL include manifold absolute pressure (MAP) in the CAN vehicle-data telemetry so
the web UI can display it alongside the other engine parameters.

#### Scenario: Include map_kpa in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **AND** CAN status is "connected"
- **THEN** the telemetry JSON SHALL include a `map_kpa` field carrying the manifold absolute
  pressure in kPa (0–255)
- **WHEN** CAN status is not "connected"
- **THEN** the `map_kpa` field SHALL be omitted alongside the other CAN vehicle-data fields

#### Scenario: Display MAP in the CAN telemetry card
- **WHEN** a telemetry message containing `map_kpa` is received
- **THEN** the CAN telemetry card SHALL display the MAP value in kPa
- **AND** the value SHALL update in real time as new telemetry arrives

### Requirement: ECU Probe Results Telemetry
The telemetry broadcast SHALL carry ECU capability probe results to connected web clients while
the results are fresh, so the web UI can render them without a separate request/response channel.

#### Scenario: Include probe object while results are fresh
- **WHEN** an ECU probe is running or has completed within the freshness window
- **THEN** the telemetry JSON SHALL include a `probe` object containing the probe running/complete
  state, the supported-PID bitmaps, a per-candidate-PID list (with supported-by-bitmap, answered,
  raw bytes, and decoded value where known), and a DTC summary (count and codes)
- **AND** the `probe` object SHALL include a truncation indicator when the reported DTC count
  exceeds what a single classic frame can carry

#### Scenario: Omit probe object when stale
- **WHEN** no probe is running and the last completed results are older than the freshness window
- **THEN** the telemetry JSON SHALL omit the `probe` object
- **AND** steady-state telemetry size SHALL be unaffected by the probe feature

#### Scenario: Late-joining client still sees fresh results
- **WHEN** a web client connects after a probe completes but within the freshness window
- **THEN** the next telemetry broadcast SHALL include the `probe` object so the client can render
  the results without re-triggering the probe
