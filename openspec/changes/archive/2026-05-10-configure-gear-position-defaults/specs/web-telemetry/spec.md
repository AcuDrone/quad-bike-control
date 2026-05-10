## ADDED Requirements

### Requirement: Gear Default Positions in Telemetry
The telemetry broadcast SHALL include the current effective default positions for all four gears so the web UI can populate the defaults editor without a separate request.

#### Scenario: Include gear_defaults in telemetry JSON
- **WHEN** the system broadcasts a telemetry update
- **THEN** the JSON payload SHALL include a `gear_defaults` object with keys `R`, `N`, `L`, `H` containing the current default encoder counts for each gear
- **AND** the values SHALL reflect the active NVS defaults (or factory constants when no NVS defaults are set)

#### Scenario: Web UI populates defaults editor from telemetry
- **WHEN** a telemetry message containing `gear_defaults` is received
- **AND** the user is not currently editing the corresponding input field
- **THEN** the web UI SHALL update the input fields with the received values
