## ADDED Requirements

### Requirement: Gear Position Default Configuration via Web Interface
The system SHALL allow the user to view and update the default encoder position for each transmission gear through the web portal, without requiring a firmware recompile.

#### Scenario: Set a gear's default position via WebSocket
- **WHEN** a `set_gear_default` WebSocket command is received with a valid gear string (`"R"`, `"N"`, `"L"`, `"H"`) and an integer position value
- **THEN** the system SHALL call `setDefaultPosition()` on the transmission controller
- **AND** SHALL respond with `{"ok": true, "gear": "<X>", "position": <N>}`
- **AND** this command SHALL be accepted regardless of the active input source (same privilege level as calibration commands)

#### Scenario: Reject invalid gear or out-of-range position
- **WHEN** a `set_gear_default` command is received with an unrecognised gear string or a position outside a reasonable range (e.g., negative or > 10 000 counts)
- **THEN** the system SHALL respond with `{"ok": false, "error": "<reason>"}` and leave NVS unchanged

#### Scenario: Display gear position defaults editor in web UI
- **WHEN** the web portal is open
- **THEN** a "Gear Position Defaults" section SHALL be visible (or accessible via expand/collapse) on the page
- **AND** it SHALL show four numeric input fields labelled R, N, L, H
- **AND** the fields SHALL be pre-populated with the current default values received via telemetry

#### Scenario: Save a default from the web UI
- **WHEN** the user edits a position field and clicks its Save button
- **THEN** the web UI SHALL send a `set_gear_default` WebSocket command with the updated value
- **AND** on success response, the input field SHALL reflect the saved value

#### Scenario: Show live encoder count near gear controls
- **WHEN** the Manual Control section is displayed
- **THEN** the current encoder count (from `hall_position` telemetry) SHALL be visible adjacent to the gear buttons
- **AND** it SHALL update in real time so the user can drive to a gear position and read the exact count to enter as a default
