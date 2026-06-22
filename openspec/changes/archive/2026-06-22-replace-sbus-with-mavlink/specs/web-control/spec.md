## MODIFIED Requirements

### Requirement: Web-Based Manual Control
The system SHALL accept manual control commands from the web interface when the MAVLink
command stream is inactive.

#### Scenario: Receive brake control command from web
- **WHEN** set_brake command is received via WebSocket with value 0 to 100
- **AND** input source is WEB (MAVLink inactive)
- **THEN** brake value is validated (0 to 100 range, integer or float)
- **AND** BrakeController or BTS7960Controller is called with brake percentage
- **AND** success response is sent to web client
- **AND** brake position updates in real-time

#### Scenario: Reject brake control when MAVLink active
- **WHEN** set_brake command is received via WebSocket
- **AND** input source is MAVLINK (MAVLink command stream valid)
- **THEN** command is rejected
- **AND** error response is sent with reason "MAVLink control active"
- **AND** no brake actuator state change occurs

#### Scenario: Validate brake command value range
- **WHEN** set_brake command is received with value <0 or >100
- **THEN** command is rejected
- **AND** error response is sent with validation error message "Value must be 0-100"
- **AND** no brake actuator state change occurs

### Requirement: Input Source Priority Management
The system SHALL enforce input source priority to prevent control conflicts.

#### Scenario: Activate web control when MAVLink inactive
- **WHEN** the MAVLink command stream is lost or times out (>500ms)
- **AND** no recent `SERVO_OUTPUT_RAW` commands received
- **THEN** input source switches to WEB (if web control commands present) or FAILSAFE (if no commands)
- **AND** web control commands are accepted
- **AND** web interface enables manual control UI

#### Scenario: Deactivate web control when MAVLink reconnects
- **WHEN** the MAVLink command stream becomes valid after being inactive
- **THEN** input source switches to MAVLINK immediately
- **AND** web control commands are rejected
- **AND** web interface disables manual control UI (read-only telemetry only)
- **AND** MAVLink commands take control of vehicle

#### Scenario: Apply fail-safe when both sources inactive
- **WHEN** the MAVLink command stream is inactive (>500ms timeout)
- **AND** no web control commands received for >1 second
- **THEN** input source switches to FAILSAFE
- **AND** fail-safe commands are applied (brakes on, neutral, center steering, idle throttle)
- **AND** telemetry shows FAILSAFE input source

### Requirement: Web Control User Interface
The system SHALL provide web interface controls for manual operation.

#### Scenario: Display brake control slider
- **WHEN** web page loads
- **THEN** manual control section includes brake slider
- **AND** brake slider range is 0% to 100% (released to fully applied)
- **AND** slider default position is 0% (released)
- **AND** slider is styled consistently with steering/throttle sliders

#### Scenario: Enable brake control when web control active
- **WHEN** input source is WEB
- **THEN** brake slider is enabled
- **AND** brake slider position reflects telemetry feedback
- **AND** brake percentage value is displayed next to slider

#### Scenario: Disable brake control when MAVLink active
- **WHEN** input source is MAVLINK
- **THEN** brake slider is disabled (greyed out)
- **AND** brake slider shows current MAVLink-controlled position (read-only)
- **AND** slider cannot be moved by user

#### Scenario: Send brake command from web UI
- **WHEN** user moves brake slider
- **THEN** set_brake command is sent via WebSocket with slider value (0-100)
- **AND** commands are rate-limited (max 10 Hz, one per 100ms) to avoid flooding
- **AND** slider position shows visual feedback during command transmission
