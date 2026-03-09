# web-control Spec Delta

## MODIFIED Requirements

### Requirement: Web-Based Manual Control
The system SHALL accept manual control commands from the web interface when S-bus input is inactive.

#### Scenario: Receive brake control command from web
- **WHEN** set_brake command is received via WebSocket with value 0 to 100
- **AND** input source is WEB (S-bus inactive)
- **THEN** brake value is validated (0 to 100 range, integer or float)
- **AND** BrakeController or BTS7960Controller is called with brake percentage
- **AND** success response is sent to web client
- **AND** brake position updates in real-time

#### Scenario: Reject brake control when S-bus active
- **WHEN** set_brake command is received via WebSocket
- **AND** input source is SBUS (S-bus signal valid)
- **THEN** command is rejected
- **AND** error response is sent with reason "S-bus control active"
- **AND** no brake actuator state change occurs

#### Scenario: Validate brake command value range
- **WHEN** set_brake command is received with value <0 or >100
- **THEN** command is rejected
- **AND** error response is sent with validation error message "Value must be 0-100"
- **AND** no brake actuator state change occurs

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

#### Scenario: Disable brake control when S-bus active
- **WHEN** input source is SBUS
- **THEN** brake slider is disabled (greyed out)
- **AND** brake slider shows current SBUS-controlled position (read-only)
- **AND** slider cannot be moved by user

#### Scenario: Send brake command from web UI
- **WHEN** user moves brake slider
- **THEN** set_brake command is sent via WebSocket with slider value (0-100)
- **AND** commands are rate-limited (max 10 Hz, one per 100ms) to avoid flooding
- **AND** slider position shows visual feedback during command transmission

### Requirement: Web Control Safety
The system SHALL enforce safety constraints on web-based control commands.

#### Scenario: Apply rate limiting to brake commands
- **WHEN** web client moves brake slider rapidly
- **THEN** brake commands are sent at maximum 10 Hz (one command per 100ms)
- **AND** intermediate slider values are debounced
- **AND** final position is always sent after user stops sliding

#### Scenario: Reset brake on client disconnect
- **WHEN** WebSocket client disconnects
- **AND** brake was being controlled via web
- **THEN** brake is released to 0% (fail-safe)
- **AND** system switches to FAILSAFE input source
- **AND** fail-safe brake state is applied (fully released unless emergency)
