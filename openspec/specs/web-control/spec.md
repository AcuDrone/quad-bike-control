# web-control Specification

## Purpose
TBD - created by archiving change add-web-portal. Update Purpose after archive.
## Requirements
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

### Requirement: Input Source Priority Management
The system SHALL enforce input source priority to prevent control conflicts.

#### Scenario: Activate web control when S-bus inactive
- **WHEN** S-bus signal is lost or times out (>500ms)
- **AND** no recent S-bus frames received
- **THEN** input source switches to WEB (if web control commands present) or FAILSAFE (if no commands)
- **AND** web control commands are accepted
- **AND** web interface enables manual control UI

#### Scenario: Deactivate web control when S-bus reconnects
- **WHEN** S-bus signal becomes valid after being inactive
- **THEN** input source switches to SBUS immediately
- **AND** web control commands are rejected
- **AND** web interface disables manual control UI (read-only telemetry only)
- **AND** S-bus commands take control of vehicle

#### Scenario: Apply fail-safe when both sources inactive
- **WHEN** S-bus signal is inactive (>500ms timeout)
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

### Requirement: Ignition Control via Web Interface
The system SHALL accept ignition control commands from the web interface to manage vehicle ignition states.

#### Scenario: Set ignition state via WebSocket
- **WHEN** set_ignition command is received via WebSocket with value OFF/ACC/IGNITION/START
- **THEN** ignition state value is validated (must be one of: OFF, ACC, IGNITION, START)
- **AND** VehicleController.setIgnitionState() is called with the requested state
- **AND** success response is sent to web client
- **AND** ignition state updates in real-time telemetry

#### Scenario: Reject invalid ignition state command
- **WHEN** set_ignition command is received with invalid state value
- **THEN** command is rejected
- **AND** error response is sent with validation error message "Invalid ignition state"
- **AND** no ignition state change occurs

#### Scenario: Display ignition control buttons in web UI
- **WHEN** web page loads
- **THEN** "Vehicle Ignition & Lighting" card is displayed
- **AND** card includes four ignition buttons: OFF, ACC, IGNITION, START
- **AND** active ignition state button is visually highlighted
- **AND** buttons are always enabled regardless of S-bus status (ignition control available in all modes)

#### Scenario: Highlight active ignition state
- **WHEN** telemetry updates ignition state
- **THEN** corresponding button is highlighted with active styling
- **AND** other ignition buttons show inactive styling
- **AND** START button returns to inactive after cranking completes

#### Scenario: Send ignition command on button click
- **WHEN** user clicks ignition button
- **THEN** set_ignition command is sent via WebSocket with button value (OFF/ACC/IGNITION/START)
- **AND** button shows visual feedback during command transmission

### Requirement: Front Light Control via Web Interface
The system SHALL accept front light control commands from the web interface.

#### Scenario: Set front light state via WebSocket
- **WHEN** set_light command is received via WebSocket with boolean value
- **THEN** light state is validated (boolean true/false)
- **AND** VehicleController.setFrontLight() is called with on/off state
- **AND** success response is sent to web client
- **AND** light state updates in real-time telemetry

#### Scenario: Reject invalid light command
- **WHEN** set_light command is received with non-boolean value
- **THEN** command is rejected
- **AND** error response is sent with validation error message "Invalid light state"
- **AND** no light state change occurs

#### Scenario: Display front light toggle in web UI
- **WHEN** web page loads
- **THEN** "Vehicle Ignition & Lighting" card includes front light toggle switch
- **AND** toggle switch reflects current light state from telemetry
- **AND** toggle is always enabled regardless of S-bus status

#### Scenario: Toggle light state on user interaction
- **WHEN** user clicks light toggle switch
- **THEN** set_light command is sent via WebSocket with new state (true/false)
- **AND** toggle shows visual feedback during command transmission

### Requirement: Ignition and Light Telemetry Broadcasting
The system SHALL broadcast current ignition state and front light status in telemetry updates.

#### Scenario: Include ignition state in WebSocket telemetry
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes ignition_state field with current state (OFF/ACC/IGNITION/CRANKING)
- **AND** telemetry includes is_cranking boolean field (true during starter motor operation)

#### Scenario: Include light state in WebSocket telemetry
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes front_light_on boolean field with current light state
- **AND** light state reflects actual relay output

#### Scenario: Update ignition display in telemetry section
- **WHEN** web page receives telemetry update
- **THEN** ignition state indicator updates to show current state
- **AND** cranking indicator shows animated/highlighted state during START operation
- **AND** front light indicator updates to show ON/OFF state

