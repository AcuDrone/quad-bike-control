## ADDED Requirements

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
