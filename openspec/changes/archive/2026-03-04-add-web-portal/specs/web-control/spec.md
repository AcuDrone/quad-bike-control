# web-control Capability Delta

## ADDED Requirements

### Requirement: Web-Based Manual Control
The system SHALL accept manual control commands from the web interface when S-bus input is inactive.

#### Scenario: Receive gear selection command from web
- **WHEN** set_gear command is received via WebSocket
- **AND** input source is WEB (S-bus inactive)
- **THEN** gear value is validated (R/N/L/H)
- **AND** TransmissionController.setGear() is called with requested gear
- **AND** success response is sent to web client
- **AND** telemetry update reflects new gear position

#### Scenario: Receive steering control command from web
- **WHEN** set_steering command is received via WebSocket with value -100 to +100
- **AND** input source is WEB (S-bus inactive)
- **THEN** steering value is validated (-100 to +100 range)
- **AND** ServoController.setAngle() is called with calculated angle
- **AND** success response is sent to web client
- **AND** steering position updates in real-time

#### Scenario: Receive throttle control command from web
- **WHEN** set_throttle command is received via WebSocket with value 0 to 100
- **AND** input source is WEB (S-bus inactive)
- **THEN** throttle value is validated (0 to 100 range)
- **AND** ServoController.setAngle() is called with calculated angle
- **AND** success response is sent to web client
- **AND** throttle position updates in real-time

#### Scenario: Reject web control when S-bus active
- **WHEN** control command is received via WebSocket
- **AND** input source is SBUS (S-bus signal valid)
- **THEN** command is rejected
- **AND** error response is sent to web client with reason "S-bus control active"
- **AND** no vehicle state change occurs

#### Scenario: Validate command value ranges
- **WHEN** web control command is received with out-of-range value
- **THEN** command is rejected
- **AND** error response is sent with validation error message
- **AND** no vehicle state change occurs

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

#### Scenario: Display manual control UI section
- **WHEN** web page loads
- **THEN** manual control section is displayed with:
  - Gear selector buttons (R, N, L, H)
  - Steering slider (-100% to +100% with center detent)
  - Throttle slider (0% to 100%)
- **AND** control elements are styled for easy touch/mouse interaction

#### Scenario: Enable controls when web control active
- **WHEN** input source is WEB
- **THEN** gear buttons are enabled
- **AND** steering slider is enabled
- **AND** throttle slider is enabled
- **AND** visual indicator shows "Manual Control Active"

#### Scenario: Disable controls when S-bus active
- **WHEN** input source is SBUS
- **THEN** gear buttons are disabled (greyed out)
- **AND** steering slider is disabled
- **AND** throttle slider is disabled
- **AND** visual indicator shows "S-bus Control Active - Manual Control Disabled"

#### Scenario: Send gear selection from web UI
- **WHEN** user clicks gear button (R/N/L/H)
- **THEN** set_gear command is sent via WebSocket with selected gear
- **AND** button visual feedback shows command sent (loading state)
- **AND** response or error is displayed to user

#### Scenario: Send steering command from web UI
- **WHEN** user moves steering slider
- **THEN** set_steering command is sent via WebSocket with slider value
- **AND** commands are rate-limited (max 10 Hz) to avoid flooding
- **AND** slider position reflects telemetry feedback

#### Scenario: Send throttle command from web UI
- **WHEN** user moves throttle slider
- **THEN** set_throttle command is sent via WebSocket with slider value
- **AND** commands are rate-limited (max 10 Hz) to avoid flooding
- **AND** slider position reflects telemetry feedback

### Requirement: Web Control Safety
The system SHALL enforce safety constraints on web-based control commands.

#### Scenario: Apply rate limiting to web commands
- **WHEN** web client sends control commands rapidly
- **THEN** commands are accepted at maximum 10 Hz (one command per 100ms)
- **AND** excessive commands are dropped or queued
- **AND** warning is sent if rate limit exceeded

#### Scenario: Timeout inactive web control session
- **WHEN** web client is connected but sends no commands for >10 seconds
- **AND** S-bus is also inactive
- **THEN** web control session is considered inactive
- **AND** system switches to FAILSAFE input source
- **AND** fail-safe commands are applied

#### Scenario: Clear web control on client disconnect
- **WHEN** WebSocket client disconnects
- **AND** input source was WEB
- **THEN** web control is released immediately
- **AND** system switches to FAILSAFE input source
- **AND** fail-safe commands are applied
