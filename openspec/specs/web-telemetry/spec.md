# web-telemetry Specification

## Purpose
TBD - created by archiving change add-web-portal. Update Purpose after archive.
## Requirements
### Requirement: Real-Time Telemetry Broadcasting
The system SHALL broadcast vehicle telemetry data to connected web clients at 5 Hz.

#### Scenario: Collect telemetry data
- **WHEN** telemetry collection is triggered (every 200ms in control loop)
- **THEN** current vehicle state is gathered including:
  - Current gear (R/N/L/H)
  - Transmission hall sensor position (encoder counts)
  - Brake position (percentage, 0-100%)
  - Throttle servo position (angle in degrees)
  - Steering servo position (angle in degrees)
  - Input source (SBUS/WEB/FAILSAFE)
  - S-bus signal status (active/inactive)
  - System timestamp (milliseconds since boot)

#### Scenario: Format telemetry as JSON
- **WHEN** telemetry data is collected
- **THEN** data is formatted as JSON object with fields:
  - `timestamp` (uint32_t milliseconds)
  - `gear` (string: "R"/"N"/"L"/"H")
  - `hall_position` (int32_t encoder counts)
  - `brake_pct` (float percentage)
  - `throttle_angle` (int degrees)
  - `steering_angle` (int degrees)
  - `input_source` (string: "SBUS"/"WEB"/"FAILSAFE")
  - `sbus_active` (boolean)

#### Scenario: Broadcast telemetry to all clients
- **WHEN** telemetry JSON is formatted
- **THEN** JSON message is sent to all connected WebSocket clients
- **AND** message is broadcast simultaneously (no per-client delay)
- **AND** broadcast completes within 5ms

#### Scenario: Handle client disconnection during broadcast
- **WHEN** WebSocket send fails to disconnected client
- **THEN** error is caught and logged
- **AND** broadcast continues to remaining clients
- **AND** disconnected client is removed from active list

### Requirement: Telemetry Display on Web Interface
The system SHALL display real-time telemetry on the web interface.

#### Scenario: Display current gear selection
- **WHEN** telemetry message is received by web client
- **THEN** gear indicator is updated to display current gear (R/N/L/H)
- **AND** visual highlighting shows active gear
- **AND** update is applied within 50ms of receiving message

#### Scenario: Display transmission hall sensor position
- **WHEN** telemetry message is received
- **THEN** hall sensor position is displayed in encoder counts
- **AND** position value is updated with each telemetry message

#### Scenario: Display brake status
- **WHEN** telemetry message is received
- **THEN** brake position percentage is displayed (0-100%)
- **AND** visual indicator shows brake applied/released state
- **AND** indicator changes color based on brake level (green: <10%, yellow: 10-50%, red: >50%)

#### Scenario: Display throttle position
- **WHEN** telemetry message is received
- **THEN** throttle servo position is displayed in degrees or percentage
- **AND** visual bar or slider shows throttle level

#### Scenario: Display steering position
- **WHEN** telemetry message is received
- **THEN** steering servo position is displayed in degrees or percentage
- **AND** visual indicator shows steering direction (left/center/right)

#### Scenario: Display input source indicator
- **WHEN** telemetry message is received
- **THEN** input source is displayed (SBUS/WEB/FAILSAFE)
- **AND** indicator color reflects source (blue: SBUS, green: WEB, red: FAILSAFE)
- **AND** S-bus active status is shown separately

### Requirement: Telemetry Performance
The system SHALL ensure telemetry broadcasting does not degrade control loop performance.

#### Scenario: Maintain 5 Hz telemetry rate
- **WHEN** telemetry broadcasting is active
- **THEN** telemetry messages are sent every 200ms ±10ms
- **AND** rate remains stable with 1-5 connected clients

#### Scenario: Limit telemetry impact on control loop
- **WHEN** telemetry is being collected and broadcast
- **THEN** control loop timing remains <10ms average
- **AND** telemetry collection takes <2ms
- **AND** WebSocket broadcast takes <5ms

