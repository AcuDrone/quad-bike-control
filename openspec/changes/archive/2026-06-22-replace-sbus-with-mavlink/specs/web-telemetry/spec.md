## MODIFIED Requirements

### Requirement: Real-Time Telemetry Broadcasting
The system SHALL broadcast vehicle telemetry data to connected web clients at 5 Hz.

#### Scenario: Collect MAVLink command and link data
- **WHEN** telemetry collection is triggered
- **AND** the MAVLink command stream is active
- **THEN** the decoded command channel values (steering, throttle, gear, brake, ignition,
  light) are collected from the MAVLink interface
- **AND** MAVLink link metrics are collected (command rate, frames received, time since last
  heartbeat, signal age)
- **AND** MAVLink command/link data is included in the telemetry broadcast

#### Scenario: Collect gear switching state
- **WHEN** telemetry collection is triggered
- **THEN** transmission position control state is checked via `transmissionController.isPositionControlActive()`
- **AND** gear switching boolean is set to true if position control is active (gear change in progress)
- **AND** gear switching boolean is set to false if transmission is stable at target gear

#### Scenario: Format telemetry with MAVLink command and link data
- **WHEN** telemetry data is formatted as JSON
- **THEN** the decoded command values are included (e.g. `"cmd_steering"`, `"cmd_throttle"`,
  `"cmd_brake"`, `"cmd_gear"`)
- **AND** MAVLink link metrics are included: `"mav_cmd_rate"` (Hz), `"mav_link_age"` (ms),
  `"mav_heartbeat_age"` (ms)
- **AND** gear switching state is included: `"gear_switching": true/false`

### Requirement: Telemetry Display on Web Interface
The system SHALL display real-time telemetry on the web interface.

#### Scenario: Display decoded command values
- **WHEN** a telemetry message with MAVLink command data is received
- **THEN** the decoded command channels (steering, throttle, gear, brake) are displayed
- **AND** normalized values are shown (0-100% or -100 to +100%)
- **AND** values update in real-time when the MAVLink command stream is active

#### Scenario: Display MAVLink link status
- **WHEN** a telemetry message with MAVLink link metrics is received
- **THEN** command rate is displayed in Hz with 1 decimal precision
- **AND** time since last heartbeat and command signal age are displayed (human-readable)
- **AND** link indicators use color coding (green: good, yellow: degraded, red: lost/timeout)

#### Scenario: Display CAN bus vehicle data
- **WHEN** telemetry message with CAN data is received
- **AND** CAN status is "connected"
- **THEN** engine RPM is displayed (0-16383 rpm)
- **AND** vehicle speed is displayed (0-255 km/h)
- **AND** coolant temperature is displayed with color coding:
  - Green: <90°C (normal)
  - Yellow: 90-105°C (warm)
  - Red: >105°C (hot/overheating)
- **AND** oil temperature is displayed with color coding:
  - Green: <110°C (normal)
  - Yellow: 110-130°C (warm)
  - Red: >130°C (hot)
- **AND** throttle position is displayed as percentage (0-100%)
- **AND** CAN data age is displayed (time since last update)

#### Scenario: Display CAN disconnected state
- **WHEN** telemetry message has `can_status` != "connected"
- **THEN** CAN card shows "Disconnected" or "No Data" status
- **AND** CAN data values are greyed out or hidden
- **AND** data age shows time since last valid CAN message

#### Scenario: Display gear transition indicator
- **WHEN** telemetry message has `gear_switching` = true
- **THEN** gear transition indicator is shown with animation (e.g., spinner, loading dots)
- **AND** indicator shows target gear being switched to (e.g., "⚙️ Switching to H...")
- **AND** current gear display does not flash neutral during transition
- **WHEN** `gear_switching` = false
- **THEN** gear transition indicator is hidden
- **AND** stable gear is displayed normally

### Requirement: Telemetry Performance
The system SHALL ensure telemetry broadcasting does not degrade control loop performance.

#### Scenario: Maintain 5 Hz telemetry rate with extended data
- **WHEN** telemetry with MAVLink command/link data and CAN data is broadcast
- **THEN** messages are sent every 200ms ±10ms
- **AND** JSON message size remains under 1KB
- **AND** broadcast completes within 5ms for 5 concurrent clients
- **AND** control loop timing remains <10ms average
