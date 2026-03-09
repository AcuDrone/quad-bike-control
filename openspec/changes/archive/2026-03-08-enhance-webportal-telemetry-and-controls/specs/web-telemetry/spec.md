# web-telemetry Spec Delta

## MODIFIED Requirements

### Requirement: Real-Time Telemetry Broadcasting
The system SHALL broadcast vehicle telemetry data to connected web clients at 5 Hz.

#### Scenario: Collect SBUS channel data
- **WHEN** telemetry collection is triggered
- **AND** SBUS signal is active
- **THEN** all 16 SBUS channel values (raw microseconds) are collected via `sbusInput.getRawChannels()`
- **AND** SBUS signal quality metrics are collected (frame rate, error rate, signal age)
- **AND** SBUS data is included in telemetry broadcast

#### Scenario: Collect gear switching state
- **WHEN** telemetry collection is triggered
- **THEN** transmission position control state is checked via `transmissionController.isPositionControlActive()`
- **AND** gear switching boolean is set to true if position control is active (gear change in progress)
- **AND** gear switching boolean is set to false if transmission is stable at target gear

#### Scenario: Format telemetry with SBUS channels
- **WHEN** telemetry data is formatted as JSON
- **THEN** SBUS channel array is included: `"sbus_channels": [ch1, ch2, ..., ch16]` (microseconds)
- **AND** SBUS quality metrics are included: `"sbus_frame_rate"` (Hz), `"sbus_error_rate"` (%), `"sbus_signal_age"` (ms)
- **AND** gear switching state is included: `"gear_switching": true/false`

### Requirement: Telemetry Display on Web Interface
The system SHALL display real-time telemetry on the web interface.

#### Scenario: Display SBUS channel values
- **WHEN** telemetry message with SBUS data is received
- **THEN** all 16 SBUS channels are displayed showing raw value in microseconds
- **AND** key mapped channels (Ch1-4: steering, throttle, gear, brake) are highlighted or labeled
- **AND** normalized percentage is shown for key channels (0-100% or -100 to +100%)
- **AND** SBUS values update in real-time when signal is active

#### Scenario: Display SBUS signal quality
- **WHEN** telemetry message with SBUS quality is received
- **THEN** frame rate is displayed in Hz with 1 decimal precision
- **AND** error rate is displayed as percentage
- **AND** signal age is displayed in milliseconds or seconds (human-readable)
- **AND** quality indicators use color coding (green: good, yellow: degraded, red: poor/timeout)

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
- **WHEN** telemetry with SBUS channels and CAN data is broadcast
- **THEN** messages are sent every 200ms ±10ms
- **AND** JSON message size remains under 1KB
- **AND** broadcast completes within 5ms for 5 concurrent clients
- **AND** control loop timing remains <10ms average
