# web-telemetry Specification

## Purpose
TBD - created by archiving change add-web-portal. Update Purpose after archive.
## Requirements
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

### Requirement: Firmware Version Display
The system SHALL display the firmware version in the web portal interface.

#### Scenario: Include firmware version in telemetry broadcast
- **WHEN** telemetry data is collected for broadcast
- **THEN** the firmware version string is included from the `FIRMWARE_VERSION` constant defined in Constants.h
- **AND** the version is added to the telemetry struct as `firmware_version` field
- **AND** the version is serialized to JSON as `"firmware_version": "<version>"`

#### Scenario: Display firmware version in web UI
- **WHEN** the web portal receives telemetry data via WebSocket
- **AND** the telemetry message contains a `firmware_version` field
- **THEN** the firmware version is displayed in the status bar
- **AND** the version display uses the existing `.status-item` CSS pattern
- **AND** the version display has the format: "Firmware: X.X.X"
- **AND** the version is visible without scrolling (always in status bar)

#### Scenario: Handle missing firmware version gracefully
- **WHEN** the web portal connects but version data is not yet received
- **THEN** the firmware version display shows "Loading..." as placeholder text
- **WHEN** the firmware version field is missing from telemetry
- **THEN** the display shows "N/A" or retains "Loading..." state
- **AND** no JavaScript errors are thrown

#### Scenario: Firmware version constant is centrally defined
- **WHEN** developers need to update the firmware version
- **THEN** the version is defined as `FIRMWARE_VERSION` constant in `include/Constants.h`
- **AND** the constant uses semantic versioning format (e.g., "1.0.0")
- **AND** updating the constant automatically propagates to web portal display

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

### Requirement: Steering Motor Telemetry
The telemetry broadcast SHALL include steering-motor health data sourced from the VESC, and the web interface SHALL display it, so operators can see steering motor current, temperature, and driver health.

#### Scenario: Include steering VESC data in telemetry JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** the payload SHALL include `steer_motor_current` (motor current in amps), `steer_fet_temp` (VESC FET temperature in °C), `steer_vesc_fault` (fault indicator), and `steer_driver_ok` (boolean VESC-link health)
- **AND** the additions SHALL keep the total telemetry message under 1 KB

#### Scenario: Populate steering VESC data from the driver
- **WHEN** telemetry collection is triggered
- **THEN** the steering motor current, FET temperature, fault code, and driver-ok flag SHALL be read from the VESC steering driver's most recent `COMM_GET_VALUES` poll

#### Scenario: Display steering motor telemetry in the web UI
- **WHEN** a telemetry message with steering VESC fields is received
- **THEN** the web UI SHALL display steering motor current (A) and FET temperature (°C)
- **AND** SHALL show a fault / driver-ok indicator (e.g. green when `steer_driver_ok` is true and no fault, red on fault or driver down)
- **AND** all new UI labels SHALL have i18n keys present in both the `en` and `uk` dictionaries with maintained parity

### Requirement: 24V Rail Telemetry
The system SHALL include the 24V boost rail state in the telemetry broadcast so the rail powering
the steering VESC, Jetson, Starlink and cameras is observable from the web interface.

#### Scenario: Collect rail data
- **WHEN** telemetry collection is triggered
- **THEN** the latest averaged 24V rail voltage SHALL be collected
- **AND** the commanded boost enable state SHALL be collected
- **AND** the low-rail warning flag SHALL be collected

#### Scenario: Format rail data as JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** `"rail_24v"` SHALL be included as a float with 1 decimal place
- **AND** `"boost_on"` SHALL be included as a boolean
- **AND** `"rail_low"` SHALL be included as a boolean
- **AND** these fields SHALL be emitted unconditionally, independent of CAN or MAVLink status

#### Scenario: Display rail voltage on the web interface
- **WHEN** a telemetry message with `rail_24v` is received
- **THEN** the rail voltage SHALL be displayed in volts with 1 decimal place
- **AND** the reading SHALL be colour-coded: normal when at or above the low threshold, warning when
  `rail_low` is true
- **AND** the boost enable state SHALL be shown alongside it, so a deliberately disabled boost is
  distinguishable from a failed rail
- **AND** labels SHALL have i18n keys present in both the `en` and `uk` dictionaries

### Requirement: Board I/O Health Telemetry
The system SHALL report the health of the two I2C port expanders in telemetry, because the gear
interlock, the brake endstop and the ignition/starter path now depend on them.

#### Scenario: Collect expander health
- **WHEN** telemetry collection is triggered
- **THEN** the opto-input expander fault state SHALL be collected from `BoardInputs::isFaulted()`
- **AND** the relay expander fault state SHALL be collected from `RelayController::isFaulted()`

#### Scenario: Format expander health as JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** `"io_input_fault"` and `"io_relay_fault"` SHALL be included as booleans
- **AND** they SHALL be emitted unconditionally, independent of CAN or MAVLink status

#### Scenario: Display expander faults on the web interface
- **WHEN** a telemetry message reports `io_input_fault` or `io_relay_fault` as true
- **THEN** a visible fault indicator SHALL be shown identifying which expander is faulted
- **AND** the indicator SHALL clear automatically when the flag returns to false
- **AND** labels SHALL have i18n keys present in both the `en` and `uk` dictionaries

#### Scenario: Distinguish a fault from a stale telemetry stream
- **WHEN** the input expander is faulted but the control loop is healthy
- **THEN** telemetry SHALL continue to broadcast at the normal 5 Hz rate with the fault flag set
- **AND** gear SHALL be reported as unknown rather than as a last-known gear presented as current

