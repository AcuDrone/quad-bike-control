## ADDED Requirements
### Requirement: Tacho and RPM Source Telemetry
The telemetry JSON SHALL carry the tacho state and the effective RPM source, outside the CAN-gated
block, so that the web console can show and calibrate the tacho whatever the CAN health.

#### Scenario: Emit tacho fields
- **WHEN** a telemetry update is broadcast
- **THEN** the JSON SHALL include `tacho_hz` (float, averaged edge frequency, 0 when timed out),
  `tacho_ppr` (float, the ppr in effect), `tacho_rpm` (integer, the tacho's own RPM, 0 when timed out)
  and `rpm_source` ("TACHO", "CAN" or "NONE")
- **AND** these fields SHALL be present whether or not `can_status` is "connected"

#### Scenario: Emit tacho signal diagnostics
- **WHEN** a telemetry update is broadcast
- **THEN** the JSON SHALL include `tacho_periods` (string: the last 8 accepted periods in ms with one
  decimal, space separated, oldest first, e.g. "42.9 43.1 21.4 21.6 42.8"; "" before the first
  period), `tacho_min_ms` and `tacho_max_ms` (float, shortest / longest accepted period in the last
  1 s window, 0 when none), `tacho_acc` and `tacho_rej` (integer, accepted / rejected edges per
  second over that window) and `tacho_rej_total` (integer, rejected edges since boot)
- **AND** these fields SHALL be present whether or not `can_status` is "connected"
- **AND** the tacho card SHALL show them in a monospace "Діагностика" / "Diagnostics" block under its
  tiles (uk / en)

#### Scenario: engine_rpm carries the effective RPM
- **WHEN** a telemetry update is broadcast
- **THEN** `engine_rpm` SHALL be the effective engine RPM and SHALL be emitted outside the
  `can_status == "connected"` block
- **AND** it SHALL be 0 when `rpm_source` is "NONE"

## MODIFIED Requirements
### Requirement: Telemetry Display on Web Interface
The system SHALL display real-time telemetry on the web interface. Vehicle speed SHALL be displayed
from the hall-effect speed sensor independently of CAN bus status (it is no longer part of the
CAN-gated vehicle-data display). Engine RPM SHALL likewise be displayed outside the CAN gate,
because on aztec it is sourced from the ECU tachometer output rather than from CAN.

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

#### Scenario: Display vehicle speed from hall sensor
- **WHEN** a telemetry message containing `vehicle_speed` is received
- **THEN** the vehicle speed is displayed (0-255 km/h) regardless of `can_status`
- **AND** when the accompanying `speed_valid` flag is false the display indicates the reading is
  unavailable/unhealthy rather than showing a misleading 0
- **AND** the value updates in real-time as new telemetry arrives

#### Scenario: Display effective engine RPM with its source
- **WHEN** a telemetry message is received
- **THEN** the EXISTING engine RPM tile SHALL show `engine_rpm` (the effective RPM, 0-16383 rpm)
  regardless of `can_status`
- **AND** a small badge next to the value SHALL show `rpm_source`: "TACHO" or "CAN"; when
  `rpm_source` is "NONE" the tile SHALL show "—" instead of a misleading 0
- **AND** no second RPM tile SHALL be added

#### Scenario: Display CAN bus vehicle data
- **WHEN** telemetry message with CAN data is received
- **AND** CAN status is "connected"
- **THEN** coolant temperature is displayed with color coding:
  - Green: <90°C (normal)
  - Yellow: 90-105°C (warm)
  - Red: >105°C (hot/overheating)
- **AND** oil temperature is displayed with color coding:
  - Green: <110°C (normal)
  - Yellow: 110-130°C (warm)
  - Red: >130°C (hot)
- **AND** throttle position is displayed as percentage (0-100%)
- **AND** CAN data age is displayed (time since last update)
- **AND** vehicle speed is NOT part of this CAN-gated block (it is displayed independently from the
  hall sensor)

#### Scenario: Display CAN disconnected state
- **WHEN** telemetry message has `can_status` != "connected"
- **THEN** CAN card shows "Disconnected" or "No Data" status
- **AND** CAN data values are greyed out or hidden
- **AND** data age shows time since last valid CAN message
- **AND** the hall-sensor vehicle speed display remains active and unaffected

#### Scenario: Display gear transition indicator
- **WHEN** telemetry message has `gear_switching` = true
- **THEN** gear transition indicator is shown with animation (e.g., spinner, loading dots)
- **AND** indicator shows target gear being switched to (e.g., "⚙️ Switching to H...")
- **AND** current gear display does not flash neutral during transition
- **WHEN** `gear_switching` = false
- **THEN** gear transition indicator is hidden
- **AND** stable gear is displayed normally
