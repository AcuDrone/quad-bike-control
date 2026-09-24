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
The system SHALL display real-time telemetry on the web interface. Vehicle speed SHALL be displayed
from the hall-effect speed sensor independently of CAN bus status (it is no longer part of the
CAN-gated vehicle-data display).

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

#### Scenario: Display CAN bus vehicle data
- **WHEN** telemetry message with CAN data is received
- **AND** CAN status is "connected"
- **THEN** engine RPM is displayed (0-16383 rpm)
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

### Requirement: Manifold Absolute Pressure Telemetry
The system SHALL include manifold absolute pressure (MAP) in the CAN vehicle-data telemetry so
the web UI can display it alongside the other engine parameters.

#### Scenario: Include map_kpa in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **AND** CAN status is "connected"
- **THEN** the telemetry JSON SHALL include a `map_kpa` field carrying the manifold absolute
  pressure in kPa (0–255)
- **WHEN** CAN status is not "connected"
- **THEN** the `map_kpa` field SHALL be omitted alongside the other CAN vehicle-data fields

#### Scenario: Display MAP in the CAN telemetry card
- **WHEN** a telemetry message containing `map_kpa` is received
- **THEN** the CAN telemetry card SHALL display the MAP value in kPa
- **AND** the value SHALL update in real time as new telemetry arrives

### Requirement: ECU Probe Results Telemetry
The telemetry broadcast SHALL carry ECU capability probe results to connected web clients while
the results are fresh, so the web UI can render them without a separate request/response channel.

#### Scenario: Include probe object while results are fresh
- **WHEN** an ECU probe is running or has completed within the freshness window
- **THEN** the telemetry JSON SHALL include a `probe` object containing the probe running/complete
  state, the supported-PID bitmaps, a per-candidate-PID list (with supported-by-bitmap, answered,
  raw bytes, and decoded value where known), and a DTC summary (count and codes)
- **AND** the `probe` object SHALL include a truncation indicator when the reported DTC count
  exceeds what a single classic frame can carry

#### Scenario: Omit probe object when stale
- **WHEN** no probe is running and the last completed results are older than the freshness window
- **THEN** the telemetry JSON SHALL omit the `probe` object
- **AND** steady-state telemetry size SHALL be unaffected by the probe feature

#### Scenario: Late-joining client still sees fresh results
- **WHEN** a web client connects after a probe completes but within the freshness window
- **THEN** the next telemetry broadcast SHALL include the `probe` object so the client can render
  the results without re-triggering the probe

### Requirement: Speed-Limit and Limiter-Ceiling Telemetry
The telemetry stream SHALL report the speed limit the limiter is enforcing and how much throttle
authority the limiter is currently withdrawing. Because there is only one possible source for the
limit, no source key SHALL be reported. Road speeds SHALL be carried through the firmware in
metres per second and converted to km/h only at serialisation.

#### Scenario: Report the enforced limit
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_kmh`, the enforced limit in km/h to one decimal
- **AND** the value SHALL be zero whenever there is no usable `SPEED_MAX`, so zero unambiguously
  means "no limit"

#### Scenario: Report the current throttle ceiling
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_ceil`, the throttle ceiling the taper is currently
  applying as a percentage
- **AND** a value of 100 SHALL mean the limiter is withdrawing no authority

#### Scenario: The retired limiter keys are gone
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL NOT include `speed_limit_on`, `speed_limit_max`, `speed_limit_eff`,
  `speed_limit_src` or `mav_speed_max`
- **AND** no web command SHALL exist for setting a local ceiling or toggling the limiter

#### Scenario: Convert to km/h only at the presentation edge
- **WHEN** `vehicle_speed` or `speed_limit_kmh` is serialised
- **THEN** the value SHALL be converted from the firmware's internal metres per second at that
  point
- **AND** `vehicle_speed` SHALL remain a km/h value with one decimal, unchanged on the wire

#### Scenario: Web interface shows the limit and the ceiling
- **WHEN** the web interface renders a telemetry frame
- **THEN** it SHALL display the limit in both km/h and m/s when one is in force, and a "none"
  label when `speed_limit_kmh` is zero
- **AND** the current throttle ceiling SHALL be shown only while the limiter is actually
  withdrawing authority (`speed_limit_ceil` below 100)
- **AND** the card SHALL state that the limit comes only from the autopilot's `SPEED_MAX`, that
  zero means no limit, and that the limiter does not act without a valid sensor reading
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary

### Requirement: ECU Electrical and Air Telemetry
The system SHALL include control module voltage, intake air temperature and calculated engine load
in the CAN vehicle-data telemetry, and the web interface SHALL display them in the CAN telemetry
card, so operators can see charging-system health, intake air conditions and the ECU's own load
estimate alongside the other engine parameters.

#### Scenario: Include ECU voltage, intake temperature and engine load in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **AND** CAN status is "connected"
- **THEN** the telemetry JSON SHALL include an `ecu_voltage` field carrying the control module
  supply voltage in volts with two decimal places
- **AND** SHALL include an `intake_temp` field carrying the intake air temperature in °C
- **AND** SHALL include an `engine_load` field carrying the calculated engine load as a percentage
  (0–100)
- **WHEN** CAN status is not "connected"
- **THEN** all three fields SHALL be omitted alongside the other CAN vehicle-data fields, rather
  than being emitted as zeros

#### Scenario: Populate the fields from CAN vehicle data
- **WHEN** telemetry collection is triggered
- **AND** the CAN `VehicleData` is valid
- **THEN** the values SHALL be copied from the most recent `VehicleData` module voltage, intake
  temperature and engine load readings
- **WHEN** the CAN `VehicleData` is not valid
- **THEN** the fields SHALL be reset to their zero defaults and the CAN-gated block SHALL be
  omitted from the payload

#### Scenario: Display the new values in the CAN telemetry card
- **WHEN** a telemetry message containing `ecu_voltage`, `intake_temp` and `engine_load` is
  received
- **THEN** the CAN telemetry card SHALL display module voltage in volts, intake air temperature in
  °C, and engine load as a percentage
- **AND** the values SHALL update in real time as new telemetry arrives
- **WHEN** `can_status` is not "connected"
- **THEN** the three readouts SHALL show a placeholder (e.g. `-- V`, `--°C`, `--%`) rather than a
  stale or misleading value

#### Scenario: New UI labels have full i18n parity
- **WHEN** the new CAN-card labels are added to the web UI
- **THEN** each label SHALL use a `data-i18n` key
- **AND** every new key SHALL be present in BOTH the `en` and `uk` dictionaries with maintained
  parity (no key defined in only one dictionary)

#### Scenario: Additions stay within the telemetry size budget
- **WHEN** the three fields are added to the steady-state telemetry payload
- **THEN** the serialized JSON document SHALL remain within the configured
  `StaticJsonDocument` capacity, including the peak case where the transient ECU probe object is
  also present
- **AND** the steady-state payload growth SHALL be bounded by the three scalar fields (no arrays
  or nested objects are introduced)
- **AND** the 5 Hz broadcast cadence SHALL be unaffected

### Requirement: Odometer and Trip Telemetry
The telemetry stream SHALL report the total odometer and the trip distance, and the web interface
SHALL display both, so that the distance the vehicle has driven is readable from the portal as well
as from the ground station. The web portal SHALL be **display only**: it SHALL NOT offer any control
that resets the trip or the odometer, because the reset is a ground-station action.

#### Scenario: Include the distance counters in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **THEN** the JSON payload SHALL include `odo_km` (total distance) and `trip_km` (resettable trip
  distance), both in kilometres with 3 decimal places
- **AND** both fields SHALL be emitted unconditionally, independent of CAN status, MAVLink status
  and the speed sensor's own validity flag, because distance already driven does not stop being
  true when a bus goes quiet
- **AND** the two additional keys SHALL keep the total telemetry message under 1 KB

#### Scenario: Populate the distance counters from the speed sensor
- **WHEN** telemetry collection is triggered
- **THEN** `odo_km` and `trip_km` SHALL be read from the hall speed sensor's distance counters via
  the vehicle controller, alongside the existing `vehicle_speed` and `speed_valid` collection
- **AND** the values SHALL be the exact millimetre counters scaled to kilometres at this point, not
  a separately maintained running total

#### Scenario: Display both counters in the speed section
- **WHEN** a telemetry message containing `odo_km` and `trip_km` is received
- **THEN** the web UI SHALL display both in the speed section, in kilometres to 3 decimal places
- **AND** they SHALL NOT be greyed out or blanked when `speed_valid` is false — unlike the live
  speed reading, a recorded distance is not invalidated by the sensor going quiet
- **AND** a missing field SHALL render as a placeholder rather than as `0.000` or a JavaScript error
- **AND** every new label SHALL have an i18n key present in BOTH the `en` and the `uk` dictionaries,
  with parity maintained

#### Scenario: The portal offers no reset control
- **WHEN** the web interface renders the speed section
- **THEN** it SHALL present the odometer and trip values as read-only text
- **AND** there SHALL be NO button, input or web command that resets the trip or the odometer
- **AND** no `WebPortal::WebCommand` SHALL exist for either counter

