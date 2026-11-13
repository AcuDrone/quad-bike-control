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
The system SHALL display the firmware version and the target board name in the web portal interface.

#### Scenario: Include firmware version in telemetry broadcast
- **WHEN** telemetry data is collected for broadcast
- **THEN** the firmware version string is included from the `FIRMWARE_VERSION` constant defined in Constants.h
- **AND** the board name is included from the `BOARD_NAME` constant defined in Constants.h
- **AND** both are serialized to JSON as `"firmware_version": "<semver>"` and `"board": "<name>"`

#### Scenario: Display firmware version in web UI
- **WHEN** the web portal receives telemetry data via WebSocket
- **AND** the telemetry message contains a `firmware_version` field
- **THEN** the firmware status item at the bottom of the sidebar shows `Firmware: <version> · <board>`, next to the WebSocket status item
- **AND** when `board` is absent only the version is shown
- **AND** on wide screens the item is visible in every section without scrolling (the sidebar stays in view)
- **AND** at 768 px or less the item is shown in the navigation opened by the ☰ button

#### Scenario: Handle missing firmware version gracefully
- **WHEN** the web portal connects but version data is not yet received
- **THEN** the firmware version display shows "Loading..." as placeholder text
- **WHEN** the firmware version field is missing from telemetry
- **THEN** the display retains its previous state
- **AND** no JavaScript errors are thrown

#### Scenario: Firmware version constant is centrally defined
- **WHEN** developers release new firmware
- **THEN** `FIRMWARE_VERSION` and `BOARD_NAME` are defined in `include/Constants.h`
- **AND** each hardware branch defines its own `BOARD_NAME` value

### Requirement: Gear Default Positions in Telemetry
The telemetry broadcast SHALL include the current effective default positions for all four gears so the web UI can show them without a separate request.

#### Scenario: Include gear_defaults in telemetry JSON
- **WHEN** the system broadcasts a telemetry update
- **THEN** the JSON payload SHALL include a `gear_defaults` object with keys `R`, `N`, `L`, `H` containing the current default transmission servo position for each gear (0.0–100.0 %)
- **AND** the values SHALL reflect the active NVS defaults (or factory constants when no NVS defaults are set)

#### Scenario: Web UI populates defaults editor from telemetry
- **WHEN** a telemetry message containing `gear_defaults` is received
- **THEN** the web UI SHALL show each gear's value on the second line of its "Зберегти як" / "Save as" button in the Gear position defaults card
- **AND** the "▶ <gear>" buttons SHALL use these values as their targets
- **AND** the active-gear highlight SHALL be recomputed against the current slider value

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
- **AND** the help of the "Обмеження швидкості та кермо" / "Speed limit and steering" card (its ⓘ
  button) SHALL state that the limit comes only from the autopilot's `SPEED_MAX`, that zero means
  no limit, and that the limiter does not act without a valid sensor reading
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

### Requirement: Engine Hours Telemetry
The telemetry stream SHALL report BOTH engine hour counters — the total and the resettable TRIP
hours — and the web interface SHALL display both, so that the accumulated running time is readable
from the portal as well as from the ground station. The web portal SHALL be **display only**: it
SHALL NOT offer any control that resets either counter — the total has no reset on any interface,
and the trip hours are cleared only by the ground station's existing trip-reset command.

#### Scenario: Include the engine hour meter in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **THEN** the JSON payload SHALL include `engine_hours` and `engine_trip_hours`, each in hours
  with 2 decimal places
- **AND** both fields SHALL be emitted unconditionally, independent of CAN status, MAVLink status
  and the engine's current state, because time already run does not stop being true when a bus goes
  quiet
- **AND** a zero `engine_trip_hours` SHALL be emitted as a genuine `0.00`, not omitted or replaced
  by a placeholder, because zero is what the counter reads immediately after a trip reset
- **AND** the additional keys SHALL keep the total telemetry message under 1 KB

#### Scenario: Populate the engine hour meter from the vehicle layer
- **WHEN** telemetry collection is triggered
- **THEN** `engine_hours` and `engine_trip_hours` SHALL be read from the vehicle controller's
  engine hour meter, alongside the existing engine and distance collection
- **AND** each value SHALL be the exact seconds counter scaled to hours at this point, not a
  separately maintained running total

#### Scenario: Display the engine hour meter with the engine data
- **WHEN** a telemetry message containing `engine_hours` and `engine_trip_hours` is received
- **THEN** the web UI SHALL display both with the engine/CAN data, in hours to 2 decimal places,
  the TRIP hours directly beneath the total
- **AND** neither SHALL be greyed out or blanked when the CAN bus is disconnected — unlike the live
  engine readings, accumulated running time is not invalidated by the bus going quiet
- **AND** a missing field SHALL render as a placeholder rather than as `0.00` or a JavaScript error,
  the placeholder being reserved for an ABSENT field so that a displayed `0.00` always means a
  genuine zero
- **AND** every new label SHALL have an i18n key present in BOTH the `en` and the `uk` dictionaries,
  with parity maintained

#### Scenario: The portal offers no reset control
- **WHEN** the web interface renders the engine data
- **THEN** it SHALL present BOTH engine hour counters as read-only text
- **AND** there SHALL be NO button, input or web command that resets either of them
- **AND** no `WebPortal::WebCommand` SHALL exist for either
- **AND** clearing the trip hours SHALL remain a GROUND-STATION action, performed by the same
  command that clears the trip distance, on the same terms as the trip distance itself

### Requirement: Steering Speed-Scaling Telemetry
The telemetry stream SHALL report the steering speed-scaling factor being applied, the base value
that produced it, where that base came from, and whether a bench test speed is overriding the
measured speed — so that "the steering feels weak" can be diagnosed from the portal without a
serial console.

#### Scenario: Report the applied scale
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `steer_scale`, the scale actually applied after rate limiting, in the
  range 0 to 1 with two decimals
- **AND** a value of 1 SHALL mean no authority is being withdrawn

#### Scenario: Report the base and its source
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `steer_sca_base`, the base in use in metres per second, zero when
  there is none
- **AND** it SHALL include `steer_sca_src`, naming the source of that base as the locally stored
  value, the autopilot's `MOT_SPD_SCA_BASE`, or none

#### Scenario: Report a live test-speed override
- **WHEN** a telemetry frame is produced while a `set_test_speed` override is in force
- **THEN** it SHALL include `steer_test_speed`, the overriding speed in metres per second
- **AND** the field SHALL be zero or absent when no override is in force, so a stale bench setting
  cannot be mistaken for a real reading

#### Scenario: Web interface shows why the steering is being scaled
- **WHEN** the web interface renders a telemetry frame
- **THEN** it SHALL display the applied scale, and alongside it the base and its source
- **AND** it SHALL state when the scaling is inactive and why — no base, an invalid speed reading,
  or the autopilot not being in MANUAL
- **AND** a live test-speed override SHALL be shown as a temporary bench state, not as a normal
  reading
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary

### Requirement: Firmware Changelog
The system SHALL ship a user-facing changelog inside the firmware binary and expose it to the web portal.

#### Scenario: Changelog is served by the firmware
- **WHEN** a client requests `GET /api/changelog`
- **THEN** the portal responds `200 application/json` with the document from `include/Changelog.h`
- **AND** the document is an object with a `releases` array ordered newest first
- **AND** each release has `version`, `date`, and `uk` / `en` arrays of plain-language strings

#### Scenario: Changelog is viewable from the web UI
- **WHEN** the user activates the firmware status item or the "What's new" button
- **THEN** a modal opens and fetches `/api/changelog` (at most once per page load)
- **AND** every release is listed with its version and date and the bullets of the active language
- **AND** the release matching the running `firmware_version` is marked as current
- **AND** switching the UI language re-renders the open modal in the other language

#### Scenario: Changelog fetch fails
- **WHEN** `/api/changelog` is unreachable or returns invalid JSON
- **THEN** the modal shows a localized error message instead of an empty list
- **AND** no JavaScript errors are thrown

### Requirement: Relay Test Telemetry
Web telemetry SHALL include `relay_test`, an unsigned integer with one bit per relay in
relay-number order (bit0 = Relay 1 … bit3 = Relay 4 on the DevKitC-1), set for each relay currently
energized by the web relay test. It SHALL be derived from the relay output shadow byte (the last
written GPIO state), limited to the test bits, so a vehicle function relay is never reported as a
test relay. The existing function fields (`ignition_state`, `is_cranking`, `front_light_on`,
`wheel_lock_on`, `wheel_lock_fitted`) SHALL be unchanged. There is no relay-fault field on this board
(direct GPIO).

#### Scenario: Test relay reported
- **WHEN** Relay 3 (GPIO38) is test-energized
- **THEN** telemetry SHALL report `relay_test` = 4 (bit2)

#### Scenario: No test active
- **WHEN** no relay is test-energized
- **THEN** telemetry SHALL report `relay_test` = 0

#### Scenario: Auto-off or forced-off is visible
- **WHEN** a test relay times out or is forced off (function command, fail-safe)
- **THEN** the next telemetry broadcast SHALL report its bit as 0

### Requirement: Steering Angle Sensor Telemetry
The telemetry broadcast SHALL report the Bosch LWS steering angle, its status and its error counters, together with the stored limits and the zero-set progress, and the web interface SHALL display them in a read-only "Кермо: стан" / "Steering: state" card in the Diagnostics section, so that calibration and sensor faults can be diagnosed from the portal.

#### Scenario: Angle and limits
- **WHEN** telemetry data is formatted as JSON
- **THEN** `steer_angle` SHALL carry the current steering angle in degrees with one decimal, positive = right, or `null` unless a fresh accepted frame carries a valid angle
- **AND** `steer_rate` SHALL carry the signed rotation rate in °/s (positive = turning right), or `null` when the speed is invalid or no fresh frame exists
- **AND** `steer_left` and `steer_right` SHALL carry the stored limits in degrees, or `null` when unset
- **AND** `steer_calibrated` SHALL be `true` only when both limits are stored and valid
- **AND** `steer_sensor_inv` SHALL be `true` while the stored sensor-direction flag says the sensor is mounted mirrored, otherwise `false`, and SHALL always be emitted (it does not depend on a fresh frame)
- **AND** `steer_angle` and `steer_rate` SHALL already carry the firmware sign under that flag
- **AND** `steer_range_warning` SHALL be `true` while either stored limit is beyond ±`STEER_LWS_SAFE_RANGE` (550.0°), otherwise `false`
- **AND** `steering_pct` SHALL keep its percent-of-calibrated-travel meaning

#### Scenario: Sensor status and counters
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `steer_lws_ok`, `steer_lws_cal` and `steer_lws_adj` (booleans) and `steer_lws_fault_code` (0–15) from the last accepted frame, each `null` until a frame has been accepted
- **AND** it SHALL include `steer_sensor_fault`, one of `none`, `zeroing`, `can_down`, `timeout`, `range_latched`, `sensor_fault`, `not_calibrated` or `angle_invalid`, which SHALL NOT be `none` while `steer_sensor_ok` is `false`
- **AND** it SHALL include `steer_lws_fps` (accepted frames in the last second) and the cumulative counters `steer_lws_crc_err`, `steer_lws_cnt_err` and `steer_lws_miss`
- **AND** it SHALL include `steer_zero_state` (`idle`, `running`, `ok` or `failed`) and `steer_zero_error` (a reason token, or `null`)
- **AND** the fields `steer_position`, `steer_center`, `steer_fine_raw`, `steer_coarse_raw`, `steer_turn`, `steer_ratio`, `steer_residual`, `steer_cal_points`, `steer_cal_error` and `steer_cal_lost` SHALL NOT be emitted

#### Scenario: Calibration card shows the sensor readout
- **WHEN** the web UI receives steering telemetry
- **THEN** the "Кермо: стан" card SHALL show a 2-column tile grid: Кут / Angle, Швидкість / Rate (°/s), Межа Л / Limit L, Межа П / Limit R, Датчик / Sensor, Режим / Mode, Калібрування / Calibration, Драйвер VESC / VESC driver, Нуль датчика / Sensor zero, Струм мотора / Motor current, Темп. FET / FET temp and Напруга VESC / VESC voltage, with `--` for `null` or missing values
- **AND** while `steer_range_warning` is true, a limit beyond ±550° SHALL carry a short ⚠ note in its tile and a full-width note SHALL explain that the sensor latches a fault at about ±600°
- **AND** the Sensor tile SHALL show the `steer_sensor_fault` token as a localized label (en/uk); for `range_latched` the label SHALL say to restart the vehicle
- **AND** while `steer_sensor_inv` is `true` the Sensor tile SHALL also show a "дзеркально" / "mirrored" marker, and none while it is `false` or missing
- **AND** the Sensor zero tile SHALL show the zero-set progress and any failure reason
- **AND** the Mode tile SHALL show `steer_mode` as described in "Steering Control Mode Telemetry"
- **AND** below the tiles one full-width, muted, monospace line SHALL show the raw LWS status bits (OK, cal, adj), the fault code, the frame rate and the checksum, counter and missed-frame counters

#### Scenario: Compact readout in the calibration card
- **WHEN** the Calibration section is shown
- **THEN** the Steering calibration card SHALL show only the angle, the Left/Right limits with their ⚠ notes and one sensor/calibration status line, rendered from the same telemetry frame as the "Кермо: стан" card
- **AND** the status line SHALL include the "дзеркально" / "mirrored" marker while `steer_sensor_inv` is `true`

#### Scenario: Sensor direction help text
- **WHEN** the engineer opens the ⓘ help of the "Кермо: стан" card
- **THEN** its help text (en/uk) SHALL explain the "mirrored" marker: the sensor direction is detected at the first limit capture, and angles are always shown with + = right

### Requirement: Diagnostics Attention Summary
The Diagnostics section SHALL start with an "Увага" / "Attention" box that lists the problems
active in the latest telemetry frame, derived only from existing telemetry fields, and SHALL be
hidden when no problem is active.

#### Scenario: Sensorless steering
- **WHEN** a frame has `steer_sensor_ok` false
- **THEN** the box SHALL list "steering sensor unhealthy, jog/nudge run open loop without stall
  protection" in the danger colour

#### Scenario: Steering in rate mode
- **WHEN** a frame has `steer_mode` equal to `rate`
- **THEN** the box SHALL list "Кермо без датчика: режим швидкості" / "Steering without sensor:
  rate mode" in the danger colour, in addition to the sensorless steering line
- **AND** the line SHALL disappear on the first frame whose `steer_mode` is not `rate`

#### Scenario: Steering VESC down or faulted
- **WHEN** a frame has `steer_driver_ok` false
- **THEN** the box SHALL list "steering driver down" in the danger colour
- **WHEN** `steer_driver_ok` is true and `steer_vesc_fault` is nonzero
- **THEN** the box SHALL list a VESC fault line with the fault code in the danger colour

#### Scenario: Steering limit beyond the safe range
- **WHEN** a frame has `steer_range_warning` true
- **THEN** the box SHALL list "a steering limit is beyond ±550°" in the warning colour

#### Scenario: MAVLink inactive
- **WHEN** a frame has `mav_active` false
- **THEN** the box SHALL list "MAVLink inactive, autopilot commands are not arriving" in the warning
  colour

#### Scenario: Low steering VESC input voltage
- **WHEN** a frame has `steer_vin_low` true and `steer_driver_ok` true
- **THEN** the box SHALL list "Напруга VESC низька (xx.x В)" / "VESC voltage low (xx.x V)" with the
  value of `steer_vin`, in the warning colour
- **AND** while `steer_driver_ok` is false no voltage line SHALL be listed, because there is no
  valid VESC data

#### Scenario: Low 12 V ECU voltage
- **WHEN** a frame has `can_status` equal to `connected` and `ecu_voltage_low` true
- **THEN** the box SHALL list "Напруга 12 В низька (xx.x В)" / "12 V voltage low (xx.x V)" with the
  value of `ecu_voltage`, in the warning colour

#### Scenario: No ECU link
- **WHEN** a frame has `can_status` not equal to `connected`
- **THEN** the box SHALL list "Немає зв'язку з ЕБК" / "No ECU link" in the warning colour
- **AND** no 12 V voltage line SHALL be listed

#### Scenario: Lines are localized text
- **WHEN** any line is listed
- **THEN** it SHALL be localized (en/uk) and inserted as text, not HTML

#### Scenario: Nothing to report
- **WHEN** none of the conditions is active
- **THEN** the box SHALL be hidden

#### Scenario: Condition clears
- **WHEN** a condition that was listed is no longer active in a later frame
- **THEN** its line SHALL disappear on that frame without a page reload

### Requirement: Steering VESC Input Voltage Telemetry
The telemetry broadcast SHALL include the steering VESC's measured input voltage and a low-voltage
flag computed in the firmware against the compile-time constant `STEER_VIN_LOW_V` (default 21.5 V,
chosen to suit both a 6s NMC and a 7s LiFePO4 steering pack and adjustable per build), and SHALL
include a 12 V low-voltage flag computed against `ECU_VOLTAGE_LOW_V` (default 12.0 V, lead-acid starter battery), so the web
page never needs a copy of either threshold.

#### Scenario: Include the VESC input voltage
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `steer_vin`, the input voltage from `SteeringController::getInputVoltage()`
  (the VESC `COMM_GET_VALUES` input voltage) in volts with one decimal
- **AND** it SHALL include `steer_vin_low`, true only when `steer_driver_ok` is true and `steer_vin`
  is below `STEER_VIN_LOW_V`

#### Scenario: Include the 12 V low flag
- **WHEN** telemetry data is formatted as JSON and CAN status is "connected"
- **THEN** it SHALL include `ecu_voltage_low`, true when the ECU module voltage is below
  `ECU_VOLTAGE_LOW_V`
- **WHEN** CAN status is not "connected"
- **THEN** `ecu_voltage_low` SHALL be omitted along with the other CAN vehicle-data fields

#### Scenario: Show the VESC voltage tile
- **WHEN** the "Кермо: стан" card renders a telemetry frame
- **THEN** a "Напруга VESC" / "VESC voltage" tile SHALL show `steer_vin` in volts
- **AND** it SHALL show `--` while `steer_driver_ok` is false or the key is missing
- **AND** it SHALL use the warning colour while `steer_vin_low` is true

#### Scenario: Thresholds are firmware constants
- **WHEN** the firmware is built
- **THEN** `STEER_VIN_LOW_V` and `ECU_VOLTAGE_LOW_V` SHALL be defined in `include/Constants.h`
- **AND** `data/index.html` SHALL NOT contain either threshold value

### Requirement: Speed State Card
The Diagnostics section SHALL contain a read-only "Швидкість" / "Speed" card, separate from the
two speed settings cards in the Calibration section ("Датчик швидкості" / "Speed sensor" and
"Обмеження швидкості та кермо" / "Speed limit and steering"), showing the speed sensor state and
the limiter and steering-scaling state without any input or command.

#### Scenario: Read-only speed readout
- **WHEN** the Diagnostics section renders a telemetry frame
- **THEN** the card SHALL show the vehicle speed, the sensor state, the speed limit, the limiting
  row (only while `speed_limit_ceil` is below 100), the steering authority, the scaling base with
  its source, the inactive reason, the bench test-speed row (only while an override is in force),
  the odometer and the trip
- **AND** the card SHALL contain no input field, button or other control that sends a command

#### Scenario: Same values in both places
- **WHEN** a value is shown both in this card and in one of the two speed settings cards
- **THEN** both SHALL be rendered from the same telemetry frame by one rendering path, so they never
  disagree

### Requirement: Network Identity Telemetry
The telemetry broadcast SHALL include the WiFi access point identity from the firmware constants,
delivered the same way as `firmware_version`, and the System section's "Мережа" / "Network" card
SHALL display it read-only. The web page SHALL NOT hard-code these values or derive them from the
browser location.

#### Scenario: Include the network identity in telemetry JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `wifi_ssid` equal to the compiled `WIFI_AP_SSID`
- **AND** `wifi_ip` equal to the compiled `WIFI_AP_IP` as a dotted-quad string (e.g. `"192.168.10.1"`)
- **AND** both SHALL be emitted unconditionally, independent of CAN or MAVLink status, and SHALL
  keep the telemetry document within its configured capacity

#### Scenario: Network card shows the values read-only
- **WHEN** a telemetry message containing `wifi_ssid` and `wifi_ip` is received
- **THEN** the Network card SHALL show the SSID and the IP address as read-only text, with a note
  that the settings will come later
- **AND** the card SHALL offer no input or command

#### Scenario: Values not yet received
- **WHEN** no telemetry with these keys has been received, or a frame lacks them
- **THEN** the card SHALL show `--` until the first value arrives and otherwise keep the previous
  value
- **AND** no JavaScript error SHALL be thrown

### Requirement: Steering Control Mode Telemetry
The telemetry broadcast SHALL include the current steering mode as `steer_mode`, and the "Кермо: стан" / "Steering: state" card SHALL display it in a "Режим" / "Mode" tile, so the operator can see whether the autopilot stick commands a position, a rotation rate, or nothing.

#### Scenario: Include the steering mode in telemetry JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `steer_mode`, one of `position`, `rate` or `blocked`, equal to the mode selected by the vehicle layer in the latest control-loop iteration (see "Sensorless Steering Rate Mode")
- **AND** the key SHALL be emitted unconditionally and SHALL keep the telemetry document within its configured capacity
- **AND** `steer_mode` SHALL be `position` whenever `steer_sensor_ok` is true, and SHALL NOT be `position` while `steer_sensor_ok` is false

#### Scenario: Mode tile
- **WHEN** the "Кермо: стан" card renders a telemetry frame
- **THEN** the Mode tile SHALL show "Позиція" / "Position" for `position`, "Швидкість (без датчика)" / "Rate (no sensor)" in the danger colour for `rate`, and "Заблоковано" / "Blocked" in the warning colour for `blocked`
- **AND** it SHALL show `--` when the key is missing or carries an unknown value, without a JavaScript error
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary

#### Scenario: Help text
- **WHEN** the help (ⓘ) of the "Кермо: стан" card is opened
- **THEN** it SHALL explain that without the angle sensor the autopilot stick steers by rate in MANUAL only (deflection = rotation, centre = stop, centre the stick once to arm), that there are no end-stop limits except the VESC current limit, and that other modes and the web slider are blocked

### Requirement: Speed-Limiter Floor Telemetry
The telemetry stream SHALL report the speed-limiter throttle floor currently in use, so the web
portal can show the stored setting and recognise a limiter sitting on its floor. The existing
`speed_limit_ceil` key SHALL keep carrying the final applied ceiling: the proportional taper plus
the integral trim, after clamping and the rate limit. No key SHALL be added for the integral trim
itself.

#### Scenario: Report the floor
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_floor`, the floor in percent to one decimal
- **AND** the key SHALL be present whether or not a limit is in force

#### Scenario: The ceiling key carries the final ceiling
- **WHEN** a telemetry frame is produced while a limit is in force
- **THEN** `speed_limit_ceil` SHALL be the ceiling actually applied to the throttle demand, after
  the taper, the integral trim, the floor and 100 % clamps and the ceiling rate limit
- **AND** it SHALL be 100 whenever no limit is in force or the speed reading is invalid

#### Scenario: The portal shows a limiter at its floor
- **WHEN** the portal renders a telemetry frame in which a limit is in force and
  `speed_limit_ceil` equals `speed_limit_floor`
- **THEN** the limiting row SHALL mark the ceiling as being at the floor
- **AND** the marker SHALL be present in both the English and the Ukrainian dictionary

### Requirement: Speed-Limiter Saturation Indicator
The system SHALL detect when the maximum-speed limiter has run out of authority, that is when
the applied ceiling is at the floor and the vehicle nevertheless stays above the limit. It SHALL
report this in the telemetry stream, and the web portal SHALL show it as a warning on the limiter
row. Because the operator cannot read the serial log in the field, the indicator SHALL be carried
by telemetry and the web portal and not by a serial log line. The firmware SHALL hold the
indicator for a fixed time after the condition clears, so the operator can still see it after the
event and every client shows the same state.

#### Scenario: Saturation is detected
- **WHEN** a limit is in force, the speed reading is valid, the applied ceiling is within 0.5 %
  of the floor, and the measured speed exceeds the limit by more than `SPEED_LIMIT_SAT_MARGIN_MS`
  (0.3 m/s), continuously for `SPEED_LIMIT_SAT_HOLD_MS` (2000 ms)
- **THEN** telemetry SHALL report `speed_limit_sat` as `true`
- **AND** `speed_limit_sat_held` SHALL be `true`

#### Scenario: Saturation clears immediately
- **WHEN** `speed_limit_sat` is `true` and, on any evaluation, the ceiling is no longer within
  0.5 % of the floor, the speed no longer exceeds the limit by more than
  `SPEED_LIMIT_SAT_MARGIN_MS`, the limit becomes zero, or the speed reading becomes invalid
- **THEN** `speed_limit_sat` SHALL become `false` at once, without a release delay
- **AND** a later entry SHALL again require the full `SPEED_LIMIT_SAT_HOLD_MS` of the condition

#### Scenario: The firmware holds the indicator after it clears
- **WHEN** `speed_limit_sat` changes from `true` to `false`
- **THEN** `speed_limit_sat_held` SHALL stay `true` for `SPEED_LIMIT_SAT_UI_HOLD_MS` (10 000 ms)
  measured on the firmware clock, and then become `false`
- **AND** a new saturation during the hold SHALL keep it `true` and restart the hold when that
  saturation clears
- **AND** the hold SHALL expire on time even if the limiter is no longer being evaluated
- **AND** a web client that connects or reconnects during the hold SHALL receive
  `speed_limit_sat_held` = `true`

#### Scenario: Report the saturation keys
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_sat` and `speed_limit_sat_held` as JSON booleans
- **AND** both keys SHALL be present whether or not a limit is in force
- **AND** both SHALL be `false` on a unit that has not saturated within the last
  `SPEED_LIMIT_SAT_UI_HOLD_MS`

#### Scenario: The portal shows saturation as a warning on the limiter row
- **WHEN** the portal renders a telemetry frame in which `speed_limit_sat_held` is `true`
- **THEN** the limiter row SHALL be shown even if `speed_limit_ceil` is 100
- **AND** it SHALL be drawn as a warning badge in the console's standard warning styling
- **AND** it SHALL read "Limiter at floor, speed above limit" while `speed_limit_sat` is `true`,
  and "Limiter was at floor, speed above limit" while only `speed_limit_sat_held` is `true`
- **AND** the portal SHALL NOT keep its own hold timer, but SHALL render what the firmware reports
- **AND** both texts SHALL be present in both the English and the Ukrainian dictionary

#### Scenario: The portal without saturation keys behaves as before
- **WHEN** the portal renders a frame in which `speed_limit_sat_held` is absent or `false`
- **THEN** the limiter row SHALL be shown and labelled exactly as "Speed-Limit and
  Limiter-Ceiling Telemetry" requires, with the "at floor" marker of "Speed-Limiter Floor
  Telemetry"

#### Scenario: No serial log line for saturation
- **WHEN** saturation is entered or cleared
- **THEN** no serial debug line SHALL be emitted for it, and no MAVLink message SHALL be sent for it
- **AND** the existing rate-limited log of limit changes SHALL be unaffected

### Requirement: Speed Sensor Pulse Total Telemetry
The telemetry stream SHALL report the speed sensor's running pulse total since boot, so the web
portal can measure pulses per revolution by itself. The firmware SHALL keep no measurement state
and SHALL offer no command for it: the value is read-only and is the same total that speed and
distance are derived from.

#### Scenario: Report the pulse total
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_pulse_total`, the value of `SpeedSensor::getPulseTotal()` as an
  unsigned 32-bit JSON integer
- **AND** the key SHALL be present in every frame, whether or not the speed reading is valid, and
  whatever the CAN status

#### Scenario: The total only increases
- **WHEN** the wheel turns in either direction
- **THEN** every counted falling edge of the sensor signal SHALL increase `speed_pulse_total` by one,
  because the single-channel sensor carries no direction
- **AND** the total SHALL never decrease except by wrapping modulo 2³² or by a reboot, which
  resets it to 0

#### Scenario: The total is independent of speed validity
- **WHEN** pulses are counted while `speed_valid` is false, the reading has decayed to 0 after the
  stale timeout, or a sample is discarded by the per-sample glitch guard
- **THEN** `speed_pulse_total` SHALL still include those pulses

#### Scenario: The firmware stays stateless
- **WHEN** the portal measures pulses per revolution
- **THEN** no web command, NVS key or firmware state SHALL be involved until the operator saves a
  result through the existing `speed_cal_ppr` command

### Requirement: Throttle Opening Rate Telemetry
The telemetry stream SHALL report the throttle opening rate currently in use, so the web portal can
show the stored setting.

#### Scenario: Report the rate
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `throttle_open_slew`, the opening rate in whole %/s
- **AND** 0 SHALL mean the opening rate limit is off

