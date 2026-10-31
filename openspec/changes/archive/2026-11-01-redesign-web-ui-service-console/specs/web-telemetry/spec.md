## ADDED Requirements
### Requirement: Diagnostics Attention Summary
The Diagnostics section SHALL start with an "Увага" / "Attention" box that lists the problems
active in the latest telemetry frame, derived only from existing telemetry fields, and SHALL be
hidden when no problem is active.

#### Scenario: Sensorless steering
- **WHEN** a frame has `steer_sensor_ok` false
- **THEN** the box SHALL list "steering sensor unhealthy, jog/nudge run open loop without stall
  protection" in the danger colour

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

#### Scenario: Board I/O fault
- **WHEN** a frame has `io_input_fault` or `io_relay_fault` true
- **THEN** the box SHALL list a board I/O fault line naming the faulted expander(s) in the danger
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

#### Scenario: Board boost rail not used
- **WHEN** a frame has `rail_low` true
- **THEN** the attention box SHALL NOT list it; `rail_24v` / `rail_low` remain shown only where
  they are today

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
Speed sensor settings card in the Calibration section, showing the speed sensor state and the
limiter and steering-scaling state without any input or command.

#### Scenario: Read-only speed readout
- **WHEN** the Diagnostics section renders a telemetry frame
- **THEN** the card SHALL show the vehicle speed, the sensor state, the speed limit, the limiting
  row (only while `speed_limit_ceil` is below 100), the steering authority, the scaling base with
  its source, the inactive reason, the bench test-speed row (only while an override is in force),
  the odometer and the trip
- **AND** the card SHALL contain no input field, button or other control that sends a command

#### Scenario: Same values in both places
- **WHEN** a value is shown both in this card and in the Speed sensor settings card
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

## MODIFIED Requirements
### Requirement: Steering Angle Sensor Telemetry
The telemetry broadcast SHALL report the Bosch LWS steering angle, its status and its error counters, together with the stored limits and the zero-set progress, and the web interface SHALL display them in a read-only "Кермо: стан" / "Steering: state" card in the Diagnostics section, so that calibration and sensor faults can be diagnosed from the portal.

#### Scenario: Angle and limits
- **WHEN** telemetry data is formatted as JSON
- **THEN** `steer_angle` SHALL carry the current steering angle in degrees with one decimal, positive = right, or `null` unless a fresh accepted frame carries a valid angle
- **AND** `steer_rate` SHALL carry the signed rotation rate in °/s (positive = turning right), or `null` when the speed is invalid or no fresh frame exists
- **AND** `steer_left` and `steer_right` SHALL carry the stored limits in degrees, or `null` when unset
- **AND** `steer_calibrated` SHALL be `true` only when both limits are stored and valid
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
- **THEN** the "Кермо: стан" card SHALL show a 2-column tile grid: Кут / Angle, Швидкість / Rate (°/s), Межа Л / Limit L, Межа П / Limit R, Датчик / Sensor, Калібрування / Calibration, Драйвер VESC / VESC driver, Нуль датчика / Sensor zero, Струм мотора / Motor current, Темп. FET / FET temp and Напруга VESC / VESC voltage, with `--` for `null` or missing values
- **AND** while `steer_range_warning` is true, a limit beyond ±550° SHALL carry a short ⚠ note in its tile and a full-width note SHALL explain that the sensor latches a fault at about ±600°
- **AND** the Sensor tile SHALL show the `steer_sensor_fault` token as a localized label (en/uk); for `range_latched` the label SHALL say to restart the vehicle
- **AND** the Sensor zero tile SHALL show the zero-set progress and any failure reason
- **AND** below the tiles one full-width, muted, monospace line SHALL show the raw LWS status bits (OK, cal, adj), the fault code, the frame rate and the checksum, counter and missed-frame counters

#### Scenario: Compact readout in the calibration card
- **WHEN** the Calibration section is shown
- **THEN** the Steering calibration card SHALL show only the angle, the Left/Right limits with their ⚠ notes and one sensor/calibration status line, rendered from the same telemetry frame as the "Кермо: стан" card

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
- **AND** the help of the Speed sensor card (its ⓘ button) SHALL state that the limit comes only
  from the autopilot's `SPEED_MAX`, that zero means no limit, and that the limiter does not act
  without a valid sensor reading
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary
