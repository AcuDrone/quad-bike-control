## ADDED Requirements
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

## MODIFIED Requirements
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
- **THEN** the "Кермо: стан" card SHALL show a 2-column tile grid: Кут / Angle, Швидкість / Rate (°/s), Межа Л / Limit L, Межа П / Limit R, Датчик / Sensor, Режим / Mode, Калібрування / Calibration, Драйвер VESC / VESC driver, Нуль датчика / Sensor zero, Струм мотора / Motor current, Темп. FET / FET temp and Напруга VESC / VESC voltage, with `--` for `null` or missing values
- **AND** while `steer_range_warning` is true, a limit beyond ±550° SHALL carry a short ⚠ note in its tile and a full-width note SHALL explain that the sensor latches a fault at about ±600°
- **AND** the Sensor tile SHALL show the `steer_sensor_fault` token as a localized label (en/uk); for `range_latched` the label SHALL say to restart the vehicle
- **AND** the Sensor zero tile SHALL show the zero-set progress and any failure reason
- **AND** the Mode tile SHALL show `steer_mode` as described in "Steering Control Mode Telemetry"
- **AND** below the tiles one full-width, muted, monospace line SHALL show the raw LWS status bits (OK, cal, adj), the fault code, the frame rate and the checksum, counter and missed-frame counters

#### Scenario: Compact readout in the calibration card
- **WHEN** the Calibration section is shown
- **THEN** the Steering calibration card SHALL show only the angle, the Left/Right limits with their ⚠ notes and one sensor/calibration status line, rendered from the same telemetry frame as the "Кермо: стан" card
