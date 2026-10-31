## MODIFIED Requirements
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
