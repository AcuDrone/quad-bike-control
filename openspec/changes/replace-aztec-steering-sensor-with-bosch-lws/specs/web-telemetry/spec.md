## ADDED Requirements
### Requirement: Steering Angle Sensor Telemetry
The telemetry broadcast SHALL report the Bosch LWS steering angle, its status and its error counters, together with the stored limits and the zero-set progress, and the web calibration card SHALL display them, so that calibration and sensor faults can be diagnosed from the portal.

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
- **THEN** the calibration card SHALL show the angle, the rate, the Left/Right limits (`--` for `null`), the status bits and fault code, the frame rate and the error counters
- **AND** next to a limit beyond ±550° it SHALL show a note that the sensor's latched-fault threshold is near (shown while `steer_range_warning` is true)
- **AND** it SHALL show the `steer_sensor_fault` token as a localized label (en/uk); for `range_latched` the label SHALL say to restart the vehicle
- **AND** it SHALL show the zero-set progress and any failure reason

## REMOVED Requirements
### Requirement: Steering Position Sensor Telemetry
**Reason**: It described the dual-AS5600 readout (fine/coarse raw angles, turn, ratio, residual, session calibration progress), all of which is removed with the AS5600 sensors.
**Migration**: Use "Steering Angle Sensor Telemetry". Clients reading `steer_position` (counts), `steer_center` or the dual-sensor fields switch to `steer_angle` (degrees) and the `steer_lws_*` fields.
