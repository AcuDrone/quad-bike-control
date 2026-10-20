## ADDED Requirements
### Requirement: Steering Position Sensor Telemetry
The telemetry broadcast SHALL report the dual-sensor steering position in a single consistent
frame — unwrapped fine-sensor counts relative to the captured center, in sensor direction — plus
per-sensor diagnostics, and the web calibration card SHALL display them, so that calibration and
sensor faults can be diagnosed from the portal.

#### Scenario: Position and calibration points in one frame
- **WHEN** telemetry data is formatted as JSON
- **THEN** `steer_position` SHALL carry the current unwrapped fine position relative to the captured
  center in sensor direction (relative to the boot-time fine sample when no center is captured)
- **AND** `steer_center` SHALL be `0` when a center is captured and `null` otherwise
- **AND** `steer_left` and `steer_right` SHALL carry the stored limit positions in the same frame, or
  `null` when unset
- **AND** while calibration points of an unfinished (or rejected) calibration are pending,
  `steer_position`, `steer_left`, `steer_center` and `steer_right` SHALL instead carry the session
  position and the recorded points, relative to the recorded center point, or relative to the first
  point captured when no center is recorded (so `steer_center` stays `0` or `null`), and `null`
  for points not yet recorded
- **AND** the former `-1` "unset" sentinel SHALL NOT be used, because valid positions may be negative

#### Scenario: Dual-sensor diagnostics
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `steer_fine_raw` and `steer_coarse_raw` (0–4095, `null` while that sensor
  is not readable), `steer_turn` (signed turn index relative to the center turn), `steer_ratio`
  (derived coarse counts per fine-shaft turn, `null` when not calibrated) and `steer_residual`
  (fine-count disagreement between the sensors, `null` when not calibrated or a sensor is faulted)
- **AND** it SHALL include `steer_sensor_fault`, one of `none`, `fine_i2c`, `fine_magnet`,
  `coarse_i2c`, `coarse_magnet`, `mismatch` or `recovering` (both sensors healthy and calibrated,
  turn not yet recovered), and SHALL NOT be `none` while `steer_sensor_ok` is `false`
- **AND** `steer_sensor_ok` SHALL be `true` only when both sensors are healthy and, when calibrated,
  the turn has been recovered and no mismatch is present or latched
- **AND** `steering_pct` SHALL keep its percent-of-calibrated-travel meaning

#### Scenario: Calibration progress
- **WHEN** telemetry data is formatted as JSON
- **THEN** `steer_cal_points` SHALL list the points recorded in the current power-on as a subset of
  `"LCR"` (`""` when none are pending)
- **AND** `steer_cal_error` SHALL name the failed check when all three points are set but the
  calibration was rejected, and SHALL be `null` otherwise
- **AND** `steer_cal_lost` SHALL be `true` after unfinished points were discarded by a sensor fault,
  until the next capture

#### Scenario: Calibration card shows the dual-sensor readout
- **WHEN** the web UI receives steering position telemetry
- **THEN** the calibration card SHALL show the position and the L / C / R values in the reported frame,
  showing `--` for `null`
- **AND** it SHALL show the fine raw, coarse raw, turn, ratio and residual values
- **AND** it SHALL show the `steer_sensor_fault` token as a localized label (en/uk)
- **AND** it SHALL mark each point recorded in the current power-on (✓), list the points still
  needed, show the rejection reason from `steer_cal_error`, and show a notice when `steer_cal_lost`
  is set
- **AND** the calibration help text (en/uk) SHALL describe the dual-sensor flow: Left, Center and
  Right may be captured in any order, each button records only its own point, the calibration
  completes once all three are set, and all three must be captured in one power-on because a
  reboot or a sensor fault discards unfinished points

#### Scenario: Capture response reports both sensors
- **WHEN** a steering capture command succeeds
- **THEN** the response message SHALL start with `Steering <what> recorded:` and report the fine
  position (display frame) and the coarse raw angle captured
- **AND** it SHALL say whether the calibration is now complete and saved, or which points are still
  needed
- **WHEN** a steering capture command fails
- **THEN** the response message SHALL name the reason (sensor fault, too close to center, too close
  to a limit)
- **WHEN** a capture completes the three points but the calibration is rejected
- **THEN** the response SHALL be a failure naming the failed ratio/geometry check
