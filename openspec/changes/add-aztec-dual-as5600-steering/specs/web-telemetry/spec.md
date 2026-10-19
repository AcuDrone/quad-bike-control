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

#### Scenario: Calibration card shows the dual-sensor readout
- **WHEN** the web UI receives steering position telemetry
- **THEN** the calibration card SHALL show the position and the L / C / R values in the reported frame,
  showing `--` for `null`
- **AND** it SHALL show the fine raw, coarse raw, turn, ratio and residual values
- **AND** it SHALL show the `steer_sensor_fault` token as a localized label (en/uk)
- **AND** the calibration help text SHALL describe the dual-sensor flow: capture Center first (this
  clears both limits), then Left and Right at the travel limits

#### Scenario: Capture response reports both sensors
- **WHEN** a steering capture command succeeds
- **THEN** the response message SHALL keep the `Steering <what> saved:` prefix and report the fine
  position and the coarse raw angle captured
- **WHEN** a steering capture command fails
- **THEN** the response message SHALL name the reason (sensor fault, no calibration session, too close
  to center, or the failed ratio/geometry check)
