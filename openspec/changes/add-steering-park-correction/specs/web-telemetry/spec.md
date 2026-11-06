## ADDED Requirements
### Requirement: Steering Park Correction Telemetry
The telemetry stream SHALL report the park correction settings and the result of the last correction, so that the web card fills from telemetry and the correction can be tuned without a serial console. The result SHALL be held in RAM only.

#### Scenario: Report the settings
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `steer_park_enabled` (bool), `steer_park_d1`, `steer_park_d2`, `steer_park_d3` (duty), `steer_park_t1`, `steer_park_t2`, `steer_park_t3` (ms), `steer_park_tol` (degrees, 1 decimal) and `steer_park_max` (count)

#### Scenario: Report the last result
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include an object `steer_park_last` with `outcome` ("none", "done", "gave_up" or "cancelled"), `pulses`, `residual` (degrees, 2 decimals, null for "none"), `duty` and `ms` of the last pulse
- **AND** "done" and "gave_up" SHALL be recorded when a correction ends that way, "cancelled" only when a correction that had sent at least one pulse is cancelled by a target move or a main-loop restart, and other cancels SHALL leave the last result unchanged
- **AND** after boot the outcome SHALL be "none"

#### Scenario: Document capacity
- **WHEN** the new fields are added
- **THEN** the telemetry document SHALL keep its headroom without overflowing, and the capacity note in `createTelemetryJSON` SHALL account for them
