## ADDED Requirements
### Requirement: Speed-Limiter Ceiling Rate Telemetry
The telemetry stream SHALL report the speed-limiter ceiling rate currently in use, so the web
portal can show the stored setting.

#### Scenario: Report the rate
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_slew`, the rate in %/s to one decimal
- **AND** the key SHALL be present whether or not a limit is in force
