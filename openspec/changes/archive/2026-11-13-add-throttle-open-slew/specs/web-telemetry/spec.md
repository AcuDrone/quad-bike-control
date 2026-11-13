## ADDED Requirements
### Requirement: Throttle Opening Rate Telemetry
The telemetry stream SHALL report the throttle opening rate currently in use, so the web portal can
show the stored setting.

#### Scenario: Report the rate
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `throttle_open_slew`, the opening rate in whole %/s
- **AND** 0 SHALL mean the opening rate limit is off
