## ADDED Requirements
### Requirement: Pre-Crank Delay Telemetry
The telemetry stream SHALL report the pre-crank dwell currently in use, so the web portal can show
the stored setting.

#### Scenario: Report the delay
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `crank_dwell_ms`, the pre-crank dwell in use in whole ms (1000–8000)

#### Scenario: Telemetry document capacity
- **WHEN** the new key is added
- **THEN** the worst-case telemetry document SHALL still fit `StaticJsonDocument<9216>` with
  headroom, and the capacity comment SHALL be updated
