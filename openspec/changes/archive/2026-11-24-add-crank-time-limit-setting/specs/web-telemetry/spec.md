## ADDED Requirements
### Requirement: Starter Time Limit Telemetry
The telemetry stream SHALL report the starter time limit currently in use, so the web portal can
show the stored setting.

#### Scenario: Report the limit
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `crank_max_ms`, the starter time limit in use in whole ms (2000–10000)

#### Scenario: Telemetry document capacity
- **WHEN** the new key is added
- **THEN** the worst-case telemetry document SHALL still fit its `StaticJsonDocument` with headroom
  on every branch (9216 on `master` and `old-board`; grown to 10240 on `aztec`, whose headroom at
  9216 would drop to ≈ 0.15 KB), and the capacity comment SHALL be updated
