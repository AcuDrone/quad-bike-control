## ADDED Requirements
### Requirement: Brake Position and Tuning Telemetry
The telemetry stream SHALL report the brake position estimate, the brake tuning values in use and
the released endstop state, so the operator can tune the brake from the web portal.

#### Scenario: Report the position estimate
- **WHEN** a telemetry frame is produced
- **THEN** `brake_pct` SHALL be the firmware's current brake position estimate, 0–100, instead of
  a constant 0
- **AND** the panel's "Гальмо" / "Brake" tile SHALL show it as a whole percentage

#### Scenario: Report the tuning values
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `brake_travel_ms` (whole ms) and `brake_hold_pwm` (0–100, 0 = no hold
  push), the values in use

#### Scenario: Report the endstop
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `brake_released`, true when the released endstop (In5) is asserted in a
  valid input-expander snapshot
- **AND** it SHALL be false when the endstop is not asserted or the snapshot is invalid

#### Scenario: Telemetry document capacity
- **WHEN** the new keys are added
- **THEN** the worst-case telemetry document SHALL still fit `StaticJsonDocument<9216>` with
  headroom, and the capacity comment SHALL be updated
