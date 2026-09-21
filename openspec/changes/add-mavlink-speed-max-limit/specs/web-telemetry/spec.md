## ADDED Requirements

### Requirement: Speed-Limit and Limiter-Ceiling Telemetry
The telemetry stream SHALL report the speed limit the limiter is enforcing and how much throttle
authority the limiter is currently withdrawing. Because there is only one possible source for the
limit, no source key SHALL be reported. Road speeds SHALL be carried through the firmware in
metres per second and converted to km/h only at serialisation.

#### Scenario: Report the enforced limit
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_kmh`, the enforced limit in km/h to one decimal
- **AND** the value SHALL be zero whenever there is no usable `SPEED_MAX`, so zero unambiguously
  means "no limit"

#### Scenario: Report the current throttle ceiling
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_ceil`, the throttle ceiling the taper is currently
  applying as a percentage
- **AND** a value of 100 SHALL mean the limiter is withdrawing no authority

#### Scenario: The retired limiter keys are gone
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL NOT include `speed_limit_on`, `speed_limit_max`, `speed_limit_eff`,
  `speed_limit_src` or `mav_speed_max`
- **AND** no web command SHALL exist for setting a local ceiling or toggling the limiter

#### Scenario: Convert to km/h only at the presentation edge
- **WHEN** `vehicle_speed` or `speed_limit_kmh` is serialised
- **THEN** the value SHALL be converted from the firmware's internal metres per second at that
  point
- **AND** `vehicle_speed` SHALL remain a km/h value with one decimal, unchanged on the wire

#### Scenario: Web interface shows the limit and the ceiling
- **WHEN** the web interface renders a telemetry frame
- **THEN** it SHALL display the limit in both km/h and m/s when one is in force, and a "none"
  label when `speed_limit_kmh` is zero
- **AND** the current throttle ceiling SHALL be shown only while the limiter is actually
  withdrawing authority (`speed_limit_ceil` below 100)
- **AND** the card SHALL state that the limit comes only from the autopilot's `SPEED_MAX`, that
  zero means no limit, and that the limiter does not act without a valid sensor reading
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary
