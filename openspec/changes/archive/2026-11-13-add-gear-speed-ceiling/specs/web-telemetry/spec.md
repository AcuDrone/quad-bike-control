## MODIFIED Requirements
### Requirement: Speed-Limit and Limiter-Ceiling Telemetry
The telemetry stream SHALL report the effective speed limit the limiter is enforcing, where that
limit comes from, and how much throttle authority the limiter is currently withdrawing. Road
speeds SHALL be carried through the firmware in metres per second and converted to km/h only at
serialisation.

#### Scenario: Report the enforced limit
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_kmh`, the effective limit (the smaller of the usable
  `SPEED_MAX` and the engaged gear's fixed ceiling) in km/h to one decimal
- **AND** the value SHALL be zero whenever neither provides a ceiling, so zero unambiguously means
  "no limit"

#### Scenario: Report the source of the limit
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_src`: `"speed_max"` when the autopilot's `SPEED_MAX` is
  the effective limit (including a tie with the gear ceiling), `"gear_l"`, `"gear_h"` or `"gear_r"`
  when the fixed ceiling of that expected gear (L 45, H 75, R 20 km/h) is, and `"none"` when there is no limit
- **AND** the gear letter SHALL come from the same expected gear the limiter used, not from the
  sensed-gear key `gear`

#### Scenario: Report the current throttle ceiling
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_ceil`, the throttle ceiling the taper is currently
  applying as a percentage
- **AND** a value of 100 SHALL mean the limiter is withdrawing no authority

#### Scenario: The retired limiter keys are gone
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL NOT include `speed_limit_on`, `speed_limit_max`, `speed_limit_eff` or
  `mav_speed_max`
- **AND** no web command SHALL exist for setting a local ceiling, toggling the limiter or changing
  the gear ceilings

#### Scenario: Convert to km/h only at the presentation edge
- **WHEN** `vehicle_speed` or `speed_limit_kmh` is serialised
- **THEN** the value SHALL be converted from the firmware's internal metres per second at that
  point
- **AND** `vehicle_speed` SHALL remain a km/h value with one decimal, unchanged on the wire

#### Scenario: Web interface shows the limit and the ceiling
- **WHEN** the web interface renders a telemetry frame
- **THEN** it SHALL display the limit in both km/h and m/s when one is in force, and a "none"
  label when `speed_limit_kmh` is zero
- **AND** when `speed_limit_src` names a gear the limit SHALL be followed by the gear marker
  ("· передача L" / "· gear L", likewise H and R, e.g. "20.0 km/h (5.6 m/s) · gear R"), and when the key is absent the display SHALL be as before
- **AND** the current throttle ceiling SHALL be shown only while the limiter is actually
  withdrawing authority (`speed_limit_ceil` below 100)
- **AND** the help of the "Обмеження швидкості та кермо" / "Speed limit and steering" card (its ⓘ
  button) SHALL state that the operator limit comes only from the autopilot's `SPEED_MAX`, that
  zero means no operator limit, that a fixed gear ceiling (L 45 km/h, H 75 km/h, R 20 km/h) always applies and
  cannot be switched off, that the lower of the two is enforced, and that the limiter does not act
  without a valid sensor reading
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary
