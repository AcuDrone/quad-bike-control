## ADDED Requirements
### Requirement: Speed-Limiter Floor Telemetry
The telemetry stream SHALL report the speed-limiter throttle floor currently in use, so the web
portal can show the stored setting and recognise a limiter sitting on its floor. The existing
`speed_limit_ceil` key SHALL keep carrying the final applied ceiling: the proportional taper plus
the integral trim, after clamping and the rate limit. No key SHALL be added for the integral trim
itself.

#### Scenario: Report the floor
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_floor`, the floor in percent to one decimal
- **AND** the key SHALL be present whether or not a limit is in force

#### Scenario: The ceiling key carries the final ceiling
- **WHEN** a telemetry frame is produced while a limit is in force
- **THEN** `speed_limit_ceil` SHALL be the ceiling actually applied to the throttle demand, after
  the taper, the integral trim, the floor and 100 % clamps and the ceiling rate limit
- **AND** it SHALL be 100 whenever no limit is in force or the speed reading is invalid

#### Scenario: The portal shows a limiter at its floor
- **WHEN** the portal renders a telemetry frame in which a limit is in force and
  `speed_limit_ceil` equals `speed_limit_floor`
- **THEN** the limiting row SHALL mark the ceiling as being at the floor
- **AND** the marker SHALL be present in both the English and the Ukrainian dictionary

### Requirement: Speed-Limiter Saturation Indicator
The system SHALL detect when the maximum-speed limiter has run out of authority, that is when
the applied ceiling is at the floor and the vehicle nevertheless stays above the limit. It SHALL
report this in the telemetry stream, and the web portal SHALL show it as a warning on the limiter
row. Because the operator cannot read the serial log in the field, the indicator SHALL be carried
by telemetry and the web portal and not by a serial log line. The firmware SHALL hold the
indicator for a fixed time after the condition clears, so the operator can still see it after the
event and every client shows the same state.

#### Scenario: Saturation is detected
- **WHEN** a limit is in force, the speed reading is valid, the applied ceiling is within 0.5 %
  of the floor, and the measured speed exceeds the limit by more than `SPEED_LIMIT_SAT_MARGIN_MS`
  (0.3 m/s), continuously for `SPEED_LIMIT_SAT_HOLD_MS` (2000 ms)
- **THEN** telemetry SHALL report `speed_limit_sat` as `true`
- **AND** `speed_limit_sat_held` SHALL be `true`

#### Scenario: Saturation clears immediately
- **WHEN** `speed_limit_sat` is `true` and, on any evaluation, the ceiling is no longer within
  0.5 % of the floor, the speed no longer exceeds the limit by more than
  `SPEED_LIMIT_SAT_MARGIN_MS`, the limit becomes zero, or the speed reading becomes invalid
- **THEN** `speed_limit_sat` SHALL become `false` at once, without a release delay
- **AND** a later entry SHALL again require the full `SPEED_LIMIT_SAT_HOLD_MS` of the condition

#### Scenario: The firmware holds the indicator after it clears
- **WHEN** `speed_limit_sat` changes from `true` to `false`
- **THEN** `speed_limit_sat_held` SHALL stay `true` for `SPEED_LIMIT_SAT_UI_HOLD_MS` (10 000 ms)
  measured on the firmware clock, and then become `false`
- **AND** a new saturation during the hold SHALL keep it `true` and restart the hold when that
  saturation clears
- **AND** the hold SHALL expire on time even if the limiter is no longer being evaluated
- **AND** a web client that connects or reconnects during the hold SHALL receive
  `speed_limit_sat_held` = `true`

#### Scenario: Report the saturation keys
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_sat` and `speed_limit_sat_held` as JSON booleans
- **AND** both keys SHALL be present whether or not a limit is in force
- **AND** both SHALL be `false` on a unit that has not saturated within the last
  `SPEED_LIMIT_SAT_UI_HOLD_MS`

#### Scenario: The portal shows saturation as a warning on the limiter row
- **WHEN** the portal renders a telemetry frame in which `speed_limit_sat_held` is `true`
- **THEN** the limiter row SHALL be shown even if `speed_limit_ceil` is 100
- **AND** it SHALL be drawn as a warning badge in the console's standard warning styling
- **AND** it SHALL read "Limiter at floor, speed above limit" while `speed_limit_sat` is `true`,
  and "Limiter was at floor, speed above limit" while only `speed_limit_sat_held` is `true`
- **AND** the portal SHALL NOT keep its own hold timer, but SHALL render what the firmware reports
- **AND** both texts SHALL be present in both the English and the Ukrainian dictionary

#### Scenario: The portal without saturation keys behaves as before
- **WHEN** the portal renders a frame in which `speed_limit_sat_held` is absent or `false`
- **THEN** the limiter row SHALL be shown and labelled exactly as "Speed-Limit and
  Limiter-Ceiling Telemetry" requires, with the "at floor" marker of "Speed-Limiter Floor
  Telemetry"

#### Scenario: No serial log line for saturation
- **WHEN** saturation is entered or cleared
- **THEN** no serial debug line SHALL be emitted for it, and no MAVLink message SHALL be sent for it
- **AND** the existing rate-limited log of limit changes SHALL be unaffected
