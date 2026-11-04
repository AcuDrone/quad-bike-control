## MODIFIED Requirements
### Requirement: Autopilot-Sourced Maximum-Speed Limit
The maximum-speed limiter's ceiling SHALL have exactly one source: the autopilot's `SPEED_MAX`
parameter, in metres per second. There SHALL be no locally stored ceiling, no local enable toggle
and no arbitration. The limiter SHALL always be armed, and SHALL simply not limit whenever no
usable `SPEED_MAX` is available. The value SHALL be held in RAM only and SHALL NOT be persisted.
The only persisted limiter parameter SHALL be the throttle floor (see "Configurable
Speed-Limiter Throttle Floor"), which shapes how the limit is enforced but SHALL never itself
impose a limit.

#### Scenario: A usable autopilot value is the limit
- **WHEN** the MAVLink interface reports a usable `SPEED_MAX`
- **THEN** the limit SHALL be that value in metres per second
- **AND** no other source SHALL be consulted, because there is no other source

#### Scenario: No usable value means no limiting
- **WHEN** the MAVLink interface reports no usable `SPEED_MAX` — never received, zero, rejected,
  stale, or the link is down
- **THEN** the limit SHALL be reported as zero
- **AND** a zero limit SHALL mean NO LIMITING: the throttle demand SHALL pass through untouched
- **AND** the system SHALL NOT substitute any stored, default or last-known ceiling, because an
  invisible ceiling the ground station cannot see or change is the more dangerous outcome

#### Scenario: Zero means "no limit", as the autopilot reads it
- **WHEN** the autopilot's `SPEED_MAX` is zero
- **THEN** the limiter SHALL not limit
- **AND** this SHALL match ArduPilot's own reading of a zero `SPEED_MAX` and the value a ground
  station writes to clear a speed limit

#### Scenario: No persisted limiter configuration
- **WHEN** any `SPEED_MAX` value is received, accepted or applied
- **THEN** no non-volatile write SHALL be performed on this path
- **AND** the system SHALL NOT expose any command, key or web control for setting a local ceiling
  or for enabling and disabling the limiter
- **AND** the retired non-volatile limiter keys (`lim_on`, `lim_kmh`) SHALL be deleted on
  initialisation so a downgraded unit cannot resurrect a ceiling in the old units
- **AND** the persisted throttle floor SHALL NOT be stored under either retired key name

#### Scenario: The floor alone never limits
- **WHEN** a throttle floor is stored and no usable `SPEED_MAX` is available
- **THEN** the demand SHALL pass through untouched, exactly as with no floor stored

#### Scenario: Log a limit change once
- **WHEN** the limit changes by more than `SPEED_LIMIT_LOG_EPSILON_MS`, or changes between "a
  limit" and "no limit"
- **THEN** the new limit SHALL be logged once, in m/s with km/h in parentheses, or as "none"
- **AND** a further log SHALL be suppressed until at least `SPEED_LIMIT_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered limit SHALL NOT be updated, so the settled
  value is logged on the next opportunity rather than lost

### Requirement: Proportional Speed-Limiter Throttle Taper
The maximum-speed limiter SHALL withdraw throttle authority proportionally as the vehicle
approaches its limit, instead of applying a single reduced cap once the limit is crossed. The
width of the taper band SHALL be proportional to the limit, within an absolute minimum and
maximum. The applied ceiling SHALL be the sum of this proportional taper and the integral trim
(see "Speed-Limiter Integral Trim"), clamped to the range from the configured floor to 100 %.
All comparisons SHALL be made in metres per second. The limiter SHALL be evaluated on every
control-loop iteration for both the autopilot and the web throttle paths. It SHALL continue to
fail open on an invalid speed reading, and SHALL continue to leave the gear-change throttle boost
untouched.

#### Scenario: No limit means no taper
- **WHEN** the limit is zero
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 % and the integral trim to zero, so a limit
  that arrives later never resumes from a stale ceiling or a stale trim

#### Scenario: The taper band scales with the limit
- **WHEN** a limit `L` is in force
- **THEN** the taper band SHALL be `SPEED_LIMIT_TAPER_FRACTION × L` (0.20), but not less than
  `SPEED_LIMIT_TAPER_MIN_MS` (0.5 m/s) and not more than `SPEED_LIMIT_TAPER_MAX_MS` (1.4 m/s)
- **AND** for a limit of 2.78 m/s (10 km/h) the band SHALL be 0.56 m/s, for 5.56 m/s (20 km/h)
  1.11 m/s, and for 11.1 m/s (40 km/h) 1.4 m/s
- **AND** the band SHALL be recomputed on every evaluation from the current limit

#### Scenario: Below the taper band the demand passes through
- **WHEN** a limit is in force and the speed reading is valid
- **AND** the measured speed is at or below `limit − band`
- **THEN** the throttle ceiling SHALL be 100 %
- **AND** the commanded throttle SHALL equal the arbitrated demand

#### Scenario: Inside the taper band the ceiling falls linearly
- **WHEN** the measured speed is between `limit − band` and the limit
- **THEN** the proportional term SHALL fall linearly from 100 % to the configured floor across
  that band
- **AND** the target ceiling SHALL be that proportional term plus the integral trim, clamped to
  the floor and 100 %
- **AND** the commanded throttle SHALL be the lesser of the arbitrated demand and the applied
  ceiling

#### Scenario: At or above the limit the floor holds
- **WHEN** the measured speed is at or above the limit
- **THEN** the proportional term SHALL be the configured floor
- **AND** the target ceiling SHALL be the floor plus the integral trim, which falls while the
  speed stays above the limit
- **AND** the ceiling SHALL never step down to the floor; it SHALL reach it only through the
  taper, the trim and the ceiling rate limit

#### Scenario: The ceiling is rate-limited in both directions
- **WHEN** the computed target ceiling differs from the ceiling currently applied
- **THEN** the applied ceiling SHALL move toward the target by at most
  `SPEED_LIMIT_CEILING_SLEW_PCT_S` per second, whether rising or falling
- **AND** the rate limit SHALL be applied to the ceiling and not to the operator's demand, so the
  driver's own throttle movements are never slowed by the limiter
- **AND** the elapsed time used SHALL be capped at one second, and the first evaluation after the
  limiter becomes active SHALL adopt the proportional target directly with a zero trim, so a
  stalled or restarted loop cannot produce a step change through an unbounded interval

#### Scenario: Invalid speed reading fails open
- **WHEN** a limit is in force but the speed reading is invalid or stale
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 % and the integral trim to zero
- **AND** a warning SHALL be logged no more often than `SPEED_LIMIT_WARN_MS`

#### Scenario: The web throttle demand is re-limited every loop
- **WHEN** a web throttle command is received
- **THEN** the requested percentage SHALL be recorded as the standing web throttle demand
- **AND** while the web input source is active, no gear boost is running and the throttle is not
  being calibrated, the limiter SHALL be re-applied to that standing demand on every control-loop
  iteration
- **AND** a vehicle that accelerates past the limit on an unchanged web command SHALL therefore be
  limited without the operator sending a new command

#### Scenario: The standing web demand is cleared whenever the throttle is idled
- **WHEN** the throttle is forced to idle — web control is engaged, fail-safe is entered, or the
  gear-change boost releases the throttle
- **THEN** the standing web throttle demand SHALL be reset to zero
- **AND** it SHALL NOT be re-applied on the next iteration, because a demand that outlives an idle
  command would silently reopen the throttle

#### Scenario: Gear-change boost is outside the limiter
- **WHEN** the gear-change throttle boost is active
- **THEN** it SHALL continue to command the throttle servo directly in microseconds
- **AND** the limiter SHALL NOT be applied to it, because the boost holds engine speed while the
  drivetrain is disengaged and no road speed is being produced

## ADDED Requirements
### Requirement: Speed-Limiter Integral Trim
The maximum-speed limiter SHALL carry an integral trim, in percent, that is added to the
proportional taper so the steady-state speed converges to the limit whatever throttle is needed
to hold it, as long as that throttle lies between the configured floor and 100 %. The trim SHALL
only remove authority while the vehicle is above the limit, and SHALL only return authority while
the vehicle is below the limit and the limiter is actually restricting the demand. The trim SHALL
be bounded so the applied ceiling always stays between the floor and 100 %, and it SHALL never
move faster than the ceiling rate limit.

#### Scenario: Over the limit the trim falls in proportion to the excess
- **WHEN** a limit is in force, the speed reading is valid, and the measured speed exceeds the
  limit by `e` m/s
- **THEN** the trim SHALL decrease by `SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS × e × dt` (30 %/s per
  m/s), but by no more than `SPEED_LIMIT_CEILING_SLEW_PCT_S × dt`
- **AND** an excess of 1.4 m/s SHALL therefore take the trim from 100 % to zero in about 2.4 s
- **AND** this SHALL apply whether or not the demand is below the ceiling, because lowering the
  ceiling while over the limit is always the conservative direction

#### Scenario: Under the limit and binding the trim recovers gradually
- **WHEN** the measured speed is below the limit but above `limit − band`
- **AND** the demand is at or above the applied ceiling minus `SPEED_LIMIT_BIND_MARGIN_PCT` (1 %)
- **THEN** the trim SHALL increase by `SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS × (limit − speed) × dt`,
  but by no more than `SPEED_LIMIT_TRIM_RISE_PCT_S × dt` (10 %/s)
- **AND** the resulting ceiling rise SHALL never exceed the ceiling rate limit

#### Scenario: No accumulation while the driver is below the ceiling
- **WHEN** the measured speed is below the limit and the demand is below the applied ceiling by
  more than `SPEED_LIMIT_BIND_MARGIN_PCT`
- **THEN** the trim SHALL be held unchanged
- **AND** a driver who lifts off and re-applies the throttle SHALL meet the proportional taper
  for the current speed plus a trim that has not grown in the meantime

#### Scenario: The trim is bounded
- **WHEN** the trim is updated
- **THEN** it SHALL be clamped to the range from zero to `100 − floor`
- **AND** the target ceiling SHALL be clamped to the range from the floor to 100 %

#### Scenario: The trim is reset outside its working range
- **WHEN** the measured speed is at or below `limit − band`, the limit is zero, the speed reading
  is invalid, or the limiter evaluates for the first time after any of these
- **THEN** the trim SHALL be zero
- **AND** the next approach to the limit SHALL meet the full proportional taper

#### Scenario: Steady state converges to the limit
- **WHEN** the vehicle is held against the limit on a constant grade and the throttle `u` that
  holds the limit lies between the floor and 100 %
- **THEN** the applied ceiling SHALL settle at `u` and the measured speed SHALL settle at the limit
  within the speed-sensor resolution, rather than at a point inside the taper band below it

#### Scenario: The law is host-testable
- **WHEN** the limiter law is built
- **THEN** the band, taper, trim and slew arithmetic SHALL live in a pure header with no Arduino or
  `Constants.h` dependency
- **AND** it SHALL be covered by a native Unity test suite

### Requirement: Configurable Speed-Limiter Throttle Floor
The lowest throttle ceiling the maximum-speed limiter may apply SHALL be a persisted, operator-set
value in percent of the calibrated throttle span. It SHALL default to
`SPEED_LIMIT_FLOOR_DEFAULT_PCT` (0 %, the calibrated idle position) and SHALL be accepted in the
range `SPEED_LIMIT_FLOOR_MIN_PCT`–`SPEED_LIMIT_FLOOR_MAX_PCT` (0–20 %). It SHALL be stored in NVS
namespace `"speed"` under the key `thr_floor` (float, percent), loaded at start-up and written
immediately when changed. It SHALL apply to both vehicles alike.

#### Scenario: Default floor on a unit without a stored value
- **WHEN** the system starts and `thr_floor` has never been written
- **THEN** the floor SHALL be 0 %
- **AND** at and above the limit the limiter SHALL therefore be able to bring the throttle to the
  calibrated idle position

#### Scenario: Stored floor is restored
- **WHEN** the system starts and `thr_floor` holds a value within 0–20 %
- **THEN** that value SHALL be the floor

#### Scenario: Invalid stored floor is ignored
- **WHEN** the system starts and `thr_floor` holds NaN or a value outside 0–20 %
- **THEN** the stored value SHALL be ignored and logged, and the default SHALL be used
- **AND** the stored value SHALL NOT be repaired or rewritten

#### Scenario: Floor changed at runtime
- **WHEN** a valid new floor is set
- **THEN** it SHALL be persisted to `thr_floor` immediately
- **AND** it SHALL take effect on the next limiter evaluation, with the trim and the ceiling
  re-clamped to the new range and the ceiling still moving no faster than its rate limit

#### Scenario: The floor key does not collide with retired or sensor keys
- **WHEN** the speed sensor removes the retired limiter keys at start-up
- **THEN** `thr_floor` SHALL be left untouched
- **AND** the `"speed"` namespace SHALL never be cleared as a whole
