## ADDED Requirements

### Requirement: Autopilot-Sourced Maximum-Speed Limit
The maximum-speed limiter's ceiling SHALL have exactly one source: the autopilot's `SPEED_MAX`
parameter, in metres per second. There SHALL be no locally stored ceiling, no local enable toggle
and no arbitration. The limiter SHALL always be armed, and SHALL simply not limit whenever no
usable `SPEED_MAX` is available. The value SHALL be held in RAM only and SHALL NOT be persisted.

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
- **AND** the retired non-volatile limiter keys SHALL be deleted on initialisation so a downgraded
  unit cannot resurrect a ceiling in the old units

#### Scenario: Log a limit change once
- **WHEN** the limit changes by more than `SPEED_LIMIT_LOG_EPSILON_MS`, or changes between "a
  limit" and "no limit"
- **THEN** the new limit SHALL be logged once, in m/s with km/h in parentheses, or as "none"
- **AND** a further log SHALL be suppressed until at least `SPEED_LIMIT_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered limit SHALL NOT be updated, so the settled
  value is logged on the next opportunity rather than lost

### Requirement: Proportional Speed-Limiter Throttle Taper
The maximum-speed limiter SHALL withdraw throttle authority proportionally as the vehicle
approaches its limit, instead of applying a single reduced cap once the limit is crossed. All
comparisons SHALL be made in metres per second. The limiter SHALL be evaluated on every
control-loop iteration for both the autopilot and the web throttle paths. It SHALL continue to
fail open on an invalid speed reading, and SHALL continue to leave the gear-change throttle boost
untouched.

#### Scenario: No limit means no taper
- **WHEN** the limit is zero
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 %, so a limit that arrives later never
  resumes from a stale low ceiling

#### Scenario: Below the taper band the demand passes through
- **WHEN** a limit is in force and the speed reading is valid
- **AND** the measured speed is at or below `limit − SPEED_LIMIT_TAPER_BAND_MS`
- **THEN** the throttle ceiling SHALL be 100 %
- **AND** the commanded throttle SHALL equal the arbitrated demand

#### Scenario: Inside the taper band the ceiling falls linearly
- **WHEN** the measured speed is between `limit − SPEED_LIMIT_TAPER_BAND_MS` and the limit
- **THEN** the throttle ceiling SHALL fall linearly from 100 % to `SPEED_LIMIT_FLOOR_PCT` across
  that band
- **AND** the commanded throttle SHALL be the lesser of the arbitrated demand and that ceiling

#### Scenario: At or above the limit the floor holds
- **WHEN** the measured speed is at or above the limit
- **THEN** the throttle ceiling SHALL be `SPEED_LIMIT_FLOOR_PCT`
- **AND** the throttle SHALL NOT be cut to zero, because removing all drive mid-corner is a
  stability event rather than a safety measure

#### Scenario: The ceiling is rate-limited in both directions
- **WHEN** the computed target ceiling differs from the ceiling currently applied
- **THEN** the applied ceiling SHALL move toward the target by at most
  `SPEED_LIMIT_CEILING_SLEW_PCT_S` per second, whether rising or falling
- **AND** the rate limit SHALL be applied to the ceiling and not to the operator's demand, so the
  driver's own throttle movements are never slowed by the limiter
- **AND** the elapsed time used SHALL be capped at one second, and the first evaluation after the
  limiter becomes active SHALL adopt the target directly, so a stalled or restarted loop cannot
  produce a step change through an unbounded interval

#### Scenario: Invalid speed reading fails open
- **WHEN** a limit is in force but the speed reading is invalid or stale
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 %
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
