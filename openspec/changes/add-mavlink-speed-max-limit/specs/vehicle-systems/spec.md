## ADDED Requirements

### Requirement: Maximum-Speed Limiter Ceiling Source Arbitration
The system SHALL arbitrate the maximum-speed limiter's km/h ceiling between the autopilot's
`SPEED_MAX` parameter and the locally stored setting. The local enable toggle SHALL remain the sole
master switch: the autopilot supplies the ceiling's *value* and SHALL NOT be able to arm or disarm
the limiter. A ceiling supplied over MAVLink SHALL be held in RAM only and SHALL NOT be persisted.

#### Scenario: Limiter disabled
- **WHEN** the effective ceiling is requested
- **AND** the local limiter enable toggle is off
- **THEN** the source SHALL be reported as `OFF` and the ceiling as zero
- **AND** no autopilot value SHALL be consulted, because the master switch is local by design

#### Scenario: Autopilot value available
- **WHEN** the limiter is enabled
- **AND** the MAVLink interface reports a usable `SPEED_MAX`
- **THEN** the source SHALL be reported as `MAVLINK`
- **AND** the ceiling SHALL be the autopilot value converted to km/h

#### Scenario: Fall back to the stored value
- **WHEN** the limiter is enabled
- **AND** the MAVLink interface reports no usable `SPEED_MAX` — never received, zero, out of
  range, stale, or the link is down
- **THEN** the source SHALL be reported as `LOCAL`
- **AND** the ceiling SHALL be the stored limiter maximum

#### Scenario: Clamp an out-of-range autopilot value rather than discarding it
- **WHEN** the autopilot value converted to km/h falls outside
  `[SPEED_LIMIT_MIN_KMH, SPEED_LIMIT_MAX_KMH]`
- **THEN** it SHALL be clamped into that range and still used, with the source reported as
  `MAVLINK`
- **AND** it SHALL NOT be discarded in favour of the stored value, because silently reverting a
  deliberate crawl setting to a much higher stored ceiling is the more dangerous outcome

#### Scenario: Never persist the autopilot value
- **WHEN** any `SPEED_MAX` value is received, accepted, clamped or applied
- **THEN** the stored limiter maximum SHALL NOT be modified
- **AND** no non-volatile write SHALL be performed on this path, because the setter that persists
  the limit writes flash on every call and a periodic poll would wear it out
- **AND** the stored value SHALL survive a power cycle unchanged, so the operator's fallback and
  the value shown in the configuration input are never overwritten by autopilot traffic

#### Scenario: Log a source or ceiling change once
- **WHEN** the arbitrated source changes, or the arbitrated ceiling changes by more than
  `SPEED_LIMIT_LOG_EPSILON_KMH`
- **THEN** the new source and ceiling SHALL be logged once
- **AND** a further log SHALL be suppressed until at least `SPEED_LIMIT_SRC_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered source and ceiling SHALL NOT be updated, so the
  settled state is logged on the next opportunity rather than lost

### Requirement: Proportional Speed-Limiter Throttle Taper
The maximum-speed limiter SHALL withdraw throttle authority proportionally as the vehicle
approaches its ceiling, instead of applying a single reduced cap once the ceiling is crossed. The
limiter SHALL be evaluated on every control-loop iteration for both the autopilot and the web
throttle paths. It SHALL continue to fail open on an invalid speed reading, and SHALL continue to
leave the gear-change throttle boost untouched.

#### Scenario: Below the taper band the demand passes through
- **WHEN** the limiter is enabled and the speed reading is valid
- **AND** the measured speed is at or below `ceiling − SPEED_LIMIT_TAPER_BAND_KMH`
- **THEN** the throttle ceiling SHALL be 100 %
- **AND** the commanded throttle SHALL equal the arbitrated demand

#### Scenario: Inside the taper band the ceiling falls linearly
- **WHEN** the measured speed is between `ceiling − SPEED_LIMIT_TAPER_BAND_KMH` and the ceiling
- **THEN** the throttle ceiling SHALL fall linearly from 100 % to `SPEED_LIMIT_FLOOR_PCT` across
  that band
- **AND** the commanded throttle SHALL be the lesser of the arbitrated demand and that ceiling

#### Scenario: At or above the ceiling the floor holds
- **WHEN** the measured speed is at or above the ceiling
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
- **WHEN** the limiter is enabled but the speed reading is invalid or stale
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 %
- **AND** a warning SHALL be logged no more often than `SPEED_LIMIT_WARN_MS`

#### Scenario: Disabling the limiter resets the ceiling
- **WHEN** the limiter enable toggle is off
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 %, so re-enabling the limiter never resumes
  from a stale low ceiling

#### Scenario: The web throttle demand is re-limited every loop
- **WHEN** a web throttle command is received
- **THEN** the requested percentage SHALL be recorded as the standing web throttle demand
- **AND** while the web input source is active, no gear boost is running and the throttle is not
  being calibrated, the limiter SHALL be re-applied to that standing demand on every control-loop
  iteration
- **AND** a vehicle that accelerates past the ceiling on an unchanged web command SHALL therefore
  be limited without the operator sending a new command

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
