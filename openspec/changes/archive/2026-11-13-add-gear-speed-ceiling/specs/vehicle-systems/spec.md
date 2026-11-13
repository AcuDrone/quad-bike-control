## ADDED Requirements
### Requirement: Fixed Per-Gear Maximum-Speed Ceiling
On the buggy (aztec), the maximum-speed limiter SHALL also enforce a fixed ceiling for the
engaged gear. The ceilings SHALL be firmware constants in metres per second:
`GEAR_SPEED_CEILING_LOW_MS` (45 km/h) for L, `GEAR_SPEED_CEILING_HIGH_MS` (75 km/h) for H and
`GEAR_SPEED_CEILING_REVERSE_MS` (20 km/h) for R. N and an uninitialised (UNKNOWN) expected gear
SHALL have no gear ceiling. The engaged
gear SHALL be the transmission's expected (target) gear, the same gear the N/UNKNOWN throttle clip
uses, not the opto gear-sensor reading. The ceilings SHALL NOT be configurable at runtime: no web
control, no non-volatile key and no MAVLink parameter.

#### Scenario: Gear ceiling applies alone when SPEED_MAX is "no limit"
- **WHEN** no usable `SPEED_MAX` is available (zero, never received, rejected, stale or link down)
- **AND** the expected gear is L
- **THEN** the effective limit SHALL be `GEAR_SPEED_CEILING_LOW_MS` (12.5 m/s, 45 km/h)
- **AND** in H it SHALL be `GEAR_SPEED_CEILING_HIGH_MS` (≈20.83 m/s, 75 km/h)
- **AND** in R it SHALL be `GEAR_SPEED_CEILING_REVERSE_MS` (≈5.56 m/s, 20 km/h)

#### Scenario: The lower of SPEED_MAX and the gear ceiling wins
- **WHEN** a usable `SPEED_MAX` and a gear ceiling are both in force
- **THEN** the effective limit SHALL be the smaller of the two
- **AND** when they are equal the source SHALL be reported as `SPEED_MAX`
- **AND** a `SPEED_MAX` above the gear ceiling SHALL NOT raise the limit above the gear ceiling

#### Scenario: Reverse is capped at 20 km/h
- **WHEN** the expected gear is R
- **THEN** the effective limit SHALL be the smaller of the usable `SPEED_MAX` and
  `GEAR_SPEED_CEILING_REVERSE_MS` (20 km/h), or 20 km/h when there is no usable `SPEED_MAX`

#### Scenario: No gear ceiling in N or an unknown gear
- **WHEN** the expected gear is N or UNKNOWN
- **THEN** the gear SHALL contribute no ceiling, and the effective limit SHALL be the usable
  `SPEED_MAX`, or no limit
- **AND** the throttle there SHALL remain clipped to `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` as before

#### Scenario: Enforced by the existing limiter law
- **WHEN** the gear ceiling is the effective limit
- **THEN** it SHALL be enforced exactly as a `SPEED_MAX` limit: the same taper band, floor, ceiling
  rate limit, integral trim and saturation indicator
- **AND** an invalid speed reading SHALL fail open in the same way
- **AND** while the gear ceiling is the source of the effective limit, the "speed reading invalid"
  warning SHALL NOT be logged, even when a usable `SPEED_MAX` above the gear ceiling is in force

#### Scenario: Changing gear causes no throttle step
- **WHEN** the expected gear changes and the effective limit changes with it
- **THEN** the limiter state SHALL NOT be reset, and the next evaluation SHALL use the new limit
- **AND** because gear changes are only permitted at or below `TRANS_SPEED_INTERLOCK_THRESHOLD_MS`,
  below every taper band, the applied ceiling SHALL already be 100 % at that moment

## MODIFIED Requirements
### Requirement: Autopilot-Sourced Maximum-Speed Limit
The maximum-speed limiter's operator-set ceiling SHALL have exactly one source: the autopilot's
`SPEED_MAX` parameter, in metres per second. There SHALL be no locally stored ceiling, no local
enable toggle and no arbitration between operator-set sources. The value SHALL be held in RAM
only and SHALL NOT be persisted. The effective limit SHALL be the smaller of the usable
`SPEED_MAX` and the fixed ceiling of the engaged gear (see "Fixed Per-Gear Maximum-Speed
Ceiling"). Whichever side is absent or zero SHALL be ignored. The limiter SHALL always be armed,
and SHALL not limit only when neither side provides a ceiling. The only persisted limiter
parameters SHALL be the throttle floor and the ceiling rate. They shape how the limit is enforced
but SHALL never themselves impose a limit.

#### Scenario: A usable autopilot value is the limit
- **WHEN** the MAVLink interface reports a usable `SPEED_MAX`
- **AND** the engaged gear has no gear ceiling, or a gear ceiling at or above that value
- **THEN** the limit SHALL be that value in metres per second, with source `SPEED_MAX`

#### Scenario: No usable value means no limiting
- **WHEN** the MAVLink interface reports no usable `SPEED_MAX` — never received, zero, rejected,
  stale, or the link is down
- **THEN** `SPEED_MAX` SHALL contribute no ceiling
- **AND** the system SHALL NOT substitute any stored, default or last-known `SPEED_MAX` value
- **AND** the effective limit SHALL be the fixed gear ceiling when the engaged gear has one, and
  otherwise zero
- **AND** a zero effective limit SHALL mean NO LIMITING: the throttle demand SHALL pass through
  untouched

#### Scenario: Zero means "no limit", as the autopilot reads it
- **WHEN** the autopilot's `SPEED_MAX` is zero
- **THEN** `SPEED_MAX` SHALL impose no limit, matching ArduPilot's own reading of a zero
  `SPEED_MAX` and the value a ground station writes to clear a speed limit
- **AND** the fixed gear ceiling SHALL still apply in a gear that has one

#### Scenario: No persisted limiter configuration
- **WHEN** any `SPEED_MAX` value is received, accepted or applied
- **THEN** no non-volatile write SHALL be performed on this path
- **AND** the system SHALL NOT expose any command, key or web control for setting a local ceiling
  or for enabling and disabling the limiter or the gear ceiling
- **AND** the retired non-volatile limiter keys (`lim_on`, `lim_kmh`) SHALL be deleted on
  initialisation so a downgraded unit cannot resurrect a ceiling in the old units
- **AND** the persisted throttle floor SHALL NOT be stored under either retired key name

#### Scenario: The floor alone never limits
- **WHEN** a throttle floor is stored and the effective limit is zero
- **THEN** the demand SHALL pass through untouched, exactly as with no floor stored

#### Scenario: Log a limit change once
- **WHEN** the effective limit changes by more than `SPEED_LIMIT_LOG_EPSILON_MS`, changes between
  "a limit" and "no limit", or its source changes
- **THEN** the new limit SHALL be logged once, in m/s with km/h in parentheses and its source
  (`SPEED_MAX` or the gear ceiling with the gear letter L, H or R), or as "none"
- **AND** a further log SHALL be suppressed until at least `SPEED_LIMIT_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered limit and source SHALL NOT be updated, so the
  settled value is logged on the next opportunity rather than lost

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
  driver's own throttle movements are never slowed by the limiter (the separate throttle opening
  rate limit at the servo input may still slow an opening, see "Configurable Throttle Opening
  Rate")
- **AND** the elapsed time used SHALL be capped at one second, and the first evaluation after the
  limiter becomes active SHALL adopt the proportional target directly with a zero trim, so a
  stalled or restarted loop cannot produce a step change through an unbounded interval

#### Scenario: Invalid speed reading fails open
- **WHEN** a limit is in force but the speed reading is invalid or stale
- **THEN** the demand SHALL pass through unclamped
- **AND** the applied ceiling SHALL be reset to 100 % and the integral trim to zero
- **AND** while the source of the effective limit is `SPEED_MAX` (including a tie with the gear
  ceiling), a warning SHALL be logged no more often than `SPEED_LIMIT_WARN_MS`
- **AND** while the source is the fixed gear ceiling — whether `SPEED_MAX` is unusable or a usable
  `SPEED_MAX` lies above the gear ceiling — no warning SHALL be logged

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

#### Scenario: The ceiling slew and the opening rate limit stack
- **WHEN** a limit is in force and the ceiling is rising at the ceiling rate while the demand is
  above it
- **THEN** the limiter output SHALL then pass through the throttle opening rate limit before the
  servo, so on opening the stricter of the two rates SHALL govern the throttle
- **AND** on closing only the ceiling rate limit SHALL slow the fall, the opening rate limit
  passing any lower value through immediately
