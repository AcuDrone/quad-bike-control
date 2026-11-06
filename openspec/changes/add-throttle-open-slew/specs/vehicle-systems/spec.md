## MODIFIED Requirements
### Requirement: Throttle Control System
The system SHALL provide throttle control through a servo-driven acceleration mechanism. Throttle
percent SHALL map linearly from the calibrated idle pulse width (0 %) to the calibrated full pulse
width (100 %); the two endpoints MAY be in either order so that a mirrored servo mount (full below
idle) is supported. Every percentage command SHALL pass through the throttle opening rate limit
(see "Configurable Throttle Opening Rate") at the servo input, before the percent-to-µs mapping:
opening SHALL be rate-limited, closing SHALL never be slowed.

#### Scenario: Set throttle position by percentage
- **WHEN** ThrottleSystem.setPosition() is called with value from 0 to 100
- **THEN** throttle servo moves to corresponding position, rising no faster than the configured
  opening rate
- **AND** 0% maps to idle/closed throttle
- **AND** 100% maps to full throttle

#### Scenario: Return to idle
- **WHEN** ThrottleSystem.idle() is called
- **THEN** throttle servo returns to 0% position immediately, without any rate limit
- **AND** throttle remains at idle until new command
- **AND** the opening rate limit's last commanded percentage SHALL be reset to 0, so the next
  opening ramps from idle

#### Scenario: Throttle safety override
- **WHEN** brake system is engaged above threshold (e.g., >50%)
- **THEN** throttle is automatically reduced to idle
- **AND** throttle commands are ignored until brakes release

#### Scenario: Inverted servo calibration accepted
- **WHEN** a throttle calibration is saved with idle = 2200 µs and full = 800 µs
- **THEN** the calibration SHALL be accepted and persisted to NVS
- **AND** 0 % SHALL command 2200 µs, 50 % 1500 µs and 100 % 800 µs
- **AND** the reported throttle percent SHALL be 0..100 for pulse widths between the endpoints and
  clamped to 0 / 100 outside them
- **AND** the save log SHALL mark the pair as inverted
- **AND** after a reboot the stored inverted pair SHALL be loaded, not replaced by the defaults

#### Scenario: Span check applies to absolute difference
- **WHEN** a throttle calibration is saved or loaded from NVS
- **THEN** it SHALL be accepted only if both endpoints lie within `THROTTLE_SERVO_MIN_US` ..
  `THROTTLE_SERVO_MAX_US` and `|full - idle| >= THROTTLE_MIN_SPAN_US`, regardless of which endpoint
  is larger
- **AND** a rejected save SHALL report "Idle and full must differ by >= 100us"
- **AND** an invalid stored pair SHALL fall back to the default endpoints

#### Scenario: Opening is rate-limited at the servo input
- **WHEN** the opening rate is 60 %/s, the last commanded throttle is 0 % and percentage commands
  of 100 % arrive every control-loop iteration
- **THEN** the commanded throttle SHALL rise by at most 60 %/s × the elapsed time per iteration,
  reaching 100 % after ≈1.67 s
- **AND** this SHALL hold identically for the MAVLink throttle, the web `set_throttle` command, the
  per-loop web re-limit and the speed-limiter output, because all of them reach the servo through
  the same percentage command

#### Scenario: Closing is immediate
- **WHEN** a percentage command is lower than or equal to the last commanded percentage
- **THEN** it SHALL be applied in the same call, whatever the opening rate

#### Scenario: Elapsed time is bounded
- **WHEN** a percentage command arrives after a gap since the previous throttle write
- **THEN** the elapsed time used by the opening rate limit SHALL be capped at
  `THROTTLE_OPEN_SLEW_MAX_DT_S` (0.1 s), and the first command after start-up SHALL use zero
- **AND** a stalled loop or a path that was silent for seconds SHALL therefore never open the
  throttle in one step

#### Scenario: Direct pulse-width writes bypass the opening limit
- **WHEN** the gear-change boost PID writes the throttle in microseconds
- **THEN** the opening rate limit SHALL NOT be applied, the PID keeping its own
  `TRANS_GEAR_BOOST_SLEW_RATE_US` slew
- **AND** the opening limit's last commanded percentage SHALL be re-seeded from the written pulse
  width, so a later percentage command ramps from the servo's real position
- **AND** the throttle calibration jog SHALL also drive the servo directly, and saving or
  cancelling the calibration SHALL reset the last commanded percentage to 0

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

#### Scenario: The ceiling slew and the opening rate limit stack
- **WHEN** a limit is in force and the ceiling is rising at the ceiling rate while the demand is
  above it
- **THEN** the limiter output SHALL then pass through the throttle opening rate limit before the
  servo, so on opening the stricter of the two rates SHALL govern the throttle
- **AND** on closing only the ceiling rate limit SHALL slow the fall, the opening rate limit
  passing any lower value through immediately

## ADDED Requirements
### Requirement: Configurable Throttle Opening Rate
The rate at which the throttle may OPEN SHALL be a persisted, operator-set, per-vehicle value in
percent per second, applied by `ThrottleController::setThrottlePercent()` through the pure law
`throttleSlewStep(targetPct, lastPct, dtS, ratePctS)` (`include/ThrottleSlewLaw.h`). A value of 0
SHALL turn the limit off; any other accepted value SHALL lie in
`THROTTLE_OPEN_SLEW_MIN_PCT_S`–`THROTTLE_OPEN_SLEW_MAX_PCT_S` (30–100 %/s). The default SHALL be
`THROTTLE_OPEN_SLEW_DEFAULT_PCT_S` (60 %/s, idle to full in ≈1.67 s). It SHALL be stored in the
throttle calibration NVS namespace `"throttle"` under the key `open_slew` (float, %/s), loaded at
start-up and written immediately when changed. Closing the throttle SHALL never be rate-limited.

#### Scenario: The law limits rising and passes falling
- **WHEN** `throttleSlewStep` is called with a rate above 0
- **THEN** a target above `lastPct` SHALL return `min(target, lastPct + rate × dt)`
- **AND** a target at or below `lastPct` SHALL return the target unchanged
- **AND** the target SHALL be clamped to 0–100 % first

#### Scenario: Rate 0 is off
- **WHEN** the opening rate is 0
- **THEN** every percentage command SHALL reach the servo unchanged, exactly as before this
  setting existed

#### Scenario: Default rate on a unit without a stored value
- **WHEN** the system starts and `open_slew` has never been written
- **THEN** the opening rate SHALL be 60 %/s

#### Scenario: Stored rate is restored
- **WHEN** the system starts and `open_slew` holds 0 or a value within 30–100 %/s
- **THEN** that value SHALL be the opening rate

#### Scenario: Invalid stored rate is ignored
- **WHEN** the system starts and `open_slew` holds NaN, a negative value, a value between 0 and
  30 %/s exclusive, or a value above 100 %/s
- **THEN** the stored value SHALL be ignored and logged, and the default SHALL be used
- **AND** the stored value SHALL NOT be repaired or rewritten

#### Scenario: Rate changed at runtime
- **WHEN** a valid new rate is set
- **THEN** it SHALL be persisted to `open_slew` immediately, and the value in use SHALL change only
  after the write succeeded
- **AND** it SHALL take effect on the next percentage command

#### Scenario: Snap from idle on the bench
- **WHEN** the rate is 60 %/s and the throttle is commanded from idle to 100 % in one step
- **THEN** the servo SHALL take ≈1.7 s to reach full throttle and the engine SHALL not stall
- **AND** with the rate set to 0 the servo SHALL snap open as before
