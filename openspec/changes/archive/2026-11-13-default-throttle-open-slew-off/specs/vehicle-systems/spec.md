## MODIFIED Requirements
### Requirement: Configurable Throttle Opening Rate
The rate at which the throttle may OPEN SHALL be a persisted, operator-set, per-vehicle value in
percent per second, applied by `ThrottleController::setThrottlePercent()` through the pure law
`throttleSlewStep(targetPct, lastPct, dtS, ratePctS)` (`include/ThrottleSlewLaw.h`). A value of 0
SHALL turn the limit off; any other accepted value SHALL lie in
`THROTTLE_OPEN_SLEW_MIN_PCT_S`–`THROTTLE_OPEN_SLEW_MAX_PCT_S` (30–100 %/s). The default SHALL be
`THROTTLE_OPEN_SLEW_DEFAULT_PCT_S` = 0 (off) on every vehicle; a non-zero rate is optional and
set by the operator (for example 60 %/s, idle to full in ≈1.67 s).
It SHALL be stored in the throttle calibration NVS namespace `"throttle"` under the key
`open_slew` (float, %/s), loaded at start-up and written only when the operator sets a new value;
the firmware SHALL NOT write it at start-up, on first run or when the default is used. Closing the
throttle SHALL never be rate-limited.

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
- **THEN** the opening rate SHALL be 0 (off) and the throttle SHALL open without a ramp
- **AND** `open_slew` SHALL still be absent afterwards

#### Scenario: Stored rate is restored
- **WHEN** the system starts and `open_slew` holds 0 or a value within 30–100 %/s
- **THEN** that value SHALL be the opening rate
- **AND** this SHALL hold after a firmware update that changed the default, so a unit on which
  60 %/s was saved keeps 60 %/s until the operator sets another value

#### Scenario: Invalid stored rate is ignored
- **WHEN** the system starts and `open_slew` holds NaN, a negative value, a value between 0 and
  30 %/s exclusive, or a value above 100 %/s
- **THEN** the stored value SHALL be ignored and logged, and the default (0, off) SHALL be used
- **AND** the stored value SHALL NOT be repaired or rewritten

#### Scenario: Rate changed at runtime
- **WHEN** a valid new rate is set
- **THEN** it SHALL be persisted to `open_slew` immediately, and the value in use SHALL change only
  after the write succeeded
- **AND** it SHALL take effect on the next percentage command

#### Scenario: Snap from idle on the bench
- **WHEN** the rate is 60 %/s and the throttle is commanded from idle to 100 % in one step
- **THEN** the servo SHALL take ≈1.7 s to reach full throttle
- **AND** with the rate at 0 (the default) the servo SHALL snap open with no ramp
