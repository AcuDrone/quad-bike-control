## ADDED Requirements
### Requirement: Configurable Speed-Limiter Ceiling Rate
The rate limit on the speed-limiter throttle ceiling SHALL be a persisted, operator-set value in
percent per second. It SHALL default to `SPEED_LIMIT_SLEW_DEFAULT_PCT_S` (80 %/s) and SHALL be
accepted in the range `SPEED_LIMIT_SLEW_MIN_PCT_S`–`SPEED_LIMIT_SLEW_MAX_PCT_S` (10–200 %/s). It
SHALL be stored in NVS namespace `"speed"` under the key `thr_slew` (float, %/s), loaded at
start-up and written immediately when changed. Wherever the speed-limiter requirements name
`SPEED_LIMIT_CEILING_SLEW_PCT_S` — the ceiling rate limit in both directions and the cap on the
trim's fall — the configured rate SHALL be used.

#### Scenario: Default rate on a unit without a stored value
- **WHEN** the system starts and `thr_slew` has never been written
- **THEN** the ceiling rate limit SHALL be 80 %/s, exactly as before this setting existed

#### Scenario: Stored rate is restored
- **WHEN** the system starts and `thr_slew` holds a value within 10–200 %/s
- **THEN** that value SHALL be the ceiling rate limit

#### Scenario: Invalid stored rate is ignored
- **WHEN** the system starts and `thr_slew` holds NaN or a value outside 10–200 %/s
- **THEN** the stored value SHALL be ignored and logged, and the default SHALL be used
- **AND** the stored value SHALL NOT be repaired or rewritten

#### Scenario: Rate changed at runtime
- **WHEN** a valid new rate is set
- **THEN** it SHALL be persisted to `thr_slew` immediately, and the value in use SHALL change
  only after the write succeeded
- **AND** it SHALL take effect on the next limiter evaluation, for the ceiling movement in both
  directions and for the cap on the trim's fall

#### Scenario: A lower rate closes the throttle more gently
- **WHEN** the speed is far above the limit and the rate is 20 %/s instead of 80 %/s
- **THEN** over the same 0.5 s the ceiling SHALL fall by 10 % instead of 40 %
