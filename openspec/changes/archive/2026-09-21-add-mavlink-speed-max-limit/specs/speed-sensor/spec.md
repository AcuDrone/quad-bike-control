## MODIFIED Requirements

### Requirement: Hall Pulse Counting via PCNT
The system SHALL count hall-sensor pulses using the ESP32-S3 PCNT hardware pulse counter with a
configurable glitch filter, and SHALL derive frequency by sampling the accumulated count on a fixed
interval from the cooperative `update()` loop without using interrupts or a FreeRTOS task. The
derived speed SHALL be expressed in metres per second, the firmware's internal speed unit.

#### Scenario: Configure PCNT on initialization
- **WHEN** `SpeedSensor::begin()` is called
- **THEN** a PCNT unit SHALL be configured on `PIN_SPEED_SENSOR` to count a single edge (falling or
  rising — either yields the same pulse rate through the inverting optocoupler)
- **AND** the hardware glitch filter SHALL be enabled using `SPEED_GLITCH_FILTER_NS`
- **AND** counter overflow SHALL be accumulated so that fast pulse trains between samples do not lose
  counts

#### Scenario: Derive speed from sampled pulse count
- **WHEN** `SpeedSensor::update()` is called and a `SPEED_SAMPLE_INTERVAL_MS` window has elapsed
- **THEN** the delta pulse count over the window SHALL be read from the PCNT unit
- **AND** speed SHALL be computed using `distance_per_pulse = wheel_circumference_mm / pulses_per_rev`
  and the elapsed time, and exposed via `getSpeedMs()`
- **AND** the result SHALL be taken directly as metres per second, because millimetres per
  millisecond is metres per second by definition — no unit conversion SHALL be applied
- **AND** `update()` SHALL return without blocking the main loop

### Requirement: Speed Signal Validity and Timeout
The system SHALL distinguish "vehicle stopped" from "sensor unhealthy". A silent sensor SHALL report
0 m/s, and a separate validity signal SHALL indicate whether the reading can be trusted, so that
each consumer can apply its own fail-safe policy.

#### Scenario: Report zero speed when no pulses arrive
- **WHEN** no hall pulses have been counted for `SPEED_STALE_TIMEOUT_MS`
- **THEN** the reported speed SHALL decay to 0 m/s

#### Scenario: Decay the reading between pulses rather than holding it
- **WHEN** a sample window sees no edges and the stale timeout has not yet elapsed
- **THEN** the reported speed SHALL be reduced to at most the speed still reachable from the last
  observed speed at `SPEED_MAX_PLAUSIBLE_DECEL_MS2`, rather than held at its last value
- **AND** the plausibility check SHALL be evaluated on every such sample, not only at the stale
  timeout, so a mid-motion wire fault is latched as soon as it is detectable

#### Scenario: Validity is false until the sensor has produced pulses
- **WHEN** the system boots and no plausible pulse has yet been counted
- **THEN** `isValid()` SHALL return false
- **AND** `isValid()` SHALL become true after at least one plausible pulse is counted

#### Scenario: Flag implausible pulse loss as suspicious
- **WHEN** the vehicle was recently moving above `TRANS_SPEED_INTERLOCK_THRESHOLD_MS` and pulses
  cease faster than a physically plausible deceleration
- **THEN** the reading SHALL be flagged suspicious (`isValid()` returns false)
- **AND** consumers SHALL treat the speed as unknown rather than as a genuine 0 m/s, except the
  transmission interlock, which MAY use the decaying reading as an upper bound

## REMOVED Requirements

### Requirement: Configurable Maximum-Speed Throttle Limiter
**Reason**: The limiter no longer has any local configuration. Its ceiling comes solely from the
autopilot's `SPEED_MAX` parameter (see `vehicle-systems` → "Autopilot-Sourced Maximum-Speed
Limit"), it is always armed, and "no usable `SPEED_MAX`" means no limiting rather than a fallback
to a stored value. Two places to set one ceiling made the enforced number unknowable from the
ground station, and an invisible stored ceiling taking over on a dropped link was the dangerous
failure direction.

**Migration**: The NVS keys `lim_on` and `lim_kmh` in namespace `"speed"` are deleted on
`SpeedSensor::begin()` and are not replaced. The web commands `speed_limit_enable` and
`speed_limit_set` are removed, as are the web portal's maximum-speed input and limiter toggle.
Operators set the limit with the autopilot parameter `SPEED_MAX` (m/s); `0` means no limit.
Setting `SPEED_MAX` becomes a required commissioning step, because a vehicle whose autopilot has
no `SPEED_MAX` is now unlimited where it was previously held at the stored ceiling. The
fail-open-on-invalid-sensor behaviour and the gear-boost exclusion are retained and restated in
`vehicle-systems` → "Proportional Speed-Limiter Throttle Taper".
