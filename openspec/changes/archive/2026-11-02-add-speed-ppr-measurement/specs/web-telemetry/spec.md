## ADDED Requirements
### Requirement: Speed Sensor Pulse Total Telemetry
The telemetry stream SHALL report the speed sensor's running pulse total since boot, so the web
portal can measure pulses per revolution by itself. The firmware SHALL keep no measurement state
and SHALL offer no command for it: the value is read-only and is the same total that speed and
distance are derived from.

#### Scenario: Report the pulse total
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_pulse_total`, the value of `SpeedSensor::getPulseTotal()` as an
  unsigned 32-bit JSON integer
- **AND** the key SHALL be present in every frame, whether or not the speed reading is valid, and
  whatever the CAN status

#### Scenario: The total only increases
- **WHEN** the wheel turns in either direction
- **THEN** every counted falling edge of the sensor signal SHALL increase `speed_pulse_total` by one,
  because the single-channel sensor carries no direction
- **AND** the total SHALL never decrease except by wrapping modulo 2³² or by a reboot, which
  resets it to 0

#### Scenario: The total is independent of speed validity
- **WHEN** pulses are counted while `speed_valid` is false, the reading has decayed to 0 after the
  stale timeout, or a sample is discarded by the per-sample glitch guard
- **THEN** `speed_pulse_total` SHALL still include those pulses

#### Scenario: The firmware stays stateless
- **WHEN** the portal measures pulses per revolution
- **THEN** no web command, NVS key or firmware state SHALL be involved until the operator saves a
  result through the existing `speed_cal_ppr` command

## MODIFIED Requirements
### Requirement: Speed-Limit and Limiter-Ceiling Telemetry
The telemetry stream SHALL report the speed limit the limiter is enforcing and how much throttle
authority the limiter is currently withdrawing. Because there is only one possible source for the
limit, no source key SHALL be reported. Road speeds SHALL be carried through the firmware in
metres per second and converted to km/h only at serialisation.

#### Scenario: Report the enforced limit
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_kmh`, the enforced limit in km/h to one decimal
- **AND** the value SHALL be zero whenever there is no usable `SPEED_MAX`, so zero unambiguously
  means "no limit"

#### Scenario: Report the current throttle ceiling
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL include `speed_limit_ceil`, the throttle ceiling the taper is currently
  applying as a percentage
- **AND** a value of 100 SHALL mean the limiter is withdrawing no authority

#### Scenario: The retired limiter keys are gone
- **WHEN** a telemetry frame is produced
- **THEN** it SHALL NOT include `speed_limit_on`, `speed_limit_max`, `speed_limit_eff`,
  `speed_limit_src` or `mav_speed_max`
- **AND** no web command SHALL exist for setting a local ceiling or toggling the limiter

#### Scenario: Convert to km/h only at the presentation edge
- **WHEN** `vehicle_speed` or `speed_limit_kmh` is serialised
- **THEN** the value SHALL be converted from the firmware's internal metres per second at that
  point
- **AND** `vehicle_speed` SHALL remain a km/h value with one decimal, unchanged on the wire

#### Scenario: Web interface shows the limit and the ceiling
- **WHEN** the web interface renders a telemetry frame
- **THEN** it SHALL display the limit in both km/h and m/s when one is in force, and a "none"
  label when `speed_limit_kmh` is zero
- **AND** the current throttle ceiling SHALL be shown only while the limiter is actually
  withdrawing authority (`speed_limit_ceil` below 100)
- **AND** the help of the "Обмеження швидкості та кермо" / "Speed limit and steering" card (its ⓘ
  button) SHALL state that the limit comes only from the autopilot's `SPEED_MAX`, that zero means
  no limit, and that the limiter does not act without a valid sensor reading
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary

### Requirement: Speed State Card
The Diagnostics section SHALL contain a read-only "Швидкість" / "Speed" card, separate from the
two speed settings cards in the Calibration section ("Датчик швидкості" / "Speed sensor" and
"Обмеження швидкості та кермо" / "Speed limit and steering"), showing the speed sensor state and
the limiter and steering-scaling state without any input or command.

#### Scenario: Read-only speed readout
- **WHEN** the Diagnostics section renders a telemetry frame
- **THEN** the card SHALL show the vehicle speed, the sensor state, the speed limit, the limiting
  row (only while `speed_limit_ceil` is below 100), the steering authority, the scaling base with
  its source, the inactive reason, the bench test-speed row (only while an override is in force),
  the odometer and the trip
- **AND** the card SHALL contain no input field, button or other control that sends a command

#### Scenario: Same values in both places
- **WHEN** a value is shown both in this card and in one of the two speed settings cards
- **THEN** both SHALL be rendered from the same telemetry frame by one rendering path, so they never
  disagree
