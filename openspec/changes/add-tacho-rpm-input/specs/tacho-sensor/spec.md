## ADDED Requirements
### Requirement: Tachometer Input Hardware Interface
The system SHALL read engine speed from the ECU tachometer output (Delphi MT05 grey connector J1
pin 6, a ≈12 V square wave of ≈50 % duty that the ECU drives with or without the original dash)
through an external PC817 optocoupler, wired as an emitter follower, into Hall_1 channel A on connector X3, which reaches the
ESP32-S3 on `PIN_TACHO` (GPIO14) through the board's 1.2K/2.2K (×0.65) divider and TVS. The crank
position sensor SHALL NOT be tapped.

#### Scenario: Signal path through the optocoupler
- **WHEN** the tacho input is wired
- **THEN** the TACHO wire SHALL drive the PC817 LED through a 1.5 kΩ 0.25 W series resistor (LED
  anode pin 1, cathode pin 2 to vehicle GND), ≈7.5 mA at 12.5 V and ≈9 mA at 14.5 V
- **AND** the PC817 collector (pin 4) SHALL connect to X3 pin 1 (5 V) and its emitter (pin 3) to
  X3 pin 2 (GPIO14), with NO pull-up resistor; X3 pin 4 SHALL be unused
- **AND** the board's 1.2K/2.2K divider SHALL be the emitter load, so GPIO14 reads ≈3.1 V (and at
  least 2.5 V) with the LED on and ≈0 V with it off
- **AND** the board side SHALL be galvanically isolated from the ECU output
- **AND** `PIN_TACHO` SHALL be defined as GPIO14 in `Constants.h`, configured as a plain input with
  no internal pull resistor

#### Scenario: The opto output is not inverted
- **WHEN** the ECU output is high and the LED conducts
- **THEN** GPIO14 SHALL read high (the signal at the GPIO is not inverted); when the LED is off the
  divider SHALL pull GPIO14 low
- **AND** the firmware SHALL time a single chosen edge, so the RPM SHALL NOT depend on the polarity
  or on the duty cycle of the signal at the GPIO

#### Scenario: The crank sensor is not used as an RPM source
- **WHEN** an engine-speed source is wired
- **THEN** the 24-1 crank-wheel Hall sensor SHALL NOT be connected to the board, because any load on
  it risks the ECU's ignition timing
- **AND** it MAY be measured with a separate instrument as a calibration reference
  (`RPM = crank_Hz × 60 / 23`)

### Requirement: Tacho Pulse Period Measurement
The system SHALL measure the tacho signal by the PERIOD between consecutive edges, timestamped in a
GPIO interrupt, and SHALL NOT derive RPM by counting pulses per fixed window. The pure arithmetic
(period → frequency → RPM, plausibility, timeout, pulses-per-revolution validation and snapping)
SHALL live in the Arduino-free header `include/TachoLaw.h` and SHALL be covered by the native
Unity suite `test/test_tacho`.

#### Scenario: Timestamp edges in the interrupt
- **WHEN** `TachoSensor::begin()` is called
- **THEN** an IRAM interrupt SHALL be attached to `PIN_TACHO` on ONE edge (rising)
- **AND** the ISR SHALL record the edge time in microseconds (`micros()` / `esp_timer_get_time()`)
  into a small ring buffer, and SHALL do nothing else
- **AND** an edge arriving less than `TACHO_MIN_PERIOD_US` (1000 µs) after the last accepted edge
  SHALL be ignored as a glitch (software glitch filter)

#### Scenario: Compute RPM from the averaged period
- **WHEN** `TachoSensor::update()` runs and at least two accepted edges are buffered
- **THEN** the frequency SHALL be computed from the mean of the last up to `TACHO_AVG_PERIODS` (4)
  periods as `Hz = 1e6 / mean_period_us`
- **AND** RPM SHALL be `Hz × 60 / ppr`
- **AND** a computed RPM above `TACHO_MAX_RPM` (10000) SHALL be discarded as implausible, leaving
  the previous reading
- **AND** `update()` SHALL return without blocking and SHALL copy the ring buffer inside a short
  critical section

#### Scenario: Report zero RPM after the timeout
- **WHEN** no accepted edge has arrived for `TACHO_TIMEOUT_MS` (500 ms)
- **THEN** the tacho frequency and RPM SHALL be reported as 0
- **AND** the period buffer SHALL be cleared, so the first period after a restart is not averaged
  with periods from before the stop
- **AND** the timeout SHALL be evaluated with wrap-safe unsigned arithmetic

#### Scenario: Host-testable math
- **WHEN** `pio test -e native` runs
- **THEN** `test/test_tacho` SHALL run at least 10 cases covering period→RPM at several ppr values,
  averaging, timeout (including `millis()` wrap), the implausible-RPM rejection, allowed-ppr
  validation, snapping, calibration acceptance and each calibration refusal, and RPM source
  selection
- **AND** the suite SHALL be registered as ONE indented line in the `[env:native]` `test_filter`

### Requirement: Tacho Pulses-per-Revolution Setting
The system SHALL store the tacho pulses-per-revolution `ppr` in NVS namespace `"tacho"`, key `"ppr"`
(float), restricted to the allowed set {0.5, 1, 2, 3, 4}, defaulting to 1.0.

#### Scenario: Load at boot
- **WHEN** `TachoSensor::begin()` is called
- **THEN** `ppr` SHALL be read from NVS
- **AND** an absent key, NaN, or a value not in the allowed set SHALL be ignored and 1.0 used,
  without rewriting NVS

#### Scenario: Apply a new ppr
- **WHEN** a new allowed `ppr` is accepted by a calibration or set command
- **THEN** it SHALL be written to NVS BEFORE being applied, and SHALL be applied to the next RPM
  computation without a reboot
- **AND** if the NVS write fails the previous `ppr` SHALL stay in effect

### Requirement: Tacho Fitted Detection
The system SHALL treat the tacho as FITTED when at least one accepted edge has been seen since boot,
or when a `ppr` key is present in NVS (it was calibrated or set). Fitted SHALL latch for the rest of
the boot. A fitted tacho that is silent SHALL read a valid 0 RPM, not "unknown".

#### Scenario: Fitted by pulses
- **WHEN** the first accepted tacho edge arrives after boot
- **THEN** the tacho SHALL be fitted until the next reboot, even if the pulses later stop

#### Scenario: Fitted by calibration
- **WHEN** the firmware boots with a `ppr` key stored in NVS namespace `"tacho"`
- **THEN** the tacho SHALL be fitted from boot, before any pulse, so a stopped engine reads 0 RPM
  from the tacho rather than from CAN

#### Scenario: Not fitted
- **WHEN** no `ppr` key is stored and no edge has been seen since boot
- **THEN** the tacho SHALL NOT be fitted and SHALL NOT take part in RPM source selection
