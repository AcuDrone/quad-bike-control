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
The system SHALL measure the tacho signal by the PERIOD between consecutive accepted edges,
timestamped in a GPIO interrupt, and SHALL NOT derive RPM by counting pulses per fixed window. The
period SHALL be robust against spurious and missed edges (field revision 1.5.10): edges are filtered
by an absolute floor and an adaptive lock window, and the period is the MEDIAN of recent periods.
The pure arithmetic (period → frequency → RPM, median, edge filter, plausibility, timeout,
pulses-per-revolution validation and snapping, diagnostics formatting) SHALL live in the
Arduino-free header `include/TachoLaw.h` and SHALL be covered by the native Unity suite
`test/test_tacho`.

#### Scenario: Timestamp edges in the interrupt
- **WHEN** `TachoSensor::begin()` is called
- **THEN** an IRAM interrupt SHALL be attached to `PIN_TACHO` on ONE edge (rising)
- **AND** the ISR SHALL record the accepted edge time in microseconds (`esp_timer_get_time()`) into a
  ring buffer of `TACHO_MEDIAN_PERIODS` + 1 (9) edges and count rejected edges, and SHALL do nothing
  else (integer compares only, no floating point in the ISR)

#### Scenario: Reject edges below the absolute floor
- **WHEN** an edge arrives less than `TACHO_MIN_PERIOD_US` (1500 µs) after the last ACCEPTED edge
- **THEN** it SHALL be rejected and counted, whether or not the sensor is locked

#### Scenario: Reject edges inside the lock window
- **WHEN** the sensor is live with a median resting on at least 3 periods
- **AND** an edge arrives less than `TACHO_LOCK_FRACTION` (0.5) × the current median period after the
  last accepted edge
- **THEN** it SHALL be rejected and counted, and the last-accepted reference SHALL NOT move
- **AND** the lock threshold SHALL be written by `update()` to a variable shared with the ISR under
  the same spinlock, 0 meaning "floor only"

#### Scenario: Lock released after a silence
- **WHEN** no accepted edge has arrived for `TACHO_TIMEOUT_MS` (500 ms)
- **THEN** the lock threshold SHALL be reset to 0, by `update()` on the timeout and by the ISR on the
  first edge after the silence, so a restarted engine is never filtered by the old period

#### Scenario: Compute RPM from the median period
- **WHEN** `TachoSensor::update()` runs and at least two accepted edges are buffered
- **THEN** the period SHALL be the median of the last up to `TACHO_MEDIAN_PERIODS` (8) periods (the
  mean of the two middle values for an even count), or the mean of the available periods when fewer
  than 3 exist, and `Hz = 1e6 / period_us`
- **AND** RPM SHALL be `Hz × 60 / ppr`
- **AND** one or two spurious half-length periods, or one double-length period from a missed edge,
  among 8 SHALL NOT change the result
- **AND** a computed RPM above `TACHO_MAX_RPM` (10000) SHALL be discarded as implausible, leaving
  the previous reading
- **AND** `update()` SHALL return without blocking and SHALL copy the ring buffer inside a short
  critical section

#### Scenario: Report zero RPM after the timeout
- **WHEN** no accepted edge has arrived for `TACHO_TIMEOUT_MS` (500 ms)
- **THEN** the tacho frequency and RPM SHALL be reported as 0
- **AND** the period buffer SHALL be cleared, so the first period after a restart is not mixed
  with periods from before the stop
- **AND** the timeout SHALL be evaluated with wrap-safe unsigned arithmetic

#### Scenario: Host-testable math
- **WHEN** `pio test -e native` runs
- **THEN** `test/test_tacho` SHALL run at least 16 cases covering period→RPM at several ppr values,
  the mean, the median (clean, one and two spurious half-periods, one missed edge, the fewer-than-3
  fallback), the edge filter (`tachoEdgeAccepted(sinceUs, lockPeriodUs, fraction, floorUs)`,
  including lock 0), timeout (including `millis()` wrap), the implausible-RPM rejection,
  allowed-ppr validation, snapping, calibration acceptance and each calibration refusal, and RPM
  source selection
- **AND** the suite SHALL be registered as ONE indented line in the `[env:native]` `test_filter`

### Requirement: Tacho Signal Diagnostics
The system SHALL expose enough of the raw tacho signal to diagnose a noisy or lossy wire remotely,
without a scope: the last 8 raw accepted periods, and over a rolling `TACHO_DIAG_WINDOW_MS` (1 s)
window the shortest and longest accepted period and the accepted and rejected edges per second, plus
the rejected edges since boot.

#### Scenario: Raw periods kept
- **WHEN** new accepted edges are copied by `update()`
- **THEN** the periods ending at those edges SHALL be appended to a list of the last 8 raw periods
  (µs, oldest first), which SHALL be kept after the engine stops

#### Scenario: Rolling window statistics
- **WHEN** at least `TACHO_DIAG_WINDOW_MS` has elapsed since the window started
- **THEN** the min / max accepted period in the window (0 when none) and the accepted / rejected
  edge counts normalised to edges per second SHALL be published and a new window SHALL start

#### Scenario: Rate-limited serial log
- **WHEN** a window closes while the tacho is live and `DebugFeature::VEHICLE` is enabled
- **THEN** ONE line `[TACHO] hz=.. rpm=.. med=..us acc=.. rej=.. periods=[..]` SHALL be logged
- **AND** nothing SHALL be logged per edge or while the tacho is not live

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
