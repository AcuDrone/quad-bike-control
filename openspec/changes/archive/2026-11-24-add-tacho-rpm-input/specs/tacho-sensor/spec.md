## ADDED Requirements
### Requirement: Tachometer Input Hardware Interface
The system SHALL read engine speed from the ECU tachometer output (Delphi MT05 grey connector J1
pin 6, a ≈12 V square wave of ≈50 % duty that the ECU drives with or without the original dash)
through an external PC817 optocoupler, wired as an emitter follower, into Hall_1 channel A on connector X3, which reaches the
ESP32-S3 on `PIN_TACHO` (GPIO14) through the board's 1.2K/2.2K (×0.65) divider and TVS. The crank
position sensor SHALL NOT be tapped.

#### Scenario: Signal path through the optocoupler
- **WHEN** the tacho input is wired
- **THEN** the TACHO wire SHALL drive the PC817 LED through a 1 kΩ 0.25 W series resistor (LED
  anode pin 1, cathode pin 2 to vehicle GND), ≈11 mA at 12.5 V and ≈13 mA at 14.5 V (field revision
  1.5.11; 1.5 kΩ before, too little margin on the ≈3.0 V high level)
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
- **AND** the firmware SHALL time the period between validated RISING edges; the pulse-width check
  (high and low each ≥ `TACHO_MIN_LEVEL_US`) relies on the ≈50 % duty of the ECU signal

#### Scenario: The crank sensor is not used as an RPM source
- **WHEN** an engine-speed source is wired
- **THEN** the 24-1 crank-wheel Hall sensor SHALL NOT be connected to the board, because any load on
  it risks the ECU's ignition timing
- **AND** it MAY be measured with a separate instrument as a calibration reference
  (`RPM = crank_Hz × 60 / 23`)

### Requirement: Tacho Pulse Period Measurement
The system SHALL measure the tacho signal by the PERIOD between consecutive accepted edges,
timestamped in a GPIO interrupt, and SHALL NOT derive RPM by counting pulses per fixed window. The
period SHALL be robust against spurious and missed edges: edges are validated by pulse width (field
revision 1.5.11 — a rising edge counts only when the low before it and the high after it each last at
least `TACHO_MIN_LEVEL_US`; this replaces the 1.5.10 lock window) behind an absolute sanity floor,
and the period is the MEDIAN of recent periods (field revision 1.5.10).
The pure arithmetic (period → frequency → RPM, median, edge validator state machine, plausibility, timeout,
pulses-per-revolution validation and snapping, diagnostics formatting) SHALL live in the
Arduino-free header `include/TachoLaw.h` and SHALL be covered by the native Unity suite
`test/test_tacho`.

#### Scenario: Timestamp both edges in the interrupt
- **WHEN** `TachoSensor::begin()` is called
- **THEN** the validator state SHALL be initialised from the pin level and an IRAM interrupt SHALL be
  attached to `PIN_TACHO` on BOTH edges (CHANGE)
- **AND** the ISR SHALL read the pin level with an IRAM-safe direct register read, timestamp the edge
  in microseconds (`esp_timer_get_time()`), feed the pure integer state machine
  `tachoEdgeStep(state, level, nowUs, minLevelUs)` from `TachoLaw.h` (result NONE, ACCEPTED_RISE or
  GLITCH), store only ACCEPTED rise times into a ring buffer of `TACHO_MEDIAN_PERIODS` + 1 (9) edges
  and count rejected edges, with no floating point in the ISR

#### Scenario: Accept a rising edge only after a long low and a long high
- **WHEN** a rising edge follows a low lasting at least `TACHO_MIN_LEVEL_US` (2000 µs)
- **THEN** it SHALL become a candidate, and SHALL be accepted (with its own rise time) only at a
  following falling edge that ends a high lasting at least `TACHO_MIN_LEVEL_US`, at most once per
  pulse
- **AND** a candidate whose high ended sooner and that is followed by a low of at least
  `TACHO_MIN_LEVEL_US` SHALL be rejected and counted (a narrow pulse)

#### Scenario: Reject bounce and glitches
- **WHEN** a rising edge follows a low shorter than `TACHO_MIN_LEVEL_US` (bounce on the falling edge,
  or a glitch low inside a high pulse)
- **THEN** it SHALL be rejected and counted, and the current high pulse SHALL continue (its rise time
  and accepted state unchanged), so the period SHALL NOT change
- **AND** an interrupt that reads the same level as the previous one SHALL be ignored

#### Scenario: Reject accepted rises below the sanity floor
- **WHEN** an accepted rise is less than `TACHO_MIN_PERIOD_US` (1500 µs) after the previous accepted
  rise
- **THEN** it SHALL be rejected and counted

#### Scenario: Fresh start after a silence
- **WHEN** no accepted edge has arrived for `TACHO_TIMEOUT_MS` (500 ms)
- **THEN** a low in progress SHALL count as long (the first rise after the silence is a candidate) and
  a candidate stuck high SHALL be dropped, so a stale rise time never enters the ring
- **AND** the first accepted rise after the silence SHALL start a fresh buffer

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
  fallback), the edge validator `tachoEdgeStep` (clean 50 % square, falling-edge bounce, glitch low
  inside a high pulse, narrow high pulse, first edge after boot / silence, µs wrap-around), timeout (including `millis()` wrap), the implausible-RPM rejection,
  allowed-ppr validation, snapping, calibration acceptance and each calibration refusal, and RPM
  source selection
- **AND** the suite SHALL be registered as ONE indented line in the `[env:native]` `test_filter`

### Requirement: Tacho Signal Diagnostics
The system SHALL expose enough of the raw tacho signal to diagnose a noisy or lossy wire remotely,
without a scope: the last 8 raw accepted periods, the last high span and the last validated low time,
and over a rolling `TACHO_DIAG_WINDOW_MS` (1 s) window the shortest and longest accepted period, the
shortest and longest high span and the accepted and rejected edges per second, plus the rejected
edges since boot.

#### Scenario: Raw periods kept
- **WHEN** new accepted edges are copied by `update()`
- **THEN** the periods ending at those edges SHALL be appended to a list of the last 8 raw periods
  (µs, oldest first), which SHALL be kept after the engine stops

#### Scenario: Rolling window statistics
- **WHEN** at least `TACHO_DIAG_WINDOW_MS` has elapsed since the window started
- **THEN** the min / max accepted period in the window (0 when none), the min / max high span in the
  window (0 when none) and the accepted / rejected edge counts normalised to edges per second SHALL be
  published and a new window SHALL start

#### Scenario: Rate-limited serial log
- **WHEN** a window closes while the tacho is live and `DebugFeature::VEHICLE` is enabled
- **THEN** ONE line `[TACHO] hz=.. rpm=.. med=..us acc=.. rej=.. hi=..ms lo=..ms hi_min/max=../..ms periods=[..]`
  SHALL be logged
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
