## MODIFIED Requirements

### Requirement: Hall Pulse Counting via PCNT
The system SHALL count hall-sensor pulses using the ESP32-S3 PCNT hardware pulse counter with a
hardware glitch filter, and SHALL derive frequency by sampling the accumulated count on a fixed
interval from the cooperative `update()` loop without using interrupts or a FreeRTOS task. The
derived speed SHALL be expressed in metres per second, the firmware's internal speed unit.

Because the input stage is active LOW, the counted edge SHALL be the **falling** one. The pulse
rate is identical on either edge, so this is a documentation choice rather than a correctness one,
and the specification fixes it so that the polarity, the constant and the log line agree.

#### Scenario: Configure PCNT on initialization
- **WHEN** `SpeedSensor::begin()` is called
- **THEN** a PCNT unit SHALL be configured on `PIN_SPEED_SENSOR` counting exactly ONE edge: the
  falling edge SHALL increment and the rising edge SHALL hold
- **AND** the hardware glitch filter SHALL be enabled using `SPEED_GLITCH_FILTER_NS`
- **AND** counter overflow SHALL be accumulated so that fast pulse trains between samples do not
  lose counts
- **AND** a failure to apply the glitch filter SHALL abort `begin()` with a logged error rather
  than running unfiltered, so the boot line is proof of the value actually in force

#### Scenario: The glitch filter is set to the hardware maximum
- **WHEN** `SPEED_GLITCH_FILTER_NS` is chosen
- **THEN** it SHALL be 12500 ns (12.5 µs) — 1000 APB cycles at 80 MHz, inside the ESP32-S3's
  1023-cycle `PCNT_LL_MAX_GLITCH_WIDTH`, and therefore the largest round value the driver accepts
- **AND** it SHALL be justified by the signal it must not damage: at the vehicle's 80 km/h maximum
  the pulse train is ≈782 Hz, a ~639 µs half-period, some 50x wider than the filter
- **AND** every pulse narrower than 12.5 µs SHALL be rejected in hardware, which is where the
  ignition-coil ringing lives

#### Scenario: Derive speed from sampled pulse count
- **WHEN** `SpeedSensor::update()` is called and a `SPEED_SAMPLE_INTERVAL_MS` window has elapsed
- **THEN** the delta pulse count over the window SHALL be read from the PCNT unit
- **AND** speed SHALL be computed using `distance_per_pulse = wheel_circumference_mm / pulses_per_rev`
  and the elapsed time, and exposed via `getSpeedMs()`
- **AND** the result SHALL be taken directly as metres per second, because millimetres per
  millisecond is metres per second by definition — no unit conversion SHALL be applied
- **AND** `update()` SHALL return without blocking the main loop

#### Scenario: A window above the physical pulse ceiling is discarded whole
- **WHEN** a sample window reads a delta greater than `SPEED_MAX_PULSES_PER_SAMPLE`
- **THEN** the whole window SHALL be discarded: no speed SHALL be derived from it and no distance
  SHALL be accumulated
- **AND** `SPEED_MAX_PULSES_PER_SAMPLE` SHALL be 240 pulses per 200 ms window, derived from the
  measured calibration (70 pulses/rev, 1990 mm ⇒ 28.4 mm/pulse) at the vehicle's 80 km/h maximum —
  ≈156 pulses per window — with a 1.5x margin for calibration error and a downhill overrun
- **AND** the discard SHALL be understood as rejecting a noise burst or a counter glitch, not
  motion, because the rate it represents (≈120 km/h) is not reachable by this vehicle

## ADDED Requirements

### Requirement: Opto-Isolated Hall Speed Sensor Input Stage
The system SHALL read vehicle speed from a 12V toothed-ring hall pickup that reaches
`PIN_SPEED_SENSOR` (GPIO 17) **through a PC817 opto-coupler**, NOT directly. The wiring is
**confirmed on the bench (2026-09-29)** and is fixed by this specification: pickup output → 1 kΩ
series resistor → PC817 LED; PC817 phototransistor collector → `PIN_SPEED_SENSOR`, emitter → GND.

The opto stage performs BOTH jobs the ESP32 needs. It **level-shifts** — nothing at 12V ever
reaches the pin, so the pin's lack of 12V tolerance is answered by the hardware and not by a
pending action — and it **inverts**: the signal at the pin is **active LOW** (LED conducting ⇒ pin
pulled low). The firmware SHALL therefore make an explicit polarity assumption rather than treating
the pin as an uncommitted digital input.

#### Scenario: The signal reaches the GPIO through the opto stage
- **WHEN** the speed sensor is connected
- **THEN** its 12V output SHALL drive the PC817 LED through a 1 kΩ series resistor, and the PC817
  phototransistor collector SHALL be the only thing connected to `PIN_SPEED_SENSOR` (GPIO 17),
  with its emitter on GND
- **AND** GPIO 17 SHALL be the pin freed when the BTS7960 steering driver was removed (formerly
  `PIN_STEER_RPWM`)
- **AND** GPIO 18 SHALL remain free/reserved and SHALL NOT be used for the sensor

#### Scenario: The opto stage level-shifts, so nothing 12V reaches the pin
- **WHEN** the 12V pickup is energised
- **THEN** the only 12V-side current path SHALL be through the series resistor and the PC817 LED
- **AND** the ESP32 side SHALL see only the phototransistor's collector, referenced to 3.3V through
  the pull-up below
- **AND** no additional external level shifter SHALL be required or fitted, because the opto
  already provides the isolation and the level shift

#### Scenario: The signal at the pin is active LOW
- **WHEN** the pickup's active phase drives the PC817 LED into conduction
- **THEN** the phototransistor SHALL pull `PIN_SPEED_SENSOR` LOW
- **AND** the idle state between pulses SHALL be HIGH, held by the pull-up
- **AND** the firmware SHALL count the **falling** edge, i.e. the start of the active phase, and
  SHALL document that choice rather than treating the two edges as interchangeable

#### Scenario: An external 1 kΩ pull-up to 3.3V is mandatory
- **WHEN** the input stage is built or rebuilt
- **THEN** an external 1 kΩ pull-up resistor from `PIN_SPEED_SENSOR` to 3.3V SHALL be fitted
- **AND** the ESP32-S3 internal pull-up (~45 kΩ, enabled by `pcnt_new_channel()`) SHALL NOT be
  relied on as the only pull-up, because it leaves a high-impedance node between pulses
- **AND** this SHALL be treated as a hardware requirement, not a recommendation: the firmware
  cannot detect its absence and SHALL NOT attempt to compensate for it

#### Scenario: The pull-up is what silences ignition-coil pickup
- **WHEN** the first build ran with the internal pull-up alone
- **THEN** ignition-coil interference SHALL be understood to have been counted as phantom pulses,
  producing a non-zero speed on a stationary vehicle and tripping the gear-change speed interlock
- **AND** with the 1 kΩ pull-up fitted the noise diagnostics counters SHALL read 0 with the engine
  idling, which is the acceptance criterion for the input stage

### Requirement: Speed Sensor Noise Diagnostics
The system SHALL expose enough of the raw counting behaviour for interference to be **measured**
rather than inferred from a wrong speed reading, because the failure this input stage is hardened
against — ignition-coil pickup counted as phantom pulses — is otherwise invisible until it trips
the gear interlock.

Every counter in this requirement SHALL be an observation only. NONE of them SHALL feed the speed
calculation, the distance accumulation, the validity flag or any control decision; removing them
SHALL change no vehicle behaviour.

#### Scenario: Expose the raw per-window pulse count
- **WHEN** a sample window is evaluated
- **THEN** the delta exactly as the hardware reported it, BEFORE the ceiling guard and before any
  interpretation, SHALL be retained and exposed
- **AND** a non-zero value on a stationary vehicle SHALL be readable as the interference itself,
  measured

#### Scenario: Count isolated stray pulses
- **WHEN** a window holds fewer than three edges AND the speed it resolves to is below
  `TRANS_SPEED_INTERLOCK_THRESHOLD_MS`
- **THEN** those pulses SHALL be added to a running stray-pulse counter
- **AND** they SHALL still be integrated into speed and distance exactly as before, because this is
  a heuristic fingerprint of coupling and not a rejection rule
- **AND** the heuristic SHALL rest on a turning wheel producing a continuous train, so an isolated
  edge or two resolving below walking pace is far more likely to be stray coupling than motion

#### Scenario: Count rejected windows and the pulses in them
- **WHEN** a window is discarded for exceeding `SPEED_MAX_PULSES_PER_SAMPLE`
- **THEN** a rejected-windows counter SHALL be incremented and the discarded pulse count SHALL be
  added to a rejected-pulses counter
- **AND** the event SHALL be marked pending so the rate limiter below cannot swallow it entirely

#### Scenario: Publish the counters in web telemetry
- **WHEN** a telemetry update is broadcast
- **THEN** the telemetry JSON SHALL include `speed_raw_pulses`, `speed_stray_pulses` and
  `speed_rej_windows`
- **AND** they SHALL be present regardless of `can_status` and regardless of the sensor validity
  flag, because they are the evidence needed exactly when the reading is wrong

#### Scenario: Log a rate-limited diagnostics line
- **WHEN** a sample window has seen at least one raw edge, or a rejected window is still pending
- **THEN** a `[SPEED] window:` line carrying the raw count, the derived speed and the stray,
  rejected-window and rejected-pulse totals SHALL be emitted on the VEHICLE debug feature
- **AND** it SHALL be rate limited to at most once per `SPEED_NOISE_LOG_INTERVAL_MS` (2 s), rather
  than once per 200 ms sample
- **AND** a pending rejected window SHALL survive the rate limit and be reported by the next line

#### Scenario: Stay silent on a parked vehicle
- **WHEN** a sample window sees no raw edges and no rejected window is pending
- **THEN** no diagnostics line SHALL be emitted
- **AND** silence on the console SHALL therefore mean a clean input, which is the state a correctly
  built input stage holds with the engine idling

## REMOVED Requirements

### Requirement: Hall-Effect Speed Sensor Hardware Interface
**Reason**: Every load-bearing claim in it is now false. It described the sensor signal reaching
GPIO 17 **directly**, asserted that "there is no opto-isolated input stage", declined to assume a
signal polarity, left the wiring "to be confirmed on the bench", and demanded external level
shifting as a pending action. The bench confirmed the real input stage on 2026-09-29 and the
firmware was updated to match (commits `49ba19c`, `30c28af`): a PC817 opto-coupler that both
level-shifts and inverts, with a mandatory external 1 kΩ pull-up. Both of its scenarios
("Signal reaches the GPIO directly", "A 12V sensor must be level-shifted") are wrong at the level
of their names, so the requirement is removed and replaced rather than edited in place.

**Migration**: Replaced in full by the ADDED requirement
`Opto-Isolated Hall Speed Sensor Input Stage`, which carries forward the two facts that are still
true — GPIO 17 is the pin freed when the BTS7960 steering driver was removed, and GPIO 18 stays
free/reserved — and fixes the wiring, the polarity and the pull-up as normative. No hardware or
firmware action follows from this change: the vehicle is already wired and flashed this way.
