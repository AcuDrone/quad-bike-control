# speed-sensor Specification

## Purpose
TBD - created by archiving change add-hall-speed-sensor. Update Purpose after archive.
## Requirements
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

### Requirement: Runtime Speed Calibration
The system SHALL make speed calibration runtime-configurable because pulses-per-revolution and wheel
circumference are unknown at build time. It SHALL store `pulses_per_rev` and `wheel_circumference_mm`
in an NVS namespace `"speed"` (Preferences), defaulting from `Constants.h`, and SHALL allow updating
them via web commands routed through `WebPortal::WebCommand` → `VehicleController::processWebCommand`
(the same path as `steer_cal_*`).

#### Scenario: Load calibration on startup
- **WHEN** `SpeedSensor::begin()` is called
- **THEN** `pulses_per_rev` and `wheel_circumference_mm` SHALL be loaded from NVS namespace `"speed"`
- **AND** if no stored values exist, the defaults `SPEED_DEFAULT_PULSES_PER_REV` and
  `SPEED_DEFAULT_WHEEL_CIRCUMFERENCE_MM` from `Constants.h` SHALL be used

#### Scenario: Ignore invalid stored calibration
- **WHEN** `SpeedSensor::begin()` is called and the stored `ppr` is outside
  `SPEED_PPR_MIN`-`SPEED_PPR_MAX` (1-1000), or the stored `circ_mm` is outside
  `SPEED_CIRC_MIN_MM`-`SPEED_CIRC_MAX_MM` (100-10000 mm) or is NaN
- **THEN** that stored value SHALL be ignored and the compile-time default from `Constants.h`
  SHALL be used instead (70 pulses/rev, 1990 mm)
- **AND** the other stored value SHALL still be applied if it is in range

#### Scenario: Set pulses-per-revolution at runtime
- **WHEN** a `speed_cal_ppr` web command is received with a positive integer value
- **THEN** the value SHALL be validated and applied to the sensor
- **AND** SHALL be persisted to NVS namespace `"speed"` immediately so it survives reboots
- **AND** the command SHALL respond with success and SHALL be accepted regardless of the active input
  source (same privilege level as calibration commands)

#### Scenario: Set wheel circumference at runtime
- **WHEN** a `speed_cal_circ` web command is received with a positive circumference in millimetres
- **THEN** the value SHALL be validated and applied to the sensor
- **AND** SHALL be persisted to NVS namespace `"speed"` immediately
- **AND** subsequent speed calculations SHALL use the new circumference without a reflash

#### Scenario: Reject invalid calibration values
- **WHEN** a `speed_cal_ppr` or `speed_cal_circ` command is received with a non-positive or
  out-of-range value
- **THEN** the command SHALL be rejected with an error response
- **AND** the stored calibration SHALL remain unchanged

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

### Requirement: Sensor-Sourced Speed Telemetry
The system SHALL publish hall-sensor speed to web clients independently of CAN bus health, so that
speed is displayed whenever the sensor is live regardless of `can_status`.

#### Scenario: Emit vehicle_speed decoupled from the CAN gate
- **WHEN** a telemetry update is broadcast
- **THEN** the telemetry JSON SHALL include `vehicle_speed` (km/h) sourced from the hall sensor
  **outside** the `can_status == "connected"` conditional block
- **AND** the JSON SHALL include a `speed_valid` boolean reflecting the sensor validity flag
- **AND** `vehicle_speed` SHALL be present even when `can_status` is not "connected"

#### Scenario: Display sensor speed regardless of CAN status
- **WHEN** the web UI receives telemetry containing `vehicle_speed`
- **THEN** the speed value SHALL be displayed regardless of `can_status`
- **AND** when `speed_valid` is false the UI SHALL indicate the reading is unavailable/unhealthy
  rather than showing a misleading 0

### Requirement: Odometer and Trip Distance Accumulation
The system SHALL accumulate the distance the wheel has actually turned into two counters held in
`SpeedSensor`: an **odometer** (`odo_mm`), which SHALL only ever increase over the life of the
vehicle, and a **trip meter** (`trip_mm`), which SHALL accumulate on the same terms until an
operator explicitly resets it. Both SHALL be held as `uint64_t` **millimetres**, so that the
accumulation is exact, cannot overflow in any physical scenario, and is independent of the wheel
calibration in force when it is later read.

Distance SHALL be derived from counted pulses only. In `SpeedSensor::update()`, the increment
SHALL be `delta × distancePerPulseMm_` applied inside the `delta > 0` branch — the same branch and
the same arithmetic that already produce the speed reading — and SHALL be rounded rather than
truncated, so that repeated sample windows do not bias the total low. Direction SHALL NOT be
considered: distance driven in reverse SHALL add exactly as distance driven forward does.

#### Scenario: Accumulate distance from counted pulses
- **WHEN** a sample window is evaluated and a non-zero, plausible pulse `delta` is read from the
  PCNT unit
- **THEN** `delta × distancePerPulseMm_`, rounded to the nearest millimetre, SHALL be added to
  BOTH `odo_mm` and `trip_mm`
- **AND** the addition SHALL happen in the same `delta > 0` branch that computes the speed, so the
  two readings can never disagree about whether the wheel moved

#### Scenario: The decayed estimate contributes no distance
- **WHEN** a sample window sees no edges and the sensor emits its decayed speed estimate, or the
  reading has been latched suspicious after an implausible pulse loss
- **THEN** NEITHER counter SHALL be incremented
- **AND** the resulting under-count during a sensor dropout SHALL be accepted as the correct
  failure direction, because an odometer that keeps counting on a disconnected sensor hides the
  fault, while `isValid()` / `isSuspicious()` already report it

#### Scenario: A rejected wrap-guard sample contributes no distance
- **WHEN** a sample window reads a delta greater than `SPEED_MAX_PULSES_PER_SAMPLE` and is
  discarded as a counter glitch rather than as motion
- **THEN** NEITHER counter SHALL be incremented
- **AND** this SHALL follow from the accumulation being placed AFTER the existing wrap guard's
  early return, not from a second check

#### Scenario: Reverse counts the same as forward
- **WHEN** the vehicle is driven in reverse
- **THEN** both counters SHALL increase, exactly as they do when driving forward
- **AND** NEITHER counter SHALL EVER decrease, because the sensor is unidirectional and a car
  odometer measures distance travelled, not net displacement

#### Scenario: Recalibration does not rewrite recorded distance
- **WHEN** `speed_cal_ppr` or `speed_cal_circ` changes the millimetres-per-pulse at runtime
- **THEN** the already-accumulated `odo_mm` and `trip_mm` SHALL be left untouched
- **AND** only distance travelled AFTER the change SHALL use the new millimetres-per-pulse, so each
  kilometre stays frozen at the calibration that measured it

#### Scenario: The odometer cannot be reset
- **WHEN** any command, web request or MAVLink message is processed
- **THEN** there SHALL be NO code path that zeroes or decreases `odo_mm`
- **AND** a trip reset SHALL leave `odo_mm` exactly as it was

#### Scenario: Expose both counters to the vehicle layer
- **WHEN** the vehicle layer or the telemetry layer asks for the recorded distance
- **THEN** `SpeedSensor` SHALL expose the odometer and the trip distance both in exact millimetres
  and as kilometres in float (`mm × 1e-6`)
- **AND** the float form SHALL be understood as a presentation of the counter, never as the counter
  itself — it SHALL NOT be read back into, re-accumulated from, or used to reconstruct `odo_mm` or
  `trip_mm`

### Requirement: Odometer and Trip Persistence
The system SHALL persist both distance counters across power loss in the EXISTING NVS namespace
`"speed"` — the namespace `SpeedSensor` already owns — under the `uint64` keys `odo_mm` and
`trip_mm`. No new namespace, no new configurable key, and no runtime-tunable write policy SHALL be
introduced: the write interval SHALL be the hardcoded constant `ODO_NVS_WRITE_INTERVAL_MM`
(1 000 000 mm = 1 km) in `Constants.h`.

Writes SHALL occur on exactly three triggers, chosen so that a normal shutdown loses nothing and a
power cut loses at most one kilometre, while the flash write budget stays negligible against the
partition's wear-levelled endurance.

#### Scenario: Load both counters on startup
- **WHEN** `SpeedSensor::begin()` is called
- **THEN** `odo_mm` and `trip_mm` SHALL be read from NVS namespace `"speed"`
- **AND** a key that is absent SHALL read as 0 rather than as an error
- **AND** the restored values SHALL be logged, so the operator can confirm the counters survived
  the power cycle

#### Scenario: Write once per kilometre of odometer growth
- **WHEN** `odo_mm` has grown by at least `ODO_NVS_WRITE_INTERVAL_MM` since the last write
- **THEN** both keys SHALL be written and the "last written" mark SHALL be advanced to the current
  `odo_mm`
- **AND** the loss from an unexpected power cut SHALL therefore be bounded by one kilometre

#### Scenario: Write on ignition OFF
- **WHEN** the ignition state transitions to OFF, on the MAVLink path or the web path, or the
  vehicle enters fail-safe
- **THEN** both keys SHALL be written immediately, so a normal shutdown loses no distance at all
- **AND** the write SHALL happen on the TRANSITION only, not repeatedly while ignition is held OFF

#### Scenario: Write immediately on a trip reset
- **WHEN** a trip reset is performed
- **THEN** `trip_mm` SHALL be zeroed in RAM and both keys SHALL be written immediately
- **AND** `odo_mm` SHALL be written with its unchanged value, so a power cycle cannot resurrect the
  cleared trip

#### Scenario: A failed NVS write never blocks the control loop
- **WHEN** the NVS namespace cannot be opened for writing
- **THEN** the failure SHALL be logged and the call SHALL return
- **AND** the in-RAM counters SHALL remain correct and SHALL continue accumulating
- **AND** the firmware SHALL NOT retry in a loop, block, or reset

#### Scenario: The write schedule stays inside the flash write budget
- **WHEN** the write policy is evaluated over the vehicle's life
- **THEN** it SHALL average no more than one write per kilometre driven plus one per ignition cycle
- **AND** it SHALL NOT write on a wall-clock timer, on every sample window, or while the vehicle is
  stationary
