# speed-sensor Specification

## Purpose
TBD - created by archiving change add-hall-speed-sensor. Update Purpose after archive.
## Requirements
### Requirement: Hall-Effect Speed Sensor Hardware Interface
The system SHALL read vehicle speed from a 3-pin **12V** hall-effect speed sensor (VCC, GND, signal)
connected to the `Control_v0` board's **6N137 opto-isolated input on connector X2**, whose output
reaches the ESP32-S3 on `PIN_SPEED_SENSOR` (GPIO 8). The opto input is current-driven and
galvanically isolated, so it accepts the 12V signal directly with one external series resistor; the
firmware SHALL assume that opto path is present and that its output is **inverted**.

#### Scenario: Signal reaches the GPIO through the opto-isolated input
- **WHEN** the speed sensor signal is connected to connector X2
- **THEN** the signal SHALL pass through the board's opto path — `X2.1 / X2.2` → 470Ω `R22` → 6N137
  LED → open-collector output with pull-up → GPIO 8
- **AND** the input SHALL be routed to `PIN_SPEED_SENSOR` (GPIO 8) — GPIO 1 and GPIO 2 SHALL NOT be
  used, because on `Control_v0` they are the I2C1 bus
- **AND** the Hall channels (X3/X4/X5, including Hall_1 on GPIO 14 / GPIO 21) SHALL remain spare,
  because their ×0.65 divider is scaled for 5V sensors
- **AND** no external level shifter SHALL be required, because isolation and level conversion are
  performed by the optocoupler

#### Scenario: Opto output is inverted relative to the sensor signal
- **WHEN** the sensor drives current through the 6N137 LED
- **THEN** the optocoupler output SHALL pull GPIO 8 **low** (the signal at the GPIO is inverted)
- **AND** pulse counting SHALL be unaffected, because the pulse rate is identical on either edge
- **AND** the firmware SHALL count a single chosen edge (falling or rising) and SHALL record the
  inversion in the PCNT configuration

#### Scenario: LED current must be limited for a 12V sensor
- **WHEN** a 12V sensor is connected to X2
- **THEN** additional series resistance of **560Ω–1kΩ SHALL be fitted in the sensor cable** (giving
  ≈7–10 mA of LED current, within the 6–15 mA working band), or `R22` SHALL be reworked to 1kΩ
- **AND** the stock 470Ω `R22` alone SHALL NOT be used at 12V, because ≈22 mA exceeds the 6N137's
  20 mA absolute maximum LED current
- **AND** the wiring SHALL follow the sensor type: a push-pull 12V sensor connects its signal to
  `X2.1` and its ground to `X2.2` with the series resistor in line; an open-collector (NPN) sensor
  takes +12V through the series resistor to `X2.1` and its output to `X2.2`, sinking the LED current
- **AND** the series resistance SHALL be fitted before the board is first powered with the sensor
  connected

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

