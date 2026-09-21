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

