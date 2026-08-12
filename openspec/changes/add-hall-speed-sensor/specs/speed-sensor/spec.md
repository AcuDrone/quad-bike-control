## ADDED Requirements

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
interval from the cooperative `update()` loop without using interrupts or a FreeRTOS task.

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
  and the elapsed time, and exposed via `getSpeedKmh()`
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
0 km/h, and a separate validity signal SHALL indicate whether the reading can be trusted, so that
each consumer can apply its own fail-safe policy.

#### Scenario: Report zero speed when no pulses arrive
- **WHEN** no hall pulses have been counted for `SPEED_STALE_TIMEOUT_MS`
- **THEN** the reported speed SHALL decay to 0 km/h

#### Scenario: Validity is false until the sensor has produced pulses
- **WHEN** the system boots and no plausible pulse has yet been counted
- **THEN** `isValid()` SHALL return false
- **AND** `isValid()` SHALL become true after at least one plausible pulse is counted

#### Scenario: Flag implausible pulse loss as suspicious
- **WHEN** the vehicle was recently moving above the interlock threshold and pulses cease faster than
  a physically plausible deceleration
- **THEN** the reading SHALL be flagged suspicious (`isValid()` returns false)
- **AND** consumers SHALL treat the speed as unknown rather than as a genuine 0 km/h

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

### Requirement: Configurable Maximum-Speed Throttle Limiter
The system SHALL provide a runtime-configurable maximum-speed throttle limiter that reduces throttle
authority above a settable speed. The limiter SHALL be disabled by default and SHALL fail safe on
loss of a valid speed reading.

#### Scenario: Configure the limiter at runtime
- **WHEN** a `speed_limit_enable` command (boolean) or a `speed_limit_set` command (maximum km/h) is
  received via the web command path
- **THEN** the setting SHALL be validated and persisted to NVS namespace `"speed"`
- **AND** the limiter SHALL default to disabled (`SPEED_LIMIT_ENABLE_DEFAULT` = false) until enabled

#### Scenario: Reduce throttle authority above the maximum speed
- **WHEN** the limiter is enabled and the speed reading is valid and exceeds `limit_max_kmh`
- **THEN** the throttle command SHALL be clamped to `SPEED_LIMIT_THROTTLE_CAP_PCT` (a reduced ceiling,
  not a hard cut)
- **AND** when speed is at or below the limit the throttle command SHALL pass through unchanged

#### Scenario: Limiter fails open on sensor loss
- **WHEN** the limiter is enabled and the speed reading is invalid or stale
- **THEN** the limiter SHALL NOT clamp the throttle (fail open)
- **AND** a rate-limited warning SHALL be logged

#### Scenario: Limiter does not fight the gear-boost throttle PID
- **WHEN** a gear-change throttle-boost is active (the boost PID owns the throttle)
- **THEN** the speed limiter SHALL apply only to the arbitrated driver/MAVLink/web throttle command
- **AND** SHALL NOT override or conflict with the gear-boost PID output
