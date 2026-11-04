# gpio-expander Specification

## Purpose
Relay outputs and digital sensor inputs of the hand-wired ESP32-S3-DevKitC-1 ("old board",
`old-board` branch). This board has **no I/O expander and no I2C bus**: every relay is driven
straight from an ESP32 GPIO and every gear switch / the brake limit sensor is read straight with
`digitalRead()`. (The capability keeps its historical name; the PCA9557 expander requirements of the
custom `Control_v0` board live on `master`.)
## Requirements
### Requirement: Direct GPIO Relay Outputs
The system SHALL drive the four relays directly from ESP32 GPIO, active HIGH, through
`RelayController`, with one function per relay:

| Relay | GPIO | Define | Function | Mask constant |
|-------|------|--------|----------|---------------|
| 1 | 36 | `PIN_RELAY1` | Ignition / ECU line | `RELAY_MASK_IGNITION` (`0b0001`) |
| 2 | 37 | `PIN_RELAY2` | Starter solenoid | `RELAY_MASK_STARTER` (`0b0010`) |
| 3 | 38 | `PIN_RELAY3` | Front light | `RELAY_MASK_FRONT_LIGHT` (`0b0100`) |
| 4 | 39 | `PIN_WHEEL_LOCK` | Front-wheel lock | `RELAY_MASK_WHEEL_LOCK` (`0b1000`) |

`RELAY_COUNT` SHALL be 4, `RELAY_MASK_RESERVED` SHALL be `0` and `RELAY_WHEEL_LOCK_FITTED` SHALL be
`1`. The four outputs SHALL be written from one shadow byte (bit0 = Relay 1 … bit3 = Relay 4); the
function masks SHALL be disjoint and together cover the four relays, checked at compile time.

#### Scenario: Initialize relays to the safe state
- **WHEN** `RelayController::begin()` is called
- **THEN** each relay pin SHALL be driven LOW before it is configured as an output
- **AND** all four relays SHALL be off when `begin()` returns `true`

#### Scenario: Relay functions
- **WHEN** the ignition state is `ACC` or `IGNITION`
- **THEN** only Relay 1 SHALL be energized
- **WHEN** the ignition state is `CRANKING`
- **THEN** Relay 1 and Relay 2 SHALL be energized
- **WHEN** the front light is on
- **THEN** Relay 3 SHALL be energized
- **WHEN** the front-wheel lock is engaged
- **THEN** Relay 4 SHALL be energized

#### Scenario: No write verification on direct GPIO
- **WHEN** a relay output is written
- **THEN** no read-back verification, retry or starter escalation SHALL be performed, because a GPIO
  write cannot fail silently
- **AND** no relay-fault field SHALL be published in telemetry

#### Scenario: Public API matches master
- **WHEN** callers use `setIgnitionState()`, `requestCrank()`, `setFrontLight()`, `setWheelLock()`,
  `setRelayTest()`, `getRelayTestBits()`, `update()`, `allOff()` or the state getters
- **THEN** the `ACC_PRECRANK_DWELL_MS` pre-crank dwell, `CRANKING_TIMEOUT`, RPM-based crank exit and
  the wheel lock holding its state through `allOff()` SHALL behave as on master

### Requirement: Direct GPIO Sensor Inputs
The system SHALL read the gear selector switches and the brake limit sensor directly from ESP32 GPIO
with `digitalRead()`:

| GPIO | Define | Function | Electrical |
|------|--------|----------|------------|
| 19 | `PIN_GEAR_REVERSE` | Gear switch R | floating input, external pull-up, active LOW |
| 20 | `PIN_GEAR_NEUTRAL` | Gear switch N | floating input, external pull-up, active LOW |
| 21 | `PIN_GEAR_LOW` | Gear switch L | floating input, external pull-up, active LOW |
| 47 | `PIN_GEAR_HIGH` | Gear switch H | floating input, external pull-up, active LOW |
| 14 | `PIN_BRAKE_SENSOR` | Brake "fully released" limit | plain `INPUT`, HIGH = released |

#### Scenario: Physical gear from the switches
- **WHEN** `TransmissionController::getPhysicalGear()` reads the switches
- **THEN** exactly one active switch SHALL yield that gear
- **AND** zero or more than one active switch SHALL be re-read up to `TRANS_GEAR_READ_RETRY_COUNT`
  times while the servo is idle, and SHALL otherwise report `GEAR_UNKNOWN`

#### Scenario: Brake released
- **WHEN** `VehicleController::isBrakeReleased()` is called
- **THEN** it SHALL return the level of `PIN_BRAKE_SENSOR` (HIGH = released)

#### Scenario: Native USB CDC stays disabled
- **WHEN** the firmware is built
- **THEN** `ARDUINO_USB_CDC_ON_BOOT` SHALL NOT be set, because GPIO19/20 (native USB D-/D+) carry
  the R/N gear switches

### Requirement: No I2C Bus
The firmware SHALL NOT start any I2C bus on this board. GPIO41/42 (formerly the AS5600 steering
sensor's SDA/SCL) SHALL be documented as free/reserved in `Constants.h` and `GPIO_PINOUT_S3.md`;
steering angle feedback is the Bosch LWS on the CAN bus.

#### Scenario: Boot without I2C
- **WHEN** `setup()` runs
- **THEN** neither `Wire` nor `Wire1` SHALL be started and no I2C scan SHALL be performed

### Requirement: Web Relay Test
The system SHALL let the operator energize any single relay (Relay 1..Relay 4) for a wiring check,
through `RelayController::setRelayTest(relayNumber, on, reason)`. Relay numbers SHALL map to shadow
bits through `relayIoMask()` (Relay N = bit N-1), checked at compile time against the function masks.
Test bits SHALL be part of the same shadow byte as the vehicle functions. Every change of a test relay
SHALL be logged under the `RELAY` debug feature.

#### Scenario: Toggle a relay on
- **WHEN** a relay test ON request names a relay 1..4
- **AND** the ignition state is `OFF`, no crank is armed, the front light is off and the wheel lock
  is released
- **THEN** that relay's bit SHALL be set in the shadow byte and written to its GPIO
- **AND** the request SHALL succeed and the relay SHALL be reported in the `relay_test` bitfield

#### Scenario: Toggle a relay off
- **WHEN** a relay test OFF request names a relay that is test-energized
- **THEN** its bit SHALL be cleared and the outputs rewritten
- **AND** an OFF request for a relay that is not test-energized SHALL succeed without a write

#### Scenario: Auto-off after the test timeout
- **WHEN** a test relay other than Relay 2 has been on for `RELAY_TEST_TIMEOUT_MS` (30000 ms)
- **THEN** `update()` SHALL clear it, rewrite the outputs and log the auto-off

#### Scenario: Starter relay test is capped
- **WHEN** Relay 2 (the starter relay) has been test-energized for `RELAY_TEST_STARTER_MAX_MS`
  (2000 ms)
- **THEN** `update()` SHALL clear it and rewrite the outputs

#### Scenario: Test refused while a function is active
- **WHEN** a relay test ON request arrives while the ignition state is not `OFF`, a crank is armed,
  the front light is on or the wheel lock is engaged
- **THEN** the request SHALL be refused with an error text and no output SHALL change

#### Scenario: Relay number outside 1..4 is rejected
- **WHEN** a relay test request names relay 0, 5 or any other number outside 1..4
- **THEN** it SHALL be refused with the error text "Relay number must be 1-4" and no relay SHALL change

#### Scenario: A real function command takes over
- **WHEN** an ignition command to `ACC`, `IGNITION` or `CRANKING`, a crank request, a front-light ON
  or a wheel-lock LOCK command arrives while any test relay is on
- **THEN** every test bit SHALL be cleared before the function relays are written, and the
  forced-off SHALL be logged

#### Scenario: Test relays forced off on fail-safe
- **WHEN** `allOff()` runs (fail-safe)
- **THEN** every test bit SHALL be cleared immediately and the outputs rewritten
