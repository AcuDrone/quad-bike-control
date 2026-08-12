## MODIFIED Requirements

### Requirement: Relay Controller for Ignition and Lights
The system SHALL provide a `RelayController` class to manage relay outputs for ignition
states, front light and front-wheel lock control, driven by commands decoded from the MAVLink
interface or the web portal. The relays SHALL be driven through the external relay board's PCA9557
expander on `Wire1` at address `0x18` rather than through direct ESP32 GPIO, with the public API
unchanged for callers.

The relay board's PCA9557 IO routing is NOT 1:1 with the relay numbers (`Relay_1=IO4,
Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`,
per the `Relay.PrjPcb` netlist), and two functions drive a PAIR of relays that always energize
together. Each function is therefore addressed as a whole-port MASK, not a single bit:

| Function | Mask constant | Value | Relays (IO / relay-board connector) |
|----------|---------------|-------|--------------------------------------|
| Ignition / ECU line | `RELAY_MASK_IGNITION` | `0b00110000` | Relay_1 (IO4, X4) + Relay_2 (IO5, X5) |
| Starter | `RELAY_MASK_STARTER` | `0b01000000` | Relay_3 (IO6, X6) |
| Front light | `RELAY_MASK_FRONT_LIGHT` | `0b10000000` | Relay_4 (IO7, X7) |
| Front-wheel lock | `RELAY_MASK_WHEEL_LOCK` | `0b00001100` | Relay_5 (IO3, X8) + Relay_6 (IO2, X9) |
| Spare | — | `0b00000011` | Relay_7 (IO0, X10) + Relay_8 (IO1, X11) — always off |

The duplicated relays are intentional: both relays of a pair are set and cleared as one mask, so a
pair can never be left half-energized by firmware.

#### Scenario: Initialize relay controller
- **WHEN** `RelayController.begin()` is called with the `Wire1` I2C bus
- **THEN** the PCA9557 at `0x18` SHALL be detected and all eight pins configured as outputs
- **AND** the output port SHALL be written to `0x00` (all relays OFF) and verified by read-back
- **AND** internal state tracking is initialized to OFF / light off / lock off
- **AND** true is returned if initialization succeeds, false if the expander does not answer or the
  all-off write does not verify

#### Scenario: Set ignition state
- **WHEN** `setIgnitionState(OFF)` is called, `RELAY_MASK_IGNITION` (Relay_1 + Relay_2) and
  `RELAY_MASK_STARTER` (Relay_3) are LOW
- **WHEN** `setIgnitionState(ACC)` is called, `RELAY_MASK_IGNITION` is HIGH and `RELAY_MASK_STARTER`
  is LOW
- **WHEN** `setIgnitionState(IGNITION)` is called, `RELAY_MASK_IGNITION` is HIGH and
  `RELAY_MASK_STARTER` is LOW
- **WHEN** `setIgnitionState(CRANKING)` is called, `RELAY_MASK_IGNITION` and `RELAY_MASK_STARTER` are
  both HIGH
- **AND** every bit of a mask moves together, so Relay_1 and Relay_2 are always in the same state
- **AND** internal state is updated to match in each case
- **AND** each change is applied as a single whole-port write from the output shadow byte

#### Scenario: Control front light
- **WHEN** `setFrontLight(true)` is called, `RELAY_MASK_FRONT_LIGHT` (Relay_4, IO7) is set HIGH and
  state is updated
- **WHEN** `setFrontLight(false)` is called, `RELAY_MASK_FRONT_LIGHT` is set LOW and state is updated

#### Scenario: Control front-wheel lock
- **WHEN** `setWheelLock(true)` is called, `RELAY_MASK_WHEEL_LOCK` (Relay_5 + Relay_6) is set HIGH
  and state is updated
- **WHEN** `setWheelLock(false)` is called, `RELAY_MASK_WHEEL_LOCK` is set LOW and state is updated
- **AND** both relays of the pair change together in the same whole-port write

#### Scenario: Spare relays are never energized
- **WHEN** any relay state changes
- **THEN** `Relay_7` (IO0) and `Relay_8` (IO1) SHALL be written LOW in every port write

#### Scenario: Fail-safe all relays off
- **WHEN** `allOff()` is called (fail-safe trigger)
- **THEN** `RELAY_MASK_IGNITION`, `RELAY_MASK_STARTER` and `RELAY_MASK_FRONT_LIGHT` are set LOW
- **AND** internal state is reset to safe defaults (OFF, lights off)
- **AND** the front-wheel lock pair (`RELAY_MASK_WHEEL_LOCK`) is intentionally left in its last state
- **AND** the fail-safe write is verified by read-back like any other relay write

#### Scenario: Relay commands are no-ops when the expander is unavailable
- **WHEN** the expander failed to initialize or the driver is faulted
- **AND** a MAVLink- or web-decoded relay command arrives
- **THEN** the command SHALL be logged and ignored rather than silently assumed applied
- **AND** the reported ignition state SHALL be `OFF`
