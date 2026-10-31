## MODIFIED Requirements
### Requirement: Relay Controller for Ignition and Lights
The system SHALL provide a `RelayController` class to manage relay outputs for ignition
states and front light control, driven by commands decoded from the MAVLink
interface or the web portal. The relays SHALL be driven through the external relay board's PCA9557
expander at address `0x1F` rather than through direct ESP32 GPIO, with the public API unchanged for
callers. The bus is supplied by the owner at construction: `Wire1` (I2C2), the relay board's home on
header X15.

The relay board's PCA9557 IO routing is NOT 1:1 with the relay numbers (`Relay_1=IO4,
Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`,
per the `Relay.PrjPcb` netlist). Each function is therefore addressed as a whole-port MASK, not a
relay number. Aztec buggy assignment (no relay is paired):

| Function | Mask constant | Value | Relays (IO / relay-board connector) |
|----------|---------------|-------|--------------------------------------|
| Ignition / ECU line | `RELAY_MASK_IGNITION` | `0b00010000` | Relay_1 (IO4, X4) |
| Starter | `RELAY_MASK_STARTER` | `0b00100000` | Relay_2 (IO5, X5) |
| Front light | `RELAY_MASK_FRONT_LIGHT` | `0b01000000` | Relay_3 (IO6, X6) |
| Reserved | `RELAY_MASK_RESERVED` | `0b10001111` | Relay_4 (IO7, X7), Relay_5 (IO3, X8), Relay_6 (IO2, X9) — three locks specified in a later change; Relay_7 (IO0, X10), Relay_8 (IO1, X11) — spare; off except in the web relay test |

The front-wheel lock is not fitted on the aztec buggy. The MAVLink servo channel
`ServoChannelConfig::WHEEL_LOCK` (7) SHALL still be decoded and the `EFI_STATUS` digital-output
bitmask layout SHALL be unchanged, but no relay SHALL be driven by that channel.

#### Scenario: Initialize relay controller
- **WHEN** `RelayController.begin()` is called with the I2C bus the relay board is wired to
- **THEN** the PCA9557 at `0x1F` SHALL be detected and all eight pins configured as outputs
- **AND** the output port SHALL be written to `0x00` (all relays OFF) and verified by read-back
- **AND** internal state tracking is initialized to OFF / light off
- **AND** true is returned if initialization succeeds, false if the expander does not answer or the
  all-off write does not verify

#### Scenario: Set ignition state
- **WHEN** `setIgnitionState(OFF)` is called, `RELAY_MASK_IGNITION` (Relay_1) and
  `RELAY_MASK_STARTER` (Relay_2) are LOW
- **WHEN** `setIgnitionState(ACC)` is called, `RELAY_MASK_IGNITION` is HIGH and `RELAY_MASK_STARTER`
  is LOW
- **WHEN** `setIgnitionState(IGNITION)` is called, `RELAY_MASK_IGNITION` is HIGH and
  `RELAY_MASK_STARTER` is LOW
- **WHEN** `setIgnitionState(CRANKING)` is called, `RELAY_MASK_IGNITION` and `RELAY_MASK_STARTER` are
  both HIGH
- **AND** internal state is updated to match in each case
- **AND** each change is applied as a single whole-port write from the output shadow byte

#### Scenario: Control front light
- **WHEN** `setFrontLight(true)` is called, `RELAY_MASK_FRONT_LIGHT` (Relay_3, IO6) is set HIGH and
  state is updated
- **WHEN** `setFrontLight(false)` is called, `RELAY_MASK_FRONT_LIGHT` is set LOW and state is updated

#### Scenario: Control front-wheel lock
- **WHEN** servo channel 7 (`WHEEL_LOCK`) is above `RC_WHEEL_LOCK_THRESHOLD` and `setWheelLock(true)`
  is called
- **THEN** `setWheelLock()` SHALL return `false` and no port write SHALL be made
- **AND** the rejection SHALL be logged once per boot, not on every control loop
- **AND** `getWheelLock()` SHALL return `false`
- **AND** bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) of the `EFI_STATUS` digital-output bitmask SHALL be 0

#### Scenario: Spare relays are never energized
- **WHEN** any relay state changes through a MAVLink- or web-decoded vehicle function
- **THEN** every `RELAY_MASK_RESERVED` bit — Relay_4 (IO7), Relay_5 (IO3), Relay_6 (IO2), Relay_7
  (IO0) and Relay_8 (IO1) — SHALL be written LOW unless the web relay test (ignition OFF only)
  energizes that relay
- **AND** a MAVLink ignition command that leaves `OFF`, or a front-light ON, SHALL clear every web
  test relay first

#### Scenario: Fail-safe all relays off
- **WHEN** `allOff()` is called (fail-safe trigger)
- **THEN** `RELAY_MASK_IGNITION`, `RELAY_MASK_STARTER` and `RELAY_MASK_FRONT_LIGHT` are set LOW
- **AND** internal state is reset to safe defaults (OFF, lights off)
- **AND** every web test relay is released, so all eight relays are LOW
- **AND** the fail-safe write is verified by read-back like any other relay write

#### Scenario: Relay commands are no-ops when the expander is unavailable
- **WHEN** the expander failed to initialize or the driver is faulted
- **AND** a MAVLink- or web-decoded relay command arrives
- **THEN** the command SHALL be logged and ignored rather than silently assumed applied
- **AND** the reported ignition state SHALL be `OFF`
