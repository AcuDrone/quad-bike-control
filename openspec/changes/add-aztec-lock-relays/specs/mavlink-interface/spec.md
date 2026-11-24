## MODIFIED Requirements
### Requirement: Relay Controller for Ignition and Lights
The system SHALL provide a `RelayController` class to manage relay outputs for ignition
states, front light control and the lock, driven by commands decoded from the MAVLink
interface or the web portal. The relays SHALL be driven through the external relay board's expander
— the PCA9557 relay board at `0x1F` or the XL9535-K8V5 relay board at `0x20`..`0x27`, selected once
at boot (see `gpio-expander` "Relay Board Selection and Detection") — rather than through direct
ESP32 GPIO, with the public API unchanged for callers. The bus is supplied by the owner at construction: `Wire1` (I2C2), the relay board's home on
header X15.

The PCA9557 relay board's IO routing is NOT 1:1 with the relay numbers (`Relay_1=IO4,
Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`,
per the `Relay.PrjPcb` netlist); the XL9535 board maps Relay_N to P0.(N-1). Each function is defined
in `Constants.h` as a relay-number set and addressed at runtime as a whole-port MASK built from the
active board's table. Aztec buggy assignment (masks shown for the PCA9557 / XL9535 board):

| Function | Set constant | PCA9557 mask | XL9535 mask | Relays |
|----------|--------------|--------------|-------------|--------|
| Ignition / ECU line | `RELAY_SET_IGNITION` | `0b00010000` | `0b00000001` | Relay_1 |
| Starter | `RELAY_SET_STARTER` | `0b00100000` | `0b00000010` | Relay_2 |
| Front light | `RELAY_SET_FRONT_LIGHT` | `0b01000000` | `0b00000100` | Relay_3 |
| Lock | `RELAY_SET_WHEEL_LOCK` | `0b10001100` | `0b00111000` | Relay_4 + Relay_5 + Relay_6, switched together |
| Reserved | `RELAY_SET_RESERVED` | `0b00000011` | `0b11000000` | Relay_7, Relay_8 — spare; off except in the web relay test |

The lock is fitted on the aztec buggy (`RELAY_WHEEL_LOCK_FITTED = 1`). The `EFI_STATUS`
digital-output bitmask layout SHALL be unchanged, and its bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) SHALL
report the lock. MAVLink servo channel 7 SHALL NOT drive the lock on any build (see
`vehicle-systems` "Lock Not Driven by an RC Channel"); the lock is requested only by the web
`set_wheel_lock` command and the GCS lock command.

#### Scenario: Initialize relay controller
- **WHEN** `RelayController.begin()` is called with the I2C bus the relay board is wired to
- **THEN** the relay board SHALL be detected once (PCA9557 at `0x1F` or XL9535 at `0x20`..`0x27`)
  and its eight relay pins configured as outputs by `initAllOff()` (output `0x00` before
  configuration)
- **AND** the pins SHALL read back `0x00` (all relays OFF)
- **AND** internal state tracking is initialized to OFF / light off / lock released
- **AND** true is returned if initialization succeeds, false if detection fails or is ambiguous, or the
  all-off state does not verify

#### Scenario: Set ignition state
- **WHEN** `setIgnitionState(OFF)` is called, the ignition mask (Relay_1) and the starter mask
  (Relay_2) are LOW
- **WHEN** `setIgnitionState(ACC)` is called, the ignition mask is HIGH and the starter mask
  is LOW
- **WHEN** `setIgnitionState(IGNITION)` is called, the ignition mask is HIGH and
  the starter mask is LOW
- **WHEN** `setIgnitionState(CRANKING)` is called, the ignition mask and the starter mask are
  both HIGH
- **AND** internal state is updated to match in each case
- **AND** each change is applied as a single whole-port write from the output shadow byte

#### Scenario: Control front light
- **WHEN** `setFrontLight(true)` is called, the front-light mask (Relay_3) is set HIGH and
  state is updated
- **WHEN** `setFrontLight(false)` is called, the front-light mask is set LOW and state is updated

#### Scenario: Control front-wheel lock
- **WHEN** a web or GCS lock request calls `setWheelLock(true)`
- **THEN** `setWheelLock(true)` SHALL set the whole lock mask (Relay_4, Relay_5 and Relay_6) HIGH in
  one verified whole-port write and return `true`
- **AND** `getWheelLock()` SHALL return `true`
- **AND** bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) of the `EFI_STATUS` digital-output bitmask SHALL be 1
- **AND WHEN** a web or GCS lock release calls `setWheelLock(false)`
- **THEN** the lock mask SHALL be set LOW, `getWheelLock()` SHALL return `false` and bit0 SHALL be 0
- **AND** a request for the state already in force SHALL return `true` without rewriting the port

#### Scenario: Spare relays are never energized
- **WHEN** any relay state changes through a MAVLink- or web-decoded vehicle function
- **THEN** every reserved bit — Relay_7 and Relay_8, at the active board's port bits — SHALL be
  written LOW unless the web relay test (ignition OFF only) energizes that relay
- **AND** a MAVLink ignition command that leaves `OFF`, a front-light ON, or a LOCK SHALL clear every
  web test relay first

#### Scenario: Fail-safe all relays off
- **WHEN** `allOff()` is called (fail-safe trigger)
- **THEN** the ignition mask, the starter mask and the front-light mask are set LOW
- **AND** internal state is reset to safe defaults (OFF, lights off)
- **AND** the lock mask SHALL hold its last commanded state
- **AND** every web test relay is released, so every relay outside the lock mask is LOW
- **AND** the fail-safe write is verified by pin read-back like any other relay write

#### Scenario: Relay commands are no-ops when the expander is unavailable
- **WHEN** the expander failed to initialize or the driver is faulted
- **AND** a MAVLink- or web-decoded relay command arrives
- **THEN** the command SHALL be logged and ignored rather than silently assumed applied
- **AND** the reported ignition state SHALL be `OFF`
