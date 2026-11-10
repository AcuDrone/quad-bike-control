## MODIFIED Requirements
### Requirement: Servo Channel to Vehicle Command Mapping
The system SHALL map decoded servo-output channels to typed vehicle commands using a
compile-time `ServoChannelConfig` and the transport-neutral range constants in `Constants.h`,
preserving the function assignments previously used for SBUS.

#### Scenario: Define servo channel assignments
- **WHEN** `ServoChannelConfig` is defined
- **THEN** the following function→servo-output indices are available:
  - STEERING = 1
  - THROTTLE = 2 (combined throttle/brake)
  - TRANSMISSION = 3
  - IGNITION = 4
  - FRONT_LIGHT = 6 (channel 5 is intentionally unused)
  - WHEEL_LOCK = 7 (the lock; aztec: Relay_4 + Relay_5 + Relay_6)

#### Scenario: Map steering channel to percentage
- **WHEN** `getSteering()` is called
- **THEN** the steering servo-output value is read and converted using bidirectional mapping
  (center = 0%, min = -100%, max = +100%)
- **AND** the steering deadband is applied around center

#### Scenario: Map throttle and brake from the combined channel
- **WHEN** `getThrottle()` is called
- **THEN** only the upper half (above center µs) of the throttle channel maps to 0–100%
- **AND** values at or below center return 0%
- **WHEN** `getBrake()` is called
- **THEN** only the lower half (below center µs) of the same channel maps to 0–100%
- **AND** values at or above center return 0%

#### Scenario: Map transmission, ignition, and light channels
- **WHEN** `getGear()`, `getIgnitionState()`, or `getFrontLight()` is called
- **THEN** the corresponding channel µs value is mapped to the `Gear` enum, the
  `IgnitionState` enum (OFF/ACC/IGNITION), or a boolean respectively, using the configured
  microsecond ranges/threshold

#### Scenario: Map the lock channel
- **WHEN** `getWheelLock()` is called
- **THEN** it SHALL return `true` (LOCKED) when servo channel 7 is above `RC_WHEEL_LOCK_THRESHOLD`
  (1520 µs) and `false` (UNLOCKED) otherwise

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

The lock is fitted on the aztec buggy (`RELAY_WHEEL_LOCK_FITTED = 1`). The MAVLink servo channel
`ServoChannelConfig::WHEEL_LOCK` (7) SHALL drive it, and bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) of the
`EFI_STATUS` digital-output bitmask SHALL report it; the bitmask layout is unchanged.

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
- **WHEN** servo channel 7 (`WHEEL_LOCK`) is above `RC_WHEEL_LOCK_THRESHOLD` (e.g. 2000 µs) while
  MAVLink is the input source
- **THEN** `setWheelLock(true)` SHALL set the whole lock mask (Relay_4, Relay_5 and Relay_6) HIGH in
  one verified whole-port write and return `true`
- **AND** `getWheelLock()` SHALL return `true`
- **AND** bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) of the `EFI_STATUS` digital-output bitmask SHALL be 1
- **AND WHEN** channel 7 returns to or below 1520 µs (e.g. 1000 µs)
- **THEN** the lock mask SHALL be set LOW, `getWheelLock()` SHALL return `false` and bit0 SHALL be 0
- **AND** an unchanged channel value SHALL NOT rewrite the port or log on every control loop

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
