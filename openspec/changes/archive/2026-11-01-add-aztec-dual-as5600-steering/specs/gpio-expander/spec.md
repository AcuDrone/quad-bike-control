## MODIFIED Requirements
### Requirement: Dual I2C Bus Topology and Address Map
The system SHALL operate two independent I2C buses with a fixed device-to-bus assignment, and each
expander driver instance SHALL be bound to exactly one bus and one 7-bit address at construction.

Bus assignment:

| Bus | Arduino instance | SDA | SCL | Speed | Devices |
|-----|------------------|-----|-----|-------|---------|
| I2C1 | `Wire` | GPIO1 | GPIO2 | 100 kHz | External header X14 only: fine AS5600 steering sensor `0x36` (on the steering shaft) |
| I2C2 | `Wire1` | GPIO48 | GPIO47 | 100 kHz | Both on-board expanders — opto-input PCA9557 U10 `0x1A`, mux PCA9557 U2 `0x1C` — plus, on header X15, the relay-board PCA9557 `0x1F` and the coarse AS5600 steering sensor `0x36` (on the geared-down shaft) |

This assignment is **empirical** — established by live bus scans on the assembled board, which
override the schematic's port names: both on-board PCA9557s (U10 and U2) answer on I2C2, and nothing
on-board answers on I2C1.

The relay board keeps its as-shipped `A2A1A0 = 111` → `0x1F` strap, which collides with neither
`0x1A` nor `0x1C`, and runs on header X15 / `Wire1`. (On the first assembled board X15 passed no
signals until rework in the `FB3`/`FB4` ferrite area; because the strap is bus-agnostic, moving the
board to X14 / `Wire` is a change of the bus passed to `RelayController` only — a contingency.)

The two AS5600 sensors share the fixed address `0x36` and are therefore on different buses; `0x36`
collides with no I2C2 expander. A boot scan of I2C2 SHALL show `0x1A`, `0x1C`, `0x1F` and `0x36`.

Both buses SHALL run at 100 kHz (`I2C_BUS_FREQ_HZ`): both AS5600s are reached over cable runs to
the steering column (X14 and X15), where signal-integrity margin is worth more than bus speed.

#### Scenario: Initialize both buses before any I2C consumer
- **WHEN** `setup()` runs
- **THEN** `Wire` SHALL be started on `PIN_STEER_SDA` (GPIO1) / `PIN_STEER_SCL` (GPIO2) at
  `I2C_BUS_FREQ_HZ`
- **AND** `Wire1` SHALL be started on `PIN_RELAY_SDA` (GPIO48) / `PIN_RELAY_SCL` (GPIO47) at
  `I2C_BUS_FREQ_HZ`
- **AND** both buses SHALL be up before either AS5600 sensor, the opto-input reader, or the relay
  controller are initialized
- **AND** initialization of one bus SHALL NOT depend on a device on the other bus responding

#### Scenario: Drivers never re-open a bus they do not own
- **WHEN** an AS5600 (on `Wire` or `Wire1`) or an expander driver on `Wire1` initializes
- **THEN** it SHALL use the bus `main.cpp` already opened without calling `begin()` itself
- **AND** a failure of one device SHALL NOT prevent any other device from being read
- **AND** only the fine AS5600 on `Wire` SHALL perform its own wedged-bus recovery, restoring the exact
  bus parameters `main.cpp` opened
- **AND** the coarse AS5600 on `Wire1` SHALL NEVER call `end()` or `begin()` on `Wire1`; recovery of
  the shared bus SHALL be coordinated by `main.cpp`

#### Scenario: Driver bound to a single address
- **WHEN** a PCA9557 driver instance is constructed
- **THEN** it SHALL take a `TwoWire` reference and a 7-bit address as construction parameters
- **AND** it SHALL only ever address that device
- **AND** no bus-scanning "take the first expander found" behavior SHALL exist

## ADDED Requirements
### Requirement: Coordinated I2C2 Bus Recovery
The shared I2C2 bus (`Wire1`) SHALL be re-initialized only by a single coordinator in `main.cpp`,
only when evidence points to the bus rather than one device, and never while a `Wire1` transaction
is in flight.

#### Scenario: Recover a wedged shared bus
- **WHEN** the coarse AS5600 has at least `AS5600_RECOVER_AFTER_FAILS` consecutive failed reads
- **AND** `BoardInputs::isFaulted()` is true at the same time
- **AND** at least `I2C2_BUS_RECOVER_MS` has elapsed since the last recovery attempt
- **THEN** `main.cpp` SHALL call `Wire1.end()` and `Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL,
  I2C_BUS_FREQ_HZ)` from the loop task, after every `Wire1` user has finished for that iteration
- **AND** the attempt SHALL be logged once per attempt
- **AND** PCA9557 output and configuration registers SHALL NOT be written by the recovery itself
  (relay states are held by the device; `BoardInputs` re-configures on its own recovery cadence)

#### Scenario: Single-device failure does not reset the bus
- **WHEN** the coarse AS5600 fails but the opto-input expander still reads successfully
- **THEN** `Wire1` SHALL NOT be re-initialized
- **AND** the failure SHALL be handled as a steering sensor fault only

#### Scenario: All Wire1 traffic stays on the loop task
- **WHEN** any `Wire1` device (coarse AS5600, opto-input expander, relay expander) is accessed
- **THEN** the access SHALL happen on the Arduino loop task, so the coordinator never interrupts a
  transaction
