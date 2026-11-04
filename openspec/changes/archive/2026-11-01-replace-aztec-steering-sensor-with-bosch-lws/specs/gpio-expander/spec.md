## MODIFIED Requirements
### Requirement: Dual I2C Bus Topology and Address Map
The system SHALL operate two independent I2C buses with a fixed device-to-bus assignment, and each
expander driver instance SHALL be bound to exactly one bus and one 7-bit address at construction.

Bus assignment:

| Bus | Arduino instance | SDA | SCL | Speed | Devices |
|-----|------------------|-----|-----|-------|---------|
| I2C1 | `Wire` | GPIO1 | GPIO2 | 100 kHz | External header X14 only: **no device fitted** (the steering angle sensor is on CAN); `Wire` is not started |
| I2C2 | `Wire1` | GPIO48 | GPIO47 | 100 kHz | Both on-board expanders — opto-input PCA9557 U10 `0x1A`, mux PCA9557 U2 `0x1C` — plus the relay-board PCA9557 `0x1F` on header X15 |

This assignment is **empirical** — established by live bus scans on the assembled board, which
override the schematic's port names: both on-board PCA9557s (U10 and U2) answer on I2C2, and nothing
on-board answers on I2C1.

The relay board keeps its as-shipped `A2A1A0 = 111` → `0x1F` strap, which collides with neither
`0x1A` nor `0x1C`, and runs on header X15 / `Wire1`. (On the first assembled board X15 passed no
signals until rework in the `FB3`/`FB4` ferrite area; because the strap is bus-agnostic, moving the
board to X14 / `Wire` is a change of the bus passed to `RelayController`, plus starting `Wire` on
`PIN_I2C1_SDA`/`PIN_I2C1_SCL` — a contingency.)

A boot scan of I2C2 SHALL show `0x1A`, `0x1C` and `0x1F`. `Wire1` SHALL run at 100 kHz
(`I2C_BUS_FREQ_HZ`).

#### Scenario: Initialize both buses before any I2C consumer
- **WHEN** `setup()` runs
- **THEN** `Wire1` SHALL be started on `PIN_RELAY_SDA` (GPIO48) / `PIN_RELAY_SCL` (GPIO47) at
  `I2C_BUS_FREQ_HZ`
- **AND** it SHALL be up before the opto-input reader or the relay controller are initialized
- **AND** `Wire` SHALL NOT be started while no device is fitted on X14

#### Scenario: Drivers never re-open a bus they do not own
- **WHEN** an expander driver on `Wire1` initializes
- **THEN** it SHALL use the bus `main.cpp` already opened without calling `begin()` itself
- **AND** a failure of one device SHALL NOT prevent any other device from being read
- **AND** no driver and no coordinator SHALL call `end()` or `begin()` on `Wire1` after `setup()`

#### Scenario: Driver bound to a single address
- **WHEN** a PCA9557 driver instance is constructed
- **THEN** it SHALL take a `TwoWire` reference and a 7-bit address as construction parameters
- **AND** it SHALL only ever address that device
- **AND** no bus-scanning "take the first expander found" behavior SHALL exist

## REMOVED Requirements
### Requirement: Coordinated I2C2 Bus Recovery
**Reason**: Its trigger was the coarse AS5600 failing together with the opto-input expander. The coarse AS5600 is removed, so the coordinator in `main.cpp`, `AS5600_RECOVER_AFTER_FAILS` and `I2C2_BUS_RECOVER_MS` are removed with it.
**Migration**: None. The expanders keep their own fail-safes: `BoardInputs` re-configures itself on its recovery cadence, and `RelayController` verifies every write.
