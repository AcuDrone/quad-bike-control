## MODIFIED Requirements
### Requirement: Dual I2C Bus Topology and Address Map
The system SHALL operate two independent I2C buses with a fixed device-to-bus assignment, and each
expander driver instance SHALL be bound to exactly one bus and one 7-bit address at construction.

Bus assignment:

| Bus | Arduino instance | SDA | SCL | Speed | Devices |
|-----|------------------|-----|-----|-------|---------|
| I2C1 | `Wire` | GPIO1 | GPIO2 | 100 kHz | External header X14 only: **no device fitted** (the steering angle sensor is on CAN); `Wire` is not started |
| I2C2 | `Wire1` | GPIO48 | GPIO47 | 100 kHz | Both on-board expanders — opto-input PCA9557 U10 `0x1A`, mux PCA9557 U2 `0x1C` — plus ONE relay board on header X15: the PCA9557 relay board `0x1F` or the XL9535-K8V5 relay board `0x20`..`0x27` (default `0x20`) |

This assignment is **empirical** — established by live bus scans on the assembled board, which
override the schematic's port names: both on-board PCA9557s (U10 and U2) answer on I2C2, and nothing
on-board answers on I2C1.

The PCA9557 relay board keeps its as-shipped `A2A1A0 = 111` → `0x1F` strap, which collides with
neither `0x1A` nor `0x1C`. The XL9535 board's `0x20`..`0x27` range collides with nothing on I2C2.
Both run on header X15 / `Wire1`. (On the first assembled board X15 passed no signals until rework in
the `FB3`/`FB4` ferrite area; because the straps are bus-agnostic, moving the board to X14 / `Wire`
is a change of the bus passed to `RelayController`, plus starting `Wire` on
`PIN_I2C1_SDA`/`PIN_I2C1_SCL` — a contingency.)

A boot scan of I2C2 SHALL show `0x1A`, `0x1C` and either `0x1F` or one address in `0x20`..`0x27`.
`Wire1` SHALL run at 100 kHz (`I2C_BUS_FREQ_HZ`) with an explicit transaction timeout of
`I2C_TIMEOUT_MS` (20 ms), set after every `begin()`.

#### Scenario: Initialize both buses before any I2C consumer
- **WHEN** `setup()` runs
- **THEN** `Wire1` SHALL be started on `PIN_RELAY_SDA` (GPIO48) / `PIN_RELAY_SCL` (GPIO47) at
  `I2C_BUS_FREQ_HZ`
- **AND** `Wire1.setTimeOut(I2C_TIMEOUT_MS)` SHALL be applied right after it
- **AND** it SHALL be up before the opto-input reader or the relay controller are initialized
- **AND** `Wire` SHALL NOT be started while no device is fitted on X14

#### Scenario: Drivers never re-open a bus they do not own
- **WHEN** an expander driver on `Wire1` initializes
- **THEN** it SHALL use the bus `main.cpp` already opened without calling `begin()` itself
- **AND** a failure of one device SHALL NOT prevent any other device from being read
- **AND** no driver SHALL call `end()` or `begin()` on `Wire1`; after `setup()` only the main-owned
  `I2cBusRecovery` routine (see "I2C2 Bus Unlock and Recovery") SHALL restart `Wire1`

#### Scenario: Driver bound to a single address
- **WHEN** a PCA9557 or XL9535 driver instance is constructed
- **THEN** it SHALL take a `TwoWire` reference and a 7-bit address as construction parameters
- **AND** it SHALL only ever address that device
- **AND** no "take the first expander found" behavior SHALL exist; relay-board detection SHALL follow
  the "Relay Board Selection and Detection" requirement, which faults on any ambiguity

## ADDED Requirements
### Requirement: I2C2 Bus Unlock and Recovery
The system SHALL provide one main-owned recovery routine for I2C2 (`I2cBusRecovery`) that frees a bus
held low by a slave and restarts `Wire1`. It SHALL be the only code that restarts `Wire1` after
`setup()`, SHALL be rate-limited to one run per `I2C_RECOVER_MIN_MS` (1 s) across all callers, and
SHALL journal every run.

#### Scenario: SDA held low
- **WHEN** a recovery is requested and SDA reads low
- **THEN** `Wire1` SHALL be stopped and SCL SHALL be pulsed as an open-drain output up to
  `I2C_UNLOCK_CLOCKS` (9) times at about 100 kHz, stopping as soon as SDA reads high
- **AND** a STOP condition SHALL be generated
- **AND** `Wire1` SHALL be restarted on GPIO48/47 at `I2C_BUS_FREQ_HZ` with `I2C_TIMEOUT_MS`
- **AND** `I2C W bus_stuck sda=0 scl=<level> clocks=<n> released=<0|1> reinit=1 by=<caller>` SHALL be
  journaled (identical unsuccessful runs at most once per minute) and the session counter
  `i2c_unlock` incremented

#### Scenario: SCL held low
- **WHEN** a recovery is requested and SCL reads low
- **THEN** no clocking SHALL be attempted, `Wire1` SHALL still be restarted and the line SHALL report
  `scl=0 released=0`

#### Scenario: Device absent on a healthy bus
- **WHEN** the requesting driver's last error is an address or data NACK and both lines read high
- **THEN** `Wire1` SHALL NOT be restarted; only the driver's own device re-initialization SHALL run

#### Scenario: Relay recovery path
- **WHEN** the relay controller's timed recovery (`RELAY_RECOVER_MS`) is due
- **THEN** it SHALL request the bus recovery before re-initializing the relay port
- **AND** relay write faults and recoveries SHALL be journaled as `I2C E relay_fault addr err step` and
  `I2C I relay_ok` with the Wire error code

#### Scenario: Opto-input failures
- **WHEN** `BOARD_INPUT_UNLOCK_FAILS` (8) consecutive opto-input polls have failed
- **THEN** the reader SHALL request the bus recovery and then re-configure its expander
- **AND** the stale edge and the recovery SHALL be journaled as `I2C W input_fail` and `I2C I input_ok`

#### Scenario: Stuck bus cannot stretch the loop
- **WHEN** the bus does not respond at all
- **THEN** no single I2C transaction SHALL block longer than `I2C_TIMEOUT_MS` (20 ms)
