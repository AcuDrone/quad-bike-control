# gpio-expander Specification

## Purpose
TBD - created by archiving change add-mcp23017-gpio-expander. Update Purpose after archive.
## Requirements
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

### Requirement: PCA9557 Register Driver
The system SHALL provide a minimal PCA9557 driver exposing the device's four registers, with every
transaction reporting success or failure to its caller and no internal retry or blocking wait.

Registers: `0x00` input port, `0x01` output port, `0x02` polarity inversion, `0x03` configuration
(1 = input, 0 = output).

#### Scenario: Read the input port
- **WHEN** `readInputs(uint8_t& value)` is called
- **THEN** register `0x00` SHALL be read in a single I2C transaction
- **AND** `true` SHALL be returned with `value` set on success
- **AND** `false` SHALL be returned and `value` left unmodified if the transaction NACKs or times out

#### Scenario: Write the output port
- **WHEN** `writeOutputs(uint8_t value)` is called
- **THEN** register `0x01` SHALL be written in a single I2C transaction
- **AND** the transaction result SHALL be returned to the caller

#### Scenario: Read back the output register for verification
- **WHEN** `readOutputRegister(uint8_t& value)` is called
- **THEN** register `0x01` SHALL be read back so the caller can compare it against the value it wrote
- **AND** the transaction result SHALL be returned to the caller

#### Scenario: Configure pin directions
- **WHEN** `setDirection(uint8_t mask)` is called
- **THEN** register `0x03` SHALL be written with the mask (1 = input, 0 = output)
- **AND** the transaction result SHALL be returned to the caller

#### Scenario: Failure never blocks the control loop
- **WHEN** any driver method is called and the device does not respond
- **THEN** the method SHALL return `false` within one I2C transaction timeout
- **AND** the driver SHALL NOT retry internally, sleep, or spin — retry policy belongs to the caller

### Requirement: Opto-Isolated Input Expander
The system SHALL read the eight 5V opto-isolated (PC817) digital inputs through the on-board PCA9557
U10 at address `0x1A` on `Wire1` (I2C2 — scan-verified; the schematic's port names implied I2C1),
replacing direct GPIO reads for the gear selector and the brake limit sensor.

Input assignment (aztec buggy; the quad bike's one-hot four-line gear assignment is kept on
`master`):

| Port bit | Connector | Function |
|----------|-----------|----------|
| 0 | X16 | Gear code bit A — left / MSB (engaged = HIGH) |
| 1 | X16 | Gear code bit B — middle (engaged = HIGH) |
| 6 | X16 | Gear code bit C — right / LSB (engaged = HIGH) |
| 7 | X16 | Not wired on aztec — SHALL be ignored |
| 4 | X17 | Brake limit sensor ("released" endstop) |
| 2, 3, 5 | X17 | Spare — no firmware function |

The gear code SHALL be formed from the raw levels of bits 0, 1 and 6 in the order `[bit0 bit1 bit6]`,
i.e. `(bit0 << 2) | (bit1 << 1) | bit6`. `BOARD_INPUT_ACTIVE_LOW` SHALL NOT be applied to the gear
code bits.

#### Scenario: Initialize the input expander
- **WHEN** `BoardInputs::begin()` is called with the `Wire1` bus
- **THEN** all eight PCA9557 pins SHALL be configured as inputs
- **AND** a first input-port read SHALL be performed to seed the snapshot
- **AND** `true` SHALL be returned only if the device answers at `0x1A`
- **AND** `false` SHALL be returned with an error logged if it does not

#### Scenario: Consumers read a cached snapshot, never the bus
- **WHEN** `TransmissionController::getPhysicalGear()` or `VehicleController::isBrakeReleased()`
  needs an input state
- **THEN** it SHALL read the published snapshot from `BoardInputs`
- **AND** it SHALL NOT issue an I2C transaction of its own
- **AND** no `digitalRead()` of a gear or brake-sensor pin SHALL remain in the firmware

#### Scenario: Snapshot exposes gear and brake state with validity
- **WHEN** `getSnapshot()` is called
- **THEN** it SHALL return the undecoded 3-bit gear code (0..7), the brake-released state, a `valid`
  flag, and the age in milliseconds of the last successful read
- **AND** `BoardInputs` SHALL NOT decode the code into a gear

#### Scenario: Gear code extraction
- **WHEN** the raw port byte has bit 0 = 1, bit 1 = 0 and bit 6 = 1
- **THEN** the published gear code SHALL be `0b101`

#### Scenario: Bit 7 is ignored
- **WHEN** two raw port bytes differ only in bit 7
- **THEN** they SHALL produce the same gear code

### Requirement: Opto Input Polling and Fault Policy
The system SHALL poll the input expander non-blocking from the cooperative `loop()` and SHALL define
deterministic behavior when the read fails, so that a degraded I2C bus is never silently
indistinguishable from real input states.

#### Scenario: Poll at a fixed interval
- **WHEN** `BoardInputs::update()` is called from `loop()`
- **AND** at least `BOARD_INPUT_POLL_MS` (25 ms) has elapsed since the last poll
- **THEN** the input port SHALL be read once
- **AND** on success the snapshot SHALL be replaced, `valid` set to `true`, `ageMs` reset to 0, and
  the consecutive-failure counter cleared
- **AND** `update()` SHALL return without blocking regardless of the outcome

#### Scenario: Isolated read failure holds the last-known values
- **WHEN** a single poll fails
- **THEN** the previous snapshot SHALL be retained unchanged
- **AND** `valid` SHALL NOT be cleared by that single failure
- **AND** the failure counter SHALL be incremented and the next poll SHALL retry
- **AND** logging SHALL be rate-limited so a persistent fault cannot flood the console

#### Scenario: Stale inputs are marked invalid and raise a fault
- **WHEN** no successful read has occurred for longer than `BOARD_INPUT_STALE_MS` (250 ms)
- **THEN** the snapshot `valid` flag SHALL be set to `false`
- **AND** `isFaulted()` SHALL return `true`
- **AND** the fault SHALL be logged once per fault episode, not once per poll
- **AND** the fault SHALL be published in telemetry

#### Scenario: Automatic recovery
- **WHEN** the input reader is faulted
- **THEN** a device/bus re-initialization SHALL be attempted every `BOARD_INPUT_RECOVER_MS` (1000 ms)
- **AND** on the first successful read the fault SHALL clear, `valid` SHALL return to `true`, and a
  recovery message SHALL be logged
- **AND** no reboot or manual intervention SHALL be required

#### Scenario: Polling rate satisfies the consumers
- **WHEN** the brake overrun logic (`BRAKE_SENSOR_OVERRUN_TIME`, 1000 ms) and the gear read cache
  (`TRANS_GEAR_READ_INTERVAL_MS`, 100 ms) consume the snapshot
- **THEN** the 25 ms poll interval SHALL provide at least one fresh sample per consumer window
- **AND** the gear read's retry-on-ambiguous-result behavior SHALL be satisfied by consuming
  successive snapshots rather than issuing back-to-back I2C reads

### Requirement: Relay Output Expander
The system SHALL drive the external relay board's eight relays through its PCA9557 at address `0x1F`
(`PCA9557_ADDR_RELAYS`, the board's as-shipped `A2A1A0 = 111` strap), replacing direct GPIO relay
outputs, while preserving the existing `RelayController` public API and ignition semantics. The bus
is supplied by the owner at construction: `Wire1` (I2C2), the board's home on header X15.

Relay assignment (aztec buggy). The board's IO routing is **not** 1:1 with the relay numbers
(verified from the `Relay.PrjPcb` netlist), so the firmware SHALL address each function by a
whole-port mask. On the aztec buggy no function is paired, and the relays not assigned below are
reserved and SHALL be driven off except while the web relay test (see "Web Relay Test") energizes
one:

| Relay | PCA9557 IO | Relay-board connector | Function | Mask constant |
|-------|-----------|-----------------------|----------|---------------|
| Relay_1 | IO4 | X4 | Ignition / ECU line (single relay) | `RELAY_MASK_IGNITION` (`0b00010000`) |
| Relay_2 | IO5 | X5 | Starter solenoid | `RELAY_MASK_STARTER` (`0b00100000`) |
| Relay_3 | IO6 | X6 | Front light | `RELAY_MASK_FRONT_LIGHT` (`0b01000000`) |
| Relay_4 | IO7 | X7 | Lock (specified in a later change) — reserved, off outside the relay test | `RELAY_MASK_RESERVED` |
| Relay_5 | IO3 | X8 | Lock (specified in a later change) — reserved, off outside the relay test | `RELAY_MASK_RESERVED` |
| Relay_6 | IO2 | X9 | Lock (specified in a later change) — reserved, off outside the relay test | `RELAY_MASK_RESERVED` |
| Relay_7 | IO0 | X10 | Spare — off outside the relay test | `RELAY_MASK_RESERVED` |
| Relay_8 | IO1 | X11 | Spare — off outside the relay test | `RELAY_MASK_RESERVED` |

`RELAY_MASK_RESERVED` is `0b10001111`. The quad bike's front-wheel lock is not fitted on the aztec
buggy: there SHALL be no `RELAY_MASK_WHEEL_LOCK`, and `RELAY_WHEEL_LOCK_FITTED` SHALL be `0`.

#### Scenario: Initialize relays to the safe state
- **WHEN** `RelayController::begin()` is called with the bus the relay board is wired to
- **THEN** all eight PCA9557 pins SHALL be configured as outputs
- **AND** the output port SHALL be driven to `0x00` (all relays off) before `begin()` returns
- **AND** `true` SHALL be returned only if the device answers at `0x1F` and the all-off write
  verifies

#### Scenario: Report the PCA9557 address block when the relay board is missing
- **WHEN** `begin()` finds no device at `PCA9557_ADDR_RELAYS`
- **THEN** it SHALL probe the whole PCA9557 address range `0x18`-`0x1F` on its own bus
- **AND** it SHALL log the addresses that ACKed, or "none" if the bus is silent
- **AND** the log SHALL NOT attribute an answering address to a mis-strapped relay board, because
  the on-board mux U2 always answers at `0x1C` when the driver runs on I2C2
- **AND** the probe SHALL be bounded (eight zero-length writes) and SHALL NOT block the control loop
- **AND** `begin()` SHALL return `false` and the driver SHALL NOT command any other device

#### Scenario: Whole-port writes from a shadow byte
- **WHEN** any relay state changes
- **THEN** the driver SHALL update an 8-bit output shadow byte and write the entire output port
- **AND** each function SHALL be set or cleared as a whole `RELAY_MASK_*` mask
- **AND** every `RELAY_MASK_RESERVED` bit (Relay_4..Relay_8) SHALL be written as off unless the web
  relay test energizes that relay
- **AND** the function masks and `RELAY_MASK_RESERVED` SHALL be disjoint and together cover all eight
  port bits, checked at compile time
- **AND** no read-modify-write of the output register SHALL be used

#### Scenario: The non-1:1 IO routing is honored
- **WHEN** the firmware computes the output shadow byte
- **THEN** it SHALL use the `Relay.PrjPcb` routing `Relay_1=IO4, Relay_2=IO5, Relay_3=IO6,
  Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`
- **AND** it SHALL NOT assume `Relay_N` maps to port bit `N`, because that would command the starter
  when the front light is requested
- **AND** the routing SHALL be recorded in `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 with
  an explicit warning against "correcting" it

#### Scenario: Aztec relay functions
- **WHEN** the ignition state is `ACC` or `IGNITION`
- **THEN** only Relay_1 (IO4) SHALL be energized and the port SHALL read back `0x10`
- **WHEN** the ignition state is `CRANKING`
- **THEN** Relay_1 (IO4) and Relay_2 (IO5) SHALL be energized and the port SHALL read back `0x30`
- **WHEN** the front light is on
- **THEN** Relay_3 (IO6) SHALL be energized
- **AND** outside the web relay test no function or ignition state SHALL energize Relay_4, Relay_5,
  Relay_6, Relay_7 or Relay_8

#### Scenario: Public API is unchanged for callers
- **WHEN** existing callers use `setIgnitionState()`, `requestCrank()`, `setFrontLight()`,
  `setWheelLock()`, `update()`, `allOff()` or the state getters
- **THEN** their signatures and observable behavior SHALL be identical to the GPIO implementation
- **AND** the `ACC_PRECRANK_DWELL_MS` pre-crank dwell, `CRANKING_TIMEOUT`, and RPM-based crank exit
  SHALL be unchanged
- **AND** `setWheelLock()` SHALL remain callable, but SHALL reject a lock request because the
  front-wheel lock is not fitted (see the "Front-Wheel Lock Not Fitted" requirement in
  `vehicle-systems`)

### Requirement: Relay Write Verification and Fault Policy
The system SHALL verify every relay output write by reading the PCA9557 output register back, and
SHALL escalate a verification failure differently for the starter than for other relays, because a
latched starter is destructive.

#### Scenario: Verify each write by read-back
- **WHEN** the output port is written
- **THEN** the output register SHALL be read back and compared against the shadow byte
- **AND** on mismatch or transaction failure the write SHALL be retried up to `RELAY_WRITE_RETRIES`
  (3) times
- **AND** if it still does not verify, `isFaulted()` SHALL return `true` and the failure SHALL be
  logged

#### Scenario: Starter engagement requires a verified read-back
- **WHEN** the ignition state enters `CRANKING` and `RELAY_MASK_STARTER` is written
- **AND** the read-back does not confirm the starter mask is set
- **THEN** the crank SHALL be aborted
- **AND** the output port SHALL be rewritten with `RELAY_MASK_STARTER` clear
- **AND** the ignition state SHALL leave `CRANKING`

#### Scenario: Relay fault while the starter is engaged
- **WHEN** a write verification fails while any `RELAY_MASK_STARTER` bit is set in the written byte
- **THEN** an all-relays-off port write SHALL be attempted immediately
- **AND** it SHALL be repeated on every `update()` while the fault persists
- **AND** the ignition state SHALL be latched to `OFF`
- **AND** further crank requests SHALL be refused until a write verifies cleanly again

#### Scenario: Degraded operation when the relay expander is unavailable
- **WHEN** `begin()` failed or the driver is faulted
- **THEN** every relay setter SHALL become a logging no-op
- **AND** `getIgnitionState()` SHALL report `OFF`
- **AND** the rest of the vehicle (steering, throttle, brake, telemetry) SHALL continue to operate
- **AND** the fault SHALL be published in telemetry

#### Scenario: Welded contacts are out of scope for firmware detection
- **WHEN** a relay contact is physically welded closed
- **THEN** the output-register read-back SHALL still verify, because it reflects the expander pin and
  not the contact
- **AND** this limitation SHALL be documented rather than worked around in firmware

### Requirement: Reserved On-Board Mux Expander
The system SHALL treat the on-board CD4051 mux-control PCA9557 U2 at address `0x1C` on `Wire1` as
reserved and SHALL NOT address it in this change, other than as part of the read-only diagnostic
address sweep in `RelayController::begin()`.

#### Scenario: Mux expander is never written
- **WHEN** the firmware operates the relay board
- **THEN** no write transaction SHALL be addressed to `0x1C`
- **AND** the CD4051 channel-select and enable lines SHALL remain in their power-on state
- **AND** `PIN_ADC_IS_MUX` (GPIO3) SHALL be defined in `Constants.h` but not sampled

#### Scenario: Address is documented as taken
- **WHEN** a future device is added to `Wire1`
- **THEN** `0x1C` SHALL already be recorded in `Constants.h` as `PCA9557_ADDR_MUX_RESERVED`
- **AND** `0x1A` (U10) SHALL likewise be recorded as `PCA9557_ADDR_INPUTS`
- **AND** the relay board SHALL be strapped to any PCA9557 address other than `0x1A` or `0x1C` — the
  as-shipped `A2A1A0 = 111` → `0x1F` satisfies this and SHALL be kept

### Requirement: Web Relay Test
The system SHALL let the operator energize any single relay of the relay board (Relay_1..Relay_8)
for a wiring check, through `RelayController::setRelayTest(relayNumber, on, reason)`. Relay numbers
SHALL be mapped to port bits through the `Relay.PrjPcb` routing (`relayIoMask()`: Relay_1=IO4,
Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1), checked at
compile time against `RELAY_MASK_IGNITION`, `RELAY_MASK_STARTER`, `RELAY_MASK_FRONT_LIGHT` and
`RELAY_MASK_RESERVED`. Test bits SHALL be part of the same output shadow byte and the same verified
whole-port write as the vehicle functions. Every change of a test relay SHALL be logged under the
`RELAY` debug feature.

#### Scenario: Toggle a relay on
- **WHEN** a relay test ON request names a relay 1..8
- **AND** the ignition state is `OFF`, no crank is armed, the front light is off and the expander is
  available
- **THEN** that relay's IO bit SHALL be set in the shadow byte and written with read-back verification
- **AND** the request SHALL succeed and the relay SHALL be reported in the `relay_test` bitfield

#### Scenario: Toggle a relay off
- **WHEN** a relay test OFF request names a relay that is test-energized
- **THEN** its bit SHALL be cleared and the port rewritten
- **AND** an OFF request for a relay that is not test-energized SHALL succeed without a port write

#### Scenario: Auto-off after the test timeout
- **WHEN** a test relay other than Relay_2 has been on for `RELAY_TEST_TIMEOUT_MS` (30000 ms)
- **THEN** `update()` SHALL clear it and rewrite the port, and log the auto-off

#### Scenario: Starter relay test is capped
- **WHEN** Relay_2 (the starter relay) has been test-energized for `RELAY_TEST_STARTER_MAX_MS`
  (2000 ms)
- **THEN** `update()` SHALL clear it and rewrite the port

#### Scenario: Test refused while the ignition is not OFF
- **WHEN** a relay test ON request arrives while the ignition state is `ACC`, `IGNITION` or
  `CRANKING`, a crank is armed, or the front light is on
- **THEN** the request SHALL be refused with an error text (e.g. "Relay test only allowed with
  ignition OFF") and no port write SHALL be made

#### Scenario: Relay number outside 1..8 is rejected
- **WHEN** a relay test request names relay 0, 9 or any other number outside 1..8
- **THEN** it SHALL be refused with the error text "Relay number must be 1-8" and no relay SHALL change

#### Scenario: A real function command takes over
- **WHEN** an ignition command to `ACC`, `IGNITION` or `CRANKING`, a crank request, or a front-light
  ON command arrives from the web or MAVLink path while any test relay is on
- **THEN** every test bit SHALL be cleared before the function relays are written, in the same port
  write, and the forced-off SHALL be logged

#### Scenario: Test relays forced off on fault or fail-safe
- **WHEN** `allOff()` runs (fail-safe), or a relay port write does not verify (including the starter
  escalation)
- **THEN** every test bit SHALL be cleared immediately and the reduced shadow byte written
- **AND** a test ON request SHALL be refused while the expander is unavailable

