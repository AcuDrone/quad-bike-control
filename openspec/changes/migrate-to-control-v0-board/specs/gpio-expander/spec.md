## ADDED Requirements

### Requirement: Dual I2C Bus Topology and Address Map
The system SHALL operate two independent I2C buses with a fixed device-to-bus assignment, and each
expander driver instance SHALL be bound to exactly one bus and one 7-bit address at construction.

Bus assignment:

| Bus | Arduino instance | SDA | SCL | Speed | Devices |
|-----|------------------|-----|-----|-------|---------|
| I2C1 | `Wire` | GPIO1 | GPIO2 | 100 kHz | AS5600 steering sensor `0x36`, opto-input PCA9557 U10 `0x1A` |
| I2C2 | `Wire1` | GPIO48 | GPIO47 | 100 kHz | Relay-board PCA9557 `0x18`, on-board mux PCA9557 U2 `0x1C` |

Both buses SHALL run at 100 kHz (`I2C_BUS_FREQ_HZ`): the AS5600 is reached over a long cable run to
the steering column (X14), where signal-integrity margin is worth more than bus speed.

#### Scenario: Initialize both buses before any I2C consumer
- **WHEN** `setup()` runs
- **THEN** `Wire` SHALL be started on `PIN_STEER_SDA` (GPIO1) / `PIN_STEER_SCL` (GPIO2) at
  `I2C_BUS_FREQ_HZ`
- **AND** `Wire1` SHALL be started on `PIN_RELAY_SDA` (GPIO48) / `PIN_RELAY_SCL` (GPIO47) at
  `I2C_BUS_FREQ_HZ`
- **AND** both buses SHALL be up before the AS5600 sensor, the opto-input reader, or the relay
  controller are initialized
- **AND** initialization of one bus SHALL NOT depend on a device on the other bus responding

#### Scenario: AS5600 and the input expander share I2C1
- **WHEN** the steering position sensor and the opto-input expander are both in use
- **THEN** both SHALL be addressed on `Wire` without re-initializing the bus
- **AND** a failure of one device SHALL NOT prevent the other from being read

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
U10 at address `0x1A` on `Wire`, replacing direct GPIO reads for the gear selector and the brake
limit sensor.

Input assignment:

| Input | Connector | Function |
|-------|-----------|----------|
| In1 | X16 | Gear REVERSE |
| In2 | X16 | Gear NEUTRAL |
| In3 | X16 | Gear LOW |
| In4 | X16 | Gear HIGH |
| In5 | X17 | Brake limit sensor ("released" endstop) |
| In6-In8 | X17 | Spare — no firmware function |

#### Scenario: Initialize the input expander
- **WHEN** `BoardInputs::begin()` is called with the `Wire` bus
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
- **THEN** it SHALL return the four gear-input states, the brake-released state, a `valid` flag, and
  the age in milliseconds of the last successful read

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
The system SHALL drive the external relay board's eight relays through its PCA9557 at address `0x18`
on `Wire1`, replacing direct GPIO relay outputs, while preserving the existing `RelayController`
public API and ignition semantics.

Relay assignment. The board's IO routing is **not** 1:1 with the relay numbers (verified from the
`Relay.PrjPcb` netlist), and two functions drive a paralleled PAIR of relays that always energize
together, so the firmware SHALL address each function by a whole-port mask:

| Relay | PCA9557 IO | Relay-board connector | Function | Mask constant |
|-------|-----------|-----------------------|----------|---------------|
| Relay_1 | IO4 | X4 | Ignition / ECU line | `RELAY_MASK_IGNITION` (`0b00110000`) |
| Relay_2 | IO5 | X5 | Ignition / ECU line (paired duplicate) | `RELAY_MASK_IGNITION` |
| Relay_3 | IO6 | X6 | Starter solenoid | `RELAY_MASK_STARTER` (`0b01000000`) |
| Relay_4 | IO7 | X7 | Front light | `RELAY_MASK_FRONT_LIGHT` (`0b10000000`) |
| Relay_5 | IO3 | X8 | Front-wheel lock | `RELAY_MASK_WHEEL_LOCK` (`0b00001100`) |
| Relay_6 | IO2 | X9 | Front-wheel lock (paired duplicate) | `RELAY_MASK_WHEEL_LOCK` |
| Relay_7 | IO0 | X10 | Spare — always driven off | — |
| Relay_8 | IO1 | X11 | Spare — always driven off | — |

#### Scenario: Initialize relays to the safe state
- **WHEN** `RelayController::begin()` is called with the `Wire1` bus
- **THEN** all eight PCA9557 pins SHALL be configured as outputs
- **AND** the output port SHALL be driven to `0x00` (all relays off) before `begin()` returns
- **AND** `true` SHALL be returned only if the device answers at `0x18` and the all-off write
  verifies

#### Scenario: Detect a relay board strapped to the reserved mux address
- **WHEN** `begin()` finds no device at `0x18`
- **THEN** it SHALL additionally probe `0x1C`
- **AND** if `0x1C` is the only expander answering on `Wire1`, a distinct error SHALL be logged
  identifying the relay board as mis-strapped to the on-board mux address
- **AND** `begin()` SHALL return `false` and the driver SHALL NOT command that device

#### Scenario: Whole-port writes from a shadow byte
- **WHEN** any relay state changes
- **THEN** the driver SHALL update an 8-bit output shadow byte and write the entire output port
- **AND** each function SHALL be set or cleared as a whole `RELAY_MASK_*` mask, so both relays of a
  paired function (ignition Relay_1+Relay_2, wheel lock Relay_5+Relay_6) always change together
- **AND** the spare relays Relay_7 (IO0) and Relay_8 (IO1) SHALL always be written as off
- **AND** no read-modify-write of the output register SHALL be used

#### Scenario: The non-1:1 IO routing is honored
- **WHEN** the firmware computes the output shadow byte
- **THEN** it SHALL use the `Relay.PrjPcb` routing `Relay_1=IO4, Relay_2=IO5, Relay_3=IO6,
  Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`
- **AND** it SHALL NOT assume `Relay_N` maps to port bit `N`, because that would command the starter
  when the front light is requested
- **AND** the routing SHALL be recorded in `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 with
  an explicit warning against "correcting" it

#### Scenario: Public API is unchanged for callers
- **WHEN** existing callers use `setIgnitionState()`, `requestCrank()`, `setFrontLight()`,
  `setWheelLock()`, `update()`, `allOff()` or the state getters
- **THEN** their signatures and observable behavior SHALL be identical to the GPIO implementation
- **AND** the `ACC_PRECRANK_DWELL_MS` pre-crank dwell, `CRANKING_TIMEOUT`, and RPM-based crank exit
  SHALL be unchanged
- **AND** `allOff()` SHALL continue to leave the front-wheel lock in its last state

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
reserved and SHALL NOT address it in this change.

#### Scenario: Mux expander is never written
- **WHEN** the firmware operates the relay board on `Wire1`
- **THEN** no transaction SHALL be addressed to `0x1C`
- **AND** the CD4051 channel-select and enable lines SHALL remain in their power-on state
- **AND** `PIN_ADC_IS_MUX` (GPIO3) SHALL be defined in `Constants.h` but not sampled

#### Scenario: Address is documented as taken
- **WHEN** a future device is added to `Wire1`
- **THEN** `0x1C` SHALL already be recorded in `Constants.h` as `PCA9557_ADDR_MUX_RESERVED`
- **AND** the relay board SHALL be strapped `A2A1A0 = 000` so it never collides with it

## REMOVED Requirements

### Requirement: MCP23017 I2C GPIO Expander
**Reason**: The `Control_v0` board uses two PCA9557 expanders (opto inputs at `0x1A` on I2C1, relay
outputs at `0x18` on I2C2), not an MCP23017. The MCP23017 was specified by an earlier archived
change but was never fitted or implemented — no `MCP23017` symbol exists anywhere in `src/` or
`include/`; the gear switches, brake sensor and relays were on direct ESP32 GPIO until this change.

**Migration**: Superseded by the `PCA9557 Register Driver`, `Opto-Isolated Input Expander` and
`Relay Output Expander` requirements added by this change. No code migration is required because no
MCP23017 code exists.

### Requirement: MCP23017 Error Handling
**Reason**: Tied to the MCP23017 device that is not present on `Control_v0` and was never
implemented.

**Migration**: Superseded by `Opto Input Polling and Fault Policy` (last-known-value with staleness,
fault flag and automatic recovery) and `Relay Write Verification and Fault Policy` (read-back
verification with starter escalation), which specify stricter behavior than the removed requirement.

### Requirement: MCP23017 Pin Allocation
**Reason**: The pin allocation described Port A/Port B of an MCP23017 that does not exist on this
board. `Control_v0` splits the same functions across two 8-bit PCA9557 devices on two different
buses.

**Migration**: Superseded by the input assignment table (In1-In5 → REVERSE/NEUTRAL/LOW/HIGH/brake
limit) in `Opto-Isolated Input Expander` and the relay assignment table (Relay_1+Relay_2 →
ignition/ECU line, Relay_3 → starter, Relay_4 → front light, Relay_5+Relay_6 → front-wheel lock,
Relay_7/Relay_8 spare) in `Relay Output Expander`.
