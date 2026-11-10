## ADDED Requirements
### Requirement: Relay Port Interface
The system SHALL drive the relay board through an abstract `IRelayPort` interface
(`include/IRelayPort.h`, same pattern as `IMotorDriver`) exposing `isPresent()`, `initAllOff()` (and
its shadow-seeded form `initPort(seed)`, used only by the fault recovery),
`writeOutputs(uint8_t)`, `readOutputRegister(uint8_t&)`, `readPins(uint8_t&)`, `address()`, a
name string for logs, and the address range the failure scan probes. `PCA9557Expander` and
`XL9535Expander` SHALL both implement it, and `RelayController` SHALL hold no chip-specific register
logic. Every method SHALL perform at most one I2C transaction (except `initAllOff()`, which performs
a fixed bounded sequence), report the result to its caller, and never retry, sleep or spin.

#### Scenario: Pin read returns the actual pin level
- **WHEN** `readPins(value)` is called on either implementation
- **THEN** the chip's input-port register SHALL be read (PCA9557 `0x00`, XL9535 `0x00` = P0)
- **AND** the value SHALL reflect the pin levels of pins configured as outputs, not the output latch

#### Scenario: Hardware-safe initialization order
- **WHEN** `initAllOff()` is called on either implementation
- **THEN** the output register SHALL be written `0x00` first
- **AND** the polarity-inversion register SHALL then be written `0x00`
- **AND** only then SHALL the configuration register be written `0x00` (all outputs)
- **AND** `readPins()` SHALL then be required to return `0x00`
- **AND** `false` SHALL be returned, naming the failed step in the log, if any step fails
- **AND** the configuration register SHALL never be written before the output register, because the
  XL9535 output latch powers up `0xFF` and would energize all eight relays

#### Scenario: Controller logic is chip-agnostic
- **WHEN** `RelayController` writes, verifies or recovers the relay port
- **THEN** it SHALL do so only through `IRelayPort`
- **AND** no driver instance SHALL be heap-allocated

### Requirement: XL9535 Relay Port Driver
The system SHALL provide `XL9535Expander` for the XL9535-K8V5 relay board (XL9535 =
PCA9535/TCA9535-compatible 16-bit expander, 7-bit address `0x20`..`0x27` via pads A0..A2, default
`0x20`, no RESET pin), bound to one `TwoWire` bus and one address at construction. It SHALL use port
P0 only: input `0x00`, output `0x02`, polarity `0x04`, configuration `0x06`. Port P1 (`0x01`,
`0x03`, `0x05`, `0x07`) SHALL never be written and SHALL stay at its power-on default (inputs). The
bus SHALL stay at `I2C_BUS_FREQ_HZ` (100 kHz).

Relay mapping (identity, active-high: bit 1 = relay energized) — **pending bench confirmation**:

| Relay | XL9535 bit |
|-------|-----------|
| Relay_1..Relay_8 | P0.0..P0.7 |

#### Scenario: Write the relay port
- **WHEN** `writeOutputs(value)` is called
- **THEN** register `0x02` SHALL be written with `value` in a single transaction
- **AND** no P1 register SHALL be addressed

#### Scenario: P1 is never touched
- **WHEN** the XL9535 driver initializes, writes, verifies or recovers
- **THEN** no write SHALL be addressed to registers `0x03`, `0x05` or `0x07`

#### Scenario: Mapping is bench-verified before use
- **WHEN** the identity mapping is first used on a K8 board
- **THEN** a bench check (Output P0 = `0x01`, then Config P0 = `0xFE`, Relay_1 clicks; active-high
  confirmed) SHALL have been recorded in `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6
- **AND** a different observed mapping SHALL change only the XL9535 table

### Requirement: Relay Board Selection and Detection
The system SHALL select the relay board at runtime from the persisted setting `relay_board` ∈
{`auto`, `pca9557`, `xl9535`} (NVS namespace `relay`, key `board`, default `auto`; an absent or
out-of-range value SHALL be treated as `auto`, logged, and not rewritten). Detection SHALL run once
in `RelayController::begin()`, before any relay write, by zero-length probes of `0x1F` and
`0x20`..`0x27` only, through the pure function `relayBoardDecide()` in `include/RelayBoardLaw.h`.
The result SHALL be latched until reboot. `0x1A` and `0x1C` SHALL never be treated as a relay board.

#### Scenario: Auto picks the PCA9557 board
- **WHEN** the setting is `auto` and only `0x1F` ACKs among the probed addresses
- **THEN** the PCA9557 relay board at `0x1F` SHALL be used

#### Scenario: Auto picks the XL9535 board
- **WHEN** the setting is `auto`, `0x1F` does not ACK and exactly one of `0x20`..`0x27` ACKs
- **THEN** the XL9535 relay board at that address SHALL be used

#### Scenario: Manual selection resolves two boards
- **WHEN** the setting is `pca9557` and `0x1F` ACKs, or the setting is `xl9535` and exactly one of
  `0x20`..`0x27` ACKs
- **THEN** the selected board SHALL be used whatever else answers

#### Scenario: Ambiguity is a fault, never a guess
- **WHEN** the setting is `auto` and both `0x1F` and an XL9535 address ACK (`both_found`)
- **OR** none of `0x1F`, `0x20`..`0x27` ACKs (`none_found`)
- **OR** the setting is `auto` or `xl9535` and two or more of `0x20`..`0x27` ACK (`multiple_xl9535`)
- **OR** the setting is `pca9557` without `0x1F`, or `xl9535` without any `0x20`..`0x27`
  (`cfg_mismatch`)
- **THEN** no write SHALL be addressed to any device by the relay controller
- **AND** `begin()` SHALL return `false`, `isFaulted()` SHALL be `true` and every relay setter SHALL
  be a logging no-op
- **AND** the reason SHALL be logged once and published in telemetry
- **AND** the fault SHALL NOT be retried before reboot

#### Scenario: Setting changes need a reboot
- **WHEN** the stored setting changes while running
- **THEN** the active board, address and masks SHALL stay as detected at boot

### Requirement: Relay Port Mask Construction
The system SHALL define the per-vehicle relay function assignment in `Constants.h` as relay-number
sets (bit0 = Relay_1 … bit7 = Relay_8), not as port masks, and `RelayController::begin()` SHALL
build the function port masks from the detected board's Relay_N → bit table: the PCA9557
`Relay.PrjPcb` table (Relay_1=IO4, Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2,
Relay_7=IO0, Relay_8=IO1) or the XL9535 identity table. Aztec sets: ignition = {1}, starter = {2},
front light = {3}, reserved = {4..8}.

#### Scenario: Compile-time invariants on both tables
- **WHEN** the firmware or the native tests are compiled
- **THEN** constexpr checks SHALL prove the function sets disjoint and, with the reserved set,
  covering Relay_1..Relay_8
- **AND** each table SHALL be proven a permutation of the eight port bits
- **AND** the masks mapped through each table SHALL be disjoint and OR to `0xFF`

#### Scenario: Runtime check in begin()
- **WHEN** `begin()` builds the masks for the detected board
- **THEN** it SHALL re-check disjointness and full coverage
- **AND** it SHALL refuse to initialize the relay port if the check fails

#### Scenario: Masks per board
- **WHEN** the aztec sets are mapped through the PCA9557 table
- **THEN** ignition SHALL be `0x10`, starter `0x20`, front light `0x40`, reserved `0x8F`
- **WHEN** they are mapped through the XL9535 table
- **THEN** ignition SHALL be `0x01`, starter `0x02`, front light `0x04`, reserved `0xF8`

### Requirement: Relay Expander Reset Recovery
The system SHALL recover a faulted relay port without a reboot once a board has been detected: every
`RELAY_RECOVER_MS` (1000 ms) while faulted, `update()` SHALL re-initialize the port seeded with the
current shadow byte — output register = shadow, then polarity `0x00`, then configuration `0x00`,
then a pin read-back that SHALL equal the shadow — and then make one verified write of the shadow
byte (all-off while the starter escalation is latched), in place of that call's single-attempt
write. The starter bit SHALL never be seeded by a recovery. Between recovery attempts the existing
single-attempt write per `update()` SHALL be kept so the main loop stays non-blocking. At boot,
`begin()` SHALL keep the all-off `initAllOff()` sequence.

#### Scenario: Recovery seeds the latch with the shadow
- **WHEN** a recovery attempt runs while the shadow byte has the ignition relay set and no starter
  bit
- **THEN** the output register SHALL be written with the shadow byte before the polarity and
  configuration registers
- **AND** the pins SHALL be required to read back the shadow byte
- **AND** an energized ignition relay SHALL NOT be driven off by the recovery when the expander had
  not actually reset

#### Scenario: Starter bit is never seeded
- **WHEN** a recovery attempt is due while the shadow byte has a starter-mask bit set
- **THEN** the starter escalation SHALL run first (all-off write, ignition latched `OFF`, crank
  inhibit)
- **AND** the recovery SHALL seed the output register with that all-off state

#### Scenario: Silent XL9535 reset is detected and recovered
- **WHEN** the XL9535 loses VIN briefly and returns with its configuration back at all-inputs
- **THEN** the next verified write SHALL fail its pin read-back and raise the fault
- **AND** within `RELAY_RECOVER_MS` of VIN returning, the shadow-seeded re-init and the shadow
  re-write SHALL verify and the fault SHALL clear with a recovery log line

#### Scenario: Detected board that fails to initialize at boot
- **WHEN** detection succeeded but `initAllOff()` fails in `begin()`
- **THEN** the controller SHALL report the fault and the recovery timer SHALL retry the initialization
  on the latched board (seeded with the all-off shadow)
- **AND** no plain port write SHALL be made to that board until a re-init has succeeded

#### Scenario: Recovery is bounded
- **WHEN** a recovery attempt runs on a dead bus
- **THEN** it SHALL cost at most the fixed four-step re-init sequence plus one write and one read
- **AND** it SHALL NOT loop or retry within the same `update()` call

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
`Wire1` SHALL run at 100 kHz (`I2C_BUS_FREQ_HZ`).

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
- **WHEN** a PCA9557 or XL9535 driver instance is constructed
- **THEN** it SHALL take a `TwoWire` reference and a 7-bit address as construction parameters
- **AND** it SHALL only ever address that device
- **AND** no "take the first expander found" behavior SHALL exist; relay-board detection SHALL follow
  the "Relay Board Selection and Detection" requirement, which faults on any ambiguity

### Requirement: Relay Output Expander
The system SHALL drive the relay board's eight relays through the expander selected by "Relay Board
Selection and Detection" — the PCA9557 relay board at `0x1F` (`PCA9557_ADDR_RELAYS`, as-shipped
`A2A1A0 = 111` strap) or the XL9535-K8V5 at `0x20`..`0x27` — through `IRelayPort`, replacing direct
GPIO relay outputs, while preserving the existing `RelayController` public API and ignition
semantics. The bus is supplied by the owner at construction: `Wire1` (I2C2), header X15.

Relay assignment (aztec buggy). The PCA9557 board's IO routing is **not** 1:1 with the relay numbers
(verified from the `Relay.PrjPcb` netlist); the XL9535 board's is believed 1:1 (P0.n = Relay_n+1,
pending bench confirmation). The firmware SHALL therefore address each function by a whole-port mask
built at `begin()` from the relay-number set (see "Relay Port Mask Construction"). On the aztec buggy
no function is paired, and the relays not assigned below are reserved and SHALL be driven off except
while the web relay test (see "Web Relay Test") energizes one:

| Relay | PCA9557 IO (connector) | XL9535 bit | Function | Set constant |
|-------|------------------------|-----------|----------|--------------|
| Relay_1 | IO4 (X4) | P0.0 | Ignition / ECU line (single relay) | `RELAY_SET_IGNITION` |
| Relay_2 | IO5 (X5) | P0.1 | Starter solenoid | `RELAY_SET_STARTER` |
| Relay_3 | IO6 (X6) | P0.2 | Front light | `RELAY_SET_FRONT_LIGHT` |
| Relay_4 | IO7 (X7) | P0.3 | Lock (specified in a later change) — reserved | `RELAY_SET_RESERVED` |
| Relay_5 | IO3 (X8) | P0.4 | Lock (specified in a later change) — reserved | `RELAY_SET_RESERVED` |
| Relay_6 | IO2 (X9) | P0.5 | Lock (specified in a later change) — reserved | `RELAY_SET_RESERVED` |
| Relay_7 | IO0 (X10) | P0.6 | Spare | `RELAY_SET_RESERVED` |
| Relay_8 | IO1 (X11) | P0.7 | Spare | `RELAY_SET_RESERVED` |

The quad bike's front-wheel lock is not fitted on the aztec buggy: there SHALL be no wheel-lock set,
and `RELAY_WHEEL_LOCK_FITTED` SHALL be `0`.

#### Scenario: Initialize relays to the safe state
- **WHEN** `RelayController::begin()` is called with the bus the relay board is wired to
- **AND** detection selected a board
- **THEN** `initAllOff()` SHALL configure the port with the mandatory order (output `0x00`, polarity
  `0x00`, configuration `0x00`) and verify the pins read `0x00` before `begin()` returns
- **AND** `true` SHALL be returned only if the board was detected and the all-off state verifies

#### Scenario: Report the PCA9557 address block when the relay board is missing
- **WHEN** detection fails for any reason
- **THEN** `begin()` SHALL probe the range `0x18`-`0x27` on its own bus
- **AND** it SHALL log the detection reason and the addresses that ACKed, or "none" if the bus is
  silent
- **AND** the log SHALL NOT attribute `0x1A` or `0x1C` to a relay board, because the on-board
  expanders always answer there on I2C2
- **AND** the probe SHALL be bounded (sixteen zero-length writes) and SHALL NOT block the control
  loop
- **AND** `begin()` SHALL return `false` and the driver SHALL NOT command any device

#### Scenario: Whole-port writes from a shadow byte
- **WHEN** any relay state changes
- **THEN** the driver SHALL update an 8-bit output shadow byte and write the entire output port
- **AND** each function SHALL be set or cleared as its whole runtime mask
- **AND** every reserved bit (Relay_4..Relay_8) SHALL be written as off unless the web relay test
  energizes that relay
- **AND** no read-modify-write of the output register SHALL be used

#### Scenario: The non-1:1 IO routing is honored
- **WHEN** the PCA9557 board is active and the firmware computes the output shadow byte
- **THEN** it SHALL use the `Relay.PrjPcb` routing `Relay_1=IO4, Relay_2=IO5, Relay_3=IO6,
  Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`
- **AND** it SHALL NOT assume `Relay_N` maps to port bit `N`, because that would command the starter
  when the front light is requested
- **AND** the routing SHALL be recorded in `RelayBoardLaw.h`, `Constants.h` and
  `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 with an explicit warning against "correcting" it

#### Scenario: Aztec relay functions
- **WHEN** the ignition state is `ACC` or `IGNITION`
- **THEN** only Relay_1 SHALL be energized and the pins SHALL read back `0x10` on the PCA9557 board
  or `0x01` on the XL9535 board
- **WHEN** the ignition state is `CRANKING`
- **THEN** Relay_1 and Relay_2 SHALL be energized and the pins SHALL read back `0x30` (PCA9557) or
  `0x03` (XL9535)
- **WHEN** the front light is on
- **THEN** Relay_3 SHALL be energized
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
The system SHALL verify every relay output write by reading the expander's **pin levels** back
(`IRelayPort::readPins()`, the input-port register) and comparing them with the shadow byte, so that
a silent expander reset (configuration back to inputs, relays dropped, output latch unchanged) is
detected. It SHALL escalate a verification failure differently for the starter than for other
relays, because a latched starter is destructive.

#### Scenario: Verify each write by read-back
- **WHEN** the output port is written
- **THEN** the pins SHALL be read back and compared against the shadow byte
- **AND** on mismatch or transaction failure the write SHALL be retried up to `RELAY_WRITE_RETRIES`
  (3) times
- **AND** if it still does not verify, `isFaulted()` SHALL return `true` and the failure SHALL be
  logged

#### Scenario: Reset expander fails verification
- **WHEN** the expander has reset to its power-on configuration (all pins inputs)
- **AND** the output port is written
- **THEN** the pin read-back SHALL NOT match the shadow byte and the fault SHALL be raised, even if
  the output latch reads back the written value

#### Scenario: Starter engagement requires a verified read-back
- **WHEN** the ignition state enters `CRANKING` and the starter mask is written
- **AND** the pin read-back does not confirm the starter mask is set
- **THEN** the crank SHALL be aborted
- **AND** the output port SHALL be rewritten with the starter mask clear
- **AND** the ignition state SHALL leave `CRANKING`

#### Scenario: Relay fault while the starter is engaged
- **WHEN** a write verification fails while any starter-mask bit is set in the written byte
- **THEN** an all-relays-off port write SHALL be attempted immediately
- **AND** it SHALL be repeated on every `update()` while the fault persists, as a single attempt per
  call, with the timed re-init recovery of "Relay Expander Reset Recovery"
- **AND** the ignition state SHALL be latched to `OFF`
- **AND** further crank requests SHALL be refused until a write verifies cleanly again

#### Scenario: Degraded operation when the relay expander is unavailable
- **WHEN** `begin()` failed (detection or initialization) or the driver is faulted
- **THEN** every relay setter SHALL become a logging no-op
- **AND** `getIgnitionState()` SHALL report `OFF`
- **AND** the rest of the vehicle (steering, throttle, brake, telemetry) SHALL continue to operate
- **AND** the fault SHALL be published in telemetry

#### Scenario: Welded contacts are out of scope for firmware detection
- **WHEN** a relay contact is physically welded closed
- **THEN** the pin read-back SHALL still verify, because it reflects the expander pin and not the
  contact
- **AND** this limitation SHALL be documented rather than worked around in firmware

### Requirement: Web Relay Test
The system SHALL let the operator energize any single relay of the relay board (Relay_1..Relay_8)
for a wiring check, through `RelayController::setRelayTest(relayNumber, on, reason)`. Relay numbers
SHALL be mapped to port bits through the active board's Relay_N → bit table (`relayIoMask()`:
PCA9557 board Relay_1=IO4, Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2,
Relay_7=IO0, Relay_8=IO1; XL9535 board Relay_N = P0.(N-1)), checked at compile time for both tables
against the function and reserved relay-number sets. Test bits SHALL be part of the same output
shadow byte and the same verified whole-port write as the vehicle functions. Every change of a test
relay SHALL be logged under the `RELAY` debug feature.

#### Scenario: Toggle a relay on
- **WHEN** a relay test ON request names a relay 1..8
- **AND** the ignition state is `OFF`, no crank is armed, the front light is off and the expander is
  available
- **THEN** that relay's bit in the active table SHALL be set in the shadow byte and written with pin
  read-back verification
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
- **AND** a test ON request SHALL be refused while the expander is unavailable or no board was
  detected
