## MODIFIED Requirements
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

## ADDED Requirements
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
