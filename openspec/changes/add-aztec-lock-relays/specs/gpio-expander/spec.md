## MODIFIED Requirements
### Requirement: Relay Output Expander
The system SHALL drive the relay board's eight relays through the expander selected by "Relay Board
Selection and Detection" — the PCA9557 relay board at `0x1F` (`PCA9557_ADDR_RELAYS`, as-shipped
`A2A1A0 = 111` strap) or the XL9535-K8V5 at `0x20`..`0x27` — through `IRelayPort`, replacing direct
GPIO relay outputs, while preserving the existing `RelayController` public API and ignition
semantics. The bus is supplied by the owner at construction: `Wire1` (I2C2), header X15.

Relay assignment (aztec buggy). The PCA9557 board's IO routing is **not** 1:1 with the relay numbers
(verified from the `Relay.PrjPcb` netlist); the XL9535 board's is 1:1 (P0.n = Relay_n+1,
bench-confirmed). The firmware SHALL therefore address each function by a whole-port mask built at
`begin()` from the relay-number set (see "Relay Port Mask Construction"). On the aztec buggy the lock
is the only grouped function — Relay_4, Relay_5 and Relay_6 are switched together — and the relays
not assigned below are reserved and SHALL be driven off except while the web relay test (see "Web
Relay Test") energizes one:

| Relay | PCA9557 IO (connector) | XL9535 bit | Function | Set constant |
|-------|------------------------|-----------|----------|--------------|
| Relay_1 | IO4 (X4) | P0.0 | Ignition / ECU line (single relay) | `RELAY_SET_IGNITION` |
| Relay_2 | IO5 (X5) | P0.1 | Starter solenoid | `RELAY_SET_STARTER` |
| Relay_3 | IO6 (X6) | P0.2 | Front light | `RELAY_SET_FRONT_LIGHT` |
| Relay_4 | IO7 (X7) | P0.3 | Lock (with Relay_5 + Relay_6) | `RELAY_SET_WHEEL_LOCK` |
| Relay_5 | IO3 (X8) | P0.4 | Lock (with Relay_4 + Relay_6) | `RELAY_SET_WHEEL_LOCK` |
| Relay_6 | IO2 (X9) | P0.5 | Lock (with Relay_4 + Relay_5) | `RELAY_SET_WHEEL_LOCK` |
| Relay_7 | IO0 (X10) | P0.6 | Spare | `RELAY_SET_RESERVED` |
| Relay_8 | IO1 (X11) | P0.7 | Spare | `RELAY_SET_RESERVED` |

The lock is fitted on the aztec buggy: `RELAY_SET_WHEEL_LOCK` SHALL be Relay_4 | Relay_5 | Relay_6
and `RELAY_WHEEL_LOCK_FITTED` SHALL be `1`.

#### Scenario: Initialize relays to the safe state
- **WHEN** `RelayController::begin()` is called with the bus the relay board is wired to
- **AND** detection selected a board
- **THEN** `initAllOff()` SHALL configure the port with the mandatory order (output `0x00`, polarity
  `0x00`, configuration `0x00`) and verify the pins read `0x00` before `begin()` returns
- **AND** `true` SHALL be returned only if the board was detected and the all-off state verifies
- **AND** the lock SHALL start released

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
- **AND** each function SHALL be set or cleared as its whole runtime mask, so the three lock relays
  always change together in one write
- **AND** every reserved bit (Relay_7, Relay_8) SHALL be written as off unless the web relay test
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
- **WHEN** the ignition state is `ACC` or `IGNITION` and the light and lock are off
- **THEN** only Relay_1 SHALL be energized and the pins SHALL read back `0x10` on the PCA9557 board
  or `0x01` on the XL9535 board
- **WHEN** the ignition state is `CRANKING`
- **THEN** Relay_1 and Relay_2 SHALL be energized and the pins SHALL read back `0x30` (PCA9557) or
  `0x03` (XL9535), plus the light and lock bits if those are on
- **WHEN** the front light is on
- **THEN** Relay_3 SHALL be energized
- **WHEN** the lock is engaged
- **THEN** Relay_4, Relay_5 and Relay_6 SHALL be energized together — pins `0x8C` (IO7|IO3|IO2) on
  the PCA9557 board or `0x38` (P0.3..P0.5) on the XL9535 board, with the ignition OFF and the light
  off
- **AND** outside the web relay test no function or ignition state SHALL energize Relay_7 or Relay_8

#### Scenario: Public API is unchanged for callers
- **WHEN** existing callers use `setIgnitionState()`, `requestCrank()`, `setFrontLight()`,
  `setWheelLock()`, `update()`, `allOff()` or the state getters
- **THEN** their signatures and observable behavior SHALL be identical to the GPIO implementation
- **AND** the `ACC_PRECRANK_DWELL_MS` pre-crank dwell, `CRANKING_TIMEOUT`, and RPM-based crank exit
  SHALL be unchanged
- **AND** `setWheelLock()` SHALL drive the lock mask (see the "Aztec Lock on Relay_4 + Relay_5 +
  Relay_6" requirement in `vehicle-systems`)

### Requirement: Web Relay Test
The system SHALL let the operator energize any single relay of the relay board (Relay_1..Relay_8)
for a wiring check, through `RelayController::setRelayTest(relayNumber, on, reason)`. Relay numbers
SHALL be mapped to port bits through the active board's Relay_N → bit table (`relayIoMask()`:
PCA9557 board Relay_1=IO4, Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2,
Relay_7=IO0, Relay_8=IO1; XL9535 board Relay_N = P0.(N-1)), checked at compile time for both tables
against the function and reserved relay-number sets. Test bits SHALL be part of the same output
shadow byte and the same verified whole-port write as the vehicle functions. Every change of a test
relay SHALL be logged under the `RELAY` debug feature. Each of the three lock relays (Relay_4,
Relay_5, Relay_6) SHALL be testable on its own.

#### Scenario: Toggle a relay on
- **WHEN** a relay test ON request names a relay 1..8
- **AND** the ignition state is `OFF`, no crank is armed, the front light is off, the lock is
  released and the expander is available
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

#### Scenario: Test refused while the lock is engaged
- **WHEN** a relay test ON request arrives while the lock is engaged
- **THEN** the request SHALL be refused with an error text naming the wheel lock and no port write
  SHALL be made

#### Scenario: Relay number outside 1..8 is rejected
- **WHEN** a relay test request names relay 0, 9 or any other number outside 1..8
- **THEN** it SHALL be refused with the error text "Relay number must be 1-8" and no relay SHALL change

#### Scenario: A real function command takes over
- **WHEN** an ignition command to `ACC`, `IGNITION` or `CRANKING`, a crank request, a front-light
  ON command or a LOCK command arrives from the web or MAVLink path while any test relay is on
- **THEN** every test bit SHALL be cleared before the function relays are written, in the same port
  write, and the forced-off SHALL be logged

#### Scenario: Test relays forced off on fault or fail-safe
- **WHEN** `allOff()` runs (fail-safe), or a relay port write does not verify (including the starter
  escalation)
- **THEN** every test bit SHALL be cleared immediately and the reduced shadow byte written
- **AND** a test ON request SHALL be refused while the expander is unavailable or no board was
  detected

### Requirement: Relay Port Mask Construction
The system SHALL define the per-vehicle relay function assignment in `Constants.h` as relay-number
sets (bit0 = Relay_1 … bit7 = Relay_8), not as port masks, and `RelayController::begin()` SHALL
build the function port masks from the detected board's Relay_N → bit table: the PCA9557
`Relay.PrjPcb` table (Relay_1=IO4, Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2,
Relay_7=IO0, Relay_8=IO1) or the XL9535 identity table. Aztec sets: ignition = {1}, starter = {2},
front light = {3}, lock = {4, 5, 6}, reserved = {7, 8}.

#### Scenario: Compile-time invariants on both tables
- **WHEN** the firmware or the native tests are compiled
- **THEN** constexpr checks SHALL prove the function sets (lock included) disjoint and, with the
  reserved set, covering Relay_1..Relay_8
- **AND** each table SHALL be proven a permutation of the eight port bits
- **AND** the masks mapped through each table SHALL be disjoint and OR to `0xFF`
- **AND** the aztec masks below SHALL be pinned by static assertions for both boards

#### Scenario: Runtime check in begin()
- **WHEN** `begin()` builds the masks for the detected board
- **THEN** it SHALL re-check disjointness and full coverage
- **AND** it SHALL refuse to initialize the relay port if the check fails
- **AND** it SHALL log the lock mask together with the other function masks

#### Scenario: Masks per board
- **WHEN** the aztec sets are mapped through the PCA9557 table
- **THEN** ignition SHALL be `0x10`, starter `0x20`, front light `0x40`, lock `0x8C`, reserved `0x03`
- **WHEN** they are mapped through the XL9535 table
- **THEN** ignition SHALL be `0x01`, starter `0x02`, front light `0x04`, lock `0x38`, reserved `0xC0`
