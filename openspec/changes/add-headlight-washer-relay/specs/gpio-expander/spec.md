## MODIFIED Requirements
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
while the web relay test (see "Web Relay Test") energizes one. Relay_8 drives the headlight washer
pump (see `vehicle-systems` "Headlight Washer Pulse") on both vehicles; Relay_7 SHALL stay reserved
(owner decision 2026-11-25: Relay_7 was faulty on some relay boards):

| Relay | PCA9557 IO (connector) | XL9535 bit | Function | Set constant |
|-------|------------------------|-----------|----------|--------------|
| Relay_1 | IO4 (X4) | P0.0 | Ignition / ECU line (single relay) | `RELAY_SET_IGNITION` |
| Relay_2 | IO5 (X5) | P0.1 | Starter solenoid | `RELAY_SET_STARTER` |
| Relay_3 | IO6 (X6) | P0.2 | Front light | `RELAY_SET_FRONT_LIGHT` |
| Relay_4 | IO7 (X7) | P0.3 | Lock (specified in a later change) — reserved | `RELAY_SET_RESERVED` |
| Relay_5 | IO3 (X8) | P0.4 | Lock (specified in a later change) — reserved | `RELAY_SET_RESERVED` |
| Relay_6 | IO2 (X9) | P0.5 | Lock (specified in a later change) — reserved | `RELAY_SET_RESERVED` |
| Relay_7 | IO0 (X10) | P0.6 | Spare | `RELAY_SET_RESERVED` |
| Relay_8 | IO1 (X11) | P0.7 | Headlight washer pump (timed pulse) | `RELAY_SET_WASHER` |

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
- **AND** every reserved bit (Relay_4..Relay_7) SHALL be written as off unless the web relay test
  energizes that relay
- **AND** the washer bit (Relay_8) SHALL be written as on only while a headlight-washer pulse runs or
  the web relay test energizes Relay_8
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
- **WHEN** a headlight-washer pulse runs
- **THEN** Relay_8 SHALL be energized (pin bit `0x02` on the PCA9557 board, `0x80` on the XL9535
  board) together with the ignition relay, and released when the pulse ends
- **AND** outside the web relay test no function or ignition state SHALL energize Relay_4, Relay_5,
  Relay_6 or Relay_7, and nothing but a washer pulse SHALL energize Relay_8

#### Scenario: Public API is unchanged for callers
- **WHEN** existing callers use `setIgnitionState()`, `requestCrank()`, `setFrontLight()`,
  `setWheelLock()`, `update()`, `allOff()` or the state getters
- **THEN** their signatures and observable behavior SHALL be identical to the GPIO implementation
- **AND** `CRANKING_TIMEOUT` and the RPM-based crank exit SHALL be unchanged, and the pre-crank dwell
  SHALL be the configured value (see `vehicle-systems` "Configurable Pre-Crank Dwell"; default
  `ACC_PRECRANK_DWELL_MS`, 3000 ms), set through the additional RAM-only
  `setPrecrankDwellMs()`
- **AND** `setWheelLock()` SHALL remain callable, but SHALL reject a lock request because the
  front-wheel lock is not fitted (see the "Front-Wheel Lock Not Fitted" requirement in
  `vehicle-systems`)

### Requirement: Relay Port Mask Construction
The system SHALL define the per-vehicle relay function assignment in `Constants.h` as relay-number
sets (bit0 = Relay_1 … bit7 = Relay_8), not as port masks, and `RelayController::begin()` SHALL
build the function port masks from the detected board's Relay_N → bit table: the PCA9557
`Relay.PrjPcb` table (Relay_1=IO4, Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2,
Relay_7=IO0, Relay_8=IO1) or the XL9535 identity table. Aztec sets: ignition = {1}, starter = {2},
front light = {3}, headlight washer = {8} (`RELAY_SET_WASHER`), reserved = {4..7}.

#### Scenario: Compile-time invariants on both tables
- **WHEN** the firmware or the native tests are compiled
- **THEN** constexpr checks SHALL prove the function sets (washer included) disjoint and, with the
  reserved set, covering Relay_1..Relay_8
- **AND** the washer set SHALL be proven non-empty and equal to Relay_8 alone, so Relay_7 can never be
  part of a function
- **AND** each table SHALL be proven a permutation of the eight port bits
- **AND** the masks mapped through each table SHALL be disjoint and OR to `0xFF`

#### Scenario: Runtime check in begin()
- **WHEN** `begin()` builds the masks for the detected board
- **THEN** it SHALL re-check disjointness and full coverage
- **AND** it SHALL refuse to initialize the relay port if the check fails

#### Scenario: Masks per board
- **WHEN** the aztec sets are mapped through the PCA9557 table
- **THEN** ignition SHALL be `0x10`, starter `0x20`, front light `0x40`, washer `0x02`, reserved
  `0x8D`
- **WHEN** they are mapped through the XL9535 table
- **THEN** ignition SHALL be `0x01`, starter `0x02`, front light `0x04`, washer `0x80`, reserved
  `0x78`

