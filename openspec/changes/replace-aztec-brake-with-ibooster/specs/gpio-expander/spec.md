## MODIFIED Requirements
### Requirement: Opto-Isolated Input Expander
The system SHALL read the eight 5V opto-isolated (PC817) digital inputs through the on-board PCA9557
U10 at address `0x1A` on `Wire1` (I2C2 — scan-verified; the schematic's port names implied I2C1),
replacing direct GPIO reads for the gear selector.

Input assignment (aztec buggy; the quad bike's one-hot four-line gear assignment is kept on
`master`):

| Port bit | Connector | Function |
|----------|-----------|----------|
| 0 | X16 | Gear code bit A — left / MSB (engaged = HIGH) |
| 1 | X16 | Gear code bit B — middle (engaged = HIGH) |
| 6 | X16 | Gear code bit C — right / LSB (engaged = HIGH) |
| 7 | X16 | Not wired on aztec — SHALL be ignored |
| 2, 3, 4, 5 | X17 | Spare — no firmware function (bit 4, In5, was the brake endstop until the iBooster) |

The gear code SHALL be formed from the raw levels of bits 0, 1 and 6 in the order `[bit0 bit1 bit6]`,
i.e. `(bit0 << 2) | (bit1 << 1) | bit6`. No polarity inversion SHALL be applied to the gear code bits.

#### Scenario: Initialize the input expander
- **WHEN** `BoardInputs::begin()` is called with the `Wire1` bus
- **THEN** all eight PCA9557 pins SHALL be configured as inputs
- **AND** a first input-port read SHALL be performed to seed the snapshot
- **AND** `true` SHALL be returned only if the device answers at `0x1A`
- **AND** `false` SHALL be returned with an error logged if it does not

#### Scenario: Consumers read a cached snapshot, never the bus
- **WHEN** `TransmissionController::getPhysicalGear()` needs an input state
- **THEN** it SHALL read the published snapshot from `BoardInputs`
- **AND** it SHALL NOT issue an I2C transaction of its own
- **AND** no `digitalRead()` of a gear pin SHALL remain in the firmware

#### Scenario: Snapshot exposes gear and brake state with validity
- **WHEN** `getSnapshot()` is called
- **THEN** it SHALL return the undecoded 3-bit gear code (0..7), a `valid` flag, and the age in
  milliseconds of the last successful read
- **AND** it SHALL NOT carry a brake state any more (the brake-released field is removed with the
  endstop)
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
- **WHEN** the gear read cache (`TRANS_GEAR_READ_INTERVAL_MS`, 100 ms) consumes the snapshot
- **THEN** the 25 ms poll interval SHALL provide at least one fresh sample per consumer window
- **AND** the gear read's retry-on-ambiguous-result behavior SHALL be satisfied by consuming
  successive snapshots rather than issuing back-to-back I2C reads
