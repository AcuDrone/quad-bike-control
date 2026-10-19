## MODIFIED Requirements
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
