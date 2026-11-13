## MODIFIED Requirements
### Requirement: Quad Bike Relay Assignment With Fitted Front-Wheel Lock
On the quad bike (`master`) build the relay board SHALL use the quad assignment, defined in
`Constants.h` as relay-number sets and mapped at `begin()` through the detected board's Relay_N → bit
table (masks shown for the PCA9557 / XL9535 board):
- ignition/ECU line = Relay_1 + Relay_2, paralleled (`RELAY_SET_IGNITION`; mask `0b00110000` /
  `0b00000011`);
- starter = Relay_3 (`0b01000000` / `0b00000100`);
- front light = Relay_4 (`0b10000000` / `0b00001000`);
- front-wheel lock = Relay_5 + Relay_6, paralleled (`RELAY_SET_WHEEL_LOCK`; `0b00001100` /
  `0b00110000`);
- Relay_7 / Relay_8 spare (`RELAY_SET_RESERVED`; `0b00000011` / `0b11000000`).

The front-wheel lock SHALL be fitted (`RELAY_WHEEL_LOCK_FITTED` = 1). It replaces, for this build
only, the aztec buggy's "Front-Wheel Lock Not Fitted" behaviour.

#### Scenario: Wheel lock from MAVLink channel 7
- **WHEN** MAVLink servo channel 7 is above 1520 µs (e.g. 2000)
- **THEN** Relay_5 and Relay_6 SHALL both be energized, `getWheelLock()` SHALL return `true`,
  telemetry `wheel_lock_on` SHALL be `true` and `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL be set
- **AND WHEN** channel 7 returns to 1000 µs
- **THEN** both relays SHALL be released and the lock SHALL report unlocked

#### Scenario: Same relays on either board
- **WHEN** the quad build runs with the PCA9557 board or with the XL9535 board
- **THEN** the same relay numbers SHALL carry the same functions, and only the port masks SHALL differ

#### Scenario: Wheel lock reported as fitted
- **WHEN** telemetry is formatted on the quad build
- **THEN** `wheel_lock_fitted` SHALL be `true` and the web wheel-lock toggle SHALL be usable

#### Scenario: Fail-safe all-off keeps the wheel lock
- **WHEN** `RelayController::allOff()` runs
- **THEN** ignition, starter and light SHALL be switched off
- **AND** the wheel lock SHALL hold its last commanded state

#### Scenario: Relay test respects the wheel lock
- **WHEN** a web relay test asks to energize any relay while the wheel lock is engaged
- **THEN** the request SHALL be refused with a reason naming the wheel lock
- **AND WHEN** a lock command arrives while test relays are on
- **THEN** all test relays SHALL be switched off before the lock is applied
- **AND** the 2 s starter test cap SHALL apply to Relay_3
