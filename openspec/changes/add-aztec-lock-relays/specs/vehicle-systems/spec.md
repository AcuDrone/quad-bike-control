## ADDED Requirements
### Requirement: Aztec Lock on Relay_4 + Relay_5 + Relay_6
The aztec buggy SHALL drive its lock ("блокування") through the existing wheel-lock function with
Relay_4, Relay_5 and Relay_6 switched together as one relay-number set
(`RELAY_SET_WHEEL_LOCK = RELAY_N(4) | RELAY_N(5) | RELAY_N(6)`, `RELAY_WHEEL_LOCK_FITTED = 1`;
PCA9557 mask `0x8C` = IO7|IO3|IO2 on relay-board connectors X7/X8/X9, XL9535 mask `0x38` =
P0.3..P0.5). The lock SHALL be commanded from the web (`set_wheel_lock`) and from MAVLink servo
channel 7, with no input-source gate and no ignition interlock (the quad bike's lock has neither).
`setWheelLock()` SHALL keep a "not fitted" branch (reject a LOCK, log once per boot, report released)
selected only when `RELAY_WHEEL_LOCK_FITTED` is `0`.

#### Scenario: Lock from the web
- **WHEN** `VehicleController::processWheelLockCommand(true)` runs for a web `set_wheel_lock` command
- **THEN** `RelayController::setWheelLock(true)` SHALL energize Relay_4, Relay_5 and Relay_6 in one
  verified whole-port write and return `true`
- **AND** the web response SHALL be success ("Lock LOCKED")
- **AND** `getWheelLock()` SHALL return `true`

#### Scenario: Unlock
- **WHEN** `setWheelLock(false)` is called while the lock is engaged
- **THEN** all three lock relays SHALL be released in one verified write and `getWheelLock()` SHALL
  return `false`
- **AND** a `setWheelLock(false)` while already released SHALL return `true` without a port write
  and without logging

#### Scenario: Lock refused while the relay board is unavailable
- **WHEN** a LOCK is requested while the relay expander failed to initialize or is faulted
- **THEN** `setWheelLock()` SHALL return `false`, log the refusal and leave the lock state unchanged
- **AND** the web response SHALL be an error ("Wheel lock command rejected")

#### Scenario: Lock independent of the ignition
- **WHEN** a LOCK or UNLOCK command arrives with the ignition in any state, including `CRANKING`
- **THEN** it SHALL be applied without changing the ignition, starter or light relays

#### Scenario: Fail-safe all-off keeps the lock
- **WHEN** `RelayController::allOff()` runs
- **THEN** ignition, starter and light SHALL be switched off
- **AND** the lock SHALL hold its last commanded state

#### Scenario: Fitted state is queryable
- **WHEN** `VehicleController::isWheelLockFitted()` or `RelayController::isWheelLockFitted()` is called
- **THEN** it SHALL return `true`

## REMOVED Requirements
### Requirement: Front-Wheel Lock Not Fitted
**Reason**: Relay_4, Relay_5 and Relay_6 now carry the aztec buggy's lock; the wheel-lock function is
fitted (`RELAY_WHEEL_LOCK_FITTED = 1`). Superseded by "Aztec Lock on Relay_4 + Relay_5 + Relay_6".
**Migration**: None for callers — the API is unchanged. The MAVLink autopilot's servo channel 7 must
sit at or below 1520 µs unless the lock is wanted, because a high channel 7 now really locks.
