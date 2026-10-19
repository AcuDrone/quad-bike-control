## MODIFIED Requirements
### Requirement: Front Light Control Integration
The system SHALL integrate front light control with vehicle systems.

#### Scenario: Set front light state via vehicle controller
- **WHEN** VehicleController.setFrontLight() is called with on/off boolean
- **THEN** light state is passed to RelayController.setFrontLight()
- **AND** light state change is logged
- **AND** vehicle state telemetry includes light state

#### Scenario: Query current light state
- **WHEN** VehicleController.getFrontLight() is called
- **THEN** current light state is returned from RelayController (true/false)
- **AND** state reflects the actual `RELAY_MASK_FRONT_LIGHT` output (Relay_3, IO6, on the aztec buggy)

## ADDED Requirements
### Requirement: Front-Wheel Lock Not Fitted
The aztec buggy SHALL have no relay behind the front-wheel lock function (`RELAY_WHEEL_LOCK_FITTED =
0`). The wheel-lock plumbing (MAVLink servo channel 7, web `set_wheel_lock` command, telemetry fields)
SHALL be kept so a later change can replace it with the buggy's three locks, but a lock request SHALL
never be reported as applied.

#### Scenario: Lock request is rejected
- **WHEN** `VehicleController::setWheelLock(true)` is called from the MAVLink or the web path
- **THEN** `RelayController::setWheelLock()` SHALL return `false` without writing the relay port
- **AND** the first rejection after boot SHALL be logged; later ones SHALL NOT flood the log
- **AND** `getWheelLock()` SHALL return `false`

#### Scenario: Release request is a no-op
- **WHEN** `setWheelLock(false)` is called (every control loop while MAVLink channel 7 is low)
- **THEN** it SHALL return `true` without writing the relay port and without logging

#### Scenario: Fitted state is queryable
- **WHEN** `VehicleController::isWheelLockFitted()` or `RelayController::isWheelLockFitted()` is called
- **THEN** it SHALL return `false`
