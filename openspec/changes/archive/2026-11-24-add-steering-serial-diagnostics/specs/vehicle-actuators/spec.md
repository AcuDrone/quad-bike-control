## ADDED Requirements
### Requirement: Steering Diagnostics Ownership Lock
The steering controller SHALL provide a diagnostics lock for the USB steering diagnostics console. While the lock is set, `setSteeringPercent()` (MAVLink, web slider, failsafe centre, boot centring), `setRateCommand()`, `jog()`, `nudge()`, the limit captures and the sensor zero-set SHALL be refused, and only the diagnostics console SHALL drive the motor: open loop through `diagDrive()` (no duty floor, no stall latch, no refresh timeout) or one closed-loop move through `diagGoto()`, which SHALL use the same target clamping, gates and position law as `setSteeringPercent()`. Stop paths and the sensor / VESC monitors SHALL NOT be gated. Setting or clearing the lock SHALL stop the motor (coast) and cancel any move, jog or pending boot centring.

#### Scenario: External steering refused while locked
- **WHEN** the diagnostics lock is set
- **AND** a MAVLink steering command, a web steering / jog / nudge / capture / zero command or the failsafe centre arrives
- **THEN** the controller SHALL NOT move the motor for it
- **AND** a web command SHALL be answered with a failure naming the USB diagnostics console

#### Scenario: Lock changes stop the motor
- **WHEN** the lock is set or cleared
- **THEN** the motor SHALL be coasted and the closed loop and any jog SHALL be idle

#### Scenario: Monitors still stop the motor
- **WHEN** the lock is set and the VESC link drops or reports a fault
- **THEN** the controller SHALL stop the motor as without the lock

#### Scenario: Normal behaviour without the lock
- **WHEN** the lock is not set
- **THEN** every steering command path SHALL behave as before this change

### Requirement: VESC Telemetry Poll Period
The VESC steering driver SHALL poll COMM_GET_VALUES every `STEER_VESC_TELEM_MS` (300 ms) and SHALL allow the period to be changed at run time; the USB diagnostics console SHALL set it to `STEER_DIAG_VESC_TELEM_MS` (50 ms) while it owns the steering and restore it afterwards. The driver SHALL report the last commanded effort.

#### Scenario: Fast telemetry in diagnostics
- **WHEN** `diag on` is accepted
- **THEN** GET_VALUES SHALL be requested every 50 ms until `diag off`, then every 300 ms again
