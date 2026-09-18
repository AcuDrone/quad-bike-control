## REMOVED Requirements

### Requirement: VESC Telemetry Fault Monitoring and Communication Failsafe
**Reason**: This requirement bundled a firmware-level sustained-over-current backstop together with the VESC fault-code stop and the communication failsafe. The over-current half is being deleted: its 400 ms window is compared against motor-current samples that refresh only every `STEER_VESC_TELEM_MS` (300 ms) and are never zeroed on `stop()`, so a trip re-latches against the escape direction and a single lost `COMM_GET_VALUES` reply trips an idle motor. The VESC already enforces *Motor Current Max*, *Absolute Max Current* and MOSFET temperature limits, and the AS5600 position stall detector (`STEER_STALL_TIMEOUT`) already covers mechanical jams.

**Migration**: Replaced by `### Requirement: VESC Fault Monitoring and Communication Failsafe`, which keeps telemetry polling, the fault-code stop, the comm failsafe and `steer_motor_current` telemetry unchanged, and explicitly places over-current protection in the VESC and jam detection in the position stall detector. No configuration or API surface changes; `STEER_VESC_OVERCURRENT_A` and `STEER_VESC_OVERCURRENT_MS` are deleted.

#### Scenario: Sustained over-current triggers stall-stop
- **WHEN** decoded motor current exceeds `STEER_VESC_OVERCURRENT_A` for at least `STEER_VESC_OVERCURRENT_MS`
- **THEN** the controller SHALL invoke the stall-stop path (stop + stall latch in the current drive direction)

## ADDED Requirements

### Requirement: VESC Fault Monitoring and Communication Failsafe
The steering controller SHALL poll VESC telemetry to enforce a firmware-level fault backstop, and SHALL fail safe when the VESC is unresponsive. Over-current protection SHALL be left to the VESC itself (its *Motor Current Max* clamp, its *Absolute Max Current* fault, and its MOSFET temperature limiting), and mechanical jams SHALL be left to the AS5600 position stall detector; the firmware SHALL NOT run its own sustained-over-current timer.

#### Scenario: Poll VESC telemetry periodically
- **WHEN** the steering driver is active
- **THEN** it SHALL request `COMM_GET_VALUES` at approximately `STEER_VESC_TELEM_MS` intervals (~2-5 Hz)
- **AND** decode motor current, FET temperature, input voltage, and fault code

#### Scenario: Motor current is reported but not acted on
- **WHEN** decoded motor current is high, or a `COMM_GET_VALUES` reply is lost and the last decoded sample is stale
- **THEN** the controller SHALL NOT trip a stall-stop on current magnitude
- **AND** the decoded value SHALL still be published as `steer_motor_current` telemetry

#### Scenario: VESC fault code stops the motor
- **WHEN** `COMM_GET_VALUES` reports a nonzero VESC fault code
- **THEN** the controller SHALL stop the motor and raise a fault flag in telemetry
- **AND** if a move or jog was in progress it SHALL take the stall-stop path (stop + stall latch in the current drive direction)

#### Scenario: Unresponsive VESC fails safe
- **WHEN** no valid `COMM_GET_VALUES` reply is received for `STEER_VESC_COMM_TIMEOUT_MS`
- **THEN** the driver-ok flag SHALL be set false
- **AND** the motor SHALL be stopped and steering commands (position and jog) SHALL be rejected, mirroring the sensor-fault path
- **AND** normal operation SHALL resume automatically once valid replies return
- **AND** a VESC that is silent at boot SHALL NOT block startup

## MODIFIED Requirements

### Requirement: Steering Re-command Guard and Stall Latch
The steering controller SHALL NOT restart its move and stall timers when re-commanded to the current target, and SHALL latch out further motion in a stalled direction for a cooldown of `STEER_STALL_COOLDOWN_MS` (700 ms), so that stall detection and move timeout function under the continuous MAVLink command stream.

#### Scenario: Re-command within tolerance is a no-op
- **WHEN** `setSteeringPercent()` is called while a move is in progress
- **AND** the new target is within `STEER_RETARGET_TOLERANCE` counts of the current target
- **THEN** `moveStartTime_`, the stall-check timers, and `dutyBoost_` SHALL NOT be reset
- **AND** the in-flight move SHALL continue so the stall window and move timeout can elapse

#### Scenario: Target change beyond tolerance resets the move
- **WHEN** `setSteeringPercent()` is called with a target differing from the current target by more than `STEER_RETARGET_TOLERANCE` counts
- **THEN** a fresh move SHALL start with the move/stall timers reset

#### Scenario: Stall latches out the stalled direction
- **WHEN** a stall-stop occurs (position loop stall or a nonzero VESC fault code)
- **THEN** the controller SHALL record the stalled direction and latch time and stop the motor
- **AND** a subsequent move that would push further in the stalled direction SHALL be refused while less than `STEER_STALL_COOLDOWN_MS` (700 ms) has elapsed

#### Scenario: Opposite-direction escape and post-cooldown retry
- **WHEN** the controller is stall-latched
- **AND** a new move commands the opposite direction (away from or across the jam)
- **THEN** the move SHALL be accepted immediately and the latch cleared
- **WHEN** the controller is stall-latched and a same-direction move is commanded after `STEER_STALL_COOLDOWN_MS` (700 ms) has elapsed
- **THEN** the move SHALL be accepted and the latch cleared
