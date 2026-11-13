## ADDED Requirements
### Requirement: Steering Park Correction
While the closed-loop steering is PARKED in position mode, the steering controller SHALL finish a parking that ended outside the park window with a bounded series of short open-loop pulses toward the current target. The correction SHALL be the only motor command a parked loop sends, and SHALL be limited to the configured maximum number of pulses per parking. It SHALL start only when the correction is enabled, the USB diagnostics lock is not set, the loop has been parked and the target has stayed within `STEER_PARK_TARGET_STABLE` (3 = 0.3°) of its reference for at least `STEER_PARK_SETTLE_MS` (1000 ms), and the absolute error exceeds the larger of `STEER_POSITION_TOLERANCE` (0.4°) and the done tolerance. Positions and tolerances SHALL be in 0.1° units. The decision logic SHALL be a pure, host-tested law (`include/SteerParkLaw.h`). With the correction disabled the steering SHALL behave exactly as without this requirement.

#### Scenario: Correction starts after the settle time
- **WHEN** the loop is PARKED with the target unchanged (within 0.3°) and parked for at least 1000 ms
- **AND** the absolute error exceeds the larger of 0.4° and the done tolerance
- **THEN** the controller SHALL send one pulse at rung 1 (duty × ms) toward the target with `IMotorDriver::setSpeed()`, without the `STEER_PWM_MIN_DUTY` floor, and SHALL coast with `IMotorDriver::stop()` after the rung's length
- **AND** it SHALL work relative to the current target, whatever its value, including the exact 0° that a centred RC stick produces

#### Scenario: No correction before settling or inside the window
- **WHEN** the target moved by more than 0.3° or the loop entered PARKED less than 1000 ms ago, or the absolute error is within the trigger
- **THEN** no pulse SHALL be sent

#### Scenario: Pulse evaluation and ladder
- **WHEN** a pulse has ended and `STEER_PARK_PULSE_GAP_MS` (500 ms) has passed since its end
- **THEN** the controller SHALL compare the position with the position before the pulse and, in this order:
- **AND** if the absolute error is at most the done tolerance, it SHALL finish (done)
- **AND** else if the error changed sign (overshoot beyond the done tolerance), it SHALL log the overshoot and send the next pulse at rung 1 in the new direction of the error
- **AND** else if the wheel moved away from the target by at least `STEER_PARK_MOVE_MIN` (0.1°), it SHALL log it and send the next pulse at rung 1 toward the target
- **AND** else if the wheel moved toward the target by at least 0.1°, the next pulse SHALL use the same rung
- **AND** otherwise the next pulse SHALL use the next rung, staying at rung 3 once there
- **AND** every pulse, including one after an overshoot, SHALL count toward the maximum
- **AND** once started, the correction SHALL continue until the done tolerance, not the 0.4° trigger

#### Scenario: Give up after the maximum pulses
- **WHEN** the maximum number of pulses has been sent and the evaluation after the last one is not done
- **THEN** the controller SHALL stop correcting and log `park correction gave up at X°`
- **AND** it SHALL NOT pulse again until the settle clock is reset by a target move beyond 0.3° or a new entry into PARKED (a main-loop restart beyond `STEER_RESTART_TOLERANCE` or a command from idle)

#### Scenario: Budget per parking
- **WHEN** a correction finished done and the error later exceeds the trigger again without a target move or a new park entry
- **THEN** the controller SHALL resume from rung 1 with only the remaining pulses of the same budget

#### Scenario: Cancel at once
- **WHEN** a correction is armed or running
- **AND** the target moves by more than 0.3°, or the loop leaves PARKED for any reason (main-loop restart beyond `STEER_RESTART_TOLERANCE`, `stop()`, sensor fault, VESC fault or link loss, stall-stop), or a jog, nudge, rate drive, limit capture or sensor zero-set starts, or the USB diagnostics lock is set, or the correction is disabled or a park setting is changed
- **THEN** a running pulse SHALL be ended with a coast at once and the correction state SHALL reset
- **AND** the normal position loop SHALL keep priority: an error beyond `STEER_RESTART_TOLERANCE` restarts the drive as a start from rest

#### Scenario: Stall latch respected
- **WHEN** a pulse would push in a direction blocked by the stall latch within `STEER_STALL_COOLDOWN_MS`
- **THEN** the pulse SHALL NOT be sent and SHALL NOT count, and the correction SHALL wait for the cooldown

#### Scenario: Debug log lines
- **WHEN** a pulse is evaluated, an overshoot or wrong-way move is seen, or the correction finishes or gives up
- **THEN** the SERVO debug feature SHALL print lines of the form `[STEER] park nudge 85x90 -> moved 0.15 deg (err 0.35)`, `[STEER] park overshoot -> rung 1, reversing (err -0.30)`, `[STEER] park nudge moved the wrong way -> rung 1 (err 0.70)`, `[STEER] park correction done at 0.10 deg (3 pulses)` and `[STEER] park correction gave up at 0.60 deg (5 pulses)`

#### Scenario: Closed-loop state unchanged
- **WHEN** a correction runs
- **THEN** the closed-loop state SHALL stay PARKED (token `park`), and no jog SHALL be active

### Requirement: Steering Park Correction Settings
The park correction SHALL be configured per vehicle by persisted settings in the existing NVS namespace `"steering"`, owned by the steering controller: enabled (`park_en`, default true), three ladder rungs of duty (`park_d1`..`park_d3`, 40–150, defaults 70 / 85 / 100) and length (`park_t1`..`park_t3`, 40–200 ms, defaults 80 / 90 / 100), the done tolerance (`park_tol`, 1–5 in 0.1°, default 2 = 0.2°) and the maximum pulses per parking (`park_max`, 1–10, default 5). The ladder SHALL be non-decreasing in both duty and length. The settle time (1000 ms), the target-stable band (0.3°), the pulse gap (500 ms) and the move threshold (0.1°) SHALL be compile-time constants only. The namespace SHALL never be cleared; the calibration migration and the sensor zero-set SHALL NOT remove the park keys.

#### Scenario: Defaults on a unit without the keys
- **WHEN** the firmware boots and a park key is absent
- **THEN** that setting SHALL use its default, and NVS SHALL NOT be written

#### Scenario: Invalid stored value
- **WHEN** a stored park value is out of range, or the stored ladder is not non-decreasing
- **THEN** the value (for the ladder: the whole ladder) SHALL be logged and replaced by the default in RAM, and NVS SHALL NOT be rewritten

#### Scenario: Change persisted first
- **WHEN** a valid setting change is requested
- **THEN** it SHALL be written to NVS first and applied only after the write succeeds, and any running correction SHALL be cancelled
- **AND** on a write failure the RAM value SHALL stay unchanged

#### Scenario: Decreasing rung refused
- **WHEN** a rung change would make its duty or length lower than the previous rung's or higher than the next rung's
- **THEN** it SHALL be refused and nothing SHALL be stored
