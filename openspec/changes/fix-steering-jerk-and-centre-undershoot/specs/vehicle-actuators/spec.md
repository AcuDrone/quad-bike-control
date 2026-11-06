## MODIFIED Requirements
### Requirement: Steering Break-away and Tracking Duty Floors
While driving, the closed-loop steering duty SHALL be `clamp(max(|error| × STEER_KP, floor) + dutyBoost_, floor, STEER_PWM_MAX_DUTY)`, signed toward the target, so that the anti-stall boost always adds on top of the floor in force and is never hidden under it. The floor SHALL be the break-away floor `STEER_PWM_MIN_DUTY` (85 on the quad) from a start from rest or a drive direction flip until the wheel has moved at least `STEER_BOOST_MIN_PROGRESS` (0.3°) from the position where that drive or reversal started, and the tracking floor `STEER_PWM_TRACK_MIN_DUTY` (85 on the quad, bench-tune) after that; `STEER_PWM_MIN_DUTY` SHALL NOT be below `STEER_PWM_TRACK_MIN_DUTY`. Every `STEER_BOOST_CHECK_MS` (100 ms) the boost SHALL rise by `STEER_BOOST_STEP` (30) when the wheel progressed less than `STEER_BOOST_MIN_PROGRESS` since the previous check, and, when it progressed at least that much, SHALL be held unchanged while `|error|` exceeds the hold band `STEER_BOOST_HOLD_ERR` (2.0°) and SHALL fall by `STEER_BOOST_DECAY` (30 on the quad), never below 0, inside the hold band. The boost SHALL be reset to 0 on a start from rest and on a drive direction flip. The duty law SHALL be a pure, host-tested function (`include/SteerPositionLaw.h`). The sensorless rate drive SHALL keep `STEER_PWM_MIN_DUTY` as its floor.

#### Scenario: Break-away floor from rest
- **WHEN** the closed loop starts driving from idle or from parked
- **THEN** the duty SHALL be at least `STEER_PWM_MIN_DUTY` until the position differs from the start position by at least `STEER_BOOST_MIN_PROGRESS`

#### Scenario: Tracking floor after first progress
- **WHEN** the closed loop is driving and the wheel has moved at least `STEER_BOOST_MIN_PROGRESS` since the drive started or last reversed
- **THEN** the duty floor SHALL be `STEER_PWM_TRACK_MIN_DUTY`
- **AND** a target update SHALL NOT switch the floor back to `STEER_PWM_MIN_DUTY`

#### Scenario: Reversal re-arms break-away
- **WHEN** the drive direction (sign of the error) flips while driving
- **THEN** the floor SHALL return to `STEER_PWM_MIN_DUTY` until the wheel has moved at least `STEER_BOOST_MIN_PROGRESS` from the reversal position
- **AND** the boost SHALL be reset to 0

#### Scenario: Stalled wheel gets more duty on the first check
- **WHEN** the closed loop is driving with `|error| × STEER_KP` below the floor in force and the boost at 0
- **AND** a boost check finds less than `STEER_BOOST_MIN_PROGRESS` of progress
- **THEN** the boost SHALL become `STEER_BOOST_STEP` (30)
- **AND** the duty SHALL be the floor plus 30 (115 at a floor of 85), not the floor

#### Scenario: Boost decays gently
- **WHEN** the boost is 30, `|error|` is within `STEER_BOOST_HOLD_ERR` (2.0°) and the following boost checks each find at least `STEER_BOOST_MIN_PROGRESS` of progress
- **THEN** each check SHALL lower the boost by `STEER_BOOST_DECAY`, never below 0 (30 → 0 after one check at the quad's 30; 30 → 20 → 10 → 0 at a decay of 10)
- **AND** it SHALL stay at 0 on further moving checks
- **AND** a check that finds the wheel stalled again SHALL add `STEER_BOOST_STEP` to the current boost

#### Scenario: Boost is held while far from the target
- **WHEN** the closed loop is driving with a non-zero boost and `|error|` greater than `STEER_BOOST_HOLD_ERR` (2.0°)
- **AND** a boost check finds at least `STEER_BOOST_MIN_PROGRESS` of progress
- **THEN** the boost SHALL stay unchanged (no decay)
- **AND** once `|error|` is within `STEER_BOOST_HOLD_ERR` the boost SHALL fall by `STEER_BOOST_DECAY` per moving check
- **AND** a check that finds the wheel stalled SHALL add `STEER_BOOST_STEP` both outside and inside the hold band

#### Scenario: Far from the target the P term rules
- **WHEN** the closed loop is driving and `|error| × STEER_KP` exceeds the floor in force
- **THEN** the duty SHALL be `|error| × STEER_KP + dutyBoost_`, clamped to `STEER_PWM_MAX_DUTY`

#### Scenario: Stall at the tracking floor
- **WHEN** the closed loop is driving at the tracking floor and the wheel stops making progress
- **THEN** the anti-stall boost SHALL raise the duty above the floor by `STEER_BOOST_STEP` per `STEER_BOOST_CHECK_MS`
- **AND** if the position still changes by less than `STEER_STALL_THRESHOLD` over `STEER_STALL_TIMEOUT`, a stall-stop SHALL occur

### Requirement: Steering Stop Hysteresis
The closed-loop steering controller SHALL have three states: idle, driving and parked. It SHALL stop driving and park when the absolute position error is at most `STEER_POSITION_TOLERANCE` (2 = 0.2° on the quad) and SHALL NOT drive the closed loop again until the absolute error exceeds `STEER_RESTART_TOLERANCE` (10 = 1.0°). Parking SHALL be a coast through `IMotorDriver::stop()` (zero duty); the self-locking steering worm gear holds the shaft, and the controller SHALL send no further closed-loop motor commands while parked; the only exception SHALL be the bounded park correction, whose trigger SHALL be the larger of `STEER_PARK_TRIGGER_TOL` (4 = 0.4°) and its done tolerance, independent of `STEER_POSITION_TOLERANCE`. Every stop path SHALL be a coast.

#### Scenario: Park at the target
- **WHEN** the closed loop is driving and the absolute error becomes at most `STEER_POSITION_TOLERANCE`
- **THEN** the controller SHALL call `IMotorDriver::stop()` (coast) and enter the parked state
- **AND** it SHALL NOT send closed-loop motor commands while it stays parked

#### Scenario: Parks within 0.2°
- **WHEN** the closed loop is driving toward a target from either side
- **AND** the absolute error becomes 0.2°
- **THEN** the controller SHALL park
- **WHEN** the absolute error is 0.3°
- **THEN** the controller SHALL keep driving toward the target

#### Scenario: No restart inside the hysteresis band
- **WHEN** the controller is parked
- **AND** the absolute error grows above `STEER_POSITION_TOLERANCE` but stays at most `STEER_RESTART_TOLERANCE` (1.0°)
- **THEN** the controller SHALL stay parked and the closed loop SHALL NOT issue a drive command

#### Scenario: Restart beyond the hysteresis band
- **WHEN** the controller is parked and the absolute error exceeds `STEER_RESTART_TOLERANCE` (1.0°)
- **THEN** it SHALL start driving toward the current target as a start from rest

#### Scenario: Park correction trigger unchanged
- **WHEN** the controller is parked with an absolute error of 0.3°
- **AND** the park correction is enabled with its default done tolerance (0.2°)
- **THEN** the park correction SHALL NOT start, because 0.3° does not exceed `STEER_PARK_TRIGGER_TOL` (0.4°)
- **AND** an error above 0.4° SHALL start it as before

#### Scenario: Command from idle
- **WHEN** a percent command is accepted while the closed loop is idle
- **THEN** the controller SHALL start driving if the absolute error exceeds `STEER_POSITION_TOLERANCE`
- **AND** otherwise it SHALL enter the parked state without driving

#### Scenario: Leaving parked to idle
- **WHEN** the controller is parked or driving
- **AND** the steering VESC reports a fault or its link is lost, or the angle sensor becomes unhealthy, or a web jog, nudge or rate drive takes the motor, or a limit capture or sensor zero-set starts, or the steering stops being calibrated, or `stop()` is called (failsafe, emergency stop)
- **THEN** the controller SHALL command `IMotorDriver::stop()` (coast) and enter the idle state
- **AND** a stall-stop, a move timeout or a range-guard stop SHALL also end in a coast and the idle state

#### Scenario: Web slider path unchanged
- **WHEN** the web `set_steering` command sets a target
- **THEN** it SHALL reach `setSteeringPercent()` through the same path and debounce as before
- **AND** it SHALL be driven and parked by the same rules as a MAVLink target, so a target within `STEER_RESTART_TOLERANCE` of a parked wheel SHALL NOT drive the closed loop
