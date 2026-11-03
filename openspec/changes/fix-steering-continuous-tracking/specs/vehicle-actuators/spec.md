## ADDED Requirements
### Requirement: Steering Continuous Target Tracking and Stall Latch
The steering controller SHALL track a continuously changing target: while a closed-loop move is driving or parked, every accepted `setSteeringPercent()` call SHALL update the target, whatever the difference from the previous target, and SHALL NOT restart the move timer, the stall-check window, the progress baseline, the anti-stall boost or the break-away state. These SHALL be reset only when a drive starts from rest, so that stall detection and the move timeout function under the continuous MAVLink command stream. The controller SHALL latch out further motion in a stalled direction for a cooldown of `STEER_STALL_COOLDOWN_MS` (700 ms). Target and tolerance values SHALL be in 0.1° of steering angle. The sensorless rate drive is not governed by this requirement.

#### Scenario: Target update while tracking
- **WHEN** `setSteeringPercent()` is accepted while the closed loop is driving or parked
- **THEN** the target SHALL be replaced by the new target, for any difference including less than `STEER_POSITION_TOLERANCE`
- **AND** `moveStartTime_`, the stall-check position and time, the progress baseline, `dutyBoost_` and the break-away state SHALL NOT be reset
- **AND** the stall latch SHALL NOT be evaluated for the update

#### Scenario: Timers reset only on a start from rest
- **WHEN** the closed loop starts driving from rest (from idle on an accepted command, or from parked because the error exceeded `STEER_RESTART_TOLERANCE`)
- **THEN** the move timer, the stall-check window and the progress baseline SHALL restart at the current position and time
- **AND** `dutyBoost_` SHALL be reset to 0

#### Scenario: Stall detection under a moving target
- **WHEN** the closed loop is driving and the position changes by less than `STEER_STALL_THRESHOLD` over `STEER_STALL_TIMEOUT`
- **AND** target updates keep arriving with the error in the same direction
- **THEN** a stall-stop SHALL occur after `STEER_STALL_TIMEOUT`, as for a single command, whichever duty floor is active
- **AND** the stall-check window SHALL restart, and `dutyBoost_` SHALL be reset to 0, only when the drive direction (sign of the error) flips while driving
- **AND** the stall check SHALL NOT run while the loop is parked

#### Scenario: Move timeout measures time behind the target
- **WHEN** the closed loop is driving
- **THEN** the move timer SHALL be re-armed on every iteration where the absolute error is at most `STEER_RESTART_TOLERANCE`
- **AND** the move SHALL be stopped (coast, no stall latch) once the error has stayed above `STEER_RESTART_TOLERANCE` for `STEER_MOVE_TIMEOUT` without re-arming
- **AND** a continuously moving target that the wheel follows within `STEER_RESTART_TOLERANCE` SHALL NOT time out

#### Scenario: Stall latches out the stalled direction
- **WHEN** a stall-stop occurs (position loop stall with a healthy sensor, or a nonzero VESC fault code)
- **THEN** the controller SHALL record the stalled direction and latch time and stop the motor (coast)
- **AND** a subsequent move, jog or nudge that would push further in the stalled direction SHALL be refused while less than `STEER_STALL_COOLDOWN_MS` (700 ms) has elapsed

#### Scenario: Opposite-direction escape and post-cooldown retry
- **WHEN** the controller is stall-latched
- **AND** a new move commands the opposite direction (away from or across the jam)
- **THEN** the move SHALL be accepted immediately and the latch cleared
- **WHEN** the controller is stall-latched and a same-direction move is commanded after `STEER_STALL_COOLDOWN_MS` (700 ms) has elapsed
- **THEN** the move SHALL be accepted and the latch cleared

### Requirement: Steering Stop Hysteresis
The closed-loop steering controller SHALL have three states: idle, driving and parked. It SHALL stop driving and park when the absolute position error is at most `STEER_POSITION_TOLERANCE` (10 = 1.0°) and SHALL NOT drive again until the absolute error exceeds `STEER_RESTART_TOLERANCE` (30 = 3.0°, bench-tune). Parking SHALL be a coast through `IMotorDriver::stop()` (zero duty); the self-locking steering worm gear holds the shaft, and the controller SHALL send no further motor commands while parked. Every stop path SHALL be a coast.

#### Scenario: Park at the target
- **WHEN** the closed loop is driving and the absolute error becomes at most `STEER_POSITION_TOLERANCE`
- **THEN** the controller SHALL call `IMotorDriver::stop()` (coast) and enter the parked state
- **AND** it SHALL NOT send motor commands while it stays parked

#### Scenario: No restart inside the hysteresis band
- **WHEN** the controller is parked
- **AND** the absolute error grows above `STEER_POSITION_TOLERANCE` but stays at most `STEER_RESTART_TOLERANCE`
- **THEN** the controller SHALL stay parked and SHALL NOT issue a drive command

#### Scenario: Restart beyond the hysteresis band
- **WHEN** the controller is parked and the absolute error exceeds `STEER_RESTART_TOLERANCE`
- **THEN** it SHALL start driving toward the current target as a start from rest

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
- **AND** it SHALL be driven and parked by the same rules as a MAVLink target, so a target within `STEER_RESTART_TOLERANCE` of a parked wheel SHALL NOT drive

### Requirement: Steering Break-away and Tracking Duty Floors
While driving, the closed-loop steering duty SHALL be `clamp(|error| × STEER_KP + dutyBoost_, floor, STEER_PWM_MAX_DUTY)`, signed toward the target. The floor SHALL be the break-away floor `STEER_PWM_MIN_DUTY` (140) from a start from rest or a drive direction flip until the wheel has moved at least `STEER_BOOST_MIN_PROGRESS` (0.3°) from the position where that drive or reversal started, and the tracking floor `STEER_PWM_TRACK_MIN_DUTY` (100, bench-tune) after that. The anti-stall boost SHALL apply on top of either floor. The sensorless rate drive SHALL keep `STEER_PWM_MIN_DUTY` as its floor.

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

#### Scenario: Stall at the tracking floor
- **WHEN** the closed loop is driving at the tracking floor and the wheel stops making progress
- **THEN** the anti-stall boost SHALL raise the duty by `STEER_BOOST_STEP` per `STEER_BOOST_CHECK_MS` as today
- **AND** if the position still changes by less than `STEER_STALL_THRESHOLD` over `STEER_STALL_TIMEOUT`, a stall-stop SHALL occur

## REMOVED Requirements
### Requirement: Steering Re-command Guard and Stall Latch
**Reason**: The 5° re-command no-op guard (`STEER_RETARGET_TOLERANCE`) turned a smoothly moving autopilot stick into 5° steps, and every accepted step restarted the move with a break-away kick. Continuous tracking replaces it: a target update never restarts the timers, so the guard has nothing left to protect.
**Migration**: Replaced by "Steering Continuous Target Tracking and Stall Latch", which keeps the stall latch, the cooldown and the escape scenarios unchanged and defines when the move and stall timers reset. `STEER_RETARGET_TOLERANCE` is removed from `include/Constants.h`.
