## MODIFIED Requirements
### Requirement: Sensorless Manual Steering
Jog and nudge commands SHALL drive the steering motor whenever the steering motor driver link is healthy, regardless of the steering angle sensor state, so that the steering can be centred and the sensor zeroed without a working sensor. Percent position commands SHALL still require a healthy sensor and both limits. Without a healthy sensor, the autopilot steering channel SHALL reach the motor only through the rate drive (see "Steering Rate Drive").

#### Scenario: Jog without a sensor
- **WHEN** a jog or nudge command is received while the sensor is faulted, absent or invalid
- **AND** the VESC link is healthy and the direction is not stall-latched
- **THEN** the motor SHALL be driven open-loop at the jog or nudge duty
- **AND** it SHALL stop when the jog is not refreshed within `STEER_JOG_TIMEOUT_MS`, when the nudge pulse of `STEER_NUDGE_MS` ends, or on a VESC fault code
- **AND** no position stall detection SHALL apply, because there is no position

#### Scenario: Sensor fault during a jog
- **WHEN** the sensor becomes unhealthy while a jog or nudge is in progress
- **THEN** the jog or nudge SHALL continue open-loop under the same refresh and pulse bounds
- **AND** a closed-loop position move in progress SHALL be stopped instead

#### Scenario: Jog with a healthy sensor
- **WHEN** a jog or nudge command is received while the sensor is healthy
- **THEN** position stall detection and the stall latch SHALL apply as for closed-loop moves
- **AND** the range guard SHALL NOT apply, so the operator may jog past `STEER_LWS_SAFE_RANGE`

#### Scenario: Percent commands still gated
- **WHEN** a percent steering command arrives from MAVLink, the web slider or test buttons, boot centering or the failsafe
- **AND** the sensor is not healthy or either limit is missing
- **THEN** the command SHALL be rejected and the motor SHALL NOT be driven by it
- **AND** only the separate rate command of "Steering Rate Drive", issued by the vehicle layer in rate mode, MAY drive the motor from the autopilot steering channel

#### Scenario: Web jog has priority over the rate drive
- **WHEN** a web jog or nudge is active (refreshed within `STEER_JOG_TIMEOUT_MS`, or its pulse not yet ended)
- **THEN** rate commands SHALL neither stop nor override it
- **AND** rate commands SHALL take effect again once that jog or nudge has ended
- **AND** a web jog stop command SHALL stop whatever jog is active, including a rate drive

## ADDED Requirements
### Requirement: Steering Rate Drive
The steering controller SHALL provide a rate command, `setRateCommand(percent)`, that drives the motor open-loop with a duty derived from the stick deflection, for use only while the steering angle sensor is unhealthy. It SHALL be implemented as a continuous jog with variable duty, owned by the rate path, and SHALL reuse the jog refresh timeout. The deflection-to-duty mapping SHALL be a pure function without Arduino dependencies, covered by a host-side unit test.

#### Scenario: Deflection to duty mapping
- **WHEN** the mapping is evaluated for a percentage `pct` in −100..+100 with deadband `RC_STEERING_DEADBAND`, minimum duty `STEER_PWM_MIN_DUTY` and maximum duty `STEER_PWM_MAX_DUTY`
- **THEN** `|pct|` below the deadband SHALL give duty 0
- **AND** otherwise the duty magnitude SHALL be `round(min + (max − min) × d²)` with `d = (|pct| − deadband) / (100 − deadband)` clamped to at most 1, and `|pct|` clamped to 100
- **AND** the result SHALL carry the sign of `pct` (positive = wheel right = positive motor speed)
- **AND** with the default constants, 100 % SHALL give 255, 50 % SHALL give 168 and a value just outside the deadband SHALL give 140

#### Scenario: Rate command drives a jog
- **WHEN** `setRateCommand()` is called with a nonzero resulting duty
- **AND** the sensor is not healthy, the VESC link is healthy, the rate path is armed and no web jog or nudge is active
- **THEN** the motor SHALL be driven at that signed duty through the jog path, marked as owned by the rate path
- **AND** the jog refresh time SHALL be renewed, so that the drive stops after `STEER_JOG_TIMEOUT_MS` without a further call
- **AND** the range guard and the software limits SHALL NOT apply, because there is no position

#### Scenario: Zero duty stops only the rate drive
- **WHEN** `setRateCommand()` yields duty 0
- **THEN** an active rate-owned drive SHALL be stopped
- **AND** a web jog or nudge SHALL NOT be stopped by it

#### Scenario: Refused with a healthy sensor
- **WHEN** `setRateCommand()` is called while the sensor is healthy
- **THEN** it SHALL be refused and the motor SHALL NOT be driven by it, so that rate driving is impossible while a position is available

#### Scenario: Refused while the VESC is down or the direction is latched
- **WHEN** `setRateCommand()` is called while the VESC link is down
- **THEN** it SHALL be refused
- **WHEN** it is called in a direction that is stall-latched and less than `STEER_STALL_COOLDOWN_MS` has passed
- **THEN** it SHALL be refused; an opposite-direction command SHALL be accepted and clear the latch, as for jog

#### Scenario: Stall detection bypassed, VESC protections kept
- **WHEN** a rate-owned drive is running
- **THEN** position stall detection, the anti-stall boost, the move timeout and the position tolerance SHALL NOT be evaluated for it, and SHALL NOT trigger a stall-stop or latch
- **AND** a nonzero VESC fault code SHALL still stop it through the stall-stop path (stop + latch in the drive direction)
- **AND** the VESC communication failsafe SHALL still stop it
- **AND** the firmware SHALL NOT add any current-based stop; at the mechanical end stops the VESC's own current limit is the only protection

#### Scenario: Explicit stop of the rate drive
- **WHEN** the vehicle layer calls `stopRate()`
- **THEN** a rate-owned drive SHALL be stopped at once (duty 0)
- **AND** a web-owned jog or nudge and a closed-loop move SHALL NOT be affected

#### Scenario: Arming
- **WHEN** the vehicle layer disarms the rate path on entry into rate mode
- **THEN** nonzero rate commands SHALL be refused until a call yields duty 0, which SHALL arm the path
