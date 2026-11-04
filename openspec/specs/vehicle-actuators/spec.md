# vehicle-actuators Specification

## Purpose
TBD - created by archiving change add-vehicle-control-system. Update Purpose after archive.
## Requirements
### Requirement: PWM Servo Control
The system SHALL provide hardware PWM control for servo motors using ESP32 LEDC peripheral.

#### Scenario: Initialize servo with valid configuration
- **WHEN** ServoController.begin() is called with valid pin, channel, and pulse width range
- **THEN** LEDC channel is configured for 50Hz PWM output
- **AND** servo is positioned to default safe position (90 degrees)

#### Scenario: Set servo angle
- **WHEN** ServoController.setAngle() is called with angle between 0-180 degrees
- **THEN** pulse width is calculated and applied to LEDC channel
- **AND** servo moves to target angle within 100ms

#### Scenario: Set servo pulse width directly
- **WHEN** ServoController.setMicroseconds() is called with valid pulse width
- **THEN** LEDC duty cycle is set to generate requested pulse width
- **AND** pulse width is constrained to configured min/max range

#### Scenario: Disable servo output
- **WHEN** ServoController.disable() is called
- **THEN** PWM output is stopped
- **AND** servo holds last position or relaxes based on servo type

### Requirement: BTS7960 Motor Driver Control
The system SHALL use a BTS7960 motor driver for the brake actuator, driven through `BTS7960Controller` with LEDC PWM on RPWM/LPWM. The steering actuator SHALL NOT use `BTS7960Controller`; it is driven by a VESC over UART (see "VESC Brushed-DC Steering Driver"). Transmission control remains on a PWM servo.

#### Scenario: BTS7960 for brake actuator
- **WHEN** the brake actuator is commanded via `BTS7960Controller::setSpeed()`
- **THEN** RPWM/LPWM PWM outputs SHALL be set as specified in the existing brake scenarios
- **AND** RPWM and LPWM SHALL never be driven high simultaneously
- **AND** the outputs SHALL be `PIN_BRAKE_LPWM` (GPIO6) and `PIN_BRAKE_RPWM` (GPIO7)

#### Scenario: Steering no longer uses BTS7960
- **WHEN** the steering actuator is commanded
- **THEN** the command SHALL be issued through the `IMotorDriver` abstraction backed by the VESC UART driver
- **AND** no `BTS7960Controller` instance SHALL be associated with the steering actuator

#### Scenario: Former steering PWM pins on the DevKitC-1
- **WHEN** the pin map is read
- **THEN** GPIO17 SHALL be the hall speed sensor input (`PIN_SPEED_SENSOR`) and GPIO18 SHALL stay
  free/reserved
- **AND** LEDC channels 6 and 7 SHALL stay unallocated (see "Reserved and unused pins")

### Requirement: Actuator Safety Limits
The system SHALL enforce safety limits on all actuator operations.

#### Scenario: Validate servo angle input
- **WHEN** servo angle is set outside 0-180 degree range
- **THEN** value is clamped to valid range
- **AND** warning is logged

#### Scenario: Validate motor speed input
- **WHEN** motor speed is set outside -255 to +255 range
- **THEN** value is clamped to valid range
- **AND** warning is logged

#### Scenario: Emergency stop all actuators
- **WHEN** emergency stop function is called
- **THEN** all servo PWM outputs are disabled
- **AND** all BTS7960 drivers are set to coast (both PWM = 0)
- **AND** the steering VESC driver is commanded to zero duty (coast)
- **AND** system enters safe state requiring explicit reset

**Note**: BTS7960 enable pins are hardwired, so the brake driver cannot be fully disabled and coasts to a stop. The steering VESC releases the motor (coast) when commanded to zero duty and also releases automatically on its own configured comm timeout if commands stop.

### Requirement: Hardware Configuration
The system SHALL define all hardware pin assignments in centralized configuration
(`include/Constants.h`), matching the hand-wired ESP32-S3-DevKitC-1 as documented in
`GPIO_PINOUT_S3.md`. There SHALL be exactly one pin map and no board-selection build flag.

#### Scenario: Load pin configuration at startup
- **WHEN** system initializes
- **THEN** all pin assignments are loaded from `Constants.h`
- **AND** pins are validated against ESP32-S3 available GPIO (not ESP32-C6)
- **AND** pin conflicts are checked and reported

#### Scenario: DevKitC-1 pin assignments
- **WHEN** `Constants.h` is read
- **THEN** the following assignments SHALL be in effect:
  - `PIN_THROTTLE_PWM` = GPIO3 (LEDC channel 1, 50 Hz)
  - `PIN_TRANS_SERVO` = GPIO9 (LEDC channel 2, 50 Hz)
  - `PIN_MAVLINK_RX` = GPIO8, `PIN_MAVLINK_TX` = GPIO15 (UART1)
  - `PIN_VESC_TX` = GPIO4, `PIN_VESC_RX` = GPIO5 (UART2)
  - `PIN_BRAKE_LPWM` = GPIO6, `PIN_BRAKE_RPWM` = GPIO7 (LEDC channels 5/4)
  - `PIN_CAN_CS` = GPIO10, `PIN_CAN_MOSI` = GPIO11, `PIN_CAN_SCK` = GPIO12, `PIN_CAN_MISO` = GPIO13
    (MCP2515 — engine ECU and the Bosch LWS steering-angle sensor on the same bus)
  - `PIN_BRAKE_SENSOR` = GPIO14
  - `PIN_SPEED_SENSOR` = GPIO17 (PCNT hall pulse input)
  - `PIN_GEAR_REVERSE` = GPIO19, `PIN_GEAR_NEUTRAL` = GPIO20, `PIN_GEAR_LOW` = GPIO21,
    `PIN_GEAR_HIGH` = GPIO47
  - `PIN_RELAY1` = GPIO36, `PIN_RELAY2` = GPIO37, `PIN_RELAY3` = GPIO38, `PIN_WHEEL_LOCK` = GPIO39

#### Scenario: Reserved and unused pins
- **WHEN** `Constants.h` is read
- **THEN** GPIO18 SHALL be left free/reserved (the former `PIN_STEER_LPWM`)
- **AND** GPIO41/42 SHALL be left free/reserved (the former AS5600 I2C pins); `PIN_STEER_SDA`,
  `PIN_STEER_SCL` and `I2C_BUS_FREQ_HZ` SHALL NOT be defined and no I2C bus SHALL be started
- **AND** GPIO43/44 SHALL be the UART0 debug console and GPIO19/20 SHALL be the R/N gear switches
  (native USB CDC disabled)
- **AND** `LEDC_CH_STEER_RPWM` (6) and `LEDC_CH_STEER_LPWM` (7) SHALL remain defined but unallocated
- **AND** `PIN_STEER_RPWM` and `PIN_STEER_LPWM` SHALL NOT be defined — GPIO17 is now the speed sensor
- **AND** no reference to those two symbols SHALL remain in `src/` or `include/`

#### Scenario: Configure PWM parameters
- **WHEN** PWM peripherals are initialized
- **THEN** servo PWM frequency is set to 50Hz (20ms period)
- **AND** motor PWM frequency is set to 1-20kHz (configurable)
- **AND** PWM resolution is set to 12-16 bits based on frequency
- **AND** LEDC channels 1 (throttle), 2 (transmission), 4 (brake RPWM) and 5 (brake LPWM) SHALL be
  the only channels allocated; channels 3, 6 and 7 SHALL be free

#### Scenario: Steering sensor needs no GPIO
- **WHEN** the steering controller is initialized
- **THEN** the steering angle SHALL be received through the existing MCP2515 CAN controller
- **AND** no GPIO and no I2C bus SHALL be allocated to steering position feedback

#### Scenario: Steering sensors share the pins of their buses
- **WHEN** the steering controller is initialized
- **THEN** the Bosch LWS steering-angle sensor SHALL share the CAN bus (MCP2515 on SPI,
  `PIN_CAN_*`) with the ECU OBD-II traffic
- **AND** no dedicated steering GPIO SHALL be allocated
- **AND** the former steering I2C pins (`Wire` / `Wire1`) SHALL NOT be used for steering

#### Scenario: Wiring document is authoritative
- **WHEN** `Constants.h` and `GPIO_PINOUT_S3.md` disagree about a pin
- **THEN** the document SHALL be treated as authoritative for what the board is physically wired to
- **AND** the divergence SHALL be resolved before the firmware is run on the vehicle

### Requirement: Actuator State Monitoring
The system SHALL track and report current state of all actuators.

#### Scenario: Query servo position
- **WHEN** getCurrentAngle() is called on ServoController
- **THEN** last commanded angle is returned
- **AND** actual position matches command within tolerance (if feedback available)

#### Scenario: Query motor state
- **WHEN** getSpeed() is called on BTS7960Controller
- **THEN** current speed setting is returned
- **AND** direction (forward/reverse/stopped) is indicated

#### Scenario: Check actuator health
- **WHEN** isHealthy() is called on any controller
- **THEN** health status is returned based on last operation success
- **AND** fault conditions are reported (overcurrent, timeout, etc.)

### Requirement: Transmission Servo Control
The transmission servo SHALL use an overshoot-dwell-return motion profile and SHALL enforce sequential gear shifting through the physical order `[R, N, H, L]`.

#### Scenario: Overshoot-and-dwell motion profile
- **WHEN** `setGear(target)` is called
- **THEN** the servo SHALL first move to the overshoot position (`targetPct ± overshootPct`)
- **AND** once servo movement time has elapsed, the controller SHALL enter an overshoot dwell of exactly `TRANS_OVERSHOOT_DWELL_MS` milliseconds
- **AND** after `TRANS_OVERSHOOT_DWELL_MS` elapses, the gear SHALL be assumed engaged regardless of physical sensor state
- **AND** the controller SHALL transition directly to the RETURN phase

#### Scenario: Return phase after assumed gear engagement
- **WHEN** the overshoot dwell period elapses
- **THEN** the servo SHALL move from the overshoot position back to `finalGearPositionPct_`
- **AND** once return movement time has elapsed, the gear change SHALL be marked complete
- **AND** `saveState()` SHALL be called with `state_valid=true`

#### Scenario: Sequential gear shifting — step through sequence
- **WHEN** `setGear(finalGear)` is called
- **AND** `finalGear` is not adjacent to the current gear in the sequence `[R, N, H, L]`
- **THEN** the controller SHALL compute the immediate next step toward `finalGear`
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** the gear change SHALL start toward the immediate next step only

#### Scenario: Sequential gear shifting — advance queue after step completes
- **WHEN** a gear change RETURN phase completes
- **AND** `queuedGear_ != GEAR_UNKNOWN`
- **AND** `targetGear_ != queuedGear_`
- **THEN** the controller SHALL automatically start the next step toward `queuedGear_` using `targetGear_` as the current position
- **AND** steps SHALL continue until `targetGear_ == queuedGear_`
- **AND** `queuedGear_` SHALL be cleared to `GEAR_UNKNOWN` when the final gear is reached

#### Scenario: Sequential shifting interrupted by new command
- **WHEN** `setGear(newFinalGear)` is called while a sequence is in progress
- **THEN** the in-flight gear change to the current intermediate target SHALL complete normally
- **AND** `queuedGear_` SHALL be updated to `newFinalGear`
- **AND** subsequent steps SHALL route toward `newFinalGear`

#### Scenario: Sequential shifting — unknown current gear
- **WHEN** `setGear(finalGear)` is called
- **AND** `targetGear_` is `GEAR_UNKNOWN` or no prior change has completed
- **THEN** the controller SHALL route through `GEAR_NEUTRAL` first (safe intermediate)
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** once NEUTRAL step completes, normal sequencing toward `finalGear` resumes

### Requirement: Steering Motor Driver Abstraction
The steering controller SHALL command its motor through an `IMotorDriver` abstraction exposing `setSpeed(int16_t -255..+255)` and `stop()`, so that the position control loop is independent of the concrete driver.

#### Scenario: Position loop drives through the abstraction
- **WHEN** the steering position control loop computes a drive effort
- **THEN** it SHALL call `IMotorDriver::setSpeed()` with a value in `-255..+255` (positive = wheel right)
- **AND** the proportional position loop (angle error in 0.1° → clamped duty, `STEER_KP` duty per 0.1°) SHALL be unchanged in structure by the driver substitution

#### Scenario: Coast on stop
- **WHEN** the steering controller calls `IMotorDriver::stop()`
- **THEN** the motor SHALL coast (zero drive), matching the prior BTS7960 coast semantics

### Requirement: VESC Brushed-DC Steering Driver
The steering `IMotorDriver` SHALL be implemented by a VESC driver that commands a Flipsky 75200 VESC in brushed-DC motor mode over a dedicated hardware UART using the VESC serial protocol.

#### Scenario: Map speed to duty command
- **WHEN** `setSpeed(speed)` is called on the VESC steering driver
- **THEN** it SHALL send `COMM_SET_DUTY` with `duty = constrain(speed / 255.0, -1.0, +1.0)`
- **AND** `setSpeed(0)` and `stop()` SHALL send zero duty (coast)

#### Scenario: Dedicated UART, isolated from MAVLink
- **WHEN** the VESC driver initializes
- **THEN** it SHALL use `UART_NUM_2` on `PIN_VESC_TX` (GPIO4) and `PIN_VESC_RX` (GPIO5) at `VESC_UART_BAUD`,
  passing both pins explicitly
- **AND** it SHALL NOT use `UART_NUM_1` (MAVLink) or `UART_NUM_0` (the debug console)

#### Scenario: Console separate from the VESC UART
- **WHEN** the firmware is running
- **THEN** console and log output SHALL go to UART0 (GPIO43/44, the DevKit's USB-UART bridge) only
- **AND** no console or log output SHALL be written to the VESC UART (UART2)

#### Scenario: Minimal in-repo protocol layer
- **WHEN** the driver frames or parses a VESC packet
- **THEN** it SHALL use an in-repo packet layer implementing VESC short/long framing with a CRC16 check
- **AND** it SHALL implement only `COMM_SET_DUTY` and `COMM_GET_VALUES`

### Requirement: Steering Re-command Guard and Stall Latch
The steering controller SHALL NOT restart its move and stall timers when re-commanded to the current target, and SHALL latch out further motion in a stalled direction for a cooldown of `STEER_STALL_COOLDOWN_MS` (700 ms), so that stall detection and move timeout function under the continuous MAVLink command stream. Target and tolerance values SHALL be in 0.1° of steering angle.

#### Scenario: Re-command within tolerance is a no-op
- **WHEN** `setSteeringPercent()` is called while a move is in progress
- **AND** the new target is within `STEER_RETARGET_TOLERANCE` (0.1° units) of the current target
- **THEN** `moveStartTime_`, the stall-check timers, and `dutyBoost_` SHALL NOT be reset
- **AND** the in-flight move SHALL continue so the stall window and move timeout can elapse

#### Scenario: Target change beyond tolerance resets the move
- **WHEN** `setSteeringPercent()` is called with a target differing from the current target by more than `STEER_RETARGET_TOLERANCE` (0.1° units)
- **THEN** a fresh move SHALL start with the move/stall timers reset

#### Scenario: Stall latches out the stalled direction
- **WHEN** a stall-stop occurs (position loop stall with a healthy sensor, or a nonzero VESC fault code)
- **THEN** the controller SHALL record the stalled direction and latch time and stop the motor
- **AND** a subsequent move, jog or nudge that would push further in the stalled direction SHALL be refused while less than `STEER_STALL_COOLDOWN_MS` (700 ms) has elapsed

#### Scenario: Opposite-direction escape and post-cooldown retry
- **WHEN** the controller is stall-latched
- **AND** a new move commands the opposite direction (away from or across the jam)
- **THEN** the move SHALL be accepted immediately and the latch cleared
- **WHEN** the controller is stall-latched and a same-direction move is commanded after `STEER_STALL_COOLDOWN_MS` (700 ms) has elapsed
- **THEN** the move SHALL be accepted and the latch cleared

### Requirement: VESC Fault Monitoring and Communication Failsafe
The steering controller SHALL poll VESC telemetry to enforce a firmware-level fault backstop, and SHALL fail safe when the VESC is unresponsive. Over-current protection SHALL be left to the VESC itself (its *Motor Current Max* clamp, its *Absolute Max Current* fault, and its MOSFET temperature limiting), and mechanical jams SHALL be left to the steering-angle stall detector while the angle sensor is healthy, and to the VESC current clamp and fault stop while steering is driven sensorless; the firmware SHALL NOT run its own sustained-over-current timer.

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
- **AND** if a move, jog or nudge was in progress — including a sensorless jog — it SHALL take the stall-stop path (stop + stall latch in the current drive direction)

#### Scenario: Unresponsive VESC fails safe
- **WHEN** no valid `COMM_GET_VALUES` reply is received for `STEER_VESC_COMM_TIMEOUT_MS`
- **THEN** the driver-ok flag SHALL be set false
- **AND** the motor SHALL be stopped and all steering commands (position, jog and nudge) SHALL be rejected
- **AND** normal operation SHALL resume automatically once valid replies return
- **AND** a VESC that is silent at boot SHALL NOT block startup

### Requirement: Bosch LWS Steering Angle Sensor
The steering position SHALL be read from a Bosch LWS steering-angle sensor 0 265 005 486 (PSA 9650236180) mounted directly on the steering shaft, which transmits standard CAN frame `0x305` (DLC 6, 100 Hz, 500 kbit/s) by itself after 12 V power-up from the ignition key. The decoder SHALL be free of Arduino dependencies and SHALL reproduce the bench-verified `lws305.h` logic.

#### Scenario: Decode an accepted frame
- **WHEN** a standard-ID `0x305` frame with DLC ≥ 6 is received
- **AND** the high nibble of byte 4 equals the XOR of the eight nibbles of bytes 0..3 and the counter (low nibble of byte 4)
- **AND** the counter differs from that of the last accepted frame
- **THEN** the frame SHALL be accepted
- **AND** the angle SHALL be bytes 0–1 as big-endian int16 in 0.1°, with `0x7FFF` meaning invalid
- **AND** the rotation speed SHALL be byte 2 × 4 °/s, with `0xFF` meaning invalid, and the direction SHALL be byte 3 bit 7 (1 = clockwise)
- **AND** the status SHALL be byte 3: bits 6..3 fault code (0 = none), bit 2 adjusted, bit 1 calibrated, bit 0 sensor OK
- **AND** the firmware position SHALL be the negated angle for a normal mount (sensor positive = left; firmware positive = right), or the angle unchanged when the sensor-direction flag says the sensor is mounted mirrored (see "Steering Sensor Direction Inference" in vehicle-systems)
- **AND** the signed rotation rate SHALL follow the same rule, so that a positive rate always means turning right
- **AND** the decoder itself SHALL stay sign-neutral: the raw angle and direction bit SHALL remain available in the sensor's own sign for logs

#### Scenario: Reject corrupt or repeated frames
- **WHEN** a `0x305` frame fails the checksum
- **THEN** it SHALL be dropped and a checksum-error counter incremented
- **WHEN** a `0x305` frame carries the same counter as the last accepted frame
- **THEN** it SHALL be dropped and a counter-error counter incremented
- **WHEN** the counter advanced by more than one since the last accepted frame
- **THEN** the frame SHALL be accepted and the number of missed frames added to a miss counter, since every frame is absolute

#### Scenario: Sensor health
- **WHEN** the steering controller evaluates the sensor
- **THEN** it SHALL report exactly one fault token, in this priority order: `zeroing` (zero-set sequence running), `can_down` (CAN controller not initialized), `timeout` (no accepted frame for `STEER_LWS_TIMEOUT_MS`), `range_latched` (fault code 9), `sensor_fault` (OK bit clear or any other nonzero fault code), `not_calibrated` (calibrated bit clear), `angle_invalid` (angle `0x7FFF`), otherwise `none`
- **AND** a fault SHALL take effect on the first frame or timeout that shows it
- **AND** the sensor SHALL be healthy only when the token is `none` for at least `STEER_LWS_RECOVER_FRAMES` consecutive accepted frames

#### Scenario: Latched out-of-range fault
- **WHEN** the sensor reports fault code 9 (it was turned beyond its ~±600° range)
- **THEN** the token SHALL be `range_latched` and telemetry SHALL tell the operator to restart the vehicle
- **AND** the firmware SHALL take no other action to clear it (the sensor is powered from the ignition key and clears only on a power cycle)

#### Scenario: Range guard for closed-loop moves only
- **WHEN** the sensor is healthy and a closed-loop position move driven by a percent command (MAVLink, web slider or test buttons, boot centering, failsafe centre) would reach or pass `STEER_LWS_SAFE_RANGE` (550.0°)
- **THEN** its target SHALL be clamped to ±`STEER_LWS_SAFE_RANGE`, and the move SHALL be stopped rather than drive further outward at or beyond it
- **AND** driving back toward zero SHALL remain allowed
- **AND** jog, nudge and limit captures SHALL NOT be limited by the range guard

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

