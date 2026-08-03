## MODIFIED Requirements

### Requirement: BTS7960 Motor Driver Control
The system SHALL use a BTS7960 motor driver for the brake actuator, driven through `BTS7960Controller` with LEDC PWM on RPWM/LPWM. The steering actuator SHALL NOT use `BTS7960Controller`; it is driven by a VESC over UART (see "VESC Brushed-DC Steering Driver"). Transmission control remains on a PWM servo.

#### Scenario: BTS7960 for brake actuator
- **WHEN** the brake actuator is commanded via `BTS7960Controller::setSpeed()`
- **THEN** RPWM/LPWM PWM outputs SHALL be set as specified in the existing brake scenarios
- **AND** RPWM and LPWM SHALL never be driven high simultaneously

#### Scenario: Steering no longer uses BTS7960
- **WHEN** the steering actuator is commanded
- **THEN** the command SHALL be issued through the `IMotorDriver` abstraction backed by the VESC UART driver
- **AND** no `BTS7960Controller` instance SHALL be associated with the steering actuator
- **AND** the freed steering PWM GPIOs (17/18) and LEDC channels (6/7) SHALL remain reserved and unassigned to other functions

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

## ADDED Requirements

### Requirement: Steering Motor Driver Abstraction
The steering controller SHALL command its motor through an `IMotorDriver` abstraction exposing `setSpeed(int16_t -255..+255)` and `stop()`, so that the position control loop is independent of the concrete driver.

#### Scenario: Position loop drives through the abstraction
- **WHEN** the steering position control loop computes a drive effort
- **THEN** it SHALL call `IMotorDriver::setSpeed()` with a value in `-255..+255` (positive = wheel right)
- **AND** the existing AS5600 proportional position loop (error → clamped duty) SHALL be unchanged by the driver substitution

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
- **THEN** it SHALL use `UART_NUM_2` on `PIN_VESC_TX` (GPIO4) and `PIN_VESC_RX` (GPIO5) at `VESC_UART_BAUD`
- **AND** it SHALL NOT use `UART_NUM_1` (MAVLink) or `UART_NUM_0` (debug console)

#### Scenario: Minimal in-repo protocol layer
- **WHEN** the driver frames or parses a VESC packet
- **THEN** it SHALL use an in-repo packet layer implementing VESC short/long framing with a CRC16 check
- **AND** it SHALL implement only `COMM_SET_DUTY` and `COMM_GET_VALUES`

### Requirement: Steering Re-command Guard and Stall Latch
The steering controller SHALL NOT restart its move and stall timers when re-commanded to the current target, and SHALL latch out further motion in a stalled direction for a cooldown, so that stall detection and move timeout function under the continuous MAVLink command stream.

#### Scenario: Re-command within tolerance is a no-op
- **WHEN** `setSteeringPercent()` is called while a move is in progress
- **AND** the new target is within `STEER_RETARGET_TOLERANCE` counts of the current target
- **THEN** `moveStartTime_`, the stall-check timers, and `dutyBoost_` SHALL NOT be reset
- **AND** the in-flight move SHALL continue so the stall window and move timeout can elapse

#### Scenario: Target change beyond tolerance resets the move
- **WHEN** `setSteeringPercent()` is called with a target differing from the current target by more than `STEER_RETARGET_TOLERANCE` counts
- **THEN** a fresh move SHALL start with the move/stall timers reset

#### Scenario: Stall latches out the stalled direction
- **WHEN** a stall-stop occurs (position loop stall, firmware over-current, or a VESC fault)
- **THEN** the controller SHALL record the stalled direction and latch time and stop the motor
- **AND** a subsequent move that would push further in the stalled direction SHALL be refused while less than `STEER_STALL_COOLDOWN_MS` has elapsed

#### Scenario: Opposite-direction escape and post-cooldown retry
- **WHEN** the controller is stall-latched
- **AND** a new move commands the opposite direction (away from or across the jam)
- **THEN** the move SHALL be accepted immediately and the latch cleared
- **WHEN** the controller is stall-latched and a same-direction move is commanded after `STEER_STALL_COOLDOWN_MS` has elapsed
- **THEN** the move SHALL be accepted and the latch cleared

### Requirement: VESC Telemetry Fault Monitoring and Communication Failsafe
The steering controller SHALL poll VESC telemetry to enforce a firmware-level over-current and fault backstop, and SHALL fail safe when the VESC is unresponsive.

#### Scenario: Poll VESC telemetry periodically
- **WHEN** the steering driver is active
- **THEN** it SHALL request `COMM_GET_VALUES` at approximately `STEER_VESC_TELEM_MS` intervals (~2-5 Hz)
- **AND** decode motor current, FET temperature, input voltage, and fault code

#### Scenario: Sustained over-current triggers stall-stop
- **WHEN** decoded motor current exceeds `STEER_VESC_OVERCURRENT_A` for at least `STEER_VESC_OVERCURRENT_MS`
- **THEN** the controller SHALL invoke the stall-stop path (stop + stall latch in the current drive direction)

#### Scenario: VESC fault code stops the motor
- **WHEN** `COMM_GET_VALUES` reports a nonzero VESC fault code
- **THEN** the controller SHALL stop the motor and raise a fault flag in telemetry

#### Scenario: Unresponsive VESC fails safe
- **WHEN** no valid `COMM_GET_VALUES` reply is received for `STEER_VESC_COMM_TIMEOUT_MS`
- **THEN** the driver-ok flag SHALL be set false
- **AND** the motor SHALL be stopped and steering commands (position and jog) SHALL be rejected, mirroring the sensor-fault path
- **AND** normal operation SHALL resume automatically once valid replies return
- **AND** a VESC that is silent at boot SHALL NOT block startup
