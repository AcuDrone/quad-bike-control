## MODIFIED Requirements
### Requirement: Hardware Configuration
The system SHALL define all hardware pin assignments in centralized configuration
(`include/Constants.h`), matching the custom `Control_v0` control board (ESP32-S3-WROOM-1U) as
documented in `GPIO_PINOUT_CUSTOM_BOARD_S3.md`. The DevKitC-1 hand-wired pin map SHALL NOT be
supported; there SHALL be exactly one pin map and no board-selection build flag.

#### Scenario: Load pin configuration at startup
- **WHEN** system initializes
- **THEN** all pin assignments are loaded from `Constants.h`
- **AND** pins are validated against ESP32-S3 available GPIO (not ESP32-C6)
- **AND** GPIO19/20 SHALL NOT be assigned any firmware function — they are the native USB data lines
- **AND** GPIO43/44 (UART0) SHALL be assigned only to the steering VESC link on the Prog header X1
  (the aztec board has R63/R64 to the CH9121T removed); GPIO4/5 (X6) SHALL be left free for a
  fourth BTS7960
- **AND** pin conflicts are checked and reported

#### Scenario: Control_v0 pin assignments
- **WHEN** `Constants.h` is read
- **THEN** the following assignments SHALL be in effect:
  - `PIN_THROTTLE_PWM` = GPIO15 (LEDC channel 1, 50 Hz, X8 pin 2)
  - `PIN_TRANS_SERVO` = GPIO16 (LEDC channel 2, 50 Hz, X8 pin 3)
  - `PIN_MAVLINK_TX` = GPIO17, `PIN_MAVLINK_RX` = GPIO18 (UART1, X9)
  - `PIN_VESC_TX` = GPIO43, `PIN_VESC_RX` = GPIO44 (UART0, Prog header X1)
  - `PIN_BRAKE_LPWM` = GPIO6, `PIN_BRAKE_RPWM` = GPIO7 (LEDC channels 5/4, X7) — unchanged
  - `PIN_CAN_CS` = GPIO39, `PIN_CAN_SCK` = GPIO40, `PIN_CAN_MISO` = GPIO41, `PIN_CAN_MOSI` = GPIO42
    (MCP2515 — engine ECU and the Bosch LWS steering-angle sensor on the same bus)
  - `PIN_RELAY_SDA` = GPIO48, `PIN_RELAY_SCL` = GPIO47 (I2C2 — on-board expanders U10/U2 and the
    relay board `0x1F` on header X15)
  - `PIN_BOOST_EN` = GPIO46, `PIN_ADC_24V` = GPIO9 (X10)
  - `PIN_I2C1_SDA` = GPIO1, `PIN_I2C1_SCL` = GPIO2 (I2C1 — external header X14, no device fitted),
    `PIN_CAN_INT` = GPIO38 and `PIN_ADC_IS_MUX` = GPIO3 SHALL be defined but unused

#### Scenario: Functions with no GPIO
- **WHEN** `Constants.h` is read
- **THEN** `PIN_GEAR_REVERSE`, `PIN_GEAR_NEUTRAL`, `PIN_GEAR_LOW`, `PIN_GEAR_HIGH` and
  `PIN_BRAKE_SENSOR` SHALL NOT be defined — those inputs are read from the opto-input expander
- **AND** `PIN_RELAY1`, `PIN_RELAY2`, `PIN_RELAY3` and `PIN_WHEEL_LOCK` SHALL NOT be defined — those
  outputs are driven by the relay-board expander
- **AND** `PIN_STEER_RPWM`, `PIN_STEER_LPWM`, `LEDC_CH_STEER_RPWM` and `LEDC_CH_STEER_LPWM` SHALL NOT
  be defined — GPIO17/18 are now the MAVLink UART
- **AND** `PIN_STEER_SDA` and `PIN_STEER_SCL` SHALL NOT be defined — the steering sensor is on CAN
- **AND** no reference to any of these symbols SHALL remain in `src/` or `include/`

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

#### Scenario: Wiring document is authoritative
- **WHEN** `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` disagree about a pin
- **THEN** the document SHALL be treated as authoritative for what the board is physically wired to
- **AND** the divergence SHALL be resolved before the firmware is run on the vehicle

### Requirement: Steering Motor Driver Abstraction
The steering controller SHALL command its motor through an `IMotorDriver` abstraction exposing `setSpeed(int16_t -255..+255)` and `stop()`, so that the position control loop is independent of the concrete driver.

#### Scenario: Position loop drives through the abstraction
- **WHEN** the steering position control loop computes a drive effort
- **THEN** it SHALL call `IMotorDriver::setSpeed()` with a value in `-255..+255` (positive = wheel right)
- **AND** the proportional position loop (angle error in 0.1° → clamped duty, `STEER_KP` duty per 0.1°) SHALL be unchanged in structure by the driver substitution

#### Scenario: Coast on stop
- **WHEN** the steering controller calls `IMotorDriver::stop()`
- **THEN** the motor SHALL coast (zero drive), matching the prior BTS7960 coast semantics

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

## ADDED Requirements
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
- **AND** the firmware position SHALL be the negated angle (sensor positive = left; firmware positive = right)

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
Jog and nudge commands SHALL drive the steering motor whenever the steering motor driver link is healthy, regardless of the steering angle sensor state, so that the steering can be centred and the sensor zeroed without a working sensor. Percent position commands SHALL still require a healthy sensor and both limits.

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
