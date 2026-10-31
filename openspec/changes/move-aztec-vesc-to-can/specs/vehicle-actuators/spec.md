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
- **AND** GPIO43/44 (UART0, Prog header X1) SHALL NOT be assigned any firmware function — the
  steering VESC is on the CAN bus; the aztec board keeps R63/R64 to the CH9121T removed and the
  CH9121T is not used; GPIO4/5 (X6) SHALL be left free for a fourth BTS7960
- **AND** pin conflicts are checked and reported

#### Scenario: Control_v0 pin assignments
- **WHEN** `Constants.h` is read
- **THEN** the following assignments SHALL be in effect:
  - `PIN_THROTTLE_PWM` = GPIO15 (LEDC channel 1, 50 Hz, X8 pin 2)
  - `PIN_TRANS_SERVO` = GPIO16 (LEDC channel 2, 50 Hz, X8 pin 3)
  - `PIN_MAVLINK_TX` = GPIO17, `PIN_MAVLINK_RX` = GPIO18 (UART1, X9)
  - `PIN_BRAKE_LPWM` = GPIO6, `PIN_BRAKE_RPWM` = GPIO7 (LEDC channels 5/4, X7) — unchanged
  - `PIN_CAN_CS` = GPIO39, `PIN_CAN_SCK` = GPIO40, `PIN_CAN_MISO` = GPIO41, `PIN_CAN_MOSI` = GPIO42
    (MCP2515 — engine ECU, the Bosch LWS steering-angle sensor and the steering VESC on the same
    bus)
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
- **AND** `PIN_VESC_TX`, `PIN_VESC_RX`, `VESC_UART_NUM` and `VESC_UART_BAUD` SHALL NOT be defined — the
  steering VESC is on CAN
- **AND** no reference to any of these symbols SHALL remain in `src/` or `include/`

#### Scenario: Configure PWM parameters
- **WHEN** PWM peripherals are initialized
- **THEN** servo PWM frequency is set to 50Hz (20ms period)
- **AND** motor PWM frequency is set to 1-20kHz (configurable)
- **AND** PWM resolution is set to 12-16 bits based on frequency
- **AND** LEDC channels 1 (throttle), 2 (transmission), 4 (brake RPWM) and 5 (brake LPWM) SHALL be
  the only channels allocated; channels 3, 6 and 7 SHALL be free

#### Scenario: Steering sensor and steering driver need no GPIO
- **WHEN** the steering controller is initialized
- **THEN** the steering angle SHALL be received through the existing MCP2515 CAN controller
- **AND** the steering VESC SHALL be commanded and monitored through the same MCP2515 CAN controller
- **AND** no GPIO, no UART and no I2C bus SHALL be allocated to steering position feedback or to the
  steering motor driver

#### Scenario: Wiring document is authoritative
- **WHEN** `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` disagree about a pin
- **THEN** the document SHALL be treated as authoritative for what the board is physically wired to
- **AND** the divergence SHALL be resolved before the firmware is run on the vehicle

### Requirement: VESC Brushed-DC Steering Driver
The steering `IMotorDriver` SHALL be implemented by a VESC driver that commands a Flipsky 75200 VESC in brushed-DC motor mode over the shared MCP2515 CAN bus (500 kbit/s) using the native VESC CAN protocol: 29-bit extended identifiers `(cmd << 8) | id`, with the VESC at controller ID `STEER_VESC_CAN_ID` (1) and the firmware's own reply address `CAN_CONTROLLER_VESC_SENDER_ID` (2).

#### Scenario: Map speed to duty command
- **WHEN** `setSpeed(speed)` is called on the VESC steering driver
- **THEN** the duty SHALL be `duty_e5 = constrain(speed, -255, +255) × 100000 / 255` in integer arithmetic (255 → 100000, 140 → 54901)
- **AND** it SHALL be sent as `CAN_PACKET_SET_DUTY` (cmd 0), extended ID `0x00000001`, DLC 4, payload int32 big-endian `duty_e5`
- **AND** `setSpeed(0)` and `stop()` SHALL send zero duty (coast)

#### Scenario: Rate-limited sending with keep-alive
- **WHEN** the wanted duty differs from the last duty sent
- **THEN** it SHALL be sent at once if at least `STEER_VESC_CAN_MIN_TX_MS` (10 ms) has passed since the last duty frame, otherwise as soon as that gap has passed, always with the newest wanted value
- **AND** a zero duty SHALL be sent on the call that requests it, without waiting for the gap
- **WHEN** the last duty sent is non-zero and no duty frame has been sent for `STEER_VESC_CAN_KEEPALIVE_MS` (200 ms)
- **THEN** the same duty SHALL be sent again, so the VESC's own command timeout (1000 ms) does not expire while driving
- **AND** a zero duty SHALL NOT be re-sent once it has been transmitted successfully
- **WHEN** a duty frame fails to transmit
- **THEN** it SHALL stay pending and be retried at the next opportunity, including a zero duty

#### Scenario: Controller IDs
- **WHEN** the firmware is built
- **THEN** `STEER_VESC_CAN_ID` SHALL be 1 and `CAN_CONTROLLER_VESC_SENDER_ID` SHALL be 2
- **AND** VESC controller ID 0 SHALL NOT be used, and the two IDs SHALL differ
- **AND** the operator documentation SHALL state that the VESC must be set in VESC Tool to controller ID 1, CAN baud 500K, CAN mode VESC, and status messages STATUS_1, STATUS_4 and STATUS_5 at 5 Hz

#### Scenario: Dedicated UART, isolated from MAVLink
- **WHEN** the firmware initializes
- **THEN** no UART SHALL be opened for the steering VESC (the former dedicated UART0 link is replaced by CAN), and `UART_NUM_1` SHALL remain the MAVLink link only

#### Scenario: Minimal in-repo protocol layer
- **WHEN** the driver encodes or decodes a VESC CAN frame
- **THEN** it SHALL use an in-repo, Arduino-free, header-only layer (`include/VescCan.h`, with the CRC16 and command IDs in `include/VescProtocol.h`) covered by host unit tests
- **AND** it SHALL transmit only `CAN_PACKET_SET_DUTY` and `CAN_PACKET_PROCESS_SHORT_BUFFER` carrying `COMM_GET_VALUES_SELECTIVE`
- **AND** it SHALL decode only `CAN_PACKET_STATUS`, `CAN_PACKET_STATUS_4`, `CAN_PACKET_STATUS_5` and the forwarded fault-code reply

### Requirement: VESC Fault Monitoring and Communication Failsafe
The steering controller SHALL monitor the VESC's CAN status broadcasts and a periodic fault-code poll to enforce a firmware-level fault backstop, and SHALL fail safe when the VESC is silent. Over-current protection SHALL be left to the VESC itself (its *Motor Current Max* clamp, its *Absolute Max Current* fault, and its MOSFET temperature limiting), and mechanical jams SHALL be left to the steering-angle stall detector while the angle sensor is healthy, and to the VESC current clamp and fault stop while steering is driven sensorless; the firmware SHALL NOT run its own sustained-over-current timer.

#### Scenario: Poll VESC telemetry periodically
- **WHEN** the steering driver is active
- **THEN** the VESC SHALL broadcast `CAN_PACKET_STATUS`, `CAN_PACKET_STATUS_4` and `CAN_PACKET_STATUS_5` at 5 Hz (VESC Tool setting), and the fault code SHALL be polled every `STEER_VESC_FAULT_POLL_MS`
- **WHEN** an extended frame from the VESC (ID low byte = `STEER_VESC_CAN_ID`) is received with cmd 9 (`CAN_PACKET_STATUS`), 16 (`CAN_PACKET_STATUS_4`) or 27 (`CAN_PACKET_STATUS_5`)
- **THEN** motor current SHALL be STATUS bytes 4–5 (int16 big-endian ÷ 10, A)
- **AND** FET temperature SHALL be STATUS_4 bytes 0–1 (int16 big-endian ÷ 10, °C)
- **AND** input voltage SHALL be STATUS_5 bytes 4–5 (int16 big-endian ÷ 10, V)
- **AND** a frame shorter than the bytes it needs SHALL be dropped and counted

#### Scenario: Fault code from the periodic poll
- **WHEN** a valid fault-code reply is received (requirement "VESC Fault Code Poll via CAN Forwarding")
- **THEN** the driver's fault code SHALL be set to the reported value

#### Scenario: Motor current is reported but not acted on
- **WHEN** decoded motor current is high, or a status frame is lost and the last decoded sample is stale
- **THEN** the controller SHALL NOT trip a stall-stop on current magnitude
- **AND** the decoded value SHALL still be published as `steer_motor_current` telemetry

#### Scenario: VESC fault code stops the motor
- **WHEN** the fault-code poll reports a nonzero VESC fault code
- **THEN** the controller SHALL stop the motor and raise a fault flag in telemetry
- **AND** if a move, jog or nudge was in progress — including a sensorless jog — it SHALL take the stall-stop path (stop + stall latch in the current drive direction)

#### Scenario: Unresponsive VESC fails safe
- **WHEN** no valid `CAN_PACKET_STATUS` (STATUS_1) frame from the VESC is received for `STEER_VESC_COMM_TIMEOUT_MS` (1000 ms)
- **THEN** the driver-ok flag SHALL be set false
- **AND** the motor SHALL be stopped and all steering commands (position, jog and nudge) SHALL be rejected
- **AND** normal operation SHALL resume automatically once STATUS_1 frames return
- **AND** a VESC that is silent at boot SHALL NOT block startup
- **AND** STATUS_4, STATUS_5 and fault-code replies SHALL NOT set the driver-ok flag

## ADDED Requirements
### Requirement: VESC Fault Code Poll via CAN Forwarding
The VESC steering driver SHALL read the VESC fault code every `STEER_VESC_FAULT_POLL_MS` (5000 ms) with `COMM_GET_VALUES_SELECTIVE` sent through VESC CAN forwarding, and SHALL parse the reply in both the single-frame and the multi-frame form. There SHALL be no on-demand fault poll.

#### Scenario: Poll request frame
- **WHEN** `STEER_VESC_FAULT_POLL_MS` has elapsed since the last request and the driver-ok flag is true
- **THEN** the driver SHALL send `CAN_PACKET_PROCESS_SHORT_BUFFER` (cmd 8) to the VESC, extended ID `0x00000801`, DLC 7, data `[CAN_CONTROLLER_VESC_SENDER_ID, 0x00, 0x32, 0x00, 0x00, 0x80, 0x00]` — sender ID, send flag 0, `COMM_GET_VALUES_SELECTIVE` (50), mask `0x00008000` big-endian (bit 15 = fault code)
- **AND** no fault poll SHALL be sent while the driver-ok flag is false

#### Scenario: Single-frame reply (expected)
- **WHEN** an extended frame with ID `0x00000802` (cmd 8, low byte = `CAN_CONTROLLER_VESC_SENDER_ID`) is received with byte 0 = `STEER_VESC_CAN_ID`
- **THEN** bytes 2.. SHALL be parsed as the selective reply: command byte 50, echoed mask `0x00008000`, then the fault code (uint8)
- **AND** a reply with a different command byte, a different echoed mask or too few bytes SHALL be rejected and counted

#### Scenario: Multi-frame reply
- **WHEN** `CAN_PACKET_FILL_RX_BUFFER` (cmd 5) frames addressed to `CAN_CONTROLLER_VESC_SENDER_ID` are received
- **THEN** each frame's bytes 1.. SHALL be stored at offset byte 0 of a reassembly buffer of 64 bytes
- **WHEN** `CAN_PACKET_PROCESS_RX_BUFFER` (cmd 7) addressed to `CAN_CONTROLLER_VESC_SENDER_ID` is then received with data `[sender, send flag, len_hi, len_lo, crc_hi, crc_lo]`
- **THEN** the buffer SHALL be accepted only if the sender is `STEER_VESC_CAN_ID`, `len` is at most 64 and at most the bytes received, and the CRC16-XMODEM of the first `len` bytes equals the received CRC
- **AND** an accepted buffer SHALL be parsed as the same selective reply

#### Scenario: Missing reply
- **WHEN** no valid reply arrives within `STEER_VESC_FAULT_REPLY_MS` (500 ms) of a request
- **THEN** the last fault code SHALL be kept and a fault-code-stale flag SHALL be set
- **AND** the driver-ok flag SHALL NOT be changed by the missing reply
- **WHEN** a valid reply arrives
- **THEN** the fault-code-stale flag SHALL be cleared
