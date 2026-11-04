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
  - `PIN_STEER_SDA` = GPIO1, `PIN_STEER_SCL` = GPIO2 (I2C1 — external header X14, fine steering
    AS5600 `0x36` only)
  - `PIN_RELAY_SDA` = GPIO48, `PIN_RELAY_SCL` = GPIO47 (I2C2 — on-board expanders U10/U2, the relay
    board `0x1F` and the coarse steering AS5600 `0x36`, both on header X15)
  - `PIN_BOOST_EN` = GPIO46, `PIN_ADC_24V` = GPIO9 (X10)
  - `PIN_CAN_INT` = GPIO38 and `PIN_ADC_IS_MUX` = GPIO3 SHALL be defined but unused

#### Scenario: Functions with no GPIO
- **WHEN** `Constants.h` is read
- **THEN** `PIN_GEAR_REVERSE`, `PIN_GEAR_NEUTRAL`, `PIN_GEAR_LOW`, `PIN_GEAR_HIGH` and
  `PIN_BRAKE_SENSOR` SHALL NOT be defined — those inputs are read from the opto-input expander
- **AND** `PIN_RELAY1`, `PIN_RELAY2`, `PIN_RELAY3` and `PIN_WHEEL_LOCK` SHALL NOT be defined — those
  outputs are driven by the relay-board expander
- **AND** `PIN_STEER_RPWM`, `PIN_STEER_LPWM`, `LEDC_CH_STEER_RPWM` and `LEDC_CH_STEER_LPWM` SHALL NOT
  be defined — GPIO17/18 are now the MAVLink UART
- **AND** no reference to any of these symbols SHALL remain in `src/` or `include/`

#### Scenario: Configure PWM parameters
- **WHEN** PWM peripherals are initialized
- **THEN** servo PWM frequency is set to 50Hz (20ms period)
- **AND** motor PWM frequency is set to 1-20kHz (configurable)
- **AND** PWM resolution is set to 12-16 bits based on frequency
- **AND** LEDC channels 1 (throttle), 2 (transmission), 4 (brake RPWM) and 5 (brake LPWM) SHALL be
  the only channels allocated; channels 3, 6 and 7 SHALL be free

#### Scenario: Steering sensors share the pins of their buses
- **WHEN** the steering controller is initialized
- **THEN** the fine AS5600 SHALL be driven on `Wire` (`PIN_STEER_SDA`/`PIN_STEER_SCL`)
- **AND** the coarse AS5600 SHALL be driven on `Wire1` (`PIN_RELAY_SDA`/`PIN_RELAY_SCL`)
- **AND** no new GPIO SHALL be allocated for the second sensor

#### Scenario: Wiring document is authoritative
- **WHEN** `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` disagree about a pin
- **THEN** the document SHALL be treated as authoritative for what the board is physically wired to
- **AND** the divergence SHALL be resolved before the firmware is run on the vehicle
### Requirement: VESC Brushed-DC Steering Driver
The steering `IMotorDriver` SHALL be implemented by a VESC driver that commands a Flipsky 75200 VESC in brushed-DC motor mode over a dedicated hardware UART using the VESC serial protocol.

#### Scenario: Map speed to duty command
- **WHEN** `setSpeed(speed)` is called on the VESC steering driver
- **THEN** it SHALL send `COMM_SET_DUTY` with `duty = constrain(speed / 255.0, -1.0, +1.0)`
- **AND** `setSpeed(0)` and `stop()` SHALL send zero duty (coast)

#### Scenario: Dedicated UART, isolated from MAVLink
- **WHEN** the VESC driver initializes
- **THEN** it SHALL use `UART_NUM_0` on `PIN_VESC_TX` (GPIO43) and `PIN_VESC_RX` (GPIO44) at `VESC_UART_BAUD`,
  passing both pins explicitly
- **AND** it SHALL NOT use `UART_NUM_1` (MAVLink)

#### Scenario: No log output on the VESC UART
- **WHEN** the firmware is running
- **THEN** no Arduino core, ROM `ets_printf` or ESP-IDF `esp_log` output SHALL be written to UART0
- **AND** `platformio.ini` SHALL pin `CORE_DEBUG_LEVEL=0`
- **AND** ESP-IDF `esp_log` output SHALL be routed to the USB-CDC console only

#### Scenario: Minimal in-repo protocol layer
- **WHEN** the driver frames or parses a VESC packet
- **THEN** it SHALL use an in-repo packet layer implementing VESC short/long framing with a CRC16 check
- **AND** it SHALL implement only `COMM_SET_DUTY` and `COMM_GET_VALUES`
