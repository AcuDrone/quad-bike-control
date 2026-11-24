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
  - `PIN_BOOST_EN` = GPIO46, `PIN_ADC_24V` = GPIO9 (X10; aztec has no boost board — GPIO9 is the
    12 V battery sense on X10 pin 3 and GPIO46 is driven HIGH with nothing attached)
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

#### Scenario: Steering sensors share the pins of their buses
- **WHEN** the steering controller is initialized
- **THEN** the Bosch LWS steering-angle sensor SHALL share the CAN bus (MCP2515 on SPI,
  `PIN_CAN_*`) with the ECU OBD-II traffic
- **AND** no dedicated steering GPIO SHALL be allocated
- **AND** the former steering I2C pins (`Wire` / `Wire1`) SHALL NOT be used for steering

#### Scenario: Wiring document is authoritative
- **WHEN** `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` disagree about a pin
- **THEN** the document SHALL be treated as authoritative for what the board is physically wired to
- **AND** the divergence SHALL be resolved before the firmware is run on the vehicle
