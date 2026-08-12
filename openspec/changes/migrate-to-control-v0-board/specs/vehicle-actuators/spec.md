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
- **AND** GPIO43/44 (UART0) SHALL NOT be assigned any firmware function — they are reserved for the
  future CH9121T wired LAN
- **AND** pin conflicts are checked and reported

#### Scenario: Control_v0 pin assignments
- **WHEN** `Constants.h` is read
- **THEN** the following assignments SHALL be in effect:
  - `PIN_THROTTLE_PWM` = GPIO15 (LEDC channel 1, 50 Hz, X8 pin 2)
  - `PIN_TRANS_SERVO` = GPIO16 (LEDC channel 2, 50 Hz, X8 pin 3)
  - `PIN_MAVLINK_TX` = GPIO17, `PIN_MAVLINK_RX` = GPIO18 (UART1, X9)
  - `PIN_VESC_TX` = GPIO4, `PIN_VESC_RX` = GPIO5 (UART2, X6) — unchanged
  - `PIN_BRAKE_LPWM` = GPIO6, `PIN_BRAKE_RPWM` = GPIO7 (LEDC channels 5/4, X7) — unchanged
  - `PIN_CAN_CS` = GPIO39, `PIN_CAN_SCK` = GPIO40, `PIN_CAN_MISO` = GPIO41, `PIN_CAN_MOSI` = GPIO42
  - `PIN_STEER_SDA` = GPIO1, `PIN_STEER_SCL` = GPIO2 (I2C1, X14 + input expander)
  - `PIN_RELAY_SDA` = GPIO48, `PIN_RELAY_SCL` = GPIO47 (I2C2, X15)
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

#### Scenario: Wiring document is authoritative
- **WHEN** `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` disagree about a pin
- **THEN** the document SHALL be treated as authoritative for what the board is physically wired to
- **AND** the divergence SHALL be resolved before the firmware is run on the vehicle

### Requirement: BTS7960 Motor Driver Control
The system SHALL use a BTS7960 motor driver for the brake actuator, driven through `BTS7960Controller` with LEDC PWM on RPWM/LPWM. The steering actuator SHALL NOT use `BTS7960Controller`; it is driven by a VESC over UART (see "VESC Brushed-DC Steering Driver"). Transmission control remains on a PWM servo.

#### Scenario: BTS7960 for brake actuator
- **WHEN** the brake actuator is commanded via `BTS7960Controller::setSpeed()`
- **THEN** RPWM/LPWM PWM outputs SHALL be set as specified in the existing brake scenarios
- **AND** RPWM and LPWM SHALL never be driven high simultaneously
- **AND** the outputs SHALL be `PIN_BRAKE_LPWM` (GPIO6) and `PIN_BRAKE_RPWM` (GPIO7) routed to
  connector X7

#### Scenario: Steering no longer uses BTS7960
- **WHEN** the steering actuator is commanded
- **THEN** the command SHALL be issued through the `IMotorDriver` abstraction backed by the VESC UART driver
- **AND** no `BTS7960Controller` instance SHALL be associated with the steering actuator

#### Scenario: Former steering PWM pins are reallocated, not reserved
- **WHEN** the pin map is read
- **THEN** GPIO17 and GPIO18 SHALL be allocated to the MAVLink UART1 (TX and RX respectively)
- **AND** the former steering PWM constants and their LEDC channel constants SHALL be deleted rather
  than kept as reserved placeholders
- **AND** LEDC channels 6 and 7 SHALL be available for future use with no reservation
