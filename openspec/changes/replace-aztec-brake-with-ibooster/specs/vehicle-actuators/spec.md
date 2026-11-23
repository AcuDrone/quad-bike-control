## MODIFIED Requirements
### Requirement: BTS7960 Motor Driver Control
On the aztec buggy the system SHALL use one BTS7960 motor driver, driven through `BTS7960Controller`
with LEDC PWM on RPWM/LPWM: the parking-brake actuator on X6. It SHALL configure the shared motor
timer (`LEDC_TIMER_1`, `MOTOR_PWM_FREQ` = 10 kHz, 8-bit) itself, being the only instance. The service
brake SHALL NOT use a BTS7960: it is the Bosch iBooster on CAN (see "Bosch iBooster Gen2 Service
Brake Unit"). The steering actuator SHALL NOT use `BTS7960Controller`; it is driven by a VESC over
UART (see "VESC Brushed-DC Steering Driver"). Transmission control remains on a PWM servo.

#### Scenario: BTS7960 for the parking-brake actuator (aztec)
- **WHEN** the parking-brake actuator is commanded via its `BTS7960Controller` instance
- **THEN** the outputs SHALL be `PIN_PARK_BRAKE_LPWM` (GPIO4, LEDC channel 7) and
  `PIN_PARK_BRAKE_RPWM` (GPIO5, LEDC channel 6), routed to connector X6 pins 2 and 3
- **AND** RPWM and LPWM SHALL never be driven high simultaneously
- **AND** `begin()` SHALL configure the motor timer (10 kHz, 8-bit) and leave both outputs at zero
  duty

#### Scenario: BTS7960 for brake actuator
- **WHEN** the firmware is built for the aztec buggy
- **THEN** no `BTS7960Controller` instance SHALL exist for the service brake
- **AND** `PIN_BRAKE_RPWM`, `PIN_BRAKE_LPWM`, `LEDC_CH_BRAKE_RPWM` and `LEDC_CH_BRAKE_LPWM` SHALL
  NOT be defined, and no reference to them SHALL remain in `src/` or `include/`

#### Scenario: Steering no longer uses BTS7960
- **WHEN** the steering actuator is commanded
- **THEN** the command SHALL be issued through the `IMotorDriver` abstraction backed by the VESC UART driver
- **AND** no `BTS7960Controller` instance SHALL be associated with the steering actuator

#### Scenario: Former steering PWM pins are reallocated, not reserved
- **WHEN** the pin map is read
- **THEN** GPIO17 and GPIO18 SHALL be allocated to the MAVLink UART1 (TX and RX respectively)
- **AND** the former steering PWM constants and their LEDC channel constants SHALL be deleted rather
  than kept as reserved placeholders
- **AND** LEDC channels 6 and 7 SHALL be allocated to the aztec parking-brake actuator
  (`LEDC_CH_PARK_BRAKE_RPWM` = 6, `LEDC_CH_PARK_BRAKE_LPWM` = 7)

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
  (the aztec board has R63/R64 to the CH9121T removed); GPIO4/5 (X6) SHALL be assigned to the aztec
  parking-brake BTS7960 (L_PWM / R_PWM)
- **AND** pin conflicts are checked and reported

#### Scenario: Control_v0 pin assignments
- **WHEN** `Constants.h` is read
- **THEN** the following assignments SHALL be in effect:
  - `PIN_THROTTLE_PWM` = GPIO15 (LEDC channel 1, 50 Hz, X8 pin 2)
  - `PIN_TRANS_SERVO` = GPIO16 (LEDC channel 2, 50 Hz, X8 pin 3)
  - `PIN_MAVLINK_TX` = GPIO17, `PIN_MAVLINK_RX` = GPIO18 (UART1, X9)
  - `PIN_VESC_TX` = GPIO43, `PIN_VESC_RX` = GPIO44 (UART0, Prog header X1)
  - `PIN_PARK_BRAKE_LPWM` = GPIO4, `PIN_PARK_BRAKE_RPWM` = GPIO5 (LEDC channels 7/6, 10 kHz, X6 pins
    2/3 — aztec parking brake)
  - `PIN_CAN_CS` = GPIO39, `PIN_CAN_SCK` = GPIO40, `PIN_CAN_MISO` = GPIO41, `PIN_CAN_MOSI` = GPIO42,
    `PIN_CAN_INT` = GPIO38 (MCP2515 — engine ECU, Bosch LWS steering-angle sensor and Bosch iBooster
    on the same bus; INT wakes the CAN task)
  - `PIN_RELAY_SDA` = GPIO48, `PIN_RELAY_SCL` = GPIO47 (I2C2 — on-board expanders U10/U2 and the
    relay board `0x1F` on header X15)
  - `PIN_BOOST_EN` = GPIO46, `PIN_ADC_24V` = GPIO9 (X10; aztec has no boost board — GPIO9 is the
    12 V battery sense on X10 pin 3 and GPIO46 is driven HIGH with nothing attached)
  - `PIN_I2C1_SDA` = GPIO1, `PIN_I2C1_SCL` = GPIO2 (I2C1 — external header X14, no device fitted) and
    `PIN_ADC_IS_MUX` = GPIO3 SHALL be defined but unused (the X6 IS lines on the CD4051 mux are not
    sampled)
- **AND** GPIO6 and GPIO7 (X7, the former service-brake BTS7960) SHALL have no firmware function and
  SHALL be left unconfigured

#### Scenario: Functions with no GPIO
- **WHEN** `Constants.h` is read
- **THEN** `PIN_GEAR_REVERSE`, `PIN_GEAR_NEUTRAL`, `PIN_GEAR_LOW`, `PIN_GEAR_HIGH` and
  `PIN_BRAKE_SENSOR` SHALL NOT be defined — the gear inputs are read from the opto-input expander and
  there is no brake endstop
- **AND** `PIN_RELAY1`, `PIN_RELAY2`, `PIN_RELAY3` and `PIN_WHEEL_LOCK` SHALL NOT be defined — those
  outputs are driven by the relay-board expander
- **AND** `PIN_STEER_RPWM`, `PIN_STEER_LPWM`, `LEDC_CH_STEER_RPWM` and `LEDC_CH_STEER_LPWM` SHALL NOT
  be defined — GPIO17/18 are now the MAVLink UART
- **AND** `PIN_STEER_SDA` and `PIN_STEER_SCL` SHALL NOT be defined — the steering sensor is on CAN
- **AND** `PIN_BRAKE_RPWM`, `PIN_BRAKE_LPWM`, `LEDC_CH_BRAKE_RPWM`, `LEDC_CH_BRAKE_LPWM` and
  `BOARD_IN_BIT_BRAKE_LIMIT` SHALL NOT be defined — the service brake is on CAN
- **AND** no reference to any of these symbols SHALL remain in `src/` or `include/`

#### Scenario: Configure PWM parameters
- **WHEN** PWM peripherals are initialized
- **THEN** servo PWM frequency is set to 50Hz (20ms period)
- **AND** motor PWM frequency is set to 1-20kHz (configurable)
- **AND** PWM resolution is set to 12-16 bits based on frequency
- **AND** LEDC channels 1 (throttle), 2 (transmission), 6 (parking brake RPWM) and 7 (parking brake
  LPWM) SHALL be the only channels allocated; channels 0, 3, 4 and 5 SHALL be free

#### Scenario: Steering sensor needs no GPIO
- **WHEN** the steering controller is initialized
- **THEN** the steering angle SHALL be received through the existing MCP2515 CAN controller
- **AND** no GPIO and no I2C bus SHALL be allocated to steering position feedback

#### Scenario: Steering sensors share the pins of their buses
- **WHEN** the steering controller is initialized
- **THEN** the Bosch LWS steering-angle sensor SHALL share the CAN bus (MCP2515 on SPI,
  `PIN_CAN_*`) with the ECU OBD-II traffic and the iBooster
- **AND** no dedicated steering GPIO SHALL be allocated
- **AND** the former steering I2C pins (`Wire` / `Wire1`) SHALL NOT be used for steering

#### Scenario: Service brake needs no GPIO
- **WHEN** the service brake is initialized
- **THEN** it SHALL use only the MCP2515 CAN controller and the ignition relay that wakes it
- **AND** no GPIO, LEDC channel or opto input SHALL be allocated to it

#### Scenario: Wiring document is authoritative
- **WHEN** `Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` disagree about a pin
- **THEN** the document SHALL be treated as authoritative for what the board is physically wired to
- **AND** the divergence SHALL be resolved before the firmware is run on the vehicle

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

#### Scenario: Validate iBooster flow
- **WHEN** a flow is computed for the iBooster
- **THEN** the flow word SHALL be clamped to `0x6A00…0x9200` and the rod target to 16.0 mm

#### Scenario: Emergency stop all actuators
- **WHEN** emergency stop function is called
- **THEN** all servo PWM outputs are disabled
- **AND** all BTS7960 drivers are set to coast (both PWM = 0)
- **AND** the steering VESC driver is commanded to zero duty (coast)
- **AND** system enters safe state requiring explicit reset

**Note**: BTS7960 enable pins are hardwired, so the parking-brake driver cannot be fully disabled and coasts to a stop. The steering VESC releases the motor (coast) when commanded to zero duty and also releases automatically on its own configured comm timeout if commands stop. The iBooster releases an external brake request by itself about 130–180 ms after our frames stop.

## ADDED Requirements
### Requirement: Bosch iBooster Gen2 Service Brake Unit
The aztec service brake SHALL be a Bosch iBooster Gen2 (Tesla Model 3/Y, P/N 1044671-00-x) on
the master cylinder, wired as follows: X150 pin 1 (power) directly to the 12 V battery with 4 mm²
wire, without a diode or converter; pin 9 to ground common with the control board; pin 20 (Wake) to
the ACC / ignition line switched by Relay_1; pin 18 CAN-H and pin 10 CAN-L to the board's CAN bus
(X13). The pedal-travel sensor connector (X158) SHALL stay connected. Pins 25 / 16 (the unit's second
bus) SHALL NOT be connected. No new relay SHALL be added; Relay_7/8 stay reserved.

#### Scenario: Power path can absorb regeneration
- **WHEN** the rod returns quickly and the unit feeds energy back into its supply
- **THEN** the battery SHALL absorb it, because pin 1 has no diode or converter between it and the
  battery

#### Scenario: Wake follows the ignition
- **WHEN** Relay_1 is on (ACC, IGNITION or CRANKING)
- **THEN** the unit SHALL be awake
- **AND** when Relay_1 is off the unit SHALL sleep and the firmware SHALL NOT be able to wake it

#### Scenario: Bus termination
- **WHEN** the bus is assembled with the board, the engine ECU, the LWS and the iBooster
- **THEN** the resistance between CAN-H and CAN-L with the power off SHALL be about 60 Ω (exactly two
  terminators), measured before the first power-up and recorded in `GPIO_PINOUT_CUSTOM_BOARD_S3.md`
