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
- **AND** once servo movement time has elapsed, the controller SHALL enter the overshoot dwell
- **AND** the dwell SHALL end as soon as the physical gear has read the step's target gear continuously for `TRANS_DWELL_CONFIRM_MS` (200 ms), measured from the first confirming read
- **AND** if that does not happen, the dwell SHALL end after `TRANS_OVERSHOOT_DWELL_MS` (2000 ms, the timeout) and the gear SHALL be assumed engaged regardless of physical sensor state
- **AND** in both cases the controller SHALL transition directly to the RETURN phase

#### Scenario: Return phase after assumed gear engagement
- **WHEN** the overshoot dwell ends (sensor-confirmed or timed out)
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
- **THEN** the controller SHALL pause, then automatically start the next step toward `queuedGear_` using `targetGear_` as the current position
- **AND** the pause SHALL end as soon as the physical gear has read `targetGear_` continuously for `TRANS_DWELL_CONFIRM_MS`, or after `TRANS_SEQUENCE_STEP_DWELL_MS` (1000 ms, the timeout)
- **AND** steps SHALL continue until `targetGear_ == queuedGear_`
- **AND** `queuedGear_` SHALL be cleared to `GEAR_UNKNOWN` when the final gear is reached

#### Scenario: Sequential shifting interrupted by new command
- **WHEN** `setGear(newFinalGear)` is called while a sequence is in progress
- **THEN** the in-flight gear change to the current intermediate target SHALL complete normally
- **AND** `queuedGear_` SHALL be updated to `newFinalGear`
- **AND** subsequent steps SHALL route toward `newFinalGear`

#### Scenario: Sequential shifting — unknown current gear
- **WHEN** `setGear(finalGear)` is called
- **AND** `targetGear_` is `GEAR_UNKNOWN` (the transmission has not been initialised from the sensor or NVS)
- **THEN** `setGear()` SHALL return false
- **AND** the servo SHALL NOT be commanded, NVS SHALL NOT be written and `queuedGear_` SHALL be unchanged
- **AND** there SHALL be no routing through `GEAR_NEUTRAL` from an unknown gear

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

### Requirement: VESC UART Receive Resynchronization
The VESC UART packet parser SHALL never stay misaligned for longer than one reply period. It SHALL
live in a header-only `include/VescRxParser.h` so it can be tested on the host, SHALL reset to the
start-byte search when more than `VESC_RX_GAP_MS` (5 ms) have passed since the previous received byte
while a frame is incomplete, and SHALL keep link counters for the event journal and telemetry. These
rules are additions to the existing framing, CRC and timeout behavior, which stays unchanged.

#### Scenario: Gap ends an incomplete frame
- **WHEN** the parser is in any state other than the start-byte search
- **AND** a byte arrives more than 5 ms after the previous byte
- **THEN** the parser SHALL return to the start-byte search before handling that byte
- **AND** the `resync` counter SHALL be incremented and `VESC W resync` journaled (deduplicated)

#### Scenario: One lost byte costs one reply
- **WHEN** one byte of a `COMM_GET_VALUES` reply is lost on the wire
- **AND** the following replies arrive 300 ms apart and contain no `0x02`/`0x03` data bytes
- **THEN** the reply with the lost byte SHALL be dropped
- **AND** the next reply SHALL be decoded
- **AND** a host test SHALL show that the same stream without the gap rule decodes none of 10
  following replies

#### Scenario: Link-lost transition restarts the receiver
- **WHEN** `driverOk()` changes from true to false
- **THEN** the driver SHALL discard the bytes waiting in the UART receive buffer and reset the parser

#### Scenario: Link counters
- **WHEN** the driver runs
- **THEN** it SHALL count received bytes, good packets, packets dropped for a CRC mismatch, packets
  dropped for a wrong stop byte, resyncs and the bytes since the last good packet
- **AND** expose them through `IMotorDriver` (all zero for drivers without a packet link)

#### Scenario: Loop stall in the middle of a reply
- **WHEN** the main loop reads part of a reply, then stalls for more than 5 ms before reading the rest
- **THEN** at most that one reply SHALL be lost and the next reply SHALL be decoded

### Requirement: VESC UART RX Idle Pull-Up
The VESC steering driver SHALL enable the ESP32 internal pull-up (and no pull-down) on its UART RX
pin (`PIN_VESC_RX`; GPIO44 on the Prog header X1 on aztec, which has only a 510 Ω series resistor
and no external pull-up) so the line idles high between bytes and while the VESC is unpowered or
unplugged. The pull-up SHALL be applied after the UART driver has claimed the pin, because the core's
pin assignment does not enable one, and SHALL be verified by reading the pad configuration back.

#### Scenario: Pull-up applied at UART start
- **WHEN** `VescMotorDriver::begin()` opens the UART
- **THEN** the internal pull-up SHALL be enabled on the RX pin after `HardwareSerial::begin()`
- **AND** the USB console line `[VESC] UART… started` SHALL report `pull-up on`

#### Scenario: Pull-up did not take
- **WHEN** the read-back pad configuration shows no pull-up (or a pull-down)
- **THEN** `VESC W rx_pullup pin=<n> ok=0` SHALL be journaled
- **AND** the UART SHALL still be used (structural success, as before)

#### Scenario: Link recovery keeps the pull-up
- **WHEN** the VESC link is lost and recovered
- **THEN** the UART SHALL NOT be re-opened (only the RX buffer is flushed and the parser reset), so the
  pull-up set at start SHALL remain in effect

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
The closed-loop steering controller SHALL have three states: idle, driving and parked. It SHALL stop driving and park when the absolute position error is at most `STEER_POSITION_TOLERANCE` (2 = 0.2° on the quad) and SHALL NOT drive the closed loop again until the absolute error exceeds `STEER_RESTART_TOLERANCE` (10 = 1.0°). Parking SHALL be a coast through `IMotorDriver::stop()` (zero duty); the self-locking steering worm gear holds the shaft, and the controller SHALL send no further closed-loop motor commands while parked; the only exception SHALL be the bounded park correction, whose trigger SHALL be the larger of `STEER_PARK_TRIGGER_TOL` (4 = 0.4°) and its done tolerance, independent of `STEER_POSITION_TOLERANCE`. Every stop path SHALL be a coast.

#### Scenario: Park at the target
- **WHEN** the closed loop is driving and the absolute error becomes at most `STEER_POSITION_TOLERANCE`
- **THEN** the controller SHALL call `IMotorDriver::stop()` (coast) and enter the parked state
- **AND** it SHALL NOT send closed-loop motor commands while it stays parked

#### Scenario: Parks within 0.2°
- **WHEN** the closed loop is driving toward a target from either side
- **AND** the absolute error becomes 0.2°
- **THEN** the controller SHALL park
- **WHEN** the absolute error is 0.3°
- **THEN** the controller SHALL keep driving toward the target

#### Scenario: No restart inside the hysteresis band
- **WHEN** the controller is parked
- **AND** the absolute error grows above `STEER_POSITION_TOLERANCE` but stays at most `STEER_RESTART_TOLERANCE` (1.0°)
- **THEN** the controller SHALL stay parked and the closed loop SHALL NOT issue a drive command

#### Scenario: Restart beyond the hysteresis band
- **WHEN** the controller is parked and the absolute error exceeds `STEER_RESTART_TOLERANCE` (1.0°)
- **THEN** it SHALL start driving toward the current target as a start from rest

#### Scenario: Park correction trigger unchanged
- **WHEN** the controller is parked with an absolute error of 0.3°
- **AND** the park correction is enabled with its default done tolerance (0.2°)
- **THEN** the park correction SHALL NOT start, because 0.3° does not exceed `STEER_PARK_TRIGGER_TOL` (0.4°)
- **AND** an error above 0.4° SHALL start it as before

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
- **AND** it SHALL be driven and parked by the same rules as a MAVLink target, so a target within `STEER_RESTART_TOLERANCE` of a parked wheel SHALL NOT drive the closed loop

### Requirement: Steering Break-away and Tracking Duty Floors
While driving, the closed-loop steering duty SHALL be `clamp(max(|error| × STEER_KP, floor) + dutyBoost_, floor, STEER_PWM_MAX_DUTY)`, signed toward the target, so that the anti-stall boost always adds on top of the floor in force and is never hidden under it. The floor SHALL be the break-away floor `STEER_PWM_MIN_DUTY` (85 on the quad) from a start from rest or a drive direction flip until the wheel has moved at least `STEER_BOOST_MIN_PROGRESS` (0.3°) from the position where that drive or reversal started, and the tracking floor `STEER_PWM_TRACK_MIN_DUTY` (85 on the quad, bench-tune) after that; `STEER_PWM_MIN_DUTY` SHALL NOT be below `STEER_PWM_TRACK_MIN_DUTY`. Every `STEER_BOOST_CHECK_MS` (100 ms) the boost SHALL rise by `STEER_BOOST_STEP` (30) when the wheel progressed less than `STEER_BOOST_MIN_PROGRESS` since the previous check, and, when it progressed at least that much, SHALL be held unchanged while `|error|` exceeds the hold band `STEER_BOOST_HOLD_ERR` (2.0°) and SHALL fall by `STEER_BOOST_DECAY` (30 on the quad), never below 0, inside the hold band. The boost SHALL be reset to 0 on a start from rest and on a drive direction flip. The duty law SHALL be a pure, host-tested function (`include/SteerPositionLaw.h`). The sensorless rate drive SHALL keep `STEER_PWM_MIN_DUTY` as its floor.

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
- **AND** the boost SHALL be reset to 0

#### Scenario: Stalled wheel gets more duty on the first check
- **WHEN** the closed loop is driving with `|error| × STEER_KP` below the floor in force and the boost at 0
- **AND** a boost check finds less than `STEER_BOOST_MIN_PROGRESS` of progress
- **THEN** the boost SHALL become `STEER_BOOST_STEP` (30)
- **AND** the duty SHALL be the floor plus 30 (115 at a floor of 85), not the floor

#### Scenario: Boost decays gently
- **WHEN** the boost is 30, `|error|` is within `STEER_BOOST_HOLD_ERR` (2.0°) and the following boost checks each find at least `STEER_BOOST_MIN_PROGRESS` of progress
- **THEN** each check SHALL lower the boost by `STEER_BOOST_DECAY`, never below 0 (30 → 0 after one check at the quad's 30; 30 → 20 → 10 → 0 at a decay of 10)
- **AND** it SHALL stay at 0 on further moving checks
- **AND** a check that finds the wheel stalled again SHALL add `STEER_BOOST_STEP` to the current boost

#### Scenario: Boost is held while far from the target
- **WHEN** the closed loop is driving with a non-zero boost and `|error|` greater than `STEER_BOOST_HOLD_ERR` (2.0°)
- **AND** a boost check finds at least `STEER_BOOST_MIN_PROGRESS` of progress
- **THEN** the boost SHALL stay unchanged (no decay)
- **AND** once `|error|` is within `STEER_BOOST_HOLD_ERR` the boost SHALL fall by `STEER_BOOST_DECAY` per moving check
- **AND** a check that finds the wheel stalled SHALL add `STEER_BOOST_STEP` both outside and inside the hold band

#### Scenario: Far from the target the P term rules
- **WHEN** the closed loop is driving and `|error| × STEER_KP` exceeds the floor in force
- **THEN** the duty SHALL be `|error| × STEER_KP + dutyBoost_`, clamped to `STEER_PWM_MAX_DUTY`

#### Scenario: Stall at the tracking floor
- **WHEN** the closed loop is driving at the tracking floor and the wheel stops making progress
- **THEN** the anti-stall boost SHALL raise the duty above the floor by `STEER_BOOST_STEP` per `STEER_BOOST_CHECK_MS`
- **AND** if the position still changes by less than `STEER_STALL_THRESHOLD` over `STEER_STALL_TIMEOUT`, a stall-stop SHALL occur

### Requirement: Steering Diagnostics Ownership Lock
The steering controller SHALL provide a diagnostics lock for the USB steering diagnostics console. While the lock is set, `setSteeringPercent()` (MAVLink, web slider, failsafe centre, boot centring), `setRateCommand()`, `jog()`, `nudge()`, the limit captures and the sensor zero-set SHALL be refused, and only the diagnostics console SHALL drive the motor: open loop through `diagDrive()` (no duty floor, no stall latch, no refresh timeout) or one closed-loop move through `diagGoto()`, which SHALL use the same target clamping, gates and position law as `setSteeringPercent()`. Stop paths and the sensor / VESC monitors SHALL NOT be gated. Setting or clearing the lock SHALL stop the motor (coast) and cancel any move, jog or pending boot centring.

#### Scenario: External steering refused while locked
- **WHEN** the diagnostics lock is set
- **AND** a MAVLink steering command, a web steering / jog / nudge / capture / zero command or the failsafe centre arrives
- **THEN** the controller SHALL NOT move the motor for it
- **AND** a web command SHALL be answered with a failure naming the USB diagnostics console

#### Scenario: Lock changes stop the motor
- **WHEN** the lock is set or cleared
- **THEN** the motor SHALL be coasted and the closed loop and any jog SHALL be idle

#### Scenario: Monitors still stop the motor
- **WHEN** the lock is set and the VESC link drops or reports a fault
- **THEN** the controller SHALL stop the motor as without the lock

#### Scenario: Normal behaviour without the lock
- **WHEN** the lock is not set
- **THEN** every steering command path SHALL behave as before this change

### Requirement: VESC Telemetry Poll Period
The VESC steering driver SHALL poll COMM_GET_VALUES every `STEER_VESC_TELEM_MS` (300 ms) and SHALL allow the period to be changed at run time; the USB diagnostics console SHALL set it to `STEER_DIAG_VESC_TELEM_MS` (50 ms) while it owns the steering and restore it afterwards. The driver SHALL report the last commanded effort.

#### Scenario: Fast telemetry in diagnostics
- **WHEN** `diag on` is accepted
- **THEN** GET_VALUES SHALL be requested every 50 ms until `diag off`, then every 300 ms again

### Requirement: Steering Park Correction
While the closed-loop steering is PARKED in position mode, the steering controller SHALL finish a parking that ended outside the park window with a bounded series of short open-loop pulses toward the current target. The correction SHALL be the only motor command a parked loop sends, and SHALL be limited to the configured maximum number of pulses per parking. It SHALL start only when the correction is enabled, the USB diagnostics lock is not set, the loop has been parked and the target has stayed within `STEER_PARK_TARGET_STABLE` (3 = 0.3°) of its reference for at least `STEER_PARK_SETTLE_MS` (1000 ms), and the absolute error exceeds the larger of `STEER_POSITION_TOLERANCE` (0.4°) and the done tolerance. Positions and tolerances SHALL be in 0.1° units. The decision logic SHALL be a pure, host-tested law (`include/SteerParkLaw.h`). With the correction disabled the steering SHALL behave exactly as without this requirement.

#### Scenario: Correction starts after the settle time
- **WHEN** the loop is PARKED with the target unchanged (within 0.3°) and parked for at least 1000 ms
- **AND** the absolute error exceeds the larger of 0.4° and the done tolerance
- **THEN** the controller SHALL send one pulse at rung 1 (duty × ms) toward the target with `IMotorDriver::setSpeed()`, without the `STEER_PWM_MIN_DUTY` floor, and SHALL coast with `IMotorDriver::stop()` after the rung's length
- **AND** it SHALL work relative to the current target, whatever its value, including the exact 0° that a centred RC stick produces

#### Scenario: No correction before settling or inside the window
- **WHEN** the target moved by more than 0.3° or the loop entered PARKED less than 1000 ms ago, or the absolute error is within the trigger
- **THEN** no pulse SHALL be sent

#### Scenario: Pulse evaluation and ladder
- **WHEN** a pulse has ended and `STEER_PARK_PULSE_GAP_MS` (500 ms) has passed since its end
- **THEN** the controller SHALL compare the position with the position before the pulse and, in this order:
- **AND** if the absolute error is at most the done tolerance, it SHALL finish (done)
- **AND** else if the error changed sign (overshoot beyond the done tolerance), it SHALL log the overshoot and send the next pulse at rung 1 in the new direction of the error
- **AND** else if the wheel moved away from the target by at least `STEER_PARK_MOVE_MIN` (0.1°), it SHALL log it and send the next pulse at rung 1 toward the target
- **AND** else if the wheel moved toward the target by at least 0.1°, the next pulse SHALL use the same rung
- **AND** otherwise the next pulse SHALL use the next rung, staying at rung 3 once there
- **AND** every pulse, including one after an overshoot, SHALL count toward the maximum
- **AND** once started, the correction SHALL continue until the done tolerance, not the 0.4° trigger

#### Scenario: Give up after the maximum pulses
- **WHEN** the maximum number of pulses has been sent and the evaluation after the last one is not done
- **THEN** the controller SHALL stop correcting and log `park correction gave up at X°`
- **AND** it SHALL NOT pulse again until the settle clock is reset by a target move beyond 0.3° or a new entry into PARKED (a main-loop restart beyond `STEER_RESTART_TOLERANCE` or a command from idle)

#### Scenario: Budget per parking
- **WHEN** a correction finished done and the error later exceeds the trigger again without a target move or a new park entry
- **THEN** the controller SHALL resume from rung 1 with only the remaining pulses of the same budget

#### Scenario: Cancel at once
- **WHEN** a correction is armed or running
- **AND** the target moves by more than 0.3°, or the loop leaves PARKED for any reason (main-loop restart beyond `STEER_RESTART_TOLERANCE`, `stop()`, sensor fault, VESC fault or link loss, stall-stop), or a jog, nudge, rate drive, limit capture or sensor zero-set starts, or the USB diagnostics lock is set, or the correction is disabled or a park setting is changed
- **THEN** a running pulse SHALL be ended with a coast at once and the correction state SHALL reset
- **AND** the normal position loop SHALL keep priority: an error beyond `STEER_RESTART_TOLERANCE` restarts the drive as a start from rest

#### Scenario: Stall latch respected
- **WHEN** a pulse would push in a direction blocked by the stall latch within `STEER_STALL_COOLDOWN_MS`
- **THEN** the pulse SHALL NOT be sent and SHALL NOT count, and the correction SHALL wait for the cooldown

#### Scenario: Debug log lines
- **WHEN** a pulse is evaluated, an overshoot or wrong-way move is seen, or the correction finishes or gives up
- **THEN** the SERVO debug feature SHALL print lines of the form `[STEER] park nudge 85x90 -> moved 0.15 deg (err 0.35)`, `[STEER] park overshoot -> rung 1, reversing (err -0.30)`, `[STEER] park nudge moved the wrong way -> rung 1 (err 0.70)`, `[STEER] park correction done at 0.10 deg (3 pulses)` and `[STEER] park correction gave up at 0.60 deg (5 pulses)`

#### Scenario: Closed-loop state unchanged
- **WHEN** a correction runs
- **THEN** the closed-loop state SHALL stay PARKED (token `park`), and no jog SHALL be active

### Requirement: Steering Park Correction Settings
The park correction SHALL be configured per vehicle by persisted settings in the existing NVS namespace `"steering"`, owned by the steering controller: enabled (`park_en`, default true), three ladder rungs of duty (`park_d1`..`park_d3`, 40–150, defaults 70 / 85 / 100) and length (`park_t1`..`park_t3`, 40–200 ms, defaults 80 / 90 / 100), the done tolerance (`park_tol`, 1–5 in 0.1°, default 2 = 0.2°) and the maximum pulses per parking (`park_max`, 1–10, default 5). The ladder SHALL be non-decreasing in both duty and length. The settle time (1000 ms), the target-stable band (0.3°), the pulse gap (500 ms) and the move threshold (0.1°) SHALL be compile-time constants only. The namespace SHALL never be cleared; the calibration migration and the sensor zero-set SHALL NOT remove the park keys.

#### Scenario: Defaults on a unit without the keys
- **WHEN** the firmware boots and a park key is absent
- **THEN** that setting SHALL use its default, and NVS SHALL NOT be written

#### Scenario: Invalid stored value
- **WHEN** a stored park value is out of range, or the stored ladder is not non-decreasing
- **THEN** the value (for the ladder: the whole ladder) SHALL be logged and replaced by the default in RAM, and NVS SHALL NOT be rewritten

#### Scenario: Change persisted first
- **WHEN** a valid setting change is requested
- **THEN** it SHALL be written to NVS first and applied only after the write succeeds, and any running correction SHALL be cancelled
- **AND** on a write failure the RAM value SHALL stay unchanged

#### Scenario: Decreasing rung refused
- **WHEN** a rung change would make its duty or length lower than the previous rung's or higher than the next rung's
- **THEN** it SHALL be refused and nothing SHALL be stored

