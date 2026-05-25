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
The system SHALL use the BTS7960 motor driver for the brake actuator only. Transmission
control has been moved to a PWM servo; transmission-specific encoder and position-control
methods are no longer part of `TransmissionController`.

#### Scenario: BTS7960 retained for brake actuator
- **WHEN** the brake actuator is commanded via `BTS7960Controller::setSpeed()`
- **THEN** RPWM/LPWM PWM outputs SHALL be set as specified in the existing brake scenarios
- **AND** transmission-related methods (`getPosition`, `moveToPosition`, `stopPositionControl`,
  `recalibrateEncoder`, `isPositionControlActive`, `autoHome`) are no longer called from
  `TransmissionController` or `VehicleController`

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
- **AND** system enters safe state requiring explicit reset

**Note**: Since BTS7960 enable pins are hardwired, drivers cannot be fully disabled. Motors will coast to a stop. Setting both RPWM and LPWM high would destroy the controller.

### Requirement: Hardware Configuration
The system SHALL define all hardware pin assignments in centralized configuration.

#### Scenario: Load pin configuration at startup
- **WHEN** system initializes
- **THEN** all pin assignments are loaded from Constants.h
- **AND** pin conflicts are checked and reported
- **AND** pins are validated against ESP32-C6 available GPIO

#### Scenario: Configure PWM parameters
- **WHEN** PWM peripherals are initialized
- **THEN** servo PWM frequency is set to 50Hz (20ms period)
- **AND** motor PWM frequency is set to 1-20kHz (configurable)
- **AND** PWM resolution is set to 12-16 bits based on frequency

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
The system SHALL drive the transmission gear selector through a PWM servo on GPIO 9 with a pulse range of 800–2200 µs, replacing the former BTS7960 linear actuator.

#### Scenario: Initialize transmission servo
- **WHEN** `TransmissionController::begin()` is called
- **THEN** `ServoController` SHALL be initialised with `PIN_TRANS_SERVO`, `LEDC_CH_TRANS_SERVO`, `TRANS_SERVO_MIN_US` (800), `TRANS_SERVO_MAX_US` (2200)
- **AND** the servo SHALL be commanded to the neutral default position (`TRANS_GEAR_DEFAULT_NEUTRAL_PCT`)

#### Scenario: Map gear percent to servo microseconds
- **WHEN** any gear position is commanded as a float percent
- **THEN** the mapping SHALL be `µs = TRANS_SERVO_MIN_US + pct / 100.0 * (TRANS_SERVO_MAX_US - TRANS_SERVO_MIN_US)`
- **AND** the result SHALL be clamped to `[TRANS_SERVO_MIN_US, TRANS_SERVO_MAX_US]`

#### Scenario: Overshoot-and-return motion profile
- **WHEN** `setGear(target)` is called
- **THEN** the servo SHALL first be commanded to `targetPct ± TRANS_GEAR_OVERSHOOT_PCT` (direction away from origin)
- **AND** after `TRANS_SERVO_SETTLE_MS` the servo SHALL be commanded to `targetPct`
- **AND** the physical gear switch for the target gear SHALL be checked after a second `TRANS_SERVO_SETTLE_MS`

#### Scenario: Confirm gear arrival via physical switch
- **WHEN** the return phase settle time has elapsed
- **AND** `getPhysicalGear()` returns the target gear
- **THEN** the gear change is marked complete (`gearChangePhase_ = NONE`)
- **AND** state is saved to NVS with `state_valid=true`

#### Scenario: Log warning on switch mismatch after settle
- **WHEN** the return phase settle time has elapsed
- **AND** `getPhysicalGear()` does NOT return the target gear after `2 × TRANS_SERVO_SETTLE_MS`
- **THEN** a warning SHALL be logged: "[TRANS] Gear mismatch after return"
- **AND** `gearChangePhase_` is reset to NONE to prevent indefinite retry

