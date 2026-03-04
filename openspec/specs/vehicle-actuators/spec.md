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
The system SHALL control linear actuators using BTS7960 dual H-bridge motor drivers with hardwired enable pins.

#### Scenario: Initialize BTS7960 driver
- **WHEN** BTS7960Controller.begin() is called with RPWM and LPWM pin configuration
- **THEN** GPIO pins are configured as outputs
- **AND** RPWM and LPWM pins are set to PWM mode
- **AND** both PWM outputs are set to 0 (stopped state)

**Note**: Enable pins (R_EN, L_EN) are hardwired to 5V power and always active.

#### Scenario: Set forward motion speed
- **WHEN** BTS7960Controller.setSpeed() is called with positive value (1-255)
- **THEN** RPWM duty cycle is set to requested speed
- **AND** LPWM is set to 0
- **AND** actuator extends at proportional speed

#### Scenario: Set reverse motion speed
- **WHEN** BTS7960Controller.setSpeed() is called with negative value (-255 to -1)
- **THEN** LPWM duty cycle is set to absolute speed value
- **AND** RPWM is set to 0
- **AND** actuator retracts at proportional speed

#### Scenario: Stop motor with coast
- **WHEN** BTS7960Controller.stop() is called
- **THEN** both RPWM and LPWM are set to 0
- **AND** actuator coasts to a stop (driver remains enabled via hardwired pins)

#### Scenario: Stop motor with brake command
- **WHEN** BTS7960Controller.brake() is called
- **THEN** both RPWM and LPWM are set to 0 (coast)
- **AND** actuator coasts to a stop

**CRITICAL SAFETY NOTE**: BTS7960 does NOT support electrical braking via simultaneous RPWM+LPWM high. Setting both high will short the H-bridge and destroy the controller. The brake() method is an alias for stop() (coast).

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

