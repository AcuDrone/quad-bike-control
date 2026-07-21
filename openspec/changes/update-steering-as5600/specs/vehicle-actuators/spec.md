# vehicle-actuators Spec Delta

## MODIFIED Requirements

### Requirement: BTS7960 Motor Driver Control
The system SHALL use BTS7960 motor drivers for the brake actuator and the steering actuator. Both are driven through `BTS7960Controller` with LEDC PWM on RPWM/LPWM; transmission control remains on a PWM servo and transmission-specific encoder and position-control methods are no longer part of `TransmissionController`.

#### Scenario: BTS7960 for brake actuator
- **WHEN** the brake actuator is commanded via `BTS7960Controller::setSpeed()`
- **THEN** RPWM/LPWM PWM outputs SHALL be set as specified in the existing brake scenarios
- **AND** transmission-related methods are no longer called from `TransmissionController` or `VehicleController`

#### Scenario: BTS7960 for steering actuator with proportional speed
- **WHEN** the steering controller drives its BTS7960 via `BTS7960Controller::setSpeed()`
- **THEN** the drive duty SHALL be proportional to the remaining position error, clamped between the configured minimum and maximum duty
- **AND** positive speed drives right (RPWM), negative drives left (LPWM), zero coasts
- **AND** RPWM and LPWM SHALL never be driven high simultaneously
