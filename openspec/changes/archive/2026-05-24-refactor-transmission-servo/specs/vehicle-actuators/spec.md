## ADDED Requirements

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

## MODIFIED Requirements

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
