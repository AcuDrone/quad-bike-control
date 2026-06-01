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
The transmission servo SHALL use an overshoot-dwell-return motion profile and SHALL enforce sequential gear shifting through the physical order `[R, N, H, L]`.

#### Scenario: Overshoot-and-dwell motion profile
- **WHEN** `setGear(target)` is called
- **THEN** the servo SHALL first move to the overshoot position (`targetPct ± overshootPct`)
- **AND** once servo movement time has elapsed, the controller SHALL enter an overshoot dwell of up to `TRANS_OVERSHOOT_DWELL_MS` milliseconds
- **AND** if the physical switch confirms the target gear during the dwell, the controller SHALL transition to the RETURN phase immediately (early exit)
- **AND** if `TRANS_OVERSHOOT_DWELL_MS` elapses and the switch IS confirmed, the controller SHALL transition to RETURN
- **AND** if `TRANS_OVERSHOOT_DWELL_MS` elapses and the switch is NOT confirmed, the controller SHALL transition to PULLBACK

#### Scenario: Return phase after confirmed gear engagement
- **WHEN** the overshoot dwell phase confirms gear engagement (switch active)
- **THEN** the servo SHALL move from the overshoot position back to `finalGearPositionPct_`
- **AND** once return movement time has elapsed, the gear change SHALL be marked complete
- **AND** `saveState()` SHALL be called with `state_valid=true`
- **AND** no additional switch check is performed in the RETURN phase

#### Scenario: Pullback dwell before retry
- **WHEN** overshoot dwell times out without switch confirmation
- **AND** `overshootRetryCount_ < 100`
- **THEN** the servo SHALL move away from the target by `pullbackPcts_[targetGear]`
- **AND** once pullback movement completes, the controller SHALL dwell for `TRANS_ROLLBACK_DWELL_MS` milliseconds
- **AND** after the dwell, `overshootRetryCount_` SHALL be incremented
- **AND** the controller SHALL retry OVERSHOOT with multiplier 1.5× (retry 1) or 2.0× (retry 2+)

#### Scenario: Confirm gear arrival via physical switch (updated)
- **WHEN** the overshoot dwell phase is active
- **AND** `getPhysicalGear()` returns the target gear
- **THEN** the RETURN phase SHALL start immediately without waiting for the full dwell period
- **AND** the gear change is considered confirmed

#### Scenario: Sequential gear shifting — step through sequence
- **WHEN** `setGear(finalGear)` is called
- **AND** `finalGear` is not adjacent to the current physical gear in the sequence `[R, N, H, L]`
- **THEN** the controller SHALL compute the immediate next step toward `finalGear`
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** the gear change SHALL start toward the immediate next step only

#### Scenario: Sequential gear shifting — advance queue on arrival
- **WHEN** a gear change completes (physical switch confirms)
- **AND** `queuedGear_ != GEAR_UNKNOWN`
- **AND** `physicalGear != queuedGear_`
- **THEN** the controller SHALL automatically start the next step toward `queuedGear_`
- **AND** steps SHALL continue until `physicalGear == queuedGear_`
- **AND** `queuedGear_` SHALL be cleared to `GEAR_UNKNOWN` when the final gear is reached

#### Scenario: Sequential shifting interrupted by new command
- **WHEN** `setGear(newFinalGear)` is called while a sequence is in progress
- **THEN** the in-flight gear change to the current intermediate target SHALL complete normally
- **AND** `queuedGear_` SHALL be updated to `newFinalGear`
- **AND** subsequent steps SHALL route toward `newFinalGear`

#### Scenario: Sequential shifting — unknown current gear
- **WHEN** `setGear(finalGear)` is called
- **AND** `getPhysicalGear()` returns `GEAR_UNKNOWN`
- **THEN** the controller SHALL route through `GEAR_NEUTRAL` first (safe intermediate)
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** once NEUTRAL is confirmed, normal sequencing toward `finalGear` resumes

