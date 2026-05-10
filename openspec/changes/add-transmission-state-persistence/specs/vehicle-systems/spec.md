## ADDED Requirements

### Requirement: Transmission State Persistence
The system SHALL persist the last confirmed transmission state (gear, encoder position, validity flag) to NVS so that autohome can be skipped after a clean power cycle.

#### Scenario: Mark state invalid at start of gear change
- **WHEN** `TransmissionController::setGear()` is called
- **THEN** `state_valid=false` SHALL be written to NVS before movement begins
- **AND** a mid-move power loss will therefore result in `state_valid=false` on next boot

#### Scenario: Save confirmed state on gear arrival
- **WHEN** the physical gear switch confirms the target gear is reached
- **THEN** `state_valid=true`, `state_gear`, and `state_pos` (current encoder count) SHALL be written to NVS
- **AND** the saved state reflects the physical actuator position at that moment

#### Scenario: Restore encoder and skip autohome on valid state
- **WHEN** the system starts
- **AND** NVS `state_valid=true`
- **AND** the physical gear switch reports the same gear as `state_gear`
- **THEN** the encoder SHALL be set to `state_pos` via `EncoderCounter::setPosition()`
- **AND** autohome SHALL be skipped
- **AND** the system SHALL log "Restored transmission state, skipping autohome"

#### Scenario: Fall back to autohome on invalid or mismatched state
- **WHEN** the system starts
- **AND** NVS `state_valid=false`, or the physical switch disagrees with `state_gear`, or no state is saved
- **THEN** the normal autohome sequence SHALL run
- **AND** the system SHALL log the reason (invalid flag / switch mismatch / no data)

#### Scenario: State persists independently of calibration
- **WHEN** calibration data is cleared via `clearCalibration()`
- **THEN** the state keys (`state_valid`, `state_gear`, `state_pos`) SHALL also be cleared
- **AND** the next boot will run autohome
