## MODIFIED Requirements

### Requirement: Transmission Control System
The Transmission Control System SHALL use a two-level priority for default gear **servo positions** (expressed as float percentages 0.0–100.0): user-configured NVS defaults take first priority; compile-time factory constants are the final fallback.

#### Scenario: Load user-configured defaults on startup
- **WHEN** the system starts
- **AND** NVS float keys `pct_r`, `pct_n`, `pct_l`, `pct_h` exist in the `transmission` namespace
- **THEN** `TransmissionController` SHALL load those values as its default positions

#### Scenario: Fall back to factory defaults when NVS defaults absent
- **WHEN** the system starts
- **AND** one or more `pct_*` NVS keys are absent
- **THEN** the corresponding position SHALL use the compile-time constant (`TRANS_GEAR_DEFAULT_*_PCT`) as the default
- **AND** no error SHALL be logged

#### Scenario: Update a default position at runtime
- **WHEN** `setDefaultPosition(gear, positionPct)` is called with a valid gear and a value in [0.0, 100.0]
- **THEN** the value SHALL be written to the corresponding `pct_*` NVS float key in the `transmission` namespace
- **AND** the in-RAM default SHALL be updated immediately

#### Scenario: Reject out-of-range position
- **WHEN** `setDefaultPosition(gear, positionPct)` is called with a value outside [0.0, 100.0]
- **THEN** the call SHALL return false
- **AND** neither NVS nor the in-RAM array SHALL be modified

### Requirement: Transmission State Persistence
The system SHALL persist the last confirmed transmission state (gear, servo position as float percent, validity flag) to NVS so that the servo can be restored to its last known position after a clean power cycle.

#### Scenario: Mark state invalid at start of gear change
- **WHEN** `TransmissionController::setGear()` is called
- **THEN** `state_valid=false` SHALL be written to NVS before the servo moves
- **AND** a mid-move power loss will therefore result in `state_valid=false` on next boot

#### Scenario: Save confirmed state on gear arrival
- **WHEN** the physical gear switch confirms the target gear is reached (RETURN phase complete)
- **THEN** `state_valid=true`, `state_gear`, and `state_pct` (float percent) SHALL be written to NVS
- **AND** the saved state reflects the servo position at that moment

#### Scenario: Restore servo position and skip autohome on valid state
- **WHEN** the system starts
- **AND** NVS `state_valid=true`
- **THEN** the servo SHALL be commanded to `state_pct` immediately
- **AND** the normal autohome path SHALL be skipped
- **AND** the system SHALL log "Restored transmission state, skipping autohome"

#### Scenario: Fall back to neutral default on invalid or missing state
- **WHEN** the system starts
- **AND** NVS `state_valid=false`, or no state is saved
- **THEN** the servo SHALL be commanded to the neutral default position (`TRANS_GEAR_DEFAULT_NEUTRAL_PCT`)
- **AND** the system SHALL log the reason (invalid flag / no data)
