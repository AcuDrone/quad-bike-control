## MODIFIED Requirements

### Requirement: Transmission Control System
The Transmission Control System SHALL use a three-level priority for default gear encoder positions: auto-calibrated positions (from `calibrateAllGearPositions()`) take highest priority; user-configured NVS defaults take second priority; compile-time factory constants are the final fallback.

#### Scenario: Load user-configured defaults on startup
- **WHEN** the system starts
- **AND** NVS keys `def_reverse`, `def_neutral`, `def_low`, `def_high` exist in the `transmission` namespace
- **THEN** `TransmissionController` SHALL load those values as its default positions for use when no auto-calibration is active

#### Scenario: Fall back to factory defaults when NVS defaults absent
- **WHEN** the system starts
- **AND** one or more `def_*` NVS keys are absent
- **THEN** the corresponding position SHALL use the compile-time constant (`TRANS_POSITION_*`) as the default
- **AND** no error SHALL be logged

#### Scenario: Update a default position at runtime
- **WHEN** `setDefaultPosition(gear, position)` is called with a valid gear and position value
- **THEN** the value SHALL be written to the corresponding `def_*` NVS key in the `transmission` namespace
- **AND** the in-RAM default SHALL be updated immediately so subsequent `getGearPosition()` calls (when uncalibrated) return the new value

#### Scenario: Auto-calibration overrides without clearing NVS defaults
- **WHEN** `calibrateAllGearPositions()` completes successfully
- **THEN** calibrated positions SHALL be used by `getGearPosition()`
- **AND** the `def_*` NVS keys SHALL remain unchanged
- **AND** clearing calibration via `clearCalibration()` SHALL restore the system to the NVS-default tier (not factory constants), provided `def_*` keys are present
