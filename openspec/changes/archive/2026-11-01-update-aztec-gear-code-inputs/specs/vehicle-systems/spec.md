## MODIFIED Requirements
### Requirement: Transmission Control System
The transmission control system SHALL enforce the physical gear sequence `[R, N, H, L]` and SHALL govern overshoot and rollback dwell timing via constants in `Constants.h`.

#### Scenario: Gear change routes through sequence
- **WHEN** a gear change command is received (S-bus or web)
- **AND** the target gear is not adjacent to the current gear in sequence `[R, N, H, L]`
- **THEN** the vehicle controller SHALL pass the final target gear to `TransmissionController::setGear()`
- **AND** intermediate gear changes SHALL be handled automatically by `TransmissionController`
- **AND** the telemetry SHALL report the current physical gear at each step

#### Scenario: Constants govern dwell timing
- **WHEN** `TRANS_OVERSHOOT_DWELL_MS` or `TRANS_ROLLBACK_DWELL_MS` are modified in `Constants.h`
- **THEN** the corresponding dwell durations in the transmission state machine SHALL reflect the updated values without additional code changes

#### Scenario: Physical gear is decoded from the 3-bit gear code
- **WHEN** `TransmissionController::getPhysicalGear()` reads a valid opto-input snapshot
- **THEN** it SHALL decode the gear code `[bit0 bit1 bit6]` only through
  `TransmissionController::decodeGearCode()`, using the runtime code table
- **AND** the runtime table SHALL default to `GEAR_CODE_LOW` = `101`, `GEAR_CODE_HIGH` = `001`,
  `GEAR_CODE_NEUTRAL` = `011`, `GEAR_CODE_REVERSE` = `010`, overridden by codes captured through the
  web gear code calibration (NVS `"transmission"` `code_r/n/h/l`, loaded at `begin()`)
- **AND** the four default codes SHALL be checked distinct at compile time
- **AND** in the runtime table every assigned code SHALL be distinct and never `000` or `111`; a gear
  may be UNSET (after another gear took its code over) and SHALL then never be decoded

#### Scenario: Captured code takes effect immediately
- **WHEN** a gear code is captured or the table is reset to defaults
- **THEN** the gear read cache SHALL be invalidated so the next `getPhysicalGear()` decodes with the
  new table
- **AND** the next gear code SHALL be logged again

#### Scenario: Code 101 is LOW
- **WHEN** the gear code reads `101` with the default table
- **THEN** the physical gear SHALL be `GEAR_LOW`

#### Scenario: Code 001 is HIGH
- **WHEN** the gear code reads `001` with the default table
- **THEN** the physical gear SHALL be `GEAR_HIGH`

#### Scenario: Code 011 is NEUTRAL
- **WHEN** the gear code reads `011` with the default table
- **THEN** the physical gear SHALL be `GEAR_NEUTRAL`

#### Scenario: Code 010 is REVERSE
- **WHEN** the gear code reads `010` with the default table
- **THEN** the physical gear SHALL be `GEAR_REVERSE`

#### Scenario: All lines open reads UNKNOWN
- **WHEN** the gear code reads `000` (e.g. mid-shift, every line open)
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`
- **AND** the unknown-gear behavior SHALL apply (throttle capped at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX`)

#### Scenario: Lost opto feed reads UNKNOWN
- **WHEN** the gear code reads `111` (opto feed lost, every line floats HIGH)
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`

#### Scenario: Unassigned codes read UNKNOWN
- **WHEN** the gear code reads `100` or `110` with the default table (in general: any code not in
  the runtime table)
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`

#### Scenario: Bit 7 does not affect the gear
- **WHEN** port bit 7 changes level while bits 0, 1 and 6 are unchanged
- **THEN** the physical gear SHALL be unchanged

#### Scenario: UNKNOWN code while idle is re-read from later snapshots
- **WHEN** the gear code decodes to `GEAR_UNKNOWN` and no gear change is in progress
- **THEN** `getPhysicalGear()` SHALL keep the previous cached gear and consume the next expander
  poll instead, at most `TRANS_GEAR_READ_RETRY_COUNT` times, without issuing I2C reads of its own
- **AND** once the retries are spent it SHALL report `GEAR_UNKNOWN`

#### Scenario: Gear code changes are logged
- **WHEN** the gear code differs from the last logged code
- **THEN** a TRANSMISSION debug line `[TRANS] gear code=0bXYZ -> <L|H|N|R|UNKNOWN>` SHALL be logged
