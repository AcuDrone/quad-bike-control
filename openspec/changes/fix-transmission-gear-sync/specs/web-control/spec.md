## ADDED Requirements
### Requirement: Web Gear Selection Initialises the Transmission
The web `set_gear` command SHALL be accepted whether or not the engine is running (it is needed
for gear calibration with the engine off), and SHALL never start a shift from an uninitialised
transmission.

#### Scenario: Gear request before the transmission is initialised
- **WHEN** a `set_gear` command with `"R"`, `"N"`, `"H"` or `"L"` is received and the transmission
  is not initialised
- **THEN** the system SHALL first run the transmission initialisation (physical sensor, else valid
  NVS state, else the initialisation step into NEUTRAL)
- **AND** after an initialisation from the sensor or NVS the gear change SHALL start from the
  initialised gear and servo position, with the usual "Moving to <gear> gear" reply

#### Scenario: Gear unknown
- **WHEN** a `set_gear` command is received, the transmission is not initialised, the physical gear
  reads `GEAR_UNKNOWN` and NVS holds no valid state
- **THEN** the initialisation step into NEUTRAL SHALL start, and the requested gear SHALL be queued
  as during any active step, with the usual "Moving to <gear> gear" reply
- **AND** the requested gear SHALL be shifted to after the initialisation step completes
- **AND** no "gear unknown" refusal SHALL be sent

#### Scenario: Engine running is not required on the web
- **WHEN** a `set_gear` command is received with the engine stopped
- **THEN** it SHALL be processed as above; only the MAVLink gear path SHALL require the engine to
  be running
