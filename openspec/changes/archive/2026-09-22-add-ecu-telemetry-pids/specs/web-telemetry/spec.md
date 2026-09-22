## ADDED Requirements

### Requirement: ECU Electrical and Air Telemetry
The system SHALL include control module voltage, intake air temperature and calculated engine load
in the CAN vehicle-data telemetry, and the web interface SHALL display them in the CAN telemetry
card, so operators can see charging-system health, intake air conditions and the ECU's own load
estimate alongside the other engine parameters.

#### Scenario: Include ECU voltage, intake temperature and engine load in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **AND** CAN status is "connected"
- **THEN** the telemetry JSON SHALL include an `ecu_voltage` field carrying the control module
  supply voltage in volts with two decimal places
- **AND** SHALL include an `intake_temp` field carrying the intake air temperature in °C
- **AND** SHALL include an `engine_load` field carrying the calculated engine load as a percentage
  (0–100)
- **WHEN** CAN status is not "connected"
- **THEN** all three fields SHALL be omitted alongside the other CAN vehicle-data fields, rather
  than being emitted as zeros

#### Scenario: Populate the fields from CAN vehicle data
- **WHEN** telemetry collection is triggered
- **AND** the CAN `VehicleData` is valid
- **THEN** the values SHALL be copied from the most recent `VehicleData` module voltage, intake
  temperature and engine load readings
- **WHEN** the CAN `VehicleData` is not valid
- **THEN** the fields SHALL be reset to their zero defaults and the CAN-gated block SHALL be
  omitted from the payload

#### Scenario: Display the new values in the CAN telemetry card
- **WHEN** a telemetry message containing `ecu_voltage`, `intake_temp` and `engine_load` is
  received
- **THEN** the CAN telemetry card SHALL display module voltage in volts, intake air temperature in
  °C, and engine load as a percentage
- **AND** the values SHALL update in real time as new telemetry arrives
- **WHEN** `can_status` is not "connected"
- **THEN** the three readouts SHALL show a placeholder (e.g. `-- V`, `--°C`, `--%`) rather than a
  stale or misleading value

#### Scenario: New UI labels have full i18n parity
- **WHEN** the new CAN-card labels are added to the web UI
- **THEN** each label SHALL use a `data-i18n` key
- **AND** every new key SHALL be present in BOTH the `en` and `uk` dictionaries with maintained
  parity (no key defined in only one dictionary)

#### Scenario: Additions stay within the telemetry size budget
- **WHEN** the three fields are added to the steady-state telemetry payload
- **THEN** the serialized JSON document SHALL remain within the configured
  `StaticJsonDocument` capacity, including the peak case where the transient ECU probe object is
  also present
- **AND** the steady-state payload growth SHALL be bounded by the three scalar fields (no arrays
  or nested objects are introduced)
- **AND** the 5 Hz broadcast cadence SHALL be unaffected
