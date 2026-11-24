## ADDED Requirements
### Requirement: Effective Engine RPM in MAVLink Reports
The `EFI_STATUS.rpm` field, and every other MAVLink value derived from engine RPM, SHALL carry the
EFFECTIVE engine RPM selected by the vehicle layer (tacho when fitted, CAN otherwise), gated on the
validity of that effective RPM rather than on CAN health. No tacho value SHALL be sent as a
`NAMED_VALUE_FLOAT`.

#### Scenario: Tacho RPM reported without CAN
- **WHEN** the engine-telemetry interval elapses and the effective RPM source is TACHO
- **THEN** `EFI_STATUS.rpm` SHALL be the tacho RPM, whether or not CAN `VehicleData` is valid
- **AND** a fitted, silent tacho SHALL report `0`, a genuine stopped engine

#### Scenario: CAN fallback
- **WHEN** the effective RPM source is CAN
- **THEN** `EFI_STATUS.rpm` SHALL be the CAN RPM, as before this change

#### Scenario: No RPM source
- **WHEN** the effective RPM source is NONE
- **THEN** `EFI_STATUS.rpm` SHALL be `NaN`

#### Scenario: No TACHO_HZ named value
- **WHEN** MAVLink telemetry is sent
- **THEN** no `NAMED_VALUE_FLOAT` for the tacho frequency SHALL be sent, because all
  `NAMED_VALUE_FLOAT` messages share one message id and collide in a name-agnostic store; the tacho
  frequency is a calibration value available in web telemetry
