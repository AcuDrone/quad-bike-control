## ADDED Requirements
### Requirement: CAN Engine RPM as Fallback Source
The CAN controller SHALL keep polling OBD-II PID `0x0C` and storing the decoded value in
`VehicleData.engineRPM` unchanged. `VehicleData.engineRPM` SHALL always hold the raw CAN value and
SHALL NOT be overwritten with the tacho reading; the choice between the tacho and CAN is made in the
vehicle layer, where CAN is the FALLBACK engine-RPM source.

#### Scenario: CAN value is preserved
- **WHEN** a PID `0x0C` reply arrives while the tacho is fitted
- **THEN** `VehicleData.engineRPM` SHALL be updated from the reply exactly as before
- **AND** no tacho value SHALL be written into `VehicleData`

#### Scenario: CAN is used when the tacho is not fitted
- **WHEN** the tacho is not fitted and CAN `VehicleData` is valid
- **THEN** the vehicle layer SHALL use `VehicleData.engineRPM` as the effective engine RPM, with
  source CAN, exactly as before this change

#### Scenario: Fast RPM polling during a gear change is kept
- **WHEN** the gear boost PID activates
- **THEN** `setRPMPollInterval(CAN_POLL_INTERVAL_RPM_BOOST)` SHALL still be called, whatever the
  effective RPM source, so the fallback stays fresh if the tacho stops
