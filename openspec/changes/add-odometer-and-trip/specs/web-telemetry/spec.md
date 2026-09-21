## ADDED Requirements

### Requirement: Odometer and Trip Telemetry
The telemetry stream SHALL report the total odometer and the trip distance, and the web interface
SHALL display both, so that the distance the vehicle has driven is readable from the portal as well
as from the ground station. The web portal SHALL be **display only**: it SHALL NOT offer any control
that resets the trip or the odometer, because the reset is a ground-station action.

#### Scenario: Include the distance counters in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **THEN** the JSON payload SHALL include `odo_km` (total distance) and `trip_km` (resettable trip
  distance), both in kilometres with 3 decimal places
- **AND** both fields SHALL be emitted unconditionally, independent of CAN status, MAVLink status
  and the speed sensor's own validity flag, because distance already driven does not stop being
  true when a bus goes quiet
- **AND** the two additional keys SHALL keep the total telemetry message under 1 KB

#### Scenario: Populate the distance counters from the speed sensor
- **WHEN** telemetry collection is triggered
- **THEN** `odo_km` and `trip_km` SHALL be read from the hall speed sensor's distance counters via
  the vehicle controller, alongside the existing `vehicle_speed` and `speed_valid` collection
- **AND** the values SHALL be the exact millimetre counters scaled to kilometres at this point, not
  a separately maintained running total

#### Scenario: Display both counters in the speed section
- **WHEN** a telemetry message containing `odo_km` and `trip_km` is received
- **THEN** the web UI SHALL display both in the speed section, in kilometres to 3 decimal places
- **AND** they SHALL NOT be greyed out or blanked when `speed_valid` is false — unlike the live
  speed reading, a recorded distance is not invalidated by the sensor going quiet
- **AND** a missing field SHALL render as a placeholder rather than as `0.000` or a JavaScript error
- **AND** every new label SHALL have an i18n key present in BOTH the `en` and the `uk` dictionaries,
  with parity maintained

#### Scenario: The portal offers no reset control
- **WHEN** the web interface renders the speed section
- **THEN** it SHALL present the odometer and trip values as read-only text
- **AND** there SHALL be NO button, input or web command that resets the trip or the odometer
- **AND** no `WebPortal::WebCommand` SHALL exist for either counter
