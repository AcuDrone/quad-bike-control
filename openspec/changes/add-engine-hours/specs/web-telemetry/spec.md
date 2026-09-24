## ADDED Requirements

### Requirement: Engine Hours Telemetry
The telemetry stream SHALL report BOTH engine hour counters — the total and the resettable TRIP
hours — and the web interface SHALL display both, so that the accumulated running time is readable
from the portal as well as from the ground station. The web portal SHALL be **display only**: it
SHALL NOT offer any control that resets either counter — the total has no reset on any interface,
and the trip hours are cleared only by the ground station's existing trip-reset command.

#### Scenario: Include the engine hour meter in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **THEN** the JSON payload SHALL include `engine_hours` and `engine_trip_hours`, each in hours
  with 2 decimal places
- **AND** both fields SHALL be emitted unconditionally, independent of CAN status, MAVLink status
  and the engine's current state, because time already run does not stop being true when a bus goes
  quiet
- **AND** a zero `engine_trip_hours` SHALL be emitted as a genuine `0.00`, not omitted or replaced
  by a placeholder, because zero is what the counter reads immediately after a trip reset
- **AND** the additional keys SHALL keep the total telemetry message under 1 KB

#### Scenario: Populate the engine hour meter from the vehicle layer
- **WHEN** telemetry collection is triggered
- **THEN** `engine_hours` and `engine_trip_hours` SHALL be read from the vehicle controller's
  engine hour meter, alongside the existing engine and distance collection
- **AND** each value SHALL be the exact seconds counter scaled to hours at this point, not a
  separately maintained running total

#### Scenario: Display the engine hour meter with the engine data
- **WHEN** a telemetry message containing `engine_hours` and `engine_trip_hours` is received
- **THEN** the web UI SHALL display both with the engine/CAN data, in hours to 2 decimal places,
  the TRIP hours directly beneath the total
- **AND** neither SHALL be greyed out or blanked when the CAN bus is disconnected — unlike the live
  engine readings, accumulated running time is not invalidated by the bus going quiet
- **AND** a missing field SHALL render as a placeholder rather than as `0.00` or a JavaScript error,
  the placeholder being reserved for an ABSENT field so that a displayed `0.00` always means a
  genuine zero
- **AND** every new label SHALL have an i18n key present in BOTH the `en` and the `uk` dictionaries,
  with parity maintained

#### Scenario: The portal offers no reset control
- **WHEN** the web interface renders the engine data
- **THEN** it SHALL present BOTH engine hour counters as read-only text
- **AND** there SHALL be NO button, input or web command that resets either of them
- **AND** no `WebPortal::WebCommand` SHALL exist for either
- **AND** clearing the trip hours SHALL remain a GROUND-STATION action, performed by the same
  command that clears the trip distance, on the same terms as the trip distance itself
