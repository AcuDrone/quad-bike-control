## ADDED Requirements

### Requirement: Headlight Washer Telemetry
Every telemetry frame SHALL carry `washer_fitted` (bool, the persisted washer fitted switch),
`washer_on` (bool, a washer pulse is running), `washer_left_ms`
(int, remaining pulse time in ms, 0 when no pulse runs) and `washer_pulse_ms` (int, the pulse length
in use for the next wash, ms). All four SHALL be read from `RelayController` (through
`VehicleController`) at formatting time; `washer_on` SHALL be `false` and `washer_left_ms` 0 while
the relay board is unavailable. The four keys SHALL be scalar slots with keys stored by pointer
(≈ 64 B of document memory, ≈ 80 B of JSON) and SHALL fit the existing telemetry document without
resizing it.

#### Scenario: Pulse running
- **WHEN** a 3000 ms pulse started 900 ms ago
- **THEN** telemetry SHALL report `washer_on` true and `washer_left_ms` ≈ 2100

#### Scenario: No pulse
- **WHEN** no pulse runs
- **THEN** telemetry SHALL report `washer_on` false, `washer_left_ms` 0 and `washer_pulse_ms` equal to
  the stored setting (3000 by default)

#### Scenario: Not fitted
- **WHEN** the washer fitted switch is off
- **THEN** telemetry SHALL report `washer_fitted` false and `washer_on` false
