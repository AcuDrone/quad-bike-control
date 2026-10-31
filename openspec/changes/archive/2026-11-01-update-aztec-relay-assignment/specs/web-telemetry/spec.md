## ADDED Requirements
### Requirement: Relay Test Telemetry
Web telemetry SHALL include `relay_test`, an 8-bit unsigned integer with one bit per relay in
relay-number order (bit0 = Relay_1 … bit7 = Relay_8), set for each relay currently energized by the
web relay test. It SHALL be derived from the relay output shadow byte (the last written and verified
port value) through the `Relay.PrjPcb` IO routing, limited to the test bits, so a vehicle function
relay is never reported as a test relay. The existing function fields (`ignition_state`,
`is_cranking`, `front_light_on`, `wheel_lock_on`, `wheel_lock_fitted`, `io_relay_fault`) SHALL be
unchanged.

#### Scenario: Test relay reported
- **WHEN** Relay_5 (IO3) is test-energized and verified
- **THEN** telemetry SHALL report `relay_test` = 16 (bit4)

#### Scenario: No test active
- **WHEN** no relay is test-energized, or the relay expander is unavailable
- **THEN** telemetry SHALL report `relay_test` = 0

#### Scenario: Auto-off or forced-off is visible
- **WHEN** a test relay times out or is forced off (function command, fail-safe, fault)
- **THEN** the next telemetry broadcast SHALL report its bit as 0
