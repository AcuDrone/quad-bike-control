## ADDED Requirements
### Requirement: CAN Sniffer Results Telemetry
The telemetry broadcast SHALL carry the CAN sniffer results while the sniffer runs or its results
are fresh, and omit them otherwise.

#### Scenario: Include while fresh
- **WHEN** the sniffer is running or completed less than `SNIFF_RESULT_TTL` ago
- **THEN** the telemetry JSON SHALL include a `sniff` object with running/complete flags, window
  and elapsed time, total frames, overflow events, IDs beyond 32 (with a saturation flag), and an
  `ids` array of rows `{id, x, n, d, c}` (ID, extended flag, count, data hex, changed mask)

#### Scenario: Omit when stale
- **WHEN** no sniff is running and the last results are older than the TTL
- **THEN** the `sniff` object SHALL be omitted

#### Scenario: Capacity
- **WHEN** a full 32-row `sniff` object and a `probe` object are present at the same time
- **THEN** the static telemetry document SHALL not overflow
