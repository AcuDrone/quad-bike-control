## ADDED Requirements
### Requirement: CAN Sniffer Command via Web Interface
The system SHALL accept a `can_sniff` WebSocket command that starts a 10-second CAN sniff, routed
like `can_probe` and accepted regardless of the active input source (diagnostic).

#### Scenario: Start from the web console
- **WHEN** the operator presses "CAN sniffer (10 s)" on the diagnostics page
- **THEN** the page SHALL send `{"cmd":"can_sniff"}`
- **AND** the firmware SHALL call `startSniff(10000)` and reply "CAN sniffer started"

#### Scenario: Refused
- **WHEN** the sniffer cannot start (probe active, LWS zero-set active, gear change, already
  running, CAN not initialised)
- **THEN** the reply SHALL be unsuccessful with a message that starts with "CAN sniffer refused"
- **AND** the page status line SHALL show the localised reason

#### Scenario: Results table
- **WHEN** sniffer results are present in telemetry
- **THEN** the page SHALL show the status (running with elapsed time, or done), a table ordered by
  ID with ID (hex, EXT marker), count, rate in Hz (count / window), and the last data bytes in hex
  with the changed bytes highlighted
- **AND** a summary line with IDs beyond 32, total frames and overflow events
- **AND** all labels SHALL exist in Ukrainian and English
