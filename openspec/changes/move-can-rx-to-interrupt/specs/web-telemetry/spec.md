## ADDED Requirements
### Requirement: CAN Receive Path Telemetry
The telemetry broadcast SHALL report whether CAN reception is interrupt-driven or running on the polling fallback, and how many received frames were dropped, and the web interface SHALL show it next to the CAN status.

#### Scenario: Include CAN receive path keys
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `can_int_ok` (boolean, false only while the controller has detected an inactive `INT` line) and `can_rx_queue_drops` (cumulative frames dropped because the receive queue was full)
- **AND** both keys SHALL be emitted independently of `can_status`, and SHALL be omitted when the CAN controller failed to initialize

#### Scenario: Show the receive mode on the CAN status tile
- **WHEN** the web UI renders a telemetry frame with these keys
- **THEN** the "Стан CAN" / "CAN Status" tile SHALL show one muted line "RX: переривання" / "RX: interrupt" while `can_int_ok` is true, and "RX: опитування (INT не працює)" / "RX: polling (INT dead)" in the warning colour while it is false
- **AND** it SHALL append the drop count when `can_rx_queue_drops` is greater than zero
- **AND** it SHALL show nothing extra when the keys are missing, without a JavaScript error
- **AND** the new labels SHALL have i18n keys in both the `en` and `uk` dictionaries

#### Scenario: Not an attention item
- **WHEN** `can_int_ok` is false
- **THEN** the diagnostics attention summary SHALL NOT change, because reception continues on the polling fallback and real harm already surfaces as a steering-sensor fault
