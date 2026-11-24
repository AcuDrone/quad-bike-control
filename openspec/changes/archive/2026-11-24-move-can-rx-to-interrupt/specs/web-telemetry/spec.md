## ADDED Requirements
### Requirement: CAN Driver Telemetry
The telemetry broadcast SHALL report how the CAN task receives (interrupt or polling) and its loss and load counters, and the web interface SHALL show them on the CAN status tile in Diagnostics.

#### Scenario: Include CAN driver keys
- **WHEN** telemetry data is formatted as JSON and the CAN task exists
- **THEN** it SHALL include `can_int` (`"ok"`, `"dead"` or `"none"`), `can_rx_drop` (frames dropped from the full receive queue), `can_rx_ovr` (MCP2515 RX overflow flags counted), `can_rx_hwm` (receive-queue high-water mark), `can_tx_fail` (transmit failures) and `can_busy_max_us` (longest CAN task receive / transmit service pass, µs, initialization and filter-change time excluded), all as session values
- **AND** these keys SHALL be emitted independently of `can_status`

#### Scenario: Show the receive mode on the CAN status tile
- **WHEN** the web UI renders a telemetry frame with these keys
- **THEN** the "Стан CAN" / "CAN Status" tile SHALL show one muted line: "RX: переривання" / "RX: interrupt" for `ok`, "RX: опитування (без INT)" / "RX: polling (no INT wire)" for `none`, and "RX: опитування (INT не працює)" / "RX: polling (INT dead)" in the warning colour for `dead`
- **AND** it SHALL append "втрачено N" / "dropped N" when `can_rx_drop` is greater than zero and "переповнення N" / "overflow N" when `can_rx_ovr` is greater than zero
- **AND** it SHALL show nothing extra when the keys are missing, without a JavaScript error
- **AND** the new labels SHALL have i18n keys in both the `en` and `uk` dictionaries

#### Scenario: Not an attention item and not a setting
- **WHEN** `can_int` is `dead` or a counter is non-zero
- **THEN** the diagnostics attention summary SHALL NOT change, because reception continues on the polling fallback and real harm already surfaces as a steering-sensor fault
- **AND** no control for these values SHALL be added to the Calibration section
