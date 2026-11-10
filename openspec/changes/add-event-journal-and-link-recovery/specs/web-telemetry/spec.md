## ADDED Requirements
### Requirement: Link Recovery Counters Telemetry
The WebSocket telemetry SHALL include session totals of the link recovery actions and the journal
size: `vesc_resync` (VESC parser resyncs), `can_reinit` (completed MCP2515 re-initializations of any
cause), `i2c_unlock` (I2C2 bus recoveries) and `journal_bytes` (size of both journal files plus
unflushed bytes, −1 when the journal cannot write to flash). The counters SHALL start at 0 at boot.
MAVLink output SHALL NOT change.

#### Scenario: Counters in each telemetry frame
- **WHEN** a telemetry frame is broadcast
- **THEN** it SHALL contain the four keys with their current values

#### Scenario: Counter increments
- **WHEN** the VESC parser resynchronizes once
- **THEN** `vesc_resync` SHALL be one higher in the next frame

#### Scenario: Journal unavailable
- **WHEN** file output of the journal has been disabled after write failures
- **THEN** `journal_bytes` SHALL be −1
