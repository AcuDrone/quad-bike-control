## ADDED Requirements
### Requirement: Link Recovery Counters Telemetry
The WebSocket telemetry SHALL include session totals of the link recovery actions and the journal
size: `vesc_resync` (VESC parser resyncs), `can_reinit` (completed MCP2515 re-initializations of any
cause), `i2c_unlock` (I2C2 bus recoveries), `journal_bytes` (with flash writes on: size of both
journal files plus unflushed bytes, −1 when the journal cannot write to flash; with flash writes off:
the bytes of the RAM tail) and `journal_fs` (boolean, the journal flash-write switch). The counters
SHALL start at 0 at boot.
MAVLink output SHALL NOT change.

#### Scenario: Counters in each telemetry frame
- **WHEN** a telemetry frame is broadcast
- **THEN** it SHALL contain the five keys with their current values

#### Scenario: Counter increments
- **WHEN** the VESC parser resynchronizes once
- **THEN** `vesc_resync` SHALL be one higher in the next frame

#### Scenario: Journal unavailable
- **WHEN** flash writes are switched on and file output of the journal has been disabled after
  write failures
- **THEN** `journal_bytes` SHALL be −1

#### Scenario: Journal flash writes off
- **WHEN** the journal flash-write switch is off (the default)
- **THEN** `journal_fs` SHALL be `false` and `journal_bytes` SHALL be the RAM-tail size in bytes
