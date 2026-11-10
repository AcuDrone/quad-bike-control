## ADDED Requirements
### Requirement: VESC UART Receive Resynchronization
The VESC UART packet parser SHALL never stay misaligned for longer than one reply period. It SHALL
live in a header-only `include/VescRxParser.h` so it can be tested on the host, SHALL reset to the
start-byte search when more than `VESC_RX_GAP_MS` (5 ms) have passed since the previous received byte
while a frame is incomplete, and SHALL keep link counters for the event journal and telemetry. These
rules are additions to the existing framing, CRC and timeout behavior, which stays unchanged.

#### Scenario: Gap ends an incomplete frame
- **WHEN** the parser is in any state other than the start-byte search
- **AND** a byte arrives more than 5 ms after the previous byte
- **THEN** the parser SHALL return to the start-byte search before handling that byte
- **AND** the `resync` counter SHALL be incremented and `VESC W resync` journaled (deduplicated)

#### Scenario: One lost byte costs one reply
- **WHEN** one byte of a `COMM_GET_VALUES` reply is lost on the wire
- **AND** the following replies arrive 300 ms apart and contain no `0x02`/`0x03` data bytes
- **THEN** the reply with the lost byte SHALL be dropped
- **AND** the next reply SHALL be decoded
- **AND** a host test SHALL show that the same stream without the gap rule decodes none of 10
  following replies

#### Scenario: Link-lost transition restarts the receiver
- **WHEN** `driverOk()` changes from true to false
- **THEN** the driver SHALL discard the bytes waiting in the UART receive buffer and reset the parser

#### Scenario: Link counters
- **WHEN** the driver runs
- **THEN** it SHALL count received bytes, good packets, packets dropped for a CRC mismatch, packets
  dropped for a wrong stop byte, resyncs and the bytes since the last good packet
- **AND** expose them through `IMotorDriver` (all zero for drivers without a packet link)

#### Scenario: Loop stall in the middle of a reply
- **WHEN** the main loop reads part of a reply, then stalls for more than 5 ms before reading the rest
- **THEN** at most that one reply SHALL be lost and the next reply SHALL be decoded
