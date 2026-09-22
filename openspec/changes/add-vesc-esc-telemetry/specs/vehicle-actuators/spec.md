## ADDED Requirements

### Requirement: VESC Reply and Fault-Event Counters
The steering VESC driver SHALL maintain two boot-cumulative counters — the number of valid
telemetry replies received, and the number of fault episodes observed — and SHALL expose them
through the motor-driver abstraction so the reporting layer can publish them without sampling. The
counters SHALL be observation only: no control decision SHALL depend on either value.

#### Scenario: Count valid telemetry replies
- **WHEN** a `COMM_GET_VALUES` reply is decoded successfully
- **THEN** a reply counter SHALL be incremented at that single point, together with the existing
  "have reply" flag and last-valid-reply timestamp
- **AND** the counter SHALL be a 16-bit value that is permitted to wrap, because only its advance
  is meaningful
- **AND** it SHALL NOT be incremented for a malformed frame, a failed CRC, or a reply of
  insufficient length

#### Scenario: Count fault episodes, not fault samples
- **WHEN** a decoded reply carries a non-zero fault code
- **AND** the previously decoded reply carried a zero fault code
- **THEN** a fault-event counter SHALL be incremented exactly once for that episode
- **AND** a fault that persists across many replies SHALL count once, not once per reply
- **AND** the counter SHALL NOT be reset when the fault clears, when the link recovers, or by any
  command or web control

#### Scenario: Expose both counters through the driver abstraction
- **WHEN** the reporting layer queries the steering driver
- **THEN** the `IMotorDriver` interface SHALL provide reply-count and fault-event-count accessors
  with default implementations returning 0, so drivers without telemetry (the brake H-bridge)
  need no change
- **AND** the VESC driver SHALL override both
- **AND** the steering controller SHALL forward both alongside its existing VESC telemetry
  accessors

#### Scenario: Counters never affect control
- **WHEN** either counter takes any value, including a wrapped or stalled one
- **THEN** no steering move SHALL be refused, stopped, or latched as a result
- **AND** the existing driver-health gate SHALL remain the recency of the last valid reply, not a
  counter value
