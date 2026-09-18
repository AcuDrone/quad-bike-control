## ADDED Requirements

### Requirement: Autopilot Maximum-Speed Parameter Subscription
The system SHALL subscribe to the autopilot's `SPEED_MAX` parameter over MAVLink so the vehicle's
own maximum-speed limiter can take its ceiling from the value the operator edits in the ground
station. The subscription SHALL be read-only: the system SHALL NOT send `PARAM_SET` or otherwise
write any autopilot parameter. A value that cannot be trusted SHALL be rejected rather than stored,
and an absent, zero, silent or stale value SHALL be reported as unavailable so the vehicle layer
can fall back to its own stored setting.

#### Scenario: Poll the parameter periodically
- **WHEN** the autopilot's system and component ids have been learned from its `HEARTBEAT`
- **AND** the link is up
- **THEN** the system SHALL send a `PARAM_REQUEST_READ` for `SPEED_MAX` with a parameter index of
  `-1` (look up by name), addressed to the learned autopilot
- **AND** the first request SHALL be sent `MAVLINK_PARAM_FIRST_DELAY_MS` after the autopilot was
  learned, so the request does not race the autopilot's own boot
- **AND** subsequent requests SHALL be sent every `MAVLINK_PARAM_POLL_MS`
- **AND** polling SHALL NOT stop once a value has been received, because the poll — not any
  broadcast — is what detects a change made from the ground station

#### Scenario: Accept an unsolicited parameter broadcast
- **WHEN** a `PARAM_VALUE` message arrives that was not requested by this system
- **AND** it carries the parameter id `SPEED_MAX`
- **THEN** it SHALL be accepted on the same terms as a polled reply
- **AND** this SHALL be treated as an optimisation only, because whether the autopilot broadcasts
  after a `PARAM_SET` is firmware-version and routing dependent

#### Scenario: Reject a parameter from any sender other than the autopilot
- **WHEN** a `PARAM_VALUE` for `SPEED_MAX` arrives
- **AND** its sender system id or component id does not match the learned autopilot
- **THEN** the value SHALL be discarded without being stored
- **AND** the stored value and its timestamp SHALL be left unchanged, so that a ground station or
  any other component sharing the link cannot move the vehicle's speed ceiling

#### Scenario: Compare the parameter id safely
- **WHEN** a `PARAM_VALUE` is decoded
- **THEN** the 16-byte `param_id` field SHALL be copied into a 17-byte buffer and terminated
  before any string comparison
- **AND** the field SHALL NOT be compared in place, because MAVLink does not NUL-terminate a
  `param_id` that fills all 16 bytes

#### Scenario: Reject a value that is not a usable speed
- **WHEN** a `PARAM_VALUE` for `SPEED_MAX` arrives from the autopilot
- **AND** its value is not-a-number, infinite, negative, or greater than
  `MAVLINK_PARAM_SPEED_MAX_MS`
- **THEN** the value SHALL be rejected and SHALL NOT be stored
- **AND** the rejection SHALL be logged
- **AND** the previously stored value and its timestamp SHALL be left unchanged, so one corrupt
  frame cannot invalidate a good subscription

#### Scenario: Store and expose an accepted value
- **WHEN** a `PARAM_VALUE` for `SPEED_MAX` passes every check
- **THEN** the raw value SHALL be stored in metres per second together with the time of receipt
- **AND** the system SHALL expose the value converted to km/h (`× MAVLINK_MS_TO_KMH`), the raw
  m/s value, and the age of the reading
- **AND** a change SHALL be logged only when the new value differs from the previous one by more
  than `MAVLINK_PARAM_EPSILON_MS`, so a 5-second poll of an unchanged parameter does not fill the
  console

#### Scenario: Report the parameter as unavailable when it cannot be trusted
- **WHEN** the vehicle layer asks whether a `SPEED_MAX` value is available
- **THEN** the answer SHALL be true only if a value has been received, the value is greater than
  zero, the link is up, and the reading is younger than `MAVLINK_PARAM_STALE_MS`
- **AND** a value of zero SHALL be reported as unavailable, because the autopilot itself reads
  `SPEED_MAX = 0` as "not set"
- **AND** both the link gate and the age gate SHALL apply independently, so a disconnected cable
  is reported as unavailable within the heartbeat timeout while an autopilot that is alive but has
  stopped answering this parameter is reported as unavailable within the staleness timeout

#### Scenario: Reset the subscription on re-initialisation
- **WHEN** the MAVLink interface is initialised
- **THEN** the stored value SHALL be cleared to "never received"
- **AND** the receipt timestamp and the poll timestamp SHALL be cleared, so the first poll after a
  restart is scheduled from the moment the autopilot is next learned

#### Scenario: Surface the subscription state on the diagnostic line
- **WHEN** the periodic (1 Hz) MAVLink debug line is emitted
- **THEN** it SHALL include the last received `SPEED_MAX`, the age of that reading, and whether the
  value is currently considered available
- **AND** this SHALL be sufficient to distinguish "never received", "received and fresh" and
  "received but stale" on the serial console alone
