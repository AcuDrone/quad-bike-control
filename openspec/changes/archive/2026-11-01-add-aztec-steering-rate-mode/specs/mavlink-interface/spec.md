## ADDED Requirements
### Requirement: Steering Control Mode Reporting
The system SHALL report the current steering mode (`position`, `rate` or `blocked`, see "Sensorless Steering Rate Mode") to the MAVLink network as a `NAMED_VALUE_INT` (id 252) named `STEER_MODE` sent from this component, and SHALL announce every change with a `STATUSTEXT`. This reporting SHALL NOT be a control input, and it SHALL NOT change the set of `NAMED_VALUE_FLOAT` names this component sends.

#### Scenario: Periodic and on-change STEER_MODE
- **WHEN** the slow steering telemetry interval elapses (`MAVLINK_STEER_SLOW_TX_MS`, default 1000 ms)
- **THEN** a `NAMED_VALUE_INT` named `STEER_MODE` SHALL be sent from the ESP32's own system and component id (system 1, component 25), with value 0 = `position`, 1 = `rate`, 2 = `blocked`
- **AND** its `time_boot_ms` SHALL carry the milliseconds-since-boot value of that tick
- **WHEN** the steering mode changes
- **THEN** one additional `STEER_MODE` SHALL be sent on the next report opportunity, without waiting for the slow interval
- **AND** `STEER_MODE` SHALL keep being sent whatever the sensor, VESC or link state, and SHALL never carry any value other than 0, 1 or 2

#### Scenario: Mode change status text
- **WHEN** the steering mode changes
- **THEN** a `STATUSTEXT` SHALL be sent: `Steer: RATE mode (no sensor)` with severity WARNING on entry into `rate`, `Steer: blocked (no sensor)` with severity WARNING on entry into `blocked`, and `Steer: position mode` with severity INFO on entry into `position`
- **AND** it SHALL obey the existing `STATUSTEXT` rate limit, and a change that is suppressed by the rate limit SHALL still be visible through `STEER_MODE`

#### Scenario: Separate from the NAMED_VALUE_FLOAT exception
- **WHEN** steering telemetry is reported
- **THEN** the six `NAMED_VALUE_FLOAT` names (`STEER_POS`, `STEER_A`, `VESC_V`, `VESC_TEMP`, `VESC_OK`, `STEER_SCA`) SHALL remain the only `NAMED_VALUE_FLOAT` this component sends, unchanged in meaning and gating
- **AND** `STEER_POS` SHALL remain `NaN` in `rate` and `blocked` mode, because the sensor is unhealthy
- **AND** `STEER_MODE` SHALL be the only `NAMED_VALUE_INT` this component sends

#### Scenario: Reporting only
- **WHEN** `STEER_MODE` or the mode status text is sent with any value
- **THEN** no steering stop, interlock or other control action SHALL be triggered by the act of reporting it

### Requirement: Servo Command Frame Counter
The MAVLink interface SHALL expose a monotonically increasing count of accepted `SERVO_OUTPUT_RAW` command frames, so that a consumer can tell a freshly received command from a re-read of the last decoded channels.

#### Scenario: Count accepted frames
- **WHEN** a `SERVO_OUTPUT_RAW` frame from the learned autopilot is accepted as a command
- **THEN** the counter SHALL increase by one
- **AND** frames that are not accepted as commands SHALL NOT change it

#### Scenario: Detect a fresh command
- **WHEN** the vehicle layer compares the counter with the value it saw on its previous iteration
- **THEN** an unchanged counter SHALL mean that no new steering command has arrived, and the rate drive SHALL NOT be refreshed from the stale channel value
