## MODIFIED Requirements
### Requirement: Ignition State Broadcast
The system SHALL send a `NAMED_VALUE_INT` named `IGN_STATE` from this component with the APPLIED
relay ignition state: 0 OFF, 1 ACC, 2 ON, 3 CRANKING. CRANKING SHALL cover both the pre-crank dwell
(state ACC with a crank armed; its length is the configured pre-crank dwell, 1–8 s, default 3 s —
see `vehicle-systems` "Configurable Pre-Crank Dwell") and the starter phase (at most the configured
starter time limit, 2–10 s, default 5 s — see `vehicle-systems` "Configurable Starter Time
Limit"); after the engine catches or the starter time limit ends the attempt the value SHALL be 2.
With the relay board unavailable the value SHALL be 0. It SHALL be sent on the 1 Hz slow telemetry
tick and once immediately on every change, whatever the input source.

#### Scenario: Auto-crank sequence
- **WHEN** an ignition ON command starts the auto-crank from OFF
- **THEN** `IGN_STATE` SHALL be 3 during the configured pre-crank dwell (3 s by default) and while
  the starter runs
- **AND** it SHALL become 2 when the engine starts or the configured starter time limit (5 s by
  default) ends the attempt

#### Scenario: On-change send
- **WHEN** the applied ignition state changes from 2 to 0
- **THEN** an `IGN_STATE` with value 0 SHALL be sent on the next report, without waiting for the
  1 Hz tick
