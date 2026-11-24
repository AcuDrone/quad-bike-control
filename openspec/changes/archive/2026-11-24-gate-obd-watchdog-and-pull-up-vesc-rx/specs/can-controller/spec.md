## ADDED Requirements
### Requirement: OBD-II No-Data Watchdog Ignition Gating
The CAN controller's OBD-II no-data watchdog (no valid OBD-II reply for `CAN_REINIT_NO_DATA_MS`,
15 s, while polling → MCP2515 re-initialization, at least `CAN_REINIT_MIN_INTERVAL_MS`, 15 s, after
the last re-initialization of any kind) SHALL run only while the relay controller's ignition state is
`ACC` or `IGNITION`, using the same ignition flag as the steering-angle frame silence watchdog,
because the ECU is powered from the ACC line (ignition Relay_1). The silence SHALL be counted from
the later of the last OBD-II reply (before any reply: `begin()` or the last re-initialization) and
the last ignition switch-on, so that the window starts fresh on every switch to `ACC` / `IGNITION`.
Bus-off and send-failure recovery SHALL NOT be gated.

#### Scenario: Ignition off, bus silent
- **WHEN** the ignition state is `OFF` (or `CRANKING`)
- **AND** no OBD-II reply has been received for any length of time
- **THEN** the OBD-II watchdog SHALL NOT re-initialize the MCP2515
- **AND** SHALL NOT journal `obd_silent`
- **AND** the `can_reinit` counter SHALL NOT be incremented by it

#### Scenario: Fresh window after switch-on
- **WHEN** the ignition is switched from `OFF` to `ACC` or `IGNITION`
- **AND** the last OBD-II reply (or the last re-initialization) is older than 15 s
- **THEN** the watchdog SHALL NOT fire at the switch-on
- **AND** SHALL fire only if no OBD-II reply arrives within 15 s after the switch-on

#### Scenario: Ignition on, ECU silent
- **WHEN** the ignition state is `ACC` or `IGNITION`
- **AND** no OBD-II reply has been received for 15 s counted from the later of the last reply / re-init
  and the switch-on
- **AND** at least 15 s have passed since the last re-initialization of any kind
- **THEN** the controller SHALL re-initialize the MCP2515 (reason `obd_silent`)
- **AND** journal `CAN W obd_silent ms=<silence> obd_alive=0 repeat=<n> -> reinit` under the existing
  per-episode journal economy

#### Scenario: New power cycle starts a new journal episode
- **WHEN** an `obd_silent` silence episode was running and the ignition goes `OFF` and back to `ACC`
  or `IGNITION`
- **THEN** the next `obd_silent` trigger SHALL be journaled (not counted as a quiet repeat)

#### Scenario: LWS watchdog re-init is not blocked
- **WHEN** the ignition state is `ACC` or `IGNITION` and the steering-angle frame silence watchdog
  requests a re-initialization
- **THEN** that re-initialization SHALL proceed exactly as before
- **AND** the OBD-II watchdog's minimum spacing SHALL be measured from it, as from any re-init
