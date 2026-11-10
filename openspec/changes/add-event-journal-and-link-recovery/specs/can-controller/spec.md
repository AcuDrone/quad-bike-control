## ADDED Requirements
### Requirement: Steering Angle Frame Silence Watchdog
The CAN controller SHALL re-initialize the MCP2515 when the Bosch LWS `0x305` stream stops while the
rest of the bus demonstrably works, because the existing OBD-II no-data watchdog
(`CAN_REINIT_NO_DATA_MS`, 15 s) never fires on a vehicle whose ECU keeps answering. The OBD-II
watchdog SHALL stay unchanged. Because the LWS is powered from the ACC line (ignition Relay_1), the
watchdog SHALL run only while the relay controller's ignition state is `ACC` or `IGNITION`, SHALL arm
only after the first `0x305` frame received since the ignition was switched on, and SHALL use a fixed
re-init spacing of `CAN_LWS_REINIT_MIN_MS` (5 s) without any back-off.

#### Scenario: LWS silent, ECU answering
- **WHEN** the ignition state is `ACC` or `IGNITION`
- **AND** at least one `0x305` frame has been received since the ignition was switched on
- **AND** no `0x305` frame has been received for more than `CAN_LWS_SILENT_MS` (1 s)
- **AND** an OBD-II reply was received within `CAN_LWS_WD_OBD_ALIVE_MS` (2 s)
- **AND** no LWS zero-set sequence is running
- **AND** at least `CAN_LWS_REINIT_MIN_MS` (5 s) have passed since the last re-initialization of any kind
- **THEN** the controller SHALL re-initialize the MCP2515 (reset, both acceptance paths, NORMAL mode)
- **AND** journal `CAN W lws_silent ms=<silence> obd_alive=1 -> reinit`

#### Scenario: Whole bus silent
- **WHEN** neither `0x305` frames nor OBD-II replies are being received
- **THEN** the LWS watchdog SHALL NOT re-initialize the chip
- **AND** recovery SHALL be left to the OBD-II no-data watchdog

#### Scenario: Sensor never seen
- **WHEN** no `0x305` frame has been received since the ignition was switched on
- **THEN** the LWS watchdog SHALL stay disarmed

#### Scenario: Ignition off
- **WHEN** the ignition state is `OFF` (or `CRANKING`)
- **THEN** the LWS watchdog SHALL NOT journal `lws_silent` and SHALL NOT re-initialize the chip
- **AND** switching the ignition to `ACC` or `IGNITION` again SHALL leave the watchdog disarmed until
  the next `0x305` frame

#### Scenario: Sensor stays dead with the ignition on
- **WHEN** the armed watchdog has re-initialized the chip and the LWS still sends nothing
- **THEN** further re-inits SHALL follow at the fixed `CAN_LWS_REINIT_MIN_MS` (5 s) spacing, with no
  back-off
- **AND** within that silence episode the `lws_silent … -> reinit` line SHALL be journaled at its
  start and then at most every `CAN_WD_REPEAT_LOG_MS` (10 min) with `repeat=<re-inits since the last
  line>`; the successful re-inits in between SHALL be counted (`can_reinit`) but not journaled
  one by one (the same rule applies to the OBD-II no-data watchdog while the ECU stays silent)

#### Scenario: First frame after a re-initialization
- **WHEN** a re-initialization completed
- **THEN** the times to the first `0x305` frame and the first OBD-II reply SHALL be journaled as
  `CAN I first_frame`

### Requirement: MCP2515 Re-initialization Verification
Every MCP2515 re-initialization (bus-off, send failure, OBD-II no-data watchdog, LWS silence
watchdog) SHALL check the results of the chip reset (`begin()`), of `applyRxFilters()` and of
`setMode(MCP_NORMAL)`, SHALL retry a failed attempt on later `update()` calls without looping inside
one call, at most `CAN_REINIT_MAX_ATTEMPTS` (3) times per trigger, and SHALL journal every outcome.

#### Scenario: Re-initialization succeeds
- **WHEN** all three steps succeed
- **THEN** `CAN I reinit reason=<trigger> result=ok attempt=<n>` SHALL be journaled (except for a
  watchdog repeat inside one silence episode, see "Sensor stays dead with the ignition on")
- **AND** the session counter `can_reinit` SHALL be incremented

#### Scenario: Re-initialization fails
- **WHEN** any of the three steps fails
- **THEN** the attempt SHALL be repeated on a later `update()` call, up to 3 attempts in total
- **AND** after the third failure `CAN E reinit result=fail` SHALL be journaled and the next watchdog
  interval SHALL try again

#### Scenario: Bus-off is journaled
- **WHEN** the controller detects bus-off
- **THEN** `CAN E bus_off tec=<TEC> rec=<REC> eflg=<EFLG>` SHALL be journaled before the recovery
