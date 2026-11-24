## MODIFIED Requirements
### Requirement: Ignition State Control Integration
The system SHALL integrate ignition state control with vehicle systems coordination.

#### Scenario: Set ignition state via vehicle controller
- **WHEN** VehicleController.setIgnitionState() is called with state (OFF/ACC/IGNITION/START)
- **THEN** ignition state is passed to RelayController.setIgnitionState()
- **AND** ignition state change is logged
- **AND** vehicle state telemetry includes new ignition state

#### Scenario: Query current ignition state
- **WHEN** VehicleController.getIgnitionState() is called
- **THEN** current ignition state is returned from RelayController (OFF/ACC/IGNITION/CRANKING)
- **AND** state reflects actual relay configuration

#### Scenario: Monitor engine cranking completion
- **WHEN** VehicleController.update() is called each control loop
- **THEN** RelayController.update() SHALL be called with the validated engine RPM (see "Validated
  Engine RPM for Crank Control"): the CAN engine RPM when the CAN data is valid and 0 otherwise on
  the quad bike, the effective RPM (tacho first, 0 when no source is valid) on the aztec buggy
- **AND** cranking automatically stops when the engine starts (validated RPM ≥
  `ENGINE_RUNNING_RPM_THRESHOLD`)
- **AND** cranking automatically stops after the 5-second `CRANKING_TIMEOUT`
- **AND** ignition state transitions from CRANKING to IGNITION after cranking completes

### Requirement: GCS Vehicle Command Interlocks
The vehicle layer SHALL execute the GCS vehicle commands (gear, ignition, headlight, diff lock)
through the same controllers the RC path uses and SHALL be the arbiter of every interlock. "Not
under MAVLink control" SHALL mean `currentInputSource_ != InputSource::MAVLINK` OR
`!isSignalValid()`; it therefore covers the fail-safe source, the web portal as the source after
an RC stream loss with a browser connected, and the web portal with web control latched (owner
decision D-B). Reason 3 SHALL be given for exactly this condition, and fail-safe behaviour itself
SHALL NOT change.

- **Gear**, checked in this order:
  - not under MAVLink control → `TEMPORARILY_REJECTED` reason 3;
  - engine not running (`isEngineRunning()`), or the transmission not yet initialised → reason 1,
    and the command SHALL NOT be remembered;
  - the target gear already equals the requested gear → `ACCEPTED`, nothing moves;
  - the transmission's speed interlock (`canChangeGear()`) refuses → reason 2; this interlock
    SHALL be used unchanged (N is never speed-refused; the CAN fallback applies while the hall
    sensor has not pulsed);
  - otherwise `setGear()` → `ACCEPTED`.
  - H SHALL be accepted over MAVLink when capability bit1 is set; the handbrake state SHALL NOT
    block a gear command.
- **Ignition**:
  - OFF SHALL always be `ACCEPTED` (engine kill), including in fail-safe, under web control,
    during the pre-crank dwell and while cranking; it SHALL set the relay state OFF and persist the
    distance and engine-hour counters on a transition into OFF;
  - ACC or ON not under MAVLink control → reason 3;
  - ACC → relay state ACC, cancelling a pending or active crank;
  - ON with the engine running (`isEngineRunning()`, which reads the engine RPM cleared at
    ignition OFF — see "Validated Engine RPM for Crank Control") → `ACCEPTED` without a crank;
  - ON otherwise → `requestCrank()` exactly like a fresh RC transition into IGNITION (the
    configured pre-crank dwell — see "Configurable Pre-Crank Dwell", default 3000 ms — starter at
    most `CRANKING_TIMEOUT`).
- **Headlight**: always accepted, in every input source.
- **Diff lock**: not under MAVLink control → reason 3; otherwise the same `setWheelLock()` path
  channel 7 uses (quad bike: Relay_5 + Relay_6; buggy: Relay_4 + Relay_5 + Relay_6), reported in
  `EFI_STATUS.pt_compensation` bit0.
- With the relay board unavailable, an ignition, headlight or diff-lock command SHALL be answered
  `MAV_RESULT_FAILED` and SHALL change nothing.

#### Scenario: Gear with the engine stopped
- **WHEN** a GCS gear L command arrives under MAVLink control with the engine not running
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 1
- **AND** the gear SHALL NOT be applied when the engine starts later

#### Scenario: Gear while moving
- **WHEN** a GCS gear R command arrives with the engine running at 2.0 m/s
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 2 and the gear SHALL not change

#### Scenario: Gear already selected
- **WHEN** a GCS gear N command arrives with the engine running and the target gear already N
- **THEN** the ACK SHALL be `ACCEPTED` and the transmission SHALL not move

#### Scenario: Gear in fail-safe
- **WHEN** a GCS gear command arrives while the input source is `FAILSAFE`
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 3

#### Scenario: Web portal in control
- **WHEN** the input source is the web portal
- **THEN** gear, ignition ACC / ON and diff lock commands SHALL be answered `TEMPORARILY_REJECTED`
  with reason 3
- **AND** headlight and ignition OFF commands SHALL be accepted and executed

#### Scenario: Engine kill always works
- **WHEN** a GCS ignition OFF command arrives while cranking, in fail-safe or under web control
- **THEN** the ACK SHALL be `ACCEPTED` and the ignition and starter relays SHALL be switched off

#### Scenario: Ignition ON with the engine running
- **WHEN** a GCS ignition ON command arrives with the engine running
- **THEN** the ACK SHALL be `ACCEPTED` and the starter SHALL NOT engage

#### Scenario: Auto-crank from the GCS
- **WHEN** a GCS ignition ON command arrives under MAVLink control with the ignition OFF and the
  engine stopped
- **THEN** the ACK SHALL be `ACCEPTED`
- **AND** the ignition line SHALL come up, the starter SHALL engage after the configured pre-crank
  dwell and SHALL stop on engine start or after `CRANKING_TIMEOUT`

#### Scenario: Ignition ON right after an engine kill
- **WHEN** the engine was running at or above `ENGINE_RUNNING_RPM_THRESHOLD`, a GCS ignition OFF
  stopped it, and a GCS ignition ON arrives less than `CAN_DATA_STALE_TIMEOUT` later
- **THEN** the command SHALL NOT be treated as "ON with the engine running"
- **AND** the auto-crank SHALL start as in "Auto-crank from the GCS"

#### Scenario: Relay board unavailable
- **WHEN** a GCS headlight command arrives while the relay expander is faulted
- **THEN** the ACK SHALL be `MAV_RESULT_FAILED`

## ADDED Requirements
### Requirement: Configurable Pre-Crank Dwell
The pre-crank dwell — the time the ignition / ECU line SHALL have been powered before the starter
engages on an auto-crank — SHALL be a persisted, operator-set, per-vehicle value in whole
milliseconds, range `CRANK_DWELL_MS_MIN` (1000) to `CRANK_DWELL_MS_MAX` (8000), default
`ACC_PRECRANK_DWELL_MS` (3000). It SHALL be stored in the NVS namespace `"crank"`, key `dwell_ms`
(uint16), owned by `VehicleController`, loaded at boot and handed to `RelayController` through a
RAM-only setter. It SHALL apply to every crank armed through `requestCrank()`: a fresh RC ignition
ON, the re-crank when RC control returns after fail-safe or web control, and the GCS
`MAV_CMD_USER_2` ignition ON. The dwell SHALL be counted from the moment the ignition line last
went from OFF to powered. The web START command SHALL NOT use the dwell (it sets CRANKING
directly, unchanged). The armed-crank decision SHALL be the pure function `crankArmedDecide()` in
`include/CrankLaw.h`, covered by native tests.

#### Scenario: Default on a unit without a stored value
- **WHEN** the unit boots with no `crank`/`dwell_ms` key
- **THEN** the pre-crank dwell SHALL be 3000 ms

#### Scenario: Auto-crank waits for the configured dwell
- **WHEN** the dwell is 4500 ms and a fresh RC ignition ON arrives with the ignition OFF and the
  engine stopped
- **THEN** the ignition line SHALL come up at once and the reported state SHALL be ACC
- **AND** the starter SHALL engage on the first control loop at or after 4500 ms
- **AND** the cranking SHALL then end on engine start or after `CRANKING_TIMEOUT`, as before

#### Scenario: Ignition line already powered long enough
- **WHEN** the ignition has been in ACC for longer than the dwell and a crank is requested
- **THEN** the starter SHALL engage on the next control loop

#### Scenario: Stored value out of range
- **WHEN** the stored `dwell_ms` is outside 1000–8000
- **THEN** it SHALL be logged and ignored, the default SHALL be used, and the stored value SHALL NOT
  be rewritten

#### Scenario: New value applies at once
- **WHEN** a new dwell is saved while a crank is armed
- **THEN** the armed crank SHALL use the new value from the next control loop, without restarting
  the dwell clock

#### Scenario: Engine already running
- **WHEN** a crank is armed and the validated engine RPM is at or above
  `ENGINE_RUNNING_RPM_THRESHOLD`
- **THEN** the crank SHALL be suppressed and the state SHALL settle in IGNITION, whatever the dwell

### Requirement: Validated Engine RPM for Crank Control
The crank logic SHALL act only on an engine RPM the ECU (or, on the aztec buggy, the tacho) has
actually reported. `RelayController::update()` SHALL receive `crankRpm(dataValid, engineRPM)` —
the CAN engine RPM when the CAN data is valid, 0 otherwise — on the quad bike and the old board,
and the effective RPM `getEngineRpm()` on the aztec buggy. When the applied relay ignition state
goes from any other state to OFF, by any path (RC, GCS, web, fail-safe `allOff()`, starter
escalation), `VehicleController` SHALL clear the stored CAN engine RPM to 0
(`CANController::clearEngineRpm()`), leaving the CAN validity flag, its timestamp and every other
ECU field unchanged; the next ECU reply SHALL overwrite it.

#### Scenario: Crank right after switching the engine off at high RPM
- **WHEN** the engine runs at 1800 RPM, the RC ignition switch goes OFF and back to ON 1 s later
- **THEN** the stored engine RPM SHALL be 0 from the OFF transition
- **AND** the crank SHALL NOT be suppressed as "engine already running"
- **AND** the starter SHALL engage after the configured dwell

#### Scenario: CAN data invalid with a stale value
- **WHEN** the CAN data is invalid and the stored engine RPM is 1600
- **THEN** RelayController.update() SHALL receive 0
- **AND** an armed crank SHALL proceed to the starter after the dwell, and a running crank SHALL end
  only on a fresh reply at or above the threshold or on `CRANKING_TIMEOUT`

#### Scenario: Web START after an engine kill
- **WHEN** the engine was switched off at 1800 RPM and the web START is pressed within 5 s
- **THEN** it SHALL NOT be refused with "Engine already running"
- **AND** the starter SHALL NOT be released in the same loop as "Engine started!"

#### Scenario: Other ECU fields are untouched by the clear
- **WHEN** the ignition goes OFF
- **THEN** coolant temperature, MAP, module voltage, intake temperature and engine load SHALL keep
  their values and validity exactly as before this change

#### Scenario: Aztec effective RPM after OFF
- **WHEN** on the aztec buggy the ignition goes OFF with the engine running and the tacho times out
- **THEN** the effective RPM SHALL NOT fall back to a stale CAN value above 0
