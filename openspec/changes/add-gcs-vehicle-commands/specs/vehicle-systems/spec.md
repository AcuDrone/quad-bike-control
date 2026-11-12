## ADDED Requirements
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
  - ON with the engine running → `ACCEPTED` without a crank;
  - ON otherwise → `requestCrank()` exactly like a fresh RC transition into IGNITION (pre-crank
    dwell `ACC_PRECRANK_DWELL_MS`, starter at most `CRANKING_TIMEOUT`).
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
- **AND** the ignition line SHALL come up, the starter SHALL engage after the pre-crank dwell and
  SHALL stop on engine start or after `CRANKING_TIMEOUT`

#### Scenario: Relay board unavailable
- **WHEN** a GCS headlight command arrives while the relay expander is faulted
- **THEN** the ACK SHALL be `MAV_RESULT_FAILED`

### Requirement: Last-Change-Wins Arbitration of RC Channels and GCS Commands
RC channels (CH3 gear, CH4 ignition, CH6 headlight, CH7 diff lock) and GCS commands SHALL be equal
sources, and the most recent change SHALL win. The every-loop re-apply of these four functions from
the RC channels SHALL be removed; steering, throttle and brake SHALL still follow the RC channels
every loop.

- Per function the vehicle layer SHALL keep the last in-band decoded RC value. A value outside
  880–2160 µs (including 0) SHALL be treated as absent.
- An RC value SHALL be applied only when it differs from the stored value, or when no value is
  stored yet. An absent channel, a missing frame or a fail-safe SHALL NOT be a change and SHALL keep
  the stored value and the current vehicle state, which a GCS command may have set.
- Every time RC comes back in control — at boot, on every exit from the fail-safe source (owner
  decision D-A) and on every return from the web portal to MAVLink control, whether web control was
  released or the RC stream returned while a browser was connected (owner decision D-H) — the stored
  values and the pending RC gear SHALL have been cleared, so the first valid RC frame sets gear,
  ignition, headlight and diff lock again. They SHALL be cleared in every loop in which the input
  source is not MAVLink. RC ignition ON SHALL then call `requestCrank()` (suppressed when the engine
  runs). A GCS or web command given before RC came back in control SHALL NOT be restored.
- An RC gear change requested while the engine is not running, or refused by the speed interlock,
  SHALL stay pending and SHALL be applied once the engine runs and the transmission accepts it
  (today's retry, owner decision D-E). A GCS gear command that is accepted later SHALL cancel the
  pending RC gear; a rejected GCS gear command SHALL NOT. A later RC gear change SHALL replace it.
- RC SHALL still not select H.

#### Scenario: Command holds against a constant RC channel
- **WHEN** RC CH6 stays at 1100 µs (headlight OFF) and a GCS headlight ON command is accepted
- **THEN** the headlight SHALL stay on in every following loop while CH6 does not change

#### Scenario: RC edge wins over an earlier command
- **WHEN** a GCS diff-lock ON was accepted and CH7 then moves from 1100 µs to 1900 µs and back to
  1100 µs
- **THEN** the lock SHALL follow the RC channel: on, then off

#### Scenario: Channel loss is not a change
- **WHEN** a GCS ignition ON command was accepted and CH4 reads 0 µs in the following frames
- **THEN** the ignition SHALL stay ON
- **AND** when CH4 returns to the same in-band OFF value it had before, the ignition SHALL stay ON

#### Scenario: Fail-safe exit re-sync
- **WHEN** the RC stream is lost with no browser connected (fail-safe, all relays off) and returns
  with CH4 in the ON band
- **THEN** the first valid frame SHALL call `requestCrank()` and set gear, headlight and lock from
  their channels, exactly as at boot

#### Scenario: Return from web control re-sync
- **WHEN** the web portal was the input source and switched the headlight on and the ignition off,
  and MAVLink control returns (web control released, or the RC stream returns while a browser is
  connected) with CH6 in the OFF band and CH4 in the ON band
- **THEN** the first valid frame SHALL switch the headlight off and SHALL call `requestCrank()`
- **AND** gear and diff lock SHALL be set from CH3 and CH7 in the same frame

#### Scenario: RC-only vehicle unchanged
- **WHEN** the ground station never sends a vehicle command
- **THEN** gear, ignition (including the auto-crank on a fresh ON), headlight and diff lock SHALL
  follow the RC channels as before this change, for every in-band value

#### Scenario: Pending RC gear superseded by the GCS
- **WHEN** CH3 moves to L with the engine stopped, the engine starts, and before the transmission
  accepts L a GCS gear N command is accepted
- **THEN** the pending RC gear L SHALL be cancelled and the target SHALL be N

### Requirement: GCS Handbrake Command
On builds with the parking brake (`aztec`, X6 driver initialised at boot, capability bit5 set), a
GCS handbrake command (`param1` 5) SHALL call `commandParkBrake(APPLY` for `param2` 1 or `RELEASE`
for `param2` 0`, ParkBrakeSource::GCS)` and SHALL add no interlock of its own: it SHALL be accepted
in every input source, ignition state and fail-safe (never reason 3). The result SHALL be mapped:
`STARTED`, `STARTED_SPEED_UNKNOWN`, `REVERSING`, `ALREADY_RUNNING` → `MAV_RESULT_ACCEPTED`;
`REFUSED_MOVING` → `MAV_RESULT_TEMPORARILY_REJECTED` reason 4; `REFUSED_DRIVER`, `REFUSED_NVS` →
`MAV_RESULT_FAILED` (owner decision D-C). The existing parking-brake behaviour — apply speed limit,
drive-away release, throttle hold, `EFI_STATUS.pt_compensation` bits 2 and 3 — SHALL apply to a
GCS command exactly as to a web command and SHALL NOT change. The command SHALL be logged by the
existing `[PBRAKE] … accepted (gcs)` line.

On builds without the parking brake (`master`, where X6 is the steering VESC) and on the buggy when
the X6 driver did not initialise at boot, capability bit5 SHALL be clear and a handbrake command
SHALL be answered `MAV_RESULT_UNSUPPORTED`.

#### Scenario: Apply at standstill
- **WHEN** the buggy is stopped and a GCS handbrake ON command arrives in fail-safe
- **THEN** the apply run SHALL start, the ACK SHALL be `ACCEPTED` and the log SHALL read
  `[PBRAKE] Apply accepted (gcs)`

#### Scenario: Apply while moving
- **WHEN** a GCS handbrake ON command arrives at 5 km/h
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 4 and the actuator SHALL not move

#### Scenario: Save failure
- **WHEN** the moving state cannot be written to NVS for an apply
- **THEN** the ACK SHALL be `MAV_RESULT_FAILED`

#### Scenario: Quad bike
- **WHEN** the quad bike receives a handbrake command
- **THEN** the ACK SHALL be `MAV_RESULT_UNSUPPORTED` and nothing on X6 SHALL change
