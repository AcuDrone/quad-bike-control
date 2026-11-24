## ADDED Requirements
### Requirement: Lock Not Driven by an RC Channel
The lock function (`setWheelLock()`, "wheel lock" / "diff lock" in the API) SHALL NOT be driven by
any RC / MAVLink servo channel. MAVLink servo channel 7 SHALL NOT be read by the vehicle layer, and
its value — in band or out of band, changing or constant, on any frame including the first valid
frame after a re-sync — SHALL have no effect on the lock or on any other vehicle function. The
lock SHALL be changed only by the web `set_wheel_lock` command and by the GCS `MAV_CMD_USER_2` lock
function (`param1` 4), each with its existing checks. A lock state set by either SHALL be held until
the next web or GCS lock command; the fail-safe all-off SHALL keep it, as before. At boot the lock
SHALL be released (`RelayController::begin()` writes the port all-off with the lock state false; the
lock state is not persisted), and it SHALL stay released until a web or GCS lock command.

#### Scenario: Channel 7 switch moved
- **WHEN** MAVLink is in control with the lock released and servo channel 7 moves from 1100 µs to
  1900 µs and back
- **THEN** the lock SHALL stay released in every loop, `wheel_lock_on` SHALL stay `false` and
  `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL stay clear
- **AND** no lock relay write SHALL be made

#### Scenario: Engaged lock survives channel 7 at the opposite position
- **WHEN** the lock was engaged from the web or by an accepted GCS lock ON, and servo channel 7
  reads 1100 µs, then 1900 µs, then 1100 µs
- **THEN** the lock SHALL stay engaged

#### Scenario: Gaining MAVLink control does not touch the lock
- **WHEN** the lock was engaged from the web while the web portal was in control, and MAVLink
  control returns (web control released, fail-safe exit, or the RC stream returns while a browser
  is connected) with servo channel 7 at 1100 µs
- **THEN** the first valid frame SHALL set gear, ignition and headlight from CH3, CH4 and CH6 as
  before
- **AND** the lock SHALL stay engaged

#### Scenario: Boot with channel 7 high
- **WHEN** the board boots and the first valid frames carry servo channel 7 at 1900 µs
- **THEN** the lock SHALL be released and SHALL stay released until a web or GCS lock command

#### Scenario: Web and GCS still drive the lock
- **WHEN** a web `set_wheel_lock` command, or a GCS lock command accepted under MAVLink control,
  arrives on a build where the lock is fitted
- **THEN** the lock SHALL be set as commanded and reported in `wheel_lock_on` and
  `EFI_STATUS.pt_compensation` bit0, as before this change

## MODIFIED Requirements
### Requirement: Front-Wheel Lock Not Fitted
The aztec buggy SHALL have no relay behind the front-wheel lock function (`RELAY_WHEEL_LOCK_FITTED =
0`). The wheel-lock plumbing (web `set_wheel_lock` command, GCS lock command, telemetry fields)
SHALL be kept so a later change can replace it with the buggy's three locks, but a lock request SHALL
never be reported as applied. No RC / MAVLink servo channel drives the lock (see "Lock Not Driven by
an RC Channel").

#### Scenario: Lock request is rejected
- **WHEN** `VehicleController::setWheelLock(true)` is called from the web or the GCS path
- **THEN** `RelayController::setWheelLock()` SHALL return `false` without writing the relay port
- **AND** the first rejection after boot SHALL be logged; later ones SHALL NOT flood the log
- **AND** `getWheelLock()` SHALL return `false`

#### Scenario: Release request is a no-op
- **WHEN** `setWheelLock(false)` is called
- **THEN** it SHALL return `true` without writing the relay port and without logging

#### Scenario: Fitted state is queryable
- **WHEN** `VehicleController::isWheelLockFitted()` or `RelayController::isWheelLockFitted()` is called
- **THEN** it SHALL return `false`

### Requirement: Quad Bike Relay Assignment With Fitted Front-Wheel Lock
On the quad bike (`master`) build the relay board SHALL use the quad assignment, defined in
`Constants.h` as relay-number sets and mapped at `begin()` through the detected board's Relay_N → bit
table (masks shown for the PCA9557 / XL9535 board):
- ignition/ECU line = Relay_1 + Relay_2, paralleled (`RELAY_SET_IGNITION`; mask `0b00110000` /
  `0b00000011`);
- starter = Relay_3 (`0b01000000` / `0b00000100`);
- front light = Relay_4 (`0b10000000` / `0b00001000`);
- front-wheel lock = Relay_5 + Relay_6, paralleled (`RELAY_SET_WHEEL_LOCK`; `0b00001100` /
  `0b00110000`);
- Relay_7 / Relay_8 spare (`RELAY_SET_RESERVED`; `0b00000011` / `0b11000000`).

The front-wheel lock SHALL be fitted (`RELAY_WHEEL_LOCK_FITTED` = 1). It replaces, for this build
only, the aztec buggy's "Front-Wheel Lock Not Fitted" behaviour. It SHALL be driven by the web
`set_wheel_lock` command and the GCS lock command only, never by an RC / MAVLink servo channel (see
"Lock Not Driven by an RC Channel").

#### Scenario: Wheel lock from the web or the GCS
- **WHEN** a web `set_wheel_lock` true command, or a GCS lock ON command accepted under MAVLink
  control, arrives
- **THEN** Relay_5 and Relay_6 SHALL both be energized, `getWheelLock()` SHALL return `true`,
  telemetry `wheel_lock_on` SHALL be `true` and `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL be set
- **AND WHEN** a web `set_wheel_lock` false command or an accepted GCS lock OFF command arrives
- **THEN** both relays SHALL be released and the lock SHALL report unlocked

#### Scenario: Wheel lock from MAVLink channel 7
- **WHEN** MAVLink servo channel 7 is above 1520 µs (e.g. 2000), or returns to 1000 µs
- **THEN** Relay_5 and Relay_6 SHALL keep their commanded state, `getWheelLock()`,
  `wheel_lock_on` and `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL NOT change (channel 7 no longer drives
  the lock)

#### Scenario: Same relays on either board
- **WHEN** the quad build runs with the PCA9557 board or with the XL9535 board
- **THEN** the same relay numbers SHALL carry the same functions, and only the port masks SHALL differ

#### Scenario: Wheel lock reported as fitted
- **WHEN** telemetry is formatted on the quad build
- **THEN** `wheel_lock_fitted` SHALL be `true` and the web wheel-lock toggle SHALL be usable

#### Scenario: Fail-safe all-off keeps the wheel lock
- **WHEN** `RelayController::allOff()` runs
- **THEN** ignition, starter and light SHALL be switched off
- **AND** the wheel lock SHALL hold its last commanded state

#### Scenario: Relay test respects the wheel lock
- **WHEN** a web relay test asks to energize any relay while the wheel lock is engaged
- **THEN** the request SHALL be refused with a reason naming the wheel lock
- **AND WHEN** a lock command arrives while test relays are on
- **THEN** all test relays SHALL be switched off before the lock is applied
- **AND** the 2 s starter test cap SHALL apply to Relay_3

### Requirement: GCS Vehicle Command Interlocks
The vehicle layer SHALL execute the GCS vehicle commands (gear, ignition, headlight, diff lock)
through the same controllers the RC and web paths use and SHALL be the arbiter of every interlock.
"Not under MAVLink control" SHALL mean `currentInputSource_ != InputSource::MAVLINK` OR
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
- **Diff lock**: not under MAVLink control → reason 3; otherwise the same `setWheelLock()` path the
  web `set_wheel_lock` command uses (quad bike: Relay_5 + Relay_6; buggy: Relay_4 + Relay_5 +
  Relay_6), reported in `EFI_STATUS.pt_compensation` bit0. No RC channel drives the lock (see "Lock
  Not Driven by an RC Channel"), so an accepted diff-lock command holds until the next web or GCS
  lock command.
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

#### Scenario: Diff lock holds against channel 7
- **WHEN** a GCS diff-lock ON command is accepted under MAVLink control and servo channel 7 then
  moves from 1900 µs to 1100 µs
- **THEN** the lock SHALL stay engaged

#### Scenario: Relay board unavailable
- **WHEN** a GCS headlight command arrives while the relay expander is faulted
- **THEN** the ACK SHALL be `MAV_RESULT_FAILED`

### Requirement: Last-Change-Wins Arbitration of RC Channels and GCS Commands
RC channels (CH3 gear, CH4 ignition, CH6 headlight) and GCS commands SHALL be equal sources, and
the most recent change SHALL win. The every-loop re-apply of these three functions from the RC
channels SHALL be removed; steering, throttle and brake SHALL still follow the RC channels every
loop. The diff lock is not an RC function: CH7 SHALL NOT be read and the lock SHALL be set only by
the web and GCS commands (see "Lock Not Driven by an RC Channel").

- Per function the vehicle layer SHALL keep the last in-band decoded RC value. A value outside
  880–2160 µs (including 0) SHALL be treated as absent.
- An RC value SHALL be applied only when it differs from the stored value, or when no value is
  stored yet. An absent channel, a missing frame or a fail-safe SHALL NOT be a change and SHALL keep
  the stored value and the current vehicle state, which a GCS command may have set.
- Every time RC comes back in control — at boot, on every exit from the fail-safe source (owner
  decision D-A) and on every return from the web portal to MAVLink control, whether web control was
  released or the RC stream returned while a browser was connected (owner decision D-H) — the stored
  values and the pending RC gear SHALL have been cleared, so the first valid RC frame sets gear,
  ignition and headlight again; the lock SHALL NOT be touched. They SHALL be cleared in every loop in
  which the input source is not MAVLink. RC ignition ON SHALL then call `requestCrank()` (suppressed
  when the engine runs). A GCS or web command given before RC came back in control SHALL NOT be
  restored.
- An RC gear change requested while the engine is not running, or refused by the speed interlock,
  SHALL stay pending and SHALL be applied once the engine runs and the transmission accepts it
  (today's retry, owner decision D-E). A GCS gear command that is accepted later SHALL cancel the
  pending RC gear; a rejected GCS gear command SHALL NOT. A later RC gear change SHALL replace it.
- RC SHALL still not select H.

#### Scenario: Command holds against a constant RC channel
- **WHEN** RC CH6 stays at 1100 µs (headlight OFF) and a GCS headlight ON command is accepted
- **THEN** the headlight SHALL stay on in every following loop while CH6 does not change

#### Scenario: RC edge wins over an earlier command
- **WHEN** a GCS headlight ON was accepted and CH6 then moves from 1100 µs to 1900 µs and back to
  1100 µs
- **THEN** the headlight SHALL follow the RC channel: on, then off

#### Scenario: CH7 is not an arbitrated channel
- **WHEN** a GCS diff-lock ON was accepted and CH7 then moves from 1100 µs to 1900 µs and back to
  1100 µs
- **THEN** the lock SHALL stay on

#### Scenario: Channel loss is not a change
- **WHEN** a GCS ignition ON command was accepted and CH4 reads 0 µs in the following frames
- **THEN** the ignition SHALL stay ON
- **AND** when CH4 returns to the same in-band OFF value it had before, the ignition SHALL stay ON

#### Scenario: Fail-safe exit re-sync
- **WHEN** the RC stream is lost with no browser connected (fail-safe, all relays off except the
  lock) and returns with CH4 in the ON band
- **THEN** the first valid frame SHALL call `requestCrank()` and set gear and headlight from their
  channels, exactly as at boot
- **AND** the lock SHALL keep the state it had before the loss

#### Scenario: Return from web control re-sync
- **WHEN** the web portal was the input source and switched the headlight on and the ignition off,
  and MAVLink control returns (web control released, or the RC stream returns while a browser is
  connected) with CH6 in the OFF band and CH4 in the ON band
- **THEN** the first valid frame SHALL switch the headlight off and SHALL call `requestCrank()`
- **AND** gear SHALL be set from CH3 in the same frame
- **AND** the lock SHALL keep the state the web portal left it in, whatever CH7 reads

#### Scenario: RC-only vehicle unchanged
- **WHEN** the ground station never sends a vehicle command
- **THEN** gear, ignition (including the auto-crank on a fresh ON) and headlight SHALL follow the RC
  channels as before this change, for every in-band value
- **AND** the lock SHALL change only on a web command

#### Scenario: Pending RC gear superseded by the GCS
- **WHEN** CH3 moves to L with the engine stopped, the engine starts, and before the transmission
  accepts L a GCS gear N command is accepted
- **THEN** the pending RC gear L SHALL be cancelled and the target SHALL be N
