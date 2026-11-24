## ADDED Requirements

### Requirement: Headlight Washer Pulse
Both vehicles SHALL drive a headlight washer pump ("омивач фар") from Relay_8 alone
(`RELAY_SET_WASHER = RELAY_N(8)`; PCA9557 board IO1 / connector X11, mask `0x02`; XL9535 board P0.7,
mask `0x80`). Relay_7 SHALL stay in `RELAY_SET_RESERVED` and SHALL NOT be used by any function. The
washer SHALL be a timed pulse owned by `RelayController`: one wash request energizes Relay_8 for the
configured pulse length (see "Configurable Headlight Washer Pulse Length") and `RelayController`
SHALL release it by itself; the operator never has to switch it off. The pulse SHALL be timed
non-blocking from `millis()` in `RelayController::update()` with a wrap-safe unsigned subtraction,
and the pulse length SHALL be latched when the pulse starts. Relay_8 SHALL be part of the same
output shadow byte and the same verified whole-port write as every other function. No RC / MAVLink
servo channel SHALL drive the washer; it SHALL be requested only by the web `wash_headlights`
command and the GCS washer function (`MAV_CMD_USER_2` `param1` 6).

A wash request SHALL be accepted only when the washer is fitted (see "Headlight Washer Fitted
Switch"), the relay board is available, the applied ignition state is `ACC` or `IGNITION` and no
crank is armed or running. It SHALL be refused, with nothing written, checked in this order:
- washer fitted switch off → "washer not fitted - enable it in Calibration";
- relay board not detected or faulted → "relay board unavailable";
- ignition `OFF` → "ignition is OFF";
- a crank armed (pre-crank dwell) or `CRANKING` → "engine start in progress".

A running pulse SHALL be ended at once, with Relay_8 released in the same write as the change that
caused it, when: the ignition goes to `OFF` by any path; a crank is armed (`requestCrank()`) or the
state enters `CRANKING` (including the web START button); `allOff()` runs (fail-safe); a relay port
write does not verify, or the starter escalation runs; a GCS washer stop (`param2` 0) is accepted;
the washer fitted switch is turned off.
A pulse SHALL NOT be resumed after a relay-board recovery. A wash request while a pulse is running
SHALL restart the pulse with the pulse length in force at that moment. Starting a pulse SHALL first
release every web test relay (the relay test can only be on with the ignition `OFF`, so this is
defensive).

#### Scenario: Wash with the ignition in ACC
- **WHEN** the ignition is `ACC`, no crank is armed, the pulse length is 3000 ms and a wash request
  arrives
- **THEN** Relay_8 SHALL be energized in one verified whole-port write
- **AND** after 3000 ms `update()` SHALL release Relay_8 without any further command

#### Scenario: Wash with the engine running
- **WHEN** the ignition is `IGNITION` (engine running, or after a start attempt that ended) and a
  wash request arrives
- **THEN** the pulse SHALL start exactly as in `ACC`

#### Scenario: Refused while not fitted
- **WHEN** the washer fitted switch is off and a wash request arrives with the ignition `ACC`
- **THEN** it SHALL be refused with the reason "washer not fitted - enable it in Calibration" and
  Relay_8 SHALL stay off

#### Scenario: Refused with the ignition OFF
- **WHEN** a wash request arrives while the ignition is `OFF`
- **THEN** it SHALL be refused with the reason "ignition is OFF" and Relay_8 SHALL stay off

#### Scenario: Refused during an engine start
- **WHEN** a wash request arrives during the pre-crank dwell or while the starter is cranking
- **THEN** it SHALL be refused with the reason "engine start in progress"

#### Scenario: Refused while the relay board is unavailable
- **WHEN** a wash request arrives while no relay board was detected or the driver is faulted
- **THEN** it SHALL be refused with the reason "relay board unavailable" and nothing SHALL be written

#### Scenario: Ignition OFF aborts the pulse
- **WHEN** a pulse is running and the ignition is switched `OFF` from the web, the RC, the GCS or by
  the fail-safe
- **THEN** Relay_8 SHALL be released in the same port write as the ignition relays

#### Scenario: A crank aborts the pulse
- **WHEN** a pulse is running and an RC or GCS ignition ON arms a crank, or the web START button
  enters `CRANKING`
- **THEN** Relay_8 SHALL be released immediately, before the starter can engage

#### Scenario: Restart while running
- **WHEN** a pulse started 2000 ms ago with a 3000 ms length and a new wash request arrives
- **THEN** the pulse SHALL run 3000 ms from the new request (5000 ms in total)

#### Scenario: Relay fault releases the washer
- **WHEN** a relay port write does not verify while a pulse is running
- **THEN** the washer SHALL be marked off and Relay_8 SHALL be cleared from the shadow byte
- **AND** after the board recovers Relay_8 SHALL stay off until a new wash request

#### Scenario: Timer wrap-around
- **WHEN** a pulse starts at `millis()` = `0xFFFFF000` with a 3000 ms length
- **THEN** it SHALL end 3000 ms later although `millis()` has wrapped

#### Scenario: Relay_7 never used
- **WHEN** any function, ignition state or washer pulse is applied
- **THEN** Relay_7 SHALL be written off; only the web relay test (ignition `OFF`) can energize it

### Requirement: Headlight Washer Fitted Switch
Whether a washer pump is installed SHALL be a persisted per-vehicle runtime setting ("Омивач
встановлено" / "Washer fitted"), NOT a compile-time flag: NVS namespace `"washer"`, key `fitted`
(bool), a missing key reading as `false` (not fitted), owned by `VehicleController` and handed to
`RelayController` through a RAM-only `setWasherFitted()`. `RELAY_SET_WASHER` SHALL stay Relay_8 at
compile time on both vehicles; the switch only enables the function, so one firmware serves a
vehicle with and without the pump and no per-vehicle build is needed (unlike
`RELAY_WHEEL_LOCK_FITTED`, which is tied to each branch's harness). A change SHALL be persisted first
and applied at once, without a reboot. While the switch is off: no wash request SHALL be accepted
(web refusal, GCS `UNSUPPORTED` through the cleared capability bit6), the washer function SHALL never
energize Relay_8 and the `EFI_STATUS` washer bit SHALL stay 0. Turning the switch off while a pulse
runs SHALL end it at once (journal `washer_abort why=not_fitted`). The web relay test SHALL NOT
depend on the switch: Relay_8 stays testable with the ignition `OFF`, so the wiring can be checked
before the washer is enabled.

#### Scenario: Default after the update
- **WHEN** firmware 1.6.3 boots on a vehicle whose namespace `"washer"` has no `fitted` key
- **THEN** the washer SHALL be not fitted, the GCS capability bit6 SHALL be clear and every wash
  request SHALL be refused

#### Scenario: Enable the washer
- **WHEN** the operator turns the switch on
- **THEN** `fitted` = true SHALL be stored, wash requests SHALL be accepted under the other
  interlocks without a reboot, and the setting SHALL survive a reboot

#### Scenario: Switch turned off mid-pulse
- **WHEN** the switch is turned off while a pulse runs
- **THEN** Relay_8 SHALL be released at once and `washer_abort why=not_fitted` SHALL be journaled

#### Scenario: Relay test independent of the switch
- **WHEN** the switch is off and the operator test-energizes Relay_8 with the ignition `OFF`
- **THEN** the relay test SHALL energize Relay_8 exactly as before

### Requirement: Configurable Headlight Washer Pulse Length
The headlight-washer pulse length SHALL be a persisted per-vehicle setting owned by
`VehicleController`: NVS namespace `"washer"` (shared with the `fitted` switch), key `pulse_ms`, uint16 ms, range
`WASHER_PULSE_MS_MIN` 2000 – `WASHER_PULSE_MS_MAX` 20000, default `WASHER_PULSE_MS_DEFAULT` 3000. It
SHALL be loaded at boot (a missing key reads as the default; a stored out-of-range value SHALL be
logged and ignored — the default is used and the stored value is not rewritten) and handed to
`RelayController` through a RAM-only setter `setWasherPulseMs()`, which replaces an out-of-range
value by the default. A new value SHALL be persisted first and applied only once NVS holds it; it
SHALL apply from the next wash request (a running pulse keeps the length it started with).

#### Scenario: First boot
- **WHEN** the namespace `"washer"` has never been written
- **THEN** the pulse length SHALL be 3000 ms

#### Scenario: Stored value out of range
- **WHEN** NVS holds `pulse_ms` = 1000 or 25000
- **THEN** a log line SHALL name the value and the range, 3000 ms SHALL be used and NVS SHALL NOT be
  rewritten

#### Scenario: Change while a pulse runs
- **WHEN** the pulse length is changed from 3000 ms to 20000 ms while a 3000 ms pulse runs
- **THEN** the running pulse SHALL end at 3000 ms and the next wash SHALL last 20000 ms

## MODIFIED Requirements
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
- headlight washer pump = Relay_8 (`RELAY_SET_WASHER`; `0b00000010` / `0b10000000`), a timed pulse
  (see "Headlight Washer Pulse");
- Relay_7 spare (`RELAY_SET_RESERVED`; `0b00000001` / `0b01000000`), never used by a function.

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
- **THEN** ignition, starter, light and the headlight washer SHALL be switched off
- **AND** the wheel lock SHALL hold its last commanded state

#### Scenario: Relay test respects the wheel lock
- **WHEN** a web relay test asks to energize any relay while the wheel lock is engaged
- **THEN** the request SHALL be refused with a reason naming the wheel lock
- **AND WHEN** a lock command arrives while test relays are on
- **THEN** all test relays SHALL be switched off before the lock is applied
- **AND** the 2 s starter test cap SHALL apply to Relay_3

### Requirement: GCS Vehicle Command Interlocks
The vehicle layer SHALL execute the GCS vehicle commands (gear, ignition, headlight, diff lock,
headlight washer)
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
- **Headlight washer** (`param1` 6):
  - while the washer fitted switch is off, capability bit6 is clear and `param1` 6 SHALL be
    answered `MAV_RESULT_UNSUPPORTED` by the decoder (wash and stop alike), before any interlock;
  - wash (`param2` 1) not under MAVLink control → reason 3;
  - wash with the relay board unavailable → `MAV_RESULT_FAILED`;
  - wash with the ignition `OFF`, a crank armed or `CRANKING` → `TEMPORARILY_REJECTED` reason 5
    (ignition not on / engine start in progress);
  - otherwise the same `RelayController` wash request the web `wash_headlights` command uses
    (Relay_8, see "Headlight Washer Pulse") → `ACCEPTED`, also when a pulse was already running (it
    restarts);
  - stop (`param2` 0) SHALL be accepted in every input source (safe direction, like ignition OFF),
    SHALL end a running pulse and SHALL be `ACCEPTED` also when no pulse runs.
- With the relay board unavailable, an ignition, headlight, diff-lock or headlight-washer command
  SHALL be answered `MAV_RESULT_FAILED` and SHALL change nothing.

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
- **THEN** gear, ignition ACC / ON, diff lock and headlight-washer wash commands SHALL be answered
  `TEMPORARILY_REJECTED` with reason 3
- **AND** headlight, ignition OFF and headlight-washer stop commands SHALL be accepted and executed

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

#### Scenario: Headlight washer from the GCS
- **WHEN** a GCS washer command (`param1` 6, `param2` 1) arrives under MAVLink control with the
  washer fitted and the ignition `ACC` or `IGNITION`
- **THEN** the ACK SHALL be `ACCEPTED` and Relay_8 SHALL be energized for the configured pulse length

#### Scenario: Headlight washer with the ignition OFF
- **WHEN** a GCS washer command (`param2` 1) arrives under MAVLink control with the ignition `OFF`
- **THEN** the ACK SHALL be `TEMPORARILY_REJECTED` with reason 5 and Relay_8 SHALL stay off

#### Scenario: Headlight washer not fitted
- **WHEN** a GCS washer command arrives while the washer fitted switch is off
- **THEN** the ACK SHALL be `MAV_RESULT_UNSUPPORTED` and Relay_8 SHALL stay off

