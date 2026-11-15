# Design: add-gcs-vehicle-commands

## Context
- **Source of truth.** The GCS-side brief "ESP32 vehicle-control commands" (2026-11-13, GCS
  `design.md` D17) plus the owner decisions D-A … D-E of 2026-11-13, recorded below as decided.
- **Code today (verified on `master` 40b3009 and `aztec` 85b476c).**
  - `src/MavlinkInterface.cpp` `handleCommandLong()` (master ~478): strict addressing (exactly
    `MAVLINK_SYSTEM_ID` 1 / `MAVLINK_COMPONENT_ID` 25, broadcasts dropped silently, no ACK);
    `MAV_CMD_USER_1` = trip reset (magic `param1` 1 → `ACCEPTED`, else `DENIED`), latched for the
    vehicle layer; every other command → `MAV_RESULT_UNSUPPORTED`, logged `[MAV] command N
    UNSUPPORTED`. Only `param1` reaches the handler. The ACK is addressed to the requester, with
    `progress` 0 and `result_param2` 0.
  - `include/MavlinkInterface.h` is kept free of mavlink headers (spec rule); handlers take
    decoded scalars.
  - `sendNamedInt()` (master ~1002) zero-pads the 10-byte name. `STEER_MODE` is today the only
    `NAMED_VALUE_INT`, and the `mavlink-interface` spec says so.
  - `VehicleController::processMavlinkCommands()` (master ~708, aztec ~924) runs only while the
    source is `InputSource::MAVLINK`, returns early on `!isSignalValid()`, and re-applies every
    loop: gear (`getGear()`, dedup guard `lastCommandedGear_`, only while `isEngineRunning()`,
    retried until `setGear()` accepts), ignition (`setIgnitionState(OFF/ACC)` every loop,
    `requestCrank()` only on a fresh transition into IGNITION via `previousIgnitionState_`),
    light (`setFrontLight()` every loop) and lock (`setWheelLock()` every loop).
  - RC decode (`MavlinkInterface::getGear()/getIgnitionState()/getFrontLight()/getWheelLock()`):
    gear L 880–1306, N 1307–1733, R 1734–2160, anything else → **N**; ignition OFF 880–1200, ACC
    1201–1520, ON 1521–2160, anything else → **OFF**; light / lock > 1520 = on. No H on RC.
  - `applyFailsafe()` (master ~630, aztec ~846) on **entry** into `InputSource::FAILSAFE`:
    steering centre, throttle idle, brake stop, transmission stop, `relayController_.allOff()`
    (ignition, starter, light off; the lock deliberately holds), persist counters, reset
    `previousIgnitionState_ = OFF` and `lastCommandedGear_ = UNKNOWN`. That reset is what
    re-applies the RC state, including a re-crank, when the link returns.
  - Source selection (`TelemetryManager::determineInputSource()`): web control latched and a
    browser connected → `WEB`; else `isSignalValid()` → `MAVLINK`; else a browser connected →
    `WEB`; else `FAILSAFE`. The source starts as `FAILSAFE` at boot.
  - `RelayController` (both branches): `IgnitionState` OFF / ACC / IGNITION / CRANKING. ACC and
    IGNITION drive the same relays; `requestCrank()` brings the line up to ACC if it was OFF and
    arms a dwell-gated crank (`crankArmed_`, private) — while armed the reported state is ACC;
    `update()` engages CRANKING after `ACC_PRECRANK_DWELL_MS` (2000), suppresses the crank when
    RPM ≥ `ENGINE_RUNNING_RPM_THRESHOLD` (1500), and returns to IGNITION on engine start or after
    `CRANKING_TIMEOUT` (5000). Every setter is a logged no-op when the expander is unavailable;
    `getIgnitionState()` then reports OFF.
  - `RELAY_WHEEL_LOCK_FITTED` is **1 on both branches** (quad: Relay_5 + Relay_6; buggy: Relay_4 +
    Relay_5 + Relay_6). The `RelayController.h` header comment and the `mavlink-interface` /
    `vehicle-systems` spec text that call the lock "not fitted on aztec" are stale.
  - `TransmissionController`: gears R / N / H / L on both branches (master: one opto line per gear
    incl. H; aztec: 3-bit gear code incl. `GEAR_CODE_HIGH`); the web accepts "H" on both.
    `setGear()` returns false for UNKNOWN, not initialised, refused by `canChangeGear()` (private)
    and "already at target while idle". `canChangeGear()` **always allows N**; other gears are
    refused above `TRANS_SPEED_INTERLOCK_THRESHOLD_MS` (1.4 m/s) by the hall sensor, or by the CAN
    fallback policy when the sensor has not pulsed.
  - `aztec` only: `VehicleController::commandParkBrake(ParkBrakeRequest, ParkBrakeSource)`
    (~1192) logs `[PBRAKE] Apply|Release accepted (gcs|web)`; `ParkBrakeSource::GCS` is reserved
    for this change. `ParkBrakeController::isDriverOk()` = X6 driver initialised at boot.
    `REFUSED_NVS` is returned only for APPLY (a release with a failed write proceeds and is
    `STARTED` with `stateNotSaved`).
  - `include/Constants.h`: `FIRMWARE_VERSION` "1.5.28" (master) / "1.5.29" (aztec),
    `BOARD_NAME` "Control_v0", `MAVLINK_CMD_PARAM_EPSILON` 0.01f, `MAVLINK_CMD_TIMEOUT_MS` 500.
    Relay board: `RelayBoardKind` NONE 0 / PCA9557 1 / XL9535 2, detected once at boot and
    latched; the NVS key `relay/board` stores the *setting* (AUTO 0 / PCA9557 1 / XL9535 2).

## Goals / Non-Goals
- Goals:
  - GCS commands for gear, ignition, headlight, diff lock, and (aztec) handbrake, each answered
    with a `COMMAND_ACK` that tells the truth about what the vehicle did;
  - capability discovery, so an old firmware (`UNSUPPORTED`, no `QB_CAPS`) shows nothing new in
    the clients;
  - RC and GCS as equal sources, the most recent change winning;
  - RC-only operation unchanged.
- Non-Goals:
  - ARM/DISARM (autopilot, comp 1);
  - any change to fail-safe itself, the trip reset, `EFI_STATUS`, the parking-brake controller;
  - a web view of GCS commands, journal entries, a GCS command retry;
  - an RC channel for H or for the handbrake.

## Decisions

### D1. Pure law in `include/VehicleCommandLaw.h`
Header-only, no Arduino, natively tested (`test/test_vehicle_cmd_law`), the `*Law.h` pattern:
- `VehicleCmdFunction` (QUERY 0 … HANDBRAKE 5) and `VehicleCmdResult` (numeric values equal to
  `MAV_RESULT_ACCEPTED` 0, `TEMPORARILY_REJECTED` 1, `DENIED` 2, `UNSUPPORTED` 3, `FAILED` 4 —
  `static_assert`ed against the mavlink enum in `MavlinkInterface.cpp`) and the reason codes
  (`VCMD_REASON_ENGINE_OFF` 1, `_SPEED` 2, `_FAILSAFE` 3, `_PBRAKE_SPEED` 4);
- `vehicleCmdDecode(param1, param2, capsMask)` → query / function + integer value / UNSUPPORTED /
  DENIED (D3);
- the per-function interlock decisions as pure functions of booleans (source is MAVLink, engine
  running, speed interlock clear, transmission already at target, relays available, …);
- the RC edge tracker (D6) and the absent-aware RC decoders;
- `vehicleCapsMask(…)`, `qbCapsValue(mask, proto)`, `firmwareVersionCode("1.5.30")` (constexpr
  parse, 10530), `hardwareVersionCode(board, vehicle, relayBoard)`, `ignStateCode(relayState,
  crankPending)`;
- on `aztec` only: `vehicleCmdParkBrakeAck(ParkBrakeResult)` (includes `ParkBrakeLaw.h`).

### D2. Command path: synchronous vehicle-command sink, ACK sent at once
- A small abstract interface `VehicleCommandSink` (declared in `include/VehicleCommandLaw.h` or
  `MavlinkInterface.h`, no mavlink types) with `VehicleCmdOutcome executeVehicleCommand(
  VehicleCmdFunction fn, int32_t value)`. `VehicleController` implements it; `main.cpp` registers
  it with `mavlinkInterface.setVehicleCommandSink(&vehicleController, capsMask, hwVersion)` after
  `vehicleController.begin()` (capabilities and hardware version are fixed at boot).
- `handleCommandLong()` gets `param2` too. For `MAV_CMD_USER_2` it decodes (D3); queries are
  answered in the transport from the registered mask / version; a valid function command is
  executed through the sink in the same call and the outcome is ACKed immediately.
- No sink registered → every `MAV_CMD_USER_2` answers `UNSUPPORTED` (old behaviour).
- The trip reset keeps its latch-and-consume path unchanged.
- The sink runs inside `mavlinkInterface.update()`, i.e. before `setInputSource()` of this loop.
  It therefore checks both the stored source and `mavlink_.isSignalValid()` (D4), so a source that
  is one loop stale cannot let a command through.
- A command applied here is not overwritten later in the same loop: `processMavlinkCommands()`
  applies RC only on an edge (D6).
- Alternative considered: latch the command and ACK from `VehicleController::update()`. Rejected —
  it needs a queue (several commands can arrive in one RX drain) and an overflow answer that the
  protocol has no code for; the synchronous call needs neither.

### D3. Decode order and ACK results
1. Not exactly 1/25 → dropped, no ACK (unchanged).
2. `command` not USER_1 / USER_2 → `UNSUPPORTED` (unchanged).
3. USER_2: `param1` not finite, or not within `MAVLINK_CMD_PARAM_EPSILON` of an integer 0…5 →
   `UNSUPPORTED` (unknown function). Function 1…5 whose capability bit is clear → `UNSUPPORTED`
   (gear: bit0 for R/N/L, bit1 for H — a function with bit0 clear is unsupported as a whole).
4. `param2` not finite, not within epsilon of an integer, or not in the function's table →
   `DENIED`. For the query, a `param2` other than 0 / 1 / 2 → `DENIED`. Gear H with bit1 clear →
   `UNSUPPORTED`.
5. Interlocks (D5, D8) → `ACCEPTED`, `TEMPORARILY_REJECTED` + reason, or `FAILED`.
- `param3..7` are ignored. `progress` = 0 except the caps query (= protocol version 1);
  `result_param2` = reason for `TEMPORARILY_REJECTED`, the query value for a query, else 0.
- One debug line per USER_2 (`DebugFeature::MAVLINK`): function, value, result, reason, requester.

### D4. What "failsafe" means, and what reason 3 keys on
- In code there are two different things:
  - `!mavlink_.isSignalValid()` — no `SERVO_OUTPUT_RAW` for 500 ms (or never). This is what the
    brief calls "failsafe".
  - `InputSource::FAILSAFE` — `!isSignalValid()` **and** no browser connected. Only this one runs
    `applyFailsafe()` (relays all off). With a browser connected, a lost RC stream gives
    `InputSource::WEB`, and no relay is switched off.
- Owner decision **D-B (2026-11-13)**: while the source is `WEB`, gear, ignition ACC / ON and diff
  lock are `TEMPORARILY_REJECTED` reason 3; headlight, ignition OFF and handbrake are accepted.
- Therefore reason 3 keys on **"not under MAVLink control"**:
  `currentInputSource_ != InputSource::MAVLINK || !mavlink_.isSignalValid()`. This covers
  `FAILSAFE`, `WEB` after RC loss with a browser connected, and `WEB` with web control latched
  while RC is valid. The second term covers the one-loop-stale source (D2).
- Fail-safe behaviour itself is unchanged.

### D5. Interlocks per function
- **Gear** (order: 3, then 1, then 2):
  - not under MAVLink control → reason 3;
  - `!isEngineRunning()` → reason 1, nothing remembered (owner decision **D-E**). If the
    transmission is not yet initialised in the very loop the engine starts (it initialises in
    `VehicleController::update()`), reason 1 too;
  - target already equals the requested gear (idle, or the same change in progress, or the same
    destination queued) → `ACCEPTED`, nothing moves;
  - transmission speed interlock refuses (`canChangeGear()`, made public as a const query) →
    reason 2. This keeps the transmission's own rule: **N is never speed-refused**, and the CAN
    fallback applies when the hall sensor has not pulsed (owner decision **D-F**, deviation from
    the brief);
  - else `setGear()`; true → `ACCEPTED` and any pending RC gear is cleared (D-E: last change wins).
  - Allowed with the handbrake on (the firmware does not block it today).
- **Ignition**:
  - OFF → always `ACCEPTED` (engine kill: also in fail-safe, web, during the pre-crank dwell and
    cranking): `setIgnitionState(OFF)`, and the counters are persisted on a transition into OFF,
    as the RC path does;
  - ACC / ON when not under MAVLink control → reason 3;
  - ACC → `setIgnitionState(ACC)` (cancels a pending / active crank, as RC ACC does);
  - ON while the engine runs → `ACCEPTED`, no crank; the relay state is set to IGNITION if it was
    ACC (no relay change, ACC and IGNITION drive the same relays);
  - ON otherwise → `requestCrank()` exactly like a fresh RC transition into IGNITION;
  - relay board unavailable (`!relayController_.isAvailable()`) → `FAILED` (owner decision **D-G**).
- **Headlight**: always accepted → `setFrontLight()`; relay board unavailable → `FAILED` (D-G).
- **Diff lock**: not under MAVLink control → reason 3; else `setWheelLock()` (the CH7 path);
  `setWheelLock()` false (relays unavailable) → `FAILED` (D-G). Bit4 is clear when
  `RELAY_WHEEL_LOCK_FITTED` is 0, so the command is then `UNSUPPORTED`.
- None of these create a new interlock in the RC path; the RC path keeps today's rules.

### D6. Last change wins (owner decisions D-A, D-E, D-H)
- Per RC function (CH3 gear, CH4 ignition, CH6 light, CH7 lock) the vehicle layer keeps the last
  **in-band** decoded value (`RcEdge<T>`: `has`, `last`). Each loop in `processMavlinkCommands()`:
  - decode absent-aware: out of 880–2160 µs (incl. 0) → ABSENT; gear → R / N / L (no H),
    ignition → OFF / ACC / ON, light / lock → threshold 1520;
  - ABSENT → no change, the baseline is kept;
  - first in-band value with no baseline (boot, after a re-sync) → apply;
  - a value different from the baseline → apply, store;
  - the same value → nothing.
- "Apply" means what the old every-loop code did once: gear → becomes the **pending RC gear**;
  ignition OFF → `setIgnitionState(OFF)` + persist, ACC → `setIgnitionState(ACC)`, ON →
  `requestCrank()`; light → `setFrontLight()`; lock → `setWheelLock()`.
- Pending RC gear (D-E): retried every loop while the source is MAVLink, applied once the engine
  runs and `setGear()` accepts (or the target already equals it), exactly today's retry. A later
  accepted GCS gear clears it; a rejected GCS gear does not. A later RC edge replaces it.
- **One re-sync rule for every "RC is back in control" case (D-A, D-H).** Every loop in which
  the source is not `MAVLINK` (`FAILSAFE` or `WEB`, so `processMavlinkCommands()` does not run)
  clears all four baselines and the pending RC gear. The first valid frame after the source becomes
  `MAVLINK` again therefore applies every in-band channel, exactly as at boot:
  - fail-safe exit (D-A): RC ON → `requestCrank()` (re-crank on recovery, suppressed if the engine
    still runs), RC gear → pending until the engine runs;
  - return from `WEB` (D-H, 2026-11-13): web control released, or RC back while a browser was
    connected — the same rule; no stored value survives a WEB period;
  - boot: the source starts as `FAILSAFE`, and the baselines start empty.
  - A GCS command or a web command given before the re-sync is not restored; the RC state wins.
  - This replaces today's `previousIgnitionState_ = OFF` / `lastCommandedGear_ = UNKNOWN` reset in
    `applyFailsafe()`; `applyFailsafe()` itself is otherwise unchanged.
- `previousIgnitionState_` and `lastCommandedGear_` are replaced by the trackers.
- Steering, throttle and brake are unchanged (still every loop).

### D7. Handbrake — aztec only (owner decision D-C)
- `param1 = 5`: `commandParkBrake(param2 == 1 ? APPLY : RELEASE, ParkBrakeSource::GCS)`. No new
  interlock in the MAVLink layer; accepted in any source, ignition state and fail-safe (never
  reason 3).
- Mapping: `STARTED`, `STARTED_SPEED_UNKNOWN`, `REVERSING`, `ALREADY_RUNNING` → `ACCEPTED`;
  `REFUSED_MOVING` → `TEMPORARILY_REJECTED` reason 4; `REFUSED_DRIVER`, `REFUSED_NVS` → `FAILED`.
- Bit5 is set only when `ParkBrakeController::isDriverOk()` was true at boot; otherwise function 5
  is `UNSUPPORTED`. On `master` bit5 is never set and function 5 is `UNSUPPORTED` (X6 is the
  steering VESC; the parking brake is never cherry-picked).
- Drive-away release, throttle hold and `EFI_STATUS.pt_compensation` bits 2 / 3 stay exactly as
  committed in 1.5.29. The existing `[PBRAKE] … accepted (gcs)` line is the log.

### D8. Capabilities and identity
- Mask, built once at boot in `VehicleController`:
  - bit0 gears R/N/L and bit1 gear H — compile-time 1 on both branches (both transmissions know H);
  - bit2 ignition, bit3 headlight — 1 (relay functions exist on both builds);
  - bit4 diff lock — `RELAY_WHEEL_LOCK_FITTED` (1 on both branches);
  - bit5 handbrake — aztec: `isDriverOk()` at boot; master: 0;
  - bits 6–15: 0.
  - Expected: quad `0x1F`; buggy `0x3F`, or `0x1F` without the X6 driver.
  - Relay availability is runtime and recoverable, so it does not clear bits; it gives `FAILED`.
- Firmware version: `firmwareVersionCode(FIRMWARE_VERSION)` (1.5.30 → 10530).
- Hardware version: bits 0–7 `BOARD_REVISION_CODE` 0 (`Control_v0`); bits 8–15
  `VEHICLE_TYPE_CODE` (1 on `master`, 2 on `aztec`, new constant per branch); bits 16–23 the
  **detected** relay board `RelayBoardKind` (0 none, 1 PCA9557, 2 XL9535), latched at boot — not
  the NVS setting, whose 0 means AUTO, not "none" (owner decision **D-I**, deviation from the
  brief).

### D9. Broadcasts
- `QB_CAPS` (`NAMED_VALUE_INT`): every `VCMD_CAPS_TX_MS` 10000 ms, first one on the first
  `report()`; value = mask (bits 0–15) | protocol version << 24 (bits 16–23 zero).
- `IGN_STATE` (`NAMED_VALUE_INT`): 0 OFF, 1 ACC, 2 ON, 3 CRANKING, from the applied relay state:
  `CRANKING`, or `ACC` with a crank armed (pre-crank dwell) → 3; `IGNITION` → 2; `ACC` → 1; `OFF`
  (also expander unavailable) → 0. Sent on the existing 1 Hz slow tick and once immediately on
  every change. Needs a public `RelayController::isCrankPending()` getter.
- Both are added to `StateReport` and assembled in `src/main.cpp`, like `digitalFlags`. Names are
  compile-time literals with the `static_assert(… <= 10)` guard and go through `sendNamedInt()`.
- `STEER_MODE`, `QB_CAPS` and `IGN_STATE` become the three `NAMED_VALUE_INT` names; the
  `NAMED_VALUE_FLOAT` set is unchanged.

### D10. Versions and branches (owner decision D-D)
- `master` 1.5.28 → **1.5.30** (skips 1.5.29). Merge `master` into `aztec`; on `aztec` the version
  is 1.5.30 too, the 1.5.29 entry stays in its changelog below the 1.5.30 one, and
  `VEHICLE_TYPE_CODE` stays 2. The handbrake function is then added on `aztec` in the same 1.5.30
  (the aztec 1.5.30 changelog gets the handbrake line).

## Risks / Trade-offs
- **Out-of-band RC no longer forces a safe value.** Today an ignition channel outside 880–2160 µs
  reads OFF every loop and switches the engine off; after this change it is "no change". This is
  required by the brief (a disappearing channel is not a change); the stream loss itself still
  goes to fail-safe after 500 ms. → Documented in the changelog and the bench list.
- **One-loop-stale source in the sink.** → Checked against `isSignalValid()` as well (D4).
- **Gear N at speed is accepted** (transmission rule, D-F). A GCS N at speed shifts to neutral
  exactly as an RC N does today.
- **Return from web control re-applies RC (D-H).** Releasing web control, or RC returning while a
  browser is connected, re-applies the RC gear / ignition / light / lock. With RC CH4 in ON and the
  engine stopped (for example the web switched the ignition off), this starts the auto-crank;
  today the return re-crank happens only after a real fail-safe. → Changelog line and bench task.
- **Merge conflicts on aztec** in `processMavlinkCommands()` (aztec has `applyOperatorThrottle()`),
  `Constants.h` (version, vehicle type), `Changelog.h`. → Explicit merge task.
- **Clients rely on old behaviour for old firmware.** → `UNSUPPORTED` stays for every unknown
  command; no `QB_CAPS` before 1.5.30.

## Migration Plan
1. Implement on `master` (tasks 1–6), build, native tests, validate, bench on the quad.
2. Merge `master` into `aztec` (task 7), resolve conflicts, build.
3. Add the handbrake function on `aztec` (task 8), build, tests, bench on the buggy.
4. Rollback: reflash 1.5.28 (quad) / 1.5.29 (buggy); no NVS keys are added, nothing to migrate.

## Owner decisions of 2026-11-13 (answers to the former open questions)
- **D-F (was Q1).** Follow the code: GCS gear N is accepted at any speed; R / H / L are rejected
  with reason 2 above 1.4 m/s (the transmission's `canChangeGear()`, same as RC).
- **D-G (was Q3).** Relay board unavailable → ignition, headlight and diff-lock commands answer
  `MAV_RESULT_FAILED` and change nothing.
- **D-H (was Q2).** On the return from `WEB` to `MAVLINK` control the first valid RC frame re-sets
  gear / ignition / light / lock exactly like the fail-safe-exit rule D-A — one rule for every "RC is
  back in control" case, no stored values frozen across a WEB period.
- **D-I (was Q4).** Hardware-version bits 16–23 = the relay board detected at boot (0 none,
  1 PCA9557, 2 XL9535), not the NVS setting.

## Deviations from the brief
- **Gear N at speed (D-F).** The brief says "Gear (any): reject 2 if speed > 1.4 m/s"; N is
  accepted at any speed because the transmission interlock never refuses N.
- **Hardware-version bits 16–23 (D-I).** The brief says "relay-board variant (0 none; NVS
  `relay/board`)"; the NVS value is the setting (0 = AUTO), so the detected board kind is reported.
- **Out-of-band RC channel = no change.** Not a deviation from the brief's rule ("a disappearing
  channel is not a change") but from today's firmware: a value outside 880–2160 µs no longer
  forces N / OFF / light off / unlock.
- **Reason 3 covers the web portal (D-B).** The brief defines failsafe as `!isSignalValid()`;
  reason 3 is given whenever the vehicle is not under MAVLink control (fail-safe source, web
  portal, or a stale signal), see D4.
- **`FAILED` beyond the handbrake (D-G).** The brief lists `FAILED` only for the handbrake; it is
  also used when the relay board is unavailable.

## Open Questions
- None.
