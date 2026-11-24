## Context
Relay functions are relay-number sets in `include/Constants.h` (`RELAY_SET_*`, bit0 = Relay_1),
mapped at `RelayController::begin()` through the detected board's table into whole-port masks
(`include/RelayBoardLaw.h`; `src/RelayController.cpp:240`). Every change is one verified whole-port
write from `outputShadow_` (`computeShadow()` `:525`, `writePort()` `:663`). Today:

| Relay | Quad (`master`) | Buggy (`aztec`) | PCA9557 bit (conn.) | XL9535 bit |
|---|---|---|---|---|
| 1 | ignition (pair) | ignition | IO4 (X4) | P0.0 |
| 2 | ignition (pair) | starter | IO5 (X5) | P0.1 |
| 3 | starter | light | IO6 (X6) | P0.2 |
| 4 | light | lock | IO7 (X7) | P0.3 |
| 5, 6 | lock (pair) | lock | IO3, IO2 (X8, X9) | P0.4, P0.5 |
| 7 | spare | spare | IO0 (X10) | P0.6 |
| 8 | spare → **washer** | spare → **washer** | IO1 (X11) | P0.7 |

The web relay test (`setRelayTest()` `:601`) energizes a single relay only with the ignition OFF and
no function on; any real function command drops test bits (`clearRelayTests()` `:552`), as do
`allOff()` (`:424`), an unverified write (`:714`) and the starter escalation (`:738`). The crank
state machine lives in `setIgnitionState()` (`:292`), `requestCrank()` (`:352`) and `update()`
(`:445`; test expiry at `:478`, then crank logic with early returns).

GCS functions: `include/VehicleCommandLaw.h` (functions 0–5 `:37`, decode bound `fn > 5` `:83`,
reasons 0–4 `:29-33`, `vehicleCapsMask()` `:240`), executed by
`VehicleController::executeVehicleCommand()` (`src/VehicleController.cpp:955`), caps from
`getVehicleCapsMask()` (`:947`). The comparable persisted setting is the starter time limit
(`set_crank_max_ms`, NVS `crank`/`max_ms`, `:1812-1843`).

## Goals / Non-Goals
- Goals: Relay_8 timed pulse on both vehicles; web + GCS; ignition-on interlock; persisted 2–20 s
  length; consistent with the relay fault, fail-safe and relay-test rules.
- Non-Goals: RC control, GCS client UI, Relay_7, `old-board`.

## Decisions
- **D1 Relay sets.** `RELAY_SET_WASHER = RELAY_N(8)` on both branches; `RELAY_SET_RESERVED =
  RELAY_N(7)`. `relaySetsValid()` / `RelayPortMasks` / `relayBuildMasks()` get a `washer` member
  placed after `wheelLock`; the invariant stays "pairwise disjoint, OR = 0xFF", and a new
  `static_assert(RELAY_SET_WASHER == RELAY_N(8))` pins Relay_8 so Relay_7 can never slip in.
  Pinned masks — quad PCA9557: ign `0x30`, start `0x40`, light `0x80`, lock `0x0C`, washer `0x02`,
  reserved `0x01`; quad XL9535: `0x03`, `0x04`, `0x08`, `0x30`, `0x80`, `0x40` (new assert, master
  only pinned the PCA9557 board). Buggy PCA9557: `0x10`, `0x20`, `0x40`, lock `0x8C`, washer `0x02`,
  reserved `0x01`; buggy XL9535: `0x01`, `0x02`, `0x04`, `0x38`, `0x80`, `0x40`.
  As built (2026-11-25): `relaySetsValid()` requires a non-empty washer, so the pre-1.6.3 layout
  (Relay_7 + Relay_8 reserved, no washer) no longer validates; the native test asserts that.
- **D2 Where the pulse lives.** In `RelayController` (it owns the shadow and every release path):
  members `washerOn_`, `washerSince_`, `washerLimitMs_` (latched at start), `washerPulseMs_`
  (setting, RAM). `computeShadow()` ORs the washer mask when `washerOn_`. Serviced in `update()`
  right after `expireRelayTests()` and before the crank block (whose early returns must not skip
  it); end test `washerExpired(since, now, limit)` = `(uint32_t)(now - since) >= limit`.
- **D3 Interlock definition** against the real `IgnitionState` enum (`include/RelayController.h:96`):
  allowed iff available && state ∈ {`ACC`, `IGNITION`} && `!crankArmed_` && `!isCranking_`. `ACC`
  covers "ignition on, engine off"; `IGNITION` covers "engine running" and also "ignition on after
  a start attempt that timed out" — the owner's rule is "ignition on", so that is allowed too.
- **D4 Crank aborts the pulse** (`requestCrank()` arming and `setIgnitionState(CRANKING)`, so the web
  START button is covered) and a request during the dwell / crank is refused. Reason: the starter
  draws 100+ A from the 12 V battery and the voltage sags; adding the pump load deepens the sag
  during the one phase where the ECU must keep running, and a pulse started during the 1–8 s dwell
  would anyway overlap the crank.
- **D5 Restart, not ignore**, for a wash while running: the timer restarts from the new request with
  the current setting. It matches what a press means ("wash now") and a GCS operator who sees no
  feedback can press again. Alternative "ignore while running" was rejected: a second press would
  be a silent no-op. Bounded: every press is at most 20 s from that press.
- **D6 Stop.** No web stop button (owner: no off command needed; a pulse is ≤ 20 s and every
  ignition OFF ends it). GCS `param2` 0 = stop, because every on/off-style function in the table
  takes 0/1 (the default branch of `vehicleCmdDecode()` already accepts exactly 0 or 1) and a safe
  direction abort costs one line. Stop is accepted in every input source (like ignition OFF), FAILED
  only with the relay board unavailable.
- **D7 GCS encoding.** `param1` 6 `VehicleCmdFunction::WASHER`, capability bit6 `VCMD_CAP_WASHER`
  `0x40` (next free per "Vehicle Command Protocol Extension Rules"); decode bound `fn > 6`;
  `vehicleCapsMask(..., washer)` gets a 7th parameter = the washer fitted switch (D13). Masks
  fitted / not fitted: quad `0x5F` / `0x1F`, buggy `0x7F` / `0x3F` (`0x5F` / `0x1F` if X6 not
  initialised); `QB_CAPS` 16777311 / 16777247 (quad), 16777343 / 16777279 (buggy). New reason
  `VCMD_REASON_IGNITION_OFF = 5` ("ignition not on / engine start in progress") — no existing
  reason fits (1 means "engine not running", but ACC with the engine off is allowed). Order in
  `vcmdWasherDecide(value, underMavlink, relaysOk, ignitionOk)`: stop → relaysOk ? ACCEPTED :
  FAILED; wash → !underMavlink reason 3 → !relaysOk FAILED → !ignitionOk reason 5 → ACCEPTED.
  Wash needs MAVLink control like the lock (it is an actuator the RC operator should own); the web
  command is not source-gated, like light / lock.
- **D8 EFI state bit added** (owner decision 2026-11-25). `EFI_DIGITAL_FLAG_WASHER 0x10` (bit4) in
  `include/Constants.h` next to `EFI_DIGITAL_FLAG_WHEEL_LOCK` / `_FRONT_LIGHT` (`:426-427`), on both
  vehicles; bits 2–3 stay the buggy's parking brake (0 on the quad). Set while `isWasherOn()`;
  cleared on the normal end and on every abort path (it follows `washerOn_`, so no extra code per
  path). Assembled with the other flags in `src/main.cpp:404` (`report.digitalFlags`). The washer is
  a pulse, not on/off, but takes the next free state bit anyway; the extension-rules delta says so.
  Evidence that existing clients are unaffected: both decode the word bit by bit, never by
  whole-value comparison — QGC fork `gcs/qgroundcontrol` (main, 4f5956d12)
  `src/Vehicle/FactGroups/VehicleQuadBikeFactGroup.cc:596-598` and `QuadBikeVehicleControl.cc:284-290,
  536-543, 778, 978-994` (constants `QuadBikeVehicleControl.h:133-137`); Mission Planner plugin
  `gcs/mission-planner-setup/tools/mp-plugin/QuadBikePlugin.cs:4020-4022, 4090-4107` (as reported by
  the owner's check; not re-read here). So an unknown bit4 is ignored by current clients.
- **D9 Telemetry.** `washer_fitted`, `washer_on`, `washer_left_ms`, `washer_pulse_ms` — 4 scalar
  slots, keys by pointer, ≈ 64 B memory / ≈ 80 B JSON. `master` worst case ≈ 8.70 → 8.77 KB of 9216;
  `aztec` ≈ 9.38 → 9.45 KB of 10240. No resize.
- **D10 Setting and naming** follow `add-crank-time-limit-setting`: NVS namespace `"washer"` (new,
  owned by `VehicleController`), key `pulse_ms` uint16; `WASHER_PULSE_MS_MIN` 2000, `_MAX` 20000,
  `_DEFAULT` 3000 in `Constants.h`, mirrored as `WASHER_LAW_*` literals in `WasherLaw.h` (checked by
  `static_assert` in `RelayController.cpp`); `washerPulseMsValid()` / `washerPulseMsOrDefault()`;
  web `set_washer_pulse_ms` (ms, rounded), slider in seconds step 0.5. A separate namespace rather
  than `"relay"`, because `"relay"` is the board-selection namespace owned by `RelayController`.
- **D11 Spec deltas vs `aztec`.** The deltas are written against `master`'s specs, whose generic
  relay requirements still carry an older buggy table (locks "reserved"). `aztec`'s copies of four
  MODIFIED requirements differ and must be re-based at merge / archive time on `aztec`:
  - `gpio-expander` "Relay Output Expander" (aztec: lock fitted on Relay_4..6, reserved = Relay_7/8)
    → Relay_8 row = washer, reserved bits "Relay_7", washer scenario;
  - `gpio-expander` "Relay Port Mask Construction" (aztec: lock `0x8C`/`0x38`, reserved
    `0x03`/`0xC0`) → washer `{8}` `0x02`/`0x80`, reserved `{7}` `0x01`/`0x40`;
  - `mavlink-interface` "Relay Controller for Ignition and Lights" (aztec: Lock row, reserved
    `0b00000011`/`0b11000000`, lock scenarios) → add the washer row, reserved `0b00000001` /
    `0b01000000`, Relay_8 / allOff scenario edits, keep aztec's lock text;
  - `web-control` "Web Relay Test Control" (aztec wording differs) → only the Relay_7 / Relay_8 label
    edit;
  - the ADDED `mavlink-interface` "Headlight Washer State in the EFI_STATUS Digital-Output Bitmask"
    applies on aztec as is, but must agree with aztec-only "Parking Brake State in the EFI_STATUS
    Digital-Output Bitmask" (bits 2–3; check that requirement does not claim the layout ends at
    bit3 or that other bits are always 0 — edit it if it does);
  - `vehicle-systems` "Quad Bike Relay Assignment With Fitted Front-Wheel Lock" differs by one
    sentence on aztec → apply the same Relay_7 / Relay_8 and allOff edits to aztec's text;
  - also check aztec-only "Aztec Lock on Relay_4 + Relay_5 + Relay_6" (no Relay_7/8 mention today —
    no edit expected) and that aztec's "GCS Vehicle Command Interlocks" is still identical to
    master's (it is, 2026-11-25) so the master copy applies.
  The mavlink GCS requirements ("GCS Vehicle Commands…", "Queries", "Broadcast", "Extension Rules")
  are identical on both branches today, so master's copies apply unchanged.
- **D12 Journal.** Light / lock / ignition commands are not journaled today; the washer gets
  `RELAY I washer_on src= ms= restart=` (VehicleController knows the source) and `RELAY I
  washer_abort why= left_ms=` (RelayController knows why). Normal ends and refusals are not
  journaled; dedupe (2 s per pair) bounds button mashing.

- **D13 "Washer fitted" switch** (owner decision 2026-11-25, default OFF). NVS `"washer"` / `fitted`
  (bool, missing = false); web `set_washer_fitted` (bool), telemetry `washer_fitted`, following the
  gear-boost switch (`set_boost_enabled` / `boost_enabled`, `src/VehicleController.cpp:610`,
  `src/WebPortal.cpp:947`): persisted first, applied at once, the toggle shows the telemetry value.
  Runtime, not a compile flag like `RELAY_WHEEL_LOCK_FITTED`: the same firmware runs on vehicles with
  and without a pump, and `RELAY_SET_WASHER` stays Relay_8 at compile time.
  - `RelayController::setWasherFitted(bool)` (RAM): off → `requestWasher()` refuses first with
    "washer not fitted - enable it in Calibration" (`WasherVerdict::REFUSED_NOT_FITTED`, checked
    before availability) and a running pulse ends with `washer_abort why=not_fitted`. The status
    bit follows `washerOn_`, so it cannot be set while not fitted. The relay test ignores the switch.
  - **Slider state when not fitted:** stays editable (the value is kept), with a muted note
    "Washer off — duration not in use" — simpler than enabling/disabling the control, and the
    operator can preset the length before enabling the washer.
  - **Caps at runtime.** Today the mask is computed once at boot
    (`VehicleController::getVehicleCapsMask()` `src/VehicleController.cpp:947-953`, handed over by
    `setVehicleCommandSink()` in `src/main.cpp:306-309`, stored in `vcmdCaps_`
    `src/MavlinkInterface.cpp:642-644`, used by the decoder `:581`, the caps query `:593` and the
    `QB_CAPS` broadcast every `VCMD_CAPS_TX_MS` = 10 s `:1015-1019`). New
    `MavlinkInterface::setVehicleCapsMask(uint16_t)`: stores the mask and, if it changed, clears
    `capsSentOnce_` so `QB_CAPS` goes out on the next report and the 10 s period restarts from there.
    `VehicleController` calls it after a successful `set_washer_fitted` (through a callback/pointer
    registered in `main.cpp`, or by `main.cpp` polling `getVehicleCapsMask()` each loop — the
    implementer picks the smaller; behaviour is the same). As built: the per-loop poll in
    `main.cpp` `loop()`, just before the MAVLink report. Bits 0–5 stay boot-fixed. The decoder
    already answers `UNSUPPORTED` when a function's capability bit is clear
    (`include/VehicleCommandLaw.h:95`, `if (needBit != 0 && (capsMask & needBit) == 0) return d;`),
    so function 6 needs no extra code when not fitted.

## Risks / Trade-offs
- Pump current vs relay contact / harness fuse → bench check (task 8.2); a small pump is ≈ 3–5 A.
- A welded Relay_8 contact runs the pump dry — invisible to read-back (existing limitation).
- A GCS that cached the mask may keep showing (or hiding) the washer until it reads the next
  `QB_CAPS` (≤ one report after a toggle) → acceptable, the command is answered `UNSUPPORTED` anyway.
- Shared-reason namespace: reason 5 must not be reused by an aztec-only change; the merge task checks.
- Washer refused under web control from the GCS (reason 3) may surprise a GCS operator; it mirrors
  the lock.

## Migration Plan
New NVS namespace, missing key = default; no migration. Rollback = flash 1.6.2 (Relay_8 off again).

## Open Questions
None. Decided 2026-11-25: EFI state bit4 `0x10` added (D8); "washer fitted" switch, default OFF (D13).
