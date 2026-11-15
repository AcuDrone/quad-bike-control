# Design: add-precrank-dwell-setting

## Context
Auto-crank today (`master` 1.5.31; line numbers are `master`):
- `RelayController::requestCrank()` (`src/RelayController.cpp:347-363`) brings the ignition / ECU
  line up (state ACC, `ignitionHighSince_ = millis()` on the OFF → non-OFF edge in
  `setIgnitionState()`, `:318-324`) and sets `crankArmed_`.
- `RelayController::update(engineRpm)` (`:476-490`): while armed and not cranking, an
  `engineRpm >= ENGINE_RUNNING_RPM_THRESHOLD` suppresses the crank and settles in IGNITION;
  otherwise the starter engages once `millis() - ignitionHighSince_ >= ACC_PRECRANK_DWELL_MS`
  (2000). While cranking (`:492-512`) the starter stops on the same RPM threshold or after
  `CRANKING_TIMEOUT` (5000).
- Callers of `requestCrank()`: the RC ignition edge (`VehicleController.cpp:783-793`), the GCS
  `MAV_CMD_USER_2` ignition ON (`:874-876`) and, through the RC trackers' re-sync, the return from
  fail-safe or web control. The web START button does not use it: `setIgnitionState("START")`
  (`:1777-1804`) sets CRANKING directly after its own `dataValid && rpm >= threshold` check.
- The RPM reaching `update()` is `canData.engineRPM` raw (`:298`). The CAN side
  (`src/CANController.cpp`): `engineRPM` is written only by a PID 0x0C reply (`:512`); `dataValid`
  is set true by ANY PID reply (`:350-351`) and false only after `CAN_DATA_STALE_TIMEOUT` (5000 ms)
  without a reply (`:405-408`) or when every PID has failed `CAN_RETRY_ATTEMPTS` (`:394-396`).
  RPM is polled every `CAN_POLL_INTERVAL_RPM` (500 ms), one request in flight, response timeout
  `CAN_RESPONSE_TIMEOUT` (200 ms); this change leaves the poll rate as it is.

Branches: `aztec` (buggy, 1.5.32) computes one effective RPM per loop,
`tachoSelectRpm()` (`include/TachoLaw.h`): live tacho → `canValid && canRpm > 0` → fitted tacho 0 →
CAN → none, and already feeds `relayController_.update(getEngineRpm())` (0 when no source is valid).
`old-board` (hand-wired DevKitC-1, 1.5.31) has a GPIO `RelayController` (no NVS, no relay board)
with the same crank logic (`requestCrank()` `:121`, `update()` `:174`, dwell check `:191`) and the
same raw `relayController_.update(canData.engineRPM)` (`VehicleController.cpp:229`).

## Goals / Non-Goals
- Goals: the operator sets the pre-crank dwell per vehicle (1–8 s, default 3 s) from the web; a
  crank request is never suppressed or cut short by an RPM value the ECU did not just report.
- Non-Goals: auto-retry after a failed crank; debouncing "engine started"; idle throttle while
  cranking; the dwell for the web START button (D5, settled); faster RPM polling while cranking (see
  "Rejected alternatives"); the "Ignition Safety Interlocks" drift (brake
  ≥ 20 %, 1100 RPM in the spec, not in the code); the `clampUs` / absent-channel finding.

## Decisions
- **D1. Units and resolution: whole milliseconds, UI in seconds with step 0.5.** Every timed
  setting on the web is stored and commanded in whole ms (`brake travel_ms`, the park ladder
  `park_t1..t3`), so the wire and NVS value is ms (uint16), validated 1000–8000. The delay
  slider is in seconds (min 1, max 8, step 0.5) because that is how the operator thinks of a wait
  of a few seconds; the page sends `round(seconds × 1000)` and shows `crank_dwell_ms / 1000` with
  one decimal. The firmware accepts any whole ms in range, so a finer value can still be sent.
  Range: 1 s keeps a minimum ECU wake-up; 8 s is well beyond any fuel-pump prime we know of and
  keeps a mistyped value from turning the switch into "nothing happens".
- **D2. Owner and storage: `VehicleController`, new namespace `"crank"`, key `dwell_ms`.**
  `RelayController` gets a RAM-only `setPrecrankDwellMs(uint16_t)` / `getPrecrankDwellMs()` and a
  member `precrankDwellMs_` initialised to `ACC_PRECRANK_DWELL_MS`. Reasons: the `old-board`
  `RelayController` is a GPIO driver with no Preferences at all, so storage inside it would need a
  second, branch-specific implementation; the existing `"relay"` namespace holds the relay board
  selection (read before detection, "never clear()"), which is a different concern; and
  `VehicleController` already owns the other operator settings (`"brake"`, `"boost"`, `"speed"`)
  and the web command plumbing. Invalid stored values are logged and ignored, never rewritten
  (same rule as `open_slew`, `travel_ms`). Persist first, then apply.
- **D3. Pure law `include/CrankLaw.h`.** Only arithmetic and decisions, natively testable, in the
  style of `VehicleCommandLaw.h` / `BrakeLaw.h`:
  - `crankDwellMsValid(ms)`, `crankDwellMsOrDefault(stored)`;
  - `crankDwellElapsed(ignitionHighSinceMs, nowMs, dwellMs)` — false while the sentinel is 0,
    unsigned (wrap-safe) subtraction;
  - `enum class CrankArmedAction { WAIT, SUPPRESS, ENGAGE }` and
    `crankArmedDecide(rpm, runningThreshold, ignitionHighSinceMs, nowMs, dwellMs)` — SUPPRESS when
    `rpm >= threshold` (checked first, as today), ENGAGE when the dwell has elapsed, else WAIT;
  - `crankRpm(valid, rpm)` → `valid ? rpm : 0`;
  - `ignitionWentOff(wasOn, isOn)` — the edge that clears the stored RPM.
  `RelayController::update()` replaces its inline armed-crank comparison by
  `crankArmedDecide(...)`; the cranking branch, the timeout and every relay write stay where they
  are. This is the smallest cut that puts the configurable value and its clamping under test
  without rewriting `RelayController`.
- **D4. Stale RPM: two guards, both needed.**
  1. *Validated input.* `master` / `old-board` call
     `relayController_.update(crankRpm(canData.dataValid, canData.engineRPM))`. This covers every
     case where the CAN data is invalid but the stored value is not 0 (ECU silent for more than
     5 s, all PIDs failing, CAN fault while the ignition stays on).
  2. *Clear at ignition OFF.* `dataValid` stays **true for up to 5 s** after the ECU loses power
     (it is refreshed by any PID reply and only times out), so guard 1 alone does not fix the
     common case — OFF and straight back to ON. On the transition of the applied relay ignition
     state into OFF (`ignitionWentOff()`, evaluated at the top of `VehicleController::update()`
     where the state is already read for `setLwsPowered()`, so every path into OFF is covered:
     RC, GCS, web, `allOff()` in fail-safe and the starter escalation) `VehicleController` calls
     `CANController::clearEngineRpm()`, which sets `engineRPM = 0` only. `dataValid`,
     `lastUpdateTime` and the other ECU fields are left alone: clearing `dataValid` would turn
     coolant, MAP, voltage and the whole `EFI_STATUS` block NaN five seconds early, and touching
     `lastUpdateTime` would interfere with the OBD silence watchdog. A 0 that reads as valid is
     true here — with the ECU unpowered the engine cannot be running — and the first reply from
     the re-powered ECU replaces it.
  This one call also fixes `isEngineRunning()` (GCS ignition ON answered "engine running, no
  crank"; RC gear retry), the web START check ("Engine already running") and the `aztec`
  `tachoSelectRpm()` step `canValid && canRpm > 0`, which otherwise returns the stale CAN value
  once the tacho has timed out (500 ms after the engine stops).
- **D5. Web START keeps its behaviour, without the dwell (decided by the user, 2026-11-15).** It
  is a momentary manual "start now" with its own operator-controlled wait (press ACC, wait,
  press START). Putting it through the dwell would
  change a button the operator uses for diagnosis; it benefits from D4 (no "Engine already
  running" refusal and no instant "Engine started!" drop after OFF) without further change.
- **D6. A new value applies at once.** `update()` reads `precrankDwellMs_` every loop, so a value
  saved while a crank is armed is used for that crank (shorter → it may engage on the next loop;
  longer → it waits longer). No restart of the dwell clock.
- **D7. `aztec` merge.** The `VehicleController.cpp` conflict at the relay update is resolved by
  keeping `relayController_.update(getEngineRpm())` (already 0 when invalid; tacho first). The
  clear at OFF, `CrankLaw.h`, the setting and the web row merge as on `master`.
  `ENGINE_RUNNING_RPM_THRESHOLD` stays 800 on `aztec`. The aztec ECU (Delphi MT05) is not known to
  report RPM on CAN; with a live tacho the CAN RPM is not used for the crank exit.
- **D8. `old-board` merge.** `CANController` and `CrankLaw.h` are identical; `RelayController` is
  the GPIO version, so its armed-crank branch (`:183-197`) and the new setter are edited by hand in
  the merge; the relay update at `VehicleController.cpp:229` gets `crankRpm(...)` as on `master`.
- **D9. Telemetry and the web row.** `crank_dwell_ms` (int) — one scalar key, ≈16 B, within the
  `StaticJsonDocument<9216>` headroom; the capacity comment is updated. Control: a slider (1–8 s,
  step 0.5, value on the right) at the owner's request, 2026-11-15 — it replaced the first number
  field with Save. It follows the brake travel slider: the label tracks the thumb while dragging,
  one command on release (each is an NVS write), and telemetry does not move the thumb while it is
  held. A stored value off the 0.5 s grid (another client, a direct command) puts the thumb on the
  nearest step, while the label and the hint show the real value; local validation is gone (the
  slider cannot leave 1–8 s), the firmware range check stays. Placement: the row first sat in
  the "Vehicle Ignition & Lighting" card (Tests section) right under the OFF / ACC / START buttons;
  on 2026-11-15, before 1.5.33 was flashed anywhere, the owner moved it to its own card
  "🚀 Запуск двигуна" / "🚀 Engine start" in the Calibration section, right after Throttle
  Calibration, because it is a persisted setting, not a test control — Calibration is where every
  other stored setting lives. Element ids, function names and the reply routing are unchanged; the
  hint names the START button's section ("Тести" / "Tests") so the "no delay" note stays clear. The
  card is shown on every board.

## Rejected alternatives
- **Poll RPM at `CAN_POLL_INTERVAL_RPM_BOOST` (50 ms) while a crank is pending or running.**
  Considered to shorten the starter overrun after the engine catches (≈0.7 s → ≈0.25 s) using the
  gear-change boost's existing mechanism; declined by the owner on 2026-11-15. The RPM PID stays
  at `CAN_POLL_INTERVAL_RPM` (500 ms) throughout.

## Risks / Trade-offs
- **Starter overrun after the engine catches stays as today.** Validated or raw, "engine started"
  is only seen through a fresh PID 0x0C reply: before the engine catches the stored value is 0
  (cleared at OFF, or the ECU's own 0 / cranking speed during the dwell), so validation changes
  nothing about detection while replies arrive. With the unchanged 500 ms poll and up to 200 ms
  response the starter can keep turning for up to ≈0.7 s after the RPM reaches the threshold,
  exactly as before this change.
- **CAN silent during cranking.** If the ECU does not answer while the starter turns (voltage dip,
  CAN fault), the crank logic sees 0 and the starter runs to `CRANKING_TIMEOUT` (5 s) even if the
  engine caught. This is today's behaviour too (the stored value is 0 or cranking speed at that
  point); 5 s bounds it. → Bench check that RPM replies continue during cranking.
- **Threshold vs idle on the quad.** `ENGINE_RUNNING_RPM_THRESHOLD` is 1500 on `master`, chosen
  "safely above idle". If the engine catches without flaring above 1500, the starter runs to the
  5 s timeout on every start. Not changed here (existing behaviour); the bench notes whether the
  starter stops on catch.
- **Late reply after OFF.** A PID 0x0C reply already in flight when the relay opens could re-write
  a high RPM just after the clear. The window is the relay release plus at most
  `CAN_RESPONSE_TIMEOUT` (200 ms), and an unpowered ECU cannot answer the next request; accepted.
- **Relay board fault reads as OFF.** `getIgnitionState()` reports OFF while the relay expander is
  unavailable, so a relay fault also clears the RPM. If the ECU is in fact still powered, the next
  RPM reply (within ≈0.7 s at the normal 500 ms interval) restores it; during that time
  `isEngineRunning()` is false and GCS gear commands get reason 1. Accepted.
- **Longer default.** 3 s instead of 2 s on every auto-crank, including the re-crank after a
  fail-safe recovery. Intended; the operator can set 1–8 s.

## Migration Plan
New NVS namespace; nothing to migrate. Units without the key run at 3 s. Rollback: flashing an
older firmware ignores the `"crank"` namespace (2 s constant again).

## Open Questions
- Archive order (resolved 2026-11-15): this change MODIFIES two requirements added by
  `add-gcs-vehicle-commands`; that change was archived on `master` on 2026-11-15, so both are now in
  the main specs (see tasks 0.2 / 7.3).
