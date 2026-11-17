# Change: Maximum starter time per attempt set from the web (2–10 s, default 5 s)

## Why
The owner wants the starter's maximum running time per start attempt to be a per-vehicle setting
in Calibration → «Запуск двигуна» / "Engine start", next to the pre-crank delay, with limits 2–10 s
and a default of 5 s (request 2026-11-17). Today it is a firmware constant and changing it needs a
reflash; it was raised 2 s → 5 s that way in 1.5.22 (`d33c1a6`, "2 s was too short").

How cranking works today (`master` 1.5.35; `aztec` and `old-board` are the same apart from the
RPM source and the relay driver):

- **The cap.** `CRANKING_TIMEOUT` = 5000 ms (`include/Constants.h:249`; 5000 on `master`, `aztec`
  `:322` and `old-board` `:131` alike). `RelayController::setIgnitionState(CRANKING)` stamps
  `crankingStartTime_` (`src/RelayController.cpp:328`); `RelayController::update()` checks, every
  control loop, first `engineRpm >= ENGINE_RUNNING_RPM_THRESHOLD` → "Engine started!" → IGNITION
  (`:504`), then `millis() - crankingStartTime_ >= CRANKING_TIMEOUT` → "Cranking timeout … Engine
  did not start" → IGNITION (`:511`). Leaving CRANKING drops the starter relay; the ignition stays
  on.
- **Who stops the starter early.** Only the running check: the validated RPM (CAN RPM while the CAN
  data is valid, 0 otherwise, on the quad and the old board; on the buggy the effective RPM, ECU
  TACHO first) at or above `ENGINE_RUNNING_RPM_THRESHOLD` (quad / old board 1500, buggy 800). Also
  any ignition OFF / ACC (RC, GCS, web, fail-safe `allOff()`) and a latched starter relay fault.
- **Where the cap applies.** Every path that reaches CRANKING, because the check lives in
  `RelayController::update()`: the RC ignition ON auto-crank and its re-crank on return from
  fail-safe / web control (`requestCrank()`, `src/VehicleController.cpp:913`), the GCS
  `MAV_CMD_USER_2` ignition ON (`requestCrank()`, `:991`), both after the pre-crank dwell; and the
  web START button in Tests (`setIgnitionState("START")` → CRANKING at once, no dwell, refused only
  when the CAN RPM is valid and ≥ the threshold, `:2152-2166`).
- **Retries and cool-down.** None. One request = one attempt; there is no attempt counter, no
  automatic retry and no minimum pause between attempts. A new attempt needs a new gesture: RC
  OFF/ACC → ON (edge-triggered), GCS ignition ON again, or web START again — each can follow the
  previous timeout immediately.
- **Relay test.** The web relay test of the starter relay has its own cap,
  `RELAY_TEST_STARTER_MAX_MS` = 2000 ms (`include/Constants.h:215`), allowed only with the ignition
  OFF. It is a wiring check, not a start attempt.

## What Changes
- **Configurable starter time limit.** A persisted per-vehicle value, the longest the starter may
  run in one start attempt:
  | Setting | NVS | Type | Range | Default |
  |---|---|---|---|---|
  | max starter time | namespace `"crank"` (exists since 1.5.33, owned by `VehicleController`), key `max_ms` (new) | uint16, ms | 2000–10000 | 5000 |
  Constants: `CRANKING_TIMEOUT` stays 5000 and becomes the default (comment "default; per vehicle
  from the web (NVS crank/max_ms)"); new `CRANK_MAX_MS_MIN` (2000), `CRANK_MAX_MS_MAX` (10000),
  `CRANK_MAX_NVS_KEY` ("max_ms"). Loaded at boot beside `dwell_ms` (`VehicleController::initCAN()`);
  a missing key reads as the default; a stored out-of-range value is logged and ignored (default
  used, not rewritten). `VehicleController` hands it to `RelayController` through a RAM-only setter
  `setCrankMaxMs()`, as for the dwell.
- **Scope.** The limit replaces `CRANKING_TIMEOUT` in `RelayController::update()`, so it applies to
  every attempt: RC ON, GCS ignition ON and web START. The RPM cut-off, the dwell, the interlocks
  and the absence of retries are unchanged.
- **When a change takes effect: the next attempt.** `RelayController` latches the limit when it
  enters CRANKING; a value saved while the starter turns is persisted and applied at once for the
  next attempt, but the running attempt keeps the limit it started with (no attempt is cut short
  or extended mid-crank by a slider release). Persist first, then apply, as the dwell.
- **Pure law.** `include/CrankLaw.h` gains `crankMaxMsValid()`, `crankMaxMsOrDefault()` and the
  wrap-safe `crankTimedOut(startMs, nowMs, limitMs)`; `RelayController::update()` calls
  `crankTimedOut()` instead of its inline comparison. Native tests extend `test/test_crank_law`.
- **Web.** Command `set_crank_max_ms` (value ms, rounded to a whole ms, validated 2000–10000, NaN
  refused, persisted first, then applied), accepted from any input source. Replies: "Starter time
  limit set to N ms", "Starter time limit out of range (2000-10000 ms)", "Failed to save the
  starter time limit". Telemetry key `crank_max_ms` (int). In the existing card "🚀 Запуск
  двигуна" / "🚀 Engine start" (Calibration), under the pre-crank delay: a slider "Макс. час роботи
  стартера" / "Max starter time" (seconds 2–10, step 0.5 — assumption: the same step as the delay
  slider; nothing in the code needs a finer one), value on the right ("5,0 с" / "5.0 s"), one
  `set_crank_max_ms` = `round(value × 1000)` on release, telemetry does not move the thumb while it
  is held, own reply line, a hint line and a help entry `crank_max_help` added to the card's ⓘ
  (`openHelp('crank_dwell_help,crank_max_help', …)`); uk + en.
- **No caution line.** The proposal's amber caution line under the slider above 5 s is dropped
  (owner decision 2026-11-17, "попередження не треба"); the risk is explained in the help text only.
- **Docs.** `WEB_PORTAL_SETUP.md` (Engine start row); `MAVLINK_SETUP.md` — the ignition row of
  the `MAV_CMD_USER_2` table says "starter ≤ 5 s" and `IGN_STATE` 3 now lasts up to the set limit —
  and therefore `../../API/README.md` (version note); `include/Changelog.h` 1.5.36 entry uk + en
  ("багі" in Ukrainian).
- **Version.** Firmware **1.5.36** on all three branches (no change in `openspec/changes/` claims
  it).

Out of scope: `RELAY_TEST_STARTER_MAX_MS` (the relay-test starter cap stays 2000 ms and fixed —
it is a bench wiring check with the ignition OFF, not a start attempt); a cool-down or attempt
counter between attempts (none exists today; out of scope by owner decision 2026-11-17); automatic retries; the
"Ignition Safety Interlocks" spec drift (brake ≥ 20 %, 1100 RPM, "5-second timeout" — not
implemented in code, left as `add-precrank-dwell-setting` left it).

## Risk
- **A running engine can be cranked for the whole limit.** The starter is released early only
  when the RPM reads at or above the running threshold. On the buggy the TACHO pulses-per-revolution
  is still unresolved (1 vs 2 ppr, field periods 21.5 / 43 / 64 ms); with a ppr stored too high the
  RPM reads half (idle ≈ 1400 → ≈ 700, under the 800 threshold), and with no TACHO signal and the
  Delphi MT05 reporting no RPM on CAN it reads 0. Then the starter stays engaged on a running
  engine until the limit: up to 10 s with this change (today up to 5 s). Overrunning the starter
  pinion is the damage to expect. The same applies on the quad if the CAN RPM is invalid while
  cranking.
- **Starter heating and battery drain.** Longer cranks and repeated attempts (nothing enforces a
  pause today) heat the starter and drain the 12 V battery; at 10 s per attempt the operator can
  chain attempts back to back.
- **Mitigation:** the default stays 5 s and the help text explains the risk; bench task 7.1 checks
  the buggy's idle RPM reads ≥ 800 before any value above 5 s is stored there. No caution line
  under the slider and no cool-down / attempt limit (owner decisions 2026-11-17).

## Owner decisions (2026-11-17, approval "роби")
1. Approved as proposed, with one amendment: **no caution line** under the slider above 5 s
   ("попередження не треба"). The risk statement above stays; the help text explains it.
2. **No minimum pause between attempts and no attempt limit** — out of scope, as today.
3. **Slider step 0.5 s** confirmed.

## Impact
- Affected specs:
  - `vehicle-systems`: MODIFIED "Ignition State Control Integration"; ADDED "Configurable Starter
    Time Limit".
  - `mavlink-interface`: MODIFIED "Ignition State Broadcast" ("the 5 s cranking timeout" → the
    configured limit).
  - `web-control`: ADDED "Starter Time Limit Configuration via Web Interface".
  - `web-telemetry`: ADDED "Starter Time Limit Telemetry".
- **Overlap with in-flight changes.**
  - `add-precrank-dwell-setting` (implemented 1.5.33, bench pending, not archived) MODIFIES the
    same two requirements ("Ignition State Control Integration", "Ignition State Broadcast"); the
    copies here are its versions with only the timeout wording changed, so it MUST be archived
    first (task 0.2). It also owns the "Engine start" card, the `"crank"` namespace and
    `CrankLaw.h` that this change extends.
  - `GCS Vehicle Command Interlocks` (vehicle-systems) and `Relay Output Expander` (gpio-expander)
    name `CRANKING_TIMEOUT` as the bound of an attempt; both are already MODIFIED by
    `add-precrank-dwell-setting` and the first also by `remove-rc-wheel-lock-channel`. Rather than
    a third copy, the ADDED requirement here states that `CRANKING_TIMEOUT` is the default and
    every requirement naming it as the attempt bound means the configured limit in use; their text
    stays valid at the default.
  - `add-speed-governor` (1.5.35): no shared requirement; it only shares the telemetry document
    budget (its 10 slots are already counted below) and the Calibration section.
- Affected code (`master`): `include/Constants.h`, `include/CrankLaw.h`, `include/RelayController.h`,
  `src/RelayController.cpp`, `include/VehicleController.h`, `src/VehicleController.cpp`,
  `include/WebPortal.h`, `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`,
  `include/Changelog.h`, `test/test_crank_law/test_main.cpp`, `WEB_PORTAL_SETUP.md`,
  `MAVLINK_SETUP.md`, `../../API/README.md`.
- **Telemetry budget.** One integer slot, key by pointer, nothing copied: 16 B of memory, ≈ 22 B of
  JSON. `master` / `old-board`: worst case ≈ 8.58 → 8.60 KB, ≈ 0.62 KB under 9216 (size unchanged).
  `aztec`: ≈ 9.05 → 9.07 KB, ≈ 0.15 KB under 9216 — it fits, but the aztec capacity comment already
  says the next telemetry addition on aztec should grow the document, so the aztec merge grows it
  to `StaticJsonDocument<10240>` (+1 KB of .bss, ≈ 1.17 KB headroom) (task 6.1).
- Behaviour change on existing units: none at the default (no stored key → 5000 ms, as today).
- No NVS migration: new key in an existing namespace, missing key reads as the default.
- LittleFS image must be uploaded on every vehicle (new row and i18n keys).
- Branches: `master` first; plain merges `master` → `aztec` and `master` → `old-board`.
