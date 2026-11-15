# Change: Pre-crank delay set from the web (default 3 s) and no stale CAN RPM in the crank logic

## Why
The RC transmitter's ignition switch has only two positions: OFF and the top band, which the
firmware reads as "ON" and answers with an auto-crank (`requestCrank()`). Flipping OFF → ON does
not always start the engine, for two reasons:

1. **The pre-crank dwell is too short and fixed.** `RelayController::update()`
   (`src/RelayController.cpp:476-490`) engages the starter once the ignition / ECU line has been
   powered for `ACC_PRECRANK_DWELL_MS` (2000 ms, `include/Constants.h:249`). The ECU and the fuel
   pump need longer than 2 s before the starter turns, and with a two-position switch the operator
   has no way to wait in ACC first. The value differs per vehicle and can only be changed by a
   reflash.
2. **A stale CAN RPM suppresses or cuts the crank.** `CANController` keeps the last engine RPM in
   `vehicleData_.engineRPM` (`src/CANController.cpp:83`, written at `:512`) and never clears it when
   the ECU loses power; only `dataValid` drops, and only after `CAN_DATA_STALE_TIMEOUT` (5000 ms,
   `:405-408`) or after every PID has failed its retries (`:394-396`). `VehicleController::update()`
   passes the raw value to `relayController_.update(canData.engineRPM)`
   (`src/VehicleController.cpp:298`) with no validity check. If the engine was switched off at or
   above `ENGINE_RUNNING_RPM_THRESHOLD` (1500 on the quad), the next crank request hits "Crank
   suppressed - engine already running" within one loop, the starter never turns, and, because the
   RC switch is edge-triggered, nothing retries. Within the first 5 s after OFF the stale value is
   even reported as *valid*, so the GCS ignition ON (`isEngineRunning()` → "ON with the engine
   running", no crank) and the web START ("Engine already running") are refused too; after 5 s the
   web START reaches CRANKING and the starter is dropped in the same loop ("Engine started!").

## What Changes
- **Configurable pre-crank dwell.** A persisted per-vehicle value, the time the ignition / ECU line
  must be powered before the starter engages on an auto-crank:
  | Setting | NVS | Type | Range | Default |
  |---|---|---|---|---|
  | pre-crank delay | namespace `"crank"` (new, owned by `VehicleController`), key `dwell_ms` | uint16, ms | 1000–8000 | 3000 |
  Constants: `ACC_PRECRANK_DWELL_MS` becomes the default and changes 2000 → **3000**;
  new `CRANK_DWELL_MS_MIN` (1000), `CRANK_DWELL_MS_MAX` (8000), `CRANK_NVS_NAMESPACE` ("crank"),
  `CRANK_DWELL_NVS_KEY` ("dwell_ms"). Loaded in `VehicleController::begin()`; a missing key reads
  as the default; a stored out-of-range value is logged and ignored (default used, not rewritten).
  `VehicleController` hands the value to `RelayController` through a new RAM-only setter
  `setPrecrankDwellMs()`; `RelayController` keeps no NVS of its own for it (design D2).
- **Scope of the dwell.** Every path that arms a crank through `requestCrank()` uses the configured
  value: a fresh RC ON, the re-crank on return from fail-safe or web control, and the GCS
  `MAV_CMD_USER_2` ignition ON. The dwell is still counted from the moment the ignition line came
  up (an ignition already powered for longer cranks at once). The web START button is unchanged: it
  sets CRANKING directly, with no dwell (user decision 2026-11-15, design D5).
- **Validated RPM for the crank logic.** `RelayController::update()` receives the engine RPM only
  when it is valid, 0 otherwise. On `master` / `old-board` that is `crankRpm(canData.dataValid,
  canData.engineRPM)`; on `aztec` the existing effective RPM `getEngineRpm()` (tacho first, 0 when
  no source is valid) already has this property and stays the input.
- **Engine RPM cleared at ignition OFF.** When the applied relay ignition state goes from any
  non-OFF state to OFF (RC, GCS, web, fail-safe `allOff()`, starter escalation), `VehicleController`
  calls a new `CANController::clearEngineRpm()`, which sets `vehicleData_.engineRPM` to 0 and leaves
  `dataValid`, `lastUpdateTime` and the other ECU fields untouched (design D4). With the ECU
  unpowered the engine cannot be running, so 0 is the true value; the next ECU reply overwrites it.
  This fixes `isEngineRunning()` (GCS ignition ON, RC gear retry), the web START interlock and, on
  `aztec`, the `canValid && canRpm > 0` fallback of `tachoSelectRpm()` in one place.
- **Pure law** `include/CrankLaw.h` (header-only, no Arduino) with native tests
  `test/test_crank_law`: dwell validation and or-default, wrap-safe dwell-elapsed, the armed-crank
  decision (WAIT / SUPPRESS / ENGAGE), the validated crank RPM and the ignition-OFF edge.
  `RelayController::update()` calls the armed-crank decision instead of its inline comparison; nothing else in `RelayController` moves.
- **Web.** Command `set_crank_dwell_ms` (value ms, rounded to a whole ms, validated 1000–8000,
  persisted first, then applied), accepted from any input source. Replies: "Pre-crank delay set to
  N ms", "Pre-crank delay out of range (1000-8000 ms)", "Failed to save the pre-crank delay".
  Telemetry key `crank_dwell_ms` (int). In the "🔑 Запалювання та освітлення" / "🔑 Vehicle
  Ignition & Lighting" card, right under the OFF / ACC / START buttons: a row "Затримка стартера,
  с" / "Pre-crank delay, s" (number input in seconds 1–8, step 0.5, Save → `saveCrankDwell()`,
  sends `value × 1000`), local validation, a hint, its own reply line and a help sentence; uk + en.
- **Docs.** `MAVLINK_SETUP.md` ignition row ("2 s pre-crank" → the configurable delay, default 3 s)
  and the API compatibility page (`../../API/README.md`, version note); `WEB_PORTAL_SETUP.md` if it
  lists the ignition card settings.
- **Version.** Firmware **1.5.33** on all three branches (`master` and `old-board` skip 1.5.32,
  which is the aztec-only gear speed ceiling); changelog entry uk + en ("багі" in Ukrainian).

Out of scope (recorded in design.md): the "Ignition Safety Interlocks" spec drift (brake ≥ 20 %,
1100 RPM — not implemented in code); auto-retry after a failed crank; debouncing the "engine
started" sample; forcing idle throttle while cranking; the `clampUs` / absent-channel finding;
putting the web START button through the dwell (decided by the user on 2026-11-15: START stays
without the dwell);
faster RPM polling while cranking (considered and declined by the owner on 2026-11-15; the RPM
PID stays at `CAN_POLL_INTERVAL_RPM`).

## Impact
- Affected specs:
  - `vehicle-systems`: MODIFIED "Ignition State Control Integration"; ADDED "Configurable Pre-Crank
    Dwell", "Validated Engine RPM for Crank Control"; MODIFIED "GCS Vehicle Command Interlocks"
    (added by `add-gcs-vehicle-commands`, archived 2026-11-15, see below).
  - `mavlink-interface`: MODIFIED "Ignition State Broadcast" (added by `add-gcs-vehicle-commands`).
  - `gpio-expander`: MODIFIED "Relay Output Expander" (the "Public API is unchanged" scenario named
    `ACC_PRECRANK_DWELL_MS` as fixed).
  - `web-control`: ADDED "Pre-Crank Delay Configuration via Web Interface".
  - `web-telemetry`: ADDED "Pre-Crank Delay Telemetry".
- **Dependency on `add-gcs-vehicle-commands`.** Two MODIFIED requirements here came from that
  change (implemented in 1.5.30, bench pending). It was archived on `master` on 2026-11-15
  (`archive/2026-11-15-add-gcs-vehicle-commands`), so both requirements are now in the main specs;
  the copies here match them apart from this change's own edits (task 0.2).
- Affected code (`master`): `include/CrankLaw.h` (new), `include/Constants.h`,
  `include/RelayController.h`, `src/RelayController.cpp`, `include/CANController.h`,
  `src/CANController.cpp`, `include/VehicleController.h`, `src/VehicleController.cpp`,
  `include/WebPortal.h`, `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`,
  `include/Changelog.h`, `platformio.ini`, `test/test_crank_law/test_main.cpp` (new),
  `MAVLINK_SETUP.md`, `../../API/README.md`.
- Behaviour change on existing units: with no stored key the auto-crank waits 3 s instead of 2 s
  (intended). The engine RPM reads 0 right after ignition OFF instead of keeping the last value for
  up to 5 s (web telemetry, `EFI_STATUS.rpm`); every other ECU field is unchanged.
- No NVS migration: new namespace, missing key reads as the default.
- LittleFS image must be uploaded on every vehicle (new row and i18n keys).
- Branches: `master` first; merge `master` → `aztec` (keep `relayController_.update(getEngineRpm())`)
  and `master` → `old-board` (GPIO `RelayController`, same changes by hand where the merge conflicts).
