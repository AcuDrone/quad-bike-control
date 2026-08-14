# Change: Add on-demand ECU capability probe and wire MAP PID into telemetry

## Why
The vehicle is a Bashan 330 4x4 quad (287cc single, Delphi MT05-family EFI ECU). The ECU
answers our OBD-II CAN polling today (so it is a CAN-capable variant), but no public PID map
exists for this ECU. We currently poll only three PIDs (0x0C RPM, 0x05 coolant, 0x11 TPS) and
have no way to discover, on the bench, which additional PIDs the ECU actually supports or to
read stored diagnostic trouble codes. This blocks any evidence-based expansion of telemetry.

This change adds a web-triggered, one-shot ECU capability probe that reports exactly what the
ECU answers, and — as the first concrete result of that discovery — wires manifold absolute
pressure (MAP, PID 0x0B) into normal polling and telemetry. Other candidate PIDs are left
disabled here; they will be enabled in a FUTURE change based on the real probe results captured
on the bike.

## What Changes
- Add a **Probe mode** to `CANController`: on a `can_probe` web command, normal PID polling is
  suspended and the controller sequentially (reusing the existing send/receive helpers, one
  request at a time) walks the Mode 01 supported-PID bitmaps (0x00, then 0x20/0x40/0x60 only if
  the previous bitmap indicates more), sends one test request per candidate PID
  (0x04, 0x0B, 0x0E, 0x0F, 0x42, 0x0D, 0x2F, 0x5C, 0x14), and issues one Mode 03 request to read
  the DTC count and codes. Results are stored (per-PID supported/answered/raw/decoded plus a DTC
  summary), then normal polling resumes.
- Add a `can_probe` web command routed through the existing
  `WebPortal::WebCommand` → `VehicleController::processWebCommand` dispatch. The probe is
  deferred/rejected while a gear change is active, and short-circuits to a "no ECU" result when
  CAN is disconnected (`dataValid` false).
- Deliver probe results to the web UI via the existing 5 Hz WebSocket telemetry broadcast (a
  `probe` object embedded while results are fresh — see design.md) and mirror them to the serial
  log via `Debug::printfFeature(DebugFeature::CAN, ...)`.
- Add a "Probe ECU" button and a results panel to the single `data/index.html` web UI.
- **Wire MAP (PID 0x0B)** into normal operation: add it to the active poll table at the 2000 ms
  interval class, add a `mapKpa` field to `CANController::VehicleData`, emit a `map_kpa`
  telemetry JSON field, and display it in the CAN telemetry card.
- Other candidate PIDs (0x04 load, 0x0E timing, 0x0F IAT, 0x42 module voltage, 0x0D speed,
  0x2F fuel, 0x5C oil temp, 0x14 O2) are **not** enabled by this change; they will be enabled in
  a future change once the bench probe confirms the ECU answers them.
- **Bench probe run 2026-08-14** (real Delphi MT05, key on/engine off): results are recorded in
  design.md → "Probe results (recorded 2026-08-14, bench, key on/engine off)". Summary: 23
  supported PIDs; 0x04/0x0B/0x0D/0x0E/0x0F/0x14/0x42 all answered with plausible values; 0x2F and
  0x5C confirmed absent; 0 stored DTCs. Note 0x0D vehicle speed is supported, contradicting this
  change's earlier assumption — it still needs a moving-vehicle check before it can be trusted.
  Enabling any of these PIDs remains future work and is out of scope here.

## Impact
- Affected specs: `can-controller` (ADDED: ECU Capability Probe; MODIFIED: CAN Controller OBD-II
  Data Reading to include MAP 0x0B), `web-control` (ADDED: can_probe command), `web-telemetry`
  (ADDED: probe results payload + `map_kpa`).
- Affected code: `include/CANController.h`, `src/CANController.cpp` (probe state machine, results
  model, MAP PID + `VehicleData.mapKpa`), `src/VehicleController.cpp` (`can_probe` dispatch),
  `src/TelemetryManager.cpp` + `include/WebPortal.h`/`src/WebPortal.cpp` (probe/MAP telemetry
  plumbing), `data/index.html` (probe button, results panel, MAP display), `include/Constants.h`
  (probe/MAP-related constants).
- Deployment note: updating `data/index.html` requires `pio run -t uploadfs` (LittleFS upload),
  not just a firmware flash.
- No change to the RPM-driven ignition/relay logic: typical probe duration (~3 s) stays under
  the 5000 ms stale-data timeout; the rare worst-case tail (~5.6 s) can transiently mark CAN data
  stale, which fails safe — it only defers, never falsifies, engine-running detection (see
  design.md).
