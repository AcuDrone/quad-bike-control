## 1. Constants
- [x] 1.1 Add probe-related constants to `include/Constants.h` (e.g. `CAN_PROBE_RETRY_ATTEMPTS = 1`, `PROBE_RESULT_TTL` ms for the fresh-results window) and the MAP PID interval reuse of `CAN_POLL_INTERVAL_TEMP`.

## 2. CANController probe machine + results storage
- [x] 2.1 Add `enum class Mode { NORMAL, PROBING }` and a probe cursor/work-list to `CANController` (`include/CANController.h`); keep the existing `OBDState` machine unchanged.
- [x] 2.2 Add a fixed-size `ProbeResults` model (bitmaps[4], candidate `pids[]`, `dtc`, `state{running,complete,completedAtMs}`, status string, `multiFrameTruncated`) and a `getProbeResults()` getter mirroring `getVehicleData()`.
- [x] 2.3 Add `startProbe()` that: rejects/defers if a gear change is active; short-circuits to a "no ECU" result if `VehicleData.dataValid` is false; otherwise sets `Mode::PROBING` and initializes the cursor.
- [x] 2.4 Implement the probe sequencer in `update()` dispatch: walk Mode 01 bitmaps (0x00 then 0x20/0x40/0x60 only when the previous bitmap's continuation bit is set), then one test request per candidate PID (0x04, 0x0B, 0x0E, 0x0F, 0x42, 0x0D, 0x2F, 0x5C, 0x14), then one Mode 03 request; one request in flight at a time, 200 ms timeout + 1 retry each; reuse `sendOBDRequest` and a generalized receive/drain helper (match 0x41 or 0x43).
- [x] 2.5 Store per-PID supported/answered/raw/decoded and the DTC count+codes; flag `multiFrameTruncated` if a Mode 03 count implies >2 DTCs; set `complete`/`completedAtMs` and return to `Mode::NORMAL` (normal polling resumes where it left off).
- [x] 2.6 Mirror probe progress and the final results table to serial via `Debug::printfFeature(DebugFeature::CAN, ...)`.

## 3. can_probe web command dispatch
- [x] 3.1 Route `can_probe` through `WebPortal::WebCommand` → `VehicleController::processWebCommand` (same dispatch pattern as `steer_cal_*`), calling `CANController::startProbe()` and responding with started / busy-deferred / no-ECU status.

## 4. WebSocket + telemetry plumbing
- [x] 4.1 Add probe snapshot fields and `map_kpa` to the `Telemetry` struct (`include/WebPortal.h`).
- [x] 4.2 In `TelemetryManager` copy `getProbeResults()` into the `Telemetry` struct each cycle, and copy `mapKpa` alongside the other CAN fields (only when `dataValid`).
- [x] 4.3 In `WebPortal::createTelemetryJSON` emit a nested `probe` object only while results are fresh (`complete && now-completedAtMs < PROBE_RESULT_TTL`) or `running`, and emit `map_kpa` inside the existing `can_status == "connected"` block.

## 5. Web UI (data/index.html)
- [x] 5.1 Add a "Probe ECU" button that sends the `can_probe` WebSocket command and disables itself while `probe.running` is true.
- [x] 5.2 Add a probe results panel rendering bitmaps, per-PID supported/answered/raw/decoded, and the DTC summary (with a truncation note when `multiFrameTruncated`), styled with existing Sage Garden `:root` tokens (`--info/--warning/--success/--danger`).
- [x] 5.3 Add a MAP (`map_kpa`) readout to the CAN telemetry card.
- [ ] 5.4 Redeploy the UI with `pio run -t uploadfs` (LittleFS) — a firmware flash alone does not update `data/index.html`. (Deferred: no board connected — no `/dev/cu.wchusbserial*` present.)

## 6. MAP PID enable + VehicleData field
- [x] 6.1 Add `uint8_t mapKpa;` to `CANController::VehicleData` and initialize it in the constructor.
- [x] 6.2 Define `PID_MAP = 0x0B`, add it to the poll table at the `CAN_POLL_INTERVAL_TEMP` (2000 ms) class, bump `PID_COUNT`, and add a `case PID_MAP` in `parseAndStore` decoding `A` (kPa absolute).

## 7. Build
- [x] 7.1 Clean firmware build: `~/.platformio/penv/bin/pio run` passes with no errors.

## 8. Bench verification
- [x] 8.1 Run the probe on the real Bashan 330 via the web UI; confirm supported-PID bitmaps, per-PID answers, and DTC summary appear in the UI and serial log. (Done 2026-08-14 on the real Delphi MT05, bench, key on/engine off; ran twice with identical output. Bitmaps, per-PID answers and the DTC summary were confirmed **in the serial log**; the web-UI panel rendering is still unconfirmed pending the LittleFS upload in 5.4.)
- [x] 8.2 Record the actual probe results (which candidate PIDs answered, raw/decoded values, DTCs) into this change's design.md/proposal.md so a future change can enable the confirmed PIDs. (Done: see "Probe results (recorded 2026-08-14, bench, key on/engine off)" in design.md. Enabling the confirmed PIDs remains future work and is explicitly NOT part of this change.)
- [ ] 8.3 Confirm MAP (`map_kpa`) shows a plausible live value in the CAN card while the engine runs. (Not done: the 2026-08-14 run was engine-off. 0x0B answered 101 kPa = atmospheric, which is plausible but does not prove the value tracks manifold vacuum. Still requires a running-engine check.)

## 9. Validate
- [x] 9.1 `openspec validate add-can-pid-probe --strict` passes with no errors.
