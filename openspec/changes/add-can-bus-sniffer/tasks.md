## 0. Preconditions
- [x] 0.1 Version slot: `master` is at 1.5.6; take **1.5.7** (patch: diagnostic convenience, no recalibration or wiring change).
- [x] 0.2 Branch: implement on `master`, then merge `master` into `aztec`.

## 1. CAN controller
- [x] 1.1 `include/Constants.h`: `CAN_SNIFF_MAX_IDS` (32), `CAN_SNIFF_DRAIN_PER_UPDATE` (32), `CAN_SNIFF_WINDOW_MS` (10000), `CAN_SNIFF_MAX_WINDOW_MS`, `SNIFF_RESULT_TTL`.
- [x] 1.2 `CANController.h`: `SniffEntry`, `SniffResults`, `SniffStart`; `startSniff(ms, gearChangeActive)`, `sniffActive()`, `getSniffResults()` (const ref); `Mode::SNIFFING`.
- [x] 1.3 `startSniff()`: refuse while not initialised, probe running, LWS zero-set running, gear change, or already sniffing; reset results; program masks 0 + std/ext filters (`applySniffFilters()`); enter `SNIFFING`.
- [x] 1.4 `updateSniff()`: drain up to 32 frames per call; route 0x305 to `LwsSensor` too; table update (count, DLC, data, changed mask); IDs beyond 32 counted distinct (dedup list of 32 more IDs, then saturated); total frames; EFLG RX0OVR/RX1OVR counted and cleared.
- [x] 1.5 `finishSniff()`: restore with `applyRxFilters()` (full re-init via `recoverFromBusOff()` on failure), `state_ = IDLE`, postpone the no-data re-init watchdog, results marked complete with completion time.
- [x] 1.6 `startProbe()` returns `BUSY` while sniffing; `startLwsZero()` returns `BUSY` while sniffing.

## 2. Command and telemetry
- [x] 2.1 `VehicleController`: `can_sniff` command → `processCanSniffCommand()` with English reply messages ("CAN sniffer started", "CAN sniffer refused: ECU probe active", ...); `getSniffResults()` accessor.
- [x] 2.2 `WebPortal::Telemetry`: `const CANController::SniffResults* sniff` (nullptr unless running or fresh; pointer, not a copy, to keep the loop stack small); filled in `TelemetryManager`.
- [x] 2.3 `createTelemetryJSON()`: `sniff` object `{run, done, win, el, fr, ovf, xid, xsat, ids:[{id, x, n, d, c}]}`; static document 4608 → 8192 with the capacity comment updated.

## 3. Web UI
- [x] 3.1 Button "CAN sniffer (10 s)" next to "Probe ECU", status line, results panel.
- [x] 3.2 Table ordered by ID: ID (hex, EXT), count, Hz, data with changed bytes highlighted; summary line.
- [x] 3.3 Reply handling for the refusal messages; uk/en strings.

## 4. Version and verification
- [x] 4.1 `FIRMWARE_VERSION` 1.5.7; `Changelog.h` uk/en line.
- [x] 4.2 `pio run -e esp32-s3-devkitc-1`, `pio test -e native`, `pio run -t buildfs` pass.
- [x] 4.3 Commit on `master`; merge into `aztec` (build there too).
- [ ] 4.4 Bench (aztec, standing still, engine idling): run the sniffer, note the ECU's own IDs; check that the LWS and OBD polling recover afterwards without a reboot.
