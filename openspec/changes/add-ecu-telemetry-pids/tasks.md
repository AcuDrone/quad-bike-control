## 1. CANController: PID constants and data fields
- [ ] 1.1 Define `PID_MODULE_VOLTAGE = 0x42`, `PID_INTAKE_TEMP = 0x0F`, `PID_ENGINE_LOAD = 0x04` in `include/CANController.h` next to the existing PID constants; leave `0x2F` / `0x5C` commented out (confirmed absent).
- [ ] 1.2 Add `uint16_t moduleVoltageMv;` (raw `(A*256)+B` = millivolts), `int8_t intakeTemp;` (°C) and `uint8_t engineLoad;` (%) to `CANController::VehicleData`, with Doxygen-style unit comments.
- [ ] 1.3 Initialize all three to 0 in the `CANController` constructor alongside the existing `vehicleData_` defaults.

## 2. CANController: poll table and parsing
- [ ] 2.1 Bump `PID_COUNT` from 4 to 7.
- [ ] 2.2 Add the three entries to `pidTable_` at the `CAN_POLL_INTERVAL_TEMP` (2000 ms) class: indices 4/5/6 = `0x42`, `0x0F`, `0x04`. Keep index 0 = RPM at `CAN_POLL_INTERVAL_RPM` (500 ms) — `setRPMPollInterval()` writes `pidTable_[0]` and MUST keep working.
- [ ] 2.3 Add `case PID_MODULE_VOLTAGE` (`len >= 2` → `((data[0]*256)+data[1])`), `case PID_INTAKE_TEMP` (`len >= 1` → `data[0] - 40`) and `case PID_ENGINE_LOAD` (`len >= 1` → `(data[0]*100)/255`) to `parseAndStore()`.
- [ ] 2.4 (Optional hardening, see design.md) Stagger the initial `nextPollTime` of the three new entries (e.g. +150/+300/+450 ms) so the power-on tie-break burst does not delay the first RPM sample.
- [ ] 2.5 Extend the rate-limited `[CAN]` data log line in `update()` to include voltage / IAT / load so the values are visible on serial without the web UI.

## 3. Telemetry plumbing
- [ ] 3.1 Add `uint16_t module_voltage_mv;`, `int8_t intake_temp;`, `uint8_t engine_load;` to `WebPortal::Telemetry` in the "CAN bus vehicle data" block (`include/WebPortal.h`).
- [ ] 3.2 In `TelemetryManager::collectTelemetry()` copy the three fields inside the existing `if (vehicleData.dataValid)` branch, and zero them in the `else` branch (same pattern as `map_kpa`).
- [ ] 3.3 In `WebPortal::createTelemetryJSON()` emit `ecu_voltage` (as `serialized(String(mv / 1000.0f, 2))`), `intake_temp` and `engine_load` inside the existing `can_status == "connected"` block, next to `map_kpa`.

## 4. Web UI (data/index.html)
- [ ] 4.1 Add three `telemetry-item` blocks to the "🚗 Vehicle Data (CAN)" card with ids `can-ecu-voltage`, `can-intake-temp`, `can-engine-load` and `data-i18n` labels `lbl_ecu_voltage`, `lbl_intake_temp`, `lbl_engine_load`.
- [ ] 4.2 Update the telemetry handler to populate the three ids when `can_status == "connected"`, and to reset them to `-- V` / `--°C` / `--%` in the disconnected branch (mirroring the existing `can-map` handling).
- [ ] 4.3 Add `lbl_ecu_voltage` / `lbl_intake_temp` / `lbl_engine_load` to **both** the `en` and `uk` dictionaries in the CAN-card section (e.g. uk: "Напруга ЕБК", "Темп. впуск. повітря", "Навантаження двигуна").
- [ ] 4.4 i18n parity check: confirm the `en` and `uk` dictionaries have identical key sets (no key added to one only).

## 5. Budget checks
- [ ] 5.1 `StaticJsonDocument` headroom check: log `doc.memoryUsage()` from `createTelemetryJSON()` both in steady state and with the transient `probe` object present; confirm it stays under the 4096-byte capacity (bump the capacity if not) and record both numbers in design.md.
- [ ] 5.2 Measure the serialized steady-state payload length (`json.length()`) and compare against the `web-telemetry` → *Telemetry Performance* "under 1KB" figure. If it is already over budget before this change's ~54 bytes, report it as a pre-existing finding — do not silently amend that requirement here.

## 6. MAVLink EFI_STATUS mapping
- [ ] 6.1 Add `int8_t intakeTemp;` and `uint16_t moduleVoltageMv;` to `MavlinkInterface::StateReport` (`include/MavlinkInterface.h`), keeping the struct free of CAN headers; populate them at the `report()` call site from `VehicleData`.
- [ ] 6.2 In `MavlinkInterface::report()` map intake air temp to `intake_manifold_temperature` and module voltage (mV/1000.0f) to `ignition_voltage` in the `mavlink_msg_efi_status_pack()` call; both `NaN` when `state.canValid` is false, matching `rpmVal`/`chtVal`.
- [ ] 6.3 Do NOT touch `engine_load` (carries GEAR) or `pt_compensation` (carries the digital-output bitmask); update the field-mapping comment block above the pack call to list the two new fields.

## 7. Build gate
- [ ] 7.1 `~/.platformio/penv/bin/pio run` completes with no errors or new warnings.

## 8. Bench verification
- [ ] 8.1 Flash firmware and run `pio run -t uploadfs` (LittleFS) so the updated `data/index.html` is actually served; confirm the three new readouts appear in the CAN card in both `en` and `uk`.
- [ ] 8.2 Engine-running bench check: confirm `ecu_voltage` reads ~13.5-14.5 V with the engine running (vs ~12.8 V key-on/engine-off) and drops under an applied load (headlight / cranking), `intake_temp` is plausible for ambient, `engine_load` rises off 0 % with throttle, and `map_kpa` tracks manifold vacuum (falls well below 101 kPa at idle) — closing the inherited open item from `add-can-pid-probe` task 8.3.
- [ ] 8.3 Gear-change check: perform several shifts and confirm the boost PID still behaves with 7 PIDs in the scheduler (RPM feedback cadence during the shift is unchanged, no new "Timeout for PID" warnings).
- [ ] 8.4 **`0x0D` verification (report-only)**: with the vehicle actually moving, log OBD-II `0x0D` alongside the hall speed sensor reading and compare. Record the outcome (tracks / always 0 / offset) in this change's design.md and cross-reference `add-hall-speed-sensor`. Do NOT wire `0x0D` into `pidTable_`, telemetry or any control path in this change.
- [ ] 8.5 Confirm no CAN regressions: `[CAN] health` lines stay quiet, no new RX overflow reports, `can_status` remains "connected" throughout.

## 9. Validate
- [ ] 9.1 `openspec validate add-ecu-telemetry-pids --strict` passes with no errors.
- [ ] 9.2 Confirm the archive order stated in proposal.md still holds (this change archives after `add-can-pid-probe` and `add-hall-speed-sensor`); if either has already archived, re-check the MODIFIED blocks against the updated `openspec/specs/` before archiving.
