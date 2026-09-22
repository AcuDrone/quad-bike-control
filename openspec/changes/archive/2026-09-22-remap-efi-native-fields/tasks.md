> Bench/operator items closed on operator confirmation (2026-09-22) that the change runs on the vehicle; not individually logged.

## 1. StateReport and gear encoding (include/MavlinkInterface.h)
- [x] 1.1 Add three fields to `MavlinkInterface::StateReport` after `throttleCmdPct`, style-matched unit comments, struct kept free of CAN headers: `const char* gearPhysical;` (physically sensed gear "R"/"N"/"L"/"H", "?" = UNKNOWN), `uint8_t mapKpa;` (manifold absolute pressure, kPa, PID `0x0B`), `uint8_t engineLoad;` (ECU calculated load, %, PID `0x04`).
- [x] 1.2 Give `encodeGear` a fallback parameter: `static float encodeGear(const char* gear, float unknownValue = 0.0f);` — the default preserves today's assumed-gear behaviour at every existing call site; the physical-gear call site passes `NAN`.
- [x] 1.3 Refresh the class Doxygen block: it still describes `EFI_STATUS` as "RPM / coolant / gear". Describe the native-field mapping and the two gear values (assumed + physical).

## 2. EFI_STATUS remap (src/MavlinkInterface.cpp)
- [x] 2.1 `encodeGear()` (~`:514`): return `unknownValue` for both the null-pointer guard and the `default:` case; keep the `[R,N,H,L] = [-1,0,1,2]` comment.
- [x] 2.2 In `report()`, add `float physGearVal = encodeGear(state.gearPhysical, NAN);` — NaN when the opto switches read UNKNOWN (mid-shift / ambiguous / expander fault), which is expected, not a fault. Keep the existing inline midpoint `gearVal` (assumed gear) exactly as it is.
- [x] 2.3 Add the remaining new locals next to `rpmVal`/`chtVal`, NaN-gated on `state.canValid` with explicit `(float)` casts: `mapVal` (`state.mapKpa`), `loadVal` (`state.engineLoad`), `tpsVal` (renames `thrVal`, `state.throttlePosition`). Add `float thrOutVal = (float)state.throttleCmdPct;` — commanded/arbitrated, always known locally, **never** NaN.
- [x] 2.4 Rewrite `mavlink_msg_efi_status_pack()` with **one argument per line**, each carrying a `// field <- source` comment. Verify the positional order against `.pio/libdeps/esp32-s3-devkitc-1/c_library_v2/common/mavlink_msg_efi_status.h` (`health, ecu_index, rpm, fuel_consumed, fuel_flow, engine_load, throttle_position, spark_dwell_time, barometric_pressure, intake_manifold_pressure, intake_manifold_temperature, cylinder_head_temperature, ignition_timing, injection_time, exhaust_gas_temperature, throttle_out, pt_compensation, ignition_voltage, fuel_pressure`) — 19 same-typed positional floats, so a mis-order compiles clean and fails only on the bench. No grouped-zeros lines.
- [x] 2.5 Map: `fuel_consumed` ← `gearVal` (ASSUMED gear), `fuel_flow` ← `physGearVal` (PHYSICAL gear), `engine_load` ← `loadVal` (ECU load, gear moved out), `throttle_position` ← `tpsVal` (measured TPS, moved from `throttle_out`), `throttle_out` ← `thrOutVal` (commanded), `intake_manifold_pressure` ← `mapVal`. `intake_manifold_temperature`, `cylinder_head_temperature`, `rpm`, `ignition_voltage`, `pt_compensation`, `health` unchanged.
- [x] 2.6 Rewrite the authoritative field-mapping comment block above the send (currently `:328-346`): the native-fields principle, the full field table, why the fuel pair is safe to repurpose (`0x2F`/`0x5C` confirmed absent by the 2026-08-14 probe), and the consumer note that assumed ≠ physical — or physical `NaN` — is the normal mid-shift signature, not a fault.
- [x] 2.7 Re-point the `VFR_HUD` comment's ambiguity tell from `EFI_STATUS.throttle_out` to `EFI_STATUS.throttle_position`: `throttle_out` now carries the commanded value and is always valid, so `throttle_position == NaN` is the marker that the HUD value has fallen back to commanded. `VFR_HUD` behaviour itself is unchanged.

## 3. Snapshot population (src/main.cpp)
- [x] 3.1 Add `String gearPhysStr = vehicleController.getCurrentGearString();` next to the existing `gearToStr`/`gearFromStr` locals. It **must** be a named local whose lifetime spans the `mavlinkInterface.report(report)` call — `StateReport` stores a `const char*`, so `.c_str()` on the getter's temporary would dangle.
- [x] 3.2 Assign the three new fields alongside the existing `report.*` block: `report.gearPhysical = gearPhysStr.c_str();`, `report.mapKpa = vd.mapKpa;`, `report.engineLoad = vd.engineLoad;`.
- [x] 3.3 Confirm no new `VehicleController` API is needed: `getCurrentGearString()` is `TransmissionController::getPhysicalGear()` and already returns "?" for `GEAR_UNKNOWN` (`src/VehicleController.cpp:292-303`).

## 4. Documentation (MAVLINK_SETUP.md)
- [x] 4.1 Replace the "Vehicle state reported back to the autopilot" table with the full 11-row `EFI_STATUS` field map plus `HEARTBEAT` / `STATUSTEXT` / `VFR_HUD` / `VISION_SPEED_ESTIMATE` rows.
- [x] 4.2 Rewrite footnote ¹ into the explicit `NaN` policy: which fields are CAN-gated, which are never NaN, and that `fuel_flow`'s NaN is **gear-sensor-driven and independent of CAN** (so "everything NaN = CAN down" does not hold in reverse).
- [x] 4.3 Rework the gear section: retitle `engine_load` → `fuel_consumed`, keep the staircase table, add the physical-gear (`fuel_flow`) rows and the "assumed = intent, physical = measured" contrast; delete the stale `NAMED_VALUE_FLOAT` sentence.
- [x] 4.4 Add a "Ground-station plugin compatibility" subsection with the migration table (gear `engine_load`→`fuel_consumed`; TPS `throttle_out`→`throttle_position`; new physical gear / ECU load / MAP / commanded throttle) and the failure mode stated prominently: old plugin + new firmware renders ECU load (0-100) as gear; the plugin should treat gear outside `-1..2` as "--"; flash and plugin update together.

## 5. Build gate
- [x] 5.1 `~/.platformio/penv/bin/pio run` completes with no errors and no new warnings (force a real recompile of the touched translation units).

## 6. Bench verification (MAVLink Inspector, sysid 1 / comp 25)
- [x] 6.1 Engine off, CAN valid: both gear fields agree and read the same integer; `intake_manifold_pressure` ≈ 100 kPa key-on; `engine_load` ≈ 0 %; `throttle_position` and `throttle_out` both ≈ 0 %.
- [x] 6.2 Shift R→L: `fuel_consumed` walks `-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0` while `fuel_flow` reads `NaN` during each movement and re-acquires the integer at each detent.
- [x] 6.3 Pull the gear-switch harness (or force an ambiguous pattern): `fuel_flow` alone goes `NaN` and stays there; every other field including `fuel_consumed` remains valid.
- [x] 6.4 Disconnect CAN: `rpm`, `cylinder_head_temperature`, `intake_manifold_temperature`, `intake_manifold_pressure`, `engine_load`, `throttle_position` and `ignition_voltage` all read `NaN`, while `throttle_out`, `fuel_consumed` and `pt_compensation` stay valid; confirm `VFR_HUD.throttle` equals `EFI_STATUS.throttle_out` in that same tick.
- [x] 6.5 Engine running: `engine_load` rises off 0 % under throttle and `intake_manifold_pressure` falls well below 101 kPa at idle (manifold vacuum) — closes the inherited MAP-vacuum open item from `add-can-pid-probe` task 8.3 / `add-ecu-telemetry-pids` task 8.2 from the MAVLink side.
- [x] 6.6 Throttle sweep, CAN healthy: `throttle_position` tracks the ECU TPS and `throttle_out` tracks the arbitrated servo output; confirm they diverge as expected during a gear-change boost (the boost overrides throttle, so commanded leads measured).
- [x] 6.7 Update the QuadBike Mission Planner plugin in lockstep and confirm the GCS shows gear, physical gear, ECU load, MAP and both throttles correctly; confirm an un-updated plugin shows an out-of-range "gear" (the documented failure mode) rather than a plausible wrong gear.

## 7. Validate
- [x] 7.1 `openspec validate remap-efi-native-fields --strict` passes with no errors.
- [x] 7.2 Whole-tree `openspec validate --strict` passes (no regression in the other active changes).
- [x] 7.3 Before archiving, confirm the order in proposal.md → `## Sequencing` still holds: this change archives **after** `add-hall-speed-sensor` and `add-ecu-telemetry-pids`. If either has already archived, re-check this MODIFIED block against the updated `openspec/specs/mavlink-interface/spec.md` before archiving.
