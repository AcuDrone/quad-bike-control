# Change: Wire probe-confirmed ECU PIDs (module voltage, intake air temp, engine load) into telemetry

## Why
The `add-can-pid-probe` bench run on 2026-08-14 (real Bashan 330, Delphi MT05-family ECU, key
on / engine off, probe executed twice with identical output) produced hard evidence of what this
ECU actually answers: 23 supported PIDs, bitmaps that agree with every live answer, and two PIDs
(`0x2F` fuel level, `0x5C` oil temp) confirmed absent. That change deliberately enabled only MAP
(`0x0B`) and left the rest as documented-but-deferred future work
(`openspec/changes/add-can-pid-probe/design.md` → "Probe results (recorded 2026-08-14…)" and
"Implications for a future change (NOT enabled here)").

This is that follow-up change. It reads the three confirmed PIDs that carry real operational
value for a remotely-operated quad — supply voltage, intake air temperature, and engine load —
and pushes them into web telemetry and, where a natural `EFI_STATUS` field exists, into MAVLink.
Since the data is already on the bus and the request budget cost is negligible (see design.md),
we pull what we can read; whether every consumer uses it today is secondary.

## What Changes
- **Add three PIDs to the 2000 ms poll class** (the same interval class as coolant / TPS / MAP),
  taking `PID_COUNT` from **4 to 7** in `include/CANController.h` / `src/CANController.cpp`:
  - `0x42` control module voltage — `((A*256)+B)/1000` V (raw value is exactly millivolts)
  - `0x0F` intake air temperature — `A - 40` °C
  - `0x04` calculated engine load — `A * 100 / 255` %
- **RPM (`0x0C`) keeps its 500 ms cadence unchanged.** The scheduler still keeps exactly one
  request in flight; design.md shows the bus- and slot-occupancy math (≈0.3 % bus load,
  ≈25 % request-slot duty at the measured ~50 ms ECU response time) and the tie-break behaviour
  that governs worst-case RPM jitter.
- **New `VehicleData` fields**: `moduleVoltageMv` (`uint16_t`, millivolts), `intakeTemp`
  (`int8_t`, °C), `engineLoad` (`uint8_t`, %), parsed in `parseAndStore()`.
- **New telemetry JSON fields** inside the existing `can_status == "connected"` block:
  `ecu_voltage` (volts, 2 decimals), `intake_temp` (°C), `engine_load` (%), plumbed through
  `WebPortal::Telemetry` and `TelemetryManager::collectTelemetry()`.
- **Web UI**: three new readouts in the existing "🚗 Vehicle Data (CAN)" card in
  `data/index.html`, with new `data-i18n` keys added to **both** the `en` and `uk` dictionaries
  (parity maintained).
- **MAVLink `EFI_STATUS` mapping** (single message already sent at 5 Hz from
  `src/MavlinkInterface.cpp:262`), using only fields that are currently unused zeros:
  - `intake_manifold_temperature` ← intake air temp (`0x0F`)
  - `ignition_voltage` ← control module voltage (`0x42`)
  - **engine load is NOT mapped to MAVLink**: `EFI_STATUS.engine_load` is already occupied by the
    GEAR encoding (`src/MavlinkInterface.cpp:243-247, 268`) and MUST NOT be overwritten. Engine
    load stays web-only in this change.
  Both new fields are reported as `NaN` while CAN data is invalid, matching the existing
  `rpm` / `cylinder_head_temperature` convention so the Mission Planner plugin shows "--" instead
  of a misleading 0.
- **Populate the two hardcoded-zero MAVLink throttle fields** (`src/MavlinkInterface.cpp:272` and
  `:291`), using the ECU TPS (`0x11`) that is **already polled** and already stored in
  `VehicleData.throttlePosition` — no new request-slot cost:
  - `EFI_STATUS.throttle_out` (today `0.0f`) ← **measured** ECU TPS in %, and `NaN` while CAN data
    is invalid, matching this message's existing NaN-when-invalid convention.
  - `VFR_HUD.throttle` (today `0`; `uint16_t` percent) ← the same measured TPS when CAN is valid,
    **falling back to the commanded/arbitrated throttle percent** when it is not. `VFR_HUD` has no
    NaN encoding, so a HUD throttle bar pinned at 0 while the operator is holding throttle would be
    actively misleading; the locally-known commanded value keeps it honest. See design.md for the
    measured-vs-commanded policy.
  - `EFI_STATUS.throttle_position` stays an unused zero — the same value is not duplicated into two
    fields of one message.
  - **`MavlinkInterface::StateReport` carries neither value today** and gains both:
    `throttlePosition` (`uint8_t`, %, copied from `VehicleData`) and `throttleCmdPct` (`uint8_t`,
    0-100). The commanded value must come from the throttle servo's live pulse width — the
    *arbitrated* output of autopilot / web / gear-boost PID / speed-limit cap / fail-safe idle —
    so a percent accessor is added to `ThrottleController` (`percentToUs()` currently has no
    inverse) and surfaced via `VehicleController`, then copied into the snapshot in `main.cpp`.
- **Explicitly NOT polled** (available per the probe, deliberately deferred to keep the request
  budget small): `0x0E` timing advance, `0x14` O2 sensor B1S1, `0x06`/`0x07` fuel trims,
  `0x03` fuel-system status, `0x13` O2 sensors present, `0x1C` OBD standard, `0x1F` run time,
  `0x21` distance with MIL on, `0x41` monitor status, `0x45` relative throttle, `0x4D` time run
  with MIL on. `0x2F` and `0x5C` remain confirmed-absent and stay disabled.
- **`0x0D` vehicle speed — verified dead, no verification run outstanding**: resolved on the bench
  2026-08-14 (user-verified) — `0x0D` reports **0 at all times, including in motion**. The ECU
  answers the PID but has no vehicle-speed sensor input on this vehicle (dash-wired speedo), so the
  value is permanently 0 and useless. `0x0D` is **NOT** added to the poll table, telemetry, or any
  control path here, and cannot be adopted later. Task 8.4 records this outcome; no comparison
  against the hall sensor is needed. Cross-reference: `add-hall-speed-sensor` (GPIO8/X2) is the
  only speed source on this vehicle and is in no sense redundant.

## Impact
- Affected specs:
  - `can-controller` — **MODIFIED** `CAN Controller OBD-II Data Reading` (poll table gains
    `0x42` / `0x0F` / `0x04`).
  - `web-telemetry` — **ADDED** `ECU Electrical and Air Telemetry` (new JSON fields + CAN-card
    display + i18n parity).
  - `mavlink-interface` — **MODIFIED** `Vehicle State Reporting via Standard MAVLink Messages`
    (`EFI_STATUS` field mapping for intake temp and module voltage; `EFI_STATUS.throttle_out` and
    `VFR_HUD.throttle` populated with measured TPS, with a commanded-throttle fallback for
    `VFR_HUD`).
- Affected code: `include/CANController.h`, `src/CANController.cpp` (PID constants, poll table,
  `VehicleData` fields, `parseAndStore`), `include/WebPortal.h`, `src/WebPortal.cpp`
  (`Telemetry` struct + `createTelemetryJSON`), `src/TelemetryManager.cpp` (field copy),
  `include/MavlinkInterface.h` + `src/MavlinkInterface.cpp` (`StateReport` fields + `EFI_STATUS`
  and `VFR_HUD` pack calls), `include/ThrottleController.h` + `src/ThrottleController.cpp`
  (µs → percent accessor), `include/VehicleController.h` (commanded-throttle percent getter),
  `src/main.cpp` (`StateReport` population), `data/index.html` (CAN card readouts + en/uk i18n
  keys).
- Deployment note: updating `data/index.html` requires `pio run -t uploadfs` (LittleFS upload);
  a firmware flash alone does not update the web UI.
- **Ground-station note**: the Mission Planner plugin decodes `EFI_STATUS` by packet
  subscription (see `include/Constants.h:252-256`). The two new fields are additive — existing
  decodes are untouched — but the plugin must be extended before the values are visible in the
  GCS. That plugin work is outside this repository and outside this change. `VFR_HUD.throttle` is
  the exception: it is a standard HUD field the GCS already renders, so it needs no plugin work.

## Sequencing with active changes
Spec deltas apply to `openspec/specs/` **as they exist today**, so ordering matters where two
active changes touch the same requirement. This change SHOULD land and archive **after**:

1. **`add-can-pid-probe`** — it also `MODIFIED`s `can-controller` →
   `CAN Controller OBD-II Data Reading` (adding the MAP `0x0B` scenario). This change's MODIFIED
   block is written on top of that version and **includes the MAP scenario**, so archiving this
   change last preserves both. Archiving it *before* `add-can-pid-probe` would let the probe
   change's older text overwrite the three new PID scenarios.
2. **`add-hall-speed-sensor`** — it also `MODIFIED`s `mavlink-interface` →
   `Vehicle State Reporting via Standard MAVLink Messages` (adding the `VFR_HUD` ground-speed
   scenarios). This change's MODIFIED block is written on top of that version and **includes the
   `VFR_HUD` scenarios**, so archiving this change last preserves both.

No conflict with `migrate-to-control-v0-board` (its `web-telemetry` and `mavlink-interface`
deltas touch different requirements). The `web-telemetry` delta here is `ADDED`-only and is
orthogonal to `add-hall-speed-sensor`'s `MODIFIED Telemetry Display on Web Interface` and to
`add-can-pid-probe`'s `ADDED Manifold Absolute Pressure Telemetry`.
