# Change: Remap EFI_STATUS onto native fields and report both assumed and physical gear

## Why
`EFI_STATUS` (component 25, 5 Hz) currently carries the GEAR encoding in `engine_load`. That was a
pragmatic choice when gear was the only non-engine value we had to ship, but it has two costs that
have now both come due:

1. **It blocks a real value.** The ECU's own calculated engine load (PID `0x04`) has been polled
   since `add-ecu-telemetry-pids` and reaches the web UI, but `add-ecu-telemetry-pids` had to
   explicitly refuse to map it to MAVLink ("engine load is NOT mapped to MAVLink:
   `EFI_STATUS.engine_load` is already occupied by the GEAR encoding … and MUST NOT be
   overwritten"). MAP (`0x0B`) is in the same position — polled, on the web UI, and left out of
   MAVLink even though `intake_manifold_pressure` is free and is literally the field for it.
2. **Only the assumed gear is reported.** The transmission is commanded time-based/sensorless, but
   the vehicle *does* have opto gear switches (`TransmissionController::getPhysicalGear()`), which
   the fail-safe and the `VISION_SPEED_ESTIMATE` direction sign already trust. A consumer watching
   MAVLink today cannot tell "the controller believes it is in LOW" from "the box is measurably in
   LOW" — precisely the distinction that matters when a shift fails.

This change puts every value in the `EFI_STATUS` field that actually means it, and reports the
assumed and physical gear as two separate values in the permanently-free fuel fields. The fuel
fields are safe to repurpose forever on this vehicle: the `add-can-pid-probe` bench run
(2026-08-14) confirmed `0x2F` (fuel level) and `0x5C` (oil temp) are **absent** on this ECU, so no
genuine fuel quantity or flow can ever compete for them.

## What Changes
- **BREAKING (ground-station plugin):** gear moves out of `EFI_STATUS.engine_load` into
  `EFI_STATUS.fuel_consumed`, and the measured ECU throttle position moves out of
  `EFI_STATUS.throttle_out` into `EFI_STATUS.throttle_position`. The external QuadBike Mission
  Planner plugin decodes `EFI_STATUS` by packet subscription and MUST be updated in lockstep with
  the firmware flash. See "Failure mode" below and `MAVLINK_SETUP.md` →
  "Ground-station plugin compatibility".
- **Full native remap of `EFI_STATUS`** (one message, comp 25, 5 Hz, unchanged rate):

  | Field | Content | Validity |
  |---|---|---|
  | `rpm` | engine RPM (`0x0C`) | `NaN` if `!canValid` |
  | `cylinder_head_temperature` | coolant °C (`0x05`) | `NaN` if `!canValid` |
  | `intake_manifold_temperature` | intake air °C (`0x0F`) | `NaN` if `!canValid` |
  | `intake_manifold_pressure` | **MAP kPa (`0x0B`) — NEW** | `NaN` if `!canValid` |
  | `engine_load` | **ECU calculated load % (`0x04`) — NEW** (gear moves out) | `NaN` if `!canValid` |
  | `throttle_position` | **measured TPS % (`0x11`)** (moved from `throttle_out`) | `NaN` if `!canValid` |
  | `throttle_out` | **commanded/arbitrated throttle %** — NEW | always valid |
  | `ignition_voltage` | module supply voltage V (`0x42`) | `NaN` if `!canValid` |
  | `fuel_consumed` | **ASSUMED gear** (repurposed) `[R,N,H,L] = [-1,0,1,2]`, 0.5 midpoint staircase while shifting | always valid |
  | `fuel_flow` | **PHYSICAL gear** (repurposed, same encoding, opto switches) | **`NaN` when UNKNOWN** |
  | `pt_compensation` | digital-output bitmask (`EFI_DIGITAL_FLAG_*`) | always valid |
  | `health` | `1` | — |

- **Two gear values instead of one.** `fuel_consumed` = intent (what the controller commanded,
  including the mid-shift midpoint staircase already specified today). `fuel_flow` = measurement
  (what the opto switches see), `NaN` whenever the physical gear reads UNKNOWN — mid-shift,
  ambiguous switch pattern, or a faulted input expander. **`fuel_flow = NaN` while shifting is the
  expected signature, not a fault**; a *persistent* disagreement between the two after a shift
  settles is the interesting condition.
- **`fuel_flow`'s validity is gear-sensor-driven, not CAN-driven** — it is the only `EFI_STATUS`
  field whose `NaN` does not track `canValid`.
- **`VFR_HUD` is unchanged in behaviour.** `throttle` stays measured-TPS-with-commanded-fallback
  (a `uint16_t` percent has no `NaN` encoding, and a HUD bar frozen at 0 while throttle is applied
  misleads in the more dangerous direction). Only the "ambiguity tell" moves with the data: the
  field that reads `NaN` in the same tick to mark the fallback is now `throttle_position`, not
  `throttle_out` — `throttle_out` is now always valid because it *is* the commanded value.
- **The two stale `NAMED_VALUE_FLOAT` `GEAR` scenarios are corrected.** No `NAMED_VALUE_FLOAT` is
  sent by the shipped firmware — the value has been in `EFI_STATUS` since the message-id collision
  problem was solved — so the spec text is simply false today. The scenario headers are kept
  (`openspec validate --strict` rejects a MODIFIED block that drops a scenario the current spec
  still has), but their bodies are rewritten onto `fuel_consumed` and now state explicitly *why*
  `NAMED_VALUE_FLOAT` is not used.
- **No new constants, no new PIDs, no new poll-table entries, no new `VehicleController` API.** MAP
  and engine load are already in `VehicleData`; `getCurrentGearString()` already returns the
  physical gear and already yields `"?"` for `GEAR_UNKNOWN`. This change is a remap plus three
  snapshot fields.
- **Not touched:** `travelDirection` / `VISION_SPEED_ESTIMATE`, `EFI_DIGITAL_FLAG_*`, the CAN poll
  schedule, web telemetry, and the web UI.

**Failure mode if the plugin is not updated in lockstep:** an old plugin reading gear from
`engine_load` will render the ECU's calculated load (0–100 %) as a gear value, i.e. a wildly
out-of-range "gear" that tracks engine effort. It fails loudly rather than silently, but it fails.
Flash the firmware and update the plugin together; the plugin should additionally treat any gear
value outside `-1..2` as "--".

## Impact
- Affected specs:
  - `mavlink-interface` — **MODIFIED** `Vehicle State Reporting via Standard MAVLink Messages`
    (full `EFI_STATUS` field remap, dual gear reporting, extended `NaN` policy, breaking-consumer
    scenario; the two `NAMED_VALUE_FLOAT` gear scenarios removed).
- Affected code: `include/MavlinkInterface.h` (three new `StateReport` fields, `encodeGear`
  fallback parameter), `src/MavlinkInterface.cpp` (`encodeGear`, `report()` locals and the
  `EFI_STATUS` pack call, field-mapping comment block, `VFR_HUD` comment), `src/main.cpp`
  (`StateReport` population), `MAVLINK_SETUP.md` (reported-state table, gear section, new
  ground-station plugin compatibility subsection).
- Out of repo: the QuadBike Mission Planner plugin must be updated in lockstep (migration table in
  `MAVLINK_SETUP.md`).
- No NVS keys, no web API change, no `data/index.html` change — a LittleFS upload is **not**
  required for this change.

## Sequencing
Spec deltas apply to `openspec/specs/` **as they exist at archive time**, and three active changes
all `MODIFY` the same requirement (`mavlink-interface` → *Vehicle State Reporting via Standard
MAVLink Messages*). This change's MODIFIED block is written as a superset of the other two, so it
MUST archive **last**:

1. **`add-hall-speed-sensor`** — adds the `VFR_HUD` ground-speed scenarios. Included here.
2. **`add-ecu-telemetry-pids`** — adds the intake-temp / module-voltage / throttle scenarios and
   the `VFR_HUD` commanded-throttle fallback. Included here, with the throttle scenarios retargeted
   to the new fields. **This change's MODIFIED block is based on
   `openspec/changes/add-ecu-telemetry-pids/specs/mavlink-interface/spec.md`, not on the current
   `openspec/specs/` text.**
3. **`remap-efi-native-fields`** (this change) — archives last.

Archiving out of order silently reverts the field mapping: an earlier change's older requirement
text would overwrite this one wholesale, leaving `openspec/specs/` claiming gear lives in
`engine_load` while the firmware ships it in `fuel_consumed`. Before archiving, re-read
`openspec/specs/mavlink-interface/spec.md` and confirm the `VFR_HUD` ground-speed and
intake-temp/voltage scenarios are present in this delta (they are) — if either of the two
predecessors is still unarchived, archive it first.

`add-extnav-velocity` is **not** a sequencing constraint: its `mavlink-interface` delta is
`ADDED`-only (*Autopilot Attitude Subscription*, *External Navigation Velocity Reporting*) and
touches a different requirement, so the two can archive in any order.
