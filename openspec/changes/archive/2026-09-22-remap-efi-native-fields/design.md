## Context
`MavlinkInterface::report()` (`src/MavlinkInterface.cpp:307`) sends one `EFI_STATUS` per 5 Hz tick
from component 25. The message has 19 float fields; the firmware currently uses six of them and
transmits the rest as zeros. One of the six is a deliberate repurposing: `engine_load` carries the
GEAR encoding (`[R,N,H,L] = [-1,0,1,2]`, midpoint while shifting), by an explicit contract with the
external QuadBike Mission Planner plugin (`include/Constants.h:252-256`).

Two prior changes accumulated pressure against that contract:

- `add-can-pid-probe` enabled MAP (`0x0B`) and left `intake_manifold_pressure` unmapped, recording
  the question "Should MAP also be mapped to the free `EFI_STATUS.intake_manifold_pressure`?" as an
  open question.
- `add-ecu-telemetry-pids` enabled engine load (`0x04`) and had to state outright that it could not
  be mapped, because gear owned the field. It also parked measured TPS in `throttle_out` while
  leaving `throttle_position` — the field named after the value — unused, on the "don't duplicate a
  value into two fields of one message" rule.

The vehicle also has opto gear switches that the fail-safe and the `VISION_SPEED_ESTIMATE`
direction sign already trust (`TransmissionController::getPhysicalGear()`,
`VehicleController::getTravelDirection()`), but MAVLink reports only the commanded gear.

This change is therefore a *breaking* remap rather than another additive patch: the plugin contract
has to move anyway to unblock two real values, so it moves once, completely, to the semantically
correct fields.

## Goals / Non-Goals
- **Goals**: every value transmitted in `EFI_STATUS` sits in the field that names it; MAP and ECU
  engine load reach MAVLink; measured and commanded throttle are reported side by side and
  distinguishable; assumed and physical gear are reported as two independent values with an honest
  "unknown" encoding for the physical one; `MAVLINK_SETUP.md` documents the migration well enough
  that the out-of-repo plugin can be updated from it alone.
- **Non-Goals**: no new PIDs, poll-table entries or scheduler work; no `VFR_HUD` behaviour change;
  no `VISION_SPEED_ESTIMATE` change; no web telemetry or UI change; no new `VehicleController` or
  `TransmissionController` API; no in-repo plugin work (the plugin lives outside this repository);
  no stuck-linkage / failed-shift *detection* logic on the ESP32 — this change only reports the two
  values a consumer needs to detect it.

## Decisions

### Why a breaking remap instead of another additive mapping
The additive options were considered and rejected:

- **Leave gear in `engine_load`, put ECU load nowhere** (status quo): permanently forfeits a real
  ECU value and a real MAP value, and keeps measured TPS in the wrong field.
- **Leave gear in `engine_load`, put ECU load in a mismatched free field** (e.g. `fuel_flow`):
  swaps one mis-signposted field for two. `add-ecu-telemetry-pids` already rejected this reasoning
  ("a mis-signposted field is worse than an absent one").
- **Send a second message** (`NAMED_VALUE_FLOAT`, or a second `EFI_STATUS` with a different
  `ecu_index`): `NAMED_VALUE_FLOAT` is exactly what the single-`EFI_STATUS` design was adopted to
  escape (shared message id, name-agnostic stores keep only the last one). A second `EFI_STATUS`
  doubles the 5 Hz packet budget on TELEM2 and needs the same plugin change anyway.

Since any option that unblocks the two values requires touching the plugin, the change is made once
and completely. The cost is a single lockstep deploy; the benefit is that the message becomes
self-describing — a consumer reading the MAVLink field names alone gets the right meaning for
everything except the two deliberately repurposed fuel fields, which are documented in one place.

### Why the fuel fields are the right home for gear
`fuel_consumed` `[cm^3]` and `fuel_flow` `[cm^3/min]` are the only free `EFI_STATUS` fields that are
**permanently** free on this vehicle, and that permanence is *measured*, not assumed: the
`add-can-pid-probe` bench run (2026-08-14, executed twice with identical output) confirmed PID
`0x2F` (fuel level) and `0x5C` (oil temp) are absent from this ECU's supported-PID bitmaps and do
not answer. There is no fuel-quantity or fuel-flow signal on this vehicle to displace, now or later.

The remaining free fields were rejected as gear homes: `spark_dwell_time`, `ignition_timing`,
`injection_time`, `exhaust_gas_temperature`, `barometric_pressure` and `fuel_pressure` all name real
engine quantities this ECU could plausibly expose in a future change (several are in the supported
bitmap already — `0x0E` timing advance is listed as deferred-but-available), so parking gear there
would create exactly the collision this change exists to remove. `ecu_index` is structural, not a
data field.

Using two adjacent fuel fields for the two gear values also keeps the repurposing legible: one
documented exception ("the fuel pair carries gear"), not two scattered ones.

### Assumed vs physical gear, and why physical gets `NaN`
The two values answer different questions and are deliberately not merged:

- **`fuel_consumed` = assumed gear — intent.** The controller's commanded/time-based gear, keeping
  the existing midpoint staircase (`R→L` renders `-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0`). It is
  **always valid**: the controller always knows what it commanded, and this value is independent of
  both CAN health and gear-switch health.
- **`fuel_flow` = physical gear — measurement.** Sourced from
  `VehicleController::getCurrentGearString()`, which is `TransmissionController::getPhysicalGear()`
  (opto switches) and already returns `"?"` for `GEAR_UNKNOWN`. `"?"` encodes to `NaN`.

`NaN` rather than a sentinel number is required here because every plausible sentinel collides with
a real gear or with the midpoint staircase, and because `NaN` is already this message's established
"no data" marker (`rpm`, `cylinder_head_temperature`, and everything else CAN-gated). A consumer
that renders `NaN` as "--" gets the correct behaviour for free.

`getPhysicalGear()` reads UNKNOWN in three distinct situations, and the spec deliberately does not
try to distinguish them on the wire:

1. mid-shift, while the mechanism is between detents (the common case — several hundred ms per
   step, so **`fuel_flow = NaN` during a shift is the expected signature, not a fault**);
2. an ambiguous switch pattern (two switches closed, or none);
3. a faulted input GPIO expander.

Cases 2 and 3 are distinguishable from case 1 only by *duration* and by comparison against
`fuel_consumed`: once `fuel_consumed` settles on an integer and stays there, `fuel_flow` should
re-acquire an integer within the settle window. Persistent `NaN` after settle, or a settled
disagreement between the two, is the interesting condition. That inference belongs to the consumer,
not to a firmware field — the ESP32 would have to invent a timeout policy to encode it, and the
consumer already has both values and a clock.

Reusing `getCurrentGearString()` (rather than adding a physical-gear accessor) is deliberate: it
already exists, is already the physical gear, and already maps UNKNOWN to `"?"`. The only new API
surface is `encodeGear`'s fallback parameter.

### `encodeGear` fallback parameter
`encodeGear()` currently folds both `nullptr` and any unrecognised name to `0.0f` — i.e. NEUTRAL.
That is the correct behaviour for the assumed-gear path (a missing name should not be reported as a
drive gear) but is actively wrong for the physical path, where "I cannot tell" must not read as
"NEUTRAL". Rather than a second encoder function or a caller-side `"?"` test, the fallback becomes a
defaulted parameter:

```
static float encodeGear(const char* gear, float unknownValue = 0.0f);
```

The assumed-gear call sites are unchanged (default preserves today's behaviour exactly, including
for the two `gearFrom`/`gearTo` calls inside the midpoint average); the physical call site passes
`NAN`. `NaN` propagating through the midpoint average is not a concern because the physical path
does not use the average — it encodes a single settled name.

### `throttle_position` vs `throttle_out`, and the `VFR_HUD` ambiguity tell
MAVLink documents `throttle_position` as `[%] Throttle position` and `throttle_out` as
`[%] Output throttle`. Measured ECU TPS is a position; the arbitrated servo command is an output.
The previous mapping had them swapped-by-omission (measured TPS in `throttle_out`, nothing in
`throttle_position`) only because `add-ecu-telemetry-pids` was avoiding duplication of a single
value into two fields. With two genuinely different values now reported, both fields carry their
own meaning and the no-duplication rule is satisfied by construction.

This moves the `VFR_HUD` ambiguity tell. `VFR_HUD.throttle` is a `uint16_t` percent with no `NaN`
encoding, so it keeps its measured-with-commanded-fallback policy unchanged. The marker that tells a
consumer *which* it is in a given tick was `EFI_STATUS.throttle_out == NaN`; it is now
`EFI_STATUS.throttle_position == NaN`, because `throttle_out` is now the commanded value and is
therefore **always** valid. The invariant is preserved exactly: `throttle_position` reads `NaN` in
precisely the ticks where `VFR_HUD.throttle` has fallen back to commanded, and in those ticks
`VFR_HUD.throttle == EFI_STATUS.throttle_out`.

A useful side effect: with measured and commanded throttle now reported side by side in the same
message, a consumer can compare them directly — the "stuck linkage detection would need both values
reported side by side, which is a follow-up" note from `add-ecu-telemetry-pids`'s design is now
satisfied at the transport level. No detection logic is added here.

### `NaN` policy after the remap
Eight of the eleven populated fields are CAN-gated and read `NaN` when `state.canValid` is false:
`rpm`, `cylinder_head_temperature`, `intake_manifold_temperature`, `intake_manifold_pressure`,
`engine_load`, `throttle_position`, `ignition_voltage` — plus `fuel_flow`, which is `NaN` on a
*different* condition (gear-sensor validity) and is the only field whose `NaN` does not track
`canValid`. Three fields are never `NaN`: `fuel_consumed` (assumed gear, always known),
`throttle_out` (commanded, always known locally) and `pt_compensation` (relay ground truth).

This is worth stating explicitly because it is the one place the "all `NaN` at once means CAN is
down" heuristic a consumer might reach for breaks: a lone `fuel_flow = NaN` with everything else
populated means the gear sensor is unsure, and says nothing about CAN.

### Implementation shape
- Three new `StateReport` fields after `throttleCmdPct`: `gearPhysical` (`const char*`, `"R"`/`"N"`/
  `"L"`/`"H"`/`"?"`), `mapKpa` (`uint8_t`, kPa), `engineLoad` (`uint8_t`, %). The struct's
  no-CAN-headers rule is preserved — all three are plain scalars/strings.
- `gearPhysical` is a `const char*` into a `String` owned by the caller, exactly like the existing
  `gearFrom`/`gearTo`. In `main.cpp` this **must** be a named local
  (`String gearPhysStr = vehicleController.getCurrentGearString();`) held until
  `mavlinkInterface.report(report)` returns; taking `.c_str()` on the temporary returned by the
  getter would dangle. The two existing gear strings already establish this pattern immediately
  above, so the risk is one of forgetting, not of ambiguity.
- The `EFI_STATUS` pack call is rewritten one argument per line with a `// field <- source` comment
  each. The call has 19 positional float arguments of identical type, so a mis-ordered argument
  compiles cleanly and fails only on the bench; the positional order is taken from
  `.pio/libdeps/esp32-s3-devkitc-1/c_library_v2/common/mavlink_msg_efi_status.h` (`health`,
  `ecu_index`, `rpm`, `fuel_consumed`, `fuel_flow`, `engine_load`, `throttle_position`,
  `spark_dwell_time`, `barometric_pressure`, `intake_manifold_pressure`,
  `intake_manifold_temperature`, `cylinder_head_temperature`, `ignition_timing`, `injection_time`,
  `exhaust_gas_temperature`, `throttle_out`, `pt_compensation`, `ignition_voltage`,
  `fuel_pressure`). One-per-line is the mitigation: it makes the mapping reviewable against the
  header line by line, and grouped-zeros lines like
  `0.0f, 0.0f, 0.0f, 0.0f, // throttle_position, spark_dwell, baro, intake_press` — which is how
  two of the newly-populated fields hid in the current code — become impossible.
- `uint8_t → float` conversions are written as explicit casts, matching the existing
  `(float)state.engineRpm` style and keeping the build warning-free.

## Risks / Trade-offs
- **Old plugin + new firmware renders ECU load as gear** → the headline risk. `engine_load` is
  `0-100` where a gear is `-1..2`, so it fails loudly (an implausible "gear" that tracks engine
  effort) rather than plausibly. Mitigations: flash and plugin update deploy together; the
  migration table in `MAVLINK_SETUP.md` is written so the plugin can be updated from it alone; and
  the plugin should treat any gear outside `-1..2` as "--", which makes the mismatch self-evident
  in the GCS.
- **New firmware + new plugin, gear field misread as fuel by a *generic* consumer** (MP's MAVLink
  Inspector, mavlink2rest, a log analyser) → a generic tool will label `fuel_consumed`/`fuel_flow`
  by their MAVLink names and show `-1..2` cm³. Accepted: this is inherent to any repurposing, it was
  already true of `engine_load`, and the values are visibly implausible as fuel. Documented in
  `MAVLINK_SETUP.md`.
- **A 19-argument positional pack call is silently mis-orderable** → mitigated by one-argument-per-
  line with field comments, and by the bench matrix which observes every field individually.
- **`gearPhysical` dangling pointer** → the `String` is a named local in `main.cpp` with a lifetime
  spanning the `report()` call, matching the two existing gear strings; called out in tasks.md and
  in design so a future refactor does not "simplify" it into a temporary.
- **`fuel_flow = NaN` read as a fault by an operator** → it is the *normal* mid-shift state.
  Documented in the spec scenario, in the code comment block and in `MAVLINK_SETUP.md`; the bench
  matrix explicitly exercises it.
- **Spec clobbering between active changes** → three changes MODIFY the same requirement; the
  required archive order is stated in proposal.md → `## Sequencing`.

## Migration Plan
1. Flash the firmware and update the QuadBike Mission Planner plugin **together**. There is no
   compatible intermediate state; the two field moves (gear, measured TPS) are simultaneous.
2. Plugin changes, in migration-table form (also in `MAVLINK_SETUP.md`):
   - gear: `engine_load` → `fuel_consumed`
   - measured throttle: `throttle_out` → `throttle_position`
   - new: physical gear ← `fuel_flow` (render `NaN` as "--", fall back to the assumed gear for a
     single gear indicator)
   - new: ECU calculated load ← `engine_load`
   - new: MAP ← `intake_manifold_pressure`
   - new: commanded throttle ← `throttle_out`
   - hardening: render any gear value outside `-1..2` as "--"
3. Rollback is firmware-only and symmetric: revert the pack call and the three `StateReport` fields,
   and re-deploy the old plugin. No NVS keys, no persisted state, no web API or LittleFS change is
   involved, so a rollback needs no filesystem upload.
4. `VFR_HUD` needs no plugin work in either direction — it is a standard HUD field and its behaviour
   is unchanged.

## Open Questions
- Should the ESP32 itself detect a failed shift (assumed settled ≠ physical, or physical `NaN`
  persisting past the settle window) and raise a `STATUSTEXT`? Deliberately out of scope: it needs a
  timeout policy and a debounce that the consumer can apply better with both values in hand. This
  change makes that detection possible for the first time; a follow-up can decide where it lives.
- Should `barometric_pressure` be populated? There is no barometer on this vehicle and no OBD-II PID
  for it on this ECU; it stays zero rather than being fed a constant.
