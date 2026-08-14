## Context
`CANController` (`src/CANController.cpp`) polls the ECU over a non-blocking
`IDLE → WAITING_RESPONSE` state machine with **exactly one OBD-II request in flight at a time**
(`update()` `:158-275`). The active table (`include/CANController.h:157`, `PID_COUNT = 4`) is:

| Index | PID | Meaning | Interval |
|-------|-----|---------|----------|
| 0 | `0x0C` | Engine RPM | `CAN_POLL_INTERVAL_RPM` = 500 ms (50 ms during a gear change) |
| 1 | `0x05` | Coolant temp | `CAN_POLL_INTERVAL_TEMP` = 2000 ms |
| 2 | `0x11` | Throttle position | 2000 ms |
| 3 | `0x0B` | MAP | 2000 ms |

`selectNextPID()` (`:340-355`) picks the most-overdue entry; `CAN_RESPONSE_TIMEOUT` is 200 ms with
`CAN_RETRY_ATTEMPTS` = 3; `CAN_DATA_STALE_TIMEOUT` is 5000 ms. Since `add-can-pid-probe` the RX
path runs behind hardware acceptance filters restricted to `0x7E8-0x7EF` (`applyRxFilters()`
`:454-476`), so the ECU's continuous non-OBD broadcasts (e.g. standard ID `0x301`) never reach the
RX buffers.

The 2026-08-14 bench probe (recorded in `openspec/changes/add-can-pid-probe/design.md`) is the
evidence base for this change: 23 supported PIDs, bitmaps that matched every live answer, and
`0x2F` / `0x5C` confirmed absent. This change enables the subset of confirmed PIDs that is useful
and cheap, and nothing else.

## Goals / Non-Goals
- **Goals**: read `0x42` module voltage, `0x0F` intake air temp and `0x04` engine load at the
  existing 2000 ms interval class; surface them in web telemetry and the CAN card (en + uk);
  map the two that have a natural, currently-unused `EFI_STATUS` field; fill the two hardcoded-zero
  MAVLink throttle fields (`EFI_STATUS.throttle_out`, `VFR_HUD.throttle`) from the already-polled
  TPS (`0x11`), with a commanded-throttle fallback for the HUD; keep RPM at 500 ms.
  (The `0x0D` vehicle-speed question is closed — verified always 0 in motion on 2026-08-14, see
  below — so no in-motion comparison run remains.)
- **Non-Goals**: no new poll-rate classes or scheduler redesign; no enabling of `0x0E`, `0x14`,
  fuel trims or the other bitmap-reported PIDs; **no** `0x0D` in telemetry or control; no change
  to the probe machine; no Mission Planner plugin work (out of repo).

## Decisions

### Which PIDs, and why exactly these three
All three answered on the bench with plausible values and are confirmed by the supported-PID
bitmaps (`0x00 = BE 3E B0 13`, `0x40 = C8 08 00 00`):

| PID | Field | Formula | Bench value | Why it earns a request slot |
|-----|-------|---------|-------------|------------------------------|
| `0x42` | Control module voltage | `((A*256)+B)/1000` V | 12.86 V | The only battery/charging-system health signal the vehicle exposes to us. A remotely-operated quad that loses charging fails silently today. |
| `0x0F` | Intake air temperature | `A - 40` °C | 29 °C | Ambient/engine-bay temperature reference; the only air-side temperature available (there is no oil temp — `0x5C` is absent). |
| `0x04` | Calculated engine load | `A * 100 / 255` % | 0.00 % | The ECU's own load estimate — a better "is the engine working hard" signal than TPS alone, and useful when reviewing gear-change behaviour. |

Deferred despite being supported: `0x0E` timing advance and `0x14` O2 B1S1 (diagnostic
curiosities with no consumer), `0x03`, `0x06`/`0x07` fuel trims, `0x13`, `0x1C`, `0x1F`, `0x21`,
`0x41`, `0x45`, `0x4D`. They stay listed here so a future change does not have to re-probe.
`0x2F` fuel level and `0x5C` oil temp remain confirmed-absent and stay commented out.

Raw-value storage keeps the existing integer-field style of `VehicleData` (no floats on the hot
path): `0x42`'s raw `(A*256)+B` **is already millivolts**, so it is stored verbatim as
`uint16_t moduleVoltageMv` and only divided by 1000 at the JSON/MAVLink edge.

### Scheduler and bus headroom: 4 → 7 PIDs
The scarce resource is not the bus, it is the **single request slot** (one request in flight).

**Request rate.** RPM at 500 ms = 2 requests/s. Slow class at 2000 ms = 0.5 requests/s each.

- Before: 2 + (3 × 0.5) = **3.5 requests/s**
- After: 2 + (6 × 0.5) = **5.0 requests/s**

**Bus load.** A standard 11-bit CAN data frame with 8 data bytes is 111 bits plus up to ~20 stuff
bits plus 3-bit IFS ≈ **134 bits ≈ 0.27 ms** at 500 kbps. Each poll costs a request **and** a
response ≈ 0.54 ms.

- Before: 3.5 × 0.54 ms ≈ 1.9 ms/s = **0.19 %** bus utilisation
- After: 5.0 × 0.54 ms ≈ 2.7 ms/s = **0.27 %** bus utilisation (delta ≈ +0.08 pp)

Even at the 200 ms-timeout worst case the frames themselves are unchanged in size, so bus load is
never the constraint. Headroom is ample by ~2.5 orders of magnitude.

**Request-slot occupancy.** The healthy Delphi MT05 answers in ~50 ms (measured during the probe;
`CAN_RESPONSE_TIMEOUT` = 200 ms is the ceiling, not the norm).

- Before: 3.5 × 50 ms = 175 ms per second = **17.5 % slot duty**
- After: 5.0 × 50 ms = 250 ms per second = **25 % slot duty**, i.e. **75 % of wall time the
  machine is IDLE** and doing nothing but draining RX buffers.

The pathological case (every request times out at 200 ms, 5/s → 1000 ms/s) is the dead-ECU case,
where `dataValid` drops after `CAN_DATA_STALE_TIMEOUT` anyway; and the timeout path reschedules
`nextPollTime = now + 10 + interval` (`:244`), so failed requests do not queue back-to-back.

**Effect on RPM cadence (must stay 500 ms).** RPM keeps its own 500 ms interval; adding slow PIDs
does not change it, only the chance that RPM waits behind one in-flight transaction.

- In any 500 ms RPM window, the expected number of slow-PID requests rises from
  `3 × 500/2000 = 0.75` to `6 × 500/2000 = 1.5`. At ~50 ms each, expected added RPM jitter is
  **~75 ms** (was ~38 ms); absolute worst case per interposed request is one 200 ms timeout.
  Both are far below the 5000 ms stale threshold and below the 500 ms nominal period.
- **Tie-break caveat**: `selectNextPID()` uses `if (overdue >= mostOverdue)`, so on an exact tie
  the **highest index wins** — RPM is index 0 and therefore loses ties. The only realistic exact
  tie is the power-on instant, where all entries have `nextPollTime = 0`: RPM would be served
  after the other six (~300 ms typical, ~1.2 s if all six timed out) — a one-shot startup
  transient, because each serviced entry reschedules to `now + interval` and the cluster
  disperses by one transaction time per entry and never re-forms exactly.
  *Optional hardening (task 2.4, not required for correctness):* stagger the initial
  `nextPollTime` of the three new entries (e.g. +150/+300/+450 ms) so the boot burst is spread
  and RPM is reached sooner. Changing `>=` to `>` would also fix it but alters existing selection
  behaviour, so it is deliberately not proposed.
- **During a gear change** RPM polls at 50 ms, so its overdue time grows 40× faster than a
  2000 ms entry's and it dominates selection almost immediately after any tie. Over a typical
  ~500 ms shift the expected slow-PID interruptions rise from 0.75 to 1.5 — roughly one extra
  ~50 ms gap out of ~10 RPM samples. Bench task 6.3 confirms the boost PID is unaffected.

Conclusion: 7 PIDs is comfortably inside the envelope. The design limit is reached around
~20 requests/s (100 % slot duty at 50 ms/response), which is 4× the new rate.

### MAVLink `EFI_STATUS` mapping
`src/MavlinkInterface.cpp:262-274` already sends one `EFI_STATUS` per 5 Hz tick. Current
occupancy of the 19 fields (`.pio/libdeps/.../mavlink_msg_efi_status.h`):

| Field | Today | This change |
|-------|-------|-------------|
| `health` | 1 (present) | unchanged |
| `rpm` | engine RPM (NaN when CAN invalid) | unchanged |
| `engine_load` | **GEAR** (physical-sequence encoding, 0.5 staircase) | **unchanged — do not touch** |
| `cylinder_head_temperature` | coolant °C (NaN when CAN invalid) | unchanged |
| `pt_compensation` | digital-output bitmask (`EFI_DIGITAL_FLAG_*`) | unchanged |
| `intake_manifold_temperature` | 0.0 (unused) | **← intake air temp `0x0F` (°C)** |
| `ignition_voltage` | 0.0 (unused) | **← control module voltage `0x42` (V)** |
| `intake_manifold_pressure` | 0.0 (unused) | free — natural home for MAP `0x0B`, deferred (see Open Questions) |
| `throttle_out` | 0.0 (unused) | **← measured ECU TPS `0x11` (%), NaN when CAN invalid** |
| `throttle_position` | 0.0 (unused) | free — deliberately left unused (see "Throttle reporting") |
| `ecu_index`, `fuel_consumed`, `fuel_flow`, `spark_dwell_time`, `barometric_pressure`, `ignition_timing`, `injection_time`, `exhaust_gas_temperature`, `fuel_pressure` | 0.0 (unused) | unchanged |

- `intake_manifold_temperature` is documented `[degC]` and `ignition_voltage` is documented `[V]`
  ("supply voltage to EFI sparking system") — both are semantically honest homes for our values,
  and both are currently transmitted as zeros, so nothing is displaced.
- **Engine load gets no MAVLink field.** `EFI_STATUS.engine_load` carries GEAR by an explicit,
  documented contract with the Mission Planner plugin (`src/MavlinkInterface.cpp:243-247`,
  `include/Constants.h:252-256`). Overwriting it would break gear display. Mapping load onto
  `throttle_position` or `throttle_out` instead was rejected: those fields mean throttle, we
  already have a real TPS value (`0x11`) that claims `throttle_out` in this change (see below),
  and a mis-signposted field is worse than an absent one. Engine load stays web-only.
- **Invalid-data convention**: both new fields are sent as `NaN` while `state.canValid` is false,
  matching `rpmVal` / `chtVal` so the plugin renders "--". (MAVLink also documents `0` as
  "unknown" for `ignition_voltage`; `NaN` is used here for consistency with the sibling fields
  and because 0 V is indistinguishable from a genuine dead-rail reading.)
- `MavlinkInterface::StateReport` gains `intakeTemp` (`int8_t`) and `moduleVoltageMv`
  (`uint16_t`), keeping the struct dependency-free of CAN headers as its comment requires.
- The GCS will not show these until the Mission Planner plugin's packet subscription is extended.
  That is additive, out-of-repo work; firmware behaviour is unaffected either way.

### Throttle reporting: measured vs commanded, per message
Both throttle fields we fill are currently hardcoded zeros (`src/MavlinkInterface.cpp:272`
`throttle_out = 0.0f`, `:291` `VFR_HUD.throttle = 0`), and the data to fill them is already on
hand: TPS (`0x11`) has been in the poll table since before this change and lands in
`VehicleData.throttlePosition`, so there is **no extra request-slot cost** — only the two zeros are
new. It also closes a standing spec/code gap: the `mavlink-interface` requirement already states
`EFI_STATUS` carries "throttle position", which the zeroed field did not honour.

Two values exist and they are not the same thing:

- **Measured** — the ECU's own TPS reading. Ground truth about the engine, but only available while
  the CAN link is healthy.
- **Commanded (arbitrated)** — the percent the throttle servo is actually being driven with. Always
  known locally, and it is what the *vehicle* was told to do, not necessarily what the engine did.

The policy differs per message because the two messages promise different things:

- **`EFI_STATUS` = ECU truth, measured only.** It is the engine-data message and every other field
  in it is a measurement, with `NaN` as the established "no data" marker (`rpm`,
  `cylinder_head_temperature`, and the two fields added above). Putting a commanded value into it
  would silently change the meaning of an ECU field. So `throttle_out` carries measured TPS when
  `state.canValid`, and `NaN` otherwise — a plugin renders "--".
- **`VFR_HUD` = pilot-facing HUD, must always say something useful.** `throttle` is a `uint16_t`
  percent: **there is no NaN and no "unknown" encoding**, so the only alternatives to a fallback are
  a frozen 0 or suppressing the whole message (which would also take ground speed down with it —
  ground speed is governed by the hall sensor's validity, deliberately independent of CAN). A HUD
  throttle bar reading 0 % while the operator is holding throttle is misleading in the more
  dangerous direction than a bar showing intent. So: measured TPS when CAN is valid, commanded
  percent when it is not.

The substitution is not ambiguous to a consumer, because the two messages are sent in the same tick:
`EFI_STATUS.throttle_out` reads `NaN` **exactly when** `VFR_HUD.throttle` has fallen back to the
commanded value. When CAN is valid both carry the same measured number, so agreement is the normal
state and a sustained divergence is not possible by construction (we do not attempt stuck-linkage
detection here — that would need both values reported side by side, which is a follow-up).

**Sourcing the commanded value.** `MavlinkInterface::StateReport` (`include/MavlinkInterface.h:46`)
carries neither throttle value today, so it gains `throttlePosition` (`uint8_t`, %, straight from
`VehicleData`) and `throttleCmdPct` (`uint8_t`, 0-100), keeping the struct's no-CAN-headers rule.
The commanded field must **not** be taken from `MavlinkInterface::getThrottle()` (the autopilot's
raw demand): `VehicleController::update()` (`src/VehicleController.cpp:356-362`) applies a
speed-limit cap, the gear-boost PID overrides throttle entirely during a shift
(`:159`, `:990`), the web `set_throttle` path is a separate source, and fail-safe forces idle. The
one place all of that has already been arbitrated is the servo's live pulse width, so the value is
derived from `ThrottleController::getCurrentUs()` mapped back through the active calibration
window — i.e. a new `usToPercent()` inverse of `percentToUs()` (`src/ThrottleController.cpp:31-35`;
clamp to 0-100, guard `fullUs_ == idleUs_`), exposed as `VehicleController::getThrottlePercent()`
and copied into the snapshot in `main.cpp` next to the other `report.*` assignments. Rounding to an
integer percent at the edge is fine — the field is a percent bar, not a control signal.

### Web telemetry and JSON budget
The three fields are emitted inside the existing `if (telemetry.can_status == "connected")` block
in `createTelemetryJSON` (`src/WebPortal.cpp:614-621`), next to `map_kpa`, so they are omitted
wholesale when CAN is down — the same contract the CAN card already relies on.

- Serialized cost: `"ecu_voltage":12.86,` (21 B) + `"intake_temp":29,` (17 B) +
  `"engine_load":0,` (16 B) ≈ **~54 bytes** on the wire.
- Document cost: 3 additional ArduinoJson v6 key/value slots ≈ **~48 bytes** of the
  `StaticJsonDocument<4096>` (`:578`). The capacity peak is the transient `probe` object, not
  steady state, so 4096 is expected to remain sufficient — but it is **measured**, not assumed
  (task 5.1: `doc.memoryUsage()` with the probe object present, and `json.length()` in steady
  state).
- `ecu_voltage` is emitted via `serialized(String(v, 2))` — the idiom already used for
  `vehicle_speed` / `rail_24v` — to get 2 decimals without float formatting surprises.
- **Flagged**: `web-telemetry` → *Telemetry Performance* still specifies "JSON message size
  remains under 1KB". With the Control_v0 rail fields, six hall-speed fields and four steering
  VESC fields already present, steady-state payload may already be near or past that figure.
  Task 5.1 measures it. If it is already over, that is pre-existing drift and MUST be reported
  as a finding — this change adds ~54 B and does not amend that requirement.

#### Measured budget (static, 2026-08-14 — runtime confirmation still pending)

Measured by rebuilding the exact document shape against the **same ArduinoJson 6.21.6 headers the
firmware links**, replicating which values are copied into the string pool (`String`,
`serialized(String)` and `char*` buffers) versus stored by pointer (string literals). The host has
64-bit pointers (32 B `VariantSlot`), so the ESP32 figure is derived as `slots × 16 + stringPool`;
the slot count is architecture-independent and cross-checks exactly against a hand count (159).

| Case | Slots | Doc bytes (ESP32) | % of 4096 | Serialized |
|------|-------|-------------------|-----------|------------|
| Steady state, before | 72 | 1203 | 29.4 % | ~1058 B |
| Steady state, after | 75 | 1257 | 30.7 % | ~1111 B |
| Probe present, before | 156 | 2580 | 63.0 % | ~1971 B |
| **Probe present, after (peak)** | **159** | **2634** | **64.3 %** | ~2024 B |

`StaticJsonDocument<4096>` is **sufficient — capacity NOT bumped.** The peak (transient `probe`
object present, all 4 bitmap groups answered, all 9 candidate PIDs decoded, 2 DTCs) leaves ~1462
bytes / 36 % headroom. Document growth from this change is +54 B, wire growth +53 B, both matching
the ~54 B estimate above.

**Pre-existing finding (task 5.2): the steady-state payload was already over the "under 1KB"
figure before this change** — ~1058 B, i.e. the requirement had already drifted through the
Control_v0 rail, hall-speed and steering-VESC additions. This change takes it to ~1111 B. Per
task 5.2 this is reported, **not** silently amended; the *Telemetry Performance* requirement is
left untouched and needs its own change. (Serialized lengths are modelled with representative
string values — `firmware_version`, `input_source` etc. — so they carry ±tens of bytes of
uncertainty; the >1 KB conclusion is robust because the model uses short placeholders and so
under-states the real payload.)

A high-water-mark log in `createTelemetryJSON()` reports `memoryUsage()` / `capacity()` /
`json.length()` on each new peak (not every 5 Hz broadcast), so the bench run confirms these
numbers on real hardware for free. Tasks 5.1/5.2 stay open until that runtime observation exists.

### `0x0D` vehicle speed: verified dead, nothing to do
The bench probe answered `0x0D` = 0 km/h stationary, which proved the PID exists but not that a
speed source reaches the ECU. **Resolved on the bench 2026-08-14 (user-verified): `0x0D` reports
0 at all times, including with the wheel in motion.** The ECU answers the request but has no
vehicle-speed sensor input on this vehicle — the speedo is dash-wired — so the value is
permanently 0 and useless.

Task 8.4 is therefore **already satisfied**: the outcome is "always 0", no side-by-side
comparison against the hall sensor is needed, and there is nothing to wire. This change **does
not** add `0x0D` to `pidTable_`, telemetry, MAVLink or any control path, and no future change
should either. `add-hall-speed-sensor` (GPIO8/X2) is the only speed source on this vehicle — a
directly-owned, independently-validated signal feeding the gear-change interlock and `VFR_HUD`
— and is in no sense redundant.

## Risks / Trade-offs
- **RPM jitter from a busier scheduler** → quantified above (~75 ms expected, one 200 ms
  worst-case interposition); mitigated by RPM's own 500 ms/50 ms interval and verified at the
  bench (task 6.3). The boot-time tie-break burst is a one-shot transient with an optional
  stagger mitigation.
- **`0x42` behaviour under load is unverified** (bench run was engine-off, 12.86 V) → task 6.2
  checks it with the engine running and a load applied (headlight/starter), where a charging
  system should read ~13.5-14.5 V.
- **MAP vacuum still unproven** (inherited open item `add-can-pid-probe` task 8.3) → folded into
  the same engine-running bench check (task 6.2) since the operator is already there.
- **GCS shows nothing until the MP plugin is updated** → documented; no firmware risk.
- **Spec clobbering between active changes** → the two MODIFIED blocks are written on top of the
  active changes' versions and the required archive order is stated in proposal.md.

## Migration Plan
Purely additive: three new poll-table entries, three new `VehicleData` fields, three new JSON
fields, four previously-zero MAVLink fields (intake temp, module voltage, `EFI_STATUS.throttle_out`,
`VFR_HUD.throttle`), one new read-only accessor pair for commanded throttle, three new UI readouts.
No NVS keys, no removed API, no wire-format break — an older web client simply ignores the new JSON
keys, and a GCS that ignored the four zeroed MAVLink fields keeps working. Rollback is reverting
`PID_COUNT` to 4, dropping the three table entries and restoring the four zero literals; nothing
else depends on the new fields, and the new throttle accessor is read-only (no control path
touches it).

## Open Questions
- ~~Does `0x0D` report real speed while moving, or always 0? (Task 8.4, report-only. Blocks any
  future adoption; does not block this change.)~~ **Resolved 2026-08-14** (bench, user-verified):
  **always 0** in motion — the ECU answers the PID but has no speed input on this vehicle. `0x0D`
  is dead; no future adoption is possible and task 8.4 needs no bench run.
- Should MAP (`0x0B`) also be mapped to the free `EFI_STATUS.intake_manifold_pressure`? Natural
  and nearly free, but MAP belongs to `add-can-pid-probe`'s scope, which states no `EFI_STATUS`
  consumer change is required. Deferred to a follow-up so this change's MAVLink delta stays
  limited to PIDs it introduces.
- Should engine load ever reach the GCS? Only by moving GEAR off `EFI_STATUS.engine_load`, which
  breaks the documented MP plugin contract. Not worth it unless the plugin is being reworked
  anyway.
