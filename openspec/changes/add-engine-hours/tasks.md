## 1. Constants (`include/Constants.h`)
- [x] 1.1 Add a new `ENGINE HOUR METER` block (after the cranking parameters, beside
  `ENGINE_RUNNING_RPM_THRESHOLD`) holding `ENGINE_HOURS_MIN_RPM` (`300`), commented as: an hour
  meter MUST count idling, anything above ~300 RPM means the crank is turning under its own power
  or under the starter, and this is deliberately **not** `ENGINE_RUNNING_RPM_THRESHOLD` (1500),
  which is a gear-change/crank safety threshold that sits above idle.
- [x] 1.2 Same block: `ENGINE_HOURS_MAX_DELTA_MS` (`5000`), commented as the single-update cap —
  a delta longer than this is a stalled loop or a clock anomaly, not running time, and is discarded
  whole.
- [x] 1.3 Same block: `ENGINE_HOURS_NVS_WRITE_INTERVAL_S` (`600ULL` = 10 min of **accumulated**
  growth) with a comment quantifying the write budget in the style of `ODO_NVS_WRITE_INTERVAL_MM`:
  6 writes per engine-hour plus one per ignition-OFF; NVS is wear-levelled and log-structured, so
  ~126 writes of the one `uint64` key fill a 4096-byte page and cost one erase — ≈480 erases over
  10 000 engine hours, ≈0.48 % of the flash's ~100 000-cycle endurance.
- [x] 1.4 **No new config key, no NVS-tunable value, no web-settable parameter** — grep the diff to
  confirm nothing in `Constants.h` gained a runtime setter, and that
  `ENGINE_RUNNING_RPM_THRESHOLD` is unchanged.

## 2. New class `EngineHourMeter`
- [x] 2.1 `include/EngineHourMeter.h`: new header with a Doxygen class block stating that it owns
  the NVS namespace `"engine"` / key `"hours_s"`, that the counter only ever increases and has no
  reset path, that time accrues only while the engine is turning under valid CAN, and that hours
  are a presentation of the exact second counter.
- [x] 2.2 Public API, each with a Doxygen comment: `void begin()` (NVS load + boot log),
  `void update(bool engineTurning, uint32_t nowMs)`, `uint64_t getEngineSeconds() const`,
  `float getEngineHours() const` (`seconds / 3600.0f`), `void persist()`.
- [x] 2.3 Private state: `uint64_t engineSeconds_`, `uint32_t remainderMs_`, `uint32_t
  lastUpdateMs_`, `uint64_t lastWriteSeconds_`, `bool started_`, `bool dirty_` — all initialised in
  the constructor initialiser list.
- [x] 2.4 `src/EngineHourMeter.cpp`, `begin()`: open `"engine"` read-only,
  `getULong64("hours_s", 0)` (a missing key reads 0 — the correct starting meter for a virgin NVS,
  there is no seeding path), seed `lastWriteSeconds_`, clear `dirty_`, and log the restored value
  on `DebugFeature::VEHICLE`: `[ENGINE] hour meter 123.45 h restored`. A failed
  `Preferences::begin()` logs that nothing was restored rather than claiming a restored 0.00.
- [x] 2.5 `update()`: on the first call only, latch `lastUpdateMs_ = nowMs` and return (no delta
  exists yet). Otherwise compute `dtMs = nowMs - lastUpdateMs_` (unsigned, so the `millis()` wrap
  is handled by modular arithmetic), advance `lastUpdateMs_` **unconditionally**, then return
  without accumulating if `!engineTurning` (the interval is DISCARDED, never banked) or if
  `dtMs > ENGINE_HOURS_MAX_DELTA_MS`.
- [x] 2.6 `update()`: accumulate `remainderMs_ += dtMs`, carry whole seconds into `engineSeconds_`,
  keep `remainderMs_ %= 1000`, set `dirty_`. Then, if
  `engineSeconds_ - lastWriteSeconds_ >= ENGINE_HOURS_NVS_WRITE_INTERVAL_S`, call `persist()`. The
  comparison is unsigned and monotonic — no wrap handling needed.
- [x] 2.7 `persist()`: return immediately when `!dirty_` (NVS already holds this value — no open,
  no log, no wear), mirroring `SpeedSensor::persistDistance()`. Otherwise open `"engine"`
  read-write, `putULong64("hours_s", engineSeconds_)`, `end()`, advance `lastWriteSeconds_`, clear
  `dirty_` only on a non-zero write result.
- [x] 2.8 `persist()`: tolerate a failed `Preferences::begin()` by logging and returning — never
  block, never retry in a loop. Advance `lastWriteSeconds_` on the failure path too, so the
  interval trigger cannot re-fire (and re-log) every loop iteration while the engine keeps running;
  `dirty_` stays set so the next interval or the next ignition-OFF retries.
- [x] 2.9 Confirm by inspection that no code path anywhere can decrease or zero `engineSeconds_`,
  and that the class exposes no setter, no reset and no seed.

## 3. VehicleController: own the meter, update it, flush it
- [x] 3.1 `include/VehicleController.h`: `#include "EngineHourMeter.h"`, a private
  `EngineHourMeter engineHours_;` member (a VALUE, not a reference — the controller owns it), and a
  public `void initEngineHourMeter();`.
- [x] 3.2 `include/VehicleController.h`: read-only accessors `getEngineSeconds()` and
  `getEngineHours()` beside `getOdoKm()` / `getTripKm()`, with a Doxygen note that the meter is
  always valid, only ever increases, and has no reset path on any interface.
- [x] 3.3 `src/VehicleController.cpp`: implement `initEngineHourMeter()` as `engineHours_.begin();`
  and call it from `setup()` in `src/main.cpp`, beside `vehicleController.initCAN()`. The NVS load
  cannot live in the constructor — the controller is a global constructed before `setup()` runs.
- [x] 3.4 `src/VehicleController.cpp`, `update()`: immediately after the existing
  `CANController::VehicleData canData = canController_.getVehicleData();` (line ~144), call
  `engineHours_.update(canData.dataValid && canData.engineRPM >= ENGINE_HOURS_MIN_RPM, millis());`
  with a comment that an invalid CAN reading is an UNKNOWN engine state and must not invent hours.
  Reuse that same snapshot — do not add a second `getVehicleData()` call.
- [x] 3.5 `processMavlinkCommands()`, the `case IgnitionState::OFF` transition guard (line ~437):
  add `engineHours_.persist();` beside the existing `speedSensor_.persistDistance();`.
- [x] 3.6 `setIgnitionState()`, the web ignition-OFF transition (line ~806): same pairing.
- [x] 3.7 `applyFailsafe()`, the fail-safe entry branch (line ~383): same pairing.
- [x] 3.8 Confirm `isEngineRunning()` and `ENGINE_RUNNING_RPM_THRESHOLD` are untouched, and that
  the meter has no influence on any interlock, actuator or arbitration path.

## 4. MavlinkInterface: report engine hours in EFI_STATUS
- [x] 4.1 `include/MavlinkInterface.h`: add `float engineHours;` to `StateReport`, beside `odoKm` /
  `tripKm`, documented as always valid (never `NaN`, independent of CAN health — it stops growing
  rather than going unknown). The header stays free of mavlink headers.
- [x] 4.2 `src/MavlinkInterface.cpp`, the `mavlink_msg_efi_status_pack()` call: replace the
  `0.0f, // spark_dwell_time (unused)` argument with `state.engineHours` and update the field
  comment to `<- ENGINE HOURS (h, total)`, with a short note that spark dwell has no standard
  OBD-II Mode 01 PID, which is what makes the field permanently free.
- [x] 4.3 `src/main.cpp`: populate `StateReport::engineHours` from the new
  `VehicleController::getEngineHours()` accessor, beside the existing `odoKm` / `tripKm` lines.
- [x] 4.4 Confirm no other `EFI_STATUS` field moved, that the fields still reserved send literal
  `0.0f`, and that the CAN-invalid path does not `NaN` the new field. (`injection_time` was one of
  the three at this point; section 9 below takes it for the TRIP hours, leaving `ignition_timing`
  and `exhaust_gas_temperature`.)

## 5. Telemetry and web UI
- [x] 5.1 `include/WebPortal.h`: add `float engine_hours;` to the telemetry struct, commented as
  always valid and NOT gated on `can_status`.
- [x] 5.2 `src/TelemetryManager.cpp`: populate it from `vehicleController_.getEngineHours()`.
- [x] 5.3 `src/WebPortal.cpp`: serialise `doc["engine_hours"]` with
  `serialized(String(value, 2))`, unconditionally — outside the `can_status == "connected"` block,
  with the same `isfinite()` guard the distance counters use (a bare `nan` token would break the
  whole document for the browser's `JSON.parse`).
- [x] 5.4 `data/index.html`: one read-only `telemetry-item` in the Vehicle Data (CAN) card next to
  Engine RPM, `id="can-engine-hours"`, `data-i18n="lbl_engine_hours"`; **no button, no input, no
  reset control**.
- [x] 5.5 `data/index.html`, `updateTelemetry()`: render it **outside** the
  `can_status === 'connected'` branch (and outside its `else`), as
  `value.toFixed(2) + ' ' + t('unit_h')`, defaulting to `--` when the field is absent or
  non-finite. Accumulated running time is not invalidated by the bus going quiet, so it must not be
  blanked with the live CAN values.
- [x] 5.6 `data/index.html`: add `lbl_engine_hours` and `unit_h` to **both** the `en` and `uk`
  dictionaries (Engine hours / h — Мотогодини / год) and verify EN/UK key parity.
- [x] 5.7 `MAVLINK_SETUP.md`: change row 7 of the 19-field table from `spark_dwell_time` →
  "unused, reserved" to **ENGINE HOURS, total, h**, always valid, source `state.engineHours`, with
  the ⚑ repurposing marker; amend the "**four** reserved fields" paragraph to **three**
  (`ignition_timing`, `injection_time`, `exhaust_gas_temperature`) and state why dwell is the one
  that could leave the set (no standard Mode 01 PID, unlike PID `0x0E` timing, PID `0x78` EGT and
  injection time from the fuel trims).
- [x] 5.8 `MAVLINK_SETUP.md`: add an **Engine hours** subsection next to "Odometer and trip
  distance", stating the counting rule (valid CAN **and** RPM ≥ `ENGINE_HOURS_MIN_RPM`, so idling
  counts and ignition-on-engine-off does not), that it stops growing rather than going `NaN` during
  a CAN outage, and that **it cannot be reset on any interface** — only an NVS erase.
- [x] 5.9 `MAVLINK_SETUP.md`: add `EFI_STATUS.spark_dwell_time` to the "Fields that are ALWAYS
  valid (never `NaN`)" table, and add a short **ground-station plugin hand-off note** under
  "Ground-station plugin compatibility": **non-breaking**, one new read-only field, hours, render
  to 2 decimals, never `NaN`, and it degrades to `0.00` against legacy firmware rather than to a
  wrong number.
- [x] 5.10 `WEB_PORTAL_SETUP.md`: one sentence that the portal **displays** engine hours
  (`engine_hours`, 2 decimals, shown whatever the CAN status) and offers no reset, because the
  meter has no reset on any interface.

## 6. Build and static checks
- [x] 6.1 `pio run -e esp32-s3-devkitc-1` completes with no errors and no new warnings from `src/`
  or `include/`.
- [x] 6.2 `rg -n 'engineSeconds_' src include` shows assignment only in `begin()` (the NVS load)
  and the accumulate path in `update()` — never a decrement, never a zero.
- [x] 6.3 `rg -n 'persist\(' src include` shows exactly the three flush sites
  (ignition-OFF on the MAVLink path, ignition-OFF on the web path, fail-safe entry) plus the
  interval write inside `EngineHourMeter::update()`.
- [x] 6.4 `rg -n 'engine_hours|lbl_engine_hours|unit_h' data/index.html` shows the row, the render
  and BOTH dictionaries; no reset control anywhere.
- [x] 6.5 `[WEB] telemetry JSON peak:` stays below 4096 with no overflow warning (one extra key,
  ~20 bytes). *Verified by inspection only (`StaticJsonDocument<4096>`, +~22 B); the runtime peak
  is confirmed on the bench.*

## 7. Bench and road verification
- [ ] 7.1 **[OPERATOR]** Flash firmware **and** `pio run -t uploadfs` (`data/` changed).
- [ ] 7.2 **[OPERATOR]** First boot on a virgin NVS: serial shows `[ENGINE] hour meter 0.00 h
  restored` (or the "nothing restored" line), the portal shows `0.00 h`, and Mission Planner shows
  `spark_dwell_time = 0`.
- [ ] 7.3 **[OPERATOR]** **Read this engine's actual idle RPM** off the portal with the engine
  warm and idling, and record it in the commissioning notes. It is expected to be 800–1500 RPM,
  comfortably above the 300 RPM `ENGINE_HOURS_MIN_RPM` floor. **If it idles below ~400 RPM, raise
  the question before the meter accumulates anything worth keeping** — the floor is reasoned, not
  measured.
- [ ] 7.4 **[OPERATOR]** **Ignition ON, engine OFF, does not count.** Sit with the ignition on and
  the engine stopped for 10 minutes: CAN is valid, `rpm` reads 0, and the hour meter does **not**
  move in the portal or in `spark_dwell_time`.
- [ ] 7.5 **[OPERATOR]** **Idling counts.** Start the engine and let it idle for a measured
  30 minutes without moving the vehicle: the meter increases by 0.50 h ± one reporting tick, and
  the odometer is unchanged — the case that justifies the whole feature.
- [ ] 7.6 **[OPERATOR]** **The wire agrees with the portal.** `EFI_STATUS.spark_dwell_time` and the
  portal's Engine hours row show the same value to 2 decimals throughout 7.5.
- [ ] 7.7 **[OPERATOR]** **NVS survives a normal shutdown.** With a non-zero meter, switch ignition
  OFF, pull power, re-power: the meter comes back at exactly its pre-shutdown value.
- [ ] 7.8 **[OPERATOR]** **The interval write works without an ignition cycle.** Run the engine for
  ≥ 20 minutes, then pull power **without** switching ignition OFF: at most the last 10 minutes are
  lost and the meter is non-zero.
- [ ] 7.9 **[OPERATOR]** **A CAN outage stops the meter and does not pay it back.** With the engine
  running, unplug the CAN lead for 5 minutes: `rpm` and its six CAN-gated siblings go `NaN`, the
  hour meter **stops** (it does not go `NaN`), and on reconnect it resumes from where it stopped —
  the 5 minutes are **not** credited.
- [ ] 7.10 **[OPERATOR]** **The meter never decreases.** Across all of the above, and across a
  fail-safe entry (cut the MAVLink link with the engine running), the value reported in
  `spark_dwell_time` is monotonically non-decreasing.
- [ ] 7.11 **[OPERATOR]** **The TOTAL has no reset.** Confirm the portal shows the meter as plain
  text with no control, and that `MAV_CMD_USER_1` with the trip-reset magic leaves the TOTAL hour
  meter untouched. (It now also clears the TRIP hours — see 12.2.)
- [ ] 7.12 **[OPERATOR]** **Legacy-firmware sanity.** With the previous firmware flashed,
  `EFI_STATUS.spark_dwell_time` reads 0 — confirming a plugin that shows the new field degrades to
  `0.00 h`, not to a wrong number, against an un-updated vehicle.
- [ ] 7.13 **[OPERATOR]** Record the engine hour reading alongside the odometer reading in the
  commissioning notes, and note that both start from this flash rather than from the engine's real
  history.

## 8. Trip engine hours ("мотогодини місії")
- [x] 8.1 `include/EngineHourMeter.h`: add `uint64_t tripSeconds_` beside `engineSeconds_` in the
  private state and in the constructor initialiser list, and update the class Doxygen block — the
  meter now HAS a resettable twin, the total still has no reset on any interface.
- [x] 8.2 `include/EngineHourMeter.h`: public `uint64_t getTripSeconds() const` and
  `float getTripHours() const` (`seconds / 3600.0`), each with a Doxygen comment saying the float is
  a presentation of the counter, never read back.
- [x] 8.3 `src/EngineHourMeter.cpp`, `update()`: hoist the whole-second carry into one
  `const uint64_t wholeSeconds = remainderMs_ / 1000;` and add it to BOTH counters, from the SINGLE
  shared `remainderMs_`. Comment why a second remainder would let the two drift apart by rounding
  the same elapsed time twice. Same counting rule, same `ENGINE_HOURS_MAX_DELTA_MS` cap, same
  `millis()` deltas — no second gate and no second clock latch.
- [x] 8.4 `include/EngineHourMeter.h` / `src/EngineHourMeter.cpp`: new NVS key constant
  `NVS_KEY_TRIP` = `"trip_s"` in the SAME `"engine"` namespace (well inside the 15-character cap).
- [x] 8.5 `begin()`: load it with `getULong64("trip_s", 0)` — a key absent on a vehicle upgrading
  from the total-only build reads 0, which is the correct starting point for a never-reset trip —
  and extend the boot log to `[ENGINE] hour meter %.2f h, trip %.2f h restored`.
- [x] 8.6 `persist()`: write `trip_s` alongside `hours_s` in the same open, under the same dirty
  flag, on the same three triggers — **no new trigger**. Treat the write as successful (clear
  `dirty_`) only when BOTH `putULong64` calls returned non-zero, so a half-written pair is retried.
- [x] 8.7 `src/EngineHourMeter.cpp`: new `void resetTrip()` — zero `tripSeconds_`, set `dirty_`,
  call `persist()` immediately (the operator's gesture must survive a power cut, as
  `SpeedSensor::resetTrip()` does), and log
  `[ENGINE] trip hours reset to 0 (total %.2f h unchanged)`. It MUST NOT name or touch
  `engineSeconds_`.
- [x] 8.8 `include/VehicleController.h`: read-only `getEngineTripSeconds()` / `getEngineTripHours()`
  beside the existing pair, and amend the Doxygen note — the TOTAL has no reset, the TRIP is zeroed
  only by the SAME latched trip reset that clears the trip DISTANCE.
- [x] 8.9 `src/VehicleController.cpp`, `update()`: inside the EXISTING
  `if (mavlink_.consumeTripResetRequest())` block, beside `speedSensor_.resetTrip()`, call
  `engineHours_.resetTrip()`, with a comment that the two trip readings describe one interval in two
  units and so are cleared by one gesture. **No new MAVLink command, no new `param1` magic, no web
  control, no new constant**, and `handleCommandLong()` is not touched.
- [x] 8.10 Confirm by inspection that `EngineHourMeter::resetTrip()` has EXACTLY ONE call site and
  that nothing outside `EngineHourMeter` can reach `tripSeconds_`.

## 9. Trip engine hours on the wire and in the portal
- [x] 9.1 `include/MavlinkInterface.h`: add `float engineTripHours;` to `StateReport` beside
  `engineHours`, documented as always valid, never `NaN`, and `0` a GENUINE zero after a reset. The
  header stays free of mavlink headers.
- [x] 9.2 `src/MavlinkInterface.cpp`, the `mavlink_msg_efi_status_pack()` call: replace the
  `0.0f, // injection_time (unused)` argument with `state.engineTripHours` and a field comment in
  the style of the `spark_dwell_time` one — injection time has **no PID of its own**, being only
  derivable from the fuel trims plus load, unlike `ignition_timing` (PID `0x0E`) and
  `exhaust_gas_temperature` (PID `0x78`).
- [x] 9.3 `src/MavlinkInterface.cpp`: amend the block comment above the pack site — the reserved set
  is now TWO fields, and both `spark_dwell_time` and `injection_time` carry hour meters.
- [x] 9.4 `src/main.cpp`: populate `StateReport::engineTripHours` from
  `VehicleController::getEngineTripHours()`, beside the existing `engineHours` line.
- [x] 9.5 `include/WebPortal.h`: `float engine_trip_hours;` in the telemetry struct, commented as
  always valid and NOT gated on `can_status`.
- [x] 9.6 `src/TelemetryManager.cpp`: populate it from `vehicleController_.getEngineTripHours()`.
- [x] 9.7 `src/WebPortal.cpp`: `doc["engine_trip_hours"]` with the same
  `serialized(String(isfinite(...) ? value : 0.0f, 2))` guard, unconditionally, beside
  `engine_hours`.
- [x] 9.8 `data/index.html`: a read-only `telemetry-item` with `id="can-engine-trip-hours"` and
  `data-i18n="lbl_engine_trip_hours"`, directly under the `can-engine-hours` row; **no button, no
  input, no reset control**.
- [x] 9.9 `data/index.html`, `updateTelemetry()`: render it OUTSIDE the `can_status === 'connected'`
  branch (and outside its `else`), `toFixed(2) + ' ' + t('unit_h')`, `--` when the field is absent
  or non-finite — the placeholder reserved for an absent field, so a shown `0.00` is a genuine zero.
- [x] 9.10 `data/index.html`: `lbl_engine_trip_hours` in BOTH dictionaries (en `Trip hours`, uk
  `Мотогодини місії`); verify EN/UK key parity programmatically, not by eye.

## 10. Trip engine hours in the documentation
- [x] 10.1 `MAVLINK_SETUP.md`: change row 13 of the 19-field table from `injection_time` →
  "unused, reserved" to **TRIP ENGINE HOURS, resettable, h**, always valid, `0` a genuine zero,
  source `state.engineTripHours`, with the ⚑ repurposing marker.
- [x] 10.2 `MAVLINK_SETUP.md`: the reserved-fields paragraph goes from **three** to **two**
  (`ignition_timing`, `exhaust_gas_temperature`), stating why both fields that left have no direct
  Mode 01 PID while the two that remain do (`0x0E`, `0x78`).
- [x] 10.3 `MAVLINK_SETUP.md`, the "Engine hours" subsection: two counters, the lockstep rule, the
  one shared `persist()`, the trip's field, and that the trip reset is shared with the TRIP km.
- [x] 10.4 `MAVLINK_SETUP.md`, the "Odometer and trip distance" and "Trip reset" subsections: one
  command zeroes BOTH trip readings, no second command and no second magic value, ODO and total
  hours untouched.
- [x] 10.5 `MAVLINK_SETUP.md`, the `COMMAND_ACK` results table: the ACCEPTED row now zeroes TRIP km
  **and** trip hours, observable on the wire as `fuel_pressure` AND `injection_time` both falling to
  zero while `barometric_pressure` and `spark_dwell_time` are unchanged.
- [x] 10.6 `MAVLINK_SETUP.md`: add the trip field to the "Fields that are ALWAYS valid (never
  `NaN`)" table, and extend the ground-station plugin hand-off note — read-only, hours, 2 decimals,
  degrades to `0.00` on legacy firmware, and **the existing trip-reset command needs no change**.
- [x] 10.7 `WEB_PORTAL_SETUP.md`: one sentence that the portal displays the trip hours read-only
  beneath the total, and that clearing them is a ground-station action.

## 11. Trip engine hours — build and static checks
- [x] 11.1 `pio run -e esp32-s3-devkitc-1` completes with no errors and no new warnings from `src/`
  or `include/`.
- [x] 11.2 `rg -n 'tripSeconds_' src include` shows assignment ONLY in the constructor initialiser
  list, the `begin()` NVS load, the shared accumulate in `update()`, and `resetTrip()`.
- [x] 11.3 `rg -n 'engineSeconds_' src include` shows no decrement and no zero anywhere — the trip
  reset must be provably unable to touch the total.
- [x] 11.4 `rg -n 'resetTrip\(' src include` shows `EngineHourMeter::resetTrip` with EXACTLY ONE
  call site, in `VehicleController::update()`, inside the same `if` as `speedSensor_.resetTrip()`.
- [x] 11.5 `rg -n 'engine_trip_hours|lbl_engine_trip_hours' data/index.html` shows the row, the
  render and BOTH dictionaries; no reset control anywhere.

## 12. Trip engine hours — bench verification
- [ ] 12.1 **[OPERATOR]** **Lockstep.** With the engine idling, the portal's Trip hours and Engine
  hours advance by the same amount over a measured 30 minutes (both +0.50 h), and
  `injection_time - spark_dwell_time` stays constant on the wire.
- [ ] 12.2 **[OPERATOR]** **One command, both trips.** Send `MAV_CMD_USER_1` with `param1 = 1` to
  1/25: in the NEXT `EFI_STATUS`, `fuel_pressure` **and** `injection_time` are both `0`, while
  `barometric_pressure` and `spark_dwell_time` are **unchanged**. `MAV_RESULT_ACCEPTED` is returned
  and the serial log carries both the `[MAV] TRIP reset accepted` line and
  `[ENGINE] trip hours reset to 0 (total ... h unchanged)`.
- [ ] 12.3 **[OPERATOR]** **The portal agrees.** Immediately after 12.2 the portal's Trip hours row
  reads `0.00 h` and the Engine hours row is unchanged; neither row has a control beside it.
- [ ] 12.4 **[OPERATOR]** **The trip survives a power cycle.** Accumulate a non-zero trip, switch
  ignition OFF, pull power, re-power: the boot log reports both values
  (`[ENGINE] hour meter X h, trip Y h restored`) and the trip comes back at its pre-shutdown value —
  a power cycle is NOT a reset.
- [ ] 12.5 **[OPERATOR]** **An ignition cycle is not a reset either.** Ignition OFF and back ON with
  power maintained: the trip hours are unchanged.
- [ ] 12.6 **[OPERATOR]** **Legacy-firmware sanity.** With the previous firmware flashed,
  `EFI_STATUS.injection_time` reads 0 — confirming a plugin that shows the trip hours degrades to
  `0.00 h`, not to a wrong number, against an un-updated vehicle.
