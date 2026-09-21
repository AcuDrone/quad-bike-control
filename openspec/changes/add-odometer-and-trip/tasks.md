## 1. Constants (`include/Constants.h`)
- [ ] 1.1 In the `VEHICLE SPEED SENSOR` block, after `SPEED_MAX_PULSES_PER_SAMPLE`, add
  `ODO_NVS_WRITE_INTERVAL_MM` (`1000000ULL` = 1 km) with a comment stating the write budget:
  one NVS write per kilometre plus one per ignition-OFF, wear-levelled, ≈0.16 % of flash endurance
  over 10 000 km.
- [ ] 1.2 In the `MAVLINK_PARAM_*` neighbourhood add the inbound-command block:
  `MAVLINK_CMD_TRIP_RESET_MAGIC` (`1.0f`) and `MAVLINK_CMD_PARAM_EPSILON` (`0.01f`), commented as
  "`param1` magic for `MAV_CMD_USER_1`; never compare floats with `==`".
- [ ] 1.3 **No new config key, no NVS-tunable value, no web-settable parameter** — grep the diff to
  confirm nothing in `Constants.h` gained a runtime setter.

## 2. SpeedSensor: accumulators and persistence
- [ ] 2.1 `include/SpeedSensor.h`: private `uint64_t odoMm_`, `uint64_t tripMm_`,
  `uint64_t lastOdoWriteMm_`; public `getOdoMm()`, `getTripMm()`, `getOdoKm()`, `getTripKm()`
  (float, `× 1e-6`), `resetTrip()` and `persistDistance()`. Update the class-comment block: the
  `"speed"` namespace now holds calibration **and** distance.
- [ ] 2.2 Initialise all three to 0 in the constructor initialiser list.
- [ ] 2.3 `begin()`: load `odo_mm` and `trip_mm` from the existing `"speed"` namespace with
  `getULong64(key, 0)`; a missing key reads 0. Seed `lastOdoWriteMm_ = odoMm_`. Log the loaded
  values on the `VEHICLE` feature (`[SPEED] ODO 1234.567 km, TRIP 12.345 km restored`).
- [ ] 2.4 `update()`: inside the existing `if (delta > 0)` branch — i.e. **after** the
  `delta > SPEED_MAX_PULSES_PER_SAMPLE` wrap-guard `return` — add
  `const uint64_t stepMm = (uint64_t)llroundf(delta * distancePerPulseMm_);` and add it to both
  accumulators. Nothing is added in the decay/suspicious path.
- [ ] 2.5 `update()`: after accumulating, if `odoMm_ - lastOdoWriteMm_ >= ODO_NVS_WRITE_INTERVAL_MM`
  call `persistDistance()`. The comparison is unsigned and monotonic — no wrap handling needed.
- [ ] 2.6 `persistDistance()`: open `"speed"` read-write, `putULong64("odo_mm", odoMm_)` and
  `putULong64("trip_mm", tripMm_)`, `end()`, set `lastOdoWriteMm_ = odoMm_`. Tolerate a failed
  `begin()` by logging and returning — never block, never retry in a loop.
- [ ] 2.7 `resetTrip()`: zero `tripMm_`, call `persistDistance()` immediately, log
  `[SPEED] TRIP reset to 0 (ODO %.3f km unchanged)`. It SHALL NOT touch `odoMm_`.
- [ ] 2.8 Confirm by inspection that no code path anywhere can decrease or zero `odoMm_`, and that
  the calibration setters (`setPulsesPerRev`, `setWheelCircumferenceMm`) leave both accumulators
  untouched.

## 3. VehicleController: ignition-OFF flush and trip-reset consumption
- [ ] 3.1 `src/VehicleController.cpp`, `processMavlinkCommands()`: in the ignition switch's
  `case MavlinkInterface::IgnitionState::OFF` (currently line 422), flush on the **transition
  only** — `if (previousIgnitionState_ != MavlinkInterface::IgnitionState::OFF)
  speedSensor_.persistDistance();` — using the `previousIgnitionState_` tracker already written at
  the bottom of the same function (line 440). Do not flush while OFF is merely being held.
- [ ] 3.2 Same flush on the web ignition path: `VehicleController::setIgnitionState()` (line 764)
  when the parsed target is `OFF` and the current relay state is not already OFF.
- [ ] 3.3 `applyFailsafe()` (line 373, the `relayController_.allOff()` branch): flush once on
  fail-safe entry, alongside the existing `previousIgnitionState_` reset — a fail-safe is a
  power-down in every respect that matters to the counters.
- [ ] 3.4 `update()`: `if (mavlink_.consumeTripResetRequest()) speedSensor_.resetTrip();` —
  the transport never holds a `SpeedSensor&`.
- [ ] 3.5 `include/VehicleController.h`: read-only accessors `getOdoKm()` / `getTripKm()`
  forwarding to `speedSensor_`, beside the existing `getSpeedPulsesPerRev()` pair (line 211).

## 4. MavlinkInterface: report ODO/TRIP, handle the inbound trip reset
- [ ] 4.1 `include/MavlinkInterface.h`: add `float odoKm;` and `float tripKm;` to `StateReport`,
  documented as always valid (never `NaN`, independent of CAN health).
- [ ] 4.2 `src/MavlinkInterface.cpp`, the `mavlink_msg_efi_status_pack()` call (line 441): replace
  the `0.0f, // barometric_pressure (unused)` argument with `state.odoKm` and the
  `0.0f); // fuel_pressure (unused)` argument with `state.tripKm`, and update the field-mapping
  comments to `<- ODOMETER (km, total)` and `<- TRIP (km, resettable)`.
- [ ] 4.3 `src/main.cpp`: populate `StateReport::odoKm` / `tripKm` from the new
  `VehicleController` accessors where the rest of the report is filled.
- [ ] 4.4 `include/MavlinkInterface.h`: private `bool tripResetPending_`, `uint8_t cmdAckSysid_`,
  `uint8_t cmdAckCompid_`; public `bool consumeTripResetRequest()` (returns the flag and clears
  it); private `void handleCommandLong(uint8_t sysid, uint8_t compid, uint8_t targetSys,
  uint8_t targetComp, uint16_t command, float param1)` — **decoded scalars only**, the header stays
  free of mavlink headers.
- [ ] 4.5 Initialise the new members in the constructor list and clear them in `begin()`.
- [ ] 4.6 Add `case MAVLINK_MSG_ID_COMMAND_LONG:` to the RX dispatch (beside the existing
  `COMMAND_ACK` case at line 111): decode with `mavlink_msg_command_long_decode()` and call
  `handleCommandLong()`.
- [ ] 4.7 Implement `handleCommandLong()`:
  - return immediately unless `targetSys == MAVLINK_SYSTEM_ID && targetComp ==
    MAVLINK_COMPONENT_ID` — **no ACK** for a broadcast (`targetComp == 0`) or another component;
  - `command == MAV_CMD_USER_1` and `fabsf(param1 - MAVLINK_CMD_TRIP_RESET_MAGIC) <=
    MAVLINK_CMD_PARAM_EPSILON` → set `tripResetPending_`, ACK `MAV_RESULT_ACCEPTED`, log
    `[MAV] TRIP reset accepted from %u/%u`;
  - `command == MAV_CMD_USER_1`, any other `param1` (`NaN` included) → ACK `MAV_RESULT_DENIED`, log
    `[MAV] TRIP reset DENIED (param1=%.2f) from %u/%u`;
  - any other command → ACK `MAV_RESULT_UNSUPPORTED`, log
    `[MAV] command %u UNSUPPORTED from %u/%u`.
- [ ] 4.8 Send the `COMMAND_ACK` with `mavlink_msg_command_ack_pack()` from
  `MAVLINK_SYSTEM_ID`/`MAVLINK_COMPONENT_ID`, `target_system = msg.sysid`,
  `target_component = msg.compid`, `progress = 0`, `result_param2 = 0`.
- [ ] 4.9 Confirm the existing `COMMAND_ACK` **inbound** case still only logs
  `MAV_CMD_SET_MESSAGE_INTERVAL` results and is not confused by the ESP32's own outbound ACKs.
- [ ] 4.10 `src/MavlinkInterface.cpp:478-488`, the `VFR_HUD` pack: replace
  `float groundSpeedMs = (state.speedValid && state.speedMs > 0.0f) ? state.speedMs : 0.0f;` with
  `float groundSpeedMs = state.speedValid ? state.speedMs : NAN;`. A genuine `0.0` now reaches the
  wire as `0.0`; the `> 0.0f` guard goes, since `speedMs_` is already clamped at zero by the decay
  path. Rewrite the comment block above it: `groundspeed` is `NaN` when the SENSOR is invalid so a
  consumer can tell "no reading" from "stopped", `throttle` is unchanged (a `uint16_t` percent has
  no `NaN` encoding), and `VISION_POSITION_DELTA` is untouched — it is a fusable measurement and
  already goes silent on an invalid reading.
- [ ] 4.11 Confirm no other outbound path substitutes zero for an invalid speed: grep
  `speedValid` across `src/` and check `sendVisionPositionDelta()` still *suppresses* rather than
  reporting, and that the speed limiter's fail-open behaviour is untouched.

## 5. Telemetry and web UI
- [ ] 5.1 `include/WebPortal.h`: add `float odo_km;` and `float trip_km;` to the telemetry struct,
  in the hall-sensor block beside `vehicle_speed_ms` (line 76).
- [ ] 5.2 `src/TelemetryManager.cpp`: populate both from the `VehicleController` accessors beside
  the existing `vehicle_speed_ms` / `speed_valid` lines (line 74).
- [ ] 5.3 `src/WebPortal.cpp`: serialise `doc["odo_km"]` and `doc["trip_km"]` with
  `serialized(String(value, 3))`, unconditionally — independent of CAN and MAVLink status, beside
  `vehicle_speed` (line 578).
- [ ] 5.4 `data/index.html`: two read-only rows in the speed card (after the speed-limit rows,
  ~line 786), `id="odo-value"` and `id="trip-value"`; **no button, no input, no reset control**.
- [ ] 5.5 `data/index.html`, `updateSpeedDisplay()` (~line 2094): render both as
  `value.toFixed(3) + ' ' + t('unit_km')`, defaulting to `--` when the field is absent. They are
  **not** gated on `speed_valid`: a stale sensor does not invalidate distance already driven.
- [ ] 5.6 `data/index.html`: add `lbl_odo`, `lbl_trip` and `unit_km` to **both** the `en` and `uk`
  dictionaries (Odometer / Trip / km — Одометр / Пробіг / км) and verify EN/UK key parity.
- [ ] 5.7 `MAVLINK_SETUP.md`: add two rows to the "Vehicle state reported back to the autopilot"
  table (`Total odometer (km)` → `barometric_pressure`, `Trip distance (km)` → `fuel_pressure`,
  source "GPIO 8 / X2", component 25, 5 Hz, always valid); add a short **Trip reset** subsection
  documenting `COMMAND_LONG` / `MAV_CMD_USER_1` / `param1 = 1`, the strict addressing, the three
  ACK results, and that ODO cannot be reset.
- [ ] 5.7a `MAVLINK_SETUP.md` line 79, the `Ground speed (hall sensor)` row: note **"`NaN` when the
  sensor is invalid (never pulsed or latched suspicious); a genuine 0.0 means stopped"**, and say in
  the surrounding text that `VFR_HUD.throttle` and `VISION_POSITION_DELTA` are unaffected.
- [ ] 5.8 `WEB_PORTAL_SETUP.md`: one short note that the portal **displays** ODO and TRIP and that
  the trip reset is a ground-station action only.

## 6. Build and static checks
- [ ] 6.1 `pio run -e esp32-s3-devkitc-1` completes with no errors and no new warnings from `src/`
  or `include/`.
- [ ] 6.2 `grep -rn 'odoMm_' src include` shows assignment only in `begin()` (load) and the
  `delta > 0` branch of `update()` — never a decrement, never a zero.
- [ ] 6.3 `grep -rn 'resetTrip' src include` shows exactly one call site, in
  `VehicleController::update()`, reached only via `consumeTripResetRequest()`.
- [ ] 6.4 `[WEB] telemetry JSON peak:` stays below 4096 with no overflow warning (two extra keys,
  ~30 bytes).

## 7. Bench and road verification
- [ ] 7.1 **[OPERATOR]** Flash firmware **and** `pio run -t uploadfs` (`data/` changed).
- [ ] 7.2 **[OPERATOR]** First boot on a virgin NVS: serial shows `ODO 0.000 km, TRIP 0.000 km`,
  the portal shows `0.000 km` for both, and Mission Planner shows `barometric_pressure = 0` and
  `fuel_pressure = 0`.
- [ ] 7.3 **[OPERATOR]** **Measured 100 m drive.** Mark a 100 m course with a tape. Drive it once
  forward; ODO and TRIP both increase by 0.100 km ± the wheel-calibration error (≤ 3 m with the
  default 70 ppr × 1990 mm). If the error is larger, correct `speed_cal_circ` **now**, before real
  distance accumulates, and repeat — recalibration does not rewrite what is already stored.
- [ ] 7.4 **[OPERATOR]** **Reverse counts.** Drive the same 100 m in reverse: both counters
  increase again by 0.100 km (they never decrease).
- [ ] 7.5 **[OPERATOR]** **Trip reset from Mission Planner.** Send `MAV_CMD_USER_1` with
  `param1 = 1` to component 25. Within 200 ms: `COMMAND_ACK` = `ACCEPTED` at the GCS,
  `[MAV] TRIP reset accepted from 255/190` on serial, `fuel_pressure` = 0 in the next `EFI_STATUS`,
  and the portal's Trip row reads `0.000 km`.
- [ ] 7.6 **[OPERATOR]** **ODO unchanged after the reset.** `barometric_pressure` and the portal's
  Odometer row hold exactly the value they had before 7.5.
- [ ] 7.7 **[OPERATOR]** **Bogus `param1` is denied and logged.** Send `MAV_CMD_USER_1` with
  `param1 = 3`: `COMMAND_ACK` = `DENIED`, `[MAV] TRIP reset DENIED (param1=3.00) from 255/190`, and
  TRIP is **unchanged**. Repeat with `param1 = 0` — also denied.
- [ ] 7.8 **[OPERATOR]** **An unrelated command is answered UNSUPPORTED.** Send
  `MAV_CMD_DO_SET_MODE` addressed to 1/25: `COMMAND_ACK` = `UNSUPPORTED`, one log line, no state
  change.
- [ ] 7.9 **[OPERATOR]** **The autopilot's own commands are untouched.** With the GCS connected,
  arm, disarm and change mode normally: the ESP32 emits **no** `COMMAND_ACK` for any of them
  (broadcasts and commands for component 1 are ignored silently) and Mission Planner shows no
  duplicate or conflicting acknowledgement.
- [ ] 7.10 **[OPERATOR]** **NVS survives a power cycle.** Drive ≥ 2 km, switch ignition OFF, pull
  power, re-power: ODO and TRIP come back within a few metres of their pre-shutdown values.
- [ ] 7.11 **[OPERATOR]** **The kilometre write works without an ignition cycle.** Drive ≥ 1 km,
  then pull power **without** switching ignition OFF: at most the last kilometre is lost, and ODO
  is non-zero.
- [ ] 7.12 **[OPERATOR]** **TRIP spans ignition cycles.** With a non-zero TRIP, switch ignition OFF
  and back ON: TRIP continues from where it was (it clears only on command).
- [ ] 7.13 **[OPERATOR]** **A disconnected sensor adds no distance.** Unplug the hall lead while
  rolling: `speed_valid` goes false, `[SPEED] WARNING: pulses stopped implausibly fast …` appears,
  and neither counter moves while the lead is out.
- [ ] 7.13a **[OPERATOR]** **`VFR_HUD.groundspeed` distinguishes "no reading" from "stopped".**
  With the vehicle stationary and the sensor healthy, the component-25 `VFR_HUD.groundspeed` reads
  exactly `0.0` on the wire (MAVInspector / a `mavlink` log). Unplug the hall lead: the same field
  reads `NaN` and the widget shows "--". Replug and roll: it reads the live m/s again.
- [ ] 7.13b **[OPERATOR]** **Mission Planner's own state is unaffected by the `NaN`.** With the
  hall lead out, MP's HUD ground speed, `cs.groundspeed` and the tuning graphs continue to show the
  autopilot's own value with no `NaN`, no blanked HUD and no exception in the MP log —
  component-25 packets do not feed MP's vehicle state.
- [ ] 7.14 **[OPERATOR]** **Legacy-firmware sanity.** With the previous firmware flashed,
  `EFI_STATUS.barometric_pressure` and `fuel_pressure` read 0 — confirming a plugin that shows the
  new fields degrades to "0.000 km", not to a wrong distance, against an un-updated vehicle.
- [ ] 7.15 **[OPERATOR]** Record the final odometer reading and the calibration
  (`speed_ppr` / `speed_circ_mm`) in the commissioning notes, since the odometer is only as good as
  the calibration in force while it counted.
