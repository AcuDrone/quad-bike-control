## 1. Constants (`include/Constants.h`)
- [ ] 1.1 In the MAVLink timing block beside `MAVLINK_REPORT_TX_MS` (line 215), add
  `MAVLINK_ESC_INFO_TX_MS` (`1000`) commented as "ms between outbound ESC_INFO messages (1 Hz);
  ESC_STATUS rides the MAVLINK_REPORT_TX_MS tick".
- [ ] 1.2 Add `MAVLINK_ESC_INDEX` (`0`) and `MAVLINK_ESC_COUNT` (`1`) in the same block, commented
  as "slot 0 is the steering VESC; there is exactly one ESC on this vehicle, and `count` is what
  tells a consumer slots 1..3 are not data".
- [ ] 1.3 **No new runtime-settable value** — no NVS key, no web command, no parameter. Grep the
  diff to confirm `Constants.h` gained only compile-time defines.

## 2. Driver counters (`include/IMotorDriver.h`, `VescMotorDriver`)
- [ ] 2.1 `include/IMotorDriver.h`: add two optional telemetry hooks beside the existing ones
  (lines 38-52), both defaulting to `0` so `BTS7960Controller` needs no change:
  `virtual uint16_t replyCount() const { return 0; }` — valid telemetry replies received since
  boot, wraps; `virtual uint32_t faultEventCount() const { return 0; }` — 0 → non-zero fault-code
  transitions since boot.
- [ ] 2.2 `include/VescMotorDriver.h`: private `uint16_t replyCount_;` and
  `uint32_t faultEventCount_;` in the telemetry-state block beside `haveReply_`; override both
  accessors inline.
- [ ] 2.3 Initialise both to 0 in the constructor initialiser list (`src/VescMotorDriver.cpp`).
- [ ] 2.4 `VescMotorDriver::handlePacket()` (`src/VescMotorDriver.cpp:133-140`): inside the
  successful-decode branch and **before** `values_ = v;`, capture
  `const uint8_t prevFault = values_.faultCode;`; after the assignment, `replyCount_++;` and
  `if (v.faultCode != 0 && prevFault == 0) faultEventCount_++;`.
- [ ] 2.5 Confirm by inspection that both counters are written **only** in `handlePacket()` — never
  in `begin()`, `poll()`, `stop()` or `setSpeed()` — so they are boot-cumulative and a stalled
  `replyCount_` is itself the "VESC silent" diagnostic.
- [ ] 2.6 `include/SteeringController.h`: forward both beside the existing VESC telemetry
  accessors (lines 105-113) as `getVescReplyCount()` and `getVescFaultEvents()`, with the same
  Doxygen comment style. `getVescFault()` already exposes the raw code — do not duplicate it.

## 3. MavlinkInterface: report the two messages
- [ ] 3.1 `include/MavlinkInterface.h`: extend `StateReport` (line 71) with a commented
  steering-driver block — `bool steerDriverOk;` (validity gate, the VESC's own, independent of
  `canValid`), `float steerMotorCurrentA;`, `float steerFetTempC;`, `float steerInputVoltageV;`
  (VESC-measured 24 V boost rail, **not** the ECU module voltage in `ignition_voltage`),
  `uint8_t steerVescFault;` (raw `mc_fault_code`), `uint16_t steerReplyCount;`,
  `uint32_t steerFaultEvents;`. Decoded scalars only — the header stays free of MAVLink headers
  and of any `SteeringController` dependency.
- [ ] 3.2 `include/MavlinkInterface.h`: in the same block add the steering **position** fields,
  commented as having a **different validity gate from the driver fields above** —
  `float steerPercent;` (−100 = left limit … 0 = centre … +100 = right limit, percent of the
  calibrated lock-to-lock range, **not** degrees), `bool steerSensorOk;`,
  `bool steerCalibrated;`. `steerPercent` is meaningful only while both flags are true.
- [ ] 3.3 `include/MavlinkInterface.h`: private `uint32_t lastEscInfoTx_;` beside
  `lastHeartbeatTx_`, and a private helper
  `uint16_t mapVescFaultToEscFlags(uint8_t faultCode) const;`.
- [ ] 3.4 Initialise `lastEscInfoTx_(0)` in the constructor list (`src/MavlinkInterface.cpp:29`
  neighbourhood) and clear it in `begin()` alongside the other transmit timestamps.
- [ ] 3.5 `src/main.cpp`: fill the new `StateReport` fields from
  `vehicleController.getSteering()` beside the existing report lines (`src/main.cpp:280-302`) —
  `isDriverOk()`, `getMotorCurrent()`, `getFetTemp()`, `getInputVoltage()`, `getVescFault()`,
  `getVescReplyCount()`, `getVescFaultEvents()`.
- [ ] 3.6 `src/main.cpp`: fill the three steering-position fields from the same
  `vehicleController.getSteering()` reference, on adjacent lines with a short comment marking the
  gate — `report.steerPercent = steering.getSteeringPercent();` (declared at
  `include/SteeringController.h:57`, implemented at `src/SteeringController.cpp:363-376`),
  `report.steerSensorOk = steering.isSensorOk();` and
  `report.steerCalibrated = steering.isCalibrated();` (both inline at
  `include/SteeringController.h:91-92`). Fill all three unconditionally — the gating is applied
  where the message is packed, not here, so the report stays a plain snapshot.
- [ ] 3.7 `src/MavlinkInterface.cpp`, inside the existing
  `if (now - lastReportTx_ >= MAVLINK_REPORT_TX_MS)` block (line 499), **after** the `VFR_HUD`
  write and **before** the `sendVisionPositionDelta()` call, pack and send `ESC_STATUS`:
  - `index = MAVLINK_ESC_INDEX`;
  - `time_usec = (uint64_t)esp_timer_get_time()` — `esp_timer.h` is already included at line 7;
    **never** `micros()` (32-bit, wraps every ~71 min — the lesson already recorded at line 695);
  - `float voltage[4] = { state.steerDriverOk ? state.steerInputVoltageV : NAN, NAN, NAN, NAN };`
  - `float current[4] = { state.steerDriverOk ? state.steerMotorCurrentA : NAN, NAN, NAN, NAN };`
  - `int32_t rpm[4] = { 0, 0, 0, 0 };` — then set `rpm[0]` to the measured steering position:
    `rpm[0] = (state.steerSensorOk && state.steerCalibrated)
              ? (int32_t)lroundf(state.steerPercent * 100.0f)
              : INT32_MIN;`
    i.e. centi-percent of the calibrated lock-to-lock range, −10000 (left lock) … 0 (centre) …
    +10000 (right lock), with `INT32_MIN` as the "position unknown" sentinel. `rpm[1..3]` stay 0.
  - **The gate is `steerSensorOk && steerCalibrated`, NOT `steerDriverOk`** — different sensor,
    different failure mode. Do not fold it into the `state.steerDriverOk ? … : NAN` expressions
    above; a reviewer must be able to see the two gates are separate.
  - `<limits.h>`/`INT32_MIN` availability: confirm the constant resolves (Arduino's toolchain
    pulls `stdint.h`); if not, use `(int32_t)0x80000000` with the same comment.
  - argument order verified against
    `.pio/libdeps/esp32-s3-devkitc-1/c_library_v2/common/mavlink_msg_esc_status.h`
    (`index, time_usec, rpm, voltage, current`).
- [ ] 3.8 Write the comment block above the pack, in the style of the `EFI_STATUS` block
  (lines 460-497): why `ESC_STATUS` rather than a repurposed `EFI_STATUS` field; that `voltage[0]`
  is the **24 V boost rail measured at the VESC**, a different quantity from
  `EFI_STATUS.ignition_voltage` (ECU module voltage, PID `0x42`); that `current[0]` is **motor**
  current, not input current, because one physical ESC gets one slot; and that slots 1..3 are `NaN`
  with `ESC_INFO.count = 1` marking them as not data.
- [ ] 3.9 In the same comment block, give `rpm[0]` its own paragraph — it is the one repurposed
  field here and the one a future maintainer will question:
  - it carries the **MEASURED steering position in centi-percent** of the calibrated lock-to-lock
    range (−10000 = left lock, 0 = centre, +10000 = right lock), **not** degrees and **not** a
    shaft speed;
  - the VESC's ERPM is **not decoded**: brushed-DC, no motor sensor, so the quantity the field
    names is permanently unmeasurable on this vehicle — the same permanent-absence test
    `barometric_pressure` / `fuel_pressure` pass in the `EFI_STATUS` block above (lines 482-486);
  - **the exception to that permanence, stated plainly:** if the VESC ever gains an encoder or the
    steering motor becomes BLDC, `rpm` becomes a real measurement and the steering position must
    move elsewhere — a breaking change in lockstep with the plugin;
  - `INT32_MIN` = position unknown (sensor down or uncalibrated). **0 cannot mean unknown, because
    0 is exactly straight-ahead**, and `getSteeringPercent()` itself returns 0.0f when
    uncalibrated — the same "a real zero is a real reading" trap the `fuel_pressure` note records;
  - its validity gate is the **AS5600's, not the VESC's**, so `rpm[0]` can be live while
    `voltage[0]`/`current[0]` are `NaN` and vice versa — the same sensor-driven-`NaN` caveat
    `fuel_flow` already carries (lines 488-489);
  - consumer note: the **commanded** steering is already on the link as `SERVO_OUTPUT_RAW` from the
    autopilot's steering channel, so `rpm[0]` completes the command-vs-actual pair the same way
    `throttle_out` vs `throttle_position` does.
- [ ] 3.10 `src/MavlinkInterface.cpp`: add the `ESC_INFO` block on its own timer, following the
  `lastHeartbeatTx_` pattern (lines 434-435): `if (now - lastEscInfoTx_ >= MAVLINK_ESC_INFO_TX_MS)`
  → `index = MAVLINK_ESC_INDEX`, `time_usec` from the same `esp_timer_get_time()` source,
  `counter = state.steerReplyCount`, `count = MAVLINK_ESC_COUNT`,
  `connection_type = ESC_CONNECTION_TYPE_SERIAL`, `info = state.steerDriverOk ? 1 : 0` (bit0 =
  ESC online), `failure_flags[4]`, `error_count[4]`, `temperature[4]`. Argument order verified
  against `…/mavlink_msg_esc_info.h`
  (`index, time_usec, counter, count, connection_type, info, failure_flags, error_count,
  temperature`).
- [ ] 3.11 `ESC_INFO` slot values:
  - `temperature[0] = state.steerDriverOk ? (int16_t)constrain(lroundf(state.steerFetTempC *
    100.0f), -32768L, 32766L) : INT16_MAX;` — centi-degrees C, and `INT16_MAX` is the sentinel the
    message definition itself assigns to "data not supplied by ESC". Clamp **below** `INT16_MAX` so
    a real temperature can never be mistaken for the sentinel;
  - `temperature[1..3] = INT16_MAX`;
  - `failure_flags[0] = state.steerDriverOk ? mapVescFaultToEscFlags(state.steerVescFault) : 0` —
    a fault code read from a dead link is not a fault observation; `failure_flags[1..3] = 0`;
  - `error_count[0] = state.steerFaultEvents` (sent unconditionally — cumulative history, not a
    live reading); `error_count[1..3] = 0`.
- [ ] 3.12 Implement `mapVescFaultToEscFlags()` as a `switch` over the VESC `mc_fault_code`:
  `1 (OVER_VOLTAGE) → ESC_FAILURE_OVER_VOLTAGE (2)`;
  `2 (UNDER_VOLTAGE) → ESC_FAILURE_OVER_VOLTAGE (2)` **with a comment that this is lossy — the
  `ESC_FAILURE_FLAGS` enum has no under-voltage bit, so the category is right and the direction is
  not, and the raw code stays visible on the web portal**;
  `3 (DRV) → ESC_FAILURE_GENERIC (64)`; `4 (ABS_OVER_CURRENT) → ESC_FAILURE_OVER_CURRENT (1)`;
  `5 (OVER_TEMP_FET) → ESC_FAILURE_OVER_TEMPERATURE (4)`;
  `6 (OVER_TEMP_MOTOR) → ESC_FAILURE_OVER_TEMPERATURE (4)`; `0 → 0`; `default → ESC_FAILURE_GENERIC`.
  Comment that the numeric codes are **VESC-firmware-version dependent** (same caveat as the
  payload offsets in `include/VescProtocol.h:60-70`) and must be bench-verified (task 6.8).
- [ ] 3.13 Confirm both messages are sent **unconditionally of link state and of
  `state.steerDriverOk`** — they are gated only by their own timers, exactly like `EFI_STATUS`, so
  "peripheral alive, ESC down" is distinguishable from "peripheral gone".
- [ ] 3.14 Confirm nothing in the steering control path reads the new fields: grep
  `steerDriverOk` / `steerPercent` / `steerSensorOk` / `steerCalibrated` / `replyCount` /
  `faultEventCount` across `src/` and check every hit is a report fill, a counter increment, or an
  accessor. Reading `getSteeringPercent()` for the report MUST not change how or when the steering
  loop reads it. This change adds **no** stop, latch or interlock.

## 4. Documentation (`MAVLINK_SETUP.md`)
- [ ] 4.1 Add a `### Steering ESC telemetry (ESC_STATUS / ESC_INFO)` subsection under
  "Vehicle state reported back to the autopilot", after the odometer/trip subsection (line 159
  neighbourhood).
- [ ] 4.2 Add rows to the main reporting table (line 64 neighbourhood): `Steering ESC input
  voltage (V)` → `ESC_STATUS.voltage[0]`, `Steering motor current (A)` →
  `ESC_STATUS.current[0]`, source "VESC UART", component 25, 5 Hz; `Steering ESC FET temp (cdegC)`
  → `ESC_INFO.temperature[0]`, `Steering ESC health / failure flags` →
  `ESC_INFO.info` / `failure_flags[0]`, component 25, 1 Hz. Add a row `Steering position, MEASURED
  (centi-percent)` → `ESC_STATUS.rpm[0]`, source "AS5600", component 25, 5 Hz, carrying the ⚑
  repurposed mark — it is the only repurposed field in this change.
- [ ] 4.3 Document field by field: `index = 0` is the **steering** ESC (not a propulsion motor);
  `count = 1`, so slots 1..3 must be ignored; `voltage[0]` is the **24 V boost rail measured at the
  VESC**, explicitly distinct from `EFI_STATUS.ignition_voltage` (ECU 12 V module supply, PID
  `0x42`) and complementary to the board-side ADC reading in the web portal; `current[0]` is
  **motor** current, not input current.
- [ ] 4.4 Give `rpm[0]` its own paragraph: it is the **MEASURED steering position in
  centi-percent** of the calibrated lock-to-lock range — `−10000` = left lock, `0` = centre,
  `+10000` = right lock, **negative is left / positive is right** — and explicitly **percent of
  calibrated travel, not degrees** (the firmware holds no counts-to-degrees calibration). State
  that the VESC's ERPM is not reported (brushed-DC, no motor sensor), that this repurposing rests
  on that absence, and — beside the field — that **if the VESC ever gains an encoder or the motor
  becomes BLDC, this encoding must be revisited**. Add the consumer note: the **commanded**
  steering is already on the link as `SERVO_OUTPUT_RAW` from the autopilot's steering channel, so
  `rpm[0]` gives the measured half of the same pair, exactly as `throttle_out` /
  `throttle_position` do for throttle. Note the generic-tool hazard: a log viewer will label this
  "RPM".
- [ ] 4.5 Document the validity conventions in one place: `NaN` in `voltage[0]`/`current[0]` and
  `INT16_MAX` in `temperature[0]` mean "no reading" (VESC silent for more than
  `STEER_VESC_COMM_TIMEOUT_MS`, or never seen since boot) — a consumer MUST render "--";
  `info` bit0 clear says the same thing positively; **both messages keep flowing** in that state,
  which is how "VESC down" is told apart from "peripheral down". Note that
  `ESC_INFO.counter` is a `uint16_t` that wraps (~5.5 h at 3.3 Hz) and that only its *advance* is
  meaningful.
- [ ] 4.6 In the same place, document `rpm[0] = INT32_MIN` (`-2147483648`) as "steering position
  unknown" — AS5600 not OK or steering not calibrated — and state explicitly **why 0 is not the
  sentinel**: 0 means exactly straight-ahead. State that this gate is **independent** of the VESC's
  (`info` bit0 / the `NaN`s), so `rpm[0]` may be valid while voltage and current are `NaN`, and the
  reverse — the same caveat `fuel_flow` already carries in the `EFI_STATUS` section. Warn that a
  consumer averaging or plotting `rpm[]` without testing for the sentinel gets a wild outlier,
  the integer counterpart of the `NaN` hazard noted for the float arrays.
- [ ] 4.7 Include the VESC-fault → `ESC_FAILURE_FLAGS` mapping table from design.md, with the
  under-voltage caveat and the firmware-version caveat stated in the document itself.
- [ ] 4.8 In `### Ground-station plugin compatibility` (line 219), add a short **non-breaking**
  note: the plugin must add packet subscriptions for **msg id 291 (`ESC_STATUS`)** and **290
  (`ESC_INFO`)** to show the new data; nothing it decodes today changes, so an un-updated plugin
  keeps working and simply does not display steering-ESC telemetry. State that Mission Planner will
  **not** populate its native `esc1_*` fields from component 25 — the raw packet is delivered by
  ArduPilot's forwarding and the plugin reads it directly, exactly as it already does for
  `EFI_STATUS`.
- [ ] 4.9 `WEB_PORTAL_SETUP.md` is **not** changed — the portal already shows steering current,
  FET temperature, fault code and driver-ok, and this change adds no web field.

## 5. Build and static checks
- [ ] 5.1 `pio run -e esp32-s3-devkitc-1` completes with no errors and no new warnings from `src/`
  or `include/`.
- [ ] 5.2 Confirm `ESC_STATUS`/`ESC_INFO` compile from the already-included dialect
  (`<ardupilotmega/mavlink.h>` → `common/`) with **no** new include, no new library and no
  `platformio.ini` change.
- [ ] 5.3 `grep -n 'micros()' src/MavlinkInterface.cpp` returns nothing on either new path — both
  timestamps come from `esp_timer_get_time()`.
- [ ] 5.4 Confirm the two new `StateReport` fields blocks are filled on **every** call site of the
  report (grep `StateReport` in `src/`), so no path sends an uninitialised struct.

## 6. Bench verification
- [ ] 6.1 **[OPERATOR]** Flash firmware (`pio run -t upload`). **No `uploadfs`** — `data/` is
  unchanged.
- [ ] 6.2 **[OPERATOR]** **Messages arrive.** Mission Planner → MAVLink Inspector shows
  `ESC_STATUS` (291) at ~5 Hz and `ESC_INFO` (290) at ~1 Hz under **sysid 1, compid 25**, alongside
  the existing `EFI_STATUS` from the same component.
- [ ] 6.3 **[OPERATOR]** **Link budget.** With both new messages flowing, the inbound
  `SERVO_OUTPUT_RAW` rate on the 1 Hz MAVLink debug line stays at its previous value (≥
  `MAVLINK_STREAM_MIN_RATE_HZ`), and no stream re-request storm appears in the log.
- [ ] 6.4 **[OPERATOR]** **Plausible values, healthy VESC.** `voltage[0]` reads ≈24 V and tracks
  the portal's 24 V rail readout within the divider tolerance; `current[0]` sits near 0 A with the
  steering idle and rises visibly while the steering motor is driven against the locks;
  `ESC_INFO.temperature[0]` is a plausible FET temperature in cdegC (e.g. ~3000 = 30 °C) and
  matches the portal's `steer_fet_temp`; `info` bit0 = 1; `counter` advances by ~3 per second.
- [ ] 6.5 **[OPERATOR]** **Steering position sweeps and the sign is right.** With the steering
  calibrated, drive it slowly lock to lock and watch `ESC_STATUS.rpm[0]` in the Inspector:
  centred reads ≈ `0`, **full LEFT reads ≈ `-10000`**, **full RIGHT reads ≈ `+10000`**, the sweep
  is continuous with no jump through the sentinel, and the value tracks the web portal's steering
  percent ×100 throughout. Confirm the sign against the physical wheels, not against the portal —
  this is the one thing a wiring or calibration inversion would flip silently.
- [ ] 6.6 **[OPERATOR]** **Position matches the command.** Send a steering command from the GCS
  and confirm `rpm[0] / 100` converges on the commanded percent visible in `SERVO_OUTPUT_RAW`'s
  steering channel, lagging only while the actuator slews. A persistent offset after it settles is
  a calibration problem, not a reporting one.
- [ ] 6.7 **[OPERATOR]** **Unknown position is signalled, and only the position.** Clear the
  steering calibration (or disconnect the AS5600): `rpm[0]` reads exactly `INT32_MIN`
  (`-2147483648`), **never 0**, while `voltage[0]`, `current[0]`, `temperature[0]` and `info`
  bit0 stay live and healthy — this is the check that the two validity gates really are
  independent. Restore the calibration and confirm `rpm[0]` returns to a live value with no
  reboot.
- [ ] 6.8 **[OPERATOR]** **Fault-code mapping verified against the flashed firmware.** Read the
  `mc_fault_code` enumeration VESC Tool reports for the installed VESC firmware version and confirm
  the numeric values used in `mapVescFaultToEscFlags()` (1 = over-voltage, 2 = under-voltage,
  3 = DRV, 4 = abs over-current, 5 = over-temp FET, 6 = over-temp motor). If the flashed firmware
  numbers them differently, correct the `switch` and this table **before** the mapping is trusted.
  Record the firmware version in the commissioning notes.
- [ ] 6.9 **[OPERATOR]** **Induced fault is flagged.** Trigger one real VESC fault (the practical
  one: stall the steering against a lock until the VESC raises absolute over-current, or use VESC
  Tool to provoke a fault): `failure_flags[0]` becomes non-zero with the expected bit,
  `error_count[0]` increments by exactly 1 per fault episode, and the portal's raw
  `steer_vesc_fault` shows the underlying code. After the fault clears, `failure_flags[0]` returns
  to 0 and `error_count[0]` **holds** its value.
- [ ] 6.10 **[OPERATOR]** **VESC UART unplugged → positive "unknown".** Pull the VESC UART lead.
  Within `STEER_VESC_COMM_TIMEOUT_MS` (1 s): `voltage[0]` and `current[0]` read `NaN` (the plugin
  shows "--", not the last value), `temperature[0]` reads `INT16_MAX` (32767), `info` bit0 = 0,
  `failure_flags[0]` = 0, and `counter` **stops advancing** while `error_count[0]` holds. Both
  messages are **still arriving** in the Inspector — this is the check that distinguishes "VESC
  down" from "peripheral down". **`rpm[0]` keeps reading a live steering position** throughout
  (turn the wheel by hand to confirm it still moves) — the converse of task 6.7, and the proof that a
  dead ESC does not blind the shaft sensor.
- [ ] 6.11 **[OPERATOR]** **Recovery.** Replug the lead: within one `ESC_INFO` period `info` bit0
  returns to 1, `counter` resumes advancing, and live voltage/current/temperature return with no
  reboot.
- [ ] 6.12 **[OPERATOR]** **Cold boot with no VESC.** Power the ESP32 with the VESC unpowered: from
  the first message both report the "unknown" state above (`haveReply_` false from boot), never a
  zero-valued reading, and the boot is not blocked.
- [ ] 6.13 **[OPERATOR]** **Nothing else moved.** In the same session confirm `EFI_STATUS` fields,
  `VFR_HUD.groundspeed`, `VISION_POSITION_DELTA` and `STATUSTEXT` are unchanged in rate and value,
  and that steering behaviour (jog, centre, stall latch, fault stop) is identical to before the
  flash.
- [ ] 6.14 **[OPERATOR]** **Un-updated plugin is unaffected.** With the *previous* plugin build,
  confirm Mission Planner shows no error, no exception in its log, and the existing QuadBike
  readouts continue to work with the two unknown message ids on the link.
