## 1. Constants (`include/Constants.h`)
- [x] 1.1 In the MAVLink timing block beside `MAVLINK_REPORT_TX_MS`, replace the three ESC defines
  with a single `MAVLINK_STEER_SLOW_TX_MS` (`1000`), commented as "ms between the SLOW
  steering-VESC named floats (`VESC_TEMP`, `VESC_OK`) — 1 Hz; the other three ride the
  `MAVLINK_REPORT_TX_MS` tick at 5 Hz".
- [x] 1.2 Delete `MAVLINK_ESC_INFO_TX_MS`, `MAVLINK_ESC_INDEX` and `MAVLINK_ESC_COUNT`.
- [x] 1.3 **No new runtime-settable value** — no NVS key, no web command, no parameter. Confirm the
  diff shows `Constants.h` gained only one compile-time define and lost three.

## 2. Revert the driver counters
The counters existed only to fill `ESC_INFO.counter` / `error_count[0]`. With that message gone
they have no consumer, so the hunks of commit `89f84a6` that added them are reverse-applied exactly
— not re-implemented, not left in place.
- [x] 2.1 `include/IMotorDriver.h`: remove the `replyCount()` and `faultEventCount()` virtuals, so
  the interface returns to its pre-`89f84a6` set and `BTS7960Controller` is untouched.
- [x] 2.2 `include/VescMotorDriver.h`: remove both overrides and both private members.
- [x] 2.3 `src/VescMotorDriver.cpp`: remove both initialisers from the constructor list, and return
  `handlePacket()` to `values_ = v; haveReply_ = true; lastValidReplyTime_ = millis();` — no
  `prevFault` capture, no increments.
- [x] 2.4 `include/SteeringController.h`: remove `getVescReplyCount()` and `getVescFaultEvents()`.
  **Keep `getVescFault()`** — `src/TelemetryManager.cpp:57` still reads it for the web portal.
- [x] 2.5 Verify the revert is exact:
  `git diff 89f84a6^ -- include/IMotorDriver.h include/VescMotorDriver.h src/VescMotorDriver.cpp
  include/SteeringController.h` produces **no output**.

## 3. MavlinkInterface: the five named values
- [x] 3.1 `include/MavlinkInterface.h`: in `StateReport`, keep `steerDriverOk`,
  `steerMotorCurrentA`, `steerFetTempC`, `steerInputVoltageV` and the three steering-position
  fields; remove `steerVescFault`, `steerReplyCount` and `steerFaultEvents`. Retitle the driver
  block after the four names that now carry it.
- [x] 3.2 `include/MavlinkInterface.h`: rename `lastEscInfoTx_` to `lastSteerSlowTx_`; remove the
  `mapVescFaultToEscFlags()` declaration; update the class doc block to describe the five names,
  their rates, their gates and the scoped exception. The header stays free of MAVLink headers.
- [x] 3.3 `include/MavlinkInterface.h`: declare
  `void sendNamedFloat(uint32_t nowMs, const char* name, float value);` beside `sendStatusText`,
  with a comment recording that the name is zero-padded before packing and that the signature takes
  decoded scalars to keep the header light.
- [x] 3.4 `src/MavlinkInterface.cpp`: in an anonymous namespace after the includes, define the five
  names as `constexpr char NV_STEER_POS[] = "STEER_POS";` etc., each followed by
  `static_assert(sizeof(...) - 1 <= 10, ...)`. Record that they are never built at runtime and that
  a ten-character name goes out without a terminator.
- [x] 3.5 `src/MavlinkInterface.cpp`: rename the member in the constructor initialiser list and in
  `begin()`.
- [x] 3.6 `src/MavlinkInterface.cpp`: implement `sendNamedFloat()` beside `sendStatusText()` —
  `char name10[10] = {0}; strncpy(name10, name, sizeof(name10));`, then its own `msg`/`buf`, pack,
  `mavlink_msg_to_send_buffer`, `serial_->write`. Comment the zero-pad as load-bearing: the pack
  helper copies exactly ten bytes and a shorter literal would be read past its terminator.
- [x] 3.7 `src/MavlinkInterface.cpp`: replace the whole `ESC_STATUS` block — inside the
  `MAVLINK_REPORT_TX_MS` tick, after `VFR_HUD`, before the `MAVLINK_VISO_ENABLED` block — with a
  comment stating why the exception is scoped (the standard ESC ids are absent from the dialect MP
  decodes with; the plugin dispatches on `(compid, name)`; a Lua relay stays possible) and three
  calls:
  `const bool steerPosOk = state.steerSensorOk && state.steerCalibrated;` then
  `sendNamedFloat(now, NV_STEER_POS, steerPosOk ? state.steerPercent : NAN);`,
  `sendNamedFloat(now, NV_STEER_A, state.steerDriverOk ? state.steerMotorCurrentA : NAN);`,
  `sendNamedFloat(now, NV_VESC_V, state.steerDriverOk ? state.steerInputVoltageV : NAN);`.
  Keep the position gate as its own statement — different sensor, different failure mode.
- [x] 3.8 `src/MavlinkInterface.cpp`: replace the whole `ESC_INFO` block with the 1 Hz timer —
  `if (now - lastSteerSlowTx_ >= MAVLINK_STEER_SLOW_TX_MS) { lastSteerSlowTx_ = now;
  sendNamedFloat(now, NV_VESC_TEMP, state.steerDriverOk ? state.steerFetTempC : NAN);
  sendNamedFloat(now, NV_VESC_OK, state.steerDriverOk ? 1.0f : 0.0f); }` — with the comment
  recording that both are sent unconditionally and that `VESC_OK` is never `NaN`.
- [x] 3.9 `src/MavlinkInterface.cpp`: delete `mapVescFaultToEscFlags()` entirely.
- [x] 3.10 `src/MavlinkInterface.cpp`: `time_boot_ms` is the `now` (`millis()`) already computed in
  `report()`. Confirm `micros()` appears nowhere in the file.
- [x] 3.11 `src/MavlinkInterface.cpp`: extend the `EFI_STATUS` field-mapping comment so the
  collision argument still stands for the ECU values while naming the steering VESC's five names as
  the single scoped exception.

## 4. main.cpp
- [x] 4.1 `src/main.cpp`: remove the three assignments to `steerVescFault`, `steerReplyCount` and
  `steerFaultEvents` from the `StateReport` fill; keep `isDriverOk()`, `getMotorCurrent()`,
  `getFetTemp()`, `getInputVoltage()` and the three steering-position lines unchanged, and retitle
  the block's comment after the new names.
- [x] 4.2 Confirm `steering.getVescFault()` is still called from `src/TelemetryManager.cpp:57` for
  the web portal, and that `StateReport` is built in exactly one place.

## 5. Documentation (`MAVLINK_SETUP.md`)
- [x] 5.1 Replace the whole "Steering ESC telemetry" section with
  "### Steering VESC telemetry (`NAMED_VALUE_FLOAT`)": the five-name table with units, rates,
  unknown encodings and gates; the rates rationale; the `time_boot_ms` wrap note.
- [x] 5.2 In that section, record why the standard ESC pair was abandoned (absent from the dialect
  MP decodes with — zero frames in 50 s beside 277 `EFI_STATUS` on MP 1.3.83), and why
  `ESC_TELEMETRY_1_TO_4`, more repurposed `EFI_STATUS` fields and a Lua relay were each rejected.
- [x] 5.3 In that section, record the field notes: `VESC_V` is not `ignition_voltage`; `STEER_A` is
  motor current, not input current; `STEER_POS` is percent of calibrated travel with negative =
  left and `0.0` a real reading; the AS5600 gate is independent of the VESC gate (four-combination
  table); the command-vs-actual consumer note; and that the fault code and counters are not on the
  link. State plainly that this is a reporting path only.
- [x] 5.4 In that section, document the ten-byte `name` field: zero-padded before packing,
  compile-time literals, `static_assert`, and "trim on length, not on NUL" for the consumer.
- [x] 5.5 Update the validity subsection: the "no reading" table now lists the three `NaN` names,
  `VESC_OK` = `0.0` and `STEER_POS` as unaffected; all five keep flowing.
- [x] 5.6 Update the "fields that are always valid" list (one row for `VESC_OK`, "never `NaN`") and
  delete the non-`NaN`-sentinel table entirely — no integer sentinels remain.
- [x] 5.7 Update the validity-domains table and its bullets: the "Steering VESC link" domain now
  controls `STEER_A`/`VESC_V`/`VESC_TEMP` (→ `NaN`) and `VESC_OK` (→ `0.0`); the "position sensor"
  domain controls `STEER_POS` only (→ `NaN`).
- [x] 5.8 Replace the plugin-compatibility subsection with
  "#### Steering VESC named floats (251) — non-breaking": one subscription on id 251, mandatory
  `compid == 25` filter, dispatch on the trimmed name, the five-name table, `NaN` → `--`, and
  `VESC_OK` = `0` greying the VESC values. State that an un-updated plugin keeps working.
- [x] 5.9 Update every count and list that mentioned the ESC pair: the outbound-message table (one
  `NAMED_VALUE_FLOAT` row replacing two), the "Ten outbound message types: six periodic" count, the
  value-to-name summary table, the target-less forwarding list, the "native fields" sentence, the
  "one `EFI_STATUS`" justification (now noting the documented exception), and the
  "adding a new message id is not breaking" precedent list.
- [x] 5.10 Confirm no "Proposed" wording survives anywhere in the file: everything documented here
  is implemented.

## 6. OpenSpec, static checks and build
- [x] 6.1 Rewrite `openspec/changes/add-vesc-esc-telemetry/proposal.md` in place under the new
  carrier: the original motivation plus the dialect finding, the five names, the rejected
  alternatives, the non-breaking impact and the bandwidth estimate.
- [x] 6.2 Rewrite `design.md`: exactly five names and what was cut to get there, the rates, the NaN
  gates, why the collision argument does not bite here but still holds for the ECU values,
  `time_boot_ms` from `millis()`, the zero-padding helper and the `static_assert`s, the driver
  revert, the risks and the open questions.
- [x] 6.3 Replace `specs/mavlink-interface/spec.md` with ONE ADDED requirement,
  `Steering VESC Telemetry via NAMED_VALUE_FLOAT`, and its ten scenarios. Check the wording does not
  contradict the existing `NAMED_VALUE_FLOAT` prohibitions in
  `openspec/specs/mavlink-interface/spec.md` — the exception must be scoped to the steering VESC and
  position values, with the ECU/gear/odometer/trip prohibitions restated intact.
- [x] 6.4 Delete `specs/vehicle-actuators/spec.md` and its directory — the counters it specified no
  longer exist.
- [x] 6.5 `grep -n 'micros()' src/MavlinkInterface.cpp` → empty.
- [x] 6.6 `grep -n '#include' include/MavlinkInterface.h` → only `Arduino.h`, `Constants.h`,
  `TransmissionController.h`.
- [x] 6.7 `grep -rnE 'ESC_STATUS|ESC_INFO|MAVLINK_ESC_|lastEscInfoTx_|mapVescFaultToEscFlags|steerVescFault|steerReplyCount|steerFaultEvents|replyCount|faultEventCount|ESC_FAILURE_|ESC_CONNECTION_TYPE|INT32_MIN|INT16_MAX' include src MAVLINK_SETUP.md openspec/specs`
  → empty. The same grep over `openspec/changes/add-vesc-esc-telemetry` may match only in the prose
  of `proposal.md` / `design.md`, where the ESC pair is named as the rejected approach, and never
  under `specs/`.
- [x] 6.8 `grep -rn 'StateReport' src` → the snapshot is built in exactly one place
  (`src/main.cpp`).
- [x] 6.9 `openspec validate add-vesc-esc-telemetry --strict` and `openspec validate --all --strict`
  both pass; `openspec show add-vesc-esc-telemetry` lists only the `mavlink-interface` delta.
- [x] 6.10 `~/.platformio/penv/bin/pio run -e esp32-s3-devkitc-1` succeeds with no new warnings
  from `src/` or `include/`. Record RAM and Flash usage.

## 7. Bench verification `[OPERATOR]`
Nothing below can be done from the repo; all of it needs the vehicle powered and a GCS attached.
- [ ] 7.1 `pio run -t upload` (firmware only — **no** `uploadfs`, `data/` is unchanged).
- [ ] 7.2 MAVLink Inspector: `NAMED_VALUE_FLOAT` arriving from **sysid 1 / compid 25** at ≈17 Hz
  total, and all five names seen within a few seconds. The Inspector keeps ONE node per msgid and
  flickers between the names — that is expected; confirm the set through the plugin or a log
  alongside it.
- [ ] 7.3 Healthy VESC: `VESC_V` ≈ 24 V, `STEER_A` ≈ 0 at rest and rising against the stops,
  `VESC_TEMP` equal to the web portal's `steer_fet_temp`, `VESC_OK` = 1.
- [ ] 7.4 `STEER_POS`: ≈ 0 at centre, ≈ −100 at the left lock, ≈ +100 at the right lock. **Verify
  the sign against the actual wheels**, and that it converges on the commanded value from
  `SERVO_OUTPUT_RAW`.
- [ ] 7.5 Disconnect the AS5600 or clear the calibration: `STEER_POS` goes `NaN` while the other
  four stay live and `VESC_OK` stays 1. Restore without rebooting and confirm it recovers.
- [ ] 7.6 Disconnect the VESC UART: within ≤ 1 s `STEER_A`, `VESC_V` and `VESC_TEMP` go `NaN` and
  `VESC_OK` goes 0, **all five keep arriving**, and `STEER_POS` still moves when the wheel is turned
  by hand. Reconnect and confirm recovery within one slow period.
- [ ] 7.7 Cold start with the VESC unpowered: the very first frames already carry `NaN` and
  `VESC_OK` = 0, and boot is not blocked or delayed.
- [ ] 7.8 Bandwidth: the inbound `SERVO_OUTPUT_RAW` rate in the `[MAV]` console line is unchanged,
  and `EFI_STATUS`, `VFR_HUD`, `VISION_POSITION_DELTA` and `STATUSTEXT` all keep their rates.
  Steering behaves exactly as before. Record the measured outbound byte rate against the ≈510 B/s
  estimate.
- [ ] 7.9 Old plugin: no errors from the un-updated plugin with id 251 on the link.
