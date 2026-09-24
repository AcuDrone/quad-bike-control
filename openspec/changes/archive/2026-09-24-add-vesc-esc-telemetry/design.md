## Context

The steering VESC already produces everything this change ships. `decodeGetValues()`
(`src/VescProtocol.cpp:56-79`) pulls four fields out of every `COMM_GET_VALUES` reply:

```cpp
out.fetTempC      = getInt16(payload, 1) / 10.0f;    // src/VescProtocol.cpp:74
out.motorCurrentA = getInt32(payload, 5) / 100.0f;   // :75  avg_motor_current
out.inputVoltageV = getInt16(payload, 27) / 10.0f;   // :76  v_in
out.faultCode     = payload[53];                     // :77
```

They are polled every `STEER_VESC_TELEM_MS` (300 ms), exposed as
`SteeringController::getFetTemp()/getMotorCurrent()/getInputVoltage()/getVescFault()`, and consumed
today by exactly one surface: the web portal (`src/TelemetryManager.cpp:54-57`). The measured
steering position comes from the AS5600 on the shaft the VESC drives
(`SteeringController::getSteeringPercent()`), with its own independent validity.

The **carrier** is what this revision decides, and it is decided by a measurement rather than by
taste. The first implementation (`89f84a6`) used `ESC_STATUS` (291) + `ESC_INFO` (290). On a VM
running Mission Planner 1.3.83, a 50-second capture from this firmware logged **277 `EFI_STATUS`
frames and zero ESC frames**. Both ids are absent from the ArduPilot dialect MP generates its
`MAVLink.dll` from, so MP drops the frames in its own parser, before any plugin subscription can
see them. The ESC code is therefore reachable only in git history.

Constraints that shaped what replaces it:

- **The consumer is the QuadBike MP plugin**, reading raw packets by subscription and filtering on
  `compid == 25`. It is not Mission Planner's native state, which MP fills from the autopilot
  (component 1) only.
- **`NAMED_VALUE_FLOAT` (251) is in every dialect and every tool**, including MP's own Inspector,
  `mavlink2rest`, `pymavlink` and ArduPilot's Lua bindings. That is precisely the property the ESC
  pair lacked.
- **The collision objection that rules `NAMED_VALUE_FLOAT` out for the ECU values still stands
  where it was raised.** All instances share one message id and differ only by a `name` string, so a
  **name-agnostic** store keeps only the last one received. That is why RPM, coolant, both gears,
  the flags, the odometer and the trip live in one `EFI_STATUS` and must stay there.
- **The `name` field is a fixed 10 bytes**, and the library's
  `mavlink_msg_named_value_float_pack()` copies exactly 10 bytes out of the pointer it is given via
  `mav_array_memcpy`. A shorter string literal would be **read past its terminator**.

## Goals / Non-Goals

- **Goals:** get the steering VESC's electrical and thermal state, and the measured steering
  position, in front of the operator at the GCS on a carrier that actually decodes there; keep
  "unknown" distinguishable from "zero"; keep "VESC down" distinguishable from "peripheral down".
- **Non-Goals:** no control behaviour changes; no new inbound message; no NVS key or runtime
  setting; no change to the web portal, the VESC poll rate, or any existing MAVLink message; no
  plugin work (that is a separate task).

## Decisions

### Exactly five names, and what was cut to get there

`STEER_POS`, `STEER_A`, `VESC_V`, `VESC_TEMP`, `VESC_OK`. Each name is ≤ 10 characters, each is
unique in the first 10 characters, and the set is closed — a sixth name is a deliberate decision,
not a drive-by addition.

Deliberately **not** on the link:

- **The raw `mc_fault_code`.** The previous revision mapped it onto `ESC_FAILURE_FLAGS`, a mapping
  that was lossy in a known direction (the bitmask has no under-voltage bit) and dependent on the
  VESC firmware's enum order. `NAMED_VALUE_FLOAT` carries a float, so a code would arrive as a bare
  number needing a lookup table at the far end, for a value the operator cannot act on in motion.
  It stays on the **web portal** (`steer_vesc_fault`) and the serial console, where a technician
  reads it. `VESC_OK` covers the case that matters while driving: is the driver answering at all.
- **The reply and fault-episode counters.** They existed only to populate `ESC_INFO.counter` /
  `error_count[0]`. With that message gone their only consumer is gone, so the driver-side
  counters are reverted rather than left as dead weight in the hot path. A frozen counter as a
  liveness signal is fully replaced by `VESC_OK`.
- **The VESC's average INPUT current.** Never decoded; motor current is the one that moves with
  steering effort.

### Rates: three fast, two slow

`STEER_POS`, `STEER_A` and `VESC_V` ride the existing `MAVLINK_REPORT_TX_MS` tick (5 Hz), packed
immediately after `VFR_HUD` and before the `VISION_POSITION_DELTA` block, so a GCS-side log lines
all the live values up without interpolation. `VESC_TEMP` and `VESC_OK` sit on their own
`MAVLINK_STEER_SLOW_TX_MS` (1 Hz) timer, the `lastHeartbeatTx_` pattern: a MOSFET's thermal time
constant is seconds and a link flag needs no faster. The underlying VESC data refreshes at 3.3 Hz
anyway, so the 5 Hz names repeat a sample roughly every third message.

### NaN gating, and one name that is never NaN

Two independent gates, kept as separate statements at the call sites so a reviewer sees they are
different sensors with different failure modes:

| Gate | Names |
|------|-------|
| `steerDriverOk` (a valid VESC reply within `STEER_VESC_COMM_TIMEOUT_MS`) | `STEER_A`, `VESC_V`, `VESC_TEMP` → `NaN`; `VESC_OK` → `0.0` |
| `steerSensorOk && steerCalibrated` (AS5600 + stored calibration) | `STEER_POS` → `NaN` |

`VESC_OK` is **never** `NaN`: it is the name that *tells* a consumer why the other three went
`NaN`, so an "unknown" there would defeat its only purpose. `0.0` in `STEER_POS` is a **real
reading** — wheels straight ahead — which is exactly why "unknown" there must be `NaN` and never
zero; `getSteeringPercent()` itself returns `0.0f` when uncalibrated, so it must be gated, not
passed through.

All five keep being sent while the VESC is silent. Suppressing them would make "the peripheral is
alive and its VESC is down" look identical to "the peripheral is gone". This is the deliberate
opposite of the `VISION_POSITION_DELTA` policy: that message is a **fusable** measurement feeding
the EKF, where silence is the only safe degradation; these are **display** values, where a
positively signalled "unknown" beats silence.

### `time_boot_ms` is the report tick's own `millis()`

The `report()` function already computes `now = millis()`; the helper takes it as a parameter. The
field is defined as `uint32_t` milliseconds since boot and therefore **wraps at 49.7 days** — that
is the field's definition, not a defect, and it must not be "fixed" by substituting a 64-bit clock.
`micros()` in particular is never used anywhere in this file: 32-bit microseconds wrap every
~71 minutes and would date a fresh sample as an ancient one (the lesson `VISION_POSITION_DELTA`
already records, where `esp_timer_get_time()` is used because that field is microseconds).

### The helper zero-pads, and the names are compile-time

```cpp
char name10[10] = {0};
strncpy(name10, name, sizeof(name10));
```

This is not an optimisation target and must not be simplified away: the pack helper's
`mav_array_memcpy` copies exactly 10 bytes, so handing it a 7-byte literal directly puts whatever
follows that literal in `.rodata` on the wire. A name of exactly 10 characters legally fills the
field with **no terminator**, which is why the consumer must trim on length, not on NUL.

The five names are `constexpr char[]` in an anonymous namespace with a `static_assert` on each
length (`sizeof(x) - 1 <= 10`). Truncation of an over-long name would otherwise be **silent**, and
names are never assembled at runtime, so the check belongs at compile time.

### Reverting the driver counters exactly

The counter hunks of `89f84a6` are reverse-applied, so
`git diff 89f84a6^ -- include/IMotorDriver.h include/VescMotorDriver.h src/VescMotorDriver.cpp
include/SteeringController.h` is empty. `IMotorDriver` goes back to its pre-change virtual set, so
`BTS7960Controller` is untouched in both directions. `getVescFault()` stays — the web portal reads
it.

## Risks / Trade-offs

- **Silent truncation of an over-long name** → `static_assert` on each literal plus the zero-pad
  buffer; names are never built at runtime.
- **Over-read on the 10-byte `memcpy`** → the local `char name10[10]`, documented at the copy site
  as load-bearing.
- **`time_boot_ms` wraps at 49.7 days** → accepted; it is the field's definition. A consumer that
  needs monotonicity across a wrap has the message arrival time.
- **Fault code and counters leave the MAVLink link** → a deliberate narrowing to five names. Both
  remain on the web portal and the console; `VESC_OK` covers the in-motion case.
- **Name-agnostic stores collapse the five into one row** → accepted: `mavlink2rest` and a generic
  logger would keep only the last name seen, but neither consumed this data, and the intended
  consumer dispatches on `(compid, name)`. MP's own Inspector shows one node per msgid that flickers
  between the names — expected, and called out in the bench steps.
- **Message id 251 is shared with anything else that sends it**, including a future Lua script on
  the autopilot → the `compid == 25` filter is mandatory for the consumer, and is stated as such in
  `MAVLINK_SETUP.md` and in the spec.

## Migration Plan

Firmware-only, single flash: `pio run -t upload`, no `uploadfs`. Nothing an existing consumer
decodes changes, so an un-updated plugin keeps working and simply shows no steering-VESC data.
Rollback is the previous firmware image; there is no persisted state to undo. The ESC pair is not
kept in parallel during any transition — it was never decodable at the GCS, so there is nothing to
transition from.

## Open Questions

- **A Lua relay on the Pixhawk.** `gcs:send_named_float` could re-emit the same five names from the
  autopilot's own component, which would put them in MP's native named-value plumbing. Not needed
  today — the frames already reach the GCS from component 25 — but the option exists only because
  the carrier is now a message every dialect knows. Decide only if a consumer appears that filters
  on component 1.
- **A sixth name.** Candidates if the bench asks for them: input current, the duty cycle, or a
  numeric fault code. Each would have to justify its bandwidth and a name unique in 10 characters;
  the set is closed until then.
