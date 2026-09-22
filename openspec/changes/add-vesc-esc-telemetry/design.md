## Context

The steering VESC already produces everything this change ships. `decodeGetValues()`
(`src/VescProtocol.cpp:56-79`) pulls four fields out of every `COMM_GET_VALUES` reply:

```cpp
out.fetTempC      = getInt16(payload, 1) / 10.0f;    // src/VescProtocol.cpp:74
out.motorCurrentA = getInt32(payload, 5) / 100.0f;   // :75  avg_motor_current
out.inputVoltageV = getInt16(payload, 27) / 10.0f;   // :76  v_in
out.faultCode     = payload[53];                     // :77
```

They are polled every `STEER_VESC_TELEM_MS` (300 ms, `include/Constants.h:337`), exposed as
`SteeringController::getFetTemp()/getMotorCurrent()/getInputVoltage()/getVescFault()`
(`include/SteeringController.h:105-113`), and consumed today by exactly one surface: the web
portal (`src/TelemetryManager.cpp:54-57`). The ground station gets none of it.

Constraints that shaped the design:

- **The field must name the value.** `src/MavlinkInterface.cpp:460-470` records the rule the
  `EFI_STATUS` mapping is built on, and `add-ecu-telemetry-pids` / `remap-efi-native-fields`
  tightened it into spec text: a field may be repurposed *only* where the quantity it names is
  permanently unavailable on this vehicle. The four `EFI_STATUS` fields still at `0.0f`
  (`spark_dwell_time`, `ignition_timing`, `injection_time`, `exhaust_gas_temperature`) name engine
  quantities this ECU could plausibly expose later, so they are reserved, not free.
- **`NAMED_VALUE_FLOAT` is not an option.** It was rejected for the ECU values because every
  instance shares message id 251 and collides by name in any name-agnostic store — the reasoning
  already in the spec and in the `EFI_STATUS` comment block. Nothing about a VESC changes that.
- **`ESC_STATUS`/`ESC_INFO` are semantically exact and already compiled in.** Both headers ship in
  the dialect this firmware includes (`<ardupilotmega/mavlink.h>` pulls in `common/`):
  `.pio/libdeps/esp32-s3-devkitc-1/c_library_v2/common/mavlink_msg_esc_status.h` (291) and
  `…/mavlink_msg_esc_info.h` (290). No new dependency, no dialect change, no custom XML.
- **This is a peripheral, not the vehicle.** System id 1, component 25
  (`include/Constants.h:251-252`). ArduPilot forwards a target-less message from a serial
  peripheral to every other link, so the packet reaches the GCS the same way `EFI_STATUS` already
  does — by the plugin's own packet subscription, not through Mission Planner's native `esc1_*`
  state (which MP fills from the autopilot's own stream and will not populate from component 25).
- **The 24 V boost rail is a known weak point.** `RAIL_24V_LOW_THRESHOLD` is flagged PROVISIONAL in
  `include/Constants.h:180-184`, and the low-rail warning is warning-only. The rail feeds the
  steering VESC, the Jetson, Starlink and the cameras. A second, independent measurement of that
  rail — taken by the VESC itself, at the load, rather than by the ESP32's divider on GPIO 9 — is
  worth having at the ground station.

## Goals / Non-Goals

- Goals:
  - Put the steering driver's voltage, current, temperature, online state and fault on the GCS
    link, each in the MAVLink field that names it.
  - Make "VESC silent/unpowered" visibly different from "VESC healthy and idle", and both
    different from "the peripheral is gone".
  - Put the **measured** steering position on the link beside the steering driver's electrical
    state, so command-vs-actual is visible for steering the way it already is for throttle — with
    "position unknown" signalled positively rather than as a plausible-looking centre.
  - Cost nothing that existing consumers can notice: no field moves, no rate changes, no breaking
    decode.
  - Keep `include/MavlinkInterface.h` free of MAVLink headers and of any actuator dependency, as
    every other value already is.
- Non-Goals:
  - Reporting the *brake* BTS7960 as an ESC. It has no telemetry at all (`IMotorDriver`'s defaults
    return 0) and a slot full of zeros is worse than no slot.
  - Using ESC telemetry for control. Over-current protection lives in the VESC's own limits and
    jam detection in the AS5600 stall detector (`remove-steer-overcurrent-timer` settles this);
    this change is strictly a reporting path and adds no new stop, latch or interlock.
  - Decoding more of the `GET_VALUES` payload than the reporting needs.
  - Reporting the steering position in **degrees**. `rpm[0]` is percent of the *calibrated
    lock-to-lock range*; the firmware has no counts-to-degrees calibration anywhere (the AS5600's
    0..4095 counts are only ever normalised against the stored centre and limits), so degrees would
    need a new calibration step — a separate change, not a rider on this one.
  - Sending ESC telemetry over the web portal in a new form — it is already there.
  - Any inbound ESC command, calibration or configuration path.

## Decisions

### Decision: `ESC_STATUS` + `ESC_INFO` rather than repurposed `EFI_STATUS` fields

`EFI_STATUS` has four fields left at `0.0f`, and fitting the VESC into them is technically
possible. It is rejected because each of those fields names an engine quantity (dwell, timing,
injection time, EGT) that this ECU *might* expose later — the permanent-absence test that
`remap-efi-native-fields` requires before a repurposing is allowed. There is no version of
"`exhaust_gas_temperature` carries the steering MOSFET temperature" that a consumer reading MAVLink
field names could ever guess.

`ESC_STATUS`/`ESC_INFO` instead give five exactly-named fields (voltage, current, rpm,
temperature, failure_flags) plus the online bitmap and the packet counter, for one new pair of
message ids and ~380 B/s.

*Alternatives considered:* `NAMED_VALUE_FLOAT` (id collisions — already rejected project-wide);
`BATTERY_STATUS` (names a battery; the 24 V rail is a boost converter output, and the current is
the *motor's*, not a pack current — it would misname both values); a custom dialect message (a
dialect fork for one peripheral, and every generic tool would show it as unknown); `DEBUG_VECT`
(three floats, no names, no health).

### Decision: one physical ESC = one slot, and the slot's `current` is **motor** current

The same `GET_VALUES` payload carries two currents (`src/VescProtocol.cpp:60-65`):
`avg_motor_current` at offset 5 (decoded today) and `avg_input_current` at offset 9 (not decoded).
They answer different questions — motor current is the load/stall diagnostic for the steering
motor; input current is what the 24 V rail is being asked to supply. `ESC_STATUS` has exactly one
`current[]` entry per ESC, and this is one physical ESC, so only one of them can be reported
honestly. **Motor current wins**: it is the value already decoded, already in the portal, already
the thing the operator wants when steering feels wrong, and the one that moves with steering
effort.

Input current is *not* forgotten — decoding it later is a single line,
`getInt32(payload, 9) / 100.0f`, at an offset that is already documented in the comment block, and
its natural home would be an additional readout rather than a second ESC slot. Faking a second ESC
slot to carry it would tell every consumer this vehicle has two ESCs, which is a lie the `count`
field exists to prevent.

### Decision: `rpm[0]` carries the measured steering position, and the VESC's ERPM is not decoded

The VESC runs in **brushed-DC mode** with no motor position sensor, so its `rpm` field (offset 23,
a 4-byte ERPM) is derived from a commutation estimate that does not exist for a brushed motor.
That ERPM is therefore **not decoded and not forwarded**: forwarding it would be strictly worse
than any alternative, because a meaningless number *looks* like data.

That leaves the slot free, and it passes the same **permanent-absence test** the `EFI_STATUS`
repurposings already have to pass (`remap-efi-native-fields`; `barometric_pressure` and
`fuel_pressure` in `src/MavlinkInterface.cpp:482-486`): the quantity the field names — this ESC's
shaft speed — cannot be measured on this vehicle, because the motor is brushed and unsensored.
So `rpm[0]` carries the **measured steering position** instead:

```
rpm[0] = lroundf(SteeringController::getSteeringPercent() * 100.0f)   // −10000 … +10000
```

- **Units: centi-percent of the calibrated lock-to-lock range.** `getSteeringPercent()`
  (`src/SteeringController.cpp:363-376`) returns −100 … +100 relative to the stored AS5600
  centre / left-limit / right-limit calibration, with the two halves scaled independently because
  the travel is asymmetric (`relLeft_ < 0 < relRight_`, `include/SteeringController.h:130-131`).
  ×100 uses the int32 range without inventing precision.
- **Sign: negative = LEFT, positive = RIGHT.** This is the accessor's own convention
  (`include/SteeringController.h:51-52`: "−100 (left limit) .. 0 (center) .. +100 (right limit)"),
  and it lines up with the `rpm` field description, where a negative value denotes reverse
  rotation.
- **Why this field rather than another.** The slot describes *the drive this ESC is*, and the
  AS5600 sits on the **steering shaft** — the position of the very axis this ESC drives, measured
  downstream of its gearbox. That is a far more cohesive occupant than any free `EFI_STATUS` field
  could be, and it is the one number an operator wants beside the steering driver's voltage,
  current and temperature.

**Sentinel: `INT32_MIN` means "position unknown"** — `!isSensorOk()` (AS5600 not answering) or
`!isCalibrated()` (no stored centre/limits). `rpm[]` is `int32_t`: it has no `NaN`, so an
in-band sentinel is unavoidable, and **0 cannot be it, because 0 is exactly straight-ahead**.
`getSteeringPercent()` itself returns `0.0f` when uncalibrated, so passing it through unguarded
would report "wheels dead centre" for a vehicle whose steering position is entirely unknown. This
is the same shape of problem the `fuel_pressure` note already records in the opposite direction
(there the definition's "0 = unknown" is deliberately *not* honoured, because 0 is a genuine zero
trip distance); both are resolved by the same rule — the encoding must be one a real reading can
never produce, and `INT32_MIN` is unreachable from a ±10000 range.

**Validity is the AS5600's, not the VESC's.** `rpm[0]` is gated by `isSensorOk() && isCalibrated()`
and **not** by `isDriverOk()`, so the four combinations are all meaningful and all reachable:
a live position beside `NaN` voltage/current (VESC unplugged, steering sensor fine), a live
voltage/current beside `INT32_MIN` (VESC healthy, steering uncalibrated), both live, both unknown.
This mirrors `fuel_flow`, the one `EFI_STATUS` field whose `NaN` is sensor-driven rather than
CAN-driven (`src/MavlinkInterface.cpp:488-489`), and the consumer note is the same: "everything
unknown means one subsystem is down" does not hold in reverse.

**Consumer note.** The *commanded* steering already reaches the GCS as `SERVO_OUTPUT_RAW` from the
autopilot's steering channel; `rpm[0]` adds the *measured* side. Command-vs-actual for steering
then reads exactly like `throttle_out` vs `throttle_position` does for throttle: a transient
disagreement is the actuator slewing, a persistent one is a steering fault.

*Alternatives considered:*

- **A constant `0`** (the previous decision here) — rejected: it spends a perfectly-shaped int32
  slot on "no data" while the most useful steering number on the vehicle has no MAVLink home, and
  it reads as "shaft stopped" to any consumer that does not read the documentation.
- **`EFI_STATUS.ecu_index`** — rejected twice over: it would put a steering quantity inside an
  *engine* message, and `EFI_STATUS`'s requirement is already carried as a MODIFIED superset by
  three active changes in a strict archive order (`add-ecu-telemetry-pids` →
  `remap-efi-native-fields` → `add-odometer-and-trip`), so touching it would make this change's
  position in that queue load-bearing for no benefit. Keeping the steering value in `ESC_STATUS`
  is what lets this change stay ADDED-only (see proposal.md → Sequencing).
- **`ACTUATOR_OUTPUT_STATUS` (375)** — a genuine fit by name, but it is a *command/output* message
  keyed by actuator index with no validity encoding of its own, so "unknown" would again have to be
  a magic float, and it would add a third message id and a third cadence to carry one number that
  fits an existing slot in a message this change already sends.
- **A new `NAMED_VALUE_FLOAT`** — rejected project-wide for id collisions, as above.

### Decision: `NaN` / `INT16_MAX` while the driver is down, and keep sending

`driverOk()` (`src/VescMotorDriver.cpp:142-145`) is true only while a valid `GET_VALUES` reply
arrived within `STEER_VESC_COMM_TIMEOUT_MS` (1000 ms). It is false from boot until the first reply,
and goes false on an unplugged UART, an unpowered VESC or a dead rail. In that state `values_`
still holds whatever the last good reply contained — which is exactly the stale-number trap the
`EFI_STATUS` CAN-gating already avoids.

So, while `!driverOk()`:

| Field | Value | Why |
|---|---|---|
| `ESC_STATUS.voltage[0]`, `current[0]` | `NaN` | float fields; matches the existing CAN-gated convention, GCS renders "--" |
| `ESC_INFO.temperature[0]` | `INT16_MAX` | the sentinel the message definition itself assigns to "data not supplied by ESC" |
| `ESC_INFO.info` bit0 | 0 | the message's own offline encoding |
| `ESC_INFO.failure_flags[0]` | `0` | a fault code read from a dead link is not a fault observation |
| `ESC_INFO.counter`, `error_count[0]` | unchanged, still sent | they are cumulative history, not a live reading; a frozen counter is itself the diagnostic |
| `ESC_STATUS.rpm[0]` | **unaffected** | its gate is the AS5600's (`isSensorOk() && isCalibrated()`), not the VESC's; a dead VESC does not blind the shaft sensor |

**Both messages keep being sent while the VESC is down.** Suppression would make "VESC down" and
"ESP32 down / link down" look identical at the GCS, and those need different responses from the
operator: one is a steering fault on a live vehicle, the other is a comms failure. A sent message
carrying `NaN` with `info` bit0 clear says precisely "I am alive, my ESC is not". This is the
opposite of the `VISION_POSITION_DELTA` policy, and deliberately so — that message is a *fusable
measurement* feeding the EKF, where silence is the only safe degradation; these two are *display*
values, where a positively-signalled "unknown" beats silence.

### Decision: `ESC_STATUS` on the 5 Hz report tick, `ESC_INFO` on its own 1 Hz timer

`ESC_STATUS` is packed inside the existing `if (now - lastReportTx_ >= MAVLINK_REPORT_TX_MS)` block
(`src/MavlinkInterface.cpp:499-500`), beside `EFI_STATUS` and `VFR_HUD`. It costs no new timer,
no new drift, and it keeps every live value on the link in the same tick, so a GCS-side log lines
them up without interpolation.

`ESC_INFO` gets its own `lastEscInfoTx_` / `MAVLINK_ESC_INFO_TX_MS` (1000 ms) pair, following the
`lastHeartbeatTx_` / `MAVLINK_HEARTBEAT_TX_MS` pattern (`src/MavlinkInterface.cpp:434-435`). Its
payload is static-ish (type, count, temperature, flags) and its 46 bytes at 5 Hz would be four
fifths waste. 1 Hz is also fast enough for the values it carries: the VESC is polled at 3.3 Hz, and
a MOSFET's thermal time constant is seconds.

Note that the underlying data refreshes at **3.3 Hz** (`STEER_VESC_TELEM_MS` = 300 ms), so a 5 Hz
`ESC_STATUS` repeats a sample roughly every third message. That is accepted rather than fixed:
aligning the transmit to the poll would need a new timestamp path for no gain, and a consumer that
cares can watch `ESC_INFO.counter` advance.

### Decision: `time_usec` from `esp_timer_get_time()`, never `micros()`

`ESC_STATUS.time_usec` is a `uint64_t` "time since boot or UNIX epoch". `micros()` is a 32-bit
count that wraps every ~71 minutes; widening it to 64 bits preserves the wrap and would present a
fresh sample as an ancient one. `src/MavlinkInterface.cpp:695` already carries this exact lesson
for `VISION_POSITION_DELTA` ("NEVER micros()"), and `esp_timer.h` is already included at
`src/MavlinkInterface.cpp:7`. The same monotonic clock is used here, so both messages share a
timebase.

### Decision: map the VESC fault code onto `ESC_FAILURE_FLAGS`, and also keep the raw code visible

`mc_fault_code` is a small enum; `ESC_FAILURE_FLAGS`
(`.pio/libdeps/esp32-s3-devkitc-1/c_library_v2/common/common.h:448-458`) is a bitmask. The mapping:

| VESC `mc_fault_code` | value | `ESC_FAILURE_FLAGS` | value | Note |
|---|---|---|---|---|
| `FAULT_CODE_NONE` | 0 | — | 0 | no failure |
| `FAULT_CODE_OVER_VOLTAGE` | 1 | `ESC_FAILURE_OVER_VOLTAGE` | 2 | exact |
| `FAULT_CODE_UNDER_VOLTAGE` | 2 | `ESC_FAILURE_OVER_VOLTAGE` | 2 | **lossy** — see below |
| `FAULT_CODE_DRV` | 3 | `ESC_FAILURE_GENERIC` | 64 | gate-driver fault has no MAVLink equivalent |
| `FAULT_CODE_ABS_OVER_CURRENT` | 4 | `ESC_FAILURE_OVER_CURRENT` | 1 | exact |
| `FAULT_CODE_OVER_TEMP_FET` | 5 | `ESC_FAILURE_OVER_TEMPERATURE` | 4 | exact |
| `FAULT_CODE_OVER_TEMP_MOTOR` | 6 | `ESC_FAILURE_OVER_TEMPERATURE` | 4 | exact |
| any other non-zero | ≥7 | `ESC_FAILURE_GENERIC` | 64 | forward-compatible catch-all |

Two things about this table are load-bearing:

1. **`ESC_FAILURE_FLAGS` has no under-voltage bit.** The enum stops at `ESC_FAILURE_GENERIC` and
   offers only `OVER_VOLTAGE`. Mapping `FAULT_CODE_UNDER_VOLTAGE` to `OVER_VOLTAGE` is
   deliberately *wrong in direction but right in category*: it says "a voltage fault occurred",
   which beats `GENERIC` for triage, and the brownout case is the one this vehicle is most likely
   to hit (a sagging 24 V boost rail under Jetson + Starlink load). Because the direction is
   ambiguous, the **raw** fault code must stay reachable — see decision below.
2. **The numeric values are VESC-firmware-version dependent.** They follow the canonical bldc
   `datatypes.h` `mc_fault_code` declaration order (`NONE, OVER_VOLTAGE, UNDER_VOLTAGE, DRV,
   ABS_OVER_CURRENT, OVER_TEMP_FET, OVER_TEMP_MOTOR, …`), the same assumption
   `include/VescProtocol.h:60-70` already documents for the payload offsets, with the same
   mitigation: **bench-verify against VESC Tool for the flashed firmware** (task 6.8). Newer
   firmware only appends codes, so the ones mapped here are stable, but the catch-all exists
   precisely so an unknown code degrades to "generic failure" rather than to silence.

The mapping is a small `switch` in `MavlinkInterface`, close to the message it feeds, taking the
decoded `uint8_t` — the header stays free of both MAVLink and VESC types.

### Decision: where the raw fault code is exposed (and what is left open)

The raw `mc_fault_code` is the value a technician actually wants, and the lossy mapping above is
the reason. Options considered:

- Fold it into `failure_flags[0]`'s high bits — **rejected**: it corrupts a defined bitmask, and
  any generic tool would decode nonsense flags.
- Add it to `ESC_INFO.error_count[0]` — **rejected**: that field has a defined meaning (errors
  since boot) which this change uses as intended.
- Repurpose a free `EFI_STATUS` field — **rejected** by the permanent-absence rule above.
- A `STATUSTEXT` on every fault transition — **viable, not taken here**: it needs a debounce policy
  of its own (a flapping fault at 3.3 Hz would flood the rate limiter), which is a design question
  worth its own change rather than a rider on this one.

So the raw code is a **Non-Goal for MAVLink in this change**. It remains visible on the web portal
(`steer_vesc_fault`, already there) and on the serial console, and `error_count[0]` still tells the
GCS *that* faults happened and how many. Listed as an open question below.

### Decision: the two counters live in the driver, not in `MavlinkInterface`

`VescMotorDriver::handlePacket()` (`src/VescMotorDriver.cpp:133-140`) is the single place a valid
reply is accepted:

```cpp
if (VescProtocol::decodeGetValues(payload, len, v)) {
    values_ = v;
    haveReply_ = true;
    lastValidReplyTime_ = millis();
}
```

Both counters belong exactly there. `replyCount_` increments on every accepted reply — a transport
counter, which is what `ESC_INFO.counter` is defined to be. `faultEventCount_` increments when the
newly decoded `faultCode` is non-zero **and the previous one was zero**, which requires seeing the
previous value — something only this function has. Counting in `MavlinkInterface` instead would
sample at 5 Hz a value that changes at 3.3 Hz: a fault that raises and clears between two report
ticks would be missed entirely, and one that persists could be double-counted across a
transmit-timer coincidence. Both are added as `IMotorDriver` hooks with `0` defaults, matching the
existing optional-telemetry pattern (`include/IMotorDriver.h:38-52`), so the BTS7960 brake driver
needs no change.

### Decision: `ESC_STATUS.voltage[0]` is the 24 V rail, and that is not a duplicate

Two independent measurements of the same rail now exist: `VehicleController::getRail24V()` (ESP32
ADC on GPIO 9 through a 200K/24K divider, `include/Constants.h:75,179`) and the VESC's own `v_in`.
They are not redundant — the ADC measures at the board, the VESC measures at the load, and a
divergence between them *is* the diagnostic for a sagging rail under current. Only the VESC's
reading goes into `ESC_STATUS`, because that is the field's defined meaning ("voltage measured from
each ESC"); the board-side reading keeps its existing web-telemetry path.

## Risks / Trade-offs

- **The fault-code numbers could be wrong for the flashed firmware.** → Bench-verify with VESC Tool
  (task 6.8); the catch-all maps anything unexpected to `ESC_FAILURE_GENERIC`, so the worst case is
  a correctly-flagged fault with the wrong category, never a missed fault.
- **A GCS might render the ESC pair as a *propulsion* ESC.** Mission Planner and most log tools
  assume ESC index 0 is motor 1. Here it is a steering actuator on a ground rover. → `ESC_INFO`
  carries `count = 1` and `connection_type = SERIAL`, and `MAVLINK_SETUP.md` states plainly that
  index 0 is the steering ESC. The plugin, the intended consumer, labels it.
- **Link budget.** ≈380 B/s added to a 115200 link already carrying `SERVO_OUTPUT_RAW` inbound at
  25 Hz. Measured, not assumed, in task 6.3; if the command rate degrades, `ESC_STATUS` drops to
  the `ESC_INFO` cadence with a one-line change.
- **`NaN` in a float array.** A consumer doing `sum(current)` over all four slots without checking
  `count` gets `NaN`. That is the standard MAVLink hazard for "unknown" floats and the same one
  `EFI_STATUS` already presents; documented in `MAVLINK_SETUP.md`.
- **Counter wrap.** `ESC_INFO.counter` is `uint16_t` and wraps every ~5.5 hours at 3.3 Hz. Only its
  advance is meaningful; documented.
- **`rpm[0]`'s permanent-absence justification is NOT actually permanent.** This is the one place
  this change is weaker than the `barometric_pressure` precedent, and it is recorded here rather
  than glossed: that ECU physically lacks a barometric sensor forever, whereas *this* ESC's shaft
  speed becomes measurable the day the VESC gains an encoder or the steering motor is replaced with
  a BLDC. If that ever happens, `rpm[0]` must revert to the real ERPM the field names and the
  steering position must move elsewhere (`ACTUATOR_OUTPUT_STATUS` is the fallback) — a breaking
  change for the plugin, in lockstep with the flash, exactly like the `remap-efi-native-fields`
  precedent. → Mitigation: the constraint is stated in `MAVLINK_SETUP.md` beside the field, so the
  hardware change cannot silently invalidate the encoding, and `ESC_INFO.count`/`connection_type`
  already mark this slot as a non-propulsion ESC that a generic tool should not interpret.
- **A generic tool will render `rpm[0]` as an RPM.** A log viewer that plots `ESC_STATUS.rpm` sees
  a number swinging to ±10000 and labels it "RPM". → The intended consumer is the QuadBike plugin,
  which labels it; `MAVLINK_SETUP.md` states the unit and the sentinel; and `INT32_MIN` is
  conspicuous enough in a plot to prompt a look at the documentation rather than a wrong reading.
- **`INT32_MIN` in an integer array.** A consumer that averages or plots `rpm[]` without checking
  for the sentinel gets a wild outlier rather than a `NaN` gap. That is the cost of an integer
  field having no not-a-number; documented alongside the `NaN` hazard in `MAVLINK_SETUP.md`.

## Migration Plan

Firmware only — no `data/` change, so `pio run -t upload` alone. The change is purely additive on
the wire: an un-updated Mission Planner plugin ignores two unknown message ids and behaves exactly
as before, so firmware and plugin can be deployed in either order. Rollback is a reflash; there is
no persisted state, no NVS key and no parameter to undo.

## Open Questions

- **Should the raw `mc_fault_code` reach the GCS at all, and how?** A rate-limited `STATUSTEXT` on
  each 0 → non-zero transition is the obvious candidate (it would carry the code *and* a human
  string), but it needs its own debounce policy. Deferred out of this change.
- **Is `avg_input_current` worth decoding?** It is one line and one offset, and it answers the rail
  budget question that `voltage[0]` only half answers. It has no home in `ESC_STATUS` without
  inventing a second ESC, so it would be a web-telemetry addition — a separate, small change.
- **Should the brake BTS7960 ever appear as ESC index 1?** Only if it gains real telemetry
  (it has none today). If it ever does, `count` becomes 2 and the slots are already there.
- **Is 1 Hz right for `ESC_INFO` on this link?** Chosen by analogy with `HEARTBEAT`; the bench
  bandwidth measurement (task 6.3) is what would justify changing it.
- **Should the steering position ever be reported in degrees?** It would need a counts-to-degrees
  calibration the firmware does not have — a new commissioning step measuring actual wheel angle at
  the two locks, stored in NVS beside the existing centre/limit calibration. Worth doing only if a
  consumer needs an absolute angle rather than "how far toward the lock"; deferred as a Non-Goal
  above, and it would change only the *scale* of `rpm[0]`, not its home.
- **Does `rpm[0]` need its own rate?** It is the only field in `ESC_STATUS` that moves fast (the
  steering can traverse lock to lock in a second or two), and it is sampled at the 5 Hz report
  tick while the AS5600 is read far more often. 5 Hz is enough to *see* the steering follow its
  command; it is not enough to characterise the actuator's step response. If that is ever wanted,
  it is a separate high-rate path, not a faster `ESC_STATUS`.
