# Change: Publish the steering VESC's telemetry to the ground station as ESC_STATUS + ESC_INFO

## Why
The steering actuator is the only one on this vehicle driven by a *smart* driver: the Flipsky
75200 VESC answers every `COMM_GET_VALUES` poll with its input voltage, motor current, FET
temperature and a fault code (`src/VescProtocol.cpp:74-77`). All four already reach the web portal
(`src/TelemetryManager.cpp:54-57`) and the serial console — and **neither of those surfaces is
reachable by the operator while driving**. At the ground station, where the operator actually sits,
a VESC that is over-temperature, faulted, browning out on the 24 V boost rail, or simply silent on
its UART is *invisible*: the steering just stops responding and the only symptom is that the rover
does not turn.

MAVLink already has the message pair that names exactly these quantities — `ESC_STATUS` (291) for
the live electrical values and `ESC_INFO` (290) for temperature, health and failure flags — and
both are present in the dialect this firmware already includes
(`.pio/libdeps/esp32-s3-devkitc-1/c_library_v2/common/mavlink_msg_esc_status.h`,
`…/mavlink_msg_esc_info.h`). Using them keeps the project's standing rule intact: **every value
sits in the field that names it** (`src/MavlinkInterface.cpp:460-470`). Nothing has to be
repurposed, no `NAMED_VALUE_FLOAT` name can collide, and the four still-unused `EFI_STATUS` fields
stay reserved for the ECU PIDs they are named for.

## What Changes
- **Two new outbound messages from this component (system 1, component 25).** Nothing existing
  moves, changes field, or changes rate. `EFI_STATUS`, `VFR_HUD`, `VISION_POSITION_DELTA`,
  `HEARTBEAT` and `STATUSTEXT` are untouched.
- **`ESC_STATUS` (291), one slot, in the existing `MAVLINK_REPORT_TX_MS` tick (5 Hz)** — packed
  immediately after `EFI_STATUS` in the same `if (now - lastReportTx_ >= MAVLINK_REPORT_TX_MS)`
  block (`src/MavlinkInterface.cpp:499`):
  - `index = 0`, `time_usec` = 64-bit monotonic boot microseconds from `esp_timer_get_time()`
    (**never** `micros()` — see design.md);
  - `voltage[0]` ← VESC-measured **input voltage** (`v_in`), i.e. the 24 V boost rail that powers
    the steering VESC. This is **not** `EFI_STATUS.ignition_voltage`, which is the *ECU* module
    supply voltage (PID `0x42`) on the 12 V side; the two are different physical quantities on
    different rails and both are now reported, each in a field that names it;
  - `current[0]` ← average **motor** current (the stall/load diagnostic already decoded and already
    shown in the portal), **not** input current — see design.md for why one physical ESC can only
    report one of them honestly;
  - `rpm[0]` ← the **MEASURED steering position** in **centi-percent** of the calibrated
    lock-to-lock range: `SteeringController::getSteeringPercent()`
    (`src/SteeringController.cpp:363-376`, −100 = left limit … 0 = centre … +100 = right limit)
    × 100, rounded to `int32_t`, so the field spans **−10000 … +10000**. **Negative is left**
    (the "reverse rotation" sign the `rpm` field description itself assigns to negative values),
    **positive is right**. The VESC's own ERPM is still **not** decoded or forwarded: the Flipsky
    75200 runs brushed-DC with no motor sensor, so its ERPM is meaningless on this vehicle, while
    the AS5600 sits on the very shaft this ESC drives — see design.md for the permanent-absence
    justification;
  - `rpm[0] = INT32_MIN` when the position is **unknown** — the steering sensor is not OK
    (`SteeringController::isSensorOk()`) or the steering is not calibrated (`isCalibrated()`).
    `rpm[]` is `int32_t` and has no `NaN`, and **0 cannot mean "unknown" here, because 0 is exactly
    straight-ahead** — the same trap the existing `fuel_pressure` note records ("zero is a genuine
    zero trip distance", `src/MavlinkInterface.cpp:465,482-486`), resolved the same way: pick an
    encoding a real reading can never produce;
  - **`rpm[0]`'s validity is the AS5600's, not the VESC's.** It is independent of `isDriverOk()`,
    so `rpm[0]` can carry a live position while `voltage[0]`/`current[0]` are `NaN` (VESC
    unplugged, steering sensor fine) and equally `INT32_MIN` while voltage and current are live
    (VESC healthy, AS5600 uncalibrated). This mirrors `fuel_flow`, the one `EFI_STATUS` field whose
    `NaN` is sensor-driven rather than CAN-driven (`src/MavlinkInterface.cpp:488-489`);
  - **Consumer note:** the *commanded* steering already reaches the GCS as `SERVO_OUTPUT_RAW` from
    the autopilot's steering channel; `rpm[0]` adds the *measured* side, so command-vs-actual
    becomes visible for steering exactly as `throttle_out` vs `throttle_position` already makes it
    visible for throttle;
  - slots 1..3: `voltage`/`current` = `NaN`, `rpm` = 0, with `ESC_INFO.count = 1` telling every
    consumer to ignore them.
- **`ESC_INFO` (290), one slot, at its own 1 Hz cadence** (new `MAVLINK_ESC_INFO_TX_MS`, 1000 ms,
  in `include/Constants.h`) — the static/slow half of the pair:
  - `index = 0`, `count = 1`, `connection_type = ESC_CONNECTION_TYPE_SERIAL` (1);
  - `info` bit0 = `SteeringController::isDriverOk()` (the ESC is "online" only while a valid
    `COMM_GET_VALUES` reply arrived within `STEER_VESC_COMM_TIMEOUT_MS`);
  - `counter` = number of valid `GET_VALUES` replies received since boot (`uint16_t`, wraps —
    what matters is that it *advances*);
  - `temperature[0]` = FET temperature in **centi-degrees C** (`fetTempC × 100`, rounded, clamped
    to `int16_t`); `INT16_MAX` — the sentinel the message definition itself assigns to "not
    supplied" — while the driver is down; slots 1..3 always `INT16_MAX`;
  - `failure_flags[0]` = the VESC `mc_fault_code` mapped onto the `ESC_FAILURE_FLAGS` bitmask
    (table in design.md);
  - `error_count[0]` = number of **0 → non-zero fault transitions** since boot, counted in the
    driver, which is the only place that sees every reply.
- **Validity convention, identical to the existing `EFI_STATUS` one.** While
  `isDriverOk()` is false (VESC unplugged, unpowered, or silent past its comm timeout),
  `voltage[0]` and `current[0]` are `NaN` and `temperature[0]` is `INT16_MAX`, so the ground station
  shows "--" rather than the last number it happened to see. **Both messages keep being sent** in
  that state, with `info` bit0 clear — a consumer must be able to tell "the MAVLink link is up and
  the VESC is down" from "the whole peripheral is gone", and silence cannot express that.
  `rpm[0]` is the one field **not** gated by `isDriverOk()`: it has its own sensor and therefore
  its own gate (`isSensorOk() && isCalibrated()`) and its own sentinel (`INT32_MIN`).
- **New telemetry accessors, no new behaviour.** `IMotorDriver` gains `replyCount()` and
  `faultEventCount()` (default 0, like the other optional hooks), `VescMotorDriver` implements them
  by incrementing in `handlePacket()` (`src/VescMotorDriver.cpp:133-140`) — the single place a
  valid reply is accepted — and `SteeringController` forwards both. The raw fault code is already
  exposed as `getVescFault()`.
- **`MavlinkInterface::StateReport` gains the steering block** — the driver half (driver-ok flag,
  motor current, FET temperature, input voltage, raw fault code, reply counter, fault-event
  counter) **and the position half** (steering percent from `getSteeringPercent()`, plus the two
  validity flags `isSensorOk()` and `isCalibrated()` that gate it) — all filled in `src/main.cpp`
  from `vehicleController.getSteering()` beside the existing fields (`src/main.cpp:280-302`).
  `include/MavlinkInterface.h` stays free of MAVLink headers and of any `SteeringController`
  dependency: it receives decoded scalars, as it does for every other value.
- **Docs.** `MAVLINK_SETUP.md` gains a "Steering ESC telemetry" section: both messages field by
  field, the `rpm[0]` centi-percent scale and its `INT32_MIN` "unknown" sentinel (and that this is
  percent of the calibrated lock-to-lock range, **not** degrees), the `NaN` / `INT16_MAX`
  conventions, the fault-code mapping table, the "this is the 24 V rail, not the ECU's 12 V module
  voltage" note, and the plugin note below.
- **Not breaking for any existing consumer.** Two new message ids appear on a link that already
  carries five; nothing a consumer decodes today changes. The QuadBike Mission Planner plugin must
  **add packet subscriptions for ids 290 and 291** to display the new data — until it does, it
  simply does not show it. Mission Planner will **not** populate its own native `esc1_*` fields
  from component 25 (it fills those from the autopilot's own stream), so the plugin reads the raw
  packet, exactly as it already does for `EFI_STATUS`.
- **Not touched:** the steering control loop, the stall latch, the fault-code stop path, the VESC
  poll rate (`STEER_VESC_TELEM_MS`, 300 ms), the web telemetry JSON, `data/index.html`, and every
  existing MAVLink message.

## Impact
- Affected specs:
  - `mavlink-interface` — **ADDED** `Steering ESC Electrical Telemetry via ESC_STATUS`,
    **ADDED** `Steering ESC Health and Fault Reporting via ESC_INFO`, **ADDED**
    `Steering ESC Telemetry Validity Signalling`.
  - `vehicle-actuators` — **ADDED** `VESC Reply and Fault-Event Counters` (the two new driver-side
    counters and their accessors).
- Affected code: `include/Constants.h`, `include/IMotorDriver.h`, `include/VescMotorDriver.h` +
  `src/VescMotorDriver.cpp`, `include/SteeringController.h`, `include/MavlinkInterface.h` +
  `src/MavlinkInterface.cpp`, `src/main.cpp`, `MAVLINK_SETUP.md`.
- **No `data/` change** — this is firmware only, so `pio run -t upload` alone is enough; no
  `uploadfs` needed.
- Out of repo: the QuadBike Mission Planner plugin gains two read-only packet subscriptions
  (ids 290/291) and a steering-ESC readout. An un-updated plugin keeps working unchanged.
- Bandwidth: `ESC_STATUS` is 57 payload bytes at 5 Hz plus `ESC_INFO` at 46 bytes at 1 Hz ≈
  **380 B/s** including framing, about 3 % of the 11.5 kB/s the 115200 baud link carries. Measured
  on the bench (task 6.3), not assumed.

## Sequencing
This change is **ADDED-only in both capabilities** and therefore carries **no sequencing
constraint** against any active change. It may be archived at any point in the queue.

In particular it deliberately does **not** MODIFY `Vehicle State Reporting via Standard MAVLink
Messages`, even though that requirement enumerates the outbound message set. That requirement is
already carried as a MODIFIED superset by three active changes in a strict order
(`add-ecu-telemetry-pids` → `remap-efi-native-fields` → `add-odometer-and-trip`); adding a fourth
link would make this change's archive order load-bearing for no benefit, since nothing here changes
`EFI_STATUS`, its fields, its rate or its health policy. The precedent is `add-extnav-velocity`,
which introduced `VISION_POSITION_DELTA` — a whole new outbound message — as an ADDED requirement
touching nothing else. The ESC pair is orthogonal in exactly the same way: a different message, a
different source (the steering driver, not CAN), and a different validity gate
(`isDriverOk()`, not `canValid`).
