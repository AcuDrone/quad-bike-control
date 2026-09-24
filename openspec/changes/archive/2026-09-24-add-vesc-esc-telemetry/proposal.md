# Change: Publish the steering VESC's telemetry to the ground station as five NAMED_VALUE_FLOAT names

## Why
The steering actuator is the only one on this vehicle driven by a *smart* driver: the Flipsky
75200 VESC answers every `COMM_GET_VALUES` poll with its input voltage, motor current, FET
temperature and a fault code (`src/VescProtocol.cpp:74-77`). All four already reach the web portal
(`src/TelemetryManager.cpp:54-57`) and the serial console — and **neither of those surfaces is
reachable by the operator while driving**. At the ground station, where the operator actually sits,
a VESC that is over-temperature, browning out on the 24 V boost rail, or simply silent on its UART
is *invisible*: the steering just stops responding and the only symptom is that the rover does not
turn. The same is true of the AS5600's **measured** steering position: the *commanded* steering is
already on the link as `SERVO_OUTPUT_RAW`, but nothing carries the actual one, so command-vs-actual
— which exists for throttle — does not exist for steering.

The first implementation of this change (commit `89f84a6`) carried all of that in the two standard
ESC messages, `ESC_STATUS` (291) and `ESC_INFO` (290): semantically exact, already compiled into
the dialect, nothing repurposed but one field. **The bench proved that carrier dead.** On a VM
running Mission Planner 1.3.83, a 50-second capture logged **277 `EFI_STATUS` frames and zero ESC
frames**. Those two message ids are absent from the ArduPilot dialect MP generates its `MAVLink.dll`
from, so MP discards the frames **before** decoding them — below the level any plugin can reach. No
plugin version, and no plugin change, can recover a packet the host parser drops.

A message no consumer can decode carries no telemetry, however well its field names fit. So the
carrier changes to `NAMED_VALUE_FLOAT` (251) — a message every dialect and every tool knows — with
**exactly five names** from component 25.

## What Changes
- **The carrier.** The `ESC_STATUS` / `ESC_INFO` pair is removed entirely and replaced by five
  `NAMED_VALUE_FLOAT` (251) messages from system 1 / component 25. `EFI_STATUS`, `VFR_HUD`,
  `VISION_POSITION_DELTA`, `HEARTBEAT`, `STATUSTEXT` and the whole inbound path are untouched, as
  is every control path.

| `name` | Value | Unit | Rate | Unknown | Gate |
|--------|-------|------|------|---------|------|
| `STEER_POS` | MEASURED steering position | % of calibrated travel, −100 left … 0 centre … +100 right | 5 Hz | `NaN` | `steerSensorOk && steerCalibrated` |
| `STEER_A` | VESC average MOTOR current | A | 5 Hz | `NaN` | `steerDriverOk` |
| `VESC_V` | VESC-measured input voltage (24 V boost rail) | V | 5 Hz | `NaN` | `steerDriverOk` |
| `VESC_TEMP` | VESC FET temperature | °C | 1 Hz | `NaN` | `steerDriverOk` |
| `VESC_OK` | VESC link flag | 1.0 / 0.0 | 1 Hz | **never `NaN`** | *is* the flag |

- **A scoped exception to the single-message rule, stated as such.** The project's standing rule —
  every value in the field that names it, all of them in one `EFI_STATUS`, never a
  `NAMED_VALUE_FLOAT` — is kept for the ECU values, both gears, the digital flags, the odometer and
  the trip. These five names are the only `NAMED_VALUE_FLOAT` this component ever sends, and the
  exception is justified empirically (the standard carrier is undecodable at the GCS), not
  aesthetically.
- **Constants.** `MAVLINK_ESC_INFO_TX_MS`, `MAVLINK_ESC_INDEX` and `MAVLINK_ESC_COUNT` are replaced
  by a single `MAVLINK_STEER_SLOW_TX_MS` (1000).
- **`StateReport` shrinks.** `steerVescFault`, `steerReplyCount` and `steerFaultEvents` are removed;
  `steerDriverOk`, `steerMotorCurrentA`, `steerFetTempC`, `steerInputVoltageV` and the three
  steering-position fields stay.
- **The driver-side counters are reverted.** `IMotorDriver::replyCount()` /
  `faultEventCount()`, their `VescMotorDriver` members and overrides, and the two
  `SteeringController` forwarders are removed — nothing consumes them any more.
  `getVescFault()` stays: the web portal still reads it.
- **A zero-padding send helper.** `MavlinkInterface::sendNamedFloat()` zero-pads the name into a
  local 10-byte buffer before packing, because the library's pack helper copies **exactly 10 bytes**
  and a shorter literal would be read past its terminator. The five names are compile-time literals
  with a `static_assert` on each length; none is ever built at runtime.
- **Documentation.** `MAVLINK_SETUP.md` replaces its ESC section with the five-name contract,
  the rejected alternatives, the 10-byte `name` hazard and the plugin decode rules.

### Alternatives rejected
- **`ESC_STATUS` (291) + `ESC_INFO` (290)** — what this change replaces. Semantically exact, but
  absent from the dialect Mission Planner decodes with; measured at zero frames received.
- **`ESC_TELEMETRY_1_TO_4` (11030)** — present in the ArduPilot dialect, but its fields are
  **unsigned integers** (centivolts, centiamps, whole °C) with **no `NaN`**, so "unknown" would need
  in-band sentinels again; a signed steering position does not fit at all; and it is natively an
  autopilot-to-GCS message, so sending it from component 25 invites the "which ESC block is this?"
  confusion the plugin would then have to unpick.
- **More repurposed `EFI_STATUS` fields** — the fields still at `0.0` there name engine quantities
  this ECU could plausibly expose later, so they stay RESERVED and fail the *permanent-absence*
  test a repurposing must pass.
- **A Lua relay on the Pixhawk** re-emitting the same names with `gcs:send_named_float` — not
  needed, because the frames already reach the GCS from component 25, but it stays available as a
  future option precisely *because* the carrier is now a message every dialect knows. It would
  change nothing in this firmware.

## Impact
- Affected specs:
  - `mavlink-interface` — **ADDED** `Steering VESC Telemetry via NAMED_VALUE_FLOAT`.
  - `vehicle-actuators` — **no longer affected**: the delta that added the driver-side counters is
    withdrawn along with the counters themselves.
- Affected code: `include/Constants.h`, `include/IMotorDriver.h`, `include/VescMotorDriver.h` +
  `src/VescMotorDriver.cpp`, `include/SteeringController.h`, `include/MavlinkInterface.h` +
  `src/MavlinkInterface.cpp`, `src/main.cpp`, `MAVLINK_SETUP.md`.
- **Non-breaking.** Nothing an existing consumer decodes today changes field, message or rate. An
  un-updated plugin keeps working exactly as before and simply does not show the new data.
- **No `data/` change** — firmware only, so `pio run -t upload` alone is enough; no `uploadfs`.
- Out of repo: the QuadBike Mission Planner plugin gains **one** read-only packet subscription
  (id 251), dispatching on the trimmed `name` with `compid == 25`. That is a separate task, not
  part of this change.
- Bandwidth: 5 values × (18 payload + 12 framing) bytes at 3 × 5 Hz + 2 × 1 Hz = 17 frames/s ≈
  **510 B/s**, about 4 % of the 11.5 kB/s the 115200 baud link carries, and roughly **+110 B/s**
  over the ESC pair it replaces. To be confirmed on the bench (task 7.8), not assumed.

## Sequencing
This change is **ADDED-only** and therefore carries **no sequencing constraint** against any active
change. It may be archived at any point in the queue.

In particular it deliberately does **not** MODIFY `Vehicle State Reporting via Standard MAVLink
Messages`, even though that requirement enumerates the outbound message set. That requirement is
already carried as a MODIFIED superset by a strict chain of archived changes; adding a link would
make this change's archive order load-bearing for no benefit, since nothing here changes
`EFI_STATUS`, its fields, its rate or its health policy. The precedent is `add-extnav-velocity`,
which introduced `VISION_POSITION_DELTA` — a whole new outbound message — as an ADDED requirement
touching nothing else. These five names are orthogonal in exactly the same way: a different
message, a different source (the steering driver and the shaft sensor, not CAN), and different
validity gates (`isDriverOk()` and `isSensorOk() && isCalibrated()`, not `canValid`).
