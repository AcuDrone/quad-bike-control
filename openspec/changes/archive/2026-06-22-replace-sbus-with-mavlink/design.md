## Context
The ESP32 today is an SBUS-Out slave: `SBusInput` (wrapping the Bolder Flight SBUS
library) decodes an inverted 100000-baud UART frame into 16 channels, and a thin mapping
layer turns channel microseconds into typed commands (steering %, throttle %, gear enum,
ignition enum, light bool). `VehicleController` arbitrates `SBUS > WEB > FAILSAFE` and
drives the actuators; engine RPM/temperatures arrive over CAN (`CANController::VehicleData`)
and are surfaced only to the web portal. The autopilot is blind to vehicle state.

This change swaps the **transport** (SBUS → MAVLink 2 over a normal UART) while preserving
the **mapping layer** and actuator coordination, and adds a **return telemetry path** so the
autopilot/GCS can see engine and vehicle state.

Key enabling fact: `SERVO_OUTPUT_RAW.servoN_raw` values are PWM microseconds in the same
880–2160 µs domain the SBUS path already produced. The existing per-function range constants
(gear/ignition/light thresholds, deadbands, center) and mapping math are transport-neutral and
can be reused verbatim; only the *source of the microseconds* changes.

## Goals / Non-Goals
- **Goals**
  - One bidirectional MAVLink 2 UART between Pixhawk TELEM2 and ESP32 (115200 baud).
  - Decode `SERVO_OUTPUT_RAW` into the existing typed command API with no change to the
    vehicle/actuator layer's command contract.
  - Link-loss fail-safe equivalent to the current SBUS timeout/fail-safe behavior.
  - Report engine RPM, temperatures, vehicle speed, throttle, and fuel back via standard
    MAVLink messages; report gear/ignition/fail-safe transitions via `STATUSTEXT`.
  - Remove the SBUS library and inverter dependency.
- **Non-Goals**
  - Parameter/mission/log protocols, TELEM1 companion link, direct RPM/temp sensors,
    separate starter channel, range re-tuning.

## Decisions

### D1 — MAVLink library
Use the official header-only **`mavlink/c_library_v2`** (ArduPilotMega dialect) added to
`lib_deps`. Header-only, no dynamic allocation, deterministic parse via
`mavlink_parse_char()`. Rationale: canonical, matches Pixhawk's dialect, avoids a heavier
wrapper. Remove `bolderflight/Bolder Flight Systems SBUS`.
- *Alternative considered*: a C++ wrapper (e.g. MAVLink for Arduino forks) — rejected as
  unnecessary abstraction over the generated headers (OpenSpec "simplicity first").

### D2 — UART / wiring
Dedicate a `HardwareSerial` (default UART1) to Pixhawk TELEM2: **non-inverted, 8N1, 115200,
RX + TX**. This differs from SBUS (inverted, 100000, 8E2, RX-only). The old SBUS RX pin
(`GPIO_NUM_8`) is reused for `MAVLINK_RX`. `MAVLINK_TX` is assigned to `GPIO_NUM_15`
(confirmed free; the ESP32-S3 GPIO matrix routes UART1 TX to any GPIO). No external inverter.
Note: `GPIO_NUM_9` is **not** available — despite a stale `GPIO_PINOUT_S3.md` row it is in
use as `PIN_TRANS_SERVO` (transmission servo, LEDC Ch2); the pinout doc is corrected as part
of this change.

### D3 — Command message + rate
Decode `SERVO_OUTPUT_RAW` (#36). On link establishment (first `HEARTBEAT` from the
autopilot's sysid/compid), the ESP32 sends `MAV_CMD_SET_MESSAGE_INTERVAL` (via
`COMMAND_LONG`) requesting `SERVO_OUTPUT_RAW` at 50 Hz (20000 µs interval). The decoder does
not *depend* on exactly 50 Hz — it consumes whatever arrives and applies the link-loss
timeout — but it requests 50 Hz so the autopilot streams at the design rate.
- Function→servo-output map mirrors the retired SBUS channel map: `STEERING=1`,
  `THROTTLE=2` (combined throttle/brake), `TRANSMISSION=3`, `IGNITION=4`, `FRONT_LIGHT=6`
  (channel 5 is intentionally unused), read from the matching `servoN_raw` fields. Captured
  as a `ServoChannelConfig` in `Constants.h`,
  replacing `SBusChannelConfig`.

### D4 — Link-loss / fail-safe semantics
SBUS gave us a per-frame fail-safe flag plus a 500 ms staleness timeout. MAVLink replaces
this with **two staleness checks**:
1. **Command staleness** — no `SERVO_OUTPUT_RAW` within `MAVLINK_CMD_TIMEOUT_MS`
   (default 500 ms, reusing the old value) ⇒ commands invalid.
2. **Heartbeat loss** — no autopilot `HEARTBEAT` within `MAVLINK_HEARTBEAT_TIMEOUT_MS`
   (default 3000 ms) ⇒ link down.
`isSignalValid()` returns true only when command data is fresh. Either timeout drives the
existing `VehicleController` fail-safe (center steering, idle throttle, NEUTRAL, 30 %
parking brake). This preserves the safety-critical 500 ms-to-fail-safe constraint.

### D5 — Return telemetry (standard messages)
Sourced from `CANController::VehicleData`, emitted on a fixed schedule from the main loop:
| Data | Message | Field(s) | Rate |
|------|---------|----------|------|
| Liveness + system status | `HEARTBEAT` | `type=MAV_TYPE_GROUND_ROVER`, `system_status` reflects fail-safe/healthy | 1 Hz |
| Engine RPM, coolant temp, throttle | `EFI_STATUS` | `rpm`, `cylinder_head_temperature` (coolant), `throttle_position`, `health` | 5 Hz |
| Current gear (live, graphable) | `NAMED_VALUE_FLOAT` | name `GEAR`, sequence-encoded `[R,N,H,L]=[-1,0,1,2]`, midpoint while moving | 5 Hz |
| Gear / ignition / fail-safe transitions | `STATUSTEXT` | human-readable, on-change only (rate-limited) | event |

`NAMED_VALUE_FLOAT` is used for gear because Mission Planner surfaces it as a live value in
the Status tab and graph dropdown (unlike `NAMED_VALUE_INT`, which MP does not display, and
unlike the PARAM list, which is for config not live values).

Gear is encoded by **physical sequence position** `[R, N, H, L] = [-1, 0, 1, 2]` (exact in a
32-bit float). While the servo is **actively moving** between two gears (`isGearChangeActive()`)
the value is their **midpoint** (e.g. N→H → `0.5`); when settled or dwelling at a gear it is
that gear's integer. Sourced from `getFromGear()`/`getTargetGear()` — the step's outgoing and
incoming gears, which the transmission advances one at a time through the sequence. Result: a
multi-step change (e.g. R→L) renders as an even **0.5 staircase**
`-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0`, each `.5` being the literal middle between the two
gears in transit.

This value is the controller's **assumed (commanded) gear**: the transmission is sensorless and
time-based (`use-time-based-gear-confirmation`), so `GEAR` reflects what the controller is
driving/assuming, not a measured position. `STATUSTEXT` is kept alongside for the
human-readable Messages log.

`EFI_STATUS` is the closest standard fit for engine RPM + temperature and is understood by
ArduPilot/Mission Planner. Gear and ignition state have no standard MAVLink representation,
so they are surfaced as `STATUSTEXT` on transition (the user chose standard messages over
`NAMED_VALUE_FLOAT`). ESP32 uses a fixed `system_id`/`component_id` distinct from the
autopilot's (e.g. `MAV_COMP_ID_USER1`).

**Not reported:** vehicle speed and oil temperature are not available from the ECU over CAN
(always read 0), so they are deliberately omitted rather than sent as misleading zeros.
`VFR_HUD` is therefore not emitted — and ArduPilot already provides its own `VFR_HUD`
groundspeed from GPS.

### D6 — Capability restructuring
The reusable, transport-neutral pieces that currently live under `sbus-input` (typed command
mapping methods, channel/range config, `RelayController`, signal-quality struct) migrate into
the new `mavlink-interface` capability. `sbus-input` is retired wholesale (all requirements
REMOVED with migration notes) so the spec set has no orphaned SBUS contract.

## Risks / Trade-offs
- **Loop-time budget**: MAVLink parsing + periodic send adds work to the ~loop. Mitigation:
  `mavlink_parse_char()` is O(1) per byte; sends are rate-limited (1–5 Hz); keep parsing
  non-blocking (drain RX each loop, no busy-wait). Verify control loop stays <10 ms.
- **Stream not delivered at 50 Hz**: autopilot may ignore the interval request. Mitigation:
  decoder is rate-agnostic; fail-safe covers under-delivery; log effective command rate.
- **Wrong servo-function mapping**: ArduPilot `SERVOn_FUNCTION` must align with the ESP32's
  `ServoChannelConfig`. Mitigation: document required ArduPilot servo assignments; expose
  decoded per-channel µs in web telemetry for verification.
- **sysid/compid addressing**: mismatched IDs => no `SET_MESSAGE_INTERVAL` honored.
  Mitigation: learn target IDs from the first received `HEARTBEAT`.

## Migration Plan
1. Land `MavlinkInterface` alongside `SBusInput` (both compile); no behavior change yet.
2. Switch `main.cpp`/`VehicleController`/`TelemetryManager` to `MavlinkInterface`; rename
   `InputSource::SBUS` → `MAVLINK`.
3. Remove `SBusInput` + SBUS lib + `SBusChannelConfig`; bench-test with a MAVLink stream
   (SITL or Mission Planner) before vehicle HIL.
4. Configure ArduPilot: TELEM2 `SERIAL_PROTOCOL=MAVLink2`, baud 115200, `SERVOn_FUNCTION`
   per the documented map.
- *Rollback*: revert the commit; SBUS path is unchanged in history. No persisted-state
  migration required (NVS keys untouched).

## Open Questions
- Whether oil temp maps better to `EFI_STATUS.intake_manifold_temperature` or a second
  message — confirm against what Mission Planner surfaces usefully on a rover.
