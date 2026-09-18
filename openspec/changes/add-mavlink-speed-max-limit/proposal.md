# Change: Take the speed-limiter ceiling from the autopilot's `SPEED_MAX` parameter

## Why
The maximum-speed throttle limiter added by `add-hall-speed-sensor` can only be configured from
the ESP32's own web UI (`speed_limit_set`, persisted to NVS). The operator's actual cockpit is
Mission Planner / MAVProxy on the Pixhawk, where a speed ceiling already exists as the ArduPilot
parameter **`SPEED_MAX`** (m/s). Today the two numbers have nothing to do with each other: an
operator who lowers `SPEED_MAX` for a confined area gets a slower *autopilot* but an unchanged
*firmware* limiter, and the firmware ceiling can only be moved by opening a second, WiFi-only
interface on a vehicle that is being driven.

MAVLink can carry this. ArduPilot answers `PARAM_REQUEST_READ` with `PARAM_VALUE`, and a GCS can
change `SPEED_MAX` live with `PARAM_SET` — no reboot. The firmware currently has **zero** PARAM
handling, so nothing of the autopilot's configuration is visible to it.

Second problem, uncovered while specifying the first: the existing limiter is a **20 % step**.
`applySpeedLimit()` passes the demand through untouched below the limit and clamps it to
`SPEED_LIMIT_THROTTLE_CAP_PCT` the instant the limit is crossed. On a vehicle this heavy that is a
lurch, it invites hunting around the threshold, and it is at its worst exactly where it is most
dangerous — mid-corner. The limiter is also evaluated **once per web command** rather than every
loop, so a vehicle that accelerates past the limit on a stale 80 % web throttle command is never
clamped at all until the operator moves the slider.

## What Changes
- **New inbound MAVLink parameter subscription.** `MavlinkInterface` gains a `PARAM_VALUE` case,
  a `PARAM_REQUEST_READ` poll for `SPEED_MAX` every `MAVLINK_PARAM_POLL_MS` (5 s, first request
  `MAVLINK_PARAM_FIRST_DELAY_MS` after the autopilot is learned), and accessors
  `hasSpeedMaxParam()` / `getSpeedMaxKmh()` / `getSpeedMaxRawMs()` / `getSpeedMaxAgeMs()`.
  Both paths are implemented on purpose: ArduPilot's broadcast-on-`PARAM_SET` behaviour is
  version- and routing-dependent, so the poll — not the broadcast — is the change detector, and an
  unsolicited `PARAM_VALUE` merely makes the update arrive sooner.
- **Value hygiene at the transport.** `param_id` is a 16-byte, *not* NUL-terminated field and is
  copied into a 17-byte buffer before comparison. NaN, infinity, negative values and values above
  `MAVLINK_PARAM_SPEED_MAX_MS` (30 m/s, ArduPilot's own range bound) are rejected with a log and
  never stored. Only the learned autopilot's sysid/compid is accepted, so a GCS on the same wire
  (sysid 255) cannot move the vehicle's speed ceiling.
- **Ceiling-source arbitration in `VehicleController`.** The web `speed_limit_enable` toggle stays
  the **master switch** — MAVLink supplies the *value*, never the decision to limit. With the
  limiter enabled, a fresh `SPEED_MAX` wins; otherwise the stored local value is used. The MAVLink
  value is **RAM-only**: `SpeedSensor::setLimitMaxKmh()` is never called from this path, because it
  writes NVS on every call and a 5 s poll would burn flash for the life of the vehicle.
- **BREAKING (behavioural): the 20 % throttle step becomes a proportional taper.** The throttle
  ceiling is 100 % at `limit − SPEED_LIMIT_TAPER_BAND_KMH`, falls linearly to
  `SPEED_LIMIT_FLOOR_PCT` at the limit, and holds the floor above it. The ceiling itself is
  slew-limited to `SPEED_LIMIT_CEILING_SLEW_PCT_S` so engagement cannot snap the servo. This
  **supersedes the "clamped to `SPEED_LIMIT_THROTTLE_CAP_PCT`" wording** in the unarchived
  `add-hall-speed-sensor` `speed-sensor` delta; `SPEED_LIMIT_THROTTLE_CAP_PCT` is removed.
- **The limiter is evaluated every loop on the web path too.** `processThrottleCommand()` records
  the operator's demand in `webThrottleDemandPct_`; `update()` re-applies the limiter to it every
  iteration while the web source is active. The demand is reset to 0 on every path that idles the
  throttle (web-control engage, fail-safe entry, gear-boost release) so a stale demand can never
  re-apply itself.
- **Telemetry + web UI**: `speed_limit_eff`, `speed_limit_src` (`"off"`/`"local"`/`"mavlink"`),
  `mav_speed_max`, `speed_limit_ceil`. `speed_limit_max` keeps reporting the **stored** value, so
  the input box still edits the fallback while MAVLink is in charge. The limiter card shows the
  active limit and its source, and the input is dimmed (not disabled) while the source is MAVLink.
- **`MAVLINK_SETUP.md`**: a new "Autopilot speed limit (`SPEED_MAX`)" subsection with the
  precedence table and an explicit "never written, never persisted".
- **No** new task, ISR or timer: one more `case` in the existing parse loop, one rate-limited pack
  in `update()`, and arithmetic in the existing cooperative loop.

## Impact
- Affected specs:
  - `mavlink-interface` — **ADDED** `Autopilot Maximum-Speed Parameter Subscription`. Deliberately
    ADDED rather than MODIFIED: the existing reporting requirement is already carried as a
    MODIFIED block by three unarchived changes (`add-hall-speed-sensor`, `add-ecu-telemetry-pids`,
    `remap-efi-native-fields`), and this is an orthogonal *inbound* concern that stands alone, so
    ADDED keeps the change order-independent.
  - `vehicle-systems` — **ADDED** `Maximum-Speed Limiter Ceiling Source Arbitration` and
    `Proportional Speed-Limiter Throttle Taper`. The taper requirement supersedes the reduced-
    ceiling wording of the `speed-sensor` delta in `add-hall-speed-sensor`, which is unarchived
    and therefore not a `specs/` requirement this change could MODIFY.
  - `web-telemetry` — **ADDED** `Speed-Limiter Source and Ceiling Telemetry`.
- Affected code: `include/Constants.h` (`MAVLINK_PARAM_*` block; `SPEED_LIMIT_*` taper constants,
  minus `SPEED_LIMIT_THROTTLE_CAP_PCT`), `include/MavlinkInterface.h` + `src/MavlinkInterface.cpp`
  (parameter poll, `PARAM_VALUE` handler, accessors, debug line),
  `include/VehicleController.h` + `src/VehicleController.cpp` (arbitration, taper, per-loop web
  demand), `include/WebPortal.h` + `src/TelemetryManager.cpp` + `src/WebPortal.cpp` (4 keys),
  `data/index.html` (limiter card, `updateSpeedDisplay()`, EN + UK dictionaries),
  `MAVLINK_SETUP.md`.
- `data/` changes, so this needs **both** `pio run -t upload` and `pio run -t uploadfs`.
- **Autopilot-side configuration is optional**: with `SPEED_MAX` unset or 0 the firmware simply
  falls back to the stored local value, exactly as today. Nothing is written to the autopilot.
- **NVS is untouched by this path.** The stored limit survives every MAVLink event and every
  power cycle; `[SPEED] Limiter maximum set to …` (emitted only by `SpeedSensor::setLimitMaxKmh`)
  must never appear as a result of autopilot traffic.
