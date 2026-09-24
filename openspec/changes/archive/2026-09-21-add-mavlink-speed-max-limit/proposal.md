# Change: The speed limit comes from the autopilot's `SPEED_MAX`, and from nowhere else

## Why
The vehicle used to carry two speed ceilings that had never met: ArduPilot's `SPEED_MAX` (m/s,
edited live from Mission Planner / MAVProxy over the TELEM link the ESP32 already speaks) and a
firmware limiter with its own NVS-stored km/h ceiling and its own on/off toggle, reachable only
from the ESP32's WiFi portal on a vehicle that is being driven.

The first version of this change made the firmware ceiling *follow* `SPEED_MAX`, with the stored
value as a fallback and the web toggle as a master switch. Operating it proved that three-way
arbitration is the wrong shape for this vehicle:

- the fallback is invisible — a dropped TELEM link silently swaps a deliberate 5 km/h crawl for a
  60 km/h stored ceiling nobody at the ground station can see or change;
- two places to set one number means the one that is actually enforced is never obvious;
- the master switch is a second, WiFi-only safety control on a machine whose cockpit is the GCS.

**Decision (operator's): one source, no fallback value, no toggle.** The limit is `SPEED_MAX`.
When there is no usable `SPEED_MAX`, there is no limiting — which is what `0` already means to
ArduPilot and what Mission Planner's speed-limit sign writes to clear a limit.

Second, the firmware carried speeds in km/h internally while every interface it touches —
`SPEED_MAX`, `VFR_HUD`, `VISION_POSITION_DELTA`, the sensor's own mm/ms arithmetic — is m/s. That
put a `× 3.6` and a `/ 3.6` on opposite sides of nearly every speed path. **The internal unit is
now m/s everywhere; km/h survives only at the presentation edge** (web JSON, HTML, and
human-readable debug strings).

## What Changes
- **New inbound MAVLink parameter subscription.** `MavlinkInterface` gains a `PARAM_VALUE` case, a
  `PARAM_REQUEST_READ` poll for `SPEED_MAX` every `MAVLINK_PARAM_POLL_MS` (5 s, first request
  `MAVLINK_PARAM_FIRST_DELAY_MS` after the autopilot is learned), and the accessors
  `hasSpeedMaxParam()` / `getSpeedMaxMs()` / `getSpeedMaxAgeMs()`. Both paths are implemented on
  purpose: ArduPilot's broadcast-on-`PARAM_SET` behaviour is version- and routing-dependent, so
  the poll — not the broadcast — is the change detector.
- **Value hygiene at the transport.** `param_id` is a 16-byte, *not* NUL-terminated field and is
  copied into a 17-byte buffer before comparison. NaN, infinity, negative values and values above
  `MAVLINK_PARAM_SPEED_MAX_MS` (30 m/s, ArduPilot's own range bound) are rejected with a log and
  never stored. Only the learned autopilot's sysid/compid is accepted, so a GCS on the same wire
  (sysid 255) cannot move the vehicle's speed ceiling.
- **BREAKING: the local ceiling, the local toggle and their web commands are removed.** Gone:
  NVS keys `lim_on` / `lim_kmh`, the web commands `speed_limit_enable` / `speed_limit_set`,
  `SpeedSensor::setLimiterEnabled()` / `setLimitMaxKmh()` / `isLimiterEnabled()` /
  `getLimitMaxKmh()`, the constants `SPEED_LIMIT_ENABLE_DEFAULT`, `SPEED_LIMIT_MAX_KMH_DEFAULT`,
  `SPEED_LIMIT_MIN_KMH`, `SPEED_LIMIT_MAX_KMH`, the `SpeedLimitSource` enum and every arbitration
  accessor built on it, and the web portal's "Maximum speed (km/h)" input and "Speed limiter
  ON/OFF" button (EN + UK strings with them). `SpeedSensor::begin()` deletes the two retired NVS
  keys once. **No new NVS key is added.**
- **BREAKING: the fallback is no limiting.** `VehicleController::getSpeedLimitMs()` returns the
  autopilot value when `hasSpeedMaxParam()` is true and **0 otherwise** — never received, zero,
  rejected, stale, or link down. A ceiling of 0 means the taper is bypassed entirely.
- **The limiter is always armed.** There is no master switch. It still **fails OPEN** when the
  speed reading is invalid, with the same rate-limited warning.
- **BREAKING (behavioural, from the first version of this change): the 20 % throttle step became
  a proportional taper.** The throttle ceiling is 100 % at `limit − SPEED_LIMIT_TAPER_BAND_MS`,
  falls linearly to `SPEED_LIMIT_FLOOR_PCT` at the limit, and holds the floor above it, with the
  ceiling itself slew-limited to `SPEED_LIMIT_CEILING_SLEW_PCT_S`. This supersedes the "clamped to
  `SPEED_LIMIT_THROTTLE_CAP_PCT`" wording of the unarchived `add-hall-speed-sensor` delta.
- **The limiter is evaluated every loop on the web path too.** `processThrottleCommand()` records
  the operator's demand in `webThrottleDemandPct_`; `update()` re-applies the limiter to it every
  iteration while the web source is active, and the demand is reset to 0 on every path that idles
  the throttle.
- **BREAKING: m/s is the internal unit.** `SpeedSensor::getSpeedKmh()` → `getSpeedMs()`
  (`speedKmh_` → `speedMs_`, `lastMovingSpeedKmh_` → `lastMovingSpeedMs_`; the existing
  `(delta × mm/pulse) / windowMs` **is** m/s, so the `× 3.6f` simply goes away);
  `VehicleController::getVehicleSpeedKmh()` → `getVehicleSpeedMs()`;
  `MavlinkInterface::getSpeedMaxKmh()` → `getSpeedMaxMs()` and `StateReport::speedKmh` →
  `speedMs`, so `VFR_HUD` and `VISION_POSITION_DELTA` consume it with no division;
  `TransmissionVehicleData::sensorSpeedKmh` → `sensorSpeedMs`. Constants follow:
  `SPEED_MAX_PLAUSIBLE_DECEL_KMH_S` → `SPEED_MAX_PLAUSIBLE_DECEL_MS2` (2.2 m/s²),
  `SPEED_LIMIT_TAPER_BAND_KMH` → `SPEED_LIMIT_TAPER_BAND_MS` (1.4), `SPEED_LIMIT_LOG_EPSILON_KMH`
  → `SPEED_LIMIT_LOG_EPSILON_MS` (0.015), `MAVLINK_VISO_NEUTRAL_ZERO_KMH` →
  `MAVLINK_VISO_NEUTRAL_ZERO_MS` (0.14), `TRANS_SPEED_INTERLOCK_THRESHOLD` →
  `TRANS_SPEED_INTERLOCK_THRESHOLD_MS` (1.4). `MAVLINK_MS_TO_KMH` becomes a single
  presentation-only `MS_TO_KMH`.
- **Telemetry**: `speed_limit_on`, `speed_limit_max`, `speed_limit_eff`, `speed_limit_src` and
  `mav_speed_max` are replaced by one key, `speed_limit_kmh` (1 decimal, `0` = no limit).
  `speed_limit_ceil` and `vehicle_speed` (km/h, 1 decimal) are unchanged on the wire — both are
  converted from m/s at serialisation. The web UI shows "Speed limit: 30.0 km/h (8.3 m/s)" or
  "none", keeps the "Limiting <ceil> %" row, and says in both dictionaries that the limit comes
  only from the autopilot's `SPEED_MAX`, that 0 means none, and that the limiter does not act
  without a valid sensor reading.
- **Docs**: `MAVLINK_SETUP.md`'s "Autopilot speed limit (`SPEED_MAX`)" section loses the
  precedence table in favour of a "when the limit applies" table whose every fallback row is
  *none*; `WEB_PORTAL_SETUP.md` gains one short read-only note.
- **No** new task, ISR or timer: one more `case` in the existing parse loop, one rate-limited pack
  in `update()`, and arithmetic in the existing cooperative loop.

## Impact
- Affected specs:
  - `mavlink-interface` — **ADDED** `Autopilot Maximum-Speed Parameter Subscription`. ADDED rather
    than MODIFIED: the existing reporting requirement is already carried as a MODIFIED block by
    three unarchived changes, and this is an orthogonal *inbound* concern that stands alone.
  - `vehicle-systems` — **ADDED** `Autopilot-Sourced Maximum-Speed Limit` and
    `Proportional Speed-Limiter Throttle Taper`.
  - `speed-sensor` — **REMOVED** `Configurable Maximum-Speed Throttle Limiter`.
  - `web-telemetry` — **ADDED** `Speed-Limit and Limiter-Ceiling Telemetry`.
- **Archive order is load-bearing**: `add-hall-speed-sensor` must be archived **before** this
  change, because this change's `speed-sensor` delta REMOVES a requirement that only that change
  introduces. Archiving this one first would leave the retired limiter requirement in `specs/`.
- Affected code: `include/Constants.h`, `include/SpeedSensor.h` + `src/SpeedSensor.cpp`,
  `include/MavlinkInterface.h` + `src/MavlinkInterface.cpp`, `include/VehicleController.h` +
  `src/VehicleController.cpp`, `include/TransmissionController.h` +
  `src/TransmissionController.cpp`, `include/WebPortal.h` + `src/WebPortal.cpp`,
  `src/TelemetryManager.cpp`, `src/main.cpp`, `data/index.html`, `MAVLINK_SETUP.md`,
  `WEB_PORTAL_SETUP.md`.
- `data/` changes, so this needs **both** `pio run -t upload` and `pio run -t uploadfs`.
- **Field migration**: a vehicle whose autopilot has no `SPEED_MAX` (or `SPEED_MAX = 0`) is now
  **unlimited** where it used to be held at the stored 60 km/h ceiling. Setting `SPEED_MAX` is
  therefore a required commissioning step, not an optional one.
