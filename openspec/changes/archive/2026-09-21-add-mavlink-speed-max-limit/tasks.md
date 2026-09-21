## 1. Constants (`include/Constants.h`)
- [x] 1.1 Add the `MAVLINK_PARAM_*` block after `MAVLINK_STATUSTEXT_MIN_MS`:
  `MAVLINK_PARAM_SPEED_MAX_ID` (`"SPEED_MAX"`), `MAVLINK_PARAM_SPEED_MAX_MS` (30.0f),
  `MAVLINK_PARAM_POLL_MS` (5000), `MAVLINK_PARAM_FIRST_DELAY_MS` (1000),
  `MAVLINK_PARAM_STALE_MS` (16000), `MAVLINK_PARAM_EPSILON_MS` (0.005f).
- [x] 1.2 Remove `SPEED_LIMIT_THROTTLE_CAP_PCT` (superseded by the taper).
- [x] 1.3 Add the taper block: `SPEED_LIMIT_TAPER_BAND_MS` (1.4f), `SPEED_LIMIT_FLOOR_PCT` (10.0f),
  `SPEED_LIMIT_CEILING_SLEW_PCT_S` (200.0f), `SPEED_LIMIT_LOG_MIN_MS` (1000),
  `SPEED_LIMIT_LOG_EPSILON_MS` (0.015f).
- [x] 1.4 Delete `SPEED_LIMIT_ENABLE_DEFAULT`, `SPEED_LIMIT_MAX_KMH_DEFAULT`, `SPEED_LIMIT_MIN_KMH`
  and `SPEED_LIMIT_MAX_KMH`; rewrite the limiter comment block to say "one source, always armed,
  no NVS".
- [x] 1.5 Convert the remaining km/h constants: `SPEED_MAX_PLAUSIBLE_DECEL_KMH_S` →
  `SPEED_MAX_PLAUSIBLE_DECEL_MS2` (2.2f), `MAVLINK_VISO_NEUTRAL_ZERO_KMH` →
  `MAVLINK_VISO_NEUTRAL_ZERO_MS` (0.14f), `TRANS_SPEED_INTERLOCK_THRESHOLD` →
  `TRANS_SPEED_INTERLOCK_THRESHOLD_MS` (1.4f).
- [x] 1.6 Replace `MAVLINK_MS_TO_KMH` with a single `MS_TO_KMH` (3.6f) documented as
  PRESENTATION ONLY, and grep `Constants.h` for any remaining `KMH` identifier.

## 2. MavlinkInterface: `SPEED_MAX` subscription, m/s throughout
- [x] 2.1 `include/MavlinkInterface.h`: public accessors `hasSpeedMaxParam()`, `getSpeedMaxMs()`
  and `getSpeedMaxAgeMs()` (no km/h accessor, no separate "raw" accessor). The header stays free
  of mavlink headers.
- [x] 2.2 Private state `targetLearnedMs_`, `speedMaxMs_` (NAN = never received), `speedMaxRxMs_`,
  `lastParamRequestMs_`; private methods `handleParamValue(...)` (decoded scalars, the
  `handleServoOutputRaw` pattern) and `requestSpeedMaxParam()`.
- [x] 2.3 Initialise the new members in the constructor list and reset them in `begin()`.
- [x] 2.4 Record `targetLearnedMs_` in `handleHeartbeat()` where `targetKnown_` is first set.
- [x] 2.5 Add `case MAVLINK_MSG_ID_PARAM_VALUE` to the RX dispatch.
- [x] 2.6 Add the poll block to `update()`, gated on `targetKnown_ && isLinkUp()`; it never stops.
- [x] 2.7 Implement `requestSpeedMaxParam()` with `param_index = -1`.
- [x] 2.8 Implement `handleParamValue()`: sysid+compid filter, 17-byte `param_id` copy before
  `strcmp`, reject NaN/inf/negative/out-of-range with a log, store, log on an epsilon change only
  (m/s with km/h in parentheses).
- [x] 2.9 `hasSpeedMaxParam()` = received AND > 0 AND link up AND age < `MAVLINK_PARAM_STALE_MS`.
- [x] 2.10 Extend the 1 Hz `[MAV] viso:` debug line with `spdmax` / `age` / `valid`.
- [x] 2.11 `StateReport::speedKmh` → `speedMs`; `VFR_HUD` groundspeed and
  `sendVisionPositionDelta()` consume it directly (both `/ 3.6f` literals removed).

## 3. SpeedSensor: m/s internally, no limiter configuration
- [x] 3.1 `speedKmh_` → `speedMs_`, `getSpeedKmh()` → `getSpeedMs()`, `lastMovingSpeedKmh_` →
  `lastMovingSpeedMs_`; drop the `* 3.6f` from the sample formula (mm/ms already is m/s).
- [x] 3.2 Convert the decay / plausibility branch of `update()` — keeping its per-sample decay and
  `suspicious_` latch — to `SPEED_MAX_PLAUSIBLE_DECEL_MS2` and
  `impliedMax = distancePerPulseMm_ / silenceMs`; the warning keeps km/h in parentheses.
- [x] 3.3 Remove `limiterEnabled_`, `limitMaxKmh_`, `isLimiterEnabled()`, `getLimitMaxKmh()`,
  `setLimiterEnabled()` and `setLimitMaxKmh()`, and the `lim_on` / `lim_kmh` loads in `begin()`.
- [x] 3.4 `begin()` deletes the retired `lim_on` / `lim_kmh` keys once. No new NVS key is added.

## 4. VehicleController: one limit source, m/s taper
- [x] 4.1 Delete `enum class SpeedLimitSource` and every accessor built on it
  (`getEffectiveSpeedLimitKmh`, `getSpeedLimitSource`, `getSpeedLimitSourceName`,
  `isSpeedLimiterEnabled`, `getSpeedLimitMaxKmh`, `getMavSpeedMaxKmh`).
- [x] 4.2 Add `getSpeedLimitMs()`: the autopilot value when `hasSpeedMaxParam()`, else 0.
- [x] 4.3 `getVehicleSpeedKmh()` → `getVehicleSpeedMs()`.
- [x] 4.4 `logSpeedLimitSourceChange()` → `logSpeedLimitChange(float limitMs)`, epsilon-gated on
  `SPEED_LIMIT_LOG_EPSILON_MS`, held off by `SPEED_LIMIT_LOG_MIN_MS`, logging "none" for 0 and
  NOT updating the remembered state while suppressed.
- [x] 4.5 `applySpeedLimit()` takes its early exit on `limitMs <= 0` (resetting the ceiling to
  100 % and `limiterLastMs_` to 0), keeps the fail-open sensor exit and its rate-limited warning,
  and computes the taper band in m/s.
- [x] 4.6 Slew-limit the applied ceiling to `SPEED_LIMIT_CEILING_SLEW_PCT_S` in both directions,
  `dt` capped at 1 s, first call adopting the target directly.
- [x] 4.7 `webThrottleDemandPct_` recorded in `processThrottleCommand()` and re-limited every loop
  in `update()` while the source is WEB, no gear boost is active and the throttle is not
  calibrating; reset to 0 in `setWebControl(true)`, on fail-safe entry, and on both gear-boost
  throttle releases.
- [x] 4.8 Remove `processSpeedLimitEnableCommand()` / `processSpeedLimitSetCommand()` and their
  `speed_limit_enable` / `speed_limit_set` dispatch arms.
- [x] 4.9 `TransmissionVehicleData::sensorSpeedKmh` → `sensorSpeedMs`; the interlock compares
  `TRANS_SPEED_INTERLOCK_THRESHOLD_MS`, and the CAN-km/h fallback converts at the comparison.

## 5. Telemetry and web UI
- [x] 5.1 `include/WebPortal.h`: `vehicle_speed` → `vehicle_speed_ms`, the five limiter fields →
  one `speed_limit_ms`; `speed_limit_ceil` unchanged.
- [x] 5.2 `src/TelemetryManager.cpp`: populate both from `getVehicleSpeedMs()` /
  `getSpeedLimitMs()`.
- [x] 5.3 `src/WebPortal.cpp`: serialise `vehicle_speed` and `speed_limit_kmh` as km/h
  (`× MS_TO_KMH`, 1 decimal) and drop `speed_limit_on` / `speed_limit_max` / `speed_limit_eff` /
  `speed_limit_src` / `mav_speed_max`.
- [x] 5.4 `data/index.html`: remove the "Maximum speed (km/h)" input, its Save button and the
  "Speed limiter ON/OFF" button.
- [x] 5.5 `data/index.html`: render "Speed limit: <km/h> (<m/s>)" or "none" from
  `speed_limit_kmh`, keep the "Limiting <ceil> %" row.
- [x] 5.6 `data/index.html`: remove `saveSpeedLimit()`, `toggleSpeedLimiter()`,
  `speedLimiterState`, the `lim_src_*` / `lbl_speed_limit_max` / `lbl_speed_limiter` /
  `lbl_speed_limit_eff` / `invalid_speed_limit` strings in BOTH dictionaries; add
  `lbl_speed_limit` and `lim_none`; rewrite `speed_limiter_hint` in EN and UK.
- [x] 5.7 `MAVLINK_SETUP.md`: replace the precedence table with a "when the limit applies" table
  whose every fallback row is *none*, and state that m/s is the internal unit.
- [x] 5.8 `WEB_PORTAL_SETUP.md`: one short read-only note saying the limit is the autopilot's and
  there is nothing to set in the portal.

## 6. Build and bench verification
- [x] 6.1 `pio run -e esp32-s3-devkitc-1` completes with no errors and no warnings from `src/` or
  `include/`.
- [x] 6.2 `grep -rn 'lim_on|lim_kmh|speed_limit_enable|speed_limit_set|SpeedLimitSource|getSpeedKmh|SpeedMaxKmh'`
  over `src include data` returns only the two deliberate `prefs.remove()` cleanup lines.
- [ ] 6.3 Flash firmware **and** `pio run -t uploadfs` (`data/` changed).
- [ ] 6.4 Boot with MAVLINK + VEHICLE debug on: `[MAV] Autopilot learned`, then within ~1 s
  `[MAV] SPEED_MAX = … m/s`; the 1 Hz line shows `valid:Y` and an `age` sawtooth 0 → 5000.
- [ ] 6.5 `param set SPEED_MAX 5` → within 5 s `[SPEED] Speed limit: 5.00 m/s (18.0 km/h) from
  SPEED_MAX`, telemetry `speed_limit_kmh:18.0`, and the UI shows "18.0 km/h (5.0 m/s)".
- [ ] 6.6 `param set SPEED_MAX 0` → within 5 s `[SPEED] Speed limit: none`, telemetry
  `speed_limit_kmh:0.0`, UI shows "none", and full throttle is no longer clamped.
- [ ] 6.7 Unplug the Pixhawk TELEM lead → "none" within 3 s; replug → the limit returns.
- [ ] 6.8 Power-cycle the ESP32: NVS namespace `"speed"` still holds `ppr` and `circ_mm` and no
  longer holds `lim_on` or `lim_kmh` (dump with `nvs_get`/serial), and the UI has no limiter
  controls to restore.
- [ ] 6.9 Taper sweep on the hall rig (70 ppr × 1990 mm) at 100 % web throttle with
  `SPEED_MAX = 5`: `speed_limit_ceil` falls 100 → 10 linearly across the 1.4 m/s band with no step
  at the limit; an instant 3.6 → 7 m/s step ramps the ceiling down over ~0.45 s.
- [ ] 6.10 Per-loop web check: set web throttle 80 %, then raise the hall frequency past the limit
  without sending a new command → the servo backs off anyway.
- [ ] 6.11 Pull the hall lead while rolling → `[SPEED] WARNING: … implausibly fast …` and
  `[SPEED] WARNING: speed limit active but speed reading is invalid` — throttle NOT clamped.
- [ ] 6.12 Full GCS parameter download while the 25 Hz command stream runs → `mav_cmd_rate` stays
  ≥ 10 Hz and no `SPEED_MAX rejected` spam.
- [ ] 6.13 `[WEB] telemetry JSON peak:` stays below 4096 with no overflow warning.
- [ ] 6.14 Gear-interlock re-check in the new units: a gear change is blocked above 1.4 m/s and
  allowed below it, on both the sensor path and the CAN fallback.
- [ ] 6.15 Drive test with `SPEED_MAX = 5`: speed settles near the limit with no surge or
  oscillation.
