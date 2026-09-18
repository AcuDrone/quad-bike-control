## 1. Constants (`include/Constants.h`)
- [x] 1.1 Add a `MAVLINK_PARAM_*` block after `MAVLINK_STATUSTEXT_MIN_MS`: `MAVLINK_PARAM_SPEED_MAX_ID` (`"SPEED_MAX"`), `MAVLINK_PARAM_SPEED_MAX_MS` (30.0f), `MAVLINK_PARAM_POLL_MS` (5000), `MAVLINK_PARAM_FIRST_DELAY_MS` (1000), `MAVLINK_PARAM_STALE_MS` (16000), `MAVLINK_PARAM_EPSILON_MS` (0.005f), `MAVLINK_MS_TO_KMH` (3.6f), each with a one-line rationale comment in the existing style.
- [x] 1.2 Grep `src/`, `include/`, `data/` and `openspec/` for `SPEED_LIMIT_THROTTLE_CAP_PCT`, then remove it from the `SPEED_LIMIT` block.
- [x] 1.3 Add `SPEED_LIMIT_TAPER_BAND_KMH` (5.0f), `SPEED_LIMIT_FLOOR_PCT` (10.0f), `SPEED_LIMIT_CEILING_SLEW_PCT_S` (200.0f), `SPEED_LIMIT_SRC_LOG_MIN_MS` (1000) and `SPEED_LIMIT_LOG_EPSILON_KMH` (0.05f) to the same block.

## 2. MavlinkInterface: `SPEED_MAX` subscription
- [x] 2.1 `include/MavlinkInterface.h`: add the public accessors `hasSpeedMaxParam()`, `getSpeedMaxKmh()`, `getSpeedMaxRawMs()` and `getSpeedMaxAgeMs()`. The header stays free of mavlink headers.
- [x] 2.2 Add the private state `targetLearnedMs_`, `speedMaxMs_` (NAN = never received), `speedMaxRxMs_`, `lastParamRequestMs_`, and the private methods `handleParamValue(sysid, compid, paramId, value)` (decoded scalars, `handleServoOutputRaw` pattern) and `requestSpeedMaxParam()`.
- [x] 2.3 Initialise the new members in the constructor list and reset them in `begin()`.
- [x] 2.4 Record `targetLearnedMs_` in `handleHeartbeat()` where `targetKnown_` is first set.
- [x] 2.5 Add `case MAVLINK_MSG_ID_PARAM_VALUE` to the RX dispatch: decode and forward `msg.sysid`, `msg.compid`, `pv.param_id`, `pv.param_value`.
- [x] 2.6 Add the poll block to `update()` after the stream re-request block: first request `MAVLINK_PARAM_FIRST_DELAY_MS` after the target was learned, then every `MAVLINK_PARAM_POLL_MS`, gated on `targetKnown_ && isLinkUp()`. The poll never stops.
- [x] 2.7 Implement `requestSpeedMaxParam()` with `mavlink_msg_param_request_read_pack(..., param_index = -1)` and the existing `to_send_buffer`/`write` pattern.
- [x] 2.8 Implement `handleParamValue()`: sysid+compid filter, 17-byte `param_id` copy before `strcmp`, reject `isnan`/`isinf`/negative/`> MAVLINK_PARAM_SPEED_MAX_MS` with a log, store value + timestamp, log only on an epsilon-sized change.
- [x] 2.9 Implement `hasSpeedMaxParam()` as "received AND > 0 AND link up AND age < `MAVLINK_PARAM_STALE_MS`".
- [x] 2.10 Extend the 1 Hz `[MAV] viso:` debug line with `spdmax` / `age` / `valid`.

## 3. VehicleController: ceiling arbitration
- [x] 3.1 `include/VehicleController.h`: add `enum class SpeedLimitSource { OFF, LOCAL, MAVLINK }`.
- [x] 3.2 Add the public accessors `getEffectiveSpeedLimitKmh(SpeedLimitSource&) const`, the no-argument overload, `getSpeedLimitSource() const`, `getSpeedLimitSourceName(SpeedLimitSource)` and `getSpeedLimitCeilingPct() const`.
- [x] 3.3 Add the private state `lastSpeedLimitSource_`, `lastSpeedLimitKmh_` (NAN sentinel), `lastSpeedLimitLogMs_` and the helper `logSpeedLimitSourceChange()`.
- [x] 3.4 Implement `getEffectiveSpeedLimitKmh()`: limiter off → `OFF`/0; usable `SPEED_MAX` → `MAVLINK` clamped into `[SPEED_LIMIT_MIN_KMH, SPEED_LIMIT_MAX_KMH]`; otherwise → `LOCAL`/stored. Never calls `setLimitMaxKmh()`.
- [x] 3.5 Implement `logSpeedLimitSourceChange()`: epsilon-gated on source or ceiling, held off by `SPEED_LIMIT_SRC_LOG_MIN_MS`, and it must NOT update the remembered state while a log is suppressed.

## 4. VehicleController: proportional taper and per-loop web demand
- [x] 4.1 Add the private state `limiterCeilingPct_` (100) and `limiterLastMs_` (0), initialised in the constructor list.
- [x] 4.2 Rewrite `applySpeedLimit()` as the taper: target ceiling 100 % → `SPEED_LIMIT_FLOOR_PCT` linearly across `SPEED_LIMIT_TAPER_BAND_KMH` below the arbitrated ceiling, floor at and above it; return `min(demand, ceiling)`.
- [x] 4.3 Slew-limit the applied ceiling to `SPEED_LIMIT_CEILING_SLEW_PCT_S` in both directions, with `dt` capped at 1 s and the first call after `limiterLastMs_ == 0` adopting the target directly.
- [x] 4.4 Reset `limiterCeilingPct_` to 100 and `limiterLastMs_` to 0 on both early-return paths (limiter disabled, speed reading invalid), keeping the existing fail-open warning.
- [x] 4.5 Add `webThrottleDemandPct_`; `processThrottleCommand()` records the (clipped) demand and applies it once, as today.
- [x] 4.6 In `update()`, after `applyFailsafe()` and before the gear-boost block, re-apply the limiter to the standing web demand every loop while the source is WEB, no gear boost is active and the throttle is not calibrating — reusing the `shouldClipThrottle()` clamp.
- [x] 4.7 Reset `webThrottleDemandPct_` to 0 in `setWebControl(true)`, on fail-safe entry, and where `updateGearBoostPID()` releases the throttle to idle.
- [x] 4.8 Confirm by inspection that `updateGearBoostPID()` / `setThrottleUs()` are untouched.

## 5. Telemetry and web UI
- [x] 5.1 `include/WebPortal.h`: add `speed_limit_eff`, `speed_limit_src`, `mav_speed_max`, `speed_limit_ceil` to the telemetry struct next to the existing hall-speed fields.
- [x] 5.2 `src/TelemetryManager.cpp`: populate all four.
- [x] 5.3 `src/WebPortal.cpp`: serialise all four next to `speed_limit_max`, and update the capacity comment on the `StaticJsonDocument`.
- [x] 5.4 `data/index.html`: add the active-limit / source readout to the limiter card.
- [x] 5.5 `data/index.html`: render the effective limit, the source label and the taper ceiling in `updateSpeedDisplay()`; dim (never disable) `speed-limit-input` while the source is mavlink; keep `syncSpeedInput('speed-limit-input', data.speed_limit_max)` bound to the stored value.
- [x] 5.6 `data/index.html`: add `lbl_speed_limit_eff`, `src_mavlink`, `src_local`, `src_off` and the limiting-ceiling label to BOTH the EN and UK dictionaries, and extend `speed_limiter_hint` in both.
- [x] 5.7 `MAVLINK_SETUP.md`: add the "Autopilot speed limit (`SPEED_MAX`)" subsection with the precedence table and the "never written, never persisted" statement.

## 6. Build and bench verification
- [x] 6.1 `pio run -e esp32-s3-devkitc-1` completes with no errors and no new warnings from `src/` or `include/`.
- [ ] 6.2 Flash firmware **and** `pio run -t uploadfs` (`data/` changed).
- [ ] 6.3 Boot with MAVLINK + VEHICLE debug on: `[MAV] Autopilot learned`, then within ~1 s `[MAV] SPEED_MAX = … m/s`; the 1 Hz line shows `valid:Y` and an `age` sawtooth 0 → 5000.
- [ ] 6.4 `param set SPEED_MAX 5` → within 5 s `[SPEED] Limit source: mavlink @ 18.0 km/h`, telemetry `speed_limit_src:"mavlink"` / `speed_limit_eff:18.0`, UI shows the active limit and source. Toggle the limiter OFF → source `off` regardless.
- [ ] 6.5 `param set SPEED_MAX 0` → within 5 s the source returns to `local @ <stored>`.
- [ ] 6.6 Unplug the Pixhawk TELEM lead → source `local` within 3 s; replug → back to `mavlink`.
- [ ] 6.7 Power-cycle the ESP32 afterwards: the stored value in the input box is unchanged, and `[SPEED] Limiter maximum set to …` never appeared.
- [ ] 6.8 Taper sweep on the hall rig (70 ppr × 1990 mm → 18 km/h ≈ 176 Hz) at 100 % web throttle: `speed_limit_ceil` falls 100 → 10 linearly across the 5 km/h band with no step at the limit; an instant 13 → 25 km/h step ramps the ceiling down over ~0.45 s rather than instantly.
- [ ] 6.9 Per-loop web check: set web throttle 80 %, then raise the hall frequency past the limit without sending a new command → the servo backs off anyway.
- [ ] 6.10 Full GCS parameter download while the 25 Hz command stream runs → `mav_cmd_rate` stays ≥ 10 Hz and no `SPEED_MAX rejected` spam.
- [ ] 6.11 `[WEB] telemetry JSON peak:` stays below 4096 with no overflow warning.
- [ ] 6.12 Drive test with an 18 km/h limit: speed settles near the ceiling with no surge or oscillation.
