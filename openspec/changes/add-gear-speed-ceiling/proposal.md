# Change: Fixed per-gear maximum-speed ceiling on the buggy (L 45, H 75, R 20 km/h)

## Why
On the buggy (ODES Junglecross 800, `aztec` branch) the maximum-speed limiter has one source,
the autopilot's `SPEED_MAX`. When the operator picks "no limit" (`SPEED_MAX = 0`), or the value
is unusable (never received, stale, link down), the limiter does nothing. The buggy can then run
up to full engine RPM in whatever gear is engaged. In L that is about 57 km/h, and H is faster.
The owner wants a hard ceiling per gear that the operator cannot switch off.

## What Changes
- New firmware constants in `include/Constants.h` (aztec only; no web field, no NVS key, no
  MAVLink parameter): `GEAR_SPEED_CEILING_LOW_MS` = 45 km/h (12.5 m/s),
  `GEAR_SPEED_CEILING_HIGH_MS` = 75 km/h (≈20.83 m/s) and `GEAR_SPEED_CEILING_REVERSE_MS` =
  20 km/h (≈5.56 m/s). They are written as `km/h ÷ MS_TO_KMH`, because the firmware
  works in m/s internally.
- **Effective limit** = min(`SPEED_MAX`, ceiling of the engaged gear). A zero on either side means
  "no ceiling from that side". With `SPEED_MAX = 0` (or no usable `SPEED_MAX`) in L, H or R, the gear
  ceiling applies on its own. The gear ceiling therefore cannot be exceeded while the speed sensor
  is healthy.
- The gear ceiling is enforced by the **existing limiter law** (taper band, floor, ceiling slew,
  integral trim, saturation indicator) exactly as `SPEED_MAX` is. There is no new control law.
  `applySpeedLimit()` gets the effective limit in place of `getSpeedLimitMs()`.
- New pure helper in `include/SpeedLimiterLaw.h` (header-only, host-tested):
  `speedLimitEffective(speedMaxMs, gearCeilMs)` → `{limitMs, source}`, where the source is
  `NONE` / `SPEED_MAX` / `GEAR`. On a tie the source is `SPEED_MAX`.
- The engaged gear is the firmware's **expected gear**, `transmission_.getTargetGear()`. This is
  the same signal the N/UNKNOWN throttle clip and the parking-brake logic use (decision D2).
  - L, H and R: their ceiling (45 / 75 / 20 km/h) (D1, D3).
  - N and UNKNOWN (not initialised): no gear ceiling. The throttle there is already clipped to
    `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (8.5 %) (D4).
  - The gear changes only at or below 1.4 m/s (`TRANS_SPEED_INTERLOCK_THRESHOLD_MS`). That is
    below every taper band, so switching between ceilings never causes a throttle step (D5).
- Invalid speed reading: **fail open, unchanged**. The demand passes through and the law resets
  (D6). The "speed invalid" warning (rate-limited to `SPEED_LIMIT_WARN_MS`) fires only when the
  effective limit's source is `SPEED_MAX` (`speed_limit_src` = `"speed_max"`). Whenever a gear
  ceiling is the source it stays silent, even with a usable `SPEED_MAX` above it (e.g. `SPEED_MAX`
  30 km/h in R), so a buggy parked in gear after boot, before the first wheel pulse, does not log
  it (D7). This touches
  the "Invalid speed reading fails open" scenario of "Proportional Speed-Limiter Throttle Taper".
- Limit log names the source: `from SPEED_MAX` or `from gear ceiling (L)` / `(H)` / `(R)`. A change of
  source with the same value is logged too.
- **Telemetry**: `speed_limit_kmh` now reports the effective limit. A new key, `speed_limit_src`,
  carries `"none"` / `"speed_max"` / `"gear_l"` / `"gear_h"` / `"gear_r"`. The letter comes from the
  expected gear the limiter used, not from the sensed `gear` key. The key is removed from the "retired keys" list. Its
  old meaning, local vs. autopilot ceiling, no longer exists.
- **Web**: the speed-limit value appends "· передача L" / "· gear L" (or H, R) when the gear ceiling is
  the source; otherwise it is unchanged. The help text of the "Обмеження швидкості та кермо" card
  gains one sentence (L 45 / H 75 / R 20 km/h, cannot be switched off, does not act without a valid speed
  reading). uk + en. Needs a LittleFS upload.
- **BREAKING (behaviour, aztec only)**: this reverses "no local / default ceiling" in the
  "Autopilot-Sourced Maximum-Speed Limit" requirement, and "no source key" in the telemetry
  requirement. The "Proportional Speed-Limiter Throttle Taper" delta is based on the version in
  `add-throttle-open-slew` (implemented, not archived yet), so that change should be archived first. `master` / the quad bike / `old-board` are not changed.
- Firmware 1.5.32 (changelog bullet, uk + en); ships after `default-throttle-open-slew-off`
  (1.5.31, master, merged into aztec). `MAVLINK_SETUP.md` gets a buggy note in the
  `SPEED_MAX` section.

## Decisions (confirmed by the owner, 2026-11-13)
- **D1** Ceilings L = 45 km/h, H = 75 km/h, R = 20 km/h, as firmware constants only. *(Confirmed by
  the owner.)*
- **D2** The ceiling follows the expected (target) gear, `transmission_.getTargetGear()`, not the
  opto gear sensor. The sensor reads UNKNOWN mid-shift and on code glitches. The target is never
  UNKNOWN after initialisation and is what the throttle clip already trusts. *(Confirmed by the
  owner.)*
- **D3** R has a ceiling of 20 km/h: effective limit in R = min(`SPEED_MAX`, 20 km/h). *(Confirmed by
  the owner.)*
- **D4** N / UNKNOWN: no gear ceiling (the throttle is already clipped to 8.5 %). The buggy has no P.
  *(Confirmed by the owner.)*
- **D5** No limiter reset when the gear ceiling changes on a shift. The law recomputes the band
  every evaluation, and shifts happen below every band. *(Confirmed by the owner.)*
- **D6** Invalid speed: fail open, as the existing limiter does. The gear ceiling therefore does not
  protect against a dead wheel sensor. *(Confirmed by the owner.)*
- **D7** The "speed invalid" warning fires only when the effective limit's source is `SPEED_MAX`
  (`speed_limit_src` = `"speed_max"`, including a tie). Whenever a gear ceiling is the source it
  stays silent: with `SPEED_MAX` 0, stale or link down, and also with a usable `SPEED_MAX` above
  the gear ceiling (e.g. 30 km/h in R).
  *(Confirmed by the owner.)*
- **D8** Link down / stale `SPEED_MAX`: the gear ceiling still applies (it does not depend on
  MAVLink). *(Confirmed by the owner.)*

## Impact
- Affected specs: vehicle-systems, web-telemetry
- Affected code: `include/Constants.h`, `include/SpeedLimiterLaw.h`, `include/VehicleController.h`,
  `src/VehicleController.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`,
  `src/TelemetryManager.cpp`, `data/index.html`, `include/Changelog.h`, `MAVLINK_SETUP.md`,
  `test/test_speed_limiter/test_main.cpp`
- No NVS change, no MAVLink change. The autopilot still reads its own `SPEED_MAX`. The ground
  station does not see the gear ceiling. It only sees that the vehicle does not go faster.
- `aztec` branch only. Not cherry-picked to `master`.
