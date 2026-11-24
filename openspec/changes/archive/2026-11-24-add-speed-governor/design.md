# Design: add-speed-governor

## Context
How the throttle is produced today (`master` 1.5.34; line numbers are `master`):
- **Two throttle paths, one limiter call.** MAVLink: `processMavlinkCommands()`
  (`src/VehicleController.cpp:786-792`) reads `mavlink_.getThrottle()`, clips it to
  `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (8.5 %) when `shouldClipThrottle()` (gear change active, target
  gear N or UNKNOWN), and writes `throttle_.setThrottlePercent(applySpeedLimit(pct))`; skipped while
  `isThrottleLockedForShift()` (boost PID or shift idle hold). Web: `processThrottleCommand()`
  (`:1032-1045`) records `webThrottleDemandPct_` and applies it once; `update()` (`:391-397`)
  re-applies the standing demand every loop under the same clip and lock, and not while calibrating.
  Fail-safe (`applyFailsafe()`, `:679-707`) and web takeover call `throttle_.idle()` and zero the
  web demand. The gear-change boost PID writes µs directly after these (`:403-409`).
- **RC channel 2 is throttle and brake combined** (`src/MavlinkInterface.cpp:1195-1226`): above
  centre `getThrottle()` (with a 2 % dead-band), below centre `getBrake()`; the two are mutually
  exclusive by construction. The web brake (`processBrakeCommand()`, `:1678-1688`) is independent of
  the web throttle; nothing in the code idles the throttle on a brake (the "Throttle safety override"
  scenario in `specs/vehicle-systems` is spec drift, not implemented, and stays out of scope).
- **The limiter** (`applySpeedLimit()`, `:1093-1166`, law `include/SpeedLimiterLaw.h`): limit =
  `getSpeedLimitMs()` (usable `SPEED_MAX`, else 0; on `aztec` min(`SPEED_MAX`, gear ceiling) via
  `speedLimitEffective()`). 0 → reset, pass through. `!speedSensor_.isValid()` → reset, pass through,
  `[SPEED] WARNING … fail open` at most every 5 s (on `aztec` only when the source is `SPEED_MAX`).
  Otherwise `speedLimiterStep()`: a ceiling = proportional taper over the last 20 % of the limit
  (0.5–1.4 m/s) + integral trim (Ki 30 %/s per m/s, rise ≤ 10 %/s, only while binding), clamped to
  [floor, 100], slew-limited at `thr_slew` (80 %/s), output min(demand, ceiling); then the
  saturation badge (ceiling at the floor, speed > limit + 0.3 m/s for 2 s, held 10 s). The floor,
  taper, trim and badge from the old idea `add-speed-limiter-integral-floor` are all implemented
  (1.5.3, archived 2026-11-02); the slew setting is `add-speed-limiter-slew-setting` (1.5.12).
  Below the band the demand passes through: **the stick is a throttle, never a speed.**
- **Speed sensor** (`include/SpeedSensor.h`): 5 Hz samples (`SPEED_SAMPLE_INTERVAL_MS` 200), one
  pulse = 28.4 mm on the quad (1990 mm / 70), so the speed quantum at 5 Hz is ≈ 0.14 m/s.
  `isValid()` = initialised && pulsed at least once since boot && not suspicious (latched when the
  pulses stop faster than a 2.2 m/s² stop allows). The reading decays to 0 after 2 s without pulses.
  How the value is formed (`src/SpeedSensor.cpp:127-217`): every ≥ 200 ms the PCNT total is read;
  if pulses arrived, speed = pulses × 28.4 mm / (time since the last window that saw pulses) — so
  above ≈ 5 km/h it is an integer pulse count over one ≈ 200 ms window (the window is the loop's
  sample-to-sample time, not a pulse-edge time). At 10 km/h that is ≈ 19.6 pulses per window, so the
  reading alternates 2.70 / 2.84 m/s (one quantum = 0.51 km/h = 5 % of the setpoint); at 5 km/h
  ≈ 9.8 pulses (10 %); at 30 km/h ≈ 59 pulses (1.7 %). The value is a 200 ms average held for the
  next 200 ms: ≈ 0.2 s of effective dead time. No filtering is applied.
- **ThrottleController**: `setThrottlePercent()` passes through the opening-rate limit (default off,
  `open_slew`), `idle()` is immediate, `lastPct_` holds the last commanded percent.
  `percentToUs()` truncates to whole µs between the calibrated idle and full pulse widths (quad
  default 901 → 1344 µs, a 443 µs span; the real per-vehicle values are in NVS `"throttle"`), and
  `ServoController` writes a 14-bit LEDC duty at 50 Hz: 20 000 µs / 16 383 = 1.22 µs per count.
  The smallest distinct command step is therefore 1.22 µs = **0.28 % of a 443 µs span** (≈ 363
  steps over the full span, ≈ 3.6 per %, ≈ 11 within 3 %); on a 1 400 µs span (the mirrored
  2200 → 800 example in `Constants.h`) 0.09 %. The servo's own dead-band (unknown model; typically
  1–4 µs) and the cable slack come on top. **Per vehicle:** `aztec` compiles the same throttle
  defaults (`THROTTLE_DEFAULT_IDLE_US` 901 / `_FULL_US` 1344, checked with `git show aztec:`), so a
  fresh buggy board also has 0.28 % steps; the buggy's throttle servo is, however, mounted mirrored
  and the project notes record its calibration as idle 2200 / full 800 µs — if that is what its NVS
  holds, the buggy's step is ≈ **0.09 %** (1 400 µs span). The stored values are not in the repo
  (tasks 8.1 notes them). The quad uses the 443 µs default span as far as the repo shows.
- **NVS `"speed"`** is shared: `SpeedSensor` (`ppr`, `circ_mm`, `odo_mm`, `trip_mm`) and
  `VehicleController` (`thr_floor`, `thr_slew`); never `clear()`ed.

## Goals / Non-Goals
- Goals: with the switch on, the stick commands a speed between 0 and the effective limit and the
  throttle is closed-loop on the wheel speed; no throttle step at any engagement or hand-over; the
  operator always sees on the web page (and the GCS for the dangerous case) whether the governor is
  governing; switch off = today's firmware.
- Non-Goals: braking (down-hill hold, active deceleration) — "brake assist", a later change after the
  brake actuator fix; cruise control; GCS-set setpoints or gains; autopilot-mode awareness;
  changing the ceiling limiter law.

## Decisions
- **D1. Where it sits: one throttle function.** `applySpeedLimit(pct)` is renamed at its three call
  sites to `commandThrottlePct(pct, brakeCommanded)` → returns the percent for
  `throttle_.setThrottlePercent()`. Inside: compute the governor mode (D3); if ACTIVE run the
  governor (D2) and stand the limiter down (D7); otherwise run today's `applySpeedLimit()` body
  unchanged and, while a hand-over blend is pending, pass its result through the blend (D8). The
  call sites keep their existing clip and shift-lock conditions; the clip result is what the
  governor sees as "bypass" (D3). The web command's immediate apply and the `update()` re-apply both
  go through it; a second evaluation in the same millisecond integrates nothing (dt = 0).
- **D2. Control law (pure, `include/SpeedGovernorLaw.h`).** Per evaluation, with `dt` clamped to
  [0, 1] s:
  1. *Stop:* stick ≤ `GOV_STICK_DEADBAND_PCT` (5 %) or brake commanded → setpoint ramp 0,
     integrator 0, output 0 (idle) at once, return.
  2. *Target:* `v* = vMin + (s − 5)/(100 − 5) × (L − vMin)`, `vMin = min(GOV_MIN_SETPOINT_MS (1.4),
     0.5 × L)`.
  3. *Setpoint ramp:* `vr` moves toward `v*` by at most `acc × dt` upward and `dec × dt` downward.
     A lower limit (SPEED_MAX lowered, aztec H → L) only lowers `v*`; `vr` follows at `dec`.
  4. *PI:* `e = vr − v`; `P = Kp × e`; `u = clamp(P + I, 0, uMax)`, `uMax = startCap` (setting
     `gov_launch`) while `v < GOV_STANDSTILL_MS` (0.05 m/s, i.e. the reading is 0), else 100.
  5. *Anti-windup (conditional integration):* `I += Ki × e × dt` only if not (`P + I ≥ uMax` and
     `e > 0`) and not (`P + I ≤ 0` and `e < 0`); `I` clamped to [0, `uMax`].
  6. *Output rate:* the output moves toward `u` by at most `slew × dt` (slew = `thr_slew`, A5).
  Units: speeds m/s, gains % per m/s and %/s per m/s, throttle 0–100 % (0 = calibrated idle).
  The speed `v` is the sensor reading as it is (D15). Defaults are **per vehicle** (D12, D13):

  | Default | buggy (`aztec`) | quad bike (`master`, `old-board`) |
  |---|---|---|
  | Kp (% per m/s) | **1.0** | **3.0** |
  | Ki (%/s per m/s) | **0.35** | **1.0** |
  | setpoint acceleration (m/s²) | 1.0 | 1.0 |
  | setpoint deceleration (m/s²) | 2.5 | 2.5 |
  | start throttle limit (%) | **4** | **10** |

  The same at every limit on a vehicle (D14). acc 1.0 m/s² = 0 → 10 km/h in 2.8 s, 0 → 30 km/h in
  8.3 s, with ≤ 0.6 km/h model overshoot at every limit on each vehicle's own plant (D13); dec
  2.5 m/s² is about what engine braking gives — the governor can only close, so a faster setpoint
  fall just unwinds the integrator a little (model undershoot ≤ 0.1 km/h on the design plants and
  0.7 km/h on the sharp buggy plant on a 100 → 50 % stick step at 10 km/h). The ramps and every
  range are common to both vehicles: the simulation gave no reason to split them (D13). These are
  starting points for the field procedure (tasks 8), not tuned values.
  **No feed-forward** (A3): the output is started from the actual throttle at each engagement and
  the integrator from it up to the start throttle limit (D8), which is the feed-forward that
  matters.
- **D3. Modes and precedence.** Evaluated every call, first match wins:
  `OFF` (switch off) → `NOLIMIT` (effective limit 0) → `BYPASS` (`shouldClipThrottle()`, shift lock,
  calibrating — today's code owns the throttle) → `NOSPEED` (`!speedSensor_.isValid()`) → `ACTIVE`.
  Fail-safe never calls the function; any gap > `GOV_MAX_GAP_MS` (300 ms) between evaluations
  re-primes on the next one (so fail-safe exit, web takeover and source changes always restart
  bumplessly from the current throttle, which they set to idle). Telemetry tokens: `off`,
  `nolimit`, `bypass`, `nospeed`, `active`.
- **D4. Low speed and stand-still (A4).** Stick dead-band 5 % (above the RC 2 % dead-band) means
  "stop": idle at once, integrator and ramp reset — so standing still with the stick released never
  accumulates anything. The minimum setpoint 1.4 m/s matches `TRANS_SPEED_INTERLOCK_THRESHOLD_MS`
  (the "walking pace" the firmware already uses) and is halved for very low limits so the stick
  keeps resolution. **Re-checked for a 10 km/h limit:** the stick gives 5–10 km/h over 6–100 %.
  Kept, because below 5 km/h the speed quantum is ≥ 10 % of the reading and pulses arrive less than
  once per sample at a crawl (the reading becomes an interval measurement that lags), and 5 km/h is
  where the gear-change interlock already draws "walking pace". In H, 5–7 km/h can still be inside
  the CVT clutch take-up, where the plant gain is higher than the D13 figure and a smooth hold is
  not promised; in L the take-up happens at a lower road speed — hence the operator recommendation
  "low limits in L on the buggy" (proposal, not an interlock).
  **Start throttle limit** (`gov_launch`, default buggy 4 % / quad 10 %, range 1–40 %). Two jobs,
  both bounding what the integrator can hold before the governor has evidence: (1) while the
  reading is 0 it caps the output and the integrator — the engine-off, clutch-not-engaged and stuck
  cases, where the integrator would otherwise climb until the clutch snatches; (2) at a take-over it
  caps the inherited integrator (D8). Alternatives checked: the integrator clamp alone (0–100)
  bounds nothing useful; an output rate limit only delays the wind-up and does not bound it. The
  limit is sized per vehicle as "flat take-up + the slope reserve", where every % above the take-up
  is a possible lurch, and a % is worth ≈ 3 times more speed on the buggy than on the quad (D13):
  - *Buggy, 4 %* (take-up ≈ 1 %, 10 km/h at 2–3 %): in the model it moves off on the flat in
    ≈ 2 s (peak 10.1–10.3 km/h at a 10 km/h limit) and launches on a slope needing up to +2 %
    more throttle; +3 % needs 6 %. 4 % = 3 % above the take-up ≈ the throttle for 10–20 km/h —
    the largest lurch the owner's "sharp" observation makes acceptable.
  - *Quad, 10 %* (assumed take-up ≈ 3 %, 10 km/h at ≈ 9 %): moves off on the flat in ≈ 2 s on all
    three quad plants of D13 and launches on a slope needing up to +6 %; +9 % needs 12 % on the
    steep quad plant. The previous single default 6 % would not launch the design quad plant on a
    +3 % slope.
  A vehicle that must launch on steeper slopes raises it in the field (tasks 8.2). The range goes
  down to 1 % because on the buggy a few % already exceed 10 km/h on the flat.
- **D5. Settings and storage.** `VehicleController` owns them (as `thr_floor` / `thr_slew`), loads
  them where those are loaded (`initCAN()`), NVS `"speed"`: `gov_en` (bool), `gov_kp`, `gov_ki`,
  `gov_acc`, `gov_dec`, `gov_launch` (float). Missing → this build's default (D12); NaN / out of
  range → logged, default, not rewritten; set = validate → persist → apply (false on NVS failure).
  The values stay per vehicle in that vehicle's NVS: a compiled default applies **only while the
  key is absent**, so a stored (tuned) value is never replaced by a default, and a later firmware
  with other defaults changes nothing on a board that has saved its values. Wire and NVS in
  firmware units (m/s, m/s², %); the page shows km/h equivalents in the hints. Changes apply on the
  next evaluation without re-priming (gains move `P` at once — the output rate limit smooths it).
- **D6. Saturation badge (web only).** `gov_sat` = output ≤ 0.5 % (at idle) and
  `v > vr + SPEED_LIMIT_SAT_MARGIN_MS` (0.3 m/s) held for `SPEED_LIMIT_SAT_HOLD_MS` (2 s), shown
  for `SPEED_LIMIT_SAT_UI_HOLD_MS` (10 s) after it clears — the limiter's constants and shape, one
  telemetry key (the held flag). It is the "downhill, would need the brake" indication.
- **D7. Ceiling limiter stands down while ACTIVE (A8).** On every ACTIVE evaluation `limiter_` is
  reset, `limiterLastMs_` = 0 and its saturation is cleared, so `speed_limit_ceil` reads 100 and the
  limiter re-primes (first evaluation adopts the taper, zero trim) when it is needed again. The
  governor's setpoint never exceeds the limit, so the ceiling would only add a second, conflicting
  loop. In OFF / NOLIMIT / BYPASS / NOSPEED the limiter runs exactly as today — in NOSPEED it
  fails open as today. Spec form: an ADDED requirement naming the limiter requirements it takes
  precedence over, not a MODIFIED copy, because those requirements differ between `master` and
  `aztec` (add-gear-speed-ceiling) and a copied text would conflict on every merge.
- **D8. Hand-over, both directions.**
  - *Into ACTIVE* (from any mode, or after a gap): `vr = v` (measured), output = `u0`,
    `I = min(u0, startCap)`, where `u0` = `throttle_.getLastCommandedPct()` (new accessor for
    `lastPct_`; after an `idle()` it is 0). The first output equals the current throttle (no step);
    if `u0` was above the start throttle limit the output then moves toward `P + I` at `thr_slew`.
    Why the cap: with Ki 0.35–1.0 an integrator primed at, say, 100 % would unwind at only a few
    %/s; in the previous revision's model a full-stick first launch at a 10 km/h limit (A10) then
    peaked at ≈ 64 km/h and needed ≈ 25 s to settle. With the cap and each vehicle's defaults on
    its own plant (D13): first launch at full stick, take-over at 0.3 m/s from 100 % → peak
    **≈ 20–21 km/h on the buggy** (settled within ±1 km/h after 4–9 s) and **≈ 13–17 km/h on the
    quad** (3–13 s). The cost: switching on while cruising at 30 km/h dips to ≈ 22 km/h on both
    (buggy cruising on ≈ 10 % with a 4 % limit, quad on ≈ 25 % with a 10 % limit) while the
    integrator rebuilds — raise the limit if that matters. Gear changes happen below the 5 km/h
    interlock with at most the 8.5 % clip, so a bypass exit inherits at most the start throttle
    limit instead of 8.5 % at walking pace — no practical difference. If `v` is above `v*` the ramp
    brings `vr` down at `dec`.
  - *Out of ACTIVE* into OFF / NOLIMIT / NOSPEED: a blend starts at the governor's last output and
    moves toward today's-path value (`applySpeedLimit(demand)`) by at most `thr_slew × dt` in either
    direction; it ends when within 0.5 % or when today's value is the stop value (stick in the
    dead-band, brake). Into BYPASS: no blend — the N / shift clip and the shift lock are safety
    closures and apply at once, as today.
  - So in NOSPEED with the stick at 100 % the throttle rises from wherever the governor left it to
    100 % at 80 %/s (≤ 1.25 s), not in one step. That is the "no restriction" of decision 4 reached
    without a jump.
- **D9. Brake (decision 2).** RC: channel 2 below centre → `getThrottle()` 0 → stop (D2.1) — the
  governor cannot fight the brake because it never sees a demand while braking. Web: the call passes
  `brakeCommanded = currentBrakeTarget_ > 0` (`applyBrake()` zeroes targets below 5 %); with the
  governor ACTIVE that is the same stop. The brake path itself (`applyBrake()`, `updateBrakeControl()`)
  is not touched. With the governor OFF the web throttle and brake stay independent as today (A6).
- **D10. Web throttle (A1).** The slider's 0–100 % is the stick. The page shows "→ NN км/год" next to
  the slider value while `gov_state` = `active`, computed from `gov_sp` (the ramped setpoint), so the
  operator sees what the slider means.
- **D11. Warnings.** Web: a banner in both speed cards whenever `gov_en` and `gov_state` = `nospeed`
  ("Регулятор не діє: немає сигналу швидкості — газ напряму" / "Governor not acting: no speed signal —
  direct throttle", danger colour), and the at-idle badge (D6). GCS: the vehicle layer raises
  `StateReport.govNoSpeed` when mode = NOSPEED and the stick is above the dead-band continuously for
  `GOV_NOSPEED_WARN_DELAY_MS` (2000 ms) — the delay keeps the first-pulse-after-boot launch quiet;
  `MavlinkInterface::report()` sends `MAV_SEVERITY_WARNING` "Gov: NO SPEED - direct throttle" on the
  rising edge and every `GOV_NOSPEED_WARN_REPEAT_MS` (10 s) while it stays true, and
  `MAV_SEVERITY_INFO` "Gov: speed OK - governing" when it falls because the mode became ACTIVE; both
  through the shared `MAVLINK_STATUSTEXT_MIN_MS` limiter with the pending rule used for the steering
  mode. Serial: one `[GOV] mode A -> B` debug line per mode change (for the bench, never the only
  signal).
- **D12. Branches and per-vehicle defaults (owner, 2026-11-17).** `aztec`: `getSpeedLimitMs()`
  already returns the effective limit, so the governor uses the gear ceiling without code of its
  own (A11); the D7 warning rule of the limiter is not copied. `old-board`: same code (it has the
  PCNT speed sensor on GPIO17 and `applySpeedLimit()`).
  *Which constants differ:* `GOV_KP_DEFAULT` (quad 3.0 / buggy 1.0), `GOV_KI_DEFAULT` (1.0 /
  0.35), `GOV_LAUNCH_DEFAULT_PCT` (10 / 4) — and a new tag `GOV_DEFAULTS_VEHICLE` (1 / 2). Common:
  `GOV_ACC_DEFAULT_MS2`, `GOV_DEC_DEFAULT_MS2`, every `_MIN` / `_MAX`, the fixed constants
  (dead-band, minimum setpoint, stand-still, gap, warning timings), `GOV_ENABLED_DEFAULT` false.
  *Mechanism — the established pattern:* `master`'s `Constants.h` holds the quad bike's values
  (like `STEER_PARK_*_DEFAULT`, `STEER_PWM_*_DUTY`, `ENGINE_RUNNING_RPM_THRESHOLD` 1500), and
  `aztec`'s `Constants.h` overrides the same `#define`s in place with an "aztec (was N on the
  quad)" comment (as for the park ladder 40×50 / 40×60 / 50×60, the steering duties 30 / 30, jog
  50, nudge 40×50, `ENGINE_RUNNING_RPM_THRESHOLD` 800). Merges `master` → `aztec` are plain; a
  master edit to one of those lines then conflicts and is resolved keeping the aztec value.
  *The gap in that pattern, and the guard:* the governor block is **new**, so the first merge adds
  it to `aztec` without any conflict — with the quad numbers, silently. Those are the numbers
  that hunt on a sharp buggy plant (D13, cross-defaults). Therefore the block carries
  `#define GOV_DEFAULTS_VEHICLE 1  // the GOV_*_DEFAULT values below are the quad bike's` and, after
  `VEHICLE_TYPE_CODE` (which is 1 on `master` / `old-board` and 2 on `aztec` already, used by the
  GCS hardware-version query), `static_assert(GOV_DEFAULTS_VEHICLE == VEHICLE_TYPE_CODE, "speed
  governor defaults belong to another vehicle: set this branch's GOV_*_DEFAULT values")`. The aztec
  build fails until the merge sets the three buggy values and the tag to 2 (tasks 6.1). Later master
  edits to those lines conflict as usual. A selection by `#if VEHICLE_TYPE_CODE` with both sets
  compiled on every branch was considered: equally safe, but it puts buggy numbers into
  `master`'s `Constants.h`, against the pattern that each branch states only its own vehicle's
  values; the tag-and-assert keeps the pattern and closes the gap.
  *`old-board`:* the hand-wired DevKitC-1 reports `VEHICLE_TYPE_CODE` 1 ("1 quad bike = master /
  old-board" in its `Constants.h`), uses the quad's one-hot gear lines with H and the quad's
  steering duties — it is the quad's board, so it takes the quad values by the plain merge and the
  assert passes. Should that board ever drive the buggy, its `VEHICLE_TYPE_CODE` must become 2 for
  the GCS anyway, and the assert then forces the buggy values.
  *Tests and docs:* the native tests carry both sets as literals (they are shared by all branches
  and cannot include `Constants.h`), so both vehicles' plant tests run everywhere (tasks 4.1).
  `WEB_PORTAL_SETUP.md` lists both default columns; the page's help names no default number (the
  page shows the values in use from telemetry, and `index.html` is shared).
- **D13. Plant analysis and the defaults — two plants.** Model for each vehicle: first-order plus
  dead time, speed `v` responds to throttle `u` as `K·e^(−θs)/(τs + 1)` above the clutch take-up
  throttle `u₀`. The two vehicles are different plants; what is observed and what is assumed:

  | Figure | buggy (`aztec`) | quad bike (`master`, `old-board`) | Source |
  |---|---|---|---|
  | engine | 800 cc | 330 cc | **owner**, 2026-11-17 |
  | today's limiter at 10 km/h | "reacts very sharply to the throttle", "a few % easily exceed 10 km/h, then it works in jerks" | "almost worked" | **owner**, 2026-11-17 |
  | limits in use | 10 / 20 / 30 km/h | 10 / 20 / 30 km/h | **owner**, 2026-11-17 |
  | throttle for 10 km/h on the flat | "a few %", read as 2–3 % | ≈ 9 % (checked 6–15 %) | buggy: owner, the number is this proposal's reading; quad: **assumed**, bounded below |
  | take-up throttle `u₀` | ≈ 1 % | ≈ 3 % (2–3 %) | assumed |
  | gain at 10 km/h `K` | design **1.4**, sharp **2.8** m/s per % | design **0.46**, checked 0.23–0.7 m/s per % | derived from the two rows above |
  | gain at 30 km/h | ≈ 0.7 | ≈ 0.31 | assumed (CVT up-shift, drag) |
  | time constant `τ` | 3 s (checked 1.5 s) | 3 s (checked 1.5 s) | assumed, same for both |
  | dead time `θ` | 0.2 s sample + 0.05 s RC / servo + ≈ 0.35 s engine and CVT = **0.6 s** (checked 0.9 s) | same | sample: code; rest assumed |
  | command step | 0.28 % (443 µs default span); ≈ 0.09 % if the mirrored 2200 → 800 µs calibration is stored | 0.28 % | code / project notes (Context) |
  | speed quantum | 0.142 m/s per 200 ms (default calibration; buggy pulse distance unconfirmed) | 0.142 m/s | code, D15 |

  *What "almost worked" says about the quad.* Today's limiter (law in Context, simulated with its
  real constants: band 20 % of the limit ≥ 0.5 m/s, trim Ki 30, rise 10 %/s, slew 80 %/s, floor 0)
  forms a relay-like limit cycle on **every** plant in the model when the stick is far above the
  throttle the limit needs: at full stick 10–20 km/h p-p at a 10 km/h limit for any `K` from 0.14
  to 2.8 m/s per %. Its taper (≈ 180 % per m/s at 10 km/h) is ≈ 30 × the buggy design plant's
  ultimate gain and still ≈ 10 × the quad design plant's (table below). So stability does **not**
  bound the quad's gain — the limiter is unstable on both. What separates the vehicles is how far
  above the needed throttle an operator's stick ends up. With the stick held a little above the
  need, the model gives at 10 km/h:

  | Plant (`K` at 10 km/h) | needs | stick +1 % | +3 % | +6 % |
  |---|---|---|---|---|
  | buggy sharp (2.8) | 2 % | 4.2 km/h p-p | 9.6 | 14.9 |
  | buggy design (1.4) | 3 % | 3.2 | 6.5 | 9.3 |
  | quad steep (0.7) | 6 % | 2.9 | 4.1 | 6.1 |
  | quad design (0.46) | 9 % | 3.1 | 3.9 | 6.0 |
  | quad soft (0.23) | 15 % | 1.1 | 3.3 | 4.7 |

  On the buggy a stick error of a few % is the whole 10 km/h throttle (plus the RC 2 % dead-band),
  so the operator can never hold the stick "just above" the need: jerks. On a quad that needs
  ≈ 6 % or more the same error is a fraction of the need and the limiter only nibbles: "almost
  worked". This bounds the quad's gain **from above**: `K` ≲ 0.7 m/s per % at 10 km/h (≥ ≈ 6 % for
  10 km/h, at least half the buggy design figure) — consistent with 330 cc against 800 cc. It gives
  no lower bound; 0.46 (9 %) is the design value, 0.23 (15 %) is checked. A softer quad only makes
  the quad defaults slower, never unstable.

  *Where it oscillates.* Phase −180° where `θω + atan(τω) = π`: ω ≈ 2.81 rad/s (period 2.2 s) at
  τ 3 s, θ 0.6 s; ultimate gain `Ku = √(1 + (τω)²)/K = 8.49/K`: buggy design 6.1, buggy sharp 3.0,
  quad design 18.5, quad steep 12.1 % per m/s. The old draft's Kp 10 is 1.65 × the buggy design
  `Ku`: a guaranteed limit cycle there.
  *Tuning rule (SIMC, a standard rule for this model).* `τc = 2θ = 1.2 s`, `Kp = τ / (K(τc + θ))`,
  `τI = min(τ, 4(τc + θ)) = 3 s`, `Ki = Kp / τI`.
  - *Buggy:* SIMC gives Kp 1.19 on the design plant and 0.60 on the sharp one. The owner's
    observation puts the buggy between them, nearer the sharp end, so **Kp 1.0, Ki 0.35** (τI
    2.9 s) — the owner's estimate, confirmed: gain margin 2.8 even on the sharp plant, 1.5 in the
    worst corner (sharp and τ 1.5 s) where the previous single default 1.5 / 0.5 had GM 1.0 and
    hunted ≈ 7 km/h p-p. Lower (0.8 / 0.27) buys GM 1.9 in the worst corner at the cost of a slower
    recovery; not chosen, it is the first field step if the buggy hunts (tasks 8.3).
  - *Quad:* SIMC gives Kp 3.6 on the design plant and 2.4 on the steep one. **Kp 3.0, Ki 1.0** (τI
    3 s) — the owner's estimate, confirmed: GM 5.7 on the design plant and ≥ 2.0 over the whole
    bounded range including the worst quad corner (steep and τ 1.5 s). Not raised to 3.6: the quad
    plant is assumed, the upper bound (0.7) is what the margin must cover, and 3.0 already reaches
    90 % of 10 km/h in ≈ 7 s from rest, as fast as the buggy.
  - *Start throttle limit:* buggy 4 %, quad 10 % (owner's estimates, confirmed by the slope runs in
    D4).
  - *Ramps:* acc 1.0 / dec 2.5 on both: on each vehicle's plants the launch overshoot stays
    ≤ 0.6 km/h and the 100 → 50 % step undershoot ≤ 0.7 km/h, so nothing argues for a split.

  Gain margin (GM) and phase margin (PM) of each set on each plant (frequency response of the PI
  and the model above; the 5 Hz sample is inside θ):

  | Plant | buggy set 1.0 / 0.35 | quad set 3.0 / 1.0 | previous single set 1.5 / 0.5 |
  |---|---|---|---|
  | buggy design: K 1.4, τ 3, θ 0.6 | **GM 5.6, PM 72°** | GM 1.9, PM 42° | GM 3.7, PM 66° |
  | buggy sharp: K 2.8 | **GM 2.8, PM 57°** | GM 0.9, PM −6° (unstable) | GM 1.9, PM 42° |
  | buggy faster: K 1.4, τ 1.5 | GM 3.1, PM 80° | GM 1.0, PM 2° | GM 2.0, PM 59° |
  | buggy more dead time: θ 0.9 | GM 3.7, PM 64° | GM 1.2, PM 18° | GM 2.5, PM 54° |
  | buggy worst: K 2.8 and τ 1.5 | GM 1.5, PM 38° | GM 0.5 (unstable) | GM 1.0, PM 2° |
  | buggy at 30 km/h: K 0.7 | GM 11.3, PM 81° | GM 3.8, PM 66° | GM 7.6, PM 78° |
  | quad design: K 0.46, τ 3, θ 0.6 | GM 17, PM 83° | **GM 5.7, PM 74°** | GM 11.4, PM 82° |
  | quad steep: K 0.7 | GM 11.2, PM 80° | **GM 3.7, PM 66°** | GM 7.5, PM 78° |
  | quad soft: K 0.23 | GM 34, PM 87° | GM 11.4, PM 82° | GM 23, PM 86° |
  | quad faster: K 0.46, τ 1.5 | GM 9.3, PM 96° | GM 3.1, PM 82° | GM 6.2, PM 98° |
  | quad more dead time: θ 0.9 | GM 11.3, PM 81° | GM 3.8, PM 66° | GM 7.6, PM 78° |
  | quad worst: K 0.7 and τ 1.5 | GM 6.1, PM 96° | GM 2.0, PM 59° | GM 4.1, PM 92° |
  | quad at 30 km/h: K 0.31 | GM 25, PM 85° | GM 8.4, PM 79° | GM 17, PM 85° |

  *Closed-loop check* (time simulation, `scratchpad/twin_sim.py`: the law above at 25 Hz, 5 Hz
  pulse-counted speed with the real window rule, throttle quantised to 1.22 µs on a 443 µs span,
  static plant maps — buggy design 10 / 20 / 30 km/h at 3 / 6 / 10 % from take-up 1 %, buggy
  sharp at 2 / 3.5 / 5.5 %, quad design at 9 / 16 / 25 % from take-up 3 %, quad steep at 6 / 12 /
  20 % from 2 %, quad soft at 15 / 26 / 38 % from 3 %), launch from rest, stick at 100 %, 60 s,
  ripple over the last 25 s; peak / speed ripple p-p in km/h:

  | Set on plant | 10 km/h | 20 km/h | 30 km/h | 90 % of 10 km/h after |
  |---|---|---|---|---|
  | buggy on buggy design | 10.1 / 0.2 | 20.2 / 0.2 | 30.1 / 0.1 | 7.0 s |
  | buggy on buggy sharp | 10.3 / 0.4 | 20.4 / 0.3 | 30.6 / 0.1 | 4.7 s |
  | buggy on buggy sharp, τ 1.5 | 10.6 / 1.0 | 20.2 / 0.6 | 30.1 / 0.2 | 3.6 s |
  | quad on quad design | 10.1 / 0.1 | 20.1 / 0.1 | 30.1 / 0.1 | 7.0 s |
  | quad on quad steep, τ 1.5 | 10.1 / 0.2 | 20.1 / 0.2 | 30.1 / 0.2 | 3.9 s |
  | quad on quad soft | 10.0 / 0.0 | 20.0 / 0.1 | 30.0 / 0.0 | 12.1 s |
  | θ 0.9 s: buggy on buggy design / quad on quad design | 10.2 / 0.2 · 10.1 / 0.1 | 20.2 · 20.2 | 30.1 · 30.2 | — |

  **Cross-defaults** (a board running the other vehicle's numbers — possible because `master`,
  `aztec` and `old-board` share the code and merge into each other, D12):
  - *Quad numbers on the buggy — dangerous.* Buggy design plant: overshoot to 11.2 km/h, calm.
    Buggy sharp plant: hunts **4.3 km/h p-p** at 10 km/h (throttle ± 2 %), peak 12.6. Sharp and
    τ 1.5: **11–16 km/h p-p** at 10 / 20 / 30 km/h, peaks 18 / 30 / 36 km/h — the owner's jerks
    again, with the governor switched on. Frequency domain: GM 0.9 on the sharp plant. This is
    the case the D12 guard exists for.
  - *Buggy numbers on the quad — safe but sluggish.* Stable everywhere (GM ≥ 11), but 90 % of
    10 km/h only after 13 s (quad steep) to 36 s (quad soft), and on the soft plant the speed sits
    ≈ 0.4 km/h under the setpoint after 60 s with a slow ≈ 1–2 km/h wander at 20–30 km/h. The 4 %
    start limit moves the design quad off on the flat after ≈ 4 s and not at all on a +3 % slope.
  So the direction that must never happen silently is quad → buggy; the guard in D12 makes it a
  build error.

  Resolution checks: one speed quantum × Kp = 0.142 × 1.0 = **0.14 %** on the buggy (half a
  0.28 % step; 1.6 steps of 0.09 % on a 1 400 µs span) and 0.142 × 3.0 = **0.43 %** on the quad
  (1.5 steps of 0.28 %, ≈ 1.9 µs). On the quad the reading's 5 Hz jitter therefore reaches the
  servo as ± 1–2 steps (model throttle ripple 0.7–0.8 % p-p), but each quad step is worth only
  ≈ 0.13 m/s of steady speed and the 5 Hz jitter is far above the plant's bandwidth: speed ripple
  0.1 km/h (D15). A full 10 km/h error asks for 2.8 % on the buggy and 8.3 % on the quad.
  A hill needing as much extra throttle as the 10 km/h throttle itself (buggy +2 %, quad +6 %)
  gives ≈ −4 / +3 km/h at 10 km/h and ≈ −2…−3 / +2 km/h at 30 km/h on both vehicles with their
  own defaults: the price of the low gains. *Output rate:* the previous revision's runs with a
  governor-only opening rate of 5 or 10 %/s instead of `thr_slew` 80 %/s gave identical results at
  low gains and made a mis-tuned loop hunt more (rate limiting adds lag), so no separate rate is
  added (A5).
- **D14. Fixed gains for every limit (no gain scheduling).** The model's gain falls with speed
  (CVT up-shift, drag), so on each vehicle the loop is most fragile at the lowest limit. Gains
  chosen for 10 km/h have more margin at 20 and 30 km/h (buggy GM 2.8–5.6 → 11, quad 3.7–5.7 →
  8.4); the only cost is a slower recovery on hills there, while setpoint changes are paced by the
  ramp anyway. Scheduling (gains or an output ceiling scaled by the limit) would need the plant's
  speed-to-throttle curve per vehicle and gear, which nobody has measured, and would add knobs the
  operator cannot tune by feel. Tuning is done once per vehicle, at the 10 km/h limit, and checked
  at 20 and 30 (tasks 8). If the field shows 30 km/h too soft on hills while 10 km/h is right, a
  single "gain multiplier at the higher limits" is the follow-up.
- **D15. Speed reading used raw.** No low-pass filter and no error dead-band in the law. A filter
  that halves the quantum jitter (e.g. a 2-sample average) adds ≈ 0.1 s of dead time, which costs
  more margin than the jitter costs output: on the buggy one quantum is less than one 0.28 % step
  (Kp 1.0); on the quad it is ≈ 1.5 steps (Kp 3.0), but the quad's step is worth ≈ 0.46 km/h of
  steady speed and the model shows 0.1 km/h of speed ripple from it (D13) — the servo twitches by
  ≈ 2 µs, inside a typical servo dead-band. A dead-band of one quantum would itself create a
  ±0.5 km/h limit cycle at 10 km/h. Window jitter (a 210 ms instead of 200 ms window) adds ≈ 5 %
  noise of the same order as the quantum and is handled the same way. The buggy's pulse distance
  may differ from the quad's 28.4 mm (owner data, tasks 8.1).
- **D16. Throttle resolution is a hard floor on smoothness — mainly on the buggy.** One command
  step is 0.28 % on a 443 µs span (Context): ≈ 1.4 km/h of steady speed per step on the buggy
  design plant (K 1.4), ≈ 2.8 km/h on the sharp one, but only ≈ 0.46 km/h on the quad design plant.
  The integrator hunts between neighbouring steps; the plant's τ smooths that when the servo follows
  every step. With a servo dead-band the servo moves only every few steps; model speed ripple at
  10 / 20 / 30 km/h with each vehicle's defaults:

  | Servo dead-band | buggy design, 443 µs | buggy sharp, 443 µs | buggy sharp, 1 400 µs | quad design, 443 µs |
  |---|---|---|---|---|
  | 0 µs | 0.2 / 0.2 / 0.1 | 0.5 / 0.4 / 0.2 | 0.2 / 0.2 / 0.1 | 0.1 / 0.1 / 0.1 |
  | 2 µs | 1.2 / 0.8 / 0.6 | 2.2 / 1.9 / 1.0 | — | 0.4 / 0.1 / 0.3 |
  | 4 µs | **3.4 / 2.7 / 1.6** | **4.6 / 3.6 / 0.0** | 1.2 / 1.2 / 0.9 | 0.8 / 1.0 / 0.6 |
  | 6 µs | 3.7 / 3.3 / 2.5 | 6.5 / 5.3 / 0.0 | — | 1.4 / 0.0 / 0.9 |

  (km/h p-p, period of several seconds; a 0.0 means the servo parked on one step.) No gain setting
  removes it (lower Ki only slows it). A wider calibrated span helps proportionally — the buggy's
  mirrored 2200 → 800 µs calibration, if stored, already gives ≈ 3 × the resolution — and a
  linkage that uses more of the servo travel for the throttle plate is the real fix if the field
  shows this swing. Not changed in this proposal.

## Risks / Trade-offs
- **Fail-open at full stick** (decision 4): a lost speed signal turns a 100 % "speed" into 100 %
  throttle within ≤ 1.25 s. Mitigated only by the warning (web immediately, GCS after 2 s) and the
  blend; accepted by the owner. The same holds from boot until the first wheel pulse (A10, owner
  decision 2026-11-17) — a launch with the stick slammed to 100 % straight after boot opens fully
  until the wheel has turned ≈ 3 cm, then the governor takes over from that throttle and the
  ramp/PI bring it back.
- **No downhill hold.** Throttle at idle is all the governor has; engine braking through a CVT is
  weak. The at-idle badge tells the operator to brake.
- **Plant figures are partly assumed** (D13). Observed: the engine sizes, "a few % exceed 10 km/h"
  and "sharp" on the buggy, "almost worked" on the quad. Assumed: every τ, θ and take-up, the
  buggy's exact gain within 1.4–2.8 m/s per %, and the whole quad gain (0.46, bounded above at
  ≈ 0.7 by the "almost worked" argument, not below). If a real plant is steeper or faster than its
  vehicle's worst corner, the defaults hunt at 10 km/h and the field procedure lowers Kp; if much
  softer, they are sluggish and it raises them. The 5 Hz sample and the ≈ 0.6 s dead time cap the
  useful bandwidth either way: the loop period at which it oscillates is ≈ 2.2 s, and nothing
  faster than that can be tuned in.
- **Wrong vehicle's defaults on a board** (D12, D13 cross-defaults). The quad's numbers on the buggy
  hunt (model 4 km/h p-p on the sharp plant, up to 16 km/h p-p in its worst corner); the buggy's on
  the quad are only sluggish. Guarded at build time by `GOV_DEFAULTS_VEHICLE ==
  VEHICLE_TYPE_CODE`; not guarded against an operator copying values from one vehicle's page to
  the other — the page shows the values in use, and the docs list both default columns.
- **Throttle resolution** (D16): a servo dead-band produces a slow speed swing of a few km/h at
  10 km/h on the buggy that tuning cannot remove; on the quad it stays ≈ 1 km/h or less in the
  model.
- **CVT launch.** Below the clutch engagement speed the governor cannot hold a setpoint; the 1.4 m/s
  minimum and the start throttle limit shape a launch, but very slow crawling is not possible with
  it on. A clutch that takes up late and snatches gives a lurch up to the start throttle limit
  (buggy 4 %, quad 10 % by default).
- **Take-over overshoot.** Entering ACTIVE from a high direct throttle (A10 first launch, speed
  signal back, limit appearing at speed) overshoots before the throttle has closed — model peak
  after a full-stick first launch at a 10 km/h limit ≈ 20–21 km/h on the buggy, ≈ 13–17 km/h on the
  quad (D8).
- **Autopilot modes (A12).** In Auto / Guided / RTL ArduPilot's throttle is already a speed
  controller output; the governor would read it as a setpoint request and cascade a second loop.
  The owner does not use those modes; documented, not handled.
- **Launch with the defaults.** From rest the output grows from idle through `P + I` as the
  setpoint ramps (P alone at the 5 km/h minimum setpoint: 1.4 % on the buggy, 4.2 % on the quad);
  on each vehicle's design plant it moves off within ≈ 2 s. On a slope it waits until the
  integrator reaches the take-up throttle, at most the start throttle limit; the field procedure
  checks this first.
- **A setting change while driving** moves `P` at once; the output rate limit bounds the effect.

## Migration Plan
New keys in the existing namespace, switch default off: flashing 1.5.35 changes nothing until the
operator turns the governor on. Rollback: an older firmware ignores the `gov_*` keys; the switch
being on has no effect there. Defaults are compiled per branch (D12) and apply only to absent keys
(D5); no stored value is migrated or rewritten.

## Owner Decisions
- **A10, 2026-11-17 — variant A.** "No wheel pulse since boot" counts as an invalid speed: direct
  throttle until the first pulse (fail-open, banner and GCS warning as for any invalid speed), then
  the governor takes over without a step (D8). Rejected variant B ("standing still, speed 0"): the
  governor would act from the first moment, but a sensor dead since boot would then cap the
  throttle at the start throttle limit without any warning, because the firmware cannot tell "not
  yet moved" from "dead since boot".
- **Per-vehicle defaults, 2026-11-17.** The buggy (800 cc, "reacts very sharply to the throttle")
  and the quad bike (330 cc, "almost worked" with today's limiter) get separate compiled defaults
  (D2, D12, D13); limits in real use are 10 / 20 / 30 km/h on both.

## Open Questions
- None blocking. The plant data of tasks 8.1 (throttle % for 10 / 20 / 30 km/h, take-up, the +1 %
  step response, the stored throttle calibration µs) replaces the assumed figures of D13 per
  vehicle.
