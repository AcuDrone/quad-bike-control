# Design: Speed limiter — configurable floor, proportional taper band and integral trim

## Context
- `VehicleController::applySpeedLimit(demandPct)` runs on every control-loop iteration for the
  MAVLink path (`update()`) and the web path (standing web demand). It also runs once on each web
  throttle command. It returns `min(demand, ceiling)`.
- Today's ceiling:
  - the target is a proportional taper: 100 % up to `limit − 1.4 m/s`, linear down to 7.5 % at the
    limit, 7.5 % above it;
  - the applied ceiling follows the target at no more than 80 %/s, with dt capped at 1 s;
  - the first evaluation adopts the target directly;
  - on "no limit" or an invalid sensor the ceiling is reset to 100 % and the demand passes
    through (fail-open).
- Throttle percent maps linearly from the calibrated idle to full (`ThrottleController::percentToUs`).
  **0 % is the calibrated idle position**, not "below idle". On a CVT vehicle, idle disengages the
  drive belt.
- Speed sensor: PCNT sampled every 200 ms. With the default 70 pulses/rev and 1990 mm, one pulse
  per sample is **0.142 m/s (0.51 km/h)**. This quantum bounds how narrow a useful band can be.
- Field case: limit 2.78 m/s (10 km/h), measured 8.3 m/s (30 km/h), sensor valid. The proportional
  target is pinned at the 7.5 % floor and that throttle holds 30 km/h. Nothing in the law can go
  lower.

## Goals / Non-Goals
- Goals:
  - the limit holds (steady state → the limit) whatever the engine torque, CVT ratio or grade,
    as long as some throttle between the floor and 100 % holds it;
  - at low limits, authority is withdrawn close to the limit and not at half of it;
  - no servo snap, and the driver's own throttle moves are never slowed;
  - the saturated case (the floor itself is too much) is visible in the web console, during the
    event and for a while after it, because the serial log is not readable in the field;
  - the same behaviour on both vehicles.
- Non-goals:
  - braking. Downhill, where idle alone overspeeds, the limiter saturates and the console shows
    it. It does not brake. That is explicitly out of scope;
  - a MAVLink notification of saturation (e.g. `STATUSTEXT`). Not part of this change (see Open
    Questions);
  - any change to where the limit comes from (`SPEED_MAX` only, RAM only);
  - the gear-change boost, which stays outside the limiter.

## Decisions

### D1 — Floor: default 0 %, persisted, 0–20 %
- `SPEED_LIMIT_FLOOR_DEFAULT_PCT` = 0.0, `SPEED_LIMIT_FLOOR_MIN_PCT` = 0.0,
  `SPEED_LIMIT_FLOOR_MAX_PCT` = 20.0.
- **Why 0 %:** a floor above the throttle that holds the limit makes the limit unreachable. The
  field case proves that 7.5 % is already above it on the buggy. 0 % is the only value that is
  below that throttle on every vehicle and gradient where the limit is physically enforceable
  without brakes.
- **Why still configurable:** the floor is the one thing that depends on the vehicle and the
  driver, not on physics. On the quad, closing to idle mid-corner gives noticeable engine
  braking. An operator may prefer a few percent of residual drive there and accept the limit
  holding less tightly. The 20 % cap keeps any floor well below cruising throttle: above it, the
  floor itself would become a hidden speed floor. The saturation indicator (D6) shows when a
  chosen floor is too high.
- **Why "no hard cut" still holds at 0 %:** the old argument against cutting to zero was about a
  *step*. The ceiling still approaches the floor through the taper and the 80 %/s slew. Going
  100 → 0 takes at least 1.25 s.
- **NVS:**
  - namespace `"speed"`, key `thr_floor`, float, percent;
  - owned and written by `VehicleController`, read in `initCAN()` next to `steer_sca_base`. This
    is the same "foreign key in a related namespace" precedent as `steer_sca_base` in
    `"steering"`;
  - why `"speed"`: the setting belongs to the speed card in the UI and to the speed-limit domain.
    `SpeedSensor` removes keys only one by one (`lim_on`, `lim_kmh`) and never `clear()`s the
    namespace;
  - why not the `lim_` prefix: the retired keys are deleted on every boot. A new key under the
    same prefix would make that cleanup line easy to misread, or to "tidy" into a prefix wipe.
    `thr_floor` is unambiguous and fits the 15-character limit;
  - why not the `"throttle"` namespace: that namespace is the throttle calibration, which is
    saved and replaced as a unit by `ThrottleController`;
  - a stored value that is NaN or outside 0–20 % is ignored and logged, and the default is used;
  - a firmware downgrade leaves an orphan `thr_floor`, which older firmware ignores (it uses its
    compiled 7.5 %). Harmless.
- `SPEED_LIMIT_FLOOR_PCT` is removed. Every user switches to the runtime value `limiterFloorPct_`.

### D2 — Proportional taper band
```
band(L) = clamp(SPEED_LIMIT_TAPER_FRACTION × L, SPEED_LIMIT_TAPER_MIN_MS, SPEED_LIMIT_TAPER_MAX_MS)
        = clamp(0.20 × L, 0.5 m/s, 1.4 m/s)
```
| Limit | 0.20 × L | Band | Taper starts at |
|---|---|---|---|
| 1.39 m/s (5 km/h) | 0.28 | **0.50 m/s** (min) | 0.89 m/s (3.2 km/h) |
| 2.78 m/s (10 km/h) | 0.56 | **0.56 m/s** (2.0 km/h) | 2.22 m/s (8.0 km/h) |
| 5.56 m/s (20 km/h) | 1.11 | **1.11 m/s** (4.0 km/h) | 4.44 m/s (16.0 km/h) |
| 11.1 m/s (40 km/h) | 2.22 | **1.40 m/s** (max, 5.0 km/h) | 9.71 m/s (35.0 km/h) |

- **0.20:** the limiter takes over in the last fifth of the limit. That is enough lead to absorb
  the servo, engine and CVT lag (the ceiling also slews), without the "starts at half the limit"
  feel of today's 10 km/h case.
- **0.5 m/s minimum:**
  - it is about 3.5 speed quanta (0.142 m/s), so a single-pulse jitter moves the target by at
    most about 28 % of the span rather than a large share of it;
  - the slew then smooths that to ≤ 16 % per 200 ms sample;
  - a narrower band turns the taper into a near-bang-bang controller that hunts.
- **1.4 m/s maximum:** today's value, proven on the quad at road-speed limits. Above 7 m/s the
  behaviour is unchanged.
- The band is computed on every evaluation from the current limit. A `SPEED_MAX` change moves
  the band at once; the slew handles any resulting ceiling step.

### D3 — Integral trim on the ceiling (the update law)
Symbols:
- `v`: measured speed (m/s);
- `L`: limit;
- `B = band(L)`;
- `F`: floor (%);
- `D`: the demand passed in;
- `C`: the applied ceiling (state);
- `T`: the trim (state, %);
- `t₀`: the time of the last evaluation (state, 0 = none).

Constants:
- `Ki` = `SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS` = **30 %/s per m/s**;
- `Kr` = `SPEED_LIMIT_TRIM_RISE_PCT_S` = **10 %/s**;
- `S` = `SPEED_LIMIT_CEILING_SLEW_PCT_S` = 80 %/s (unchanged);
- `ε` = `SPEED_LIMIT_BIND_MARGIN_PCT` = 1 %.

```
if L <= 0 or !sensorValid:            # unchanged fail-open
    C = 100; T = 0; t₀ = 0; return D

P(v) = 100                                 if v <= L − B
     = 100 − (100 − F) × (v − (L − B)) / B if L − B < v < L
     = F                                   if v >= L

if t₀ == 0:                           # first evaluation after any reset
    T = 0; C = clamp(P(v), F, 100); t₀ = now; return min(D, C)

dt = min((now − t₀) / 1000, 1.0); t₀ = now
e  = v − L                            # > 0: over the limit
if v <= L − B:      T = 0                                  # outside the band: taper intact
elif e > 0:         T −= min(Ki × e × dt,  S × dt)          # over: remove authority
elif D >= C − ε:    T += min(Ki × (−e) × dt, Kr × dt)       # under, binding: give some back
else:               (hold T)                                 # driver below the ceiling
T = clamp(T, 0, 100 − F)

target = clamp(P(v) + T, F, 100)
C += clamp(target − C, −S × dt, +S × dt)                     # unchanged slew on the ceiling
C  = clamp(C, F, 100)
return min(D, C)
```

Behaviour:
- **Approach from below:** `T` = 0 on entry to the band, so the first approach meets the full
  taper. That is the least overshoot the law can give.
- **Steady state:** `T` stops changing only when `v = L` (or when it is clamped). Let `u*` be
  the throttle that holds `L`. With `F ≤ u* ≤ 100`, the loop settles at `v = L` with
  `C = F + T = u*`. The proportional-only droop (today the vehicle settles where `P(v) = u*`, up
  to one band below `L`) is removed. `u* < F` is the saturated case (D6). `u* > 100` is a grade
  the engine cannot climb at `L` anyway.
- **Over the limit:** the ceiling is already `F + T` and keeps descending at `Ki × e`:
  - 1.4 m/s excess → 42 %/s, so `T` goes 100 → 0 in ≈ 2.4 s (target 2–3 s);
  - 0.3 m/s excess → 9 %/s;
  - the field case (5.6 m/s excess) → capped at the slew, 80 %/s.
- **Recovery:** when under the limit and binding, at most 10 %/s (≥ 10 s for the full range).
  That is slow compared with the 80 %/s slew, so recovery is never the source of a surge.
  Near the limit it is slower still: 0.17 m/s deficit → 5 %/s.
- **Anti-windup:**
  1. `T` is clamped to [0, 100 − F], so `C` stays within [F, 100];
  2. `T` rises only while the limiter binds (`D ≥ C − ε`);
  3. it is reset outside the band, on "no limit", on an invalid sensor and on the first
     evaluation;
  4. each step is capped at the slew, so `T` can never run more than the slew ahead of `C`.

  The "lift off and re-apply meets a stale low ceiling" case is ruled out by construction. The
  restricting part of the ceiling is `P(v)`, which has no memory. `T` only ever *adds* authority
  and never accumulates while the driver is below the ceiling.
- **Lift-off while over the limit:** `T` keeps falling because `e > 0`. That lowers a ceiling the
  driver is not using. On re-apply, the ceiling is lower, which is the conservative side while the
  vehicle is still over the limit.
- **Multiple calls per loop** (a web command plus `update()`): each call integrates its own `dt`
  from `t₀`, so the total is the same as a single call.
- **Floor change while limiting:** `T`'s clamp and `C`'s clamp re-apply on the next evaluation.
  The slew limits any resulting step.

### D4 — Pure law, host-tested
The arithmetic lives in a header-only `include/SpeedLimiterLaw.h`, with no Arduino and no
`Constants.h` dependency. It defines:
- `struct SpeedLimiterParams`;
- `struct SpeedLimiterState { C, T, t0Valid }`;
- `float speedLimiterBand(L, ...)`;
- `float speedLimiterStep(state&, params, D, v, L, dtS, first)`.

`applySpeedLimit()` keeps the logging, `millis()`, the sensor checks and the fail-open branch. It
calls `speedLimiterStep()`. A Unity suite `test/test_speed_limiter/` covers the law (tasks 1.2).
This follows the `SteerSensorDir.h` / `test_steer_sensor_dir` precedent.

### D5 — Telemetry and web
- `speed_limit_ceil` keeps carrying the final applied ceiling `C`. Its meaning ("100 = withdrawing
  nothing") is unchanged.
- One key is added, `speed_limit_floor` (%, one decimal). It is needed so the settings input
  syncs to the stored value, as `speed_ppr` / `speed_circ_mm` do. It also lets the UI show "at
  floor" when `speed_limit_ceil` = `speed_limit_floor`.
- `T` is **not** exported. It is a derived internal quantity with no operator action attached.
  The case the operator does need to see, saturation, gets its own explicit keys (D6) instead of
  being left to the operator to infer from `speed_limit_ceil`, `speed_limit_floor`,
  `vehicle_speed` and `speed_limit_kmh`.
- Web command `set_speed_limit_floor` (float %). It is validated against 0–20, persisted
  immediately and answered "Speed-limiter floor set to X %". It is routed like `speed_cal_ppr`
  and accepted regardless of the input source.

### D6 — Saturation indicator
The operator cannot read the serial debug log in the field. The saturation case is therefore
surfaced in the web console, and **not** as a `Debug::printf` line. The existing rate-limited
log of limit *changes* ("Log a limit change once") stays as it is.

**Condition (unchanged thresholds).** The raw condition is:
- a limit is in force and the sensor is valid;
- `C ≤ F + 0.5`;
- `v > L + SPEED_LIMIT_SAT_MARGIN_MS` (0.3 m/s ≈ 1 km/h, about two speed quanta).

`speed_limit_sat` becomes true once the raw condition has held continuously for
`SPEED_LIMIT_SAT_HOLD_MS` = 2000 ms. It becomes false on the first evaluation where any part of
it fails, including "no limit" and an invalid sensor. There is no release debounce.

**Hold.** `speed_limit_sat_held` is true while `speed_limit_sat` is true, and for
`SPEED_LIMIT_SAT_UI_HOLD_MS` = 10 000 ms after it last cleared. A new saturation during the hold
restarts it. The operator looks at the screen after the event, typically after slowing down. By
then the live flag has already cleared, so without the hold the badge would be gone.

**Why the hold is in the firmware, not in the page:**
- every client agrees: two open browsers, a phone and a laptop, show the same state;
- a client that connects, reloads or reconnects after a WiFi drop during the hold still sees it.
  A page-side timer only knows about frames that page itself received;
- the hold is measured on the firmware clock, so it does not depend on the browser's frame rate
  or on a tab that was in the background;
- it costs one timestamp and one boolean key.

**Implementation shape:**
- state: `satSinceMs_` (start of the raw condition, 0 = not holding), `satActive_`,
  `satClearedMs_` (0 = never);
- the hold is evaluated from `millis()` at the time telemetry reads it. It therefore expires on
  time even if `applySpeedLimit()` stops being called, e.g. while no input source drives the
  throttle;
- the reset paths ("no limit", invalid sensor) clear `satActive_` and start the hold. They do not
  erase it.

**Presentation.** Both cards that render the limiter row (the Diagnostics Speed card and the
Speed sensor settings card) share one rendering path. The row is shown while
`speed_limit_ceil` < 100 **or** `speed_limit_sat_held` is true. When `speed_limit_sat_held` is
true, the row is drawn as a warning badge in the console's standard warning styling (the existing
`--warning` colour, as used by the attention list and the `.warning` box). It reads:
- "Limiter at floor, speed above limit" while `speed_limit_sat` is true;
- "Limiter was at floor, speed above limit" during the hold after it clears.

Otherwise the row is the existing "Limiting X %" text. In the field case the badge would have
appeared within about 2 s and stayed up for 10 s after the driver slowed down.

## Alternatives considered
- **Hard cut with hysteresis** (throttle 0 above `L`, restore below `L − h`):
  - rejected: it is the step the taper was introduced to remove;
  - on a heavy CVT vehicle it gives a lurch at every crossing, and a limit cycle with amplitude
    about `h` plus the lag overshoot;
  - its steady state is an oscillation, not `L`. It would also need its own hysteresis tuning per
    limit.
- **Pure P with a lower floor only (step 1 alone):**
  - this fixes the field case on flat ground, if the 0 % hypothesis holds;
  - but a proportional controller always settles where `P(v) = u*`, up to one band below `L`
    (up to 1.4 m/s ≈ 5 km/h at high limits), and the droop grows with load or grade;
  - narrowing the band to cut droop raises the gain until the 0.142 m/s quantum makes it hunt;
  - the integral trim removes the droop without raising the gain.
- **`min(P(v), I)` with an integrator that starts at 100 % and descends above the limit** (the
  form first sketched for this change):
  - rejected because it cannot do what was asked of it. Above the limit `P(v)` is already at
    the floor, so `min(P, I)` equals the floor whatever `I` does: the integrator adds nothing
    there;
  - below the limit `I` winds back towards 100 %, so the steady state is still the proportional
    droop point;
  - its only effect is a slower re-opening after an overshoot, which the slew already provides;
  - the additive trim (D3) keeps every property sketched for that form: it descends at a rate
    proportional to the excess down to the floor, recovers gradually and never faster than the
    slew, stays within [floor, 100], fails open, and does not accumulate below the ceiling. On
    top of that it actually converges to `L`.
- **Full PID on the throttle (replace the demand with a speed controller):**
  - rejected because the limiter must stay a *ceiling*: `min(demand, C)` guarantees the driver
    (or autopilot) is never given more throttle than they asked for, and their own movements pass
    through at full rate;
  - a PID that owns the throttle would accelerate on its own to reach `L` and would fight the
    autopilot's speed controller;
  - a D term on a 5 Hz, 0.142 m/s-quantised speed is noise.
- **Fixed (compile-time) floor of 0 %:** simpler, but it removes a per-vehicle comfort trade-off
  (engine braking at idle on the quad) for no safety gain. It would also need a reflash to tune
  in the field. The setting costs one key, one command and one input.
- **A separate `"limiter"` NVS namespace:** workable. It was rejected only for consistency with
  the UI grouping and the `steer_sca_base` precedent. No technical difference.
- **A rate-limited serial log line for saturation** (the first version of D6): rejected. The serial
  log is not readable in the field, so the line would not have helped in the case that prompted
  this change. The console indicator replaces it rather than duplicating it.
- **No new keys; let the page infer saturation** from `speed_limit_ceil`, `speed_limit_floor`,
  `vehicle_speed` and `speed_limit_kmh`: rejected. The page would have to re-implement the
  2000 ms debounce on its own frame timing, and every client would hold its own copy of the state.
- **Hold only in the page:** rejected for the reasons given in D6 (clients disagree, and late
  joiners and reconnects miss the event).

## Risks / Trade-offs
- **Integral overshoot after a long climb.** `T` can be large (e.g. 60 %) at the crest. Going
  downhill, the ceiling then falls at `Ki × e`, about 1 s for a 2 m/s excess, plus the slew.
  → `Ki` is three times faster down than the 10 %/s rise cap. The console's saturation badge
  shows the residual. Braking is the real fix and is out of scope.
- **Engine braking at 0 %.** Closing to idle at the limit is felt more on the quad. → The floor is
  configurable up to 20 %, and the approach is slew-limited.
- **Speed-quantum jitter at the limit** makes `T` integrate up and down by a few %/s. → It
  averages to zero. The 1 % binding margin and the slew keep the servo calm. Bench check 9.4.
- **Hypothesis not confirmed.** If 0 % still holds above 10 km/h on the buggy (fast idle cold,
  CVT engaged at idle), the saturation badge appears in the console. The remedy is then
  mechanical (idle speed, CVT engagement) or braking, which is a separate change.
- **Only visible while someone is watching the console.** The indicator is not logged and not sent
  to the ground station. An event older than the 10 s hold leaves no trace. → The hold covers
  the "look after slowing down" case. A persistent record or a MAVLink notification is an open
  question, not part of this change.
- **Badge flicker at the threshold.** Speed-quantum jitter around `L + 0.3 m/s` can clear and
  re-enter the live flag. → Entry needs a full 2 s of the condition. During the hold the badge
  stays up, so the operator sees "at floor" / "was at floor" alternate, not the badge appear and
  disappear.

## Migration Plan
- No migration. An absent `thr_floor` reads as the default 0 %.
- Rollback: flash the previous firmware. The orphan key is ignored. The behaviour returns to
  7.5 % and the proportional-only law.

## Open Questions
- Should the quad ship with a non-zero stored floor (e.g. 5 %) after its own bench test? Not
  decided here; the default is 0 % on both vehicles.
- Should saturation also be reported to the ground station (MAVLink `STATUSTEXT`)? Deliberately
  not part of this change. It is a separate question for the user.
