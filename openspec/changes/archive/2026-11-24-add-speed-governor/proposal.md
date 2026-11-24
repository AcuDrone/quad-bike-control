# Change: Speed governor — the throttle stick commands a speed (web switch, default off)

## Why
The owner reports that limiting speed "by throttle position regulates badly". The firmware today
has no speed control, only a maximum-speed **throttle ceiling** (`applySpeedLimit()`,
`src/VehicleController.cpp:1093-1166`, law `include/SpeedLimiterLaw.h`): the operator's throttle
passes through untouched until the measured speed enters a taper band just below the limit
(20 % of the limit, 0.5–1.4 m/s), then a proportional taper plus an integral trim pull a ceiling
down over the demand, slew-limited at the per-vehicle `thr_slew` (80 %/s). Below the band the
operator gets exactly the stick; the stick position never means a speed. With a CVT the same
throttle gives very different speeds depending on the gear, the slope and the load, so the operator
has to chase the speed with the stick, and near the limit the ceiling and the stick fight each
other. The owner wants the stick to command a speed and the firmware to hold it.

Field information (owner, 2026-11-17): the limits actually used are **10, 20 and 30 km/h** on both
vehicles. On the **buggy** (800 cc) "a few % of throttle easily exceed the 10 km/h limit, and then
it works in jerks" — it "reacts very sharply to the throttle". On the **quad bike** (330 cc) today's
limiter "almost worked". The buggy is a very high plant gain at low speed (≈ 1.4–2.8 m/s per % of
throttle, design D13) seen through a 5 Hz speed sample: at a 10 km/h limit today's taper band is
0.56 m/s wide, so the ceiling moves 100 % / 0.56 m/s ≈ 180 % per m/s — 30 to 60 times the gain at
which that plant starts to oscillate (3–6 % per m/s, D13). The limiter is a bang-bang switch at
that limit on both vehicles in the model; it was tolerable on the quad because the quad needs
several times more throttle for the same speed, so the operator's stick error is a small part of
it (D13 derives from this an upper bound on the quad's gain). The governor therefore gets **two
default sets**, one per vehicle, each derived from that vehicle's plant figures (D13) so that it
does not repeat the limiter's mistake on either.

## What Changes
- **Speed governor (new, opt-in).** When the governor switch is ON **and** an effective speed limit
  exists, the throttle input (RC channel 2 above centre, or the web throttle slider) is a **speed
  setpoint**: stick 0..100 % maps to 0..the effective limit. A PI loop on the speed error from the
  wheel-speed sensor, with a setpoint ramp, anti-windup and a start throttle limit, computes the
  throttle. The gains are deliberately low and **per vehicle** — buggy Kp 1.0 / Ki 0.35, quad Kp 3.0 /
  Ki 1.0, each with a gain margin of ≥ 2.8 (buggy) / ≥ 3.7 (quad) on its own vehicle's plant range at a
  10 km/h limit and ≥ 1.5 / ≥ 2.0 in the worst checked corner (design D13) — and the same for every limit (D14).
  The quad's values are compiled on `master` / `old-board`, the buggy's on `aztec`; a build-time
  check stops a merge from carrying the quad's values onto the buggy (design D12).
  Speeds above the limit cannot be commanded. The effective limit is the one the limiter already
  uses: `SPEED_MAX` on `master` / `old-board`; on `aztec` min(`SPEED_MAX`, gear ceiling L 45 /
  H 75 / R 20 km/h). Pure law in a new header `include/SpeedGovernorLaw.h`, native tests in
  `test/test_speed_governor` (design D2–D6).
- **No brake.** The governor can only close the throttle down to the calibrated idle. It cannot hold
  a speed downhill; a web-only badge "Регулятор на холостому, швидкість вища за задану" / "Governor at
  idle, speed above setpoint" shows when that happens. Brake assist is a later change, after the
  brake actuator's dead-travel delay is fixed.
- **Enable switch.** "Регулятор швидкості" / "Speed governor" ON/OFF in a new Calibration card,
  persisted per vehicle in NVS `"speed"` / `gov_en`, **default OFF**. With the switch OFF, or ON with
  no effective limit, the throttle path is today's, bit for bit (direct throttle, with the ceiling
  limiter where a limit exists).
- **One owner of the throttle.** While the governor is **active** it alone computes the throttle and
  the ceiling limiter stands down (reset, `speed_limit_ceil` 100, its saturation badge cleared). In
  every other governor state today's `applySpeedLimit()` path runs unchanged (design D7).
- **Speed sensor invalid → direct throttle, no restriction (fail-open, owner decision).** The
  hand-over is bumpless in both directions: governor → direct moves the throttle at the per-vehicle
  throttle rate (`thr_slew`, default 80 %/s) from the governor's last output to the stick's value;
  direct → governor starts its output at the current throttle, the setpoint ramp at the measured
  speed and the integrator at the current throttle but no higher than the start throttle limit, so a
  take-over from a high direct throttle closes toward the governed value at `thr_slew` instead of
  unwinding slowly (design D8). The web page shows a warning banner and the GCS gets a `STATUSTEXT`
  warning (design D11).

  > **Risk, stated plainly.** If the wheel-speed signal is lost while driving, the governor stops
  > governing and the stick becomes a plain throttle again, exactly as it is today without the
  > governor. **Stick at 100 % then means full throttle**, not "the speed limit". The switch from
  > "speed" to "throttle" happens at most 80 %/s (thr_slew), and the operator is warned on the web
  > page and the ground station, but nothing else holds the vehicle back. The same applies on every
  > boot until the first wheel pulse (assumption A10). When the governor takes over from a high
  > direct throttle it inherits at most the start throttle limit and closes toward its own value at
  > `thr_slew`; the speed still overshoots the setpoint first (model peak at a 10 km/h limit after
  > a full-stick first launch: ≈ 20 km/h on the buggy, ≈ 13–17 km/h on the quad, design D8 / D13).
- **Brake, gear change, fail-safe.** Braking by the operator always wins: on RC the combined
  channel 2 below centre gives a throttle of 0, which the governor treats as "stop" (setpoint 0,
  integrator reset, idle at once); on the web, while a brake is commanded (≥ 5 %) the governor holds
  idle and resets (A6). In N / UNKNOWN gear, during a gear change (boost PID or idle hold), during
  throttle calibration and in fail-safe the governor is bypassed and reset; today's code owns the
  throttle there (A7).
- **Settings** (Calibration card, NVS `"speed"`, per vehicle, persisted first then applied; a
  compiled default applies only while its key is absent — stored values are never replaced):
  | Setting | NVS key | Range | Default, buggy (`aztec`) | Default, quad bike (`master`, `old-board`) |
  |---|---|---|---|---|
  | governor on/off | `gov_en` (bool) | — | off | off |
  | proportional gain Kp | `gov_kp` (float, % per m/s) | 0.2–10 | 1.0 | 3.0 |
  | integral gain Ki | `gov_ki` (float, %/s per m/s) | 0–5 | 0.35 | 1.0 |
  | setpoint acceleration | `gov_acc` (float, m/s²) | 0.3–3.0 | 1.0 (3.6 km/h per s) | 1.0 |
  | setpoint deceleration | `gov_dec` (float, m/s²) | 0.5–5.0 | 2.5 (9 km/h per s) | 2.5 |
  | start throttle limit | `gov_launch` (float, %) | 1–40 | 4 | 10 |
  Fixed constants: stick dead-band 5 %, minimum setpoint 1.4 m/s (5 km/h, at most half the limit),
  stand-still threshold, saturation margins (design D4, D5). The output rate is the existing
  `thr_slew` (A5); no new rate setting. Previous draft (not shipped): Kp 10 (1–50), Ki 5 (0–30),
  launch cap 30 (10–60) — on the buggy's plant Kp 10 is above the oscillation gain (D13). Previous
  revision (not shipped): one set for both, Kp 1.5 / Ki 0.5 / start throttle limit 6 % — marginal
  on a sharp buggy plant (gain margin 1.0 in its worst corner) and too slow and too weak to launch
  on a slope on the quad.
- **What the governor can and cannot do at a 10 km/h limit** (from the plant models in D13, each
  vehicle with its own defaults; numbers to be confirmed on the vehicles, tasks 8):
  - *Can:* hold 10 km/h on flat ground within ≈ ±0.5 km/h when the throttle servo follows every
    command step; launch to 10 km/h with ≤ 0.6 km/h overshoot when the clutch takes up smoothly;
    reach 90 % of 10 km/h in ≈ 4–7 s from rest and follow stick changes without hunting; work the
    same way at 20 and 30 km/h with more stability margin (D14).
  - *Cannot:* brake — downhill it closes to idle and shows the at-idle badge. It is slow on purpose:
    a slope that needs as much extra throttle as the whole 10 km/h throttle (buggy ≈ +2 %, quad
    ≈ +6 %) dips the speed by ≈ 4 km/h for several seconds, and cresting it overshoots by ≈ 3 km/h.
    It cannot be finer than the throttle servo: the command has ≈ 0.28 % steps on the default
    calibration (443 µs span, both branches compile it; D16) — on the buggy each step is worth
    ≈ 1.4–2.8 km/h of steady speed and a servo dead-band of ≈ 4 µs turns that into a slow swing of
    ≈ 3–5 km/h peak-to-peak (period of several seconds) that no tuning removes; if the buggy's
    NVS holds its mirrored 2200 → 800 µs calibration (project notes; not in the repo) the step is
    ≈ 0.09 % and the swing ≈ 1 km/h. On the quad each step is worth ≈ 0.5 km/h and the swing stays
    ≤ ≈ 1 km/h. Below ≈ 5 km/h, and in H around the CVT clutch take-up, it cannot hold a speed
    smoothly — the clutch's own gain is even higher there and a late, snatchy take-up gives a lurch
    bounded only by the start throttle limit (buggy 4 %, quad 10 %). After a take-over from a high
    direct throttle (first launch after boot under A10, speed signal back, limit appearing at
    speed) the speed overshoots before the governor has closed the throttle (model peak after a
    full-stick first launch: ≈ 20 km/h on the buggy, ≈ 13–17 km/h on the quad).
  - *Operator recommendation (not an interlock):* drive the low limits (10 and 20 km/h) in **L** on
    the buggy. L puts the clutch take-up at a lower road speed, so the 5–10 km/h stick range sits
    above the take-up region and the plant gain per % of throttle is lower.
- **Web.** New card "🎯 Регулятор швидкості" / "🎯 Speed governor" in the Calibration section right
  after "Speed limit and steering": the ON/OFF button, five setting sliders (one command on release, − / + step buttons), hints and help.
  The Diagnostics "Speed" card and the Calibration speed-limit card show the governor state
  (off / no limit / no speed — direct throttle / bypassed / active), the setpoint and the governor
  output, the no-speed banner and the at-idle badge. The web throttle slider shows the setpoint in
  km/h while the governor is active. Commands `set_gov_enabled`, `set_gov_kp`, `set_gov_ki`,
  `set_gov_acc`, `set_gov_dec`, `set_gov_launch`. Ten telemetry keys (≈ 0.2 KB of the ≈ 0.83 KB
  headroom at 9216).
- **GCS.** `STATUSTEXT` WARNING "Gov: NO SPEED - direct throttle" when the governor should act but
  cannot (switch on, limit present, speed invalid, stick above the dead-band for 2 s), repeated every
  10 s while it lasts; INFO "Gov: speed OK - governing" when the speed returns. No new MAVLink
  message or parameter; ArduPilot sees the governed throttle exactly as it sees today's limited one.
- **Docs.** `WEB_PORTAL_SETUP.md`, `MAVLINK_SETUP.md` (governor section + the two `STATUSTEXT`
  texts), `../../API/README.md` (version line, `STATUSTEXT` row), `Changelog.h` 1.5.35 uk + en.
- **Version.** Firmware **1.5.35** on all three branches (no in-flight change claims it).

### Assumptions (the owner may overrule any of them)
- **A1. Web throttle is governed too.** The web throttle slider is the same setpoint (0..100 % →
  0..limit), for consistency; the web brake slider stays a brake. The code makes this easy: both
  paths already go through one function (`applySpeedLimit()`).
- **A2. Gains and ramps are per-vehicle web settings with conservative per-vehicle defaults**
  (table above; owner decision 2026-11-17 to split them), derived in design D13 from the owner's
  observations and stated assumptions (plant figures marked "owner" or "assumed" there; tasks 8.1
  measures them). Kp, Ki and the start throttle limit differ; the ramps and every range are common.
  Mechanism: the established branch-specific pattern — `master`'s `Constants.h` holds the quad's
  values, `aztec` overrides them in place (as for the steering duties, the park ladder and
  `ENGINE_RUNNING_RPM_THRESHOLD` 800) — plus a `GOV_DEFAULTS_VEHICLE == VEHICLE_TYPE_CODE`
  `static_assert` so that the first merge cannot carry the quad's values onto the buggy unnoticed
  (design D12). `old-board` is the quad's hand-wired board (`VEHICLE_TYPE_CODE` 1) and takes the
  quad's values.
- **A3. No feed-forward.** The speed → throttle relation is exactly what varies with CVT, slope and
  load; a fixed map would be wrong most of the time. The integrator is initialised from the current
  throttle at every engagement, which gives the same benefit at the moment it matters.
- **A4. Low speed.** Stick ≤ 5 % = stop (setpoint 0, throttle idle at once, integrator reset). Above
  the dead-band the setpoint starts at 1.4 m/s (≈ 5 km/h, the CVT cannot creep smoothly below that),
  capped at half the limit, and rises linearly to the limit at 100 % (at a 10 km/h limit: 5–10 km/h,
  kept, design D4). While the measured speed is 0 (vehicle stationary, clutch not yet engaged,
  engine off) the governor output is capped at the start throttle limit (default buggy 4 %, quad
  10 %), so it cannot
  wind up against a vehicle that does not move; the same limit caps the integrator inherited at a
  take-over (D8). It must be above the throttle that moves the vehicle off on the steepest slope it
  launches on, and every % above that is the size of a possible lurch.
- **A5. No separate output rate setting.** The governor's output and both hand-overs move at most at
  the existing per-vehicle limiter throttle rate `thr_slew` (default 80 %/s), renamed on the page to
  "Плавність газу (обмежувач і регулятор)" / "Throttle rate (limiter and governor)". Closing to idle
  on stick-zero, brake or fail-safe is immediate. With the D13 gains the law itself never asks for a
  fast move (one speed quantum is 0.14 % of throttle on the buggy and 0.43 % on the quad, a full
  10 km/h error 2.8 % / 8.3 %), so the rate limit is
  only a step guard; a slower governor-only rate was modelled and makes a mis-tuned loop hunt more,
  not less (D13), so it is not added.
- **A13. Same gains for every limit on a vehicle (no scheduling).** The plant gain is highest at
  low speed, so gains safe at 10 km/h have more margin at 20 and 30 km/h; the cost is a slower hill
  response there (D14).
- **A14. The speed reading is used as it is** — no filter and no error dead-band: on the buggy
  (Kp 1.0) one speed quantum moves the throttle by about half a servo step; on the quad (Kp 3.0) by
  ≈ 1.5 steps, worth ≈ 0.1 km/h of speed ripple in the model because the quad's step moves its
  speed little; a filter would add dead time (D15).
- **A6. Web brake with the governor ON forces idle.** Today a web throttle and a web brake can be
  commanded at the same time; with the governor ON, a brake target ≥ 5 % makes the governor hold
  idle and reset (it never fights the brake). With the governor OFF nothing changes.
- **A7. Bypass states.** N / UNKNOWN expected gear, gear change active, boost PID or idle hold,
  throttle calibration: the governor is not evaluated and is reset; today's clip (8.5 %) and code
  apply. Leaving the bypass re-engages bumplessly from the current throttle.
- **A8. Ceiling limiter stands down while the governor is active** (no double limiting); it remains
  the protection in every other state, exactly as today.
- **A9. GCS warning only as `STATUSTEXT`.** No new `NAMED_VALUE_INT`; the at-idle (downhill) badge is
  web-only, like the limiter's saturation badge.
- **A10. "No pulse since boot" counts as invalid speed — owner decision 2026-11-17 (variant A).**
  `SpeedSensor::isValid()` is false until the first pulse after boot, so the first launch after
  every boot is direct throttle until the wheel has turned ≈ 3 cm, then the governor takes over
  without a step (design D8, "Owner Decisions"). This follows decision 4 literally; the
  alternative ("standing still, speed 0") was rejected because a sensor dead since boot would then
  cap the throttle silently.
- **A11. aztec: the gear ceiling is an effective limit.** With the switch ON the governor is active
  in L / H / R even without `SPEED_MAX`: full stick = 45 / 75 / 20 km/h. The aztec limiter rule
  "no speed-invalid warning while the gear ceiling is the source" (add-gear-speed-ceiling D7) does
  not apply to the governor's own warning, which needs a stick demand and a 2 s grace instead.
- **A12. Autopilot mode is not checked** (owner decision 5). In an autonomous mode ArduPilot's own
  speed controller would be cascaded with this one (design, Risks).

Out of scope: brake assist (downhill hold, active deceleration); a GCS-settable setpoint or gains;
a `GOV_STATE` named value; event-journal entries; the spec drift "Throttle safety override" (brake
> 50 % → throttle idle, in `specs/vehicle-systems` but not in the code); cruise control (holding a
speed with the stick released).

## Impact
- Affected specs:
  - `vehicle-systems`: ADDED "Speed Governor", "Speed Governor Settings", "Speed Governor Hand-Over
    and Speed-Sensor Fail-Open", "Speed Governor Yields to Brake, Gear Change and Fail-Safe",
    "Ceiling Limiter Stands Down While the Speed Governor Is Active".
  - `web-control`: ADDED "Speed Governor Configuration via Web Interface".
  - `web-telemetry`: ADDED "Speed Governor Telemetry", "Speed Governor State Display".
  - `mavlink-interface`: ADDED "Speed Governor Status Text".
  No requirement is MODIFIED: the limiter requirements differ between `master` and `aztec`
  (add-gear-speed-ceiling), so the stand-down is an ADDED requirement that names the limiter
  requirements it takes precedence over (design D7).
- Affected code (`master`): `include/SpeedGovernorLaw.h` (new), `include/Constants.h`,
  `include/ThrottleController.h` (`getLastCommandedPct()`), `include/VehicleController.h`,
  `src/VehicleController.cpp`, `include/MavlinkInterface.h`, `src/MavlinkInterface.cpp`,
  `include/WebPortal.h`, `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`,
  `include/Changelog.h`, `platformio.ini`, `test/test_speed_governor/test_main.cpp` (new),
  `WEB_PORTAL_SETUP.md`, `MAVLINK_SETUP.md`, `../../API/README.md`.
- Behaviour on existing units: none until the switch is turned on (default off, absent key = off).
  The only visible change with the switch off is the renamed throttle-rate label and the new card.
- No NVS migration: new keys in the existing `"speed"` namespace (shared with `SpeedSensor` and the
  limiter, never `clear()`ed).
- LittleFS image must be uploaded on every vehicle.
- Related changes: builds on the limiter of `add-speed-limiter-integral-floor` (implemented 1.5.3,
  archived 2026-11-02 — taper, floor, integral trim, saturation badge are all in the code) and
  `add-speed-limiter-slew-setting` (implemented 1.5.12, not archived; this change reuses its
  `thr_slew` and relabels its row); `add-gear-speed-ceiling` (aztec 1.5.32, archived on `aztec`)
  provides the gear part of the effective limit; `add-throttle-open-slew` (archived) stays at the
  servo input; `add-gear-boost-enable-switch` (not archived) owns the throttle during a shift;
  `add-brake-tuning-card` (not archived) is untouched. No in-flight change MODIFIES a requirement
  this change touches.
- Branches: `master` first; plain merges `master` → `aztec` and `master` → `old-board`. The aztec
  merge sets the buggy's `GOV_KP_DEFAULT` 1.0, `GOV_KI_DEFAULT` 0.35, `GOV_LAUNCH_DEFAULT_PCT` 4 and
  `GOV_DEFAULTS_VEHICLE` 2 (the build fails until it does, design D12); `old-board` keeps the
  quad's values.
