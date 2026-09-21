## Context

The vehicle had two speed ceilings that never met:

- **ArduPilot `SPEED_MAX`** (m/s) — the autopilot's own cruise ceiling, editable live from Mission
  Planner / MAVProxy over the same TELEM link the ESP32 already speaks.
- **The firmware limiter** (`SPEED_LIMIT_*`, NVS keys `lim_on` / `lim_kmh`) — a throttle clamp
  driven by the hall wheel sensor, editable only from the ESP32's WiFi portal.

The first version of this change made the second *follow* the first, with the stored value as a
fallback and the web toggle as a master switch. That arbitration has since been withdrawn by
operator decision: **the limit is `SPEED_MAX` and nothing else, and its absence means no
limiting**. The remainder of this document is the design as it now stands.

Constraints that shaped the design:

- **The enforced number must be knowable from the ground station.** A fallback the GCS cannot see
  or change is worse than no limit, because the operator cannot reason about it while driving.
- **No NVS at all on this path.** The MAVLink-supplied ceiling is RAM-only, and with the local
  ceiling gone there is no limiter key left to wear out.
- **Existing policy.** `include/MavlinkInterface.h` must stay free of mavlink headers, so any
  new handler takes decoded scalars (the `handleServoOutputRaw` pattern).
- **Fail-open stays fail-open.** The limiter's existing contract — an invalid speed reading does
  not clamp — is safety-load-bearing and is not touched.
- **One unit inside, one unit outside.** Every interface the firmware touches (`SPEED_MAX`,
  `VFR_HUD`, `VISION_POSITION_DELTA`, the sensor's own mm/ms arithmetic) is m/s; only humans want
  km/h. So m/s is internal and km/h lives at the presentation edge.

## Goals / Non-Goals

- Goals:
  - Read `SPEED_MAX` over MAVLink and use it as the limiter ceiling within ~5 s of a change.
  - Stop limiting entirely — no fallback ceiling — when the parameter is absent, zero, invalid,
    silent or the link is down.
  - Carry every road speed in m/s inside the firmware, converting to km/h only for the web JSON,
    the HTML and human-readable debug strings.
  - Replace the limiter's 20 % throttle step with a proportional, slew-limited taper.
  - Evaluate the limiter every loop on the web throttle path, not once per command.
- Non-Goals:
  - **Writing** any autopilot parameter. The ESP32 never sends `PARAM_SET`.
  - A general parameter cache or a full `PARAM_REQUEST_LIST` download. Exactly one parameter is
    subscribed.
  - Persisting the MAVLink value, or keeping any local ceiling, toggle or NVS key for the
    limiter. It is RAM-only and dies with the link.
  - Changing the gear-boost PID, which writes µs directly via `setThrottleUs()` and is outside
    the limiter by design (it runs while the drivetrain is disengaged).
  - Changing the units on the wire. `vehicle_speed` stays km/h in the telemetry JSON and the UI
    stays a km/h instrument; only the *inside* of the firmware changes unit.

## Decisions

- **Decision: poll *and* accept unsolicited `PARAM_VALUE`.**
  ArduPilot broadcasts a `PARAM_VALUE` after a `PARAM_SET`, but whether that broadcast reaches a
  peripheral component depends on the firmware version and the routing between ports. A 5 s
  `PARAM_REQUEST_READ` poll makes the update deterministic; the unsolicited path just makes it
  arrive sooner. The poll **never stops** — it is the change detector, not a one-shot fetch.
  *Alternatives considered:* one-shot read at boot (misses every live edit — the whole point);
  broadcast-only (silently version-dependent); `PARAM_REQUEST_LIST` (hundreds of messages
  competing with the 25 Hz command stream for the same 115200 link).

- **Decision: two independent staleness gates.**
  `hasSpeedMaxParam()` requires both `isLinkUp()` (3 s heartbeat timeout) **and**
  `age < MAVLINK_PARAM_STALE_MS` (16 s ≈ 3 poll periods). An unplugged TELEM cable stops limiting
  in 3 s; an autopilot that is alive but has stopped answering this particular parameter stops
  limiting in 16 s. Tying the decision to only one of them would leave the other failure mode
  holding a ceiling nobody can see.

- **Decision: one source, and its absence means NO LIMITING.**
  `getSpeedLimitMs()` returns `mavlink_.getSpeedMaxMs()` when `hasSpeedMaxParam()` is true and 0
  otherwise. There is no stored fallback, no clamp range and no master switch.
  *Alternatives considered:* the previous three-way arbitration (OFF / LOCAL / MAVLINK), rejected
  by the operator — a dropped link silently swapping a deliberate crawl setting for a 60 km/h
  stored ceiling is exactly the invisible state the GCS cannot diagnose; a "hold the last known
  limit" fallback, rejected for the same reason, with the extra defect that the held value has no
  expiry.
  The cost is accepted and explicit: a vehicle whose autopilot has no `SPEED_MAX` is unlimited,
  so setting `SPEED_MAX` becomes a commissioning step.

- **Decision: `SPEED_MAX = 0` means no limit, not "not set".**
  It is what ArduPilot itself reads and what a ground station writes to clear a limit, so the
  firmware SHALL not reinterpret it. With no fallback ceiling left, "not set" and "no limit" are
  now the same outcome anyway, which removes a whole class of ambiguity.

- **Decision: m/s internally, km/h only at the presentation edge.**
  The sensor's `(delta × mm/pulse) / windowMs` is already m/s, `SPEED_MAX` is m/s, `VFR_HUD`
  groundspeed is m/s and `VISION_POSITION_DELTA` integrates m/s. Carrying km/h internally put a
  `× 3.6` immediately followed by a `/ 3.6` on nearly every path. One constant, `MS_TO_KMH`,
  survives, used only in `WebPortal.cpp` serialisation, in log strings, and at the single
  comparison against the CAN bus's natively-km/h speed byte.

- **Decision: linear taper with a floor, and the *ceiling* is slewed — not the demand.**
  Ceiling = 100 % at `limit − band`, linear to `SPEED_LIMIT_FLOOR_PCT` at `limit`, floor above.
  A floor rather than zero, because cutting throttle outright mid-corner is a stability event, not
  a safety feature. The rate limit is applied to the *computed ceiling*, so the driver's own stick
  movements pass through at full rate and only the limiter's authority ramps.
  *Alternatives considered:* a PI controller on speed error (needs tuning on a vehicle we cannot
  bench-tune safely, and can wind up against a hill); a hysteresis band around the hard step
  (removes the hunting but keeps the lurch).

- **Decision: `webThrottleDemandPct_` re-applied every loop.**
  The MAVLink path already re-reads its channel every loop, so it was always re-limited. The web
  path clamped once, at command time. Storing the demand and re-limiting it in `update()` makes
  both paths behave identically. The demand is reset to 0 wherever the throttle is forced to idle
  (web-control engage, fail-safe entry, gear-boost release), because a remembered demand that
  outlives an idle command would re-apply itself on the next iteration.

- **Decision: amend this change rather than open a new one, with one `REMOVED` delta.**
  None of this change's deltas have been archived into `openspec/specs/`, so amending it keeps one
  coherent story instead of layering a "simplify" proposal on an unshipped "arbitrate" proposal.
  The single exception is `speed-sensor` → `Configurable Maximum-Speed Throttle Limiter`, which
  belongs to the unarchived `add-hall-speed-sensor` change: it is REMOVED here rather than edited
  there, so the retirement is auditable. **That makes archive order load-bearing** —
  `add-hall-speed-sensor` must be archived first.

## Risks / Trade-offs

- **A GCS parameter download shares the 115200 link with the 25 Hz command stream.** → The poll is
  a single 20-byte `PARAM_REQUEST_READ` every 5 s, and the handler filters on sysid/compid before
  doing any work; the bench gate checks `mav_cmd_rate` stays ≥ 10 Hz during a full download.
- **NaN is the dangerous value.** A NaN limit makes `speed <= limit` false forever and would pin
  the throttle ceiling at the floor. → `isnan`/`isinf` rejection at the transport, before storage,
  plus epsilon comparisons everywhere (never `==`).
- **A GCS at sysid 255 on the same wire could otherwise set the vehicle's ceiling.** → The handler
  accepts only the learned autopilot's sysid *and* compid.
- **A vehicle with no `SPEED_MAX` set is now unlimited** where it was previously held at the
  stored 60 km/h ceiling. → Accepted and documented: setting `SPEED_MAX` is a commissioning step,
  `MAVLINK_SETUP.md` says so, and the UI shows "none" rather than a blank.
- **The taper reduces available throttle earlier than the old step did** (from `limit − 1.4 m/s`
  rather than at the limit). → That is the intent: authority is withdrawn gradually instead of all
  at once. The floor is 10 %, below the old 20 % cap, so steady-state behaviour above the limit is
  slightly stronger, not weaker.
- **A stalled loop could otherwise slam the ceiling** through a huge `dt`. → `dt` is capped at 1 s,
  and the first call after `limiterLastMs_ == 0` jumps straight to the target rather than
  integrating an unknown interval.
- **A stale low ceiling could survive a "limit disappears, limit returns" cycle.** →
  `limiterCeilingPct_` is reset to 100 % on both the no-limit and the sensor-invalid exits.
- **A unit rename can silently halve or triple a threshold.** → Every converted constant carries
  its old km/h value in the comment, and the greps in tasks 6.1–6.2 gate the build.

## Migration Plan

**One field step is required: set `SPEED_MAX` on the autopilot.** Until it is set, the vehicle is
unlimited — the deliberate, visible failure mode replacing the previous invisible stored ceiling.

`SpeedSensor::begin()` deletes the retired `lim_on` / `lim_kmh` keys once, so a downgrade to
firmware that still reads them finds them absent and falls back to its compile-time defaults
(limiter ENABLED at 60 km/h) rather than to a stale operator setting. That is the safe direction
for a rollback, but it means a rollback does NOT restore a previously customised local ceiling.

`data/index.html` changes, so a `pio run -t uploadfs` must accompany the firmware flash or the UI
will show the new telemetry keys nowhere.

## Open Questions

- **`VISO_DELAY_MS`-style lag on the hall reading.** The speed is a window average, so the taper
  reacts to a value that is ~100 ms old at speed and older at a crawl. Whether that needs a lead
  term (or simply a wider band) is a drive-test question, deliberately left to bench item 6.10
  rather than guessed at here.
- **Should an out-of-range `SPEED_MAX` (> 30 m/s) limit at 30 rather than not at all?** It is
  currently rejected at the transport and therefore means "no limit", which is the same treatment
  as a NaN. Clamping it would be defensible; rejecting keeps exactly one rule for "a value we do
  not trust". Left as-is pending a case where it actually happens.
