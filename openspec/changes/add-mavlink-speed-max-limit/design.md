## Context

The vehicle has two speed ceilings that have never met:

- **ArduPilot `SPEED_MAX`** (m/s) — the autopilot's own cruise ceiling, editable live from Mission
  Planner / MAVProxy over the same TELEM link the ESP32 already speaks.
- **The firmware limiter** (`SPEED_LIMIT_*`, NVS keys `lim_on` / `lim_kmh`) — a throttle clamp
  driven by the hall wheel sensor, editable only from the ESP32's WiFi portal.

The autopilot's number is the one the operator actually edits in the field. The firmware's is the
one that physically holds the throttle servo back. This change makes the second follow the first.

Constraints that shaped the design:

- **NVS wear.** `SpeedSensor::setLimitMaxKmh()` does a `Preferences::putFloat` on every call. A
  5 s poll writing the same value is ~6 300 writes/day. The MAVLink-supplied ceiling must never
  touch flash.
- **The web toggle is the master switch.** The operator standing at the vehicle must be able to
  disable the limiter without a ground station, and no autopilot traffic may re-arm it.
- **Existing policy.** `include/MavlinkInterface.h` must stay free of mavlink headers, so any
  new handler takes decoded scalars (the `handleServoOutputRaw` pattern).
- **Fail-open stays fail-open.** The limiter's existing contract — an invalid speed reading does
  not clamp — is safety-load-bearing and is not touched.

## Goals / Non-Goals

- Goals:
  - Read `SPEED_MAX` over MAVLink and use it as the limiter ceiling within ~5 s of a change.
  - Fall back deterministically to the stored local value when the parameter is absent, zero,
    invalid, silent or the link is down.
  - Replace the limiter's 20 % throttle step with a proportional, slew-limited taper.
  - Evaluate the limiter every loop on the web throttle path, not once per command.
- Non-Goals:
  - **Writing** any autopilot parameter. The ESP32 never sends `PARAM_SET`.
  - A general parameter cache or a full `PARAM_REQUEST_LIST` download. Exactly one parameter is
    subscribed.
  - Persisting the MAVLink value. It is RAM-only and dies with the link.
  - Changing the gear-boost PID, which writes µs directly via `setThrottleUs()` and is outside
    the limiter by design (it runs while the drivetrain is disengaged).
  - Retrofitting `MAVLINK_MS_TO_KMH` onto the existing `/ 3.6f` literals in
    `src/MavlinkInterface.cpp` — out of scope, deliberately untouched.

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
  `age < MAVLINK_PARAM_STALE_MS` (16 s ≈ 3 poll periods). An unplugged TELEM cable falls back to
  the local ceiling in 3 s; an autopilot that is alive but has stopped answering this particular
  parameter falls back in 16 s. Tying the fallback to only one of them would leave the other
  failure mode holding a ceiling nobody can see.

- **Decision: clamp an out-of-band MAVLink value, don't discard it.**
  A `SPEED_MAX` below `SPEED_LIMIT_MIN_KMH` is clamped up to it, and above `SPEED_LIMIT_MAX_KMH`
  clamped down. Discarding would silently revert a deliberate crawl setting to a stored 60 km/h
  ceiling — the failure direction that hurts. `SPEED_MAX = 0` is different in kind: ArduPilot
  itself reads 0 as "not set", so it is treated as *no MAVLink value* and falls back.

- **Decision: RAM-only, and the master switch is local.**
  `getEffectiveSpeedLimitKmh()` returns `OFF` whenever `speedSensor_.isLimiterEnabled()` is false,
  before it ever looks at MAVLink. The autopilot can lower the ceiling; it cannot arm the limiter.

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

- **Decision: `ADDED` deltas only, in three capabilities.**
  `speed-sensor` exists only inside the unarchived `add-hall-speed-sensor` change, so there is no
  `specs/` requirement to MODIFY; and `mavlink-interface`'s reporting requirement is already
  MODIFIED by three active changes, where a fourth rewrite would make archive order load-bearing.

## Risks / Trade-offs

- **A GCS parameter download shares the 115200 link with the 25 Hz command stream.** → The poll is
  a single 20-byte `PARAM_REQUEST_READ` every 5 s, and the handler filters on sysid/compid before
  doing any work; the bench gate checks `mav_cmd_rate` stays ≥ 10 Hz during a full download.
- **NaN is the dangerous value.** A NaN limit makes `speed <= limit` false forever and would pin
  the throttle ceiling at the floor. → `isnan`/`isinf` rejection at the transport, before storage,
  plus epsilon comparisons everywhere (never `==`).
- **A GCS at sysid 255 on the same wire could otherwise set the vehicle's ceiling.** → The handler
  accepts only the learned autopilot's sysid *and* compid.
- **The taper reduces available throttle earlier than the old step did** (from `limit − 5 km/h`
  rather than at the limit). → That is the intent: authority is withdrawn gradually instead of all
  at once. The floor is 10 %, below the old 20 % cap, so steady-state behaviour above the limit is
  slightly stronger, not weaker.
- **A stalled loop could otherwise slam the ceiling** through a huge `dt`. → `dt` is capped at 1 s,
  and the first call after `limiterLastMs_ == 0` jumps straight to the target rather than
  integrating an unknown interval.
- **A stale low ceiling could survive a disable/re-enable cycle.** → `limiterCeilingPct_` is reset
  to 100 % on both the disabled and the sensor-invalid exits.

## Migration Plan

None required in the field. With no `SPEED_MAX` on the autopilot (or `SPEED_MAX = 0`) the firmware
behaves exactly as before, using the stored local ceiling. The only unconditional behaviour change
is the taper replacing the 20 % step, which needs no configuration.

Rollback is a firmware flash: nothing is written to NVS or to the autopilot, so a downgrade finds
the stored limiter settings exactly as it left them.

`data/index.html` changes, so a `pio run -t uploadfs` must accompany the firmware flash or the UI
will show the new telemetry keys nowhere.

## Open Questions

- **`VISO_DELAY_MS`-style lag on the hall reading.** The speed is a window average, so the taper
  reacts to a value that is ~100 ms old at speed and older at a crawl. Whether that needs a lead
  term (or simply a wider band) is a drive-test question, deliberately left to bench item 6.10
  rather than guessed at here.
- **Should `speed_limit_src` distinguish "mavlink, clamped" from "mavlink"?** Currently a clamped
  out-of-band value still reports `"mavlink"` and the clamped number appears in
  `speed_limit_eff`, which is honest but does not tell the operator their `SPEED_MAX` was out of
  range. A one-shot log covers it for now.
