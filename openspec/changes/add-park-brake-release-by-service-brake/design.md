# Design: add-park-brake-release-by-service-brake

## Context
The owner asked on 2026-11-17 for the parking brake to be released by the service brake, so the
buggy can come off a trailer without the drive-away throttle. A first draft of this change protected
against the automatic fail-safe brake (an RC-alive gate, then a double press with an autopilot
fence). After the findings below were shown, together with the risk and two safer alternatives, the
owner decided on 2026-11-17: **"Без захисту" — no protection.** This document keeps the findings,
records the final rule, and marks the earlier guard decisions as superseded.

## Owner decision 2026-11-17 (final)
- Any new operator service-brake press releases an APPLIED (or UNKNOWN) parking brake.
- No double press, no threshold used as a guard, no autopilot / RC-alive fence, no web switch.
- Accepted consequence: on RC loss the Copilot's automatic brake (≈ 25 %, 15 s by default) releases
  the parking brake, and when that brake ends the buggy is held by nothing (see "Risks").

## Findings (investigation, 2026-11-17, branch `aztec` at 31752b9)
F1–F5 are kept as the record of why an RC-alive gate is impossible (F2, F3) and how the trigger
hooks into the firmware (F1, F4, F5). Under the owner decision they are background, not guards.

### F1. How a service-brake command reaches the firmware
- **RC / GCS joystick.** Combined servo channel 2 (`ServoChannelConfig::THROTTLE`), decoded from
  `SERVO_OUTPUT_RAW` (the only inbound stream, requested at 25 Hz, `src/MavlinkInterface.cpp:199-209`,
  `:372-404`).
  - `MavlinkInterface::getBrake()` (`src/MavlinkInterface.cpp:1260-1268`) maps 1500 → 1100 µs to
    0 → 100 %, so brake % = (1500 − µs) / 4; for example 1400 µs = 25 % and 1220 µs = 70 %.
  - `processMavlinkCommands()` calls `applyBrake(mavlink_.getBrake())` on every loop with input source
    MAVLINK and a valid signal (`src/VehicleController.cpp:1112-1114`). It returns early when
    `!isSignalValid()` (`:1060-1062`).
  - The GCS joystick reaches the vehicle as `RC_CHANNELS_OVERRIDE` to the autopilot, so it arrives
    on the same channel 2. The ESP32 cannot tell the hand on the RC handset from the GCS joystick or
    from the Copilot (F2).
- **Web.** `set_brake` → `processBrakeCommand()` → `applyBrake(value)` (`:2482-2492`). It is accepted
  only while the input source is WEB (`:907-923`). WEB is the source when web control is engaged,
  or when MAVLink is invalid and a browser is connected (`src/TelemetryManager.cpp:24-41`).
- **GCS command.** `MAV_CMD_USER_2` has no service-brake function: query, gear, ignition, light,
  lock and handbrake only (`include/VehicleCommandLaw.h:38`).
- **`currentBrakeTarget_`** is written only in `applyBrake()` (`src/VehicleController.cpp:2871-2879`).
  Values below 5 % become 0, so 5 % is the brake dead zone. Its two callers are the two paths above.
  Nothing else in the firmware commands the service brake. On web takeover the target is kept from
  MAVLink (`setWebControl()`, `:708-720`).
- **Position estimate.** `currentBrakePosition_` (`getBrakePositionPct()`, `include/VehicleController.h:199`)
  is an open-loop estimate that moves at 100 % per `brakeTravelMs_`, from NVS `brake`/`travel_ms`,
  1000–3000 ms, default 1500 (`include/BrakeLaw.h`, `src/VehicleController.cpp:2930-2950`).

### F2. What the firmware can know about the autopilot's RC state today, and why it is not enough
- **Parsed today** (`src/MavlinkInterface.cpp:190-241`):
  - `HEARTBEAT`: only `base_mode` and `custom_mode`, from the learned autopilot (`:329-354`).
    `system_status` is not decoded.
  - `SERVO_OUTPUT_RAW`, `PARAM_VALUE`, `COMMAND_LONG`, `COMMAND_ACK`.
  - Everything else (`SYS_STATUS`, `RC_CHANNELS`, `RC_CHANNELS_RAW`, `STATUSTEXT`, `NAMED_VALUE_*`)
    falls into `default:` and is dropped. Link liveness is the heartbeat age, with
    `MAVLINK_HEARTBEAT_TIMEOUT_MS` 3000 (`:1305-1310`). Command freshness is the `SERVO_OUTPUT_RAW`
    age, with `MAVLINK_CMD_TIMEOUT_MS` 500 (`:1291-1296`).
- **Where the automatic 15 s brake comes from.** It is not a Lua script.
  - A search of the whole `UGV/` tree found no vehicle `.lua` file. The only `.lua` files are a
    BlueBoat navlight in the BlueOS port and QGC build-cache files.
  - The brake is the BlueOS extension **Copilot**: `vehicle/blueos-workspace/extensions/ugv-copilot`,
    `metadata.json` version 0.1.11, described in
    `vehicle/blueos-workspace/docs/adr/0004-ugv-copilot-gcs-loss-brake.md`.
  - **Trigger.** The autopilot's fail-safe announcement, with `FS_GCS_ENABLE=1` and `FS_ACTION=0`:
    the `STATUSTEXT` "Radio Failsafe" from 1/1, backed up by `HEARTBEAT.system_status ==
    MAV_STATE_CRITICAL` for more than two heartbeats, plus a deferred path
    (`app/copilot.py:14-219`, `app/mavlink_link.py:118-160`).
  - **Output.** `RC_CHANNELS_OVERRIDE` on channel 2 = `brake_pwm`, as system 255 / component 194,
    at `stream_rate_hz` 5 Hz (`app/settings.py:75-85`):
    - `brake_pwm` defaults to 1400 µs, which is 25 % by the ESP32 formula (ADR-0004 rounds it to
      "≈ 20 %");
    - the setting range is 800–1490 µs, and 800 µs saturates to 100 %.
  - **Duration.** `brake_hold_s` (default 15 s, range 0–600 s, 0 = hold until a control source
    returns), then a release and the `HOLD_EXPIRED` latch, with no re-brake until a control source
    is seen (`app/settings.py:98`, ADR-0004 "Amendment 2026-09-22").
  - It announces nothing that the ESP32 receives. Its only output is the override frame. Its 1/193
    listener identity sends a heartbeat and parameter reads, over UDP to the Jetson's router.
- **The self-refresh caveat.** This is why an RC-alive gate cannot work. It is ADR-0004 lines
  170-187, `app/copilot.py` docstring "The observation itself can be poisoned", and bench
  2026-09-02. ArduPilot treats the Copilot's 255 override frames as **valid RC input** and as
  "GCS seen":
  - from the first brake frame the autopilot **clears its radio fail-safe**;
  - it sets the `SYS_STATUS` RC-receiver health bit to healthy;
  - it reports the GCS fail-safe as cleared, so `system_status` goes back from CRITICAL.

  For the whole automatic brake (15 s by default, possibly forever) the autopilot therefore says
  "RC fine". A gate built on `system_status`, the `SYS_STATUS` RC bit or `RC_CHANNELS` rssi would
  read **alive** exactly when the danger is present. A longer confirmation time does not help:
  the poisoning lasts as long as the brake.
- **Not verified** (no access to a live vehicle; reduced certainty about ArduPilot internals):
  - whether the Copilot is installed and enabled on the buggy's Jetson, and its `brake_pwm` there.
    The ADR names the boards tx1h and tx1-ixb.
  - the exact autopilot heartbeat cadence on the ESP32's serial port. 1 Hz is assumed, and the
    existing 3 s link timeout depends on it.
  - whether `STATUSTEXT` and `SYS_STATUS` are sent on the ESP32's port without a stream request.
    Today only `MAV_DATA_STREAM_RC_CHANNELS` is requested.
  - that a disarmed Rover reports `MAV_STATE_STANDBY` (3) and an armed one `MAV_STATE_ACTIVE` (4).
    The ADR records ACTIVE → CRITICAL (4 → 5) for the fail-safe; STANDBY for disarmed is the MAVLink
    convention.
  - what servo 2 outputs during the autopilot's own radio fail-safe with `FS_ACTION=0`. ADR-0008
    says the pilot input becomes zero, so 1500 µs (no brake) is expected, but it is not measured on
    the buggy.
  - what an AUTO / GUIDED mode would put on channel 2 while parked. With no mode condition (owner
    decision), a brake value output by the autopilot in such a mode also counts as a press.

### F3. Timing — which arrives first, the brake or "RC lost"?
- Timeline from ADR-0004 / ADR-0008 / `app/copilot.py`, real ArduRover 4.7:
  - at t0 the RC link is lost;
  - at about t0 + 1 s (`RC_FS_TIMEOUT`) the autopilot enters radio fail-safe: CRITICAL, RC bit
    unhealthy, "Radio Failsafe" text;
  - the Copilot brakes on the announcement at about t0 + 2.5 s (immediate path), or at 4.6–4.9 s
    (deferred path / CRITICAL streak);
  - the first brake frame clears the fail-safe.
- The visible "RC lost" window before the brake is therefore about 0–1.5 s on the immediate path.
  The heartbeat that carries `system_status` comes at 1 Hz, so the ESP32 may see **no** CRITICAL
  heartbeat at all before `SERVO_OUTPUT_RAW` (25 Hz) brings the brake. After that, every report says
  healthy.
- **Conclusion.** A rule of the form "brake held for T and the autopilot reports RC alive" is
  defeated twice over:
  - by the race (the brake may arrive first);
  - by the poisoning (after the brake starts, the autopilot really does say "alive").

  The race can be narrowed but not closed, and the poisoning cannot be addressed from the
  autopilot's state at all.

### F4. The firmware's own fail-safe and the web source
- **FAILSAFE.** The source is FAILSAFE when there has been no `SERVO_OUTPUT_RAW` for 500 ms and no
  browser is connected.
  - `applyFailsafe()` (`src/VehicleController.cpp:974-1003`) idles the throttle, calls
    `brake_.stop()` once, stops the transmission and switches the relays off.
  - It never calls `applyBrake()`, so `currentBrakeTarget_` keeps its **last** value.
    `updateBrakeControl()` keeps running and holds that stale target.
  - The parking brake is not touched, and a run in progress completes.
  - So under FAILSAFE a brake target can be non-zero with **nobody** commanding it, and it must never
    count as an operator press.
- **WEB.** Web brake commands are single, explicit operator actions on a page, which the Copilot and
  the autopilot cannot produce. The value persists as the target until changed.

### F5. The parking-brake state machine today (aztec, `add-aztec-parking-brake`, fw 1.5.29)
- **States.** UNKNOWN / RELEASED / APPLIED / APPLYING / RELEASING (`include/ParkBrakeLaw.h:29`),
  persisted in NVS `pbrake`/`state`. A moving or missing state boots as UNKNOWN, which is treated
  as possibly applied.
- **Commands.** Commands enter through `commandParkBrake(req, src)`, from the web and from the GCS
  (`MAV_CMD_USER_2` handbrake). Release is always allowed. Apply is refused above 3 km/h (hall
  sensor).
- **Drive-away** (`src/VehicleController.cpp:1390-1440`):
  - the conditions are APPLIED/UNKNOWN, ignition relay not OFF, target gear L/H/R and no gear change;
  - a rising-edge latch `pbArmed_` arms at ≤ 10 %; the release fires at > 10 %;
  - it calls `parkBrake_.request(RELEASE, OK)`;
  - `pbrake_da_wait` explains a press that did nothing.
- **Throttle hold.** In L/H/R the throttle is held while the brake is APPLIED/APPLYING/UNKNOWN, and
  while RELEASING until `thr_pass` % of `travel_ms`. This applies to every release run.
- **Telemetry.** Web: `pbrake_state`, `_remaining_ms`, `_thr_hold`, `_da_wait`, `_travel_ms`,
  `_thr_pass_pct`, `_invert`. GCS: EFI bits 2/3.
- **Spec text to supersede.** "Service brake independent": "a service brake command SHALL NOT change
  the park-brake state".
- **Governor.** On aztec the speed governor is reset while the park brake holds the throttle. A
  brake command already makes the governor stop (`applyOperatorThrottle(…, brakeActive)`).

## Goals / Non-Goals
- Goals:
  - release the parking brake with the service brake for trailer unloading, with no throttle;
  - let the service brake take the load before the parking brake lets go (D3);
  - never treat a held, inherited or frozen brake target as a new press (D2);
  - leave every existing release, apply and hold rule unchanged.
- Non-Goals:
  - any protection against the automatic Copilot brake (owner decision 2026-11-17);
  - changes to the Copilot or to the autopilot;
  - a new web setting, switch or telemetry key;
  - releasing the service brake for the operator, or holding it automatically;
  - any change on `master`.

## Decisions

### Superseded (earlier draft, kept for the record)
The following decisions of the earlier draft are **superseded by the owner decision of 2026-11-17**
and are not implemented: the double-press gesture (press ≥ 70 % – let go < 5 % – press again within
2000 ms – hold 1000 ms after the estimate reaches 70 %); the 70 % press threshold; the autopilot
fence (heartbeat ≤ 2000 ms old, `system_status` STANDBY/ACTIVE, mode MANUAL, no bad heartbeat during
the gesture) and the `HEARTBEAT.system_status` decoding it needed; the web single-press-at-70 % rule
with its separately tracked web value; the `pbrake_svc` telemetry key and its card status line; the
telemetry document growth that key needed; open questions Q1 (which protection) and Q3 (threshold).

### D1. Trigger: a new operator service-brake press while the parking brake is APPLIED or UNKNOWN
- The input is `currentBrakeTarget_`, the commanded service-brake target. It is written only by
  `applyBrake()` (`src/VehicleController.cpp:2871-2879` on aztec), which ignores an unchanged value,
  turns anything below 5 % into 0 and clamps to 100 %. Its two callers are the MAVLink path (servo
  channel 2 below centre: RC handset, GCS joystick, and any `RC_CHANNELS_OVERRIDE` sender such as the
  Copilot) and the web `set_brake` command (F1). All of them count; the firmware cannot and does not
  tell them apart.
- **Press** = the target goes from 0 to a value > 0.
- The parking brake state must be `APPLIED` or `UNKNOWN` (UNKNOWN treated as applied, as for
  drive-away and the throttle hold). In RELEASED, RELEASING or APPLYING a press does nothing.
- No gear, ignition, flight-mode, heartbeat or RC-alive condition (owner decision; for ignition see
  A2 / Q2).

### D2. Rising edge, not level — no phantom edges
- The law keeps the target it saw on the previous loop. It runs on every loop, whatever the
  parking-brake state, so a brake that is already held when the parking brake becomes APPLIED (for
  example: hold the brake, apply the parking brake from the web, keep holding) is **not** an edge.
  It takes a new press: the target back to 0, then non-zero.
- **FAILSAFE.** Under the firmware's FAILSAFE source `applyBrake()` is never called and the target is
  frozen at its last value (F4). The law does not evaluate a press under FAILSAFE, and it keeps
  updating its previous-target sample, so a frozen target is never a press, now or when a live
  source returns with the same value.
- **Input-source change** (MAVLINK ↔ WEB takeover via web control, FAILSAFE ↔ either). The target is
  inherited across the change (`setWebControl()` keeps it, `setInputSource()` does not touch it). On
  any source change the law re-seeds its previous-target sample with the current target and cancels
  a pending hand-over (D3). A takeover therefore never produces an edge by itself; under the new
  source the operator gives a new press. Example: web takeover with the MAVLink target at 80 % → no
  release; the web brake sent at 80 % → no change in `applyBrake()` → no release; web 0 then 80 % →
  release.
- A non-zero → different non-zero change (for example frozen 25 % → 40 %) is not an edge.

### D3. Hand-over without rolling (assumption A1)
- On a press the law becomes **pending**. The release run starts on the first loop where the
  service-brake position estimate `currentBrakePosition_` is at or above the current target minus
  `BRAKE_TOLERANCE` (1 %, the same "at target" test `updateBrakeControl()` uses).
- While pending, a change of the target to another non-zero value keeps the wait (against the new
  target). A target back to 0 cancels it: no release; a new press is needed. A parking-brake state
  other than APPLIED/UNKNOWN, or an input-source change, also cancels it.
- **Why.** The X7 actuator has a long dead travel (owner, 2026-11-17). On a trailer ramp the parking
  brake must not let go before the service brake is actually pressing.
- **Delay** from a released service brake (estimate 0 %) = (target − 1 %) / 100 × `travel_ms`:

  | `travel_ms` | 25 % | 50 % | 100 % |
  |---|---|---|---|
  | 1000 ms | 0.24 s | 0.49 s | 0.99 s |
  | 1500 ms (default) | 0.36 s | 0.74 s | 1.49 s |
  | 3000 ms | 0.72 s | 1.47 s | 2.97 s |

  The estimate is open loop. If the bench shows the pads bite later than the estimate says, the
  remedy is a longer `brake`/`travel_ms`, which also lengthens this wait.
- **Alternative (not recommended):** the run starts on the press itself, with no wait.

### D4. Release semantics are today's
- The trigger calls the same `parkBrake_.request(RELEASE, OK)` as drive-away, logged under
  `DebugFeature::BRAKE` as "[PBRAKE] release by service brake".
- Everything after that is the existing RELEASING behaviour: the throttle hold in L/H/R up to the
  release point; the run completes even if the service brake is let go mid-run; no automatic
  re-apply. A service-brake press never applies the parking brake.
- The firmware never moves or holds the service brake for this feature.
- The drive-away latch, `commandParkBrake()`, the 3 km/h apply interlock and the speed governor
  (`applyOperatorThrottle(…, brakeCommanded)`) are untouched.

### D5. Pure law in `include/ParkBrakeLaw.h`
- `struct ParkBrakeSvcRelease { float prevTargetPct; bool pending; ParkBrakeSvcSource prevSource; }`.
- `ParkBrakeSvcStep parkBrakeSvcStep(const ParkBrakeSvcRelease&, ParkBrakeSvcSource src, float
  targetPct, float estPct, ParkBrakeState st)` → next state + `fire`. `ParkBrakeSvcSource` is NONE
  (FAILSAFE) / MAVLINK / WEB. The tolerance is a `PB_LAW_SVC_TOL_PCT` literal checked against
  `BRAKE_TOLERANCE` by `static_assert`, like the existing `PB_LAW_*` literals.
- No Arduino dependency. Native tests in `test/test_park_brake_law`.
- Called in `VehicleController::update()` after `updateBrakeControl()` (so the estimate is this
  loop's) and before `parkBrake_.update()`.

### D6. Web help, no new telemetry
- No telemetry key is needed: a pending hand-over lasts at most `travel_ms`, and the card already
  shows "Releasing…" from `pbrake_state`. The telemetry document size is not touched.
- The card's help (ⓘ) states the rule and the RC-loss consequence (spec `web-control`).

### D7. Ordering
- Firmware 1.5.37, after `add-crank-time-limit-setting` (1.5.36) is implemented and merged into
  `aztec`.

## Assumptions (each can be overruled by the owner)
- **A1. Hand-over wait (D3).** The release starts only when the service-brake estimate has reached
  the commanded target; a command dropped to 0 first cancels it. Alternative: release on the press.
- **A2.** The X7 service-brake actuator has power with the ignition OFF, like X6 (Q2). If not, a
  press with the ignition OFF would release the parking brake with **no** service brake behind it,
  and the rule must add "ignition relay not OFF".
- **A3.** No new STATUSTEXT to the GCS. The release is visible through the existing EFI park-brake
  bits.
- **A4.** No journal entry. Debug log under `DebugFeature::BRAKE`, as for drive-away.

## Open questions
- **Q2. Is the X7 service-brake actuator powered with the ignition OFF?** Assumed yes (A2). Confirm
  with the owner or measure at X7 before implementation.

## Risks / Trade-offs
- **Owner decision 2026-11-17, accepted risk: the automatic fail-safe brake releases the parking
  brake.**
  1. The RC link is lost while the buggy stands with the parking brake APPLIED.
  2. The autopilot zeroes the pilot input (channel 2 → 1500 µs → target 0), then the Copilot brakes
     at `brake_pwm` (default 1400 µs ≈ 25 %) about 2.5–5 s after the loss (F2, F3).
  3. That is a new press. After the hand-over (≈ 0.36 s at 1500 ms travel) the parking brake
     releases.
  4. After `brake_hold_s` (default 15 s; 0 = hold until a control source returns) the Copilot lets
     the service brake go.
  5. **The buggy is held by nothing.** On a slope, or half-way down a trailer ramp, it rolls.

  The same happens with any other automatic brake on channel 2 (an autopilot mode outputting brake,
  a GCS joystick glitch). The crew sees this once on the bench (tasks 8.4).
- **Offered and declined:** a 70 % single-press threshold (the Copilot brakes at 25 %, but its
  `brake_pwm` can be set down to 800 µs = 100 %); a double press with hold; a web switch "release by
  service brake", default OFF; a "Copilot braking" flag sent by the Copilot.
- **Adding a guard later without redesign.** Each is a local change in `parkBrakeSvcStep()` or its
  call site, with no NVS or spec restructuring:
  - a minimum press level (e.g. ≥ 70 %) in the edge test — one constant;
  - a web switch (NVS bool) gating the call — the pattern of the gear-boost enable switch;
  - a double press — extra phases in the same law struct;
  - a Copilot flag — needs a Copilot change and routing to the ESP32 port (F2).
- **After the release only the service brake holds the buggy, and it depends on the link** — the
  same residual risk as any release today (drive-away, web, GCS).
- **Open-loop estimate.** A too-short `travel_ms` makes the hand-over come before the pads bite.
  Right after boot the estimate starts at 100 % until the released sensor syncs it to 0 %, so a press
  in the first seconds after power-up can hand over at once. Bench task 8.3 checks the ramp hold.
- **Ignition OFF** (A2 / Q2).

## Migration / Rollback
- Nothing is stored; no NVS or telemetry change. Rollback is a reflash of 1.5.36.
