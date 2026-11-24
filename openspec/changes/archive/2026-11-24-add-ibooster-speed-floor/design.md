## Context
- Firmware 1.6.0 (aztec) drives a Bosch iBooster Gen2 on CAN (`replace-aztec-brake-with-ibooster`).
  The loop computes the `0x38B` byte 2 speed byte once per loop (`src/IboosterBrake.cpp:223`,
  `ibSpeedByte()` in `include/IboosterLaw.h:197-203`) and hands it to the CAN task in `IbCommand`;
  the task builds the frames with `ibBuild()` (`include/IboosterProto.h:111-130`).
- The unit's `0x38F` byte 3 ("pRunout", bar) is decoded in the task (`src/IboosterBrake.cpp:438-444`)
  and already published to telemetry (`ib_runout`) and the Brake card's fields line. It is not
  journaled except as raw bytes in `fault_raw`.
- Normal braking is not journaled today (`src/IboosterBrake.cpp:299-301`). Existing `BRAKE` lines:
  `state`, `fault`, `lost`, `fault_raw`, `recovered`, `unit_drop`, `tx_gap`, `cmd_stale`,
  `release_slow`, `fault_latched`, `no_move`, `start_timeout`, `no_unit`, `stats` (60 s). None is a
  per-brake-episode line, so there is nothing to extend: one new line is added.
- Bench facts used here (`Arduino/iBooster-TEST/README.md`, «Що виміряно на стенді»): pRunout 50 idle,
  51 under a request at speed 0–2 counts, 103 at ≥ 3 counts; a speed seen during a request is
  "remembered" until the request is removed, then the value decays ~1 bar / 20 ms to 50; 0 at the
  stroke stop with positive flow (≈ reported pressure − 21 bar there, and we report 5.1 bar), back as
  soon as the flow goes negative.

## Goals / Non-Goals
- Goals: the unit never sees a speed below 2 km/h; the technician can read, per brake episode, which
  ceiling the unit reported, without a web change.
- Non-Goals: proving real pressure; changing the qualifier bits; any UI.

## Decisions

### D1 — Floor inside `ibSpeedByte()`
- `#define IB_LAW_SPEED_FLOOR_KMH 2.0f` next to `IB_LAW_SPEED_KMH_PER_BIT`.
- `ibSpeedByte(speedMs, valid)`: km/h = `speedMs * 3.6` when `valid` and `speedMs > 0`, else 0 (NaN
  falls here too); if `!(km/h >= floor)` then km/h = floor; counts = km/h / 0.4 + 0.5, ≥ 255 → 255,
  else truncated. Results: invalid / NaN / ≤ 0 / 1 km/h / 2 km/h → 5; 3 km/h → 8 (7.5 rounds up);
  40 km/h → 100; ≥ 101.8 km/h and +∞ → 255.
- Signature and the single call site stay; the floored byte exists only in `IbCommand.speedByte` and
  the frames. `SpeedSensor`, limiter, governor, telemetry, MAVLink and web read the speed sensor, not
  this byte.
- Alternatives: floor in `ibBuild()` (it takes a raw byte, would mix units into the protocol layer);
  floor in `VehicleController` before `update()` (would put a fake speed into a shared input struct).

### D2 — Journal: one `episode` line per brake request episode
- **Episode** = from the slot whose state enters `BRAKING` from `READY` to the first slot whose state
  is neither `BRAKING` nor `RELEASING`. `RELEASING → BRAKING` (a re-press) continues the episode.
- **Samples**: each `0x38F` frame newly decoded since the previous slot (fresh, trusted, valid
  checksum) while the request is sent in that slot and a fresh `0x39D` reports internal state 3
  (accepted). Frames before acceptance (idle 50 / 51 still settling) and after the request is cleared
  (the decay to 50) are therefore never sampled.
- **Fields**: `ms` = episode duration; `rod_max` = highest fresh rod (mm, `-` if none); `spd_min` =
  lowest `0x38B` byte 2 sent in the episode; `runout=<min>..<max>` over the **non-zero** samples
  (`-` if none); `stop=1` if any sample was 0 (stroke stop), else 0.
  Example: `BRAKE I episode ms=4210 rod_max=9.85 spd_min=5 runout=103..103 stop=0`.
- **Where**: a pure tracker in `include/IboosterLaw.h` (`IbEpisode` + `ibEpisodeStep(...)`, returns
  true with the summary on the episode's last slot), native-tested. It is called by the task in
  `onSlot()` after `ibStep()`, with the step's previous and new state, `o.req`, the `0x39D` freshness /
  internal state, a "new `0x38F` this slot" flag (arrival time changed), its byte 3, the speed byte
  and the rod. It is a separate helper because `IbStepOut.ev` already uses all 8 bits. The summary is
  pushed as a new `IbEvent` kind `EV_EPISODE` (fields added to `IbEvent`) and written by the loop in
  `journalEvent()`, like every other `BRAKE` line.
- **Rate**: level `I`, **not** exempt from the journal's 2 s (source, event) dedupe: at most one
  `episode` line per 2 s; held-back episodes are counted and appended as `sup=<n>` by the journal.
  An episode ending in `LOST` / `FAULT` / `STARTING` / `ASLEEP` still writes its line (after the state
  line of the same slot).
- **Why this rule**: one line per episode, never per change, so the 0 at the stroke stop and the
  post-release decay cannot cause spam; min / max show both the floor working (103..103) and any dip
  during a long hold (e.g. 51..103 if the unit lowers its ceiling after a time at standstill); `stop`
  separates "ceiling reached the stroke stop" from a low ceiling; `spd_min` shows the floor was sent.
- **Alternatives considered**: a line on every pRunout change (spam at the stroke stop and during the
  decay); adding the current pRunout to the 60 s `stats` line (misses most episodes, and the value
  between episodes is the idle 50); extending the `state` line (normal braking transitions are not
  journaled, and must stay so).
- Line length: worst case ≈ 105 characters with the prefix and `sup=`, under `JOURNAL_LINE_MAX` (120).

## Risks / Trade-offs
- No real-pressure gain (unverified) → task 4.3 compares before / after on the vehicle; the change is
  a one-constant revert.
- Higher current / heat on long standstill holds (the pressure reduction is defeated) → task 4.5;
  parking-brake hand-over for long holds.
- The unit is told 2 km/h when standing; if it uses speed for anything else (e.g. a hold function),
  that logic sees "rolling slowly" → watch for faults / drops (task 4.4).
- Journal volume with frequent short brakes: ≤ 30 lines / min worst case, typical a few per minute.

## Migration Plan
- Firmware image only; no NVS key, no LittleFS. Rollback = flash 1.6.0 (or set the floor to 0).

## Open Questions
- Does the unit honour the speed byte in idle frames (qualifier 0)? Matters for hand pushes (driver
  braking without a CAN request). Technician check: tasks 4.1–4.2 (journal after a commanded brake,
  live `ib_runout` on the Brake card during a hand push). Not changed here.

## Implementation notes (2026-11-23, implementation on `aztec`)
No deviation from the approved behaviour; precisions only.

1. **Episode boundaries in the tracker** (D2). `ibEpisodeStep()` folds the slot that enters
   `BRAKING` from `READY` and every following `BRAKING` / `RELEASING` slot into the episode; the
   slot that ends it (state neither) only closes it and is not sampled (no request is sent in it,
   and its rod / speed byte belong to the next state). `ms` = that slot's time − the entry slot's
   time (unsigned, wrap-safe). `req` is the step's `o.req` as written in D2; while the chip is down
   (`canSend` false) no `0x38F` arrives, so no sample can be taken from an unsent slot.
2. **"New `0x38F` this slot"** is the task-side comparison of `unit_.t38F` with the value seen by
   the previous slot (`epHave38F_` / `epT38F_` in `IboosterBrake`). `onFrame()` updates `t38F` only
   for checksum-valid, trusted, decodable frames, so the "checksum-valid, trusted" condition of the
   spec is inherited from the existing decoder. "Fresh rod" is the law's `rodFresh`
   (`0x39D` ≤ `IB_LAW_ROD_HOLD_MS` = 100 ms old); "fresh `0x39D`" is ≤ `IB_LAW_STATUS_STALE_MS`.
3. **Hand-over.** The summary goes through the existing task → loop event ring as `EV_EPISODE`
   (pushed after this slot's `EV_STATE`, so the `episode` line follows the `state` line) and is
   written by `journalEvent()` in the loop, like every other `BRAKE` line. Not added to
   `jDedupeExempt()`; held-back episodes that are never followed by a written one are reported by
   the journal's existing pending-suppression line, as for any deduped event.
