## 0. Preconditions
- [x] 0.1 Proposal approved by the owner. Work on branch `aztec` only; never cherry-pick into
  `master` or `old-board`.
- [x] 0.2 Check that no change in `openspec/changes/` claims 1.6.1.

## 1. Pure logic (`include/IboosterLaw.h`, `<stdint.h>` only)
- [x] 1.1 `#define IB_LAW_SPEED_FLOOR_KMH 2.0f` next to `IB_LAW_SPEED_KMH_PER_BIT`; `ibSpeedByte()`
  applies max(real km/h, floor) per design D1 (invalid / NaN / ≤ 0 → floor, never 0); comment updated.
- [x] 1.2 Episode tracker per design D2: `struct IbEpisode` (active, start ms, rod max, speed-byte min,
  runout min / max over non-zero samples, any-sample flag, stop flag) and
  `ibEpisodeStep(IbEpisode&, IbState prev, IbState now, bool req, bool fresh39D, uint8_t is,
  bool new38F, uint8_t runoutBar, uint8_t speedByte, bool rodFresh, int16_t rodCmm, uint32_t nowMs,
  IbEpisodeSummary& out)` → true on the episode's last slot.
- [x] 1.3 `test/test_ibooster_law/test_main.cpp`, speed byte: 0 m/s → 5, invalid → 5, NaN → 5,
  negative → 5, 1 km/h → 5, exactly 2 km/h → 5, 3 km/h → 8, 40 km/h → 100, 200 m/s and +∞ → 255
  (clamp). Replace the 1.6.0 asserts that expected 0 for invalid / NaN / negative and 3 for 1.2 km/h.
- [x] 1.4 Tests for the tracker: READY → BRAKING starts, → READY ends with one summary; RELEASING →
  BRAKING stays one episode; samples before acceptance (is ≠ 3) and after the request is cleared are
  ignored (a 50 / decaying value never appears); a 0 sample sets `stop=1` and is excluded from min /
  max; only zeros → runout `-`, stop 1; no `0x38F` at all → runout `-`, stop 0; a repeated (not new)
  `0x38F` is not re-sampled; an episode ended by LOST / FAULT / STARTING / ASLEEP still returns a
  summary; `spd_min` is the lowest speed byte sent; `rod_max` only from fresh rods; duration wraps
  safely across `millis()` overflow.

## 2. Firmware
- [x] 2.1 `include/IboosterBrake.h` / `src/IboosterBrake.cpp`: task-side `IbEpisode` state, the
  "new `0x38F` this slot" flag (arrival time changed since the previous slot), `ibEpisodeStep()` in
  `onSlot()` after `ibStep()`; new `EvKind` `EV_EPISODE` with its fields in `IbEvent` (pushed after a
  state event of the same slot); `journalEvent()` writes
  `BRAKE I episode ms=<n> rod_max=<mm|-> spd_min=<n> runout=<min>..<max>|- stop=<0|1>` through
  `EventJournal::log()` (not added to `jDedupeExempt()`).
- [x] 2.2 `include/IboosterProto.h` comment block: `0x38B` byte 1 `0x40` is also the speed
  qualifier (bench field table: `VehicleSpeed_Q`), so idle frames carry qualifier 0; byte 2 never
  below 5 counts. No code change there.
- [x] 2.3 `include/Constants.h`: `FIRMWARE_VERSION "1.6.1"`.
- [x] 2.4 `include/Changelog.h`: entry 1.6.1 (date of implementation), `uk` and `en`, existing format
  and tone, "багі": the iBooster is always told at least 2 km/h (possibly more brake force standing;
  unverified; may warm the unit more on long holds — use the parking brake); the journal shows
  `BRAKE I episode … runout=…` per brake; firmware only, **no** LittleFS upload needed.

## 3. Docs and validation
- [x] 3.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md` iBooster row: `0x38B` speed byte = max(speed, 2 km/h),
  1.6.1.
- [x] 3.2 `WEB_PORTAL_SETUP.md` (journal / Brake section): the `BRAKE I episode` line, its fields and
  how to read it (103..103 = high ceiling, 51 = low ceiling, `stop=1` = stroke stop).
- [x] 3.3 `pio test -e native` green (all suites).
- [x] 3.4 `pio run` for the aztec environment without new warnings.
- [x] 3.5 `openspec validate add-ibooster-speed-floor --strict`.

## 4. Bench and vehicle (technician; nothing here is done by code)
- [ ] 4.1 Commanded brake at standstill (web or RC, 30 %, wheels chocked, engine off): the journal has
  `BRAKE I episode … spd_min=5 runout=103..103` (1.6.0 gave 51). Record the line. Fail: 51 anywhere in
  the range → the floor does not act as expected; report before driving.
- [ ] 4.2 Information only (open question, idle qualifier 0): ignition on, no brake command, push the
  pedal by hand at standstill and read `ib_runout` on the Brake card's fields line (live). Record
  whether it shows ~50 / 51 or ~103. No pass / fail; the result decides whether a later change sets
  the idle qualifier.
- [ ] 4.3 Brake versus engine, before (1.6.0) and after (1.6.1), same max stroke and rates: on level
  ground, brake held at 100 %, gear L, raise the throttle in steps; record the throttle % / RPM at which
  the buggy creeps, and the rod mm and `stop` at that moment. Pass for the owner's purpose: the buggy
  holds against more engine than before; no change = the ceiling is only a number (record it).
- [ ] 4.4 During 4.1–4.3 and the first drive: no `BRAKE` `fault`, `lost`, `unit_drop`, `no_move`
  lines and no request drop attributable to the speed byte.
- [ ] 4.5 Long hold at standstill: 100 %, 2 minutes, then release; note the unit's housing warmth by
  hand before / after and the battery voltage during the hold; compare with the same hold on 1.6.0 if
  practical. Record; the owner decides on any time limit.

## 5. Archive
- [x] 5.1 Archive after `replace-aztec-brake-with-ibooster` (and everything it must follow). After
  archiving, edit `openspec/specs/vehicle-systems/spec.md` "iBooster Rod Control" scenario "Vehicle
  speed to the unit" (drop "and 0 while it is invalid", point to "iBooster Vehicle Speed Floor") and
  `openspec/specs/event-journal/spec.md` "Service Brake Journal Events" (normal-braking sentence: the
  `episode` line is the exception); `openspec validate --strict`.
