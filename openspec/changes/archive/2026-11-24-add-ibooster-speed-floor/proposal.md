# Change: iBooster is told at least 2 km/h, and each brake episode journals the unit's pressure ceiling

> **Accepted by the owner (decision 2026-11-23, not to be re-opened).**
> - It is **not verified** that the real brake pressure rises. The only measured effect is the number
>   the unit reports (`0x38F` byte 3, "pRunout"); the bench was dry. No source states that reporting a
>   speed raises the delivered pressure.
> - The unit's standstill pressure reduction (if that is what the 51 bar ceiling is) is **defeated on
>   purpose**. A long hold at standstill may draw more motor current and warm the unit more; long
>   holds are meant to be handed over to the parking brake.
> - The unit is told a speed that is **not true** below 2 km/h (it is told 2 km/h, also with a failed
>   speed sensor).
> - **Known limitation:** the effect is only known to act **during a CAN brake request** (see "What is
>   not known" below).

## Why
- First test on the buggy ("багі") with fluid (firmware 1.6.0): applied at standstill, the iBooster
  service brake cannot hold the buggy against the engine. The calipers are small single-piston ones
  and the master cylinder is the large Tesla one (~25.4 mm): the hydraulic ratio is poor, pressure
  is what is missing.
- The unit reports its own pressure ceiling in `0x38F` byte 3. On the bench
  (`Arduino/iBooster-TEST/README.md`, «Що виміряно на стенді», field table) it reads **51 bar** under a
  request while the vehicle speed we send in `0x38B` byte 2 is 0–2 counts and **103 bar** once it is
  ≥ 3 counts (0.4 km/h per count by the Rivian DBC scale, so ≥ 1.2 km/h); without a request it idles
  at 50; after a request is removed a 103 decays ~1 bar per 20 ms down to 50; at the stroke stop it
  reads 0 (then ≈ reported master-cylinder pressure − 21 bar, i.e. 0 with our fixed 5.1 bar).
- The public Rivian DBC for the same unit describes the signal as "In driver braking: maximum
  achievable master cylinder pressure with boost (runout). With an active pressure increase: maximum
  achievable brake master cylinder pressure", and the vehicle-speed input as "for standstill
  detection in iBooster … Logic for Pressure Reduction". Bosch EP2332796A1 describes lowering the
  booster's assist while holding at standstill. So it is plausible, **not verified**, that the unit
  caps its force lower at standstill.
- Firmware 1.6.0 sends the true wheel speed and **0 while the speed is invalid**
  (`include/IboosterLaw.h:197-203`, `src/IboosterBrake.cpp:223`): at standstill — and, with a failed
  speed sensor, even while moving — the unit sits at its low ceiling.

## What Changes
- **Speed floor** (owner decision): `0x38B` byte 2 = round(km/h / 0.4) of max(real speed,
  `IB_LAW_SPEED_FLOOR_KMH` = 2.0 km/h), clamped to 0..255. An invalid, NaN or non-positive speed
  gives the floor (5 counts), never 0. The constant lives in `include/IboosterLaw.h`; the floor is
  applied inside `ibSpeedByte()`, the one place the byte is computed, so it reaches every `0x38B`
  frame the firmware sends (with and without a request). 5 counts leaves 2 counts of margin above the
  measured threshold (3).
- **Nothing else sees the floored value.** The speed limiter, the governor, telemetry, MAVLink and the
  web page keep the true speed (`SpeedSensor` is not touched).
- **Journal: one line per brake episode** (source `BRAKE`, event `episode`): duration, peak rod, the
  lowest speed byte sent, the lowest and highest non-zero ceiling the unit reported after accepting
  the request, and whether it reported 0 (stroke stop). Exact rule in the spec delta and design D2.
  This is the only new journal line; it is subject to the journal's existing 2 s (source, event)
  dedupe, so pumping the brake cannot write more than one line per 2 s.
- **Unchanged:** `0x38B` byte 1 (`0x40` with a request, `0x00` without), byte 3 (`0x05`), `0x38C`,
  `0x38D` (5.1 bar, qualifier 1), the state machine, the rod law, the settings.
- **No web page change, no LittleFS upload.** (The Brake card already shows the live `ib_runout`
  value in its fields line — `data/index.html:4330`, `src/TelemetryManager.cpp:145` — which the
  technician can use for the live checks; nothing is added to it.)
- **Version 1.6.1**, aztec only (`FIRMWARE_VERSION` in `include/Constants.h`), `Changelog.h` entry
  uk + en, firmware image only.

## What is not known (recorded, not solved here)
- **Idle frames carry speed qualifier 0.** Per the bench field table, `0x38B` byte 1 bits 6–7 are
  `VehicleSpeed_Q` (0 not initialised, 1 normal, 2 faulty). `ibBuild()` sets `0x40` (= qualifier 1)
  in `0x38B` byte 1 only while `req` is true (`include/IboosterProto.h:113`, `:116`), `0x00`
  otherwise. The 51 → 103 change was measured with `0x40` set, i.e. during a request. Whether the
  unit uses the speed byte from idle frames (qualifier 0) — e.g. for a hand push on the pedal, which
  is driver braking without a request — was **not measured**. This change does **not** touch the
  qualifier bits. A technician check is added (tasks 4.1, 4.2); a later change may set the qualifier
  in idle frames if the owner wants it, after a bench test (an idle `0x40` in `0x38B` has never been
  sent to the unit).
- Whether a higher reported ceiling gives more real pressure (task 4.3 compares brake vs engine).
- The motor current / temperature cost of a long hold at standstill with the high ceiling (task 4.5).
- The speed scale (0.4 km/h per count) is the Rivian DBC value, unverified on the Tesla unit; the
  floor is chosen in counts that are known to work (5 ≥ 3).

## Relationship to other changes
- Builds on `replace-aztec-brake-with-ibooster` (firmware 1.6.0, committed on `aztec` 60205ac, not
  archived). Its requirements are not in `openspec/specs/` yet, so this change only **ADDS**
  requirements and **must be archived after** `replace-aztec-brake-with-ibooster` (and therefore
  after everything that one must follow, including `add-event-journal-and-link-recovery`, which
  creates the `event-journal` capability).
- Two sentences of `replace-aztec-brake-with-ibooster` are superseded and are reconciled by hand at
  archive time (task 5.1):
  - `vehicle-systems` → "iBooster Rod Control" → scenario "Vehicle speed to the unit" ("… and 0 while
    it is invalid");
  - `event-journal` → "Service Brake Journal Events" ("Normal braking … SHALL NOT be journaled"): the
    `BRAKING` / `RELEASING` transitions stay unjournaled; the one `episode` summary line is the
    exception.
- No other open change claims 1.6.1; `move-aztec-vesc-to-can` claims 1.7.0.

## Out of scope
- The `0x38B` / `0x38C` qualifier bits, the reported master-cylinder pressure, `0x38B` byte 3.
- Any web page change (tile line, test card), LittleFS image.
- `master` and `old-board` (no iBooster there).
- MAVLink (no new message or text; `UGV/API/README.md` unaffected).

## Impact
- Affected specs:
  - `vehicle-systems`: ADDED "iBooster Vehicle Speed Floor".
  - `event-journal`: ADDED "Service Brake Episode Line".
- Affected code (aztec): `include/IboosterLaw.h` (floor constant, `ibSpeedByte()`, the pure episode
  tracker), `src/IboosterBrake.cpp` / `include/IboosterBrake.h` (call the tracker in the slot, new
  event kind, journal line), `test/test_ibooster_law/test_main.cpp`, `include/Constants.h`
  (`FIRMWARE_VERSION`), `include/Changelog.h`.
- Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md` (iBooster row: speed byte floor), `WEB_PORTAL_SETUP.md`
  (journal `BRAKE I episode` line, how to read it), the comment block of `include/IboosterProto.h`
  (speed-byte and qualifier note).
- Hardware / flashing: firmware image only.
- Safety: possibly more brake force at standstill (unverified); possibly more unit heating on long
  standstill holds; the unit never sees speed 0.
