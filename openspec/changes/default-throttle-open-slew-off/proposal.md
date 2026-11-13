# Change: Throttle opening rate limit OFF by default on both vehicles

## Why
`add-throttle-open-slew` (1.5.16) added an opening-only throttle rate limit with a default of
60 %/s, so every unit that never had the value saved ramps the throttle idle → full over ≈1.7 s.
That change was made because the quad's engine stalled, and the stall was put down to snapping the
throttle open from idle. The owner has since found the real cause: the engine was overheating
after the operator drove at maximum RPM for a long time. The throttle snap did not cause the
stall, so the opening slew is not needed on either vehicle.

The factory behaviour therefore goes back to the plain, instant throttle on both the quad (master)
and the buggy (aztec). The feature itself stays (0 = off or 30–100 %/s, set per vehicle from the
web) for anyone who wants a softer opening; only its default changes to 0.

Depends on `add-throttle-open-slew` (implemented on master 8f8afe1 and aztec a8a8910, not archived
yet): the MODIFIED requirements below are based on that change's version of "Configurable Throttle
Opening Rate" and "Throttle Opening Rate Configuration via Web Interface", which do not exist in
`openspec/specs/` until it is archived. Archive `add-throttle-open-slew` first, then this change.

## What Changes
- `THROTTLE_OPEN_SLEW_DEFAULT_PCT_S` 60 → **0** (`include/Constants.h`), with the comment updated:
  0 = off is the default; the limit is optional and set per vehicle from the web.
  `MIN` (30), `MAX` (100), the NVS key and `MAX_DT_S` are unchanged; 0 or 30–100 %/s stay the only
  accepted values.
- No logic change. The constructor initialiser, the `loadCalibration()` fall-backs for an absent or
  invalid stored value, and the "using default" log line all take the constant, so they follow.
- Web console (`data/index.html`, uk + en): the card help sentence "default 60 = idle to full in
  1.7 s" / "типово 60 = …" becomes "default 0 = off; 60 = idle to full in 1.7 s". The off hint "0 = off (throttle opens instantly, as before)"
  loses "as before". The input has no placeholder, so nothing else shows 60.
- `test/test_throttle_slew/test_main.cpp`: the local `RATE_DEFAULT = 60.0f` (commented as the
  default constant) is renamed `RATE_60` with the comment fixed; the law tests themselves are
  unchanged (they test the law at 60 %/s, not the default).
- Firmware **1.5.31** on master, merged master → aztec as usual; a changelog bullet (uk + en)
  saying the throttle opening limit is now off by default, and that a unit with a saved value keeps
  it. The 1.5.16 changelog entry is history and is left as written.
- The pending aztec proposal `add-gear-speed-ceiling` moves to 1.5.32.

### NVS behaviour (what happens on boards already in use)
- The firmware writes `"throttle"/open_slew` **only** when the operator saves a rate:
  `ThrottleController::setOpenSlewPctS()` (`src/ThrottleController.cpp:108-123`, `putFloat` at
  :115), called only from the web command `set_throttle_open_slew`
  (`src/VehicleController.cpp:1341-1359`). Nothing writes it at boot or on first run:
  `loadCalibration()` (`src/ThrottleController.cpp:191-217`) opens the namespace read-only
  (`prefs.begin("throttle", true)`, :193) and reads the key only if it exists (`isKey`, :197);
  `saveCalibration()` writes only `idle_us` / `full_us` (:168-171).
- So:
  - A board on which nobody ever pressed Save on the "Throttle opening rate" row has no key, and
    after flashing 1.5.31 it runs with 0 (off).
  - A board on which a rate was saved — **including 60, if the operator saved the shown default** —
    keeps that stored value after flashing. The new default does not touch it.
  - A stored invalid value (NaN, 0 < v < 30, > 100) now falls back to 0 (off) instead of 60, still
    logged and not rewritten.
- To get a board with a stored value to "off": set 0 in the web row and Save (writes 0). An NVS
  erase also works but wipes every other setting, so it is not the recommended path. No migration
  is proposed; an option, if the owner asks for it, is a one-shot boot step that removes an
  `open_slew` equal to 60 (guarded by a version marker key). It is not part of this change.

## Impact
- Affected specs: vehicle-systems (MODIFIED "Configurable Throttle Opening Rate"), web-control
  (MODIFIED "Throttle Opening Rate Configuration via Web Interface"). "Throttle Control System",
  the speed-limiter taper and the telemetry requirement are unchanged (their scenarios state the
  rate they assume).
- Affected code: `include/Constants.h`, `data/index.html`, `include/Changelog.h`,
  `test/test_throttle_slew/test_main.cpp`. No change to `ThrottleController`, `VehicleController`,
  telemetry or `platformio.ini`. `MAVLINK_SETUP.md`, `WEB_PORTAL_SETUP.md` and `UGV/API/README.md`
  do not mention the setting.
- Behaviour change: on a unit with no stored rate the throttle opens instantly, as before 1.5.16.
  This is the intended behaviour: the stall that motivated 1.5.16 was engine overheating, not the
  throttle snap.
- Buggy (aztec) only: the park-brake drive-away from `add-aztec-parking-brake` (1.5.29) holds the
  throttle at idle until the release point and then "ramps up with the opening slew". With the
  rate at 0 the throttle steps straight to the operator's demand at the release point instead of
  ramping. Its spec text ("SHALL pass through ... the opening slew") stays true (0 is a
  passthrough), but the bench task 6.5 of that change expects a ramp; the owner should decide
  whether the buggy gets a rate set in the web. Flagged, not changed here (that change lives on
  aztec only).
- LittleFS image must be uploaded for the help text; the firmware alone already changes the
  behaviour.
