## 1. Firmware (master, then merge master → aztec)
- [x] 1.1 `include/Constants.h`: `THROTTLE_OPEN_SLEW_DEFAULT_PCT_S` 60.0f → 0.0f; trailing comment "0 = off by default; 60 %/s (idle→full 1.67 s) is an example opening rate; the limit is optional"; adjust the block comment above it (0 = off is the default; per-vehicle, set from the web). Bump `FIRMWARE_VERSION` to "1.5.31".
- [x] 1.2 Check that no code still assumes 60: constructor `openSlewPctS_(THROTTLE_OPEN_SLEW_DEFAULT_PCT_S)` and `loadCalibration()` fall-backs in `src/ThrottleController.cpp` (lines 12, 198, 204-205) take the constant — no edit expected; the invalid-value log now reads "using default 0 %/s".

## 2. Web
- [x] 2.1 `data/index.html` en `throttle_open_slew_help`: "… 0 = off, 30–100 %/s (default 0 = off; 60 = idle to full in 1.7 s)."
- [x] 2.2 `data/index.html` uk `throttle_open_slew_help`: "… 0 = вимкнено, 30–100 %/с (типово 0 = вимкнено; 60 = від холостих до повного за 1,7 с)."
- [x] 2.3 `data/index.html` `throttle_open_slew_off` (en + uk): drop "as before" / "як раніше" → "0 = off (throttle opens instantly)" / "0 = вимкнено (газ відкривається миттєво)".
- [x] 2.4 Confirm the input `#throttle-open-slew-input` has no placeholder / value of 60 and no other 60 default remains in the page.

## 3. Tests, changelog, docs
- [x] 3.1 `test/test_throttle_slew/test_main.cpp`: rename `RATE_DEFAULT` → `RATE_60`, comment "60 %/s test rate" (not the firmware default); no assertion changes.
- [x] 3.2 `pio test -e native` green; `pio run` builds.
- [x] 3.3 `include/Changelog.h` 1.5.31 entry (uk + en): throttle opening limit is now off by default (instant throttle, as before 1.5.16); a unit with a saved rate keeps it; to turn it off set 0 in the Speed limit and steering card; the limit stays available (30–100 %/s) for a softer opening. Leave the 1.5.16 entry as is.
- [x] 3.4 `MAVLINK_SETUP.md`, `WEB_PORTAL_SETUP.md`, `UGV/API/README.md`: checked, no mention of the opening rate — nothing to update.
- [x] 3.5 (done, aztec merge 49d0de2) Merge master → aztec (the slew code, test and `add-throttle-open-slew` folder are identical on both branches; only the line numbers in `Constants.h` / `index.html` / `Changelog.h` differ). Aztec `add-gear-speed-ceiling` becomes 1.5.32.

## 4. Bench (both vehicles: quad on master, buggy on aztec)
Archived 2026-11-13 at the user's request before bench verification; bugs found later come back as new changes.
- [ ] 4.1 Before flashing, note the "Throttle opening rate" value shown in the web. If it shows a number (e.g. 60) on a unit that should be off, that value is stored in NVS and will survive the update: after flashing set 0 and Save ("Throttle open rate off"), reboot, it reads back 0.
- [ ] 4.2 Fresh NVS (no `open_slew` key, e.g. a board that never saved the row, or after an erase): the row shows 0 and the hint "0 = off".
- [ ] 4.3 At 0: snap the stick (MAVLink) and the web slider idle → full — the servo goes to full with no ramp (≈0.15 s); closing is instant.
- [ ] 4.4 Set 60 and Save: reply "Throttle open rate set to 60 %/s"; reboot, it reads back 60; idle → full now takes ≈1.7 s; closing still instant; 20 and 110 are refused.
- [ ] 4.5 Quad: if 60 (or another rate) is stored from 1.5.16, set 0 per 4.1 — the stall was engine overheating, not the snap; record the final value.
- [ ] 4.6 Buggy: with the rate at 0, park-brake drive-away (`add-aztec-parking-brake` task 6.5) steps the throttle to the demand at the release point instead of ramping — check there is no lurch, or set a rate in the web.
