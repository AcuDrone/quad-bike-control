## 0. Preconditions
- [x] 0.1 Proposal approved by the owner, including assumptions A1–A4 in proposal.md.
- [ ] 0.2 Archive order with `add-precrank-dwell-setting` (implemented in 1.5.33, not archived): both changes MODIFY "GCS Vehicle Command Interlocks" (`specs/vehicle-systems`). The copy here is taken from the current main spec and changes only the "Diff lock" bullet, its "RC and web paths" wording in the first sentence and the scenario "Diff lock holds against channel 7". Whichever change is archived second re-copies the requirement from the main spec and re-applies only its own edit.
- [x] 0.3 Work starts on `master` (tasks 1–4); `aztec` and `old-board` get it only by merging `master` (task 6).

## 1. Firmware (`master`)
- [x] 1.1 `src/VehicleController.cpp` `processMavlinkCommands()`: delete the channel 7 block (`:840-844`, `rcLock` / `rc_.lock.step()` / `setWheelLock()`); the comment above the RC section (`:794-797`) names gear / ignition / light only and says the lock is web / GCS only.
- [x] 1.2 `include/VehicleCommandLaw.h`: `VcmdRcState` loses `lock` (`RcEdge gear, ignition, light;`), `resync()` no longer resets it; the struct comment says "three RC functions"; the header comment drops `RC_WHEEL_LOCK_THRESHOLD` from the mirrored literals; `vcmdRcSwitch()` comment "On/off switch (light)". `vcmdLockDecide()` and `VCMD_LAW_CAP_LOCK` stay.
- [x] 1.3 `include/Constants.h`: delete `ServoChannelConfig::WHEEL_LOCK` (`:302`) and `RC_WHEEL_LOCK_THRESHOLD` (`:413`); the `ServoChannelConfig` comment notes channels 5, 7 and 8 unused (7: the lock moved off RC in 1.5.34); comment at `:217-218` "RelayController drives it from the web `set_wheel_lock` and the GCS lock command"; bump `FIRMWARE_VERSION` to "1.5.34".
- [x] 1.4 `include/MavlinkInterface.h:210` / `src/MavlinkInterface.cpp:1246-1249`: delete `getWheelLock()` (no callers); the static_assert at `:53-59` drops `&& RC_WHEEL_LOCK_THRESHOLD == 1520`.
- [x] 1.5 `include/RelayController.h` comments (`:18`, `:62-66`, `:167-178`): the lock is driven by the web and the GCS; no wording about release requests "every control loop from the MAVLink servo channel". Same for the `#else` comment in `RelayController::setWheelLock()` (`src/RelayController.cpp:400-404`). No code change in `RelayController`.
- [x] 1.6 `rg -n "WHEEL_LOCK\b|RC_WHEEL_LOCK|rc_\.lock|\.lock\.step|getChannel\(7" src include test` → no hits apart from `RELAY_SET_WHEEL_LOCK` / `RELAY_WHEEL_LOCK_FITTED` / `EFI_DIGITAL_FLAG_WHEEL_LOCK`.

## 2. Web (`master`)
- [x] 2.1 `data/index.html`: `lbl_ch7` → "Ch7" (`:835` default text, en `:1725`) and "Кан7" (uk `:2244`); the row and `mav-ch7` update stay.

## 3. Tests (`master`)
- [x] 3.1 `test/test_vehicle_cmd_law/test_main.cpp`: `test_resync_every_cause` drops the two `rc.lock.step(...)` lines (`:234`, `:247`); the comment above it names gear / ignition / light. No new case is needed for the removed path: `VcmdRcState` without `lock` makes it uncompilable.
- [x] 3.2 `pio test -e native` green; `pio run -e esp32-s3-devkitc-1` builds.

## 4. Docs and release (`master`)
- [x] 4.1 `include/Changelog.h` 1.5.34 entry (uk + en, operator wording): the lock no longer follows the RC transmitter switch (channel 7) — it is switched on and off only from the web page (Tests → Front wheel lock) or the ground station; channel 7 is now unused; the lock is released after a reboot; upload the updated LittleFS image.
- [x] 4.2 `MAVLINK_SETUP.md`: servo function table — `servo7_raw` row "unused since 1.5.34 (was the lock)" (`:157`) and the note under it (channels 5, 7 unused; "Since 1.5.30 channels 3, 4 and 6 …"); "Diff lock" interlock line (`:805`) "the same relays as the web `set_wheel_lock`"; "RC and GCS: last change wins" (`:813-818`) three channels, the lock not re-applied on re-sync, a GCS lock command holds until the next web / GCS lock command.
- [x] 4.3 `../../API/README.md` (UGV/API): RC channels table — CH7 row "блокування — до 1.5.33; з 1.5.34 не використовується" and the "остання зміна перемагає" row CH3 / 4 / 6 (CH7 до 1.5.33); `param1` 4 row note that since 1.5.34 the lock is set only by the GCS or the web; as for every `MAVLINK_SETUP.md` change.
- [x] 4.4 `GPIO_PINOUT_CUSTOM_BOARD_S3.md:361-363`: "the web `set_wheel_lock` command and the GCS lock command drive it (MAVLink servo channel 7 until 1.5.33)".
- [x] 4.5 `WEB_PORTAL_SETUP.md` (`:130`): the wheel lock is set only here or from the ground station, not from the RC transmitter; the MAVLink channel rows show Ch7 without a function.
- [x] 4.6 `rg -n -i "channel 7|servo7|ch7|CH7" *.md include src data` → only the historical notes written above.
- [x] 4.7 `openspec validate remove-rc-wheel-lock-channel --strict` passes.

## 5. Bench — quad (`master` 1.5.34)
- [ ] 5.1 Upload the LittleFS image; the MAVLink channel row reads "Ch7" / "Кан7" and still shows the live channel 7 value.
- [ ] 5.2 MAVLink in control, lock released: move the channel 7 switch up and down several times — Relay_5 / Relay_6 stay off, `wheel_lock_on` false, no `[RELAY] Front Wheel Lock` log line.
- [ ] 5.3 Engage the lock from the web; move the channel 7 switch both ways — the lock stays engaged. Release it from the web — it releases.
- [ ] 5.4 Lock engaged from the web, channel 7 down: take web control and release it, then pull the RC link for > 500 ms (fail-safe) and restore it — the lock stays engaged through both re-syncs while gear / ignition / headlight re-sync from CH3 / CH4 / CH6 as before.
- [ ] 5.5 Boot with the channel 7 switch up — the lock comes up released and stays released.
- [ ] 5.6 GCS lock ON / OFF (Mission Planner plugin or QGC) under MAVLink control — applied and acknowledged `ACCEPTED`, `EFI_STATUS.pt_compensation` bit0 follows; under web control — `TEMPORARILY_REJECTED` reason 3, as before.

## 6. Merges
- [x] 6.1 Merge `master` → `aztec`: the channel 7 block in `processMavlinkCommands()` (`aztec` `src/VehicleController.cpp:1056-1060`) goes; `Constants.h` keeps the aztec `RELAY_SET_WHEEL_LOCK` (Relay_4..6) and its comment block (`:260`, `:277-285`, `:403`) is reworded to web / GCS only; `Changelog.h` 1.5.34 ("багі"); `pio test -e native`, `pio run` on `aztec`.
- [ ] 6.2 Merge `master` → `old-board`: same removal (`old-board` `src/VehicleController.cpp:763-766`); keep the direct-GPIO relays; 1.5.34; `pio test -e native`, `pio run` on `old-board`.

## 7. Bench — buggy and old board, archive
- [ ] 7.1 Buggy (`aztec` 1.5.34): LittleFS upload; 5.2–5.6 with Relay_4..6 as the lock relays.
- [ ] 7.2 Old board (`old-board` 1.5.34): LittleFS upload; 5.2, 5.3 and 5.5 on the bench harness.
- [ ] 7.3 Archive this change (after the order in 0.2 is settled).
