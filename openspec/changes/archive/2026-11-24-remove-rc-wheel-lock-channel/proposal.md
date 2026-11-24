# Change: The lock no longer follows RC channel 7

## Why
The operator switches the lock ("блокування", the front-wheel / diff lock) on with the RC
transmitter's channel 7 switch without noticing, and the vehicle then drives with the lock engaged.
Since 1.5.30 the switch is applied on every change of its decoded value
(`src/VehicleController.cpp:840-844`: `vcmdRcSwitch(getChannel(ServoChannelConfig::WHEEL_LOCK))` →
`rc_.lock.step()` → `relayController_.setWheelLock()`), and on every re-sync — boot, fail-safe
exit, return from web control — the first valid frame applies the switch position again
(`VcmdRcState::resync()`, `include/VehicleCommandLaw.h:217-223`). A switch knocked into the upper
position therefore engages the lock immediately, and a switch left there re-engages it after every
link loss even when the lock was released from the web or the ground station. The owner asked for
the lock to be removed from channel 7 (2026-11-16).

## What Changes
- **Channel 7 no longer drives the lock.** `processMavlinkCommands()` stops reading
  `ServoChannelConfig::WHEEL_LOCK`; the `rc_.lock` step and its `setWheelLock()` call are deleted.
  Channel 7 at any value — in band, out of band, changing, constant, on the first frame after a
  re-sync — has no effect on the lock or on any other function.
- **RC tracker without the lock.** `VcmdRcState` (`include/VehicleCommandLaw.h`) loses its `lock`
  member and `resync()` no longer resets it, so no code path can apply channel 7 to the lock. The
  RC "last change wins" arbitration stays for CH3 gear, CH4 ignition and CH6 headlight, unchanged.
- **Channel 7 constants removed, the channel becomes unused.** `ServoChannelConfig::WHEEL_LOCK`,
  `RC_WHEEL_LOCK_THRESHOLD` and `MavlinkInterface::getWheelLock()` (already without callers) are
  deleted; the static_assert in `src/MavlinkInterface.cpp:53-59` drops its `RC_WHEEL_LOCK_THRESHOLD`
  term; channel 7 is documented as intentionally unused, like channels 5 and 8. It is not reassigned.
- **The lock itself stays** (assumption A1). It is driven only by:
  - the web `set_wheel_lock` command (toggle in Tests → "Front Wheel Lock"), accepted from any input
    source, as today;
  - the GCS `MAV_CMD_USER_2` lock function (`param1` 4) through `executeVehicleCommand()`
    (`src/VehicleController.cpp:932-938`), with its interlock unchanged (reason 3 when not under
    MAVLink control, `FAILED` with the relay board unavailable). Capability bit4 stays set where the
    lock is fitted.
  Telemetry `wheel_lock_on` / `wheel_lock_fitted`, `EFI_STATUS.pt_compensation` bit0
  (`EFI_DIGITAL_FLAG_WHEEL_LOCK`), the relay sets and the relay-test interlock are unchanged.
- **Engaged lock holds as commanded.** A lock set from the web or the GCS stays in that state until
  the next web or GCS lock command. Channel 7 never changes it — not on a switch movement, not at
  the moment MAVLink control is gained, not on the fail-safe exit or the return from web control.
  The fail-safe all-off keeps the lock in its last state, as today (`RelayController::allOff()`).
- **Boot state unchanged:** the lock is released at boot. `RelayController::begin()` sets
  `wheelLockOn_ = false` and writes the port all-off (`src/RelayController.cpp:196-200`); the lock
  state is not persisted. Before this change the first valid RC frame could then engage it from
  channel 7; after it, the lock stays released until a web or GCS command.
- **Web.** The raw channel row keeps showing channel 7's value, relabelled like the other unused
  channels (assumption A3): `lbl_ch7` "Ch7 (Lock)" → "Ch7", "Кан7 (Блокув.)" → "Кан7"
  (`data/index.html:835`, `:1725`, `:2244`), matching "Ch5" / "Ch8".
- **Docs.** `MAVLINK_SETUP.md` (servo function table: `servo7_raw` unused since 1.5.34; the
  diff-lock interlock line "the same relays as RC channel 7"; "RC and GCS: last change wins" from
  four channels to three), `GPIO_PINOUT_CUSTOM_BOARD_S3.md:361`, `WEB_PORTAL_SETUP.md` (the wheel
  lock is set from the web or the ground station only), header comments in `include/Constants.h`,
  `include/RelayController.h`, `include/VehicleCommandLaw.h`, and the UGV API compatibility page
  `../../API/README.md` (RC channels table: CH7 removed in 1.5.34).
- **Version.** Firmware **1.5.34**; changelog entry uk + en ("багі" in Ukrainian).

### Assumptions (the owner may overrule any of them)
- **A1.** Only the RC channel 7 path is removed. The lock stays, driven by the web and the GCS; its
  telemetry stays.
- **A2.** Both vehicles: the code is shared. `master` (quad bike) first, then merged into `aztec`
  (buggy, lock = Relay_4 + Relay_5 + Relay_6) and `old-board`.
- **A3.** Channel 7 becomes unused, not reassigned to another function; the web keeps showing its
  raw value under a neutral label "Ch7" / "Кан7".
- **A4.** No automatic release of the lock on any event other than a reboot (see the open question
  in the report: with channel 7 gone, an operator with only the RC transmitter cannot release a lock
  engaged from the web or the GCS).

Out of scope: the Ch4 label "Ch4 (Brake)" / "Кан4 (Гальмо)" on the web page (channel 4 is the
ignition); the unused `MavlinkInterface::getFrontLight()`; the "Front-Wheel Lock Not Fitted" and
"Relay Controller for Ignition and Lights" requirements still describing the aztec buggy as having no
lock, although `aztec` fits it on Relay_4..6 since 1.5.25 (only their channel 7 wording is edited
here).

## Impact
- Affected specs:
  - `vehicle-systems`: ADDED "Lock Not Driven by an RC Channel"; MODIFIED "Quad Bike Relay
    Assignment With Fitted Front-Wheel Lock", "Front-Wheel Lock Not Fitted", "GCS Vehicle Command
    Interlocks", "Last-Change-Wins Arbitration of RC Channels and GCS Commands".
  - `mavlink-interface`: MODIFIED "Servo Channel to Vehicle Command Mapping", "Relay Controller for
    Ignition and Lights".
  - `web-telemetry`: ADDED "Unassigned Servo Channel Labels".
- **Overlap with `add-precrank-dwell-setting`** (implemented in 1.5.33, not archived): it also
  MODIFIES "GCS Vehicle Command Interlocks" (the ignition ON line). The copy here is taken from the
  current `openspec/specs/` and changes only the diff-lock line; whichever change is archived second
  must re-copy the requirement and re-apply its own edit (task 0.2).
- Affected code (`master`): `src/VehicleController.cpp`, `include/VehicleCommandLaw.h`,
  `include/Constants.h`, `include/MavlinkInterface.h`, `src/MavlinkInterface.cpp`,
  `include/RelayController.h` (comments), `data/index.html`, `include/Changelog.h`,
  `test/test_vehicle_cmd_law/test_main.cpp`, `MAVLINK_SETUP.md`, `GPIO_PINOUT_CUSTOM_BOARD_S3.md`,
  `WEB_PORTAL_SETUP.md`, `../../API/README.md`.
- Behaviour change on existing units: moving the channel 7 switch no longer engages or releases
  the lock; a transmitter with channel 7 in the upper position no longer engages the lock at boot or
  after a link loss. Operators release or engage the lock from the web page or the ground station.
- No NVS change. The LittleFS image must be uploaded on every vehicle (label change only).
- Branches: `master` first; merge `master` → `aztec` and `master` → `old-board`.
