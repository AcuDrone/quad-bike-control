# Change: Aztec lock on Relay_4 + Relay_5 + Relay_6

## Why
`update-aztec-relay-assignment` (archived 2026-11-01) reserved Relay_4 (PCA9557 IO7 / X7, XL9535
P0.3), Relay_5 (IO3 / X8, P0.4) and Relay_6 (IO2 / X9, P0.5) for "three separate locks whose
functions are specified in a later change", and left the firmware's wheel-lock function not fitted
(`RELAY_WHEEL_LOCK_FITTED = 0`). The buggy needs its lock ("блокування") now: the operator wants
relays 4, 5 and 6 to carry it, from the web UI and from the autopilot.

## What Changes
- **Relay map (aztec):** the existing single lock function drives Relay_4, Relay_5 and Relay_6
  **together** as one set: `RELAY_SET_WHEEL_LOCK = RELAY_N(4) | RELAY_N(5) | RELAY_N(6)` (PCA9557
  mask `0x8C` = IO7|IO3|IO2 — the non-1:1 routing is kept; XL9535 mask `0x38` = P0.3..P0.5).
  `RELAY_WHEEL_LOCK_FITTED = 1`. `RELAY_SET_RESERVED` shrinks to the spares Relay_7 + Relay_8
  (`0x03` / `0xC0`). Compile-time checks (sets disjoint and covering all eight relays, masks
  disjoint and OR to `0xFF` on BOTH boards) stay, and the aztec mask static_asserts now pin the
  lock and reserved masks on both boards.
- **Lock behaviour:** `setWheelLock(true)` energizes the whole lock mask through the same verified
  whole-port shadow write as ignition and light; `setWheelLock(false)` releases it; `getWheelLock()`
  reports the commanded state; `isWheelLockFitted()` is `true`. The "not fitted → reject + log once"
  branch stays in the code, governed by `RELAY_WHEEL_LOCK_FITTED`.
- **Commands:** web `set_wheel_lock` (answered "Lock LOCKED" / "Lock UNLOCKED") and MAVLink servo
  channel 7 (`ServoChannelConfig::WHEEL_LOCK`, > `RC_WHEEL_LOCK_THRESHOLD` 1520 µs = LOCKED), exactly
  the quad bike's paths. No input-source gate and no ignition interlock — the quad bike has none
  either.
- **Telemetry:** `wheel_lock_fitted` = `true`, `wheel_lock_on` follows the lock, `EFI_STATUS`
  digital-output bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) is set while the lock is engaged.
- **Fail-safe (same as the quad bike):** `allOff()` switches ignition, starter and light off and
  deliberately **holds** the lock in its last commanded state, exactly as master's wheel lock: the
  lock is independent of the ignition relays, so it keeps its state through a MAVLink failsafe /
  link drop. `allOff()` is master's code unchanged; there is no aztec-only switch.
- **Web relay test:** a test relay can be switched ON only with the lock released too (the quad
  bike's rule, already in the code under `RELAY_WHEEL_LOCK_FITTED`); a LOCK command switches every
  test relay off. Relay labels 4/5/6 read "lock (with 5+6)" / "(with 4+6)" / "(with 4+5)" (uk:
  "блокування (з …)"). The UI now picks the aztec / quad label set from the aztec-only `gear_codes`
  telemetry object, because `wheel_lock_fitted` is `true` on both vehicles.
- **Web UI:** the lock button is enabled (driven by `wheel_lock_fitted`); its card is renamed
  "Lock" / "Блокування".
- Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 / §6a / warning 8 / pin-migration table,
  `MAVLINK_SETUP.md` (servo7 and EFI bit0), `include/Constants.h`, `include/RelayController.h`.
- Version 1.5.25 with a changelog entry (uk + en). The LittleFS image must be re-uploaded.

## Impact
- Affected specs: `gpio-expander`, `mavlink-interface`, `vehicle-systems`, `web-control`.
  `web-telemetry` needs no change (field names and the `relay_test` encoding are unchanged).
- Affected code: `include/Constants.h`, `include/RelayController.h`, `src/RelayController.cpp`,
  `include/RelayBoardLaw.h` (comment), `include/VehicleController.h`, `src/VehicleController.cpp`,
  `src/MavlinkInterface.cpp` (comment), `src/main.cpp` (comments), `include/WebPortal.h` /
  `src/WebPortal.cpp` (comments), `data/index.html`, `include/Changelog.h`,
  `test/test_relay_board/test_main.cpp`.
- Hardware: the lock harness on X7/X8/X9 must be wired so that energizing all three relays together
  is the "locked" state. A MAVLink channel 7 that sits above 1520 µs now really locks.
