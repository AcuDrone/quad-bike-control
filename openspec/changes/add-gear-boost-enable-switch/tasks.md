## 1. Firmware
- [x] 1.1 `Constants.h`: `BOOST_ENABLED_NVS_KEY` ("enabled"), `BOOST_ENABLED_DEFAULT` (true); bump `FIRMWARE_VERSION` to "1.5.13".
- [x] 1.2 `VehicleController`: `boostGearEnabled_` loaded at boot from NVS "boost" key `enabled` (absent → true) in the existing `target_rpm` block; `isGearBoostEnabled()`.
- [x] 1.3 `update()`: on `needsThrottleBoost()` start the PID only when enabled; when disabled (or when enabled mid-shift while the idle hold is on) set `shiftIdleHold_` and hold `throttle_.idle()` — no CAN/RPM precondition, no poll-interval change; on `needsThrottleBoost()` false release the hold with `throttle_.idle()` and `webThrottleDemandPct_ = 0`. `boostManualActive_` path unchanged, so `test_boost` ignores the switch.
- [x] 1.4 `isThrottleLockedForShift()` (`gearBoostActive_ || shiftIdleHold_`) used in the MAVLink apply path, the web re-limit block in `update()` and `processThrottleCommand()` ("Gear change active" reply) in place of `gearBoostActive_`.
- [x] 1.5 `set_boost_enabled` → `processSetBoostEnabledCommand()`: persist first (failure → error reply, state unchanged), then apply; if turning off while a gear-change PID is active (not the manual test), release like the normal path (`CAN_POLL_INTERVAL_RPM`, `throttle_.idle()`, `gearBoostActive_ = false`, `webThrottleDemandPct_ = 0`) and continue as an idle hold while the shift is still in progress; turning on mid-shift does not start the PID; English ack/error replies.

## 2. Telemetry and web
- [x] 2.1 `boost_enabled` (bool) in `WebPortal::Telemetry`, `TelemetryManager`, `createTelemetryJSON` (capacity note updated if needed).
- [x] 2.2 `data/index.html`: toggle beside the "RPM Boost Target" row in the Gear Position Defaults card, sends `set_boost_enabled`, filled from `boost_enabled`; reply in `gear-default-status`; uk + en keys (`lbl_gear_boost_enabled`: "Перегазовка при перемиканні" / "Gear-change RPM boost"); one sentence added to `gear_defaults_help` (uk + en); RPM target input stays enabled when off.

## 3. Docs, bench
- [x] 3.1 `Changelog.h` 1.5.13 bullet (uk + en).
- [ ] 3.2 Bench: the toggle shows ON on first boot.
- [ ] 3.3 Bench: turn it off, reboot, it reads back OFF.
- [ ] 3.4 Bench: with it off, shift L→N→L with the engine running and the throttle held open from the web/MAVLink: the throttle drops to idle for the dwell with no RPM rise, then the web demand reads 0 / MAVLink throttle resumes.
- [ ] 3.5 Bench: with it on, the shift blips to the target RPM as before; `test_boost` works in both states.
