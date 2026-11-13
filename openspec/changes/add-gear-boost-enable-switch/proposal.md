# Change: Per-vehicle on/off switch for the gear-change RPM boost

## Why
The gear-change RPM boost ("перегазовка") holds the engine at `boostTargetRpm_` with a PID while
the shift actuator dwells (`TransmissionController::needsThrottleBoost()`). The quad bike needs it;
the aztec buggy (ODES JUNGLECROSS 800) shifts fine without an RPM blip, but its throttle must
still be closed while the shift actuator moves. Today the blip cannot be turned off without a code
change.

## What Changes
- New persisted per-vehicle switch: NVS namespace `"boost"` (already holds `target_rpm`), key
  `enabled` (bool), default **true** — the quad bike keeps today's behaviour, no code default
  differs between branches. The aztec unit is switched off once from the web.
- When the switch is off, a gear change (`transmission_.needsThrottleBoost()` true) does not start
  `updateGearBoostPID()`; instead an idle hold (`shiftIdleHold_`) keeps `throttle_.idle()` for the
  whole shift and ignores MAVLink and web throttle. Unlike the PID it needs no valid CAN or engine
  RPM (idle is always safe) and does not change the RPM poll interval. When `needsThrottleBoost()`
  goes false it releases with `throttle_.idle()` and `webThrottleDemandPct_ = 0` (as the PID does);
  MAVLink throttle resumes on the next loop.
- One accessor `isThrottleLockedForShift()` = `gearBoostActive_ || shiftIdleHold_` replaces the
  `gearBoostActive_` checks in the three throttle gates: the MAVLink apply path, the web re-limit
  block in `update()`, and `processThrottleCommand()` (rejects with "Gear change active"). ON and
  OFF share the same lock.
- Turning the switch off while the PID is active releases the PID through the normal release path
  (`CAN_POLL_INTERVAL_RPM` restored, `throttle_.idle()`, `gearBoostActive_ = false`,
  `webThrottleDemandPct_ = 0`) and, if the shift is still in progress, continues as an idle hold
  until it ends. Turning it on mid-shift keeps the idle hold until the shift ends; no PID is started
  mid-dwell.
- Gear-change logic, interlocks and the overshoot dwell are unchanged.
- The manual web test `test_boost` ignores the switch (stays usable for tuning).
- Web command `set_boost_enabled` (bool): persisted first, then applied; English ack/error replies
  ("Gear boost enabled" / "Gear boost disabled" / "Failed to save gear boost setting").
- Telemetry key `boost_enabled` (bool) beside `boost_target_rpm`.
- Web UI: a toggle "Перегазовка при перемиканні" / "Gear-change RPM boost" next to the "RPM Boost
  Target" row in the "Gear Position Defaults" card, filled from telemetry; one help sentence in
  `gear_defaults_help`. The RPM target row stays editable when the boost is off.
- Firmware 1.5.13 (`FIRMWARE_VERSION`) with a changelog bullet (uk + en).

## Impact
- Affected specs: vehicle-systems (Throttle Boost During Gear Changes), web-control (Gear Position
  Default Configuration via Web Interface + new requirement), web-telemetry (new requirement).
  The legacy can-controller "Throttle Boost During Gear Changes" requirement is not touched.
- Affected code: `include/Constants.h`, `include/VehicleController.h`, `src/VehicleController.cpp`,
  `include/WebPortal.h`, `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`,
  `include/Changelog.h`
- No NVS migration: an absent `enabled` key reads as true, so existing units behave as before.
- Ships after 1.5.12 (pending, uncommitted `add-speed-limiter-slew-setting` and
  `add-steering-serial-diagnostics`); neither touches the boost requirements, but both edit the
  same files (`VehicleController`, `WebPortal`, `TelemetryManager`, `index.html`, `Changelog.h`,
  `Constants.h`), so this is implemented on top of them.
- No native test covers the boost PID (`test/` has none); verification is a bench check.
