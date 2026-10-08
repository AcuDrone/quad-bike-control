# Change: Firmware 1.1.0 — board name in telemetry and a user-facing changelog in the web UI

## Why
`FIRMWARE_VERSION` has been `1.0.4` since 2026-03-12 while ~100 commits of new behaviour landed
(MAVLink instead of SBUS, VESC steering, the Control_v0 board, hall speed sensor, speed limiter,
odometer/trip, engine hours, ECU telemetry, localization). Worse, the `old-board` branch (DevKitC-1
hand wiring) carries the *same* `1.0.4`, so the web UI cannot tell an operator which board the
firmware was built for, nor what changed since the last flash. There is no release note anywhere a
user can read.

## What Changes
- **Version bump** to `1.1.0` on both `master` (Control_v0) and `old-board` (DevKitC-1).
- **NEW `BOARD_NAME` constant** in `Constants.h` (one value per branch), serialized into the
  telemetry JSON as `"board"` and shown next to the version in the status bar
  (`Firmware: 1.1.0 · Control_v0`). Kept as a separate field so `firmware_version` stays a clean
  semver string for anything that parses it.
- **NEW `include/Changelog.h`**: a flash-resident JSON document listing releases, each with a
  version, date and plain-language bullet lists in Ukrainian and English. The changelog lives in
  the firmware binary, not in `index.html`, so it can never drift from the version it describes
  (firmware and LittleFS are flashed independently over OTA).
- **NEW `GET /api/changelog`** on the web portal, returning that document verbatim.
- **Web UI**: the firmware status item becomes a button that opens a "What's new / Що нового"
  modal; the modal fetches `/api/changelog` once and renders the entries in the active language,
  highlighting the running version. A second entry point sits in the System Maintenance card
  next to Firmware Update.

## Impact
- Affected specs: `web-telemetry` (Firmware Version Display requirement modified; new Firmware
  Changelog requirement).
- Affected code: `include/Constants.h`, `include/Changelog.h` (new), `include/WebPortal.h`,
  `src/TelemetryManager.cpp`, `src/WebPortal.cpp`, `data/index.html`.
- No behavioural change to vehicle control. One extra HTTP route; one extra short string in the
  5 Hz telemetry JSON.
