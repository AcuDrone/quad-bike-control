## 1. Firmware
- [x] 1.1 `Constants.h`: `FIRMWARE_VERSION "1.1.0"`, add `BOARD_NAME` (`"Control_v0"` on master, `"DevKitC-1"` on old-board)
- [x] 1.2 `include/Changelog.h`: flash-resident JSON with 1.1.0 and 1.0.4 entries (uk + en)
- [x] 1.3 `WebPortal.h` / `TelemetryManager.cpp` / `WebPortal.cpp`: `board` field in telemetry struct and JSON
- [x] 1.4 `WebPortal.cpp`: `GET /api/changelog` route serving `CHANGELOG_JSON`

## 2. Web UI
- [x] 2.1 Status bar shows `version · board`; item is clickable and opens the changelog modal
- [x] 2.2 Changelog modal: fetch once, render per active language, highlight running version, re-render on language toggle
- [x] 2.3 "What's new" button in System Maintenance card
- [x] 2.4 i18n keys (en/uk) for the modal and button

## 3. Verification
- [x] 3.1 `pio run` builds on master
- [x] 3.2 Same change applied to `old-board` with its own `BOARD_NAME` and changelog wording; builds
- [ ] 3.3 Bench: status bar shows `1.1.0 · Control_v0`, modal opens in both languages
