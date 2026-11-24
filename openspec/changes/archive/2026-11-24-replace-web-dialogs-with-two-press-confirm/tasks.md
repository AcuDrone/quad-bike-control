## 1. Web UI
- [x] 1.1 `data/index.html`: shared helper `confirmTwice(btn, btnKey, warnKey, statusId)` /
  `confirmDisarm()` / `confirmRefreshLang()` (5 s window, per-button state, label swapped via
  `data-i18n`, warning cleared only if still shown); `.confirm-armed` style from `--warning`.
- [x] 1.2 `steerSensorZero`, `resetGearCodes`, `clearJournal`, `uploadFirmware` use it instead of
  `confirm()`; buttons pass `this`.
- [x] 1.3 Firmware update card: `fw-status` line; the four `alert()`s become `fwStatus()` messages
  (success / danger colours).
- [x] 1.4 i18n en + uk: approved warning texts and armed labels; no unused keys.
- [x] 1.5 `refreshDynamicLabels()` calls `confirmRefreshLang()`.
- [x] 1.6 `steerCapture` ("Set Left"/"Set Right") and `steerSensorZero` show `steer_no_connection`
  («Немає з'єднання з платою» / "No connection to the board") in `steer-center-status` in the danger
  colour when the WebSocket is not open; nothing sent, sensor zero not armed.
- [x] 1.7 No `alert(` / `confirm(` / `prompt(` left in tracked source.

## 2. Release
- [x] 2.1 `FIRMWARE_VERSION` → `"1.5.27"`; Changelog 1.5.27 uk + en.
- [x] 2.2 Amend the journal "Clear" scenario in `add-event-journal-and-link-recovery`.

## 3. Verification
- [x] 3.1 `pio run -e esp32-s3-devkitc-1` and `-t buildfs` succeed.
- [x] 3.2 Headless page harness (stubbed WebSocket/fetch/XHR): first press sends nothing, second
  press within 5 s sends the command, expiry restores the label, language switch while armed,
  "no file selected" in the status line; steering card disconnected → "no connection" message,
  nothing sent, sensor zero not armed; connected → unchanged.
- [ ] 3.3 Bench: on the phone's captive-portal webview, set the sensor zero, reset gear codes,
  clear the journal and upload firmware with the two-press confirmation.
