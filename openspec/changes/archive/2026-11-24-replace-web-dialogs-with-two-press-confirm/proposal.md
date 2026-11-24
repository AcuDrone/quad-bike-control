# Change: Replace native browser dialogs with a two-press confirmation

## Why
Operators open the service page in browsers where native dialogs are suppressed: the phone's
captive-portal "sign in to network" webview, or Chrome after "prevent this page from creating
additional dialogs". `confirm()` then returns false and the action silently does nothing — this is
what blocked an operator from setting the steering sensor zero. `alert()` messages are lost the
same way.

User decision 2026-11-12: replace every native dialog with in-page confirmation (design and texts
approved).

## What Changes
- One shared helper, `confirmTwice()`, in `data/index.html`: the first press arms the button (label
  becomes the confirm label, warning-colour outline, one-line warning in the card's status line,
  nothing sent); a second press of the same button within 5 s runs the action exactly as before;
  after 5 s the label is restored and the warning cleared only if the status line still shows it.
  Each button arms independently. The armed label is swapped through `data-i18n`, so a language
  switch re-renders it; a warning still on screen is re-rendered in the new language.
- Applies to "Set sensor zero" (`steer-center-status`), gear codes "Reset to defaults"
  (`gear-codes-status`), event journal "Clear" (`journal-status`) and "Upload Firmware" (new
  `fw-status` line in the firmware update card).
- The four firmware-upload `alert()`s (select a file first / success / failed / error) become
  messages in `fw-status` with the success/danger colours, wording unchanged.
- Existing pre-checks keep their place: WebSocket open before arming "Set sensor zero", file
  selected before arming "Upload Firmware"; the gear-codes reset still checks the WebSocket after
  confirmation, as before.
- Steering calibration card: "Set Left", "Set Right" and "Set sensor zero" no longer do nothing
  silently when the WebSocket is not open — the card's status line shows «Немає з'єднання з
  платою» / "No connection to the board" in the danger colour; nothing is sent and "Set sensor
  zero" is not armed. Other buttons are unchanged.
- i18n (en + uk): the four `*_confirm*` warning texts are replaced by the approved one-line texts,
  four armed-label keys and `steer_no_connection` added; no key left unused.
- Firmware 1.5.27 (web-only change; version and Changelog entry as for every web change).

## Impact
- Affected specs: `web-control` (new "Two-Press Confirmation for Destructive Web Actions";
  "Steering Calibration and Sensor Zero via Web Interface" sensor-zero scenario and a new
  "No connection to the board" scenario), `ota-updates`
  ("OTA Web Interface"), `web-server` ("Web UI Localization" dynamic-strings list).
- The journal "Clear" scenario lives in the not yet archived change
  `add-event-journal-and-link-recovery` (ADDED "Event Journal Card"); its wording is amended there
  to point at the two-press confirmation.
- Affected code: `data/index.html`, `include/Constants.h` (version), `include/Changelog.h`.
- No firmware logic, command or API change. The LittleFS image must be re-uploaded.
