# Change: Add Ukrainian/English localization to the web UI

## Why
The QuadBike control portal (`data/index.html`) is served entirely in English, but the
primary operators are Ukrainian speakers. A field operator working next to the vehicle
should be able to read gear, throttle, calibration, and safety-status labels in Ukrainian
without any change to the firmware, the WebSocket/HTTP protocol, or the offline (no-CDN)
constraint that governs the portal.

## What Changes
- Add an inline JavaScript translation dictionary with two locales — English (`en`) and
  Ukrainian (`uk`) — embedded directly in `data/index.html`. No external files, no fetches.
- Apply translations to all static text via `data-i18n` attributes and to all
  JS-generated strings via a `t(key)` helper. English is the fallback for any missing key.
- Add a language toggle button in the page header (showing `EN` / `УК`) that switches the
  entire UI instantly, with no page reload.
- Persist the chosen language in `localStorage`; on first visit, auto-detect from
  `navigator.language` (`uk*` → Ukrainian, otherwise English).
- Update the `<html lang>` attribute when the language changes.
- Map firmware-sourced **enum/status tokens** arriving in WebSocket telemetry (e.g.
  input source, CAN status, sensor OK/FAULT, calibration state, ignition state, MAVLink
  active/inactive, ECU probe status) to localized display labels **client-side only**.
- Translate unit labels where a Ukrainian convention exists (e.g. `km/h` → `км/год`);
  keep language-neutral units unchanged (`°C`, `kPa`, `µs`, `Hz`, `%`, `V`, `ms`).
- **No firmware/C++ changes.** The WebSocket and HTTP JSON contract is unchanged. Free-form
  firmware response `message` strings (from `sendResponse(...)`) remain in English because
  the client already routes them by matching English substrings and the protocol must not
  change (see `design.md`).

## Impact
- Affected specs: `web-server` (adds localization requirements to the capability that owns
  the served web UI / visual theme).
- Affected code (implementation stage only, not part of this proposal): `data/index.html`
  (single-file web UI on LittleFS). No changes to `src/`, `include/`, or any C++ source.
- Deployment: the web UI lives on LittleFS, so shipping requires `pio run -t uploadfs`
  (a firmware flash alone does not update the page).
- Out of scope: MAVLink text, serial/debug log strings, and any language beyond
  Ukrainian and English.
