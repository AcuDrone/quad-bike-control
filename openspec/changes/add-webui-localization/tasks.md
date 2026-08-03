## 1. String inventory
- [x] 1.1 Catalog all static user-facing text in `data/index.html` (header/title, status
      bar, card `<h2>`/`<h3>` headings, telemetry labels, button captions, help `<p>`
      blocks, warnings) and assign each a stable i18n key.
- [x] 1.2 Catalog all dynamic JS strings (connection state, MAVLink Active/Inactive,
      gear-switching indicator, capture/save/boost status lines, `alert()`/`confirm()`
      dialogs, debug/boost toggles, ECU probe panel text, PROBE_PID_INFO names) and assign
      keys, noting which need value substitution (`%s`).
- [x] 1.3 Identify firmware-sourced values: separate stable **enum tokens** (`input_source`,
      `can_status`, `gear`, `ignition_state`, `mav_active`, `steer_sensor_ok`,
      `steer_calibrated`, `is_cranking`, `probe.status`) from free-form `sendResponse`
      `message` strings; confirm the free-form ones stay English (protocol frozen).

## 2. Translation infrastructure (frontend only)
- [x] 2.1 Add an inline `const I18N = { en: {…}, uk: {…} }` dictionary in the existing
      `<script>` — no external files, no fetches.
- [x] 2.2 Add a `t(key, ...args)` helper: active-locale lookup, fallback to `en`, then to
      the key; support `%s` value substitution for dynamic strings.
- [x] 2.3 Add `applyStaticTranslations()` that walks `data-i18n` (and
      `data-i18n-title` / `data-i18n-placeholder`) attributes and fills text/attributes.
- [x] 2.4 Add client-side maps from firmware enum tokens to localized labels for both
      locales, used wherever telemetry sets those display fields.

## 3. Language state, toggle, persistence
- [x] 3.1 Add `currentLang` state; on load read `localStorage['lang']`, else auto-detect
      from `navigator.language(s)` (`uk*` → `uk`, else `en`), fallback English.
- [x] 3.2 Add the header toggle button (`EN` / `УК`) that switches language instantly with
      no page reload: re-run `applyStaticTranslations()`, refresh dynamic labels, update
      `document.documentElement.lang`, and persist to `localStorage`.

## 4. Apply translations
- [x] 4.1 Add `data-i18n*` attributes to every static element from task 1.1.
- [x] 4.2 Replace hard-coded dynamic strings in JS with `t(...)` calls (task 1.2).
- [x] 4.3 Route firmware enum tokens through the localized maps (task 2.4); leave free-form
      `message` strings and their existing English-substring routing untouched.
- [x] 4.4 Localize unit labels where applicable (`km/h` → `км/год`); keep `°C`, `kPa`,
      `µs`, `Hz`, `ms`, `%`, `V` unchanged; do not reformat numbers.

## 5. Ukrainian content
- [x] 5.1 Author proper technical Ukrainian for every `uk` key (gears, throttle, steering,
      calibration, ignition, CAN/ECU, OTA terminology).
- [x] 5.2 Verify English `en` values match the current on-screen wording so behavior is
      unchanged when English is selected.

## 6. Verification & deployment
- [x] 6.1 Confirm no remaining hard-coded user-facing English text outside the dictionary
      (grep audit); confirm every key exists in both `en` and `uk`.
- [x] 6.2 Confirm the page still has zero external/network dependencies (no CDN, no fetch,
      no remote fonts) and the Sage Garden theme is visually unchanged.
- [ ] 6.3 Deploy the web UI to LittleFS with `pio run -t uploadfs` (firmware flash alone
      does not update the served page).
- [ ] 6.4 On-device review by the native Ukrainian-speaking operator: switch EN⇄УК, verify
      persistence across reloads, first-visit auto-detection, `<html lang>` update, and
      Ukrainian quality/terminology on every card.
- [x] 6.5 Run `openspec validate add-webui-localization --strict` and resolve any issues.
