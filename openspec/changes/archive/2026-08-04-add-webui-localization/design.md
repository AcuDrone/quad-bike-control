## Context
The entire web portal is a single ~1600-line `data/index.html` served from LittleFS while
the ESP32 runs as an offline WiFi Access Point. There is no build step, no bundler, and no
network access — the recently applied "Sage Garden" theme is inlined for exactly this
reason (no CDN, no remote fonts). Any localization must live inside this one file and must
not add a runtime dependency.

User-facing strings fall into three groups:
1. **Static markup** — headings, telemetry labels, button captions, help paragraphs, and
   warnings written directly in the HTML.
2. **Dynamic JS strings** — text produced in `<script>`, e.g. connection state
   (`Connected`/`Disconnected`), MAVLink `Active`/`Inactive`, gear-switching indicator,
   ECU probe panel (`Probing ECU…`, `Supported-PID bitmaps`, `Candidate PIDs`,
   `Diagnostic Trouble Codes`, `No stored DTCs`, …), capture/save status lines,
   `alert()`/`confirm()` dialogs, and the debug/boost toggles.
3. **Firmware-sourced strings** — arriving in WebSocket telemetry JSON from
   `src/WebPortal.cpp` / `src/VehicleController.cpp`.

Group 3 needs care. Two distinct sub-kinds exist:
- **Enum/status tokens** — stable, closed-set values such as `input_source`
  (`MAVLINK`/`WEB`/`FAILSAFE`), `can_status` (`connected`/`disconnected`), `gear`
  (`R`/`N`/`H`/`L`), `ignition_state` (`OFF`/`ACC`/`START`/`CRANKING`), `mav_active`
  (bool), `steer_sensor_ok` (bool), `steer_calibrated` (bool), `is_cranking` (bool),
  and `probe.status` (`no ECU` / running / complete). These map cleanly to localized
  labels client-side.
- **Free-form response messages** — the `message` field from `sendResponse(success, msg)`
  (e.g. `"Default for N set to 45.0%"`, `"Steering left limit saved: 1234 (…)"`,
  `"Jog right"`). Crucially, `index.html` currently **routes these by matching English
  substrings** (`data.message.includes('Default for')`, `.includes('throttle')`,
  `.includes('Steering center')`, `.startsWith('Jog')`). The firmware protocol must not
  change, so these arbitrary strings cannot be safely re-keyed into the dictionary without
  brittle full-string matching against firmware internals.

## Goals / Non-Goals
- Goals:
  - Full Ukrainian + English coverage of groups 1 and 2, and of group-3 **enum tokens**.
  - Instant switching, persistence, and sensible first-visit auto-detection.
  - Zero firmware changes; zero new network/file dependencies; theme untouched.
- Non-Goals:
  - Translating free-form firmware `message` strings (kept English — see Decisions).
  - Localizing MAVLink text, serial logs, or numeric formatting/decimal separators.
  - Adding any third language or a language-selection settings page.

## Decisions
- **Decision: Inline dictionary keyed by short identifiers.** Add a single
  `const I18N = { en: {…}, uk: {…} }` object and a `t(key, ...args)` helper inside the
  existing `<script>`. `t()` looks up the active locale, falls back to `en`, then to the
  key itself. Simple `%s`-style or template substitution supports dynamic values
  (e.g. "Switching to %s…", "Captured %s%% for %s").
  - Alternatives considered: external `i18n.json` fetched at load (rejected — violates the
    offline/no-fetch constraint); a full i18n library (rejected — no bundler, adds weight,
    overkill for two locales).
- **Decision: `data-i18n` attributes for static text.** Each static element carries
  `data-i18n="key"` (and `data-i18n-title` / `data-i18n-placeholder` where an attribute
  rather than text content must be translated). An `applyStaticTranslations()` pass walks
  these on load and on every language switch. This keeps the markup readable and avoids
  hand-wiring every element in JS.
- **Decision: Header toggle button, no reload.** A single button in `<header>` shows the
  language the user can switch **to** (or the current one — finalized during
  implementation), toggles `currentLang`, re-runs `applyStaticTranslations()`, updates
  dynamic elements, sets `document.documentElement.lang`, and writes `localStorage`.
- **Decision: Persistence + auto-detect.** On load: read `localStorage['lang']`; if absent,
  use `navigator.language`/`navigator.languages` — a `uk` prefix selects Ukrainian, else
  English. English is the ultimate fallback.
- **Decision: Firmware enum tokens mapped client-side; free-form messages stay English.**
  Provide small client-side maps (e.g. `input_source`, `ignition_state`, `can_status`,
  boolean OK/FAULT + CALIBRATED/NOT CALIBRATED + ON/OFF/LOCKED/UNLOCKED/Yes/No, and
  `probe.status`) → localized labels. The free-form `sendResponse` `message` strings
  continue to display verbatim (English), because the existing English-substring routing
  in `handleWebSocketMessage` depends on their exact wording and the C++ side is frozen for
  this change. This boundary is documented so a future firmware change (e.g. sending a
  stable `code` field alongside `message`) can localize them without breaking the protocol.
- **Decision: Unit labels selectively localized.** `km/h` → `км/год` in Ukrainian;
  `°C`, `kPa`, `µs`, `Hz`, `ms`, `%`, `V` remain unchanged in both locales. Numeric values
  and decimal separators are not reformatted.
- **Decision: Native-quality Ukrainian.** Translations use proper technical Ukrainian
  (e.g. gear/throttle/steering/calibration terminology). Because the operator is a native
  speaker, all Ukrainian strings are reviewed on-device before the change is considered
  done (captured as a task).

## Risks / Trade-offs
- **Risk: Missing/late-added strings show English.** → Mitigated by the `en` fallback in
  `t()` and by an audit task that greps the file for hard-coded user text.
- **Risk: Dictionary drift as UI evolves.** → Keep keys grouped by card/section and colocate
  the dictionary with the markup it serves; audit task covers coverage.
- **Risk: Free-form firmware messages remain English**, creating a mixed-language moment on
  some status lines. → Accepted for this scope; the enum tokens (the persistent labels)
  are fully localized, and the constraint is documented for a later protocol-level fix.
- **Risk: Byte-size growth of the single file on LittleFS.** → A second locale of short
  strings is small relative to the existing CSS/JS; no partition change expected. Verify the
  image still fits during `uploadfs`.

## Migration Plan
Pure additive frontend change. No data migration. Deployment = `pio run -t uploadfs` to
push the updated `data/index.html` to LittleFS. Rollback = re-upload the previous
`index.html`. Firmware image is unaffected.

## Open Questions
- Should the header button display the current language or the target language? (Cosmetic;
  decided during implementation, default: show the language you switch **to**.)
- Localize ECU probe PID display names (e.g. `Calc Load`, `Timing Adv`) or keep them as
  OBD-II shorthand? Default: translate the descriptive names, keep unit symbols as-is.
