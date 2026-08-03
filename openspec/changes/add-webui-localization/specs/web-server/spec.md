## ADDED Requirements

### Requirement: Web UI Localization
The served control interface (`data/index.html`) SHALL provide its user-facing text in two
languages — English (`en`) and Ukrainian (`uk`) — using a translation dictionary and lookup
helper embedded inline in the single served page. The localization SHALL NOT introduce any
external network resource, additional file, or runtime fetch (no CDN, no remote i18n file),
preserving the offline operation of the portal. English SHALL be the fallback for any
translation key that is missing in the active language.

#### Scenario: Static text is translated
- **WHEN** the interface is displayed in a selected language
- **THEN** all static markup text (page title/header, status-bar labels, card headings,
  telemetry labels, button captions, help paragraphs, and warnings) renders in that language
- **AND** the text is sourced from an inline dictionary via `data-i18n` attributes

#### Scenario: Dynamic JavaScript strings are translated
- **WHEN** the interface generates text at runtime (connection state, MAVLink active/inactive
  indicator, gear-switching indicator, calibration/save status lines, `alert()`/`confirm()`
  dialogs, debug/boost toggles, and the ECU probe results panel)
- **THEN** those strings render in the active language via a `t(key)` lookup helper
- **AND** values embedded in a string (percentages, gear letters, counts) are substituted
  into the localized template

#### Scenario: Missing key falls back to English
- **WHEN** a translation key is absent from the active language's dictionary
- **THEN** the English value for that key is displayed
- **AND** no untranslated placeholder or blank text is shown

#### Scenario: No external dependency added by localization
- **WHEN** the localized page is served from LittleFS while the device has no internet access
- **THEN** all languages and the lookup logic load entirely from the served page
- **AND** no request is made to any CDN, remote font, or external i18n resource

### Requirement: Language Toggle Control
The interface SHALL present a language toggle control in the page header that switches the
entire UI between English and Ukrainian instantly, without reloading the page. When the
language changes, the `<html lang>` attribute SHALL be updated to the active language code.

#### Scenario: Toggle switches language without reload
- **WHEN** the operator activates the language toggle
- **THEN** all currently displayed static and dynamic text updates to the other language
- **AND** the page is not reloaded and current telemetry/control state is preserved

#### Scenario: Document language attribute reflects selection
- **WHEN** the active language is English
- **THEN** the `<html lang>` attribute is `en`
- **AND** when the active language is Ukrainian, the `<html lang>` attribute is `uk`

### Requirement: Language Persistence and Auto-Detection
The interface SHALL persist the selected language in `localStorage` and reapply it on
subsequent loads. On the first visit (no stored preference), the interface SHALL auto-detect
the language from the browser's `navigator.language` / `navigator.languages`, selecting
Ukrainian when a `uk` language prefix is present and English otherwise.

#### Scenario: Selection persists across reloads
- **WHEN** the operator selects a language and later reloads the page
- **THEN** the previously selected language is applied automatically from `localStorage`

#### Scenario: First-visit auto-detection
- **WHEN** the page loads with no stored language preference
- **AND** the browser's preferred language begins with `uk`
- **THEN** the interface displays in Ukrainian
- **AND** for any non-`uk` preferred language, the interface displays in English

### Requirement: Localized Display of Firmware Status Tokens
The interface SHALL map stable enum/status tokens received in WebSocket telemetry to
localized display labels on the client side, without changing the firmware or the
WebSocket/HTTP protocol. Free-form firmware response `message` strings, which the client
routes by matching their English wording, SHALL continue to be displayed as received
(English) and SHALL NOT be re-keyed, so the protocol contract remains unchanged.

#### Scenario: Enum tokens are shown in the active language
- **WHEN** telemetry reports a stable status token (input source `MAVLINK`/`WEB`/`FAILSAFE`,
  CAN status connected/disconnected, MAVLink active/inactive, steering sensor OK/fault,
  steering calibrated/not-calibrated, ignition state, cranking yes/no, or ECU probe status)
- **THEN** the corresponding label is displayed translated into the active language
- **AND** the underlying telemetry JSON field values are not altered

#### Scenario: Protocol is unchanged and free-form messages stay English
- **WHEN** the firmware sends a free-form `sendResponse` `message` string
- **THEN** the client displays that message as received without re-keying it
- **AND** no change is made to the firmware, WebSocket messages, or HTTP endpoints to
  support localization

### Requirement: Localized Unit Labels
The interface SHALL translate unit labels for which a Ukrainian convention exists while
leaving language-neutral units and numeric formatting unchanged.

#### Scenario: Locale-specific unit label
- **WHEN** the interface displays vehicle speed in Ukrainian
- **THEN** the unit label is `км/год`
- **AND** in English the unit label is `km/h`

#### Scenario: Language-neutral units are preserved
- **WHEN** the interface displays temperature, pressure, servo pulse width, frequency, or
  percentage values
- **THEN** the unit symbols `°C`, `kPa`, `µs`, `Hz`, and `%` are shown unchanged in both
  languages
- **AND** numeric values and decimal separators are not reformatted
