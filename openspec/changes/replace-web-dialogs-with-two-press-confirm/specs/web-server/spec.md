## MODIFIED Requirements
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
  indicator, gear-switching indicator, calibration/save status lines, two-press confirmation
  labels and warnings, firmware-upload status messages, debug/boost toggles, and the ECU probe
  results panel)
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
