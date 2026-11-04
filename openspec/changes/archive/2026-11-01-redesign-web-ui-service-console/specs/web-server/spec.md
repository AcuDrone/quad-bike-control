## MODIFIED Requirements
### Requirement: Language Toggle Control
The interface SHALL present a language toggle control in the content header, next to the current
section title, that switches the entire UI between English and Ukrainian instantly, without
reloading the page. The toggle SHALL be reachable in every section and at every screen width.
When the language changes, the `<html lang>` attribute SHALL be updated to the active language code.

#### Scenario: Toggle switches language without reload
- **WHEN** the operator activates the language toggle
- **THEN** all currently displayed static and dynamic text updates to the other language
- **AND** the page is not reloaded and current telemetry/control state is preserved
- **AND** the current section stays selected, and the section title, the navigation labels and an
  open help or changelog modal are re-rendered in the new language

#### Scenario: Document language attribute reflects selection
- **WHEN** the active language is English
- **THEN** the `<html lang>` attribute is `en`
- **AND** when the active language is Ukrainian, the `<html lang>` attribute is `uk`

#### Scenario: Toggle available on narrow screens
- **WHEN** the viewport is 768 px wide or less
- **THEN** the language toggle SHALL remain visible in the content header without opening the ☰ menu
