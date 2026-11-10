## ADDED Requirements
### Requirement: Event Journal Card
The System section SHALL contain an «Журнал подій» / "Event journal" card, placed after the Network
card, that shows the persistent event journal. It SHALL offer no vehicle command; its only actions
are refresh, download and clear.

#### Scenario: Show the last lines
- **WHEN** the System section is opened
- **THEN** the card SHALL fetch `GET /api/events?tail=100` and show the lines in a monospace,
  scrollable list with the newest line at the bottom
- **AND** it SHALL refresh every 5 s while the System section is visible and stop when it is hidden

#### Scenario: Source filter
- **WHEN** the engineer selects a source (All, SYS, SRC, STEER, VESC, CAN, I2C, RELAY, SNAP, RTC)
- **THEN** only lines of that source SHALL be shown, filtered in the browser

#### Scenario: Download
- **WHEN** the engineer clicks "Завантажити" / "Download"
- **THEN** the browser SHALL download `GET /api/events/download` as a text file

#### Scenario: Clear
- **WHEN** the engineer clicks "Очистити" / "Clear" and confirms the dialog
- **THEN** the card SHALL send `POST /api/events/clear` and reload the list

#### Scenario: Counters and availability
- **WHEN** telemetry arrives
- **THEN** the card header SHALL show `vesc_resync`, `can_reinit`, `i2c_unlock` and the journal size
- **AND** when `journal_bytes` is −1 the card SHALL show a localized note that the journal cannot be
  written to flash

#### Scenario: Help
- **WHEN** the ⓘ button of the card is clicked
- **THEN** the help modal SHALL explain what is journaled, the line format, and that `uploadfs` or a
  filesystem update erases the journal, in the active language
