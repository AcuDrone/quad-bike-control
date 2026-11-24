## ADDED Requirements
### Requirement: Event Journal Card
The System section SHALL contain an «Журнал подій» / "Event journal" card, placed after the Network
card, that shows the persistent event journal. It SHALL offer no vehicle command; its only actions
are refresh, download, clear and the journal's flash-write switch.

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
- **WHEN** the engineer clicks "Очистити" / "Clear" and confirms it with a second press of the same
  button within 5 s (two-press confirmation, change `replace-web-dialogs-with-two-press-confirm`)
- **THEN** the card SHALL send `POST /api/events/clear` and reload the list

#### Scenario: Counters and availability
- **WHEN** telemetry arrives
- **THEN** the card header SHALL show `vesc_resync`, `can_reinit`, `i2c_unlock` and the journal size
- **AND** when `journal_bytes` is −1 the card SHALL show a localized note that the journal cannot be
  written to flash

#### Scenario: Flash-write toggle
- **WHEN** the card is shown
- **THEN** it SHALL show a «Запис у flash» / "Write to flash" toggle reflecting telemetry
  `journal_fs`, with a localized hint that while it is off the journal lives only in RAM (and the
  RTC ring) and is lost at power-off
- **AND** a status row SHALL show «лише RAM» / "RAM only" or "flash" and the number of lines held in
  RAM (header `X-Journal-Ram-Lines` of the last tail request)
- **WHEN** the operator clicks the toggle
- **THEN** the card SHALL send `POST /api/events/config` with `{"fs":<new state>}`, show the state
  from the reply, and reload the list so that the `journal_fs on=…` line appears; a failure SHALL
  show a localized error

#### Scenario: Help
- **WHEN** the ⓘ button of the card is clicked
- **THEN** the help modal SHALL explain what is journaled, the flash-write switch (off by default,
  RAM only while off), the line format, and that `uploadfs` or a filesystem update erases the
  journal, in the active language
