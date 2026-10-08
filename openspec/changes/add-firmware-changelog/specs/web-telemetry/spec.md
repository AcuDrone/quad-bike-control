## MODIFIED Requirements
### Requirement: Firmware Version Display
The system SHALL display the firmware version and the target board name in the web portal interface.

#### Scenario: Include firmware version in telemetry broadcast
- **WHEN** telemetry data is collected for broadcast
- **THEN** the firmware version string is included from the `FIRMWARE_VERSION` constant defined in Constants.h
- **AND** the board name is included from the `BOARD_NAME` constant defined in Constants.h
- **AND** both are serialized to JSON as `"firmware_version": "<semver>"` and `"board": "<name>"`

#### Scenario: Display firmware version in web UI
- **WHEN** the web portal receives telemetry data via WebSocket
- **AND** the telemetry message contains a `firmware_version` field
- **THEN** the status bar shows `Firmware: <version> · <board>` using the existing `.status-item` pattern
- **AND** when `board` is absent only the version is shown
- **AND** the version is visible without scrolling (always in status bar)

#### Scenario: Handle missing firmware version gracefully
- **WHEN** the web portal connects but version data is not yet received
- **THEN** the firmware version display shows "Loading..." as placeholder text
- **WHEN** the firmware version field is missing from telemetry
- **THEN** the display retains its previous state
- **AND** no JavaScript errors are thrown

#### Scenario: Firmware version constant is centrally defined
- **WHEN** developers release new firmware
- **THEN** `FIRMWARE_VERSION` and `BOARD_NAME` are defined in `include/Constants.h`
- **AND** each hardware branch defines its own `BOARD_NAME` value

## ADDED Requirements
### Requirement: Firmware Changelog
The system SHALL ship a user-facing changelog inside the firmware binary and expose it to the web portal.

#### Scenario: Changelog is served by the firmware
- **WHEN** a client requests `GET /api/changelog`
- **THEN** the portal responds `200 application/json` with the document from `include/Changelog.h`
- **AND** the document is an object with a `releases` array ordered newest first
- **AND** each release has `version`, `date`, and `uk` / `en` arrays of plain-language strings

#### Scenario: Changelog is viewable from the web UI
- **WHEN** the user activates the firmware status item or the "What's new" button
- **THEN** a modal opens and fetches `/api/changelog` (at most once per page load)
- **AND** every release is listed with its version and date and the bullets of the active language
- **AND** the release matching the running `firmware_version` is marked as current
- **AND** switching the UI language re-renders the open modal in the other language

#### Scenario: Changelog fetch fails
- **WHEN** `/api/changelog` is unreachable or returns invalid JSON
- **THEN** the modal shows a localized error message instead of an empty list
- **AND** no JavaScript errors are thrown
