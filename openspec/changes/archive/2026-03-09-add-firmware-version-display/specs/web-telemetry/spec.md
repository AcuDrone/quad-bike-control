# web-telemetry Specification Delta

## ADDED Requirements

### Requirement: Firmware Version Display
The system SHALL display the firmware version in the web portal interface.

#### Scenario: Include firmware version in telemetry broadcast
- **WHEN** telemetry data is collected for broadcast
- **THEN** the firmware version string is included from the `FIRMWARE_VERSION` constant defined in Constants.h
- **AND** the version is added to the telemetry struct as `firmware_version` field
- **AND** the version is serialized to JSON as `"firmware_version": "<version>"`

#### Scenario: Display firmware version in web UI
- **WHEN** the web portal receives telemetry data via WebSocket
- **AND** the telemetry message contains a `firmware_version` field
- **THEN** the firmware version is displayed in the status bar
- **AND** the version display uses the existing `.status-item` CSS pattern
- **AND** the version display has the format: "Firmware: X.X.X"
- **AND** the version is visible without scrolling (always in status bar)

#### Scenario: Handle missing firmware version gracefully
- **WHEN** the web portal connects but version data is not yet received
- **THEN** the firmware version display shows "Loading..." as placeholder text
- **WHEN** the firmware version field is missing from telemetry
- **THEN** the display shows "N/A" or retains "Loading..." state
- **AND** no JavaScript errors are thrown

#### Scenario: Firmware version constant is centrally defined
- **WHEN** developers need to update the firmware version
- **THEN** the version is defined as `FIRMWARE_VERSION` constant in `include/Constants.h`
- **AND** the constant uses semantic versioning format (e.g., "1.0.0")
- **AND** updating the constant automatically propagates to web portal display

## MODIFIED Requirements

None - this is a net-new feature addition.

## REMOVED Requirements

None - no existing functionality is replaced.
