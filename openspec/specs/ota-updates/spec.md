# ota-updates Specification

## Purpose
TBD - created by archiving change add-web-portal. Update Purpose after archive.
## Requirements
### Requirement: Over-The-Air Firmware Update
The system SHALL support OTA (Over-The-Air) firmware updates via the web interface.

#### Scenario: Initialize OTA service
- **WHEN** WebPortal.begin() is called
- **THEN** ArduinoOTA service is initialized
- **AND** OTA hostname is set to `<OTA_HOSTNAME>-xxxx`, the compiled base hostname followed by `-`
  and the same chip suffix as the AP SSID in lowercase (e.g. "quadbike-control-5ecc")
- **AND** OTA is ready to accept firmware updates

#### Scenario: Begin firmware upload
- **WHEN** firmware binary is uploaded via web interface
- **THEN** vehicle is placed in safe state (brakes on, neutral gear, servos disabled)
- **AND** OTA update process begins
- **AND** current firmware partition is backed up (ESP32 dual partition)
- **AND** upload progress is tracked

#### Scenario: Report upload progress
- **WHEN** firmware upload is in progress
- **THEN** progress percentage is calculated (bytes received / total bytes)
- **AND** progress is broadcast to web clients via WebSocket
- **AND** progress indicator is displayed on web interface

#### Scenario: Complete successful firmware update
- **WHEN** firmware upload completes successfully
- **THEN** firmware integrity is verified (checksum)
- **AND** new firmware is written to update partition
- **AND** success message is sent to web client
- **AND** system automatically reboots to new firmware within 5 seconds
- **AND** new firmware boots from update partition

#### Scenario: Handle firmware update failure
- **WHEN** firmware upload or verification fails
- **THEN** error message is sent to web client with failure reason
- **AND** OTA process is aborted
- **AND** system remains on current firmware
- **AND** vehicle exits safe state and returns to normal operation
- **AND** error is logged with details

#### Scenario: Rollback on boot failure
- **WHEN** new firmware fails to boot successfully
- **THEN** ESP32 bootloader automatically rolls back to previous firmware
- **AND** system boots from previous partition
- **AND** OTA rollback is logged

### Requirement: OTA Security and Safety
The system SHALL enforce safety measures during OTA updates.

#### Scenario: Enforce safe state during OTA
- **WHEN** OTA update begins
- **THEN** brakes are applied to 100%
- **AND** transmission is set to NEUTRAL
- **AND** throttle is set to idle
- **AND** steering is centered
- **AND** vehicle cannot be controlled during update

#### Scenario: Prevent control input during OTA
- **WHEN** OTA update is in progress
- **THEN** S-bus commands are ignored
- **AND** web control commands are rejected
- **AND** telemetry broadcast is paused
- **AND** only OTA progress updates are sent

#### Scenario: Validate firmware binary
- **WHEN** firmware binary is uploaded
- **THEN** file size is validated (not exceeding partition size)
- **AND** file format is validated (ESP32 binary header)
- **AND** upload is rejected if validation fails
- **AND** error message describes validation failure

### Requirement: OTA Web Interface
The system SHALL provide web interface controls for firmware updates.

#### Scenario: Display OTA upload interface
- **WHEN** web page loads
- **THEN** firmware upload section is displayed
- **AND** file input for selecting firmware binary is available
- **AND** upload button is enabled
- **AND** progress bar is shown (initially at 0%)
- **AND** a status line is shown under the upload controls

#### Scenario: Trigger firmware upload from web UI
- **WHEN** user selects firmware file and clicks upload button
- **THEN** file is validated for correct type (.bin extension)
- **AND** the first press arms the button and the status line warns that the vehicle will enter safe
  state and reboot (two-press confirmation, see web-control "Two-Press Confirmation for Destructive
  Web Actions")
- **AND** file upload begins only on a second press of the same button within 5 s
- **AND** progress bar updates in real-time

#### Scenario: No file selected
- **WHEN** user clicks the upload button with no file selected
- **THEN** the status line shows "Please select a firmware file first" / «Спершу оберіть файл
  прошивки» in the danger colour and nothing is armed or sent

#### Scenario: Display OTA result on web UI
- **WHEN** the upload completes successfully
- **THEN** the status line shows the success message in the success colour, without a browser dialog

#### Scenario: Display OTA error on web UI
- **WHEN** OTA update fails
- **THEN** error message is displayed prominently on web page, in the status line in the danger
  colour, without a browser dialog
- **AND** error details are provided (e.g., "Upload failed: Invalid firmware")
- **AND** upload controls are re-enabled for retry

