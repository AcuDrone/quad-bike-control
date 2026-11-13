## MODIFIED Requirements
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
