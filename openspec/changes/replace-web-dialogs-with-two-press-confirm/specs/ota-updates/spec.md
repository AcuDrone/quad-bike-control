## MODIFIED Requirements
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
