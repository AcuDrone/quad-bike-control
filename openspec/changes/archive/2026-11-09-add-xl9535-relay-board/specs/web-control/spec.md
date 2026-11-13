## ADDED Requirements
### Requirement: Relay Board Selection via Web Interface
The web portal SHALL let the operator choose the relay board setting `auto`, `pca9557` or `xl9535`
with the WebSocket command `{"cmd":"set_relay_board","value":"<auto|pca9557|xl9535>"}`, handled in
`VehicleController::processWebCommand()` through the queued web-command path. The command SHALL be
accepted regardless of the active input source and the ignition state, because it only changes the
stored value; the running relay board SHALL NOT change until reboot.

#### Scenario: Valid selection is persisted
- **WHEN** a `set_relay_board` command carries `auto`, `pca9557` or `xl9535`
- **THEN** the value SHALL be written to NVS namespace `relay`, key `board`, before anything else
- **AND** the response SHALL be success with a text that says the change takes effect after reboot
  (e.g. "Relay board set to xl9535 — takes effect after reboot")
- **AND** the active relay board, its address and the relay states SHALL NOT change

#### Scenario: Invalid selection is rejected
- **WHEN** a `set_relay_board` command carries a missing or unknown value
- **THEN** an error response "Relay board must be auto, pca9557 or xl9535" SHALL be sent
- **AND** nothing SHALL be written to NVS

#### Scenario: NVS write fails
- **WHEN** the NVS write fails
- **THEN** an error response "Failed to save the relay board setting" SHALL be sent

#### Scenario: Relay board card in the UI
- **WHEN** the System section is shown
- **THEN** a "Relay board" / "Плата реле" card SHALL show a three-way selector (Auto / PCA9557 /
  XL9535) reflecting `relay_board_cfg`, the active board line (e.g. "XL9535 @0x20 (auto)"), and a
  note that a change takes effect after reboot
- **AND** while `relay_board_reboot` is `true` the card SHALL show "Reboot to apply" / "Перезавантажте
  для застосування"
- **AND** while `relay_board_error` is not `null` the card SHALL show the localized reason (both
  boards found, no board found, several XL9535 addresses, setting does not match the bus,
  initialization failed)
- **AND** the card SHALL have an ⓘ help text (en/uk) explaining auto detection and the reboot rule
- **AND** the Relay Test card SHALL show the active board in its subtitle
