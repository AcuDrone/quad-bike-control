## ADDED Requirements
### Requirement: Relay Board Telemetry
Web telemetry SHALL report the relay board selection and detection result on every broadcast:
- `relay_board`: the active board, `"pca9557"`, `"xl9535"` or `"none"` (detection failed);
- `relay_board_addr`: the active board's 7-bit address as an integer, or `null` when `"none"`;
- `relay_board_cfg`: the stored setting, `"auto"`, `"pca9557"` or `"xl9535"`;
- `relay_board_reboot`: `true` while the stored setting differs from the setting used at boot;
- `relay_board_error`: `null`, or one of `both_found`, `none_found`, `multiple_xl9535`,
  `cfg_mismatch`, `init_failed`.
`io_relay_fault` SHALL keep its meaning and SHALL be `true` whenever `relay_board_error` is not
`null`.

#### Scenario: XL9535 detected automatically
- **WHEN** the setting is `auto` and the XL9535 board at `0x20` was detected at boot
- **THEN** telemetry SHALL report `relay_board` = `"xl9535"`, `relay_board_addr` = 32,
  `relay_board_cfg` = `"auto"`, `relay_board_reboot` = `false`, `relay_board_error` = `null`

#### Scenario: Ambiguous bus
- **WHEN** both relay boards answered at boot with the setting `auto`
- **THEN** telemetry SHALL report `relay_board` = `"none"`, `relay_board_addr` = `null`,
  `relay_board_error` = `"both_found"` and `io_relay_fault` = `true`

#### Scenario: Setting changed, not yet rebooted
- **WHEN** the operator stored `pca9557` while the XL9535 board is active from an `auto` boot
- **THEN** `relay_board_cfg` SHALL be `"pca9557"`, `relay_board_reboot` SHALL be `true`, and
  `relay_board` SHALL still be `"xl9535"`

## MODIFIED Requirements
### Requirement: Relay Test Telemetry
Web telemetry SHALL include `relay_test`, an 8-bit unsigned integer with one bit per relay in
relay-number order (bit0 = Relay_1 … bit7 = Relay_8), set for each relay currently energized by the
web relay test. It SHALL be derived from the relay output shadow byte (the last written and verified
port value) through the active relay board's Relay_N → bit table (PCA9557 board: `Relay.PrjPcb`
routing; XL9535 board: identity), limited to the test bits, so a vehicle function relay is never
reported as a test relay and the bitfield means the same relays on either board. The existing
function fields (`ignition_state`, `is_cranking`, `front_light_on`, `wheel_lock_on`,
`wheel_lock_fitted`, `io_relay_fault`) SHALL be unchanged.

#### Scenario: Test relay reported
- **WHEN** Relay_5 is test-energized and verified (IO3 on the PCA9557 board, P0.4 on the XL9535 board)
- **THEN** telemetry SHALL report `relay_test` = 16 (bit4)

#### Scenario: No test active
- **WHEN** no relay is test-energized, the relay expander is unavailable, or no relay board was
  detected
- **THEN** telemetry SHALL report `relay_test` = 0

#### Scenario: Auto-off or forced-off is visible
- **WHEN** a test relay times out or is forced off (function command, fail-safe, fault)
- **THEN** the next telemetry broadcast SHALL report its bit as 0
