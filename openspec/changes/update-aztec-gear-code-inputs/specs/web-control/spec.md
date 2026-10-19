## ADDED Requirements
### Requirement: Gear Code Calibration via Web Interface
The system SHALL let the operator assign the current live 3-bit gear code to a gear from the web
interface, so the code table can be matched to the actual gear selector on the bench.

The runtime table SHALL start from the `GEAR_CODE_*` defaults in `Constants.h` and SHALL be
persisted in the NVS namespace `"transmission"` under the keys `code_r`, `code_n`, `code_h` and
`code_l`. Commands SHALL go through the queued web-command path (`WebCommand` →
`VehicleController::processWebCommand()` → `TransmissionController`) and SHALL be refused while the
engine is running, like throttle calibration.

#### Scenario: Show the live bits
- **WHEN** telemetry arrives with `gear_code`
- **THEN** the web interface SHALL show bits A (port bit 0), B (port bit 1) and C (port bit 6) as
  individual indicators and the packed code as `0bABC` with its decoded gear
- **AND** the indicators SHALL show "no data" while `gear_code` is `-1`
- **AND** labels SHALL have i18n keys present in both the `en` and `uk` dictionaries

#### Scenario: Capture the current code for a gear
- **WHEN** the operator presses "Save as R" (or N, L, H), sending `{"cmd":"set_gear_code","value":"R"}`
- **AND** the engine is not running, no gear change is in progress and the opto-input snapshot is valid
- **THEN** the live code SHALL be stored as that gear's code in NVS
- **AND** it SHALL take effect immediately for `getPhysicalGear()` and `gear_sensed`
- **AND** the response SHALL be success with the message `Gear code for R saved: XYZ`
- **AND** a TRANSMISSION debug line SHALL be logged

#### Scenario: Reject the reserved codes 000 and 111
- **WHEN** a capture is requested while the live code is `000` (all lines open, mid-shift) or `111`
  (opto feed lost)
- **THEN** the capture SHALL be rejected with an error naming the code as reserved for UNKNOWN
- **AND** the table SHALL be unchanged

#### Scenario: Reject a stale or missing input snapshot
- **WHEN** a capture is requested while the opto-input snapshot is stale or the reader is not bound
- **THEN** the capture SHALL be rejected with an error that the opto inputs are unavailable

#### Scenario: Take over a code assigned to another gear
- **WHEN** a capture for a gear is requested and the live code is the active code of a different gear
- **THEN** the capture SHALL succeed and the code SHALL be assigned to the requested gear
- **AND** the other gear SHALL become UNSET (sentinel `0xFF` in the runtime table and in NVS)
- **AND** the response SHALL name the other gear, e.g. `Gear code for R saved: 011 (was N; N is now unset)`
- **AND** an UNSET gear SHALL never decode, so the physical gear SHALL read `GEAR_UNKNOWN` while the
  lever is in that gear until it is captured again
- **AND** the assigned codes SHALL stay distinct

#### Scenario: Reject while the engine runs or the servo moves
- **WHEN** a capture or reset is requested while the engine is running
- **THEN** it SHALL be rejected with an error
- **AND** a capture SHALL also be rejected while a gear change is in progress

#### Scenario: Reset to defaults
- **WHEN** the operator presses "Reset to defaults", sending `{"cmd":"reset_gear_codes"}`
- **THEN** all four entries SHALL be restored to the `GEAR_CODE_*` defaults immediately, including UNSET ones
- **AND** the `code_r`/`code_n`/`code_h`/`code_l` NVS keys SHALL be removed
- **AND** a TRANSMISSION debug line SHALL be logged

#### Scenario: Codes persist across reboot
- **WHEN** a code was captured and the controller reboots
- **THEN** `TransmissionController::begin()` SHALL load the stored codes from NVS
- **AND** a gear with no stored key SHALL keep its default code

#### Scenario: Invalid NVS content falls back to defaults
- **WHEN** the loaded table assigns `000`, `111` or a value above 7 (other than the UNSET sentinel
  `0xFF`), or assigns the same code to two gears
- **THEN** the whole table SHALL fall back to the `GEAR_CODE_*` defaults
- **AND** an error SHALL be logged on the TRANSMISSION debug feature

#### Scenario: Show the active table
- **WHEN** telemetry arrives with `gear_codes`
- **THEN** the web interface SHALL list R, N, H and L with their active codes as three binary digits
- **AND** each entry SHALL be marked as default or saved according to `gear_codes_custom`
- **AND** an UNSET entry (`-1`) SHALL be shown as "—" / "не задано" in a warning colour
- **AND** the help text SHALL tell the operator that after a take-over the other gear must be captured
