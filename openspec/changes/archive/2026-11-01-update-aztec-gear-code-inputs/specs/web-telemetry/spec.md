## MODIFIED Requirements
### Requirement: Board I/O Health Telemetry
The system SHALL report the health of the two I2C port expanders in telemetry, because the gear
interlock, the brake endstop and the ignition/starter path now depend on them.

#### Scenario: Collect expander health
- **WHEN** telemetry collection is triggered
- **THEN** the opto-input expander fault state SHALL be collected from `BoardInputs::isFaulted()`
- **AND** the relay expander fault state SHALL be collected from `RelayController::isFaulted()`

#### Scenario: Format expander health as JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** `"io_input_fault"` and `"io_relay_fault"` SHALL be included as booleans
- **AND** they SHALL be emitted unconditionally, independent of CAN or MAVLink status

#### Scenario: Display expander faults on the web interface
- **WHEN** a telemetry message reports `io_input_fault` or `io_relay_fault` as true
- **THEN** a visible fault indicator SHALL be shown identifying which expander is faulted
- **AND** the indicator SHALL clear automatically when the flag returns to false
- **AND** labels SHALL have i18n keys present in both the `en` and `uk` dictionaries

#### Scenario: Distinguish a fault from a stale telemetry stream
- **WHEN** the input expander is faulted but the control loop is healthy
- **THEN** telemetry SHALL continue to broadcast at the normal 5 Hz rate with the fault flag set
- **AND** gear SHALL be reported as unknown rather than as a last-known gear presented as current

#### Scenario: Report the raw gear code for wiring checks
- **WHEN** telemetry data is formatted as JSON
- **THEN** `"gear_code"` SHALL be included as an integer: the 3-bit gear code `[bit0 bit1 bit6]`
  (0..7) from the current opto-input snapshot, or `-1` while the snapshot is stale
- **AND** `"gear_sensed"` SHALL be included as `"R"`, `"N"`, `"H"`, `"L"` or `"?"`: that code decoded
  by `TransmissionController::decodeGearCode()` without the retry cache
- **AND** the web interface SHALL show the code as three binary digits with its decoded gear under
  the current gear, with i18n keys present in both the `en` and `uk` dictionaries

#### Scenario: Report the active gear code table
- **WHEN** telemetry data is formatted as JSON
- **THEN** `"gear_codes"` SHALL be included as an object `{"R":n,"N":n,"H":n,"L":n}` holding the
  active runtime gear code (0..7) of each gear, or `-1` for a gear that is UNSET
- **AND** `"gear_codes_custom"` SHALL be included as an integer bitmask of the entries that differ from
  their `Constants.h` default (bit0 R, bit1 N, bit2 H, bit3 L)
- **AND** `"gear_code"` SHALL be read from the current snapshot on every telemetry cycle, bypassing the
  `TRANS_GEAR_READ_INTERVAL_MS` gear cache, so the calibration view is live
