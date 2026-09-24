## ADDED Requirements

### Requirement: 24V Rail Telemetry
The system SHALL include the 24V boost rail state in the telemetry broadcast so the rail powering
the steering VESC, Jetson, Starlink and cameras is observable from the web interface.

#### Scenario: Collect rail data
- **WHEN** telemetry collection is triggered
- **THEN** the latest averaged 24V rail voltage SHALL be collected
- **AND** the commanded boost enable state SHALL be collected
- **AND** the low-rail warning flag SHALL be collected

#### Scenario: Format rail data as JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** `"rail_24v"` SHALL be included as a float with 1 decimal place
- **AND** `"boost_on"` SHALL be included as a boolean
- **AND** `"rail_low"` SHALL be included as a boolean
- **AND** these fields SHALL be emitted unconditionally, independent of CAN or MAVLink status

#### Scenario: Display rail voltage on the web interface
- **WHEN** a telemetry message with `rail_24v` is received
- **THEN** the rail voltage SHALL be displayed in volts with 1 decimal place
- **AND** the reading SHALL be colour-coded: normal when at or above the low threshold, warning when
  `rail_low` is true
- **AND** the boost enable state SHALL be shown alongside it, so a deliberately disabled boost is
  distinguishable from a failed rail
- **AND** labels SHALL have i18n keys present in both the `en` and `uk` dictionaries

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
