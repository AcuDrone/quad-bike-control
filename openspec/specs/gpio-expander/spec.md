# gpio-expander Specification

## Purpose
TBD - created by archiving change add-mcp23017-gpio-expander. Update Purpose after archive.
## Requirements
### Requirement: MCP23017 I2C GPIO Expander
The system SHALL interface with an MCP23017 I2C GPIO expander to provide additional digital I/O pins for relay outputs, gear selector inputs, and brake sensor input.

#### Scenario: Initialize MCP23017 on startup
- **WHEN** `MCP23017Controller::begin(sdaPin, sclPin, address)` is called
- **THEN** I2C bus SHALL be initialized at 400 kHz
- **AND** MCP23017 SHALL be detected at the specified address (default 0x20)
- **AND** Port A (GPA0–GPA2) SHALL be configured as outputs for relays
- **AND** Port B (GPB0–GPB4) SHALL be configured as inputs with pull-ups for sensors
- **AND** all outputs SHALL be set to LOW (safe state)
- **AND** return `true` if successful, `false` if MCP23017 not detected

#### Scenario: Write relay output via MCP23017
- **WHEN** `mcp.digitalWrite(pin, value)` is called for a Port A pin
- **THEN** the corresponding MCP23017 output pin SHALL be set HIGH or LOW
- **AND** the I2C transaction SHALL complete within 200μs

#### Scenario: Read sensor input via MCP23017
- **WHEN** `mcp.digitalRead(pin)` is called for a Port B pin
- **THEN** the current logic level of the MCP23017 input pin SHALL be returned
- **AND** internal pull-up resistors SHALL be enabled for input pins

#### Scenario: Bulk read sensor port
- **WHEN** `mcp.readPort(1)` is called for Port B
- **THEN** all 8 Port B pin states SHALL be returned in a single I2C transaction
- **AND** individual sensor states SHALL be extracted using bit masks

### Requirement: MCP23017 Error Handling
The system SHALL detect and handle MCP23017 communication failures gracefully.

#### Scenario: Handle I2C communication failure
- **WHEN** an I2C transaction to the MCP23017 fails
- **THEN** the error SHALL be logged via Debug feature
- **AND** relay outputs SHALL default to LOW (all off, safe state)
- **AND** sensor reads SHALL return safe default values
- **AND** `isInitialized()` SHALL return false

#### Scenario: Handle MCP23017 not detected on startup
- **WHEN** `begin()` is called but MCP23017 is not found on I2C bus
- **THEN** `begin()` SHALL return false
- **AND** an error SHALL be logged
- **AND** the system SHALL continue operating with degraded functionality

### Requirement: MCP23017 Pin Allocation
The system SHALL define a standard pin allocation for the MCP23017 expander.

#### Scenario: Define relay output pins on Port A
- **WHEN** MCP23017 is initialized
- **THEN** the following pin mapping SHALL be used:
  - GPA0 (pin 0): Relay 1 — main power control
  - GPA1 (pin 1): Relay 2 — accessory power
  - GPA2 (pin 2): Relay 3 — front light / safety system
  - GPA3–GPA7 (pins 3–7): Available for future outputs

#### Scenario: Define sensor input pins on Port B
- **WHEN** MCP23017 is initialized
- **THEN** the following pin mapping SHALL be used:
  - GPB0 (pin 8): Gear selector REVERSE — active-low with pull-up
  - GPB1 (pin 9): Gear selector NEUTRAL — active-low with pull-up
  - GPB2 (pin 10): Gear selector LOW — active-low with pull-up
  - GPB3 (pin 11): Gear selector HIGH — active-low with pull-up
  - GPB4 (pin 12): Brake sensor — active-low with pull-up
  - GPB5–GPB7 (pins 13–15): Available for future inputs

