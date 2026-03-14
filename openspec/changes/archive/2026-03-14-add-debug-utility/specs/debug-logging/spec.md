## ADDED Requirements

### Requirement: Runtime Debug Toggle
The system SHALL provide a Debug utility class that conditionally outputs serial messages based on a runtime-toggleable flag.

#### Scenario: Debug enabled - messages printed
- **WHEN** DEBUG_ENABLED is true
- **AND** Debug::print() or Debug::println() is called
- **THEN** the message SHALL be output to Serial

#### Scenario: Debug disabled - messages suppressed
- **WHEN** DEBUG_ENABLED is false
- **AND** Debug::print() or Debug::println() is called
- **THEN** no output SHALL be sent to Serial

#### Scenario: Backward compatibility with existing code
- **WHEN** existing code uses Serial.print/println directly
- **THEN** those calls SHALL continue to work without modification
- **AND** SHALL be refactored to use Debug utility for consistency

### Requirement: Debug Utility API
The Debug utility SHALL provide methods compatible with Arduino Serial API for drop-in replacement.

#### Scenario: Print without newline
- **WHEN** Debug::print(value) is called with any printable type
- **AND** DEBUG_ENABLED is true
- **THEN** the value SHALL be printed to Serial without trailing newline

#### Scenario: Print with newline
- **WHEN** Debug::println(value) is called with any printable type
- **AND** DEBUG_ENABLED is true
- **THEN** the value SHALL be printed to Serial with trailing newline

#### Scenario: Printf-style formatted output
- **WHEN** Debug::printf(format, args...) is called
- **AND** DEBUG_ENABLED is true
- **THEN** the formatted string SHALL be printed to Serial

#### Scenario: Support all Arduino printable types
- **WHEN** Debug methods are called with String, char*, int, float, bool, or other Serial-compatible types
- **THEN** the output SHALL match Serial.print behavior exactly

### Requirement: Web Portal Debug Control
The web portal SHALL provide an API endpoint to toggle DEBUG_ENABLED at runtime.

#### Scenario: Enable debug via web portal
- **WHEN** a POST request to /api/debug with {"enabled": true} is received
- **THEN** DEBUG_ENABLED SHALL be set to true
- **AND** a success response SHALL be returned
- **AND** Debug output SHALL begin appearing immediately

#### Scenario: Disable debug via web portal
- **WHEN** a POST request to /api/debug with {"enabled": false} is received
- **THEN** DEBUG_ENABLED SHALL be set to false
- **AND** a success response SHALL be returned
- **AND** Debug output SHALL stop immediately

#### Scenario: Query debug status
- **WHEN** a GET request to /api/debug is received
- **THEN** the current DEBUG_ENABLED state SHALL be returned as JSON
- **AND** response format SHALL be {"enabled": <boolean>}

### Requirement: Debug State Persistence
The debug enabled state SHALL persist across device reboots for consistent troubleshooting.

#### Scenario: Save debug state to NVS
- **WHEN** DEBUG_ENABLED is changed via web portal
- **THEN** the new state SHALL be saved to NVS (Non-Volatile Storage)
- **AND** the state SHALL survive power loss and reboots

#### Scenario: Restore debug state on startup
- **WHEN** the device boots up
- **THEN** DEBUG_ENABLED SHALL be initialized from NVS if available
- **AND** SHALL default to the Constants.h value if no saved state exists

### Requirement: Existing Code Migration
All existing Serial.print/println calls SHALL be systematically migrated to Debug utility.

#### Scenario: Replace Serial.print with Debug::print
- **WHEN** existing code contains Serial.print(msg)
- **THEN** it SHALL be refactored to Debug::print(msg)
- **AND** functionality SHALL remain identical

#### Scenario: Replace Serial.println with Debug::println
- **WHEN** existing code contains Serial.println(msg)
- **THEN** it SHALL be refactored to Debug::println(msg)
- **AND** functionality SHALL remain identical

#### Scenario: Replace Serial.printf with Debug::printf
- **WHEN** existing code contains Serial.printf(format, args...)
- **THEN** it SHALL be refactored to Debug::printf(format, args...)
- **AND** functionality SHALL remain identical

#### Scenario: Preserve critical Serial output
- **WHEN** Serial output is critical for bootloader or emergency diagnostics
- **THEN** those calls MAY remain as direct Serial calls
- **AND** SHALL be documented with a comment explaining why
