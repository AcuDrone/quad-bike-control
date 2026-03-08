# sbus-input Capability Delta

## ADDED Requirements

### Requirement: Input Source Priority Integration
The system SHALL support prioritized input source selection between S-bus and alternative control sources.

#### Scenario: Report S-bus priority status
- **WHEN** getInputPriority() is called on SBusInput
- **THEN** current priority level is returned (PRIMARY, SECONDARY, INACTIVE)
- **AND** S-bus has PRIMARY priority when signal is valid
- **AND** S-bus has INACTIVE priority when signal lost or timed out

#### Scenario: Query if S-bus should control vehicle
- **WHEN** shouldControlVehicle() is called
- **THEN** true is returned if S-bus signal is valid (within 500ms timeout)
- **AND** false is returned if S-bus signal is lost or timed out

#### Scenario: Integrate with input source arbiter
- **WHEN** external input source arbiter queries S-bus status
- **THEN** S-bus provides signal validity status
- **AND** S-bus provides time since last valid frame
- **AND** arbiter can determine if S-bus should have control priority
