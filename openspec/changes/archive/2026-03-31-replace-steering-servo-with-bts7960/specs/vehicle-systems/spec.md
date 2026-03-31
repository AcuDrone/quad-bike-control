## MODIFIED Requirements
### Requirement: Steering Control System
The system SHALL provide steering control through a BTS7960 H-bridge motor driver with hall sensor encoder feedback and software position limits.

#### Scenario: Set steering position by percentage
- **WHEN** SteeringController.setSteeringPercent() is called with value from -100 to +100
- **THEN** the steering actuator moves at full speed toward the corresponding encoder position
- **AND** -100% maps to the calibrated left limit
- **AND** 0% maps to the calibrated center position
- **AND** +100% maps to the calibrated right limit
- **AND** movement stops when the target position is reached (within tolerance)

#### Scenario: Center steering on startup
- **WHEN** the system initializes
- **THEN** the steering actuator SHALL auto-home to a physical limit (stall detection)
- **AND** the encoder SHALL be reset to 0 at the home position
- **AND** the actuator SHALL move to the calibrated center position
- **AND** if no calibration exists, the actuator SHALL remain at the home position

#### Scenario: Enforce software steering limits
- **WHEN** a steering command would move beyond the calibrated left or right limit
- **THEN** the target position SHALL be clamped to the limit
- **AND** the actuator SHALL NOT move past the software limit

#### Scenario: Calibrate steering positions
- **WHEN** steering calibration is triggered via web portal
- **THEN** the system SHALL allow setting center position and right limit at current encoder position
- **AND** left limit SHALL be 0 (home position)
- **AND** calibrated positions SHALL be saved to NVS
- **AND** calibration SHALL persist across reboots

#### Scenario: Steering failsafe
- **WHEN** signal loss is detected or failsafe is activated
- **THEN** the steering actuator SHALL move to the calibrated center position
- **AND** if not calibrated, the actuator SHALL stop immediately

## REMOVED Requirements
### ~~Scenario: Smooth steering transitions~~
- Removed: rate-limiting is not applicable to full-speed bang-bang control. Natural mechanical resistance provides deceleration.
