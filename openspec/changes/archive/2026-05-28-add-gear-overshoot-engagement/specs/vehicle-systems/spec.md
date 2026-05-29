---
capability: vehicle-systems
---

## MODIFIED Requirements

### Requirement: Transmission Control System

The transmission actuator SHALL use an overshoot-and-return motion profile when
changing gears to ensure full mechanical engagement of the detent.

#### Scenario: Overshoot then return on gear change

- **Given** a gear change is requested to a target gear whose stored position differs
  from the current position by more than `TRANS_POSITION_TOLERANCE`
- **When** `setGear()` is called
- **Then** the actuator first moves to `targetPosition + sign(targetPosition − currentPosition) × TRANS_GEAR_OVERSHOOT` (Phase 1)
- **And** once Phase 1 position is reached, the actuator moves back to `targetPosition` (Phase 2)
- **And** physical-switch confirmation and `saveState()` are unchanged and occur only after Phase 2

#### Scenario: Skip overshoot when already near target

- **Given** the current encoder position is within `TRANS_POSITION_TOLERANCE` of the
  target gear position
- **When** `setGear()` is called
- **Then** no overshoot phase is started; the actuator moves directly to target (or
  stops immediately if already there)
