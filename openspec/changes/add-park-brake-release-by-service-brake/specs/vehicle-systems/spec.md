## ADDED Requirements
### Requirement: Parking Brake Release by Service-Brake Press
On the aztec buggy, a new operator service-brake press SHALL start a parking-brake RELEASE run while
the park brake is `APPLIED` or `UNKNOWN`. For this one trigger, this requirement takes precedence
over the rule that a service brake command does not change the park-brake state.

By owner decision of 2026-11-17 this trigger has **no protection** against an automatic brake: a
brake commanded on servo channel 2 by any sender (RC handset, GCS joystick, the BlueOS Copilot's
automatic fail-safe brake, an autopilot mode) and a web brake command all count.

**Input.** The trigger SHALL use the commanded service-brake target as `applyBrake()` sets it (below
5 % it is 0, above 100 % it is 100 %). A **press** is that target changing from 0 to a value above 0
while the input source is `MAVLINK` or `WEB`.

**Rising edge.** The firmware SHALL sample the target on every control loop in every park-brake
state, so that:
- a target that is already non-zero when the park brake becomes `APPLIED` is not a press;
- a change from one non-zero value to another is not a press;
- under the `FAILSAFE` input source no press SHALL be detected, and the target frozen there SHALL NOT
  count as a press when a live source returns;
- on any input-source change the sample SHALL be re-seeded with the current target, so a takeover
  (MAVLINK ↔ WEB, or from or to `FAILSAFE`) never produces a press by itself.

**Hand-over.** After a press the release SHALL start on the first loop in which the service-brake
position estimate is at or above the current target minus `BRAKE_TOLERANCE`. The wait SHALL be
cancelled, with no release, when the target returns to 0, the input source changes, or the park-brake
state is no longer `APPLIED` or `UNKNOWN`. A further release then needs a new press.

**Release semantics.**
- The release SHALL be the same RELEASE request as the drive-away release.
- Every existing RELEASING rule SHALL apply unchanged: the throttle hold in L/H/R up to the throttle
  release point, completion of the run if the service brake is let go, and no automatic re-apply.
- A service-brake press SHALL NOT apply the parking brake, and SHALL have no park-brake effect while
  the state is `RELEASED`, `RELEASING` or `APPLYING`.
- The trigger SHALL NOT depend on the gear, the ignition state, the flight mode or any autopilot
  status.
- The firmware SHALL NOT move or hold the service brake for this feature.

The drive-away release, the explicit web and GCS commands, the apply speed interlock and the speed
governor SHALL behave exactly as before.

#### Scenario: RC press releases after the hand-over
- **WHEN** the park brake is `APPLIED`, the source is `MAVLINK`, the service brake is released
  (target 0, estimate 0 %) and the brake travel time is 1500 ms
- **AND** the operator presses the service brake to 50 % and holds it
- **THEN** the release run SHALL start when the estimate reaches 49 %, about 735 ms after the press
- **AND** the state SHALL become `RELEASING`

#### Scenario: Release from UNKNOWN
- **WHEN** the park-brake state is `UNKNOWN` and a press completes its hand-over
- **THEN** a release run SHALL start and settle the state as `RELEASED`

#### Scenario: Web press releases
- **WHEN** the park brake is `APPLIED`, the source is `WEB`, the target is 0 and the operator sends a
  web brake of 80 %
- **THEN** a release run SHALL start once the estimate is at or above 79 %

#### Scenario: Automatic fail-safe brake releases (accepted)
- **WHEN** the park brake is `APPLIED`, the source is `MAVLINK`, and channel 2 goes from 1500 µs to
  1400 µs (25 %) because the Copilot brakes after an RC loss
- **THEN** a release run SHALL start once the estimate is at or above 24 %

#### Scenario: Brake held when the parking brake is applied
- **WHEN** the service brake target is 60 %, the park brake is applied from the web and becomes
  `APPLIED`, and the target stays at 60 %
- **THEN** no release SHALL start
- **AND** after the target goes to 0 and then to a non-zero value, a release SHALL start after the
  hand-over

#### Scenario: Frozen target under the firmware fail-safe
- **WHEN** the input source is `FAILSAFE` with a non-zero brake target left from before, and later
  `MAVLINK` returns with the same value
- **THEN** no release SHALL start

#### Scenario: Takeover with an inherited target
- **WHEN** web control is engaged while the MAVLink brake target is 80 %, and the web then sends 80 %
- **THEN** no release SHALL start
- **AND** a web brake of 0 followed by a non-zero web brake SHALL start a release after the hand-over

#### Scenario: Press dropped before the hand-over
- **WHEN** a press is pending and the target returns to 0 before the estimate reaches the target
- **THEN** no release SHALL start
- **AND** the next press SHALL start a new hand-over

#### Scenario: Service brake let go mid-release
- **WHEN** a release started by the service brake is running and the operator lets go of the service
  brake
- **THEN** the run SHALL complete and the state SHALL become `RELEASED`
- **AND** the park brake SHALL NOT be re-applied

#### Scenario: No apply and no effect when not applied
- **WHEN** the park brake is `RELEASED`, `RELEASING` or `APPLYING` and the operator presses the
  service brake
- **THEN** the park-brake state and run SHALL NOT change

#### Scenario: Any gear, any ignition
- **WHEN** the park brake is `APPLIED`, the target gear is N, the ignition relay is `OFF`, and a press
  completes its hand-over
- **THEN** a release run SHALL start

#### Scenario: Throttle hold unchanged
- **WHEN** a release started by the service brake runs with the target gear L
- **THEN** the operator throttle SHALL be held at idle up to the throttle release point, exactly as
  for any release run
