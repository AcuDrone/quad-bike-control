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

## MODIFIED Requirements
### Requirement: Aztec Parking Brake Actuator
The aztec buggy SHALL drive its parking-brake actuator through a second `BTS7960Controller` on
connector X6: `PIN_PARK_BRAKE_LPWM` = GPIO4 (X6 pin 2) and `PIN_PARK_BRAKE_RPWM` = GPIO5 (X6 pin 3).
The actuator is self-locking and has built-in limit switches. The firmware has no position
feedback and SHALL end each movement by time.

A movement ("run") SHALL drive `PARK_BRAKE_DRIVE_DUTY` (255) in the commanded direction for the
configured travel time plus `PARK_BRAKE_TRAVEL_MARGIN_MS` (500 ms). It SHALL then stop the bridge
(both PWM 0) with no hold drive.

The park-brake state SHALL be one of `UNKNOWN`, `RELEASED`, `APPLIED`, `APPLYING` and `RELEASING`.
Commands SHALL enter through one source-agnostic function,
`VehicleController::commandParkBrake(request, source)`, which returns a result code. The web is
its only caller in this change, and a later GCS path SHALL be able to call it unchanged.

The park brake SHALL be applied only by an explicit operator command. It SHALL NOT be applied
automatically on RC or link loss, on fail-safe, on ignition off, at boot, or for any other reason.
The actuator's motor supply is on the permanent battery feed, so commands SHALL NOT be gated by
input source, ignition state or web control. They SHALL be refused
with "Park brake driver not available" only when the X6 driver failed to initialise.

The park brake SHALL be independent of the X7 service brake: separate driver, state and commands.

#### Scenario: Apply run
- **WHEN** an APPLY command is accepted while the state is `RELEASED`, `UNKNOWN` or `APPLIED`
- **THEN** the state SHALL become `APPLYING` and the bridge SHALL drive the apply direction at duty
  255
- **AND** after the travel time plus 500 ms the bridge SHALL be stopped and the state SHALL become
  `APPLIED`
- **AND** no PWM SHALL be applied to X6 while the state is `APPLIED`

#### Scenario: Release run
- **WHEN** a RELEASE command is accepted while the state is `APPLIED`, `UNKNOWN` or `RELEASED`
- **THEN** the state SHALL become `RELEASING`, the bridge SHALL drive the release direction at duty
  255 for the travel time plus 500 ms, then stop
- **AND** the state SHALL become `RELEASED`

#### Scenario: Release is always allowed
- **WHEN** a RELEASE command arrives at any vehicle speed, from any input source, with any ignition
  state, or during fail-safe
- **THEN** it SHALL be accepted, unless the X6 driver failed to initialise

#### Scenario: Repeated command during a run
- **WHEN** an APPLY arrives while `APPLYING`, or a RELEASE arrives while `RELEASING`
- **THEN** the run SHALL continue unchanged and its timer SHALL NOT be restarted
- **AND** the web reply SHALL be "Park brake already applying" or "Park brake already releasing"

#### Scenario: Opposite command during a run
- **WHEN** a RELEASE arrives while `APPLYING`, or an accepted APPLY arrives while `RELEASING`
- **THEN** the bridge SHALL be stopped and held at zero duty for `PARK_BRAKE_REVERSE_DWELL_MS`
  (200 ms)
- **AND** a full run (travel time plus 500 ms) SHALL then be driven in the new direction
- **AND** the state SHALL become the new moving state as soon as the command is accepted

#### Scenario: No automatic apply
- **WHEN** the MAVLink link is lost, the input source becomes FAILSAFE, the ignition is switched
  OFF, or the firmware boots
- **THEN** the park brake SHALL NOT be applied and the X6 bridge SHALL NOT be driven by that event

#### Scenario: Fail-safe does not interrupt a run
- **WHEN** the input source becomes FAILSAFE while a run is in progress
- **THEN** the run SHALL continue to its end and the state SHALL settle as for any run

#### Scenario: Service brake independent
- **WHEN** the park brake is commanded, running or settled
- **THEN** the X7 service brake target, estimate, hold drive and telemetry SHALL be unaffected
- **AND** a service brake command SHALL NOT change the park-brake state, except through the
  requirement Parking Brake Release by Service-Brake Press

#### Scenario: Driver unavailable
- **WHEN** the X6 `BTS7960Controller::begin()` failed at boot and any park-brake command arrives
- **THEN** it SHALL be refused with "Park brake driver not available" and the state SHALL NOT change
