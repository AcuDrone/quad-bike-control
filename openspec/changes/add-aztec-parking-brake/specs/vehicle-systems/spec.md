## ADDED Requirements
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
- **AND** a service brake command SHALL NOT change the park-brake state

#### Scenario: Driver unavailable
- **WHEN** the X6 `BTS7960Controller::begin()` failed at boot and any park-brake command arrives
- **THEN** it SHALL be refused with "Park brake driver not available" and the state SHALL NOT change

### Requirement: Parking Brake Apply Speed Interlock
An APPLY SHALL be refused while the vehicle moves faster than `PARK_BRAKE_APPLY_MAX_SPEED_MS`
(0.833 m/s = 3 km/h). The speed SHALL be read from the hall speed sensor when its reading is valid
or latched suspicious, the same trust rule as the gear-change speed interlock. The CAN speed field
SHALL NOT be used.

#### Scenario: Apply refused while moving
- **WHEN** an APPLY command arrives and the trusted speed is above 0.833 m/s
- **THEN** it SHALL be refused, the state SHALL NOT change and a running release SHALL continue
- **AND** the web reply SHALL be a failure "Park brake apply refused: moving at N.N km/h (max 3.0)"
- **AND** the refusal SHALL be logged under the `BRAKE` debug feature

#### Scenario: Apply accepted at standstill
- **WHEN** an APPLY command arrives and the trusted speed is at or below 0.833 m/s
- **THEN** it SHALL be accepted and the reply SHALL be "Park brake applying"

#### Scenario: Speed sensor not yet pulsed
- **WHEN** an APPLY command arrives and the speed sensor has not pulsed since boot (not valid and not
  suspicious)
- **THEN** the APPLY SHALL be accepted and the reply SHALL be "Park brake applying (speed unknown)"

### Requirement: Parking Brake Drive-Away Release and Throttle Hold
The operator throttle is the web demand or the MAVLink throttle channel (manual or autopilot)
after the existing gear clip. "Ignition on" SHALL mean that the firmware's commanded ignition-relay
state, `RelayController::getIgnitionState()`, is not `OFF` (ACC, IGNITION or CRANKING: the state
in which `RELAY_SET_IGNITION` is energized). It SHALL NOT be derived from engine RPM.

**Throttle hold.** While the transmission target gear is L, H or R, the operator throttle SHALL be
held at idle when the park brake is:
- `APPLIED`, `APPLYING` or `UNKNOWN`; or
- `RELEASING`, until the throttle release point has elapsed.

The throttle release point is `thr_pass_pct` % of `travel_ms`, counted from the start of the
release drive (after any reverse dwell). With 100 % the hold SHALL last until the run ends, margin
included. From the release point on, the operator throttle SHALL pass through the speed limiter and
the opening slew from idle while the actuator finishes its run. This SHALL apply to every RELEASING
run, whatever started it and whatever the starting state.

**Drive-away.** The firmware SHALL start a RELEASE ("drive-away") when all of these hold:
- the park brake is `APPLIED` or `UNKNOWN`;
- the ignition is on;
- the target gear is L, H or R and no gear change is in progress;
- the throttle demand rises above `PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT` (10 %), with no dwell or
  debounce.

The demand SHALL have been at or below 10 % while the brake was `APPLIED` or `UNKNOWN` and the
ignition was on, since the last release started (rising edge).

The drive-away release SHALL NOT depend on engine start or engine RPM. In N, and while the target
gear is UNKNOWN, throttle SHALL neither be held by the park brake nor release it. A release run
SHALL complete once started, even if the throttle drops. There SHALL be no automatic re-apply.

#### Scenario: Drive away in gear with the default release point
- **WHEN** the park brake is `APPLIED`, the ignition is on, the target gear is L, the travel time is
  2000 ms, the release point is 50 %, and the operator raises the throttle from 0 % to 30 %
- **THEN** a release run SHALL start at once
- **AND** the throttle servo SHALL stay at idle for the first 1000 ms of the release drive
- **AND** from 1000 ms the 30 % demand SHALL pass through the speed limiter and the opening slew,
  rising from idle, while the actuator keeps releasing
- **AND** the run SHALL end and the state SHALL become `RELEASED` at 2500 ms

#### Scenario: Release point 100 %
- **WHEN** the release point is 100 % and a release run is in progress in gear
- **THEN** the throttle SHALL stay at idle until the run ends (travel time plus 500 ms) and the
  state is `RELEASED`

#### Scenario: Operator Release uses the same release point
- **WHEN** the operator presses Release while the brake is `APPLIED` or `UNKNOWN` and the target
  gear is H
- **THEN** the throttle SHALL be held at idle until the release point and pass from then on, exactly
  as for a drive-away release

#### Scenario: Apply after the throttle was passed
- **WHEN** a release run has passed its release point (throttle passing) and an APPLY is accepted
- **THEN** the run SHALL reverse (state `APPLYING`)
- **AND** the operator throttle SHALL be put back to idle at once

#### Scenario: Throttle dropped during a release
- **WHEN** the operator drops the throttle to 0 % during a release run
- **THEN** the run SHALL complete and the state SHALL become `RELEASED`
- **AND** the park brake SHALL NOT be re-applied

#### Scenario: Throttle held at or below the threshold
- **WHEN** the park brake is `APPLIED`, the target gear is H and the demand is 8 %
- **THEN** the throttle servo SHALL stay at idle and no release SHALL start

#### Scenario: Ignition off
- **WHEN** the park brake is `APPLIED`, the commanded ignition state is `OFF`, the target gear is L
  and the demand rises from 0 % to 30 %
- **THEN** no release SHALL start, the brake SHALL stay applied and the throttle SHALL stay at idle
- **AND** telemetry `pbrake_da_wait` SHALL be `"ign_off"`
- **AND** an explicit operator Release SHALL still be accepted

#### Scenario: ACC, IGNITION and CRANKING all count as ignition on
- **WHEN** the park brake is `APPLIED`, the target gear is L, the drive-away latch is armed, and the
  commanded ignition state is `ACC`, `IGNITION` or `CRANKING`
- **AND** the demand rises from 0 % to 30 %
- **THEN** a release run SHALL start in each of the three states
- **AND** only the state `OFF` (including `OFF` reported because the relay board is unavailable)
  SHALL block the drive-away release

#### Scenario: Ignition switched on with the throttle already up
- **WHEN** the park brake is `APPLIED`, the target gear is L, the demand is held at 30 % and the
  ignition is switched from `OFF` to `ACC` or `IGNITION`
- **THEN** no release SHALL start
- **AND** `pbrake_da_wait` SHALL be `"rearm"`
- **AND** a release SHALL start only after the demand has dropped to 10 % or below and risen above
  10 % again with the ignition on

#### Scenario: Revving in neutral
- **WHEN** the park brake is `APPLIED` and the target gear is N
- **THEN** throttle SHALL NOT start a release and SHALL NOT be held by the park brake
- **AND** the existing neutral clip (`TRANS_UNKNOWN_GEAR_THROTTLE_MAX`) SHALL still apply

#### Scenario: Unknown state behaves as possibly applied
- **WHEN** the state is `UNKNOWN`, the ignition is on and the target gear is R
- **THEN** the throttle SHALL be held at idle
- **AND** a rising demand above 10 % SHALL start a release with the same release point, which
  settles the state as `RELEASED`

#### Scenario: Held throttle does not re-release after an apply
- **WHEN** the operator applies the park brake while the demand is already above 10 % in gear and
  stays there
- **THEN** no drive-away release SHALL start when the apply completes, and the throttle SHALL stay
  at idle
- **AND** `pbrake_da_wait` SHALL be `"rearm"`
- **AND** a release SHALL start only after the demand has dropped to 10 % or below and risen above
  10 % again

#### Scenario: Gear change keeps its own throttle ownership
- **WHEN** a gear change runs (boost PID or shift idle hold) while the park brake is applied
- **THEN** the gear change SHALL own the throttle exactly as without the park brake
- **AND** the park-brake hold SHALL act on the operator throttle path only

#### Scenario: Web throttle reply while held
- **WHEN** a web `set_throttle` command is accepted while the park-brake hold is active
- **THEN** the reply SHALL be a success "Throttle set — held at idle until the park brake is released"
- **AND** with the ignition off it SHALL be "Throttle set — held at idle: ignition off, the park
  brake stays applied"

### Requirement: Parking Brake State Persistence
The park-brake state SHALL be persisted in the NVS namespace `"pbrake"`, key `state` (uint8:
0 unknown, 1 released, 2 applied, 3 applying, 4 releasing). It SHALL be written as the moving value
before a run drives the bridge, and as the settled value after the bridge stops.

At boot a stored 1 or 2 SHALL restore `RELEASED` or `APPLIED`. Any other stored value, including a
missing key, SHALL give `UNKNOWN`; it SHALL be logged and not rewritten. The firmware SHALL NOT
drive the actuator at boot.

#### Scenario: State survives a reboot
- **WHEN** the park brake was `APPLIED` and the board reboots
- **THEN** the state SHALL be `APPLIED` after boot and the X6 bridge SHALL stay at zero duty

#### Scenario: Power loss mid-run
- **WHEN** power is lost, the board resets or an OTA reboot happens during a run
- **THEN** the state after boot SHALL be `UNKNOWN`, not `APPLIED` or `RELEASED`

#### Scenario: First boot
- **WHEN** the namespace `"pbrake"` has no `state` key
- **THEN** the state SHALL be `UNKNOWN` until a run completes

#### Scenario: Save failure at run start
- **WHEN** writing the moving state fails as an APPLY starts
- **THEN** the APPLY SHALL be refused with "Failed to save the park brake state — apply refused" and
  the bridge SHALL NOT be driven
- **WHEN** writing the moving state fails as a RELEASE starts
- **THEN** the release SHALL run and the reply SHALL end with "(state not saved)"

### Requirement: Configurable Parking Brake Travel Time, Throttle Release Point and Direction
The parking-brake travel time, throttle release point and drive direction SHALL be persisted in the
NVS namespace `"pbrake"`:
- `travel_ms`: uint16, 1000–5000 ms, default 2000 ms;
- `thr_pass`: uint8, the throttle release point in % of `travel_ms`, 20–100, default 50;
- `invert`: uint8 0/1, default 0.

With `invert` = 0, APPLY SHALL drive RPWM (GPIO5) and RELEASE SHALL drive LPWM (GPIO4); `invert` =
1 SHALL swap them. A missing key SHALL read as the default. A stored out-of-range travel time
or release point SHALL be logged and ignored, and not rewritten. Changes SHALL be persisted first,
then applied. A new travel time SHALL take effect from the next run. A new release point SHALL take
effect from the next release run.

#### Scenario: Set the travel time
- **WHEN** a travel time of 3200 ms is set
- **THEN** it SHALL be persisted and the next run SHALL last 3700 ms
- **AND** a run already in progress SHALL keep its duration

#### Scenario: Travel time out of range
- **WHEN** a travel time below 1000 ms or above 5000 ms is set
- **THEN** it SHALL be refused and the value in use SHALL NOT change

#### Scenario: Set the throttle release point
- **WHEN** a release point of 70 % is set with a travel time of 2000 ms
- **THEN** it SHALL be persisted, and in the next release run the throttle SHALL be held for 1400 ms
  of the release drive

#### Scenario: Throttle release point out of range
- **WHEN** a release point below 20 % or above 100 % is set
- **THEN** it SHALL be refused and the value in use SHALL NOT change

#### Scenario: Invert the direction
- **WHEN** the direction setting is changed while no run is in progress
- **THEN** `invert` SHALL be persisted
- **AND** the state SHALL be set to and persisted as `UNKNOWN`

#### Scenario: Invert refused during a run
- **WHEN** the direction setting is changed while `APPLYING` or `RELEASING`
- **THEN** it SHALL be refused with "Park brake direction change refused: actuator moving"
