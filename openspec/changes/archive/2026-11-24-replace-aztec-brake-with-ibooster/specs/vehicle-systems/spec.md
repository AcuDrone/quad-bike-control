## MODIFIED Requirements
### Requirement: Degraded Operation on Board I/O Fault
The system SHALL define safe, observable degraded behavior when the opto-input expander or the
relay expander becomes unavailable, since the gear interlock and the ignition/starter path depend on
I2C. The service brake SHALL NOT depend on the opto-input expander.

#### Scenario: Gear inputs unavailable
- **WHEN** the opto-input snapshot is invalid (stale beyond `BOARD_INPUT_STALE_MS`)
- **THEN** `TransmissionController::getPhysicalGear()` SHALL report `GEAR_UNKNOWN`
- **AND** the existing unknown-gear behavior SHALL apply: throttle capped at
  `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (5%) and gear-change confirmation blocked
- **AND** no additional interlock SHALL be introduced — the existing unknown-gear path is the
  intended degraded mode
- **AND** the fault SHALL be visible in telemetry

#### Scenario: Brake limit sensor unavailable
- **WHEN** the opto-input snapshot is invalid
- **THEN** nothing about the service brake SHALL change: there is no brake limit sensor any more and
  the iBooster uses no opto input
- **AND** the system SHALL NOT report any brake state on the basis of the input expander

#### Scenario: Relay expander unavailable
- **WHEN** the relay expander failed to initialize or is faulted
- **THEN** ignition, starter, front light and wheel lock commands SHALL become logging no-ops
- **AND** the reported ignition state SHALL be `OFF`
- **AND** steering, throttle, MAVLink and web telemetry SHALL continue to operate
- **AND** the service brake SHALL keep sending its frames for as long as the unit is still heard on
  the bus, so a relay that physically stays on does not make the firmware drop an active brake
- **AND** the fault SHALL be visible in telemetry

#### Scenario: Faults recover without a reboot
- **WHEN** a faulted expander begins responding again
- **THEN** the affected subsystem SHALL resume normal operation automatically
- **AND** the recovery SHALL be logged
- **AND** the telemetry fault flag SHALL clear

### Requirement: Parking Brake Release by Service-Brake Press
On the aztec buggy, a new operator service-brake press SHALL start a parking-brake RELEASE run while
the park brake is `APPLIED` or `UNKNOWN`: while the service brake is available, once it reports that
it has reached the pressed position; while it is not available, at once. For this one trigger, this requirement takes precedence over the rule that a
service brake command does not change the park-brake state.

By owner decision of 2026-11-17 this trigger has **no protection** against an automatic brake: a
brake commanded on servo channel 2 by any sender (RC handset, GCS joystick, the BlueOS Copilot's
automatic fail-safe brake, an autopilot mode) and a web brake command all count.

**Input.** The trigger SHALL use the commanded service-brake target as `applyBrake()` sets it (below
5 % it is 0, above 100 % it is 100 %), the service brake's `available` indication (see "Service Brake
Abstraction") and its `reached` indication (see "iBooster Rod Control"). A **press** is that target changing from 0 to a value above 0 while the input source is
`MAVLINK` or `WEB`.

**Rising edge.** The firmware SHALL sample the target on every control loop in every park-brake
state, so that:
- a target that is already non-zero when the park brake becomes `APPLIED` is not a press;
- a change from one non-zero value to another is not a press;
- under the `FAILSAFE` input source no press SHALL be detected, and the target frozen there SHALL NOT
  count as a press when a live source returns;
- on any input-source change the sample SHALL be re-seeded with the current target, so a takeover
  (MAVLINK ↔ WEB, or from or to `FAILSAFE`) never produces a press by itself.

**Hand-over.** After a press the release SHALL start on the first loop in which the service brake
reports `reached` or is not `available`. While the service brake is not available (ignition OFF,
unit asleep, starting, lost or in fault) the release therefore starts on the loop of the press. The
wait SHALL be cancelled, with no release, when the target returns to 0, the input source changes,
the park-brake state is no longer `APPLIED` or `UNKNOWN`, or the service brake is available but
reports that its rod does not move ("no move", see "iBooster Rod Control"). A further release then
needs a new press.

By owner decision of 2026-11-23 a press releases the parking brake also while the service brake is
not acting; the buggy can then be held by neither brake.

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
- **WHEN** the park brake is `APPLIED`, the source is `MAVLINK`, the ignition is on, the service
  brake is `READY` and released
- **AND** the operator presses the service brake to 50 % and holds it
- **THEN** the release run SHALL start on the first loop in which the service brake reports
  `reached`
- **AND** the state SHALL become `RELEASING`

#### Scenario: Release from UNKNOWN
- **WHEN** the park-brake state is `UNKNOWN` and a press completes its hand-over
- **THEN** a release run SHALL start and settle the state as `RELEASED`

#### Scenario: Web press releases
- **WHEN** the park brake is `APPLIED`, the source is `WEB`, the target is 0 and the operator sends a
  web brake of 80 %
- **THEN** a release run SHALL start once the service brake reports `reached`

#### Scenario: Automatic fail-safe brake releases (accepted)
- **WHEN** the park brake is `APPLIED`, the source is `MAVLINK`, and channel 2 goes from 1500 µs to
  1400 µs (25 %) because the Copilot brakes after an RC loss
- **THEN** a release run SHALL start once the service brake reports `reached`

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
- **WHEN** a press is pending and the target returns to 0 before the service brake reports `reached`
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

#### Scenario: Press releases while the service brake is unavailable (accepted)
- **WHEN** the park brake is `APPLIED`, the iBooster is `ASLEEP`, `STARTING`, `LOST` or in `FAULT`,
  and the operator presses the service brake
- **THEN** a release run SHALL start on the loop of the press
- **AND** the service brake SHALL NOT act, so the buggy is held by neither brake (owner decision)

#### Scenario: Service brake ready but its rod does not move
- **WHEN** a press is pending, the service brake is available, and it reports "no move" (accepted,
  pushing, rod not moved 0.3 mm for 3 s)
- **THEN** the wait SHALL be cancelled with no release, `BRAKE W pbrake_handover_cancel why=no_move`
  SHALL be journaled, and the parking brake SHALL stay applied until the Release command or a new
  press

#### Scenario: Unit becomes unavailable during the hand-over
- **WHEN** a press is pending, waiting for `reached`, and the service brake becomes unavailable
- **THEN** the release run SHALL start on that loop

#### Scenario: Any gear, any ignition
- **WHEN** the park brake is `APPLIED`, the target gear is N, the ignition relay is `OFF`, and a press
  occurs
- **THEN** a release run SHALL start (the service brake is asleep, so there is no hand-over wait)

#### Scenario: Throttle hold unchanged
- **WHEN** a release started by the service brake runs with the target gear L
- **THEN** the operator throttle SHALL be held at idle up to the throttle release point, exactly as
  for any release run

## ADDED Requirements
### Requirement: Service Brake Control
On the aztec buggy the service brake SHALL be a Bosch iBooster Gen2 commanded over the shared CAN
bus (see "Bosch iBooster Gen2 Service Brake Unit", "iBooster Unit State Machine" and "iBooster Rod
Control"). There SHALL be no other service-brake actuator: no BTS7960 brake drive, no open-loop
position estimate, no hold drive and no "released" endstop input. The brake command SHALL be a
percentage from 0 to 100 set through `applyBrake()`, which SHALL keep its clamp: below 5 % becomes
0, above 100 % becomes 100 %. The applied brake SHALL be the rod position reported by the unit.

#### Scenario: Set brake by percentage
- **WHEN** `applyBrake()` is called with a value from 5 to 100
- **THEN** the service brake target SHALL become that percentage of the configured maximum stroke
- **AND** 0 % SHALL mean released and 100 % the configured maximum stroke

#### Scenario: Dead zone kept
- **WHEN** `applyBrake()` is called with 3 %
- **THEN** the target SHALL be 0 and no brake request SHALL be sent

#### Scenario: Release
- **WHEN** the target goes from a non-zero value to 0
- **THEN** the rod SHALL be returned under control and the brake request SHALL then be cleared, as
  "iBooster Rod Control" describes

#### Scenario: Applied brake is measured
- **WHEN** the unit reports its rod position
- **THEN** the applied brake percentage SHALL be that position as a percentage of the configured
  maximum stroke, 0–100
- **AND** while the unit is not available the applied percentage SHALL be reported as 0 together with
  the brake state

#### Scenario: No brake action at power-on
- **WHEN** the system powers on or resets
- **THEN** no brake request SHALL be sent until a brake target above 0 is commanded and the unit is
  ready
- **AND** no parking or start-up brake SHALL be applied by the service brake

### Requirement: Service Brake Abstraction
The firmware SHALL talk to the service brake only through one interface (`IServiceBrake`), used from
the main loop: set a percent target; read the target, the applied percentage, `reached`,
`available`, `requestActive` and the brake state. The iBooster implementation SHALL sit behind it.
`VehicleController` SHALL NOT hold a motor driver for the service brake.

#### Scenario: Commanded and applied are separate
- **WHEN** a target of 50 % is set and the rod has not moved yet
- **THEN** the target SHALL read 50 % and the applied percentage SHALL read the measured position

#### Scenario: Throttle paths keep using the command
- **WHEN** the speed governor, the speed limiter or the throttle opening slew asks whether a brake is
  commanded
- **THEN** the answer SHALL be "commanded target above 0", whatever the unit's state
- **AND** a brake commanded while the unit is unavailable SHALL still close the throttle as before

#### Scenario: Availability
- **WHEN** the brake state is `READY`, `BRAKING` or `RELEASING`
- **THEN** `available` SHALL be true, and false in every other state
- **AND** `requestActive` SHALL be true exactly in `BRAKING` and `RELEASING`

### Requirement: iBooster Unit State Machine
The service brake SHALL track the unit in the states `ASLEEP`, `STARTING`, `READY`, `BRAKING`,
`RELEASING`, `LOST` and `FAULT`, stepped once per 10 ms slot in the CAN task by a pure law
(`include/IboosterLaw.h`), using the newest valid `0x39D`, `0x38E` and `0x38F` frames. No brake
request SHALL be sent in `ASLEEP`, `STARTING`, `LOST` or `FAULT`.

#### Scenario: Asleep with the ignition off
- **WHEN** the ignition state is `OFF` and no valid unit frame has arrived for `IB_SILENT_MS`
  (500 ms)
- **THEN** the state SHALL be `ASLEEP` and no frame SHALL be sent to the unit

#### Scenario: Start-up
- **WHEN** the ignition leaves `OFF`, or a valid unit frame arrives while `ASLEEP`
- **THEN** the state SHALL be `STARTING` and idle frames SHALL be sent every 10 ms
- **AND** the state SHALL become `READY` when a `0x39D` no older than `IB_STATUS_STALE_MS` (250 ms)
  shows status 5 or 6 with driver_brake_apply other than 3, and `0x38F` byte 4 is `0x40`
- **AND** if `READY` is not reached within `IB_START_TIMEOUT_MS` (20 s) the event SHALL be journaled
  once and the state SHALL stay `STARTING`

#### Scenario: The unit's own actuation cycle is not a fault
- **WHEN** the unit reports status 6 with internal state 5 (its periodic self-cycle, or `to_idle`
  after a release) and no other fault sign
- **THEN** the state SHALL stay `READY`

#### Scenario: Unit lost
- **WHEN** frames are being sent, the ignition is reported on, the state is `READY`, `BRAKING` or
  `RELEASING`, and the newest valid `0x39D` is older than 250 ms
- **THEN** the state SHALL become `LOST`, the request SHALL be cleared and idle frames SHALL continue
- **AND** when valid frames return the state SHALL go through `STARTING` to `READY` again

#### Scenario: Unit going to sleep with the ignition reported OFF
- **WHEN** the ignition is reported `OFF` and the unit, heard until now, falls silent
- **THEN** the state SHALL NOT become `LOST`; frames SHALL continue until 500 ms of silence and the
  state SHALL then become `ASLEEP`
- **AND** while the newest `0x39D` is older than 250 ms no new request SHALL start

#### Scenario: Unit no longer ready
- **WHEN** a fresh `0x39D` shows no fault sign but the `READY` condition is gone (status other than 5
  or 6, or `0x38F` byte 4 not `0x40`) while `READY`, or a status other than 5 or 6 while `BRAKING` or
  `RELEASING`
- **THEN** the state SHALL become `STARTING` and the request SHALL be cleared

#### Scenario: Unit fault
- **WHEN** a fresh `0x39D` shows status 2 (failure) or driver_brake_apply 3 (fault)
- **THEN** the state SHALL become `FAULT`, the request SHALL be cleared at once and idle frames SHALL
  continue
- **AND** the state SHALL leave `FAULT` only when the unit itself reports the `READY` condition again
- **AND** a `FAULT` lasting `IB_FAULT_PERSIST_MS` (20 s) SHALL be journaled once as latched

#### Scenario: Cranking dip
- **WHEN** a `LOST` or `FAULT` starts while the ignition is `CRANKING` or within `IB_CRANK_GRACE_MS`
  (3 s) after it
- **THEN** the state machine SHALL behave as at any other time
- **AND** the event SHALL be journaled at level `W` with `crank=1`, not at level `E`

#### Scenario: Brake commanded while not available
- **WHEN** the target is above 0 and the state is `STARTING`, `LOST` or `FAULT`
- **THEN** no request SHALL be sent and the target SHALL be kept
- **AND** when the state becomes `READY` with the target still above 0, the state SHALL become
  `BRAKING`

#### Scenario: Ignition switched off and on again
- **WHEN** the ignition goes from `OFF` back to on while the state is `READY`, `BRAKING`,
  `RELEASING` or `LOST` (the unit lost its Wake meanwhile, however briefly)
- **THEN** the state SHALL become `STARTING` (silent, no `LOST`), any request SHALL be cleared
- **AND** `READY` SHALL then need `0x39D` and `0x38F` frames that arrived after the ignition edge

#### Scenario: Request not accepted
- **WHEN** the request has been set for `IB_ACCEPT_TIMEOUT_MS` (1000 ms) and `0x39D` internal state
  is not 3 (external request)
- **THEN** a unit drop SHALL be counted and journaled, and the request SHALL be kept

### Requirement: iBooster Rod Control
In `BRAKING` and `RELEASING` the CAN task SHALL close a proportional loop on the `0x39D` rod position
every 10 ms slot and send the result as the `0x38C` flow, with the external-request bits set.

#### Scenario: Rod target
- **WHEN** the brake target is P % and the configured maximum stroke is M mm
- **THEN** the rod target SHALL be P / 100 × M mm, never above `IB_ROD_CEIL_MM` (16.0 mm)

#### Scenario: Flow from the rod error
- **WHEN** the rod error (target − rod) is E mm
- **THEN** the flow SHALL be `IB_ROD_KP` (250 flow units per mm) × E, 0 inside the deadband
  `IB_ROD_DEADBAND_MM` (0.05 mm), limited to + apply rate and − release rate (ml/s × 128 units)
- **AND** the flow word SHALL be `0x7E00` + flow and SHALL always stay within `0x6A00…0x9200`

#### Scenario: Old rod sample
- **WHEN** a request is set and the newest valid `0x39D` is older than `IB_ROD_HOLD_MS` (100 ms) but
  not yet stale (250 ms)
- **THEN** the flow SHALL be 0 (the rod holds) until a newer `0x39D` arrives; the loop SHALL never
  drive the rod from an old position

#### Scenario: Controlled release
- **WHEN** the target becomes 0 while `BRAKING`
- **THEN** the state SHALL become `RELEASING`, the request SHALL stay set and the rod SHALL be
  returned at no more than the release rate
- **AND** when the rod is at or below `IB_RELEASE_HANDOFF_MM` (1.0 mm) the request SHALL be cleared
  and the state SHALL become `READY`
- **AND** the release time budget SHALL be 1 s + twice the nominal return time from the rod position
  at `RELEASING` entry to 1.0 mm at the release rate (≈ 1.9 mm/s per ml/s), at least
  `IB_RELEASE_TIMEOUT_MS` (3 s) and at most `IB_RELEASE_MAX_MS` (30 s; also when the entry rod is
  unknown)
- **AND** past the budget the request SHALL be cleared, and `release_slow` journaled, only once the
  rod has stopped coming down (less than 0.1 mm in the last 1 s), or at the 30 s cap; while it still
  comes down the request SHALL be kept

#### Scenario: Re-press during release
- **WHEN** a target above 0 is commanded while `RELEASING` and the newest `0x39D` is fresh
- **THEN** the state SHALL become `BRAKING` with the new target
- **AND** with a stale `0x39D` (ignition reported OFF, unit going quiet) the state SHALL stay
  `RELEASING`

#### Scenario: Re-command after a gap
- **WHEN** the unit released by itself after a frame gap and accepted the still-set request again
- **THEN** the loop SHALL drive the rod back to the target with no operator action

#### Scenario: Reached
- **WHEN** the state is `BRAKING`, the unit reports internal state 3, and either the rod is within
  `IB_REACHED_TOL_MM` (0.2 mm) of the target, or — only after the rod has advanced `IB_ARM_MM`
  (0.3 mm) from its first position in this `BRAKING` episode, because the unit first takes up
  ~2 mm of free travel with the output rod standing still — the flow is positive and the rod has
  advanced less than 0.05 mm in `IB_STALL_MS` (300 ms), or `0x38F` byte 3 is 0
- **THEN** `reached` SHALL be true
- **AND** in every other state `reached` SHALL be false
- **AND** `reached` SHALL refer to the target currently commanded: a result computed for an earlier
  target SHALL NOT be reported

#### Scenario: Rod does not move
- **WHEN** the state is `BRAKING`, the unit reports internal state 3, the flow is positive and the
  rod has not advanced 0.3 mm for `IB_NO_MOVE_MS` (3 s)
- **THEN** "no move" SHALL be reported (journal `BRAKE W no_move`, once per episode), `reached` SHALL
  stay false and the request SHALL be kept
- **AND** "no move" SHALL clear as soon as the rod moves 0.3 mm

#### Scenario: Vehicle speed to the unit
- **WHEN** the frames are built
- **THEN** `0x38B` byte 2 SHALL be round(speed km/h / 0.4), clamped to 0–255, while the wheel-speed
  signal is valid, and 0 while it is invalid
- **AND** the "faulty" speed qualifier (`0x80`) SHALL never be sent

### Requirement: Service Brake Settings
The maximum stroke, the apply rate and the release rate SHALL be persisted, operator-set, per-vehicle
values in the NVS namespace `brake`: `max_mm10` (0.1 mm, 2.0–16.0 mm, default 16.0 — the full
external-request stroke), `apply_mlps` (1–40 ml/s, default 8) and `release_mlps` (1–40 ml/s,
default 8). The former keys `travel_ms` and
`hold_pwm` SHALL be removed once at boot and never read.

#### Scenario: Defaults
- **WHEN** the system starts without stored values
- **THEN** the maximum stroke SHALL be 16.0 mm and both rates 8 ml/s
- **AND** a 100 % brake SHALL then command a rod target of 16.0 mm

#### Scenario: Invalid stored value
- **WHEN** a stored value is outside its range
- **THEN** it SHALL be ignored and logged, the default SHALL be used and the stored value SHALL NOT be
  rewritten

#### Scenario: Change applied
- **WHEN** a valid value is set and no brake request is active
- **THEN** it SHALL be persisted first and, once the write succeeded, used from the next slot
- **AND** the maximum stroke SHALL be accepted only on the 0.5 mm grid (2.0, 2.5, … 16.0 mm); a value
  off the grid is answered as out of range
- **AND** if the write fails the value in use SHALL stay unchanged

#### Scenario: Refused while braking
- **WHEN** a setting is sent while a brake request is active
- **THEN** it SHALL be refused, and neither the stored nor the in-use value SHALL change

#### Scenario: Old keys removed
- **WHEN** the system starts and `brake`/`travel_ms` or `brake`/`hold_pwm` exist
- **THEN** they SHALL be removed before the CAN bus is started

### Requirement: Service Brake With the Ignition OFF and Under Fail-Safe
The iBooster SHALL be woken only by the ACC / ignition line (Relay_1). This change SHALL NOT alter
the fail-safe. Both consequences are accepted by the owner and SHALL be stated in the operator
documentation: with the ignition `OFF` there is no service brake, and on fail-safe the brake drops
when `allOff()` opens the ignition relay.

#### Scenario: Fail-safe with a held brake
- **WHEN** the input source becomes `FAILSAFE` while the brake target is 40 %
- **THEN** the fail-safe SHALL do exactly what it did before this change (steering centre, throttle
  idle, transmission stop, `allOff()`, odometer and hours persisted) and SHALL add no brake action
- **AND** the brake target SHALL stay frozen at 40 %
- **AND** once the unit has gone silent the state SHALL become `ASLEEP` and no frames SHALL be sent

#### Scenario: Ignition off
- **WHEN** the ignition is switched `OFF` by any path
- **THEN** the service brake SHALL be reported unavailable (`ASLEEP`) once the unit is silent
- **AND** no brake request SHALL be attempted

#### Scenario: Main loop stalls
- **WHEN** the CAN task has received no brake command from the main loop for `IB_CMD_STALE_MS`
  (500 ms)
- **THEN** the task SHALL keep the last rod target and vehicle-speed byte and keep sending frames
- **AND** the episode and its length SHALL be journaled when the main loop runs again

### Requirement: Flash Writes During a Service-Brake Request
While a brake request is active (`BRAKING` or `RELEASING`) the firmware SHALL keep flash writes away
from the 10 ms frame schedule where it can, because a flash erase stalls the CAN task.

#### Scenario: Periodic saves deferred
- **WHEN** the odometer (every 1 km) or the engine-hour meter (every 600 s) is due to save while a
  request is active
- **THEN** the save SHALL be postponed and SHALL run on the first update after the request ends,
  moving or not (the due check runs on every update)
- **AND** the postponement SHALL last no longer than the request; a request ends with the brake
  release, with the ignition OFF (the unit sleeps; the ignition-OFF save runs) or on fail-safe
  (whose save runs at once, during the request)
- **AND** the fail-safe and explicit saves SHALL be unchanged

#### Scenario: Operator writes not deferred
- **WHEN** a gear change, a park-brake run or a setting on another card writes NVS while a request
  is active
- **THEN** the write SHALL happen as before

#### Scenario: Brake settings refused
- **WHEN** a brake setting is sent while a request is active
- **THEN** it SHALL be refused without a flash write

## REMOVED Requirements
### Requirement: Configurable Brake Travel Time and Hold Drive
**Reason**: The X7 actuator, its open-loop estimate and its hold drive are removed on the aztec
buggy; the iBooster reports its rod position.
**Migration**: Replaced by "Service Brake Settings" (max stroke, apply rate, release rate). The NVS
keys `brake`/`travel_ms` and `brake`/`hold_pwm` are removed at boot; there is no value migration.

### Requirement: Brake Control System
**Reason**: It described the removed X7 linear actuator: open-loop estimate, In5 endstop, hold drive,
and two scenarios that were never implemented (emergency braking to 100 %, a 30 % start-up brake).
**Migration**: Replaced by "Service Brake Control" and the iBooster requirements of this change. No
emergency-stop or start-up brake is added (the fail-safe is unchanged by owner decision).
