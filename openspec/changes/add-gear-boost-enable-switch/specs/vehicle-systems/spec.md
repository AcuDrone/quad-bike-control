## MODIFIED Requirements
### Requirement: Throttle Boost During Gear Changes
The system SHALL use a PID controller to regulate engine RPM to a configurable target value (`TRANS_GEAR_BOOST_TARGET_RPM`, overridden by NVS namespace `"boost"` key `target_rpm`) for the duration of a gear change, overriding SBUS/web throttle commands while active. Whether a gear change runs the PID or only holds the throttle at idle SHALL be governed by a persisted per-vehicle switch, NVS namespace `"boost"` key `enabled` (bool), default true, loaded at start-up and written immediately when changed. The manual boost test (`test_boost`) SHALL NOT be governed by the switch.

#### Scenario: Activate PID boost when gear change starts
**Given** a gear change is initiated (transmission actuator starts moving toward a non-NEUTRAL gear)
**And** the gear-change boost switch is enabled
**When** `transmission_.needsThrottleBoost()` returns true
**Then** the gear boost PID SHALL be activated
**And** CAN RPM polling SHALL switch to fast mode (`CAN_POLL_INTERVAL_RPM_BOOST`)
**And** PID integral and derivative state SHALL be reset to zero

#### Scenario: PID holds target RPM during gear change
**Given** the gear boost PID is active
**And** CAN engine RPM data is valid
**When** `updateGearBoostPID()` is called each loop iteration
**Then** the PID error SHALL be `TRANS_GEAR_BOOST_TARGET_RPM - currentRPM`
**And** PID output SHALL be mapped to a throttle servo angle
**And** the throttle servo SHALL be set to the PID-computed angle, overriding SBUS/web commands
**And** the integral term SHALL be clamped to prevent wind-up

#### Scenario: PID output clamped to safe throttle range
**Given** the PID is active and computing output
**When** the computed angle exceeds `THROTTLE_MAX_ANGLE` or falls below `THROTTLE_MIN_ANGLE`
**Then** the output SHALL be clamped to `[THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE]`
**And** the integral term SHALL not accumulate further past the clamp boundary (anti-windup)

#### Scenario: Freeze throttle output when CAN data is stale
**Given** the gear boost PID is active
**And** CAN data is invalid or stale
**When** `updateGearBoostPID()` is called
**Then** the throttle SHALL hold its last computed angle
**And** the integral term SHALL NOT accumulate
**And** a warning SHALL be logged (rate-limited)

#### Scenario: Deactivate PID when gear change completes
**Given** the gear boost PID is active
**When** `transmission_.needsThrottleBoost()` returns false (actuator stopped at target gear)
**Then** the PID SHALL be deactivated
**And** CAN RPM polling SHALL revert to normal rate (`CAN_POLL_INTERVAL_RPM`)
**And** throttle control SHALL return to the current SBUS/web command on the next update cycle

#### Scenario: Safety timeout releases PID boost
**Given** the gear boost PID is active
**When** `TRANS_GEAR_BOOST_TIMEOUT` milliseconds elapse since activation
**Then** the PID SHALL be forcibly deactivated
**And** throttle SHALL return to SBUS/web command
**And** a warning SHALL be logged: "[BOOST] Timeout, releasing gear boost PID"

#### Scenario: PID deactivated if engine is not running
**Given** a gear change is initiated
**And** CAN engine RPM is below `ENGINE_RUNNING_RPM_THRESHOLD`
**When** `updateGearBoostPID()` is called
**Then** the PID SHALL NOT activate (engine not running — boosting throttle is unsafe)
**And** the throttle SHALL remain at idle

#### Scenario: Default switch state on a unit without a stored value
- **WHEN** the system starts and `boost/enabled` has never been written
- **THEN** the gear-change boost SHALL be enabled, exactly as before this switch existed

#### Scenario: Stored switch state is restored
- **WHEN** the system starts and `boost/enabled` holds false
- **THEN** the gear-change boost SHALL be disabled until it is switched on again

#### Scenario: Idle hold instead of boost on a gear change while disabled
- **WHEN** the gear-change boost switch is disabled
- **AND** `transmission_.needsThrottleBoost()` returns true
- **THEN** the gear boost PID SHALL NOT be activated and the CAN RPM poll rate SHALL NOT change
- **AND** the throttle SHALL be held at idle for as long as `needsThrottleBoost()` stays true,
  whether or not CAN data is valid and whatever the engine RPM
- **AND** MAVLink throttle SHALL NOT be applied, the web throttle demand SHALL NOT re-open the
  throttle, and a web `throttle` command SHALL be rejected with "Gear change active", exactly as
  while the PID is active (one shared shift lock: PID active or idle hold)
- **AND** the gear-change sequence, its interlocks and the overshoot dwell SHALL be unchanged

#### Scenario: Idle hold released when the gear change completes
- **WHEN** the idle hold is active and `transmission_.needsThrottleBoost()` returns false
- **THEN** the hold SHALL end with the throttle set to idle and the stored web throttle demand zeroed
- **AND** MAVLink throttle SHALL be applied again from the next update cycle

#### Scenario: Disabling during an active gear-change boost
- **WHEN** the switch is turned off while the gear boost PID is active for a gear change
- **THEN** the PID SHALL be released as on gear-change completion: CAN RPM polling SHALL revert to
  `CAN_POLL_INTERVAL_RPM`, the throttle SHALL be set to idle, `gearBoostActive_` SHALL become false
  and the stored web throttle demand SHALL be zeroed
- **AND** if `needsThrottleBoost()` is still true, the idle hold SHALL continue until it goes false

#### Scenario: Enabling during a gear change
- **WHEN** the switch is turned on while the idle hold is active
- **THEN** the idle hold SHALL continue until `needsThrottleBoost()` goes false
- **AND** the gear boost PID SHALL NOT be started mid-dwell; it SHALL run from the next gear change

#### Scenario: Manual boost test ignores the switch
- **WHEN** the switch is disabled and the manual boost test (`test_boost` true) is started
- **THEN** the gear boost PID SHALL run toward the target RPM exactly as when the switch is enabled
