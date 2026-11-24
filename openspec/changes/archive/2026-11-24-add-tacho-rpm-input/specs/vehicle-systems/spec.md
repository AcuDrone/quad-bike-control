## ADDED Requirements
### Requirement: Effective Engine RPM Source Selection
The vehicle layer SHALL compose one EFFECTIVE engine RPM from the tacho input and the CAN
`VehicleData`, together with a source `rpm_source` ∈ {TACHO, CAN, NONE}, a validity flag and the
timestamp of the sample it came from. Every engine-RPM consumer — `isEngineRunning()` (MAVLink gear
gate, transmission init, throttle-calibration and gear-code refusals), the relay controller's starter
cut-off and crank interlock, the web START interlock, the gear-change boost PID, the engine hour
meter, web telemetry and MAVLink `EFI_STATUS` — SHALL read the effective RPM and SHALL NOT read
`VehicleData.engineRPM` directly. The selection SHALL be a pure function in `include/TachoLaw.h`.

#### Scenario: Live tacho wins
- **WHEN** the tacho is fitted and an accepted edge arrived within `TACHO_TIMEOUT_MS`
- **THEN** the effective RPM SHALL be the tacho RPM, source TACHO, valid
- **AND** this SHALL hold whether or not CAN `VehicleData` is valid, and whatever RPM CAN reports

#### Scenario: Silent tacho with a CAN RPM reading falls back to CAN
- **WHEN** the tacho is fitted but timed out
- **AND** CAN `VehicleData` is valid with `engineRPM` > 0
- **THEN** the effective RPM SHALL be the CAN RPM, source CAN, so that a broken tacho wire on an
  ECU that does report RPM can never make a running engine look stopped

#### Scenario: Silent fitted tacho reads a stopped engine
- **WHEN** the tacho is fitted and timed out
- **AND** CAN is invalid or reports 0 RPM
- **THEN** the effective RPM SHALL be 0, source TACHO, valid

#### Scenario: Tacho not fitted
- **WHEN** the tacho is not fitted
- **THEN** the effective RPM SHALL be the CAN RPM with source CAN while CAN `VehicleData` is valid
- **AND** SHALL be invalid with source NONE while CAN `VehicleData` is invalid, exactly as the
  behaviour before this change

#### Scenario: Engine-running predicate
- **WHEN** `isEngineRunning()` is evaluated
- **THEN** it SHALL return true only when the effective RPM is valid and at or above
  `ENGINE_RUNNING_RPM_THRESHOLD`

### Requirement: Engine-Running RPM Threshold
`ENGINE_RUNNING_RPM_THRESHOLD` SHALL be 800 RPM on the aztec buggy, whose warm idle is ≈1500 RPM:
above the starter cranking speed and safely below idle, so that every engine-running decision
recognises an idling engine.

#### Scenario: Idle counts as running
- **WHEN** the engine idles at ≈1500 RPM, or a cold or loaded idle sags to 1000 RPM
- **THEN** `isEngineRunning()` SHALL return true, so MAVLink gear changes are accepted and the
  crank interlock refuses the starter

#### Scenario: Cranking does not count as running
- **WHEN** the starter turns the engine at cranking speed (≈250–400 RPM) without it firing
- **THEN** the effective RPM SHALL stay below the threshold and the starter SHALL keep cranking until
  the engine fires or `CRANKING_TIMEOUT` elapses

#### Scenario: Engine stop is seen promptly
- **WHEN** the engine stops with the tacho as source
- **THEN** the effective RPM SHALL read 0 within `TACHO_TIMEOUT_MS` (0.5 s) of the last pulse, and
  `isEngineRunning()` SHALL become false

## MODIFIED Requirements
### Requirement: Ignition Safety Interlocks
The system SHALL enforce safety interlocks for ignition state changes to prevent unsafe operations.

#### Scenario: Require brake applied before ignition state change
- **WHEN** ignition state change is requested (to ACC, IGNITION, or START)
- **AND** current state is OFF
- **THEN** brake position is checked (must be >= 20%)
- **AND** if brake insufficient, ignition change is rejected
- **AND** error message is logged: "Apply brake before ignition"
- **AND** ignition remains in OFF state

#### Scenario: Allow ignition OFF without brake requirement
- **WHEN** ignition state change to OFF is requested
- **THEN** change is allowed regardless of brake position
- **AND** ignition state changes to OFF immediately

#### Scenario: Prevent cranking if engine already running
- **WHEN** START ignition state is requested
- **AND** the EFFECTIVE engine RPM (tacho when fitted, CAN otherwise) is valid and >=
  `ENGINE_RUNNING_RPM_THRESHOLD` (800 RPM on aztec)
- **THEN** START command is rejected
- **AND** error message is logged: "Engine already running"
- **AND** ignition remains in current state

#### Scenario: Allow cranking when engine not running
- **WHEN** START ignition state is requested
- **AND** the effective engine RPM is below `ENGINE_RUNNING_RPM_THRESHOLD` (800 RPM), or no RPM
  source is valid
- **AND** brake is applied (>= 20%)
- **THEN** ignition state changes to CRANKING (same relay config as IGNITION)
- **AND** cranking timer starts (5-second timeout)
- **AND** cranking monitors the effective RPM for engine start detection

#### Scenario: Allow ignition transitions between ACC and IGNITION freely
- **WHEN** ignition state change is requested between ACC and IGNITION
- **THEN** change is allowed without brake or RPM checks
- **AND** only initial power-on from OFF requires safety checks

#### Scenario: Stop the starter when the engine catches
- **WHEN** the starter is engaged (CRANKING) and the effective engine RPM reaches
  `ENGINE_RUNNING_RPM_THRESHOLD` (800 RPM)
- **THEN** the relay controller SHALL release the starter and settle into IGNITION
- **AND** the threshold SHALL sit above the starter's cranking speed (≈250–400 RPM) and below the
  engine's warm idle (≈1500 RPM), so that a started engine is recognised before it reaches idle and
  a cranking-only engine is never mistaken for a running one

#### Scenario: Suppress an armed crank on a running engine
- **WHEN** a crank is armed and waiting for the pre-crank dwell
- **AND** the effective engine RPM is at or above `ENGINE_RUNNING_RPM_THRESHOLD`
- **THEN** the crank SHALL be suppressed and the ignition SHALL settle into IGNITION

### Requirement: Throttle Boost During Gear Changes
The system SHALL use a PID controller to regulate the EFFECTIVE engine RPM (tacho when fitted, CAN
otherwise) to a configurable target value (`TRANS_GEAR_BOOST_TARGET_RPM`, overridden by NVS namespace `"boost"` key `target_rpm`) for the duration of a gear change, overriding SBUS/web throttle commands while active. Whether a gear change runs the PID or only holds the throttle at idle SHALL be governed by a persisted per-vehicle switch, NVS namespace `"boost"` key `enabled` (bool), default true, loaded at start-up and written immediately when changed. The manual boost test (`test_boost`) SHALL NOT be governed by the switch.

#### Scenario: Activate PID boost when gear change starts
**Given** a gear change is initiated (transmission actuator starts moving toward a non-NEUTRAL gear)
**And** the gear-change boost switch is enabled
**When** `transmission_.needsThrottleBoost()` returns true
**Then** the gear boost PID SHALL be activated
**And** CAN RPM polling SHALL switch to fast mode (`CAN_POLL_INTERVAL_RPM_BOOST`)
**And** PID integral and derivative state SHALL be reset to zero

#### Scenario: PID holds target RPM during gear change
**Given** the gear boost PID is active
**And** the effective engine RPM is valid
**When** `updateGearBoostPID()` is called each loop iteration
**Then** the PID error SHALL be `TRANS_GEAR_BOOST_TARGET_RPM - currentRPM`, where `currentRPM` is the
effective engine RPM
**And** a new PID step SHALL be computed only when the effective RPM carries a new sample timestamp
(a new CAN reply, or a new accepted tacho edge)
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
**And** no effective RPM source is valid (CAN invalid or stale and no live tacho)
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
**And** the effective engine RPM is invalid or below `ENGINE_RUNNING_RPM_THRESHOLD`
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

### Requirement: Engine Hour Meter
The system SHALL accumulate the wall-clock time the ENGINE HAS ACTUALLY BEEN TURNING into a single
total counter — an engine hour meter — held in a dedicated `EngineHourMeter` owned by the vehicle
layer. The counter SHALL be a whole-second `uint64_t`, with the sub-second remainder carried
between updates so that no time is lost to truncation, and SHALL be exposed in hours as a float
(`seconds / 3600`) for presentation only.

The total SHALL be a vehicle-lifetime counter on the same terms as the odometer: it SHALL only ever
increase, and there SHALL be NO command, parameter, magic value, web control or constant that
zeroes or decreases it.

Time SHALL accrue from elapsed-time deltas measured against the millisecond clock, never from a
fixed per-iteration tick, so that the total tracks real time regardless of the control loop's
actual rate. A single delta that is implausibly large SHALL be discarded rather than accumulated,
so that a stalled main loop, a long blocking operation or a `millis()` wrap cannot inject time the
engine did not run.

#### Scenario: Count only while the engine is turning
- **WHEN** the vehicle layer updates the meter and the EFFECTIVE engine RPM is valid (a live tacho
  when fitted, or valid CAN `VehicleData`) AND it is at or above `ENGINE_HOURS_MIN_RPM`
- **THEN** the elapsed milliseconds since the previous update SHALL be added to the meter
- **AND** the threshold SHALL be low enough to count a NORMAL IDLE — an hour meter that only counts
  above a driving RPM would under-report the wear that idling causes — and SHALL be a constant
  SEPARATE from `ENGINE_RUNNING_RPM_THRESHOLD`, which is a gear-change safety threshold chosen for
  a different purpose and may sit above idle
- **AND** whole seconds SHALL be carried into the counter while the sub-second remainder is
  retained, so that a sequence of short updates accumulates exactly as one long one would

#### Scenario: Do not count while the engine is stopped or the CAN data is invalid
- **WHEN** no effective RPM source is valid (CAN `VehicleData` invalid or stale and no fitted
  tacho), or the effective engine RPM is below `ENGINE_HOURS_MIN_RPM` (a fitted tacho that has
  timed out reads a valid 0 RPM)
- **THEN** NO time SHALL be added to the meter for that interval
- **AND** the elapsed time SHALL be discarded rather than banked, so that resuming valid RPM data
  does not credit the meter with the silent interval
- **AND** the meter SHALL NOT fall back to any assumed or inferred engine state: an unknown engine
  state SHALL NOT invent running time, on the same principle by which the odometer ignores the
  decayed speed estimate

#### Scenario: Discard an implausibly long update interval
- **WHEN** the measured interval between two updates exceeds `ENGINE_HOURS_MAX_DELTA_MS`
- **THEN** that interval SHALL be discarded entirely rather than accumulated
- **AND** the meter SHALL resume counting from the current instant, so a stalled loop, a blocking
  operation or a `millis()` wrap costs at most the discarded interval and can never add time

#### Scenario: Survive a power cycle
- **WHEN** the firmware starts
- **THEN** the meter SHALL be loaded from its own NVS namespace, a key that has never been written
  reading as 0 rather than as an error
- **AND** the restored value SHALL be logged, so the operator can confirm it survived
- **AND** the meter SHALL be written back whenever it has grown by
  `ENGINE_HOURS_NVS_WRITE_INTERVAL_S` seconds since the last write, and immediately on an ignition
  OFF transition or on entry into fail-safe, so a normal shutdown loses nothing and an unexpected
  power cut loses at most that interval
- **AND** the write interval SHALL be a hardcoded constant: no new config key, no web-settable
  parameter and no runtime tuning SHALL be introduced

#### Scenario: A failed NVS write never blocks the control loop
- **WHEN** the NVS namespace cannot be opened for writing, or the write fails
- **THEN** the failure SHALL be logged and the call SHALL return
- **AND** the in-RAM counter SHALL remain correct and SHALL continue accumulating
- **AND** the firmware SHALL NOT retry in a loop, block, or reset

#### Scenario: The total cannot be reset
- **WHEN** any MAVLink command, web command or configuration write is processed
- **THEN** there SHALL be NO code path that zeroes or decreases the TOTAL engine-seconds counter,
  the only assignments to it being the NVS load at startup and the accumulation itself
- **AND** the only way back to zero SHALL be an erase of the NVS partition
- **AND** the trip reset SHALL be the one operation that zeroes anything, and it SHALL leave the
  total untouched

#### Scenario: Expose the meter to the telemetry and MAVLink layers
- **WHEN** the telemetry layer or the MAVLink layer asks for the accumulated running time
- **THEN** the vehicle controller SHALL expose it both in exact seconds and as hours in float
- **AND** the float form SHALL be understood as a presentation of the counter, never as the counter
  itself — it SHALL NOT be read back into, re-accumulated from, or used to reconstruct the seconds
- **AND** the existing `isEngineRunning()` predicate, the odometer, the trip meter and the speed
  sensor SHALL be left untouched by the meter
