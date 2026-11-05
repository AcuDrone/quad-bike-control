## MODIFIED Requirements
### Requirement: Transmission State Persistence
The system SHALL persist the last confirmed transmission state (gear, servo position as float percent, validity flag) to NVS so that, when the physical gear sensor cannot tell the gear at initialisation, the servo can be restored to its last known position after a clean power cycle. The saved state SHALL be a fallback behind the sensor, never a reason to override it.

#### Scenario: Mark state invalid at start of gear change
- **WHEN** `TransmissionController::setGear()` starts a gear change
- **THEN** `state_valid=false` SHALL be written to NVS before the servo moves
- **AND** a mid-move power loss will therefore result in `state_valid=false` on next boot

#### Scenario: Save confirmed state on gear arrival
- **WHEN** a gear change RETURN phase completes
- **THEN** `state_valid=true`, `state_gear`, and `state_pct` (float percent) SHALL be written to NVS
- **AND** the saved state reflects the servo position at that moment

#### Scenario: Restore servo position and skip autohome on valid state
- **WHEN** the transmission is initialised, the physical gear reads `GEAR_UNKNOWN`
- **AND** NVS `state_valid=true`, `state_gear` is R, N, H or L and `state_pct` is within 0.0–100.0
- **THEN** the servo SHALL be commanded to `state_pct` directly (no overshoot)
- **AND** the expected gear SHALL become `state_gear`
- **AND** the system SHALL log `[TRANS] Initialised from NVS: <gear> at <pct>%`
- **AND** when the physical gear reads exactly one gear, NVS SHALL NOT be used (the sensor wins)

#### Scenario: Fall back to neutral default on invalid or missing state
- **WHEN** the transmission is initialised, the physical gear reads `GEAR_UNKNOWN`
- **AND** NVS `state_valid=false`, no state is saved, or the saved gear / position is implausible
- **THEN** the servo SHALL NOT jump directly to the neutral default position
- **AND** the transmission SHALL instead perform the initialisation step into NEUTRAL (see
  Transmission Gear Sync With the Physical Sensor), using the invalid record only as an
  overshoot-direction hint
- **AND** the initialisation step SHALL mark the state invalid without overwriting the stored
  `state_pct`, so a power loss during it leaves the same hint

#### Scenario: Resync and sensor initialisation save a valid state
- **WHEN** the expected gear is set from the sensor (initialisation or resync)
- **THEN** `state_valid=true`, `state_gear` = the sensor gear and `state_pct` = the current servo
  position SHALL be written to NVS

## ADDED Requirements
### Requirement: Transmission Gear Sync With the Physical Sensor
The expected gear (`TransmissionController::targetGear_`) SHALL be initialised from, and kept
reconciled with, the physical gear sensor. Until it is initialised the expected gear SHALL be
`GEAR_UNKNOWN`, which SHALL be the only meaning of an UNKNOWN expected gear. Initialisation SHALL
use, in order: the physical gear when it reads exactly one gear, else a valid NVS state, else a
real shift step into NEUTRAL; it SHALL never leave the transmission waiting for the sensor. The
choice of source, the initialisation-step overshoot direction and the resync decision SHALL live
in the header-only
`include/TransmissionLaw.h`, covered by the native suite `test_transmission_sync`.

#### Scenario: Not initialised at power-up
- **WHEN** the system starts
- **THEN** the expected gear and the outgoing gear SHALL be `GEAR_UNKNOWN`
- **AND** the transmission servo SHALL keep no pulse until initialisation commands it

#### Scenario: Initialisation on first engine running
- **WHEN** the transmission is not initialised and the engine is detected running
- **THEN** `VehicleController::update()` SHALL run the initialisation once

#### Scenario: Initialisation step into NEUTRAL
- **WHEN** initialisation runs, the physical gear reads `GEAR_UNKNOWN` and NVS holds no valid state
- **THEN** the system SHALL log `[TRANS] Init: sensor unknown, stepping to NEUTRAL`
- **AND** it SHALL perform a normal shift step to NEUTRAL (overshoot, overshoot dwell, return)
  through the gear-change state machine, so the gear-change boost or shift idle hold applies as for
  any step
- **AND** the expected gear SHALL be NEUTRAL from the start of the step, the outgoing gear
  `GEAR_UNKNOWN`
- **AND** the overshoot move SHALL use the fixed settle time `TRANS_INIT_SETTLE_MS` (1500 ms)
  instead of a travel time computed from the unknown servo position

#### Scenario: Initialisation step overshoot direction
- **WHEN** the initialisation step starts
- **THEN** with an NVS record whose `state_pct` is within 0.0–100.0, the overshoot direction SHALL be
  the sign of (NEUTRAL position − `state_pct`), so a lever on the R side overshoots toward H and a
  lever on the H/L side overshoots toward R
- **AND** when `state_pct` equals the NEUTRAL position, the sign of (NEUTRAL position − position of
  `state_gear`) SHALL be used
- **AND** with no record, a `state_pct` outside 0.0–100.0 or not a number, or no usable sign, the
  direction SHALL be +1 (toward H)

#### Scenario: Initialisation step completes
- **WHEN** the initialisation step's RETURN phase completes
- **THEN** with the physical gear reading NEUTRAL it SHALL complete like any step (state saved valid)
- **AND** with the physical gear still `GEAR_UNKNOWN` NEUTRAL SHALL be accepted as the working
  assumption, the state saved valid, and the system SHALL log
  `[TRANS] Init: sensor silent, assuming NEUTRAL`; the idle resync SHALL correct the expected gear
  if the sensor later reports another gear

#### Scenario: Gear requests during the initialisation step
- **WHEN** `setGear()` is called while the initialisation step runs
- **THEN** it SHALL behave as during any active step: the destination is queued and shifted to
  after the step completes

#### Scenario: Initialise from the sensor
- **WHEN** initialisation is tried and the physical gear reads exactly one gear
- **THEN** the expected gear SHALL become that gear, whatever NVS holds
- **AND** the servo SHALL be commanded directly (no overshoot) to that gear's configured position
- **AND** the system SHALL log `[TRANS] Initialised from sensor: <gear>`

#### Scenario: Throttle capped while not initialised
- **WHEN** the expected gear is `GEAR_UNKNOWN`
- **THEN** throttle from every source SHALL be capped at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (8.5 %),
  as for NEUTRAL or an active gear change

#### Scenario: Gear requests refused while not initialised
- **WHEN** `setGear()` is called while the expected gear is `GEAR_UNKNOWN`
- **THEN** it SHALL return false without commanding the servo, writing NVS or changing the queue
- **AND** a MAVLink gear request SHALL be retried by the existing retry loop until it is accepted

#### Scenario: Assumed gear reported as unknown before initialisation
- **WHEN** the expected gear is `GEAR_UNKNOWN`
- **THEN** `getTargetGearString()` and `getFromGearString()` SHALL return `"?"`, never `"N"`

#### Scenario: Resync after a confirmed mismatch
- **WHEN** the transmission is initialised, no gear change is active and no sequence is queued
- **AND** the idle check (every `TRANS_GEAR_CHECK_INTERVAL`, 500 ms) sees the physical gear read the
  same single gear, different from the expected gear, on `TRANS_RESYNC_CONFIRM_CHECKS` (2)
  consecutive checks
- **THEN** the expected gear SHALL become the physical gear and the state SHALL be saved valid with
  the current servo position
- **AND** the servo SHALL NOT be moved
- **AND** the system SHALL log `[TRANS] Resynced from sensor: target=<old> -> <new>`
- **AND** the first detection SHALL still log `[TRANS] MISMATCH: target=<old>, physical=<new>`

#### Scenario: No resync on a transient or unknown reading
- **WHEN** a check reads `GEAR_UNKNOWN`, the expected gear, or a different single gear than the
  previous mismatching check
- **THEN** the consecutive-mismatch count SHALL restart and no resync SHALL happen on that check
- **AND** starting a gear change SHALL reset the count
