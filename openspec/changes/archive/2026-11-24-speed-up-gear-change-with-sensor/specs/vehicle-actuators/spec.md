## MODIFIED Requirements
### Requirement: Transmission Servo Control
The transmission servo SHALL use an overshoot-dwell-return motion profile and SHALL enforce sequential gear shifting through the physical order `[R, N, H, L]`.

#### Scenario: Overshoot-and-dwell motion profile
- **WHEN** `setGear(target)` is called
- **THEN** the servo SHALL first move to the overshoot position (`targetPct ± overshootPct`)
- **AND** once servo movement time has elapsed, the controller SHALL enter the overshoot dwell
- **AND** the dwell SHALL end as soon as the physical gear has read the step's target gear continuously for `TRANS_DWELL_CONFIRM_MS` (200 ms), measured from the first confirming read
- **AND** if that does not happen, the dwell SHALL end after `TRANS_OVERSHOOT_DWELL_MS` (2000 ms, the timeout) and the gear SHALL be assumed engaged regardless of physical sensor state
- **AND** in both cases the controller SHALL transition directly to the RETURN phase

#### Scenario: Return phase after assumed gear engagement
- **WHEN** the overshoot dwell ends (sensor-confirmed or timed out)
- **THEN** the servo SHALL move from the overshoot position back to `finalGearPositionPct_`
- **AND** once return movement time has elapsed, the gear change SHALL be marked complete
- **AND** `saveState()` SHALL be called with `state_valid=true`

#### Scenario: Sequential gear shifting — step through sequence
- **WHEN** `setGear(finalGear)` is called
- **AND** `finalGear` is not adjacent to the current gear in the sequence `[R, N, H, L]`
- **THEN** the controller SHALL compute the immediate next step toward `finalGear`
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** the gear change SHALL start toward the immediate next step only

#### Scenario: Sequential gear shifting — advance queue after step completes
- **WHEN** a gear change RETURN phase completes
- **AND** `queuedGear_ != GEAR_UNKNOWN`
- **AND** `targetGear_ != queuedGear_`
- **THEN** the controller SHALL pause, then automatically start the next step toward `queuedGear_` using `targetGear_` as the current position
- **AND** the pause SHALL end as soon as the physical gear has read `targetGear_` continuously for `TRANS_DWELL_CONFIRM_MS`, or after `TRANS_SEQUENCE_STEP_DWELL_MS` (1000 ms, the timeout)
- **AND** steps SHALL continue until `targetGear_ == queuedGear_`
- **AND** `queuedGear_` SHALL be cleared to `GEAR_UNKNOWN` when the final gear is reached

#### Scenario: Sequential shifting interrupted by new command
- **WHEN** `setGear(newFinalGear)` is called while a sequence is in progress
- **THEN** the in-flight gear change to the current intermediate target SHALL complete normally
- **AND** `queuedGear_` SHALL be updated to `newFinalGear`
- **AND** subsequent steps SHALL route toward `newFinalGear`

#### Scenario: Sequential shifting — unknown current gear
- **WHEN** `setGear(finalGear)` is called
- **AND** `targetGear_` is `GEAR_UNKNOWN` (the transmission has not been initialised from the sensor or NVS)
- **THEN** `setGear()` SHALL return false
- **AND** the servo SHALL NOT be commanded, NVS SHALL NOT be written and `queuedGear_` SHALL be unchanged
- **AND** there SHALL be no routing through `GEAR_NEUTRAL` from an unknown gear
