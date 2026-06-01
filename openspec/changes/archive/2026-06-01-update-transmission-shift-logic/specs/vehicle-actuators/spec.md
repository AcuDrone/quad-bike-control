## MODIFIED Requirements

### Requirement: Transmission Servo Control
The transmission servo SHALL use an overshoot-dwell-return motion profile and SHALL enforce sequential gear shifting through the physical order `[R, N, H, L]`.

#### Scenario: Overshoot-and-dwell motion profile
- **WHEN** `setGear(target)` is called
- **THEN** the servo SHALL first move to the overshoot position (`targetPct ± overshootPct`)
- **AND** once servo movement time has elapsed, the controller SHALL enter an overshoot dwell of up to `TRANS_OVERSHOOT_DWELL_MS` milliseconds
- **AND** if the physical switch confirms the target gear during the dwell, the controller SHALL transition to the RETURN phase immediately (early exit)
- **AND** if `TRANS_OVERSHOOT_DWELL_MS` elapses and the switch IS confirmed, the controller SHALL transition to RETURN
- **AND** if `TRANS_OVERSHOOT_DWELL_MS` elapses and the switch is NOT confirmed, the controller SHALL transition to PULLBACK

#### Scenario: Return phase after confirmed gear engagement
- **WHEN** the overshoot dwell phase confirms gear engagement (switch active)
- **THEN** the servo SHALL move from the overshoot position back to `finalGearPositionPct_`
- **AND** once return movement time has elapsed, the gear change SHALL be marked complete
- **AND** `saveState()` SHALL be called with `state_valid=true`
- **AND** no additional switch check is performed in the RETURN phase

#### Scenario: Pullback dwell before retry
- **WHEN** overshoot dwell times out without switch confirmation
- **AND** `overshootRetryCount_ < 100`
- **THEN** the servo SHALL move away from the target by `pullbackPcts_[targetGear]`
- **AND** once pullback movement completes, the controller SHALL dwell for `TRANS_ROLLBACK_DWELL_MS` milliseconds
- **AND** after the dwell, `overshootRetryCount_` SHALL be incremented
- **AND** the controller SHALL retry OVERSHOOT with multiplier 1.5× (retry 1) or 2.0× (retry 2+)

#### Scenario: Confirm gear arrival via physical switch (updated)
- **WHEN** the overshoot dwell phase is active
- **AND** `getPhysicalGear()` returns the target gear
- **THEN** the RETURN phase SHALL start immediately without waiting for the full dwell period
- **AND** the gear change is considered confirmed

#### Scenario: Sequential gear shifting — step through sequence
- **WHEN** `setGear(finalGear)` is called
- **AND** `finalGear` is not adjacent to the current physical gear in the sequence `[R, N, H, L]`
- **THEN** the controller SHALL compute the immediate next step toward `finalGear`
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** the gear change SHALL start toward the immediate next step only

#### Scenario: Sequential gear shifting — advance queue on arrival
- **WHEN** a gear change completes (physical switch confirms)
- **AND** `queuedGear_ != GEAR_UNKNOWN`
- **AND** `physicalGear != queuedGear_`
- **THEN** the controller SHALL automatically start the next step toward `queuedGear_`
- **AND** steps SHALL continue until `physicalGear == queuedGear_`
- **AND** `queuedGear_` SHALL be cleared to `GEAR_UNKNOWN` when the final gear is reached

#### Scenario: Sequential shifting interrupted by new command
- **WHEN** `setGear(newFinalGear)` is called while a sequence is in progress
- **THEN** the in-flight gear change to the current intermediate target SHALL complete normally
- **AND** `queuedGear_` SHALL be updated to `newFinalGear`
- **AND** subsequent steps SHALL route toward `newFinalGear`

#### Scenario: Sequential shifting — unknown current gear
- **WHEN** `setGear(finalGear)` is called
- **AND** `getPhysicalGear()` returns `GEAR_UNKNOWN`
- **THEN** the controller SHALL route through `GEAR_NEUTRAL` first (safe intermediate)
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** once NEUTRAL is confirmed, normal sequencing toward `finalGear` resumes
