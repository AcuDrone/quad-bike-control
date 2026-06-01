## MODIFIED Requirements

### Requirement: Transmission Servo Control
The transmission servo SHALL use an overshoot-dwell-return motion profile and SHALL enforce sequential gear shifting through the physical order `[R, N, H, L]`.

#### Scenario: Overshoot-and-dwell motion profile
- **WHEN** `setGear(target)` is called
- **THEN** the servo SHALL first move to the overshoot position (`targetPct ± overshootPct`)
- **AND** once servo movement time has elapsed, the controller SHALL enter an overshoot dwell of exactly `TRANS_OVERSHOOT_DWELL_MS` milliseconds
- **AND** after `TRANS_OVERSHOOT_DWELL_MS` elapses, the gear SHALL be assumed engaged regardless of physical sensor state
- **AND** the controller SHALL transition directly to the RETURN phase

#### Scenario: Return phase after assumed gear engagement
- **WHEN** the overshoot dwell period elapses
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
- **THEN** the controller SHALL automatically start the next step toward `queuedGear_` using `targetGear_` as the current position
- **AND** steps SHALL continue until `targetGear_ == queuedGear_`
- **AND** `queuedGear_` SHALL be cleared to `GEAR_UNKNOWN` when the final gear is reached

#### Scenario: Sequential shifting interrupted by new command
- **WHEN** `setGear(newFinalGear)` is called while a sequence is in progress
- **THEN** the in-flight gear change to the current intermediate target SHALL complete normally
- **AND** `queuedGear_` SHALL be updated to `newFinalGear`
- **AND** subsequent steps SHALL route toward `newFinalGear`

#### Scenario: Sequential shifting — unknown current gear
- **WHEN** `setGear(finalGear)` is called
- **AND** `targetGear_` is `GEAR_UNKNOWN` or no prior change has completed
- **THEN** the controller SHALL route through `GEAR_NEUTRAL` first (safe intermediate)
- **AND** `queuedGear_` SHALL be set to `finalGear`
- **AND** once NEUTRAL step completes, normal sequencing toward `finalGear` resumes

## REMOVED Requirements

### Requirement: Pullback dwell before retry (removed)
The pullback-and-retry cycle (PULLBACK and PULLBACK_DWELL phases, `overshootRetryCount_`,
`pullbackPcts_`, `TRANS_ROLLBACK_DWELL_MS`) is removed. Gear engagement is determined
solely by time elapsed in OVERSHOOT_DWELL.

### Requirement: Sensor-gated early exit from dwell (removed)
The physical sensor is no longer used to exit OVERSHOOT_DWELL early. The dwell always
runs for the full `TRANS_OVERSHOOT_DWELL_MS` duration.
