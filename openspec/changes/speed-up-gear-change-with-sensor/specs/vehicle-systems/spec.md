## MODIFIED Requirements
### Requirement: Transmission Control System
The transmission control system SHALL enforce the physical gear sequence `[R, N, H, L]` and SHALL govern overshoot and rollback dwell timing via constants in `Constants.h`.

#### Scenario: Gear change routes through sequence
- **WHEN** a gear change command is received (S-bus or web)
- **AND** the target gear is not adjacent to the current gear in sequence `[R, N, H, L]`
- **THEN** the vehicle controller SHALL pass the final target gear to `TransmissionController::setGear()`
- **AND** intermediate gear changes SHALL be handled automatically by `TransmissionController`
- **AND** the telemetry SHALL report the current physical gear at each step

#### Scenario: Constants govern dwell timing
- **WHEN** `TRANS_OVERSHOOT_DWELL_MS`, `TRANS_SEQUENCE_STEP_DWELL_MS` or `TRANS_DWELL_CONFIRM_MS` are modified in `Constants.h`
- **THEN** the overshoot-dwell timeout, the inter-step pause timeout and the sensor confirmation window SHALL reflect the updated values without additional code changes

#### Scenario: Physical gear from the direct-GPIO switches
- **WHEN** `TransmissionController::getPhysicalGear()` reads the gear switches
- **THEN** it SHALL decode them as specified in "Quad Bike One-Line-Per-Gear Sensing"
- **AND** the aztec 3-bit gear code table SHALL NOT be used on this board
- **AND** every reading SHALL come straight from `digitalRead()` (an ambiguous idle reading is
  re-read immediately, at most `TRANS_GEAR_READ_RETRY_COUNT` times), so a stale cached gear can never
  count as a sensor confirmation during a gear change or a queued sequence pause

## ADDED Requirements
### Requirement: Sensor-Confirmed Gear-Change Waits
Both waits of a gear change — the overshoot dwell and the pause between sequence steps — SHALL
end as soon as the physical gear sensor confirms the expected gear, and SHALL keep their fixed
durations only as timeouts, so a shift is never slower than before and is faster whenever the
sensor works. The confirmation decision SHALL live in the header-only `include/TransmissionLaw.h`,
covered by the native suite `test_transmission_sync`.

#### Scenario: Continuous confirmation
- **WHEN** a wait is in progress and the physical gear reads the expected gear
- **THEN** the confirmation window SHALL start at that first matching read
- **AND** the wait SHALL end when the gear has read the expected gear on every read for
  `TRANS_DWELL_CONFIRM_MS` (200 ms)
- **AND** any other reading (another gear or `GEAR_UNKNOWN`) SHALL restart the window

#### Scenario: Timeout keeps today's behaviour
- **WHEN** the sensor never confirms (disconnected, stuck or reading another gear)
- **THEN** the overshoot dwell SHALL end after `TRANS_OVERSHOOT_DWELL_MS` (2000 ms) and the pause
  after `TRANS_SEQUENCE_STEP_DWELL_MS` (1000 ms), exactly as before this change

#### Scenario: Boost and idle hold follow the shorter dwell
- **WHEN** the overshoot dwell ends early on confirmation
- **THEN** `needsThrottleBoost()` SHALL become false at the same moment (it stays true exactly
  while the phase is `OVERSHOOT_DWELL`), releasing the boost PID or the shift idle hold

#### Scenario: Wait outcome is logged
- **WHEN** a wait ends
- **THEN** a TRANSMISSION debug line SHALL say whether it was confirmed (with the elapsed ms) or
  timed out
- **AND** when a single step or a whole sequence completes, its total duration SHALL be logged;
  telemetry and the web UI SHALL be unchanged
