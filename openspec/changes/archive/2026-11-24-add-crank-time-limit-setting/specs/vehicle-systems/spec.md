## MODIFIED Requirements
### Requirement: Ignition State Control Integration
The system SHALL integrate ignition state control with vehicle systems coordination.

#### Scenario: Set ignition state via vehicle controller
- **WHEN** VehicleController.setIgnitionState() is called with state (OFF/ACC/IGNITION/START)
- **THEN** ignition state is passed to RelayController.setIgnitionState()
- **AND** ignition state change is logged
- **AND** vehicle state telemetry includes new ignition state

#### Scenario: Query current ignition state
- **WHEN** VehicleController.getIgnitionState() is called
- **THEN** current ignition state is returned from RelayController (OFF/ACC/IGNITION/CRANKING)
- **AND** state reflects actual relay configuration

#### Scenario: Monitor engine cranking completion
- **WHEN** VehicleController.update() is called each control loop
- **THEN** RelayController.update() SHALL be called with the validated engine RPM (see "Validated
  Engine RPM for Crank Control"): the CAN engine RPM when the CAN data is valid and 0 otherwise on
  the quad bike, the effective RPM (tacho first, 0 when no source is valid) on the aztec buggy
- **AND** cranking automatically stops when the engine starts (validated RPM ≥
  `ENGINE_RUNNING_RPM_THRESHOLD`)
- **AND** cranking automatically stops once the starter has run for the starter time limit of the
  attempt (see "Configurable Starter Time Limit"; 2–10 s, default 5 s = `CRANKING_TIMEOUT`)
- **AND** ignition state transitions from CRANKING to IGNITION after cranking completes

## ADDED Requirements
### Requirement: Configurable Starter Time Limit
The starter time limit — the longest the starter SHALL run in one start attempt when the engine is
not detected as running — SHALL be a persisted, operator-set, per-vehicle value in whole
milliseconds, range `CRANK_MAX_MS_MIN` (2000) to `CRANK_MAX_MS_MAX` (10000), default
`CRANKING_TIMEOUT` (5000). It SHALL be stored in the NVS namespace `"crank"`, key `max_ms`
(uint16), owned by `VehicleController`, loaded at boot and handed to `RelayController` through a
RAM-only setter. It SHALL bound every start attempt, whatever started it: the RC ignition ON
auto-crank (including the re-crank when RC control returns), the GCS `MAV_CMD_USER_2` ignition ON
and the web START command. `RelayController` SHALL latch the value in use when it enters CRANKING;
that attempt SHALL end on engine start (validated RPM ≥ `ENGINE_RUNNING_RPM_THRESHOLD`, checked
first, unchanged) or once the latched limit has elapsed, whichever comes first. Wherever another
requirement names `CRANKING_TIMEOUT` as the bound of a start attempt, the bound SHALL be the limit
in use; `CRANKING_TIMEOUT` is its default. The timeout decision SHALL be the pure function
`crankTimedOut()` in `include/CrankLaw.h` (wrap-safe), covered by native tests. The web relay-test
cap of the starter relay (`RELAY_TEST_STARTER_MAX_MS`, 2000 ms, ignition OFF only) is not a start
attempt and SHALL stay fixed and separate.

#### Scenario: Default on a unit without a stored value
- **WHEN** the unit boots with no `crank`/`max_ms` key
- **THEN** the starter time limit SHALL be 5000 ms, as before this change

#### Scenario: Attempt ends at the configured limit
- **WHEN** the limit is 7000 ms and a start attempt (RC ON, GCS ignition ON or web START) engages
  the starter and the validated RPM stays below `ENGINE_RUNNING_RPM_THRESHOLD`
- **THEN** the starter SHALL be released on the first control loop at or after 7000 ms of cranking
- **AND** the state SHALL settle in IGNITION and "Cranking timeout" SHALL be logged
- **AND** no new attempt SHALL start without a new request (no automatic retry)

#### Scenario: Engine start still ends the attempt early
- **WHEN** the validated RPM reaches `ENGINE_RUNNING_RPM_THRESHOLD` before the limit
- **THEN** the starter SHALL be released at once, whatever the limit

#### Scenario: A change during cranking applies to the next attempt
- **WHEN** a new limit is saved while the starter is cranking
- **THEN** it SHALL be persisted and become the value in use at once
- **AND** the running attempt SHALL keep the limit it latched when it entered CRANKING
- **AND** the next attempt SHALL use the new value

#### Scenario: Stored value out of range
- **WHEN** the stored `max_ms` is outside 2000–10000
- **THEN** it SHALL be logged and ignored, the default SHALL be used, and the stored value SHALL NOT
  be rewritten

#### Scenario: Relay test cap unchanged
- **WHEN** the starter relay is energized from the web relay test
- **THEN** it SHALL still be released after `RELAY_TEST_STARTER_MAX_MS` (2000 ms), whatever the
  starter time limit
