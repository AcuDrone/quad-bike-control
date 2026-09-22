## ADDED Requirements

### Requirement: Odometer and Trip Distance Accumulation
The system SHALL accumulate the distance the wheel has actually turned into two counters held in
`SpeedSensor`: an **odometer** (`odo_mm`), which SHALL only ever increase over the life of the
vehicle, and a **trip meter** (`trip_mm`), which SHALL accumulate on the same terms until an
operator explicitly resets it. Both SHALL be held as `uint64_t` **millimetres**, so that the
accumulation is exact, cannot overflow in any physical scenario, and is independent of the wheel
calibration in force when it is later read.

Distance SHALL be derived from counted pulses only. In `SpeedSensor::update()`, the increment
SHALL be `delta × distancePerPulseMm_` applied inside the `delta > 0` branch — the same branch and
the same arithmetic that already produce the speed reading — and SHALL be rounded rather than
truncated, so that repeated sample windows do not bias the total low. Direction SHALL NOT be
considered: distance driven in reverse SHALL add exactly as distance driven forward does.

#### Scenario: Accumulate distance from counted pulses
- **WHEN** a sample window is evaluated and a non-zero, plausible pulse `delta` is read from the
  PCNT unit
- **THEN** `delta × distancePerPulseMm_`, rounded to the nearest millimetre, SHALL be added to
  BOTH `odo_mm` and `trip_mm`
- **AND** the addition SHALL happen in the same `delta > 0` branch that computes the speed, so the
  two readings can never disagree about whether the wheel moved

#### Scenario: The decayed estimate contributes no distance
- **WHEN** a sample window sees no edges and the sensor emits its decayed speed estimate, or the
  reading has been latched suspicious after an implausible pulse loss
- **THEN** NEITHER counter SHALL be incremented
- **AND** the resulting under-count during a sensor dropout SHALL be accepted as the correct
  failure direction, because an odometer that keeps counting on a disconnected sensor hides the
  fault, while `isValid()` / `isSuspicious()` already report it

#### Scenario: A rejected wrap-guard sample contributes no distance
- **WHEN** a sample window reads a delta greater than `SPEED_MAX_PULSES_PER_SAMPLE` and is
  discarded as a counter glitch rather than as motion
- **THEN** NEITHER counter SHALL be incremented
- **AND** this SHALL follow from the accumulation being placed AFTER the existing wrap guard's
  early return, not from a second check

#### Scenario: Reverse counts the same as forward
- **WHEN** the vehicle is driven in reverse
- **THEN** both counters SHALL increase, exactly as they do when driving forward
- **AND** NEITHER counter SHALL EVER decrease, because the sensor is unidirectional and a car
  odometer measures distance travelled, not net displacement

#### Scenario: Recalibration does not rewrite recorded distance
- **WHEN** `speed_cal_ppr` or `speed_cal_circ` changes the millimetres-per-pulse at runtime
- **THEN** the already-accumulated `odo_mm` and `trip_mm` SHALL be left untouched
- **AND** only distance travelled AFTER the change SHALL use the new millimetres-per-pulse, so each
  kilometre stays frozen at the calibration that measured it

#### Scenario: The odometer cannot be reset
- **WHEN** any command, web request or MAVLink message is processed
- **THEN** there SHALL be NO code path that zeroes or decreases `odo_mm`
- **AND** a trip reset SHALL leave `odo_mm` exactly as it was

#### Scenario: Expose both counters to the vehicle layer
- **WHEN** the vehicle layer or the telemetry layer asks for the recorded distance
- **THEN** `SpeedSensor` SHALL expose the odometer and the trip distance both in exact millimetres
  and as kilometres in float (`mm × 1e-6`)
- **AND** the float form SHALL be understood as a presentation of the counter, never as the counter
  itself — it SHALL NOT be read back into, re-accumulated from, or used to reconstruct `odo_mm` or
  `trip_mm`

### Requirement: Odometer and Trip Persistence
The system SHALL persist both distance counters across power loss in the EXISTING NVS namespace
`"speed"` — the namespace `SpeedSensor` already owns — under the `uint64` keys `odo_mm` and
`trip_mm`. No new namespace, no new configurable key, and no runtime-tunable write policy SHALL be
introduced: the write interval SHALL be the hardcoded constant `ODO_NVS_WRITE_INTERVAL_MM`
(1 000 000 mm = 1 km) in `Constants.h`.

Writes SHALL occur on exactly three triggers, chosen so that a normal shutdown loses nothing and a
power cut loses at most one kilometre, while the flash write budget stays negligible against the
partition's wear-levelled endurance.

#### Scenario: Load both counters on startup
- **WHEN** `SpeedSensor::begin()` is called
- **THEN** `odo_mm` and `trip_mm` SHALL be read from NVS namespace `"speed"`
- **AND** a key that is absent SHALL read as 0 rather than as an error
- **AND** the restored values SHALL be logged, so the operator can confirm the counters survived
  the power cycle

#### Scenario: Write once per kilometre of odometer growth
- **WHEN** `odo_mm` has grown by at least `ODO_NVS_WRITE_INTERVAL_MM` since the last write
- **THEN** both keys SHALL be written and the "last written" mark SHALL be advanced to the current
  `odo_mm`
- **AND** the loss from an unexpected power cut SHALL therefore be bounded by one kilometre

#### Scenario: Write on ignition OFF
- **WHEN** the ignition state transitions to OFF, on the MAVLink path or the web path, or the
  vehicle enters fail-safe
- **THEN** both keys SHALL be written immediately, so a normal shutdown loses no distance at all
- **AND** the write SHALL happen on the TRANSITION only, not repeatedly while ignition is held OFF

#### Scenario: Write immediately on a trip reset
- **WHEN** a trip reset is performed
- **THEN** `trip_mm` SHALL be zeroed in RAM and both keys SHALL be written immediately
- **AND** `odo_mm` SHALL be written with its unchanged value, so a power cycle cannot resurrect the
  cleared trip

#### Scenario: A failed NVS write never blocks the control loop
- **WHEN** the NVS namespace cannot be opened for writing
- **THEN** the failure SHALL be logged and the call SHALL return
- **AND** the in-RAM counters SHALL remain correct and SHALL continue accumulating
- **AND** the firmware SHALL NOT retry in a loop, block, or reset

#### Scenario: The write schedule stays inside the flash write budget
- **WHEN** the write policy is evaluated over the vehicle's life
- **THEN** it SHALL average no more than one write per kilometre driven plus one per ignition cycle
- **AND** it SHALL NOT write on a wall-clock timer, on every sample window, or while the vehicle is
  stationary
