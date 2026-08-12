## ADDED Requirements

### Requirement: 24V Boost Rail Control and Monitoring
The system SHALL enable the external 12V→24V boost converter at boot and SHALL monitor the resulting
rail voltage, because that rail powers the steering VESC, the Jetson, Starlink and the cameras.

#### Scenario: Enable the boost rail at boot
- **WHEN** `setup()` begins
- **THEN** `PIN_BOOST_EN` (GPIO46) SHALL be configured as an output and driven HIGH as the first
  hardware action, before any subsystem initialization
- **AND** the enable SHALL occur before the steering VESC UART is initialized, since the VESC is
  powered from this rail
- **AND** the board's 100K pulldown on GPIO46 SHALL keep the rail off through reset, so no enable
  glitch occurs during boot

#### Scenario: Sample the rail voltage
- **WHEN** `RAIL_SAMPLE_INTERVAL_MS` (500 ms) elapses
- **THEN** `PIN_ADC_24V` (GPIO9, ADC1) SHALL be sampled and averaged over 8 readings
- **AND** the rail voltage SHALL be computed as `Vadc × RAIL_24V_DIVIDER_RATIO` (9.33, from the
  board's 200K/24K divider)
- **AND** the result SHALL be published to telemetry
- **AND** sampling SHALL NOT block the control loop

#### Scenario: Warn on a low rail
- **WHEN** the computed rail voltage is below `RAIL_24V_LOW_THRESHOLD` (21.0 V)
- **AND** more than `BOOST_STARTUP_GRACE_MS` (1000 ms) has elapsed since the boost was enabled
- **THEN** a low-rail warning flag SHALL be raised and published in telemetry
- **AND** a rate-limited warning SHALL be logged
- **AND** the system SHALL NOT automatically disable any subsystem — dropping the rail would remove
  steering authority

#### Scenario: Suppress warnings during startup ramp
- **WHEN** the boost has been enabled for less than `BOOST_STARTUP_GRACE_MS`
- **THEN** the low-rail warning SHALL be suppressed while the converter ramps
- **AND** the measured voltage SHALL still be reported in telemetry

#### Scenario: Report boost enable state
- **WHEN** telemetry is collected
- **THEN** the commanded boost enable state SHALL be reported alongside the measured rail voltage
- **AND** an enabled boost reporting a rail well below threshold SHALL be distinguishable from a
  deliberately disabled boost

### Requirement: Degraded Operation on Board I/O Fault
The system SHALL define safe, observable degraded behavior when the opto-input expander or the
relay expander becomes unavailable, since the gear interlock and the ignition/starter path now
depend on I2C.

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
- **THEN** `VehicleController::isBrakeReleased()` SHALL return `false` ("not confirmed released")
- **AND** brake retraction SHALL remain bounded by `BRAKE_SENSOR_OVERRUN_TIME` and
  `BRAKE_FULL_TRAVEL_TIME` exactly as when the endstop has not yet been reached
- **AND** the system SHALL NOT report the brake as released on the basis of a failed read

#### Scenario: Relay expander unavailable
- **WHEN** the relay expander failed to initialize or is faulted
- **THEN** ignition, starter, front light and wheel lock commands SHALL become logging no-ops
- **AND** the reported ignition state SHALL be `OFF`
- **AND** steering, throttle, brake, MAVLink and web telemetry SHALL continue to operate
- **AND** the fault SHALL be visible in telemetry

#### Scenario: Faults recover without a reboot
- **WHEN** a faulted expander begins responding again
- **THEN** the affected subsystem SHALL resume normal operation automatically
- **AND** the recovery SHALL be logged
- **AND** the telemetry fault flag SHALL clear

## MODIFIED Requirements

### Requirement: Brake Control System
The system SHALL control braking force through a linear actuator-driven brake mechanism. The
"fully released" endstop SHALL be read from the opto-isolated brake limit sensor (In5) through the
input expander rather than from a dedicated GPIO.

#### Scenario: Set brake position by percentage
- **WHEN** BrakeSystem.setPosition() is called with value from 0 to 100
- **THEN** brake actuator extends proportionally
- **AND** 0% maps to fully released brakes
- **AND** 100% maps to maximum braking force

#### Scenario: Release brakes
- **WHEN** BrakeSystem.release() is called
- **THEN** brake actuator retracts to 0% position
- **AND** brakes are confirmed released

#### Scenario: Detect the released endstop via the input expander
- **WHEN** `VehicleController::isBrakeReleased()` is called
- **THEN** it SHALL read the brake limit sensor state (In5) from the `BoardInputs` snapshot
- **AND** it SHALL NOT perform a `digitalRead()` or issue its own I2C transaction
- **AND** the added latency (up to `BOARD_INPUT_POLL_MS`, 25 ms) SHALL be acceptable against the
  `BRAKE_SENSOR_OVERRUN_TIME` (1000 ms) overrun window

#### Scenario: Emergency braking
- **WHEN** BrakeSystem.emergencyStop() is called
- **THEN** brakes are immediately applied to 100%
- **AND** throttle is forced to idle
- **AND** transmission remains in current gear
- **AND** emergency braking SHALL NOT depend on the input expander being healthy

#### Scenario: Brake hold on startup
- **WHEN** system powers on or resets
- **THEN** brakes are automatically applied to 30% (parking brake)
- **AND** brakes remain engaged until explicitly released by user command
