## MODIFIED Requirements
### Requirement: 24V Boost Rail Control and Monitoring
The system SHALL enable the external 12V→24V boost converter at boot and SHALL monitor the resulting
rail voltage, because that rail powers the steering VESC, the Jetson, Starlink and the cameras.

#### Scenario: Enable the boost rail at boot
- **WHEN** `setup()` begins
- **THEN** `PIN_BOOST_EN` (GPIO46) SHALL be configured as an output and driven HIGH as the first
  hardware action, before any subsystem initialization
- **AND** the enable SHALL occur before the CAN controller and the steering VESC driver are
  initialized, since the VESC is powered from this rail
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
