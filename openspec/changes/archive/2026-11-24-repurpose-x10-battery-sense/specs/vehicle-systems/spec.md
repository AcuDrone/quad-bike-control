## ADDED Requirements
### Requirement: 12 V Battery Sense on X10
On aztec (no Buck-Boost board fitted) the system SHALL measure the 12 V lead-acid battery wired to
X10 pin 3 through the on-board 200K/24K divider to GPIO9 (`PIN_ADC_24V`, ADC1), apply a user
calibration factor, and use the result as the vehicle's 12 V / "ECU voltage" measurement. The math
SHALL live in a pure header (`include/BatteryVoltage.h`) covered by native unit tests.

#### Scenario: GPIO46 unchanged
- **WHEN** `setup()` begins
- **THEN** `PIN_BOOST_EN` (GPIO46) SHALL still be configured as an output and driven HIGH as the
  first hardware action, exactly as before (harmless with nothing on X10 pin 1)

#### Scenario: Sample the battery
- **WHEN** `BAT12_SAMPLE_INTERVAL_MS` (500 ms) elapses
- **THEN** GPIO9 SHALL be sampled with 11 dB attenuation and averaged over 8 readings
- **AND** the raw voltage SHALL be `Vadc × BAT12_DIVIDER_RATIO` (9.33) and the reported voltage
  `raw × cal`
- **AND** sampling SHALL NOT block the control loop

#### Scenario: Nothing connected
- **WHEN** the raw voltage is at or below `BAT12_PRESENT_MIN_V` (5 V)
- **THEN** the battery sense SHALL be reported invalid and no low warning SHALL be raised

#### Scenario: Low battery with hysteresis
- **WHEN** the calibrated voltage falls below `BAT12_LOW_V` (11.8 V)
- **THEN** the low flag SHALL be set and a rate-limited warning logged
- **AND** it SHALL clear only once the voltage rises above `BAT12_LOW_CLEAR_V` (12.1 V)
- **AND** no subsystem SHALL be disabled automatically (warning only)

#### Scenario: Cranking sag suppressed
- **WHEN** the starter relay is energized (crank or relay test), and for `BAT12_CRANK_HOLDOFF_MS`
  (2 s) after it releases
- **THEN** the low flag SHALL be suppressed while the voltage is still measured and reported

## REMOVED Requirements
### Requirement: 24V Boost Rail Control and Monitoring
**Reason**: aztec has no Buck-Boost board; the steering VESC runs from its own 7s LiFePO4 pack and
X10 pin 3 now carries the 12 V battery.
**Migration**: replaced by "12 V Battery Sense on X10"; `RAIL_24V_LOW_THRESHOLD`,
`BOOST_STARTUP_GRACE_MS` and the low-rail warning are removed; GPIO46 is still driven HIGH.
