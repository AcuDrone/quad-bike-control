## ADDED Requirements
### Requirement: 12 V Battery Calibration via Web Interface
The web portal SHALL let the operator calibrate the 12 V battery sense against a multimeter: the
operator types the measured voltage and the firmware computes, validates, stores and applies the
correction factor.

#### Scenario: Calibrate from a multimeter reading
- **WHEN** the operator enters the measured battery voltage (number input, step 0.01) and presses
  "Калібрувати" / "Calibrate"
- **THEN** a `cal_bat_v` command SHALL be sent with `value` = the measured volts
- **AND** the firmware SHALL compute `cal = measured / raw` from the CURRENT averaged raw reading
- **AND** SHALL accept it only when the measured value is within 8.0–16.0 V, the raw reading is
  above 5 V (a battery is connected) and the factor is within 0.80–1.20
- **AND** on acceptance SHALL persist the factor to NVS (`BAT12_CAL_NVS_NS` "bat12", key "cal")
  BEFORE applying it, apply it immediately and reply with the new factor and the corrected voltage
- **AND** on any refusal or a failed NVS write SHALL keep the previous factor and reply with an error
  stating the reason
- **AND** the command SHALL be accepted regardless of the active input source

#### Scenario: Reset the calibration
- **WHEN** the operator presses "Скинути" / "Reset"
- **THEN** a `reset_bat_v_cal` command SHALL be sent and the firmware SHALL store and apply a factor
  of 1.0

#### Scenario: Load the calibration at boot
- **WHEN** the firmware boots
- **THEN** the stored factor SHALL be loaded; an absent, NaN or out-of-range (0.80–1.20) value SHALL
  be ignored and 1.0 used, without rewriting NVS

#### Scenario: Boost command on a vehicle without a boost board
- **WHEN** a `set_boost` command is received
- **THEN** it SHALL be recognised (not "unknown command"), SHALL NOT change any output, and SHALL be
  answered and logged as "boost rail not fitted"

## REMOVED Requirements
### Requirement: 24V Boost Rail Control via Web Interface
**Reason**: aztec has no Buck-Boost board on X10; there is no rail to switch.
**Migration**: the UI button is removed; `set_boost` is still recognised and answered "not fitted"
(see "12 V Battery Calibration via Web Interface").
