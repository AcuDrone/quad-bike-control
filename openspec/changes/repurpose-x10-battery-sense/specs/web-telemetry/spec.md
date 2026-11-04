## ADDED Requirements
### Requirement: 12 V Battery Telemetry
The telemetry broadcast SHALL include the 12 V battery measured on X10 / GPIO9, unconditionally
(independent of CAN and MAVLink status), and the web interface SHALL show it in a "АКБ 12 В" /
"12 V battery" tile in the ignition card.

#### Scenario: Format battery data as JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** `"ecu_voltage"` SHALL carry the calibrated battery voltage in volts with two decimals
  (the key name is kept for the UI)
- **AND** `"ecu_voltage_low"` SHALL carry the low flag as a boolean
- **AND** `"bat_v_raw"` SHALL carry the voltage before calibration (V, two decimals), `"bat_v_cal"`
  the calibration factor (four decimals) and `"bat_v_valid"` whether a battery is seen on X10
- **AND** these fields SHALL be emitted whatever `can_status` is
- **AND** `"rail_24v"`, `"boost_on"` and `"rail_low"` SHALL NOT be emitted

#### Scenario: Show the battery tile
- **WHEN** a telemetry frame with `bat_v_valid` true is received
- **THEN** the tile SHALL show `ecu_voltage` in volts, in the warning colour while `ecu_voltage_low`
  is true and the success colour otherwise
- **AND** a small line SHALL show "raw X.XX V × cal 1.000" from `bat_v_raw` and `bat_v_cal`
- **WHEN** `bat_v_valid` is false
- **THEN** the tile SHALL read "не підключено" / "not connected"
- **AND** every label SHALL have i18n keys in both the `en` and `uk` dictionaries

#### Scenario: No boost rail UI
- **WHEN** the web page is loaded
- **THEN** it SHALL NOT show a 24 V boost rail button or a 24 V rail tile

## REMOVED Requirements
### Requirement: 24V Rail Telemetry
**Reason**: aztec has no Buck-Boost board; X10 pin 3 now measures the 12 V battery.
**Migration**: `rail_24v` / `boost_on` / `rail_low` are replaced by `ecu_voltage`, `ecu_voltage_low`,
`bat_v_raw`, `bat_v_cal`, `bat_v_valid` ("12 V Battery Telemetry").

## MODIFIED Requirements
### Requirement: ECU Electrical and Air Telemetry
The system SHALL include intake air temperature and calculated engine load in the CAN vehicle-data
telemetry, and the web interface SHALL display them in the CAN telemetry card, so operators can see
intake air conditions and the ECU's own load estimate alongside the other engine parameters. The
`ecu_voltage` field is no longer a CAN value: it carries the 12 V battery measured on X10 (see
"12 V Battery Telemetry") and is emitted outside the CAN-gated block.

#### Scenario: Include ECU voltage, intake temperature and engine load in telemetry JSON
- **WHEN** a telemetry update is broadcast
- **AND** CAN status is "connected"
- **THEN** the telemetry JSON SHALL include an `intake_temp` field carrying the intake air
  temperature in °C
- **AND** SHALL include an `engine_load` field carrying the calculated engine load as a percentage
  (0–100)
- **WHEN** CAN status is not "connected"
- **THEN** both fields SHALL be omitted alongside the other CAN vehicle-data fields, rather than
  being emitted as zeros
- **AND** `ecu_voltage` SHALL NOT be part of the CAN-gated block

#### Scenario: Populate the fields from CAN vehicle data
- **WHEN** telemetry collection is triggered
- **AND** the CAN `VehicleData` is valid
- **THEN** the values SHALL be copied from the most recent `VehicleData` intake temperature and
  engine load readings
- **WHEN** the CAN `VehicleData` is not valid
- **THEN** the fields SHALL be reset to their zero defaults and the CAN-gated block SHALL be
  omitted from the payload

#### Scenario: Display the new values in the CAN telemetry card
- **WHEN** a telemetry message containing `intake_temp` and `engine_load` is received
- **THEN** the CAN telemetry card SHALL display intake air temperature in °C and engine load as a
  percentage
- **AND** the values SHALL update in real time as new telemetry arrives
- **AND** the CAN card SHALL NOT show a module voltage tile (the battery tile lives in the
  ignition card)
- **WHEN** `can_status` is not "connected"
- **THEN** both readouts SHALL show a placeholder (e.g. `--°C`, `--%`) rather than a stale or
  misleading value

#### Scenario: New UI labels have full i18n parity
- **WHEN** the new CAN-card labels are added to the web UI
- **THEN** each label SHALL use a `data-i18n` key
- **AND** every new key SHALL be present in BOTH the `en` and `uk` dictionaries with maintained
  parity (no key defined in only one dictionary)

#### Scenario: Additions stay within the telemetry size budget
- **WHEN** the three fields are added to the steady-state telemetry payload
- **THEN** the serialized JSON document SHALL remain within the configured
  `StaticJsonDocument` capacity, including the peak case where the transient ECU probe object is
  also present
- **AND** the steady-state payload growth SHALL be bounded by scalar fields (no arrays or nested
  objects are introduced)
- **AND** the 5 Hz broadcast cadence SHALL be unaffected

### Requirement: Diagnostics Attention Summary
The Diagnostics section SHALL start with an "Увага" / "Attention" box that lists the problems
active in the latest telemetry frame, derived only from existing telemetry fields, and SHALL be
hidden when no problem is active.

#### Scenario: Sensorless steering
- **WHEN** a frame has `steer_sensor_ok` false
- **THEN** the box SHALL list "steering sensor unhealthy, jog/nudge run open loop without stall
  protection" in the danger colour

#### Scenario: Steering in rate mode
- **WHEN** a frame has `steer_mode` equal to `rate`
- **THEN** the box SHALL list "Кермо без датчика: режим швидкості" / "Steering without sensor:
  rate mode" in the danger colour, in addition to the sensorless steering line
- **AND** the line SHALL disappear on the first frame whose `steer_mode` is not `rate`

#### Scenario: Steering VESC down or faulted
- **WHEN** a frame has `steer_driver_ok` false
- **THEN** the box SHALL list "steering driver down" in the danger colour
- **WHEN** `steer_driver_ok` is true and `steer_vesc_fault` is nonzero
- **THEN** the box SHALL list a VESC fault line with the fault code in the danger colour

#### Scenario: Steering limit beyond the safe range
- **WHEN** a frame has `steer_range_warning` true
- **THEN** the box SHALL list "a steering limit is beyond ±550°" in the warning colour

#### Scenario: MAVLink inactive
- **WHEN** a frame has `mav_active` false
- **THEN** the box SHALL list "MAVLink inactive, autopilot commands are not arriving" in the warning
  colour

#### Scenario: Board I/O fault
- **WHEN** a frame has `io_input_fault` or `io_relay_fault` true
- **THEN** the box SHALL list a board I/O fault line naming the faulted expander(s) in the danger
  colour

#### Scenario: Low steering VESC input voltage
- **WHEN** a frame has `steer_vin_low` true and `steer_driver_ok` true
- **THEN** the box SHALL list "Напруга VESC низька (xx.x В)" / "VESC voltage low (xx.x V)" with the
  value of `steer_vin`, in the warning colour
- **AND** while `steer_driver_ok` is false no voltage line SHALL be listed, because there is no
  valid VESC data

#### Scenario: Low 12 V ECU voltage
- **WHEN** a frame has `bat_v_valid` true and `ecu_voltage_low` true, whatever `can_status` is
- **THEN** the box SHALL list "Напруга 12 В низька (xx.x В)" / "12 V voltage low (xx.x V)" with the
  value of `ecu_voltage`, in the warning colour
- **WHEN** `bat_v_valid` is false
- **THEN** no 12 V voltage line SHALL be listed

#### Scenario: No ECU link
- **WHEN** a frame has `can_status` not equal to `connected`
- **THEN** the box SHALL list "Немає зв'язку з ЕБК" / "No ECU link" in the warning colour
- **AND** the 12 V voltage line SHALL still be listed when its own condition holds, because the
  battery is measured by the board, not by the ECU


#### Scenario: Board boost rail not used
- **WHEN** a telemetry frame is received
- **THEN** there SHALL be no boost rail line in the attention box: `rail_24v` / `rail_low` are no
  longer emitted (aztec has no boost board)

#### Scenario: Lines are localized text
- **WHEN** any line is listed
- **THEN** it SHALL be localized (en/uk) and inserted as text, not HTML

#### Scenario: Nothing to report
- **WHEN** none of the conditions is active
- **THEN** the box SHALL be hidden

#### Scenario: Condition clears
- **WHEN** a condition that was listed is no longer active in a later frame
- **THEN** its line SHALL disappear on that frame without a page reload

### Requirement: Steering VESC Input Voltage Telemetry
The telemetry broadcast SHALL include the steering VESC's measured input voltage and a low-voltage
flag computed in the firmware against the compile-time constant `STEER_VIN_LOW_V` (default 21.5 V,
chosen to suit both a 6s NMC and a 7s LiFePO4 steering pack and adjustable per build), and SHALL
include a 12 V low-voltage flag computed by the X10 battery sense against `BAT12_LOW_V` /
`BAT12_LOW_CLEAR_V` (11.8 V / 12.1 V hysteresis, lead-acid starter battery), so the web page never
needs a copy of either threshold.

#### Scenario: Include the VESC input voltage
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `steer_vin`, the input voltage from `SteeringController::getInputVoltage()`
  (the VESC `COMM_GET_VALUES` input voltage) in volts with one decimal
- **AND** it SHALL include `steer_vin_low`, true only when `steer_driver_ok` is true and `steer_vin`
  is below `STEER_VIN_LOW_V`

#### Scenario: Include the 12 V low flag
- **WHEN** telemetry data is formatted as JSON, whatever the CAN status
- **THEN** it SHALL include `ecu_voltage_low`, the battery sense's low flag (set below
  `BAT12_LOW_V`, cleared above `BAT12_LOW_CLEAR_V`, suppressed while cranking), false while
  `bat_v_valid` is false

#### Scenario: Show the VESC voltage tile
- **WHEN** the "Кермо: стан" card renders a telemetry frame
- **THEN** a "Напруга VESC" / "VESC voltage" tile SHALL show `steer_vin` in volts
- **AND** it SHALL show `--` while `steer_driver_ok` is false or the key is missing
- **AND** it SHALL use the warning colour while `steer_vin_low` is true

#### Scenario: Thresholds are firmware constants
- **WHEN** the firmware is built
- **THEN** `STEER_VIN_LOW_V`, `BAT12_LOW_V` and `BAT12_LOW_CLEAR_V` SHALL be defined in
  `include/Constants.h`
- **AND** `data/index.html` SHALL NOT contain either threshold value
