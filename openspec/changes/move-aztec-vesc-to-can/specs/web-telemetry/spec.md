## MODIFIED Requirements
### Requirement: Steering Motor Telemetry
The telemetry broadcast SHALL include steering-motor health data sourced from the VESC, and the web interface SHALL display it, so operators can see steering motor current, temperature, and driver health.

#### Scenario: Include steering VESC data in telemetry JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** the payload SHALL include `steer_motor_current` (motor current in amps), `steer_fet_temp` (VESC FET temperature in °C), `steer_vesc_fault` (fault indicator), `steer_vesc_fault_stale` (boolean, true when the last fault-code poll got no valid reply) and `steer_driver_ok` (boolean VESC-link health)
- **AND** the additions SHALL keep the total telemetry message under 1 KB

#### Scenario: Populate steering VESC data from the driver
- **WHEN** telemetry collection is triggered
- **THEN** the steering motor current SHALL be read from the VESC driver's latest `CAN_PACKET_STATUS` frame, the FET temperature from its latest `CAN_PACKET_STATUS_4` frame, the fault code and its stale flag from its latest fault-code poll, and the driver-ok flag from the driver's STATUS_1 timeout

#### Scenario: Display steering motor telemetry in the web UI
- **WHEN** a telemetry message with steering VESC fields is received
- **THEN** the web UI SHALL display steering motor current (A) and FET temperature (°C)
- **AND** SHALL show a fault / driver-ok indicator (e.g. green when `steer_driver_ok` is true and no fault, red on fault or driver down), labelled "VESC (CAN)"
- **AND** SHALL mark the fault code as not refreshed while `steer_vesc_fault_stale` is true and the driver is ok
- **AND** all new UI labels SHALL have i18n keys present in both the `en` and `uk` dictionaries with maintained parity

### Requirement: Steering VESC Input Voltage Telemetry
The telemetry broadcast SHALL include the steering VESC's measured input voltage and a low-voltage
flag computed in the firmware against the compile-time constant `STEER_VIN_LOW_V` (default 21.5 V,
chosen to suit both a 6s NMC and a 7s LiFePO4 steering pack and adjustable per build), and SHALL
include a 12 V low-voltage flag computed against `ECU_VOLTAGE_LOW_V` (default 12.0 V, lead-acid starter battery), so the web
page never needs a copy of either threshold.

#### Scenario: Include the VESC input voltage
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `steer_vin`, the input voltage from `SteeringController::getInputVoltage()`
  (the input voltage from the VESC's `CAN_PACKET_STATUS_5` broadcast) in volts with one decimal
- **AND** it SHALL include `steer_vin_low`, true only when `steer_driver_ok` is true and `steer_vin`
  is below `STEER_VIN_LOW_V`

#### Scenario: Include the 12 V low flag
- **WHEN** telemetry data is formatted as JSON and CAN status is "connected"
- **THEN** it SHALL include `ecu_voltage_low`, true when the ECU module voltage is below
  `ECU_VOLTAGE_LOW_V`
- **WHEN** CAN status is not "connected"
- **THEN** `ecu_voltage_low` SHALL be omitted along with the other CAN vehicle-data fields

#### Scenario: Show the VESC voltage tile
- **WHEN** the "Кермо: стан" card renders a telemetry frame
- **THEN** a "Напруга VESC" / "VESC voltage" tile SHALL show `steer_vin` in volts
- **AND** it SHALL show `--` while `steer_driver_ok` is false or the key is missing
- **AND** it SHALL use the warning colour while `steer_vin_low` is true

#### Scenario: Thresholds are firmware constants
- **WHEN** the firmware is built
- **THEN** `STEER_VIN_LOW_V` and `ECU_VOLTAGE_LOW_V` SHALL be defined in `include/Constants.h`
- **AND** `data/index.html` SHALL NOT contain either threshold value
