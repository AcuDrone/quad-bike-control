## ADDED Requirements

### Requirement: Steering Motor Telemetry
The telemetry broadcast SHALL include steering-motor health data sourced from the VESC, and the web interface SHALL display it, so operators can see steering motor current, temperature, and driver health.

#### Scenario: Include steering VESC data in telemetry JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** the payload SHALL include `steer_motor_current` (motor current in amps), `steer_fet_temp` (VESC FET temperature in °C), `steer_vesc_fault` (fault indicator), and `steer_driver_ok` (boolean VESC-link health)
- **AND** the additions SHALL keep the total telemetry message under 1 KB

#### Scenario: Populate steering VESC data from the driver
- **WHEN** telemetry collection is triggered
- **THEN** the steering motor current, FET temperature, fault code, and driver-ok flag SHALL be read from the VESC steering driver's most recent `COMM_GET_VALUES` poll

#### Scenario: Display steering motor telemetry in the web UI
- **WHEN** a telemetry message with steering VESC fields is received
- **THEN** the web UI SHALL display steering motor current (A) and FET temperature (°C)
- **AND** SHALL show a fault / driver-ok indicator (e.g. green when `steer_driver_ok` is true and no fault, red on fault or driver down)
- **AND** all new UI labels SHALL have i18n keys present in both the `en` and `uk` dictionaries with maintained parity
