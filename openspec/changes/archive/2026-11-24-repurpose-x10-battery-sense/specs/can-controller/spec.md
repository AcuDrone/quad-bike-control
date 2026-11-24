## MODIFIED Requirements
### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.
The RPM poll interval SHALL be adjustable at runtime via `setRPMPollInterval(uint32_t ms)` to allow higher-frequency polling during gear changes.
The active poll table SHALL contain six PIDs: engine RPM (`0x0C`) in the fast class, and coolant temperature (`0x05`), throttle position (`0x11`), manifold absolute pressure (`0x0B`), intake air temperature (`0x0F`) and calculated engine load (`0x04`) in the `CAN_POLL_INTERVAL_TEMP` class. Control module voltage (`0x42`) SHALL NOT be polled, because the Delphi MT05 ECU never answers it; the 12 V supply is measured by the board on X10 instead. Exactly one OBD-II request SHALL be in flight at any time, and the RPM poll cadence SHALL remain unchanged by the additional PIDs.
The OBD-II state machine SHALL run in the main loop and SHALL receive its replies from the CAN receive queue and send its requests through the queued CAN transmit, never by accessing the MCP2515 directly.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the CAN task SHALL be created and SHALL initialize the MCP2515 at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM: `CAN_POLL_INTERVAL_RPM` ms; coolant, throttle position, MAP, intake air temperature and engine load: `CAN_POLL_INTERVAL_TEMP` ms)
**And** the state machine shall be set to IDLE
**And** `begin()` SHALL wait for the result of that first initialization for at most `CAN_BOOT_INIT_WAIT_MS` (1000 ms) and return `true` if it succeeded, `false` otherwise
**And** a `false` result SHALL NOT stop later initialization attempts (see "MCP2515 Initialization Retry")

#### Scenario: Non-blocking OBD-II polling via state machine
**Given** the CAN controller is initialized
**When** `update()` is called
**Then** if state is IDLE and a PID is due for polling, an OBD-II request SHALL be queued for transmission and state transitions to WAITING_RESPONSE
**And** if state is WAITING_RESPONSE, the frames taken from the receive queue in this call SHALL be checked for a matching response
**And** if a matching response is available, it SHALL be parsed and stored, and state transitions to IDLE
**And** if the response timeout (200ms) elapses, the request SHALL be marked as failed and state transitions to IDLE
**And** `update()` SHALL return immediately without blocking the main loop, and SHALL NOT access the MCP2515 over SPI

#### Scenario: Read engine RPM via OBD-II PID 0x0C
**Given** the CAN controller is initialized
**When** the RPM PID becomes due (current interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0C shall be sent
**And** the response shall be parsed to extract RPM value
**And** RPM shall be calculated as `((A*256)+B)/4`
**And** the value shall be stored in `VehicleData.engineRPM`

#### Scenario: Read engine coolant temperature via OBD-II PID 0x05
**Given** the CAN controller is initialized
**When** the coolant temp PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x05 shall be sent
**And** the response shall be parsed to extract temperature value
**And** temperature shall be calculated as `A - 40` (°C)
**And** the value shall be stored in `VehicleData.coolantTemp`

#### Scenario: Read throttle position via OBD-II PID 0x11
**Given** the CAN controller is initialized
**When** the throttle position PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x11 shall be sent
**And** throttle position shall be calculated as `A * 100 / 255` (percentage, 0–100%)
**And** the value shall be stored in `VehicleData.throttlePosition`

#### Scenario: Read manifold absolute pressure via OBD-II PID 0x0B
**Given** the CAN controller is initialized
**When** the MAP PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0B shall be sent
**And** the response shall be parsed to extract manifold absolute pressure
**And** pressure shall be calculated as `A` (kPa absolute, 0–255 kPa)
**And** the value shall be stored in `VehicleData.mapKpa`

#### Scenario: Read control module voltage via OBD-II PID 0x42
**Given** the Delphi MT05 ECU never answers OBD-II PID 0x42
**When** the poll table is constructed
**Then** PID 0x42 SHALL NOT be present in the poll table and `VehicleData` SHALL carry no module voltage field
**And** PID 0x42 SHALL remain in the on-demand ECU capability probe candidate list, decoded as `((A*256)+B)/1000` V

#### Scenario: Read intake air temperature via OBD-II PID 0x0F
**Given** the CAN controller is initialized
**When** the intake air temperature PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0F shall be sent
**And** temperature shall be calculated as `A - 40` (°C, -40 to +215)
**And** the value shall be stored in `VehicleData.intakeTemp`

#### Scenario: Read calculated engine load via OBD-II PID 0x04
**Given** the CAN controller is initialized
**When** the engine load PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x04 shall be sent
**And** engine load shall be calculated as `A * 100 / 255` (percentage, 0–100%)
**And** the value shall be stored in `VehicleData.engineLoad`

#### Scenario: PIDs confirmed unsupported by the ECU are not polled
**Given** the 2026-08-14 ECU capability probe confirmed that fuel tank level (`0x2F`) and engine oil temperature (`0x5C`) produce no response
**When** the poll table is constructed
**Then** neither `0x2F` nor `0x5C` SHALL be present in the poll table
**And** no request slot SHALL be spent on them
**And** PIDs that the probe reported as supported but that have no consumer (for example `0x0E` timing advance, `0x14` O2 sensor B1S1, and the fuel-trim PIDs) SHALL also be omitted, to keep the request budget small

#### Scenario: Additional PIDs do not disturb the RPM poll cadence
**Given** the poll table contains six PIDs with RPM in the fast class and five PIDs at `CAN_POLL_INTERVAL_TEMP`
**When** the scheduler runs in steady state
**Then** the RPM PID SHALL continue to be polled at its configured interval (`CAN_POLL_INTERVAL_RPM`, or `CAN_POLL_INTERVAL_RPM_BOOST` during a gear change)
**And** an RPM request SHALL be delayed by at most one in-flight transaction (bounded by the 200 ms response timeout)
**And** the added PIDs SHALL NOT cause `VehicleData.dataValid` to be lost through the stale-data timeout

#### Scenario: PID scheduling prioritizes most overdue requests
**Given** multiple PIDs are due for polling
**When** the state machine enters IDLE
**Then** the PID with the largest overdue time SHALL be selected first
**And** high-frequency PIDs (RPM at reduced interval) naturally receive priority over low-frequency PIDs (temps at 1000ms)

#### Scenario: Change RPM poll interval at runtime
**Given** the CAN controller is initialized
**When** `setRPMPollInterval(50)` is called
**Then** the RPM PID entry interval SHALL be updated to 50ms
**And** subsequent RPM polls SHALL occur at the new rate
**When** `setRPMPollInterval(CAN_POLL_INTERVAL_RPM)` is called to restore normal rate
**Then** the RPM PID entry interval SHALL revert to the default value
