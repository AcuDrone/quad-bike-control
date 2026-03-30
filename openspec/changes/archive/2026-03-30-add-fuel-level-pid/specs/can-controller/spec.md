## MODIFIED Requirements
### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the MCP2515 shall be initialized at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM/speed: 100ms, temperatures/fuel: 1000ms)
**And** the state machine shall be set to IDLE
**And** return `true` if successful, `false` otherwise

#### Scenario: Non-blocking OBD-II polling via state machine
**Given** the CAN controller is initialized
**When** `update()` is called
**Then** if state is IDLE and a PID is due for polling, an OBD-II request SHALL be sent and state transitions to WAITING_RESPONSE
**And** if state is WAITING_RESPONSE, a single non-blocking `checkReceive()` SHALL be performed
**And** if a matching response is available, it SHALL be parsed and stored, and state transitions to IDLE
**And** if the response timeout (200ms) elapses, the request SHALL be marked as failed and state transitions to IDLE
**And** `update()` SHALL return immediately without blocking the main loop

#### Scenario: Read engine RPM via OBD-II PID 0x0C
**Given** the CAN controller is initialized
**When** the RPM PID becomes due (100ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0C shall be sent
**And** the response shall be parsed to extract RPM value
**And** RPM shall be calculated as `((A*256)+B)/4`
**And** the value shall be stored in `VehicleData.engineRPM`

#### Scenario: Read vehicle speed via OBD-II PID 0x0D
**Given** the CAN controller is initialized
**When** the speed PID becomes due (100ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0D shall be sent
**And** the response shall be parsed to extract speed value
**And** speed shall be stored in `VehicleData.vehicleSpeed` (km/h)

#### Scenario: Read engine coolant temperature via OBD-II PID 0x05
**Given** the CAN controller is initialized
**When** the coolant temp PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x05 shall be sent
**And** the response shall be parsed to extract temperature value
**And** temperature shall be calculated as `A - 40` (°C)
**And** the value shall be stored in `VehicleData.coolantTemp`

#### Scenario: Read fuel tank level via OBD-II PID 0x2F
**Given** the CAN controller is initialized
**When** the fuel level PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x2F shall be sent
**And** the response shall be parsed to extract fuel level
**And** fuel level shall be calculated as `A * 100 / 255` (percentage, 0–100%)
**And** the value shall be stored in `VehicleData.fuelLevel`

#### Scenario: PID scheduling prioritizes most overdue requests
**Given** multiple PIDs are due for polling
**When** the state machine enters IDLE
**Then** the PID with the largest overdue time SHALL be selected first
**And** high-frequency PIDs (RPM, speed at 100ms) naturally receive priority over low-frequency PIDs (temps, fuel at 1000ms)
