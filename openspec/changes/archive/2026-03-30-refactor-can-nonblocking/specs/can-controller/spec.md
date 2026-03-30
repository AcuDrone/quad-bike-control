## MODIFIED Requirements
### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the MCP2515 shall be initialized at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM/speed: 100ms, temperatures: 1000ms)
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

#### Scenario: PID scheduling prioritizes most overdue requests
**Given** multiple PIDs are due for polling
**When** the state machine enters IDLE
**Then** the PID with the largest overdue time SHALL be selected first
**And** high-frequency PIDs (RPM, speed at 100ms) naturally receive priority over low-frequency PIDs (temps at 1000ms)

### Requirement: CAN Communication Error Handling
The system SHALL detect and recover from CAN communication errors without blocking the main control loop.

#### Scenario: Handle OBD-II response timeout
**Given** an OBD-II request has been sent
**When** 200ms elapses without a matching response
**Then** the per-PID retry counter shall be incremented
**And** the state machine shall transition to IDLE to process the next PID
**And** the main loop SHALL NOT be blocked during the timeout period
**And** a warning shall be logged to Serial

#### Scenario: Detect stale CAN data
**Given** CAN data was previously valid
**When** 5000ms elapses without successful updates
**Then** `VehicleData.dataValid` shall be set to `false`
**And** dependent systems shall use fallback behavior

#### Scenario: Recover from MCP2515 hardware error
**Given** the MCP2515 reports a hardware error
**When** the error is detected
**Then** the controller shall attempt to reset the MCP2515
**And** retry up to 3 times before marking CAN as failed
**And** log error details to Serial

#### Scenario: Drain MCP2515 receive buffers
**Given** the state machine is in WAITING_RESPONSE
**When** CAN messages are available
**Then** up to 5 messages SHALL be read per `update()` call to prevent MCP2515 RX buffer overflow
**And** non-matching messages SHALL be discarded
**And** the first matching response SHALL be accepted and parsed
