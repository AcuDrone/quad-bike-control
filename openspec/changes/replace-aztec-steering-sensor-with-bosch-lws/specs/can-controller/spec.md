## MODIFIED Requirements
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
**And** every reset or re-initialization SHALL re-apply both acceptance paths (OBD-II window and steering-angle frame)

#### Scenario: Drain MCP2515 receive buffers
**Given** the CAN controller is initialized, in any OBD-II state (IDLE or WAITING_RESPONSE) or in probe mode
**When** CAN messages are available
**Then** up to 5 messages SHALL be read per `update()` call to prevent MCP2515 RX buffer overflow
**And** every standard-ID `0x305` message SHALL be dispatched to the steering-angle decoder and SHALL NOT be logged as unmatched
**And** other non-matching messages SHALL be discarded
**And** while waiting for an OBD-II response, the first matching response SHALL be accepted and parsed

## ADDED Requirements
### Requirement: Shared-Bus Steering Angle Frame Reception
The MCP2515 SHALL accept the Bosch LWS steering-angle frame (`0x305`) through its own receive buffer alongside the OBD-II response window, so that the 100 Hz angle stream and engine replies on the one shared bus do not evict each other.

#### Scenario: Two acceptance paths
- **WHEN** the receive filters are applied (at `begin()` and after any re-initialization)
- **THEN** RXB0 SHALL use mask `0x7F8` with filters 0 and 1 = `0x7E8` (OBD-II replies `0x7E8`–`0x7EF`)
- **AND** RXB1 SHALL use mask `0x7FF` with filters 2–5 = `0x305` (steering angle only)
- **AND** extended-ID frames SHALL match no filter
- **AND** RXB0-to-RXB1 rollover SHALL stay enabled so an OBD-II reply arriving while RXB0 is full is not lost

#### Scenario: Lost steering frames are tolerated
- **WHEN** a `0x305` frame is lost (RXB1 overflow or a slow loop iteration)
- **THEN** the loss SHALL NOT be treated as a CAN error, because every frame carries the absolute angle
- **AND** only the steering-angle frame timeout SHALL turn missing frames into a steering sensor fault

#### Scenario: OBD-II polling unaffected
- **WHEN** the steering-angle sensor transmits at 100 Hz on the shared bus
- **THEN** the RPM poll cadence (`CAN_POLL_INTERVAL_RPM`, and `CAN_POLL_INTERVAL_RPM_BOOST` during a gear change) and the other PIDs SHALL be polled as without the sensor

### Requirement: Steering Sensor Zero-Set Sequence
The CAN controller SHALL provide a non-blocking sequence that sets the Bosch LWS sensor's zero at the current position while keeping its output ID `0x305`, pausing OBD-II polling for its duration. It SHALL be the only code that transmits on ID `0x7C0`, and it SHALL transmit only the two fixed payloads below.

#### Scenario: Zero-set frames
- **WHEN** a zero-set is started
- **THEN** OBD-II polling SHALL be paused: no new request SHALL be sent, and a request already in flight SHALL be allowed to complete or time out first
- **AND** the frame `0x7C0`, DLC 2, `05 00` (reset calibration) SHALL be sent
- **AND** after `STEER_LWS_ZERO_GAP_MS` (about 100 ms), measured without blocking, the frame `0x7C0`, DLC 2, `53 30` (zero here, output ID `0x305`) SHALL be sent
- **AND** OBD-II polling SHALL then resume where it left off

#### Scenario: No other configuration payload
- **WHEN** any code path transmits on ID `0x7C0`
- **THEN** the payload SHALL be exactly `05 00` or `53 30`
- **AND** no interface SHALL accept a `0x7C0` payload from outside the CAN controller, so that a payload such as `03 00` (which moves the sensor's output to ID `0x000`) can never be sent

#### Scenario: Start refused or aborted
- **WHEN** a zero-set is requested while the CAN controller is not initialized, an ECU capability probe is running, or a zero-set is already running
- **THEN** it SHALL be refused with a reason and no frame SHALL be sent
- **WHEN** a zero-set frame fails to transmit
- **THEN** the sequence SHALL abort with a `tx_failed` reason and OBD-II polling SHALL resume
