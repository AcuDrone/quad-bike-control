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
**And** every reset or re-initialization SHALL re-apply all acceptance paths (OBD-II window, steering VESC extended frames and steering-angle frame)
**And** steering VESC transmit failures SHALL NOT count toward the send-failure threshold that triggers a reset
**And** the whole reset or re-initialization SHALL run under the MCP2515 SPI lock, the CAN reader task SHALL skip reading while it runs, and the receive interrupts (CANINTE = RX0IE | RX1IE) SHALL be re-asserted and read back before the lock is released
**And** frames already in the receive queue SHALL be kept, and the reader task SHALL be woken once afterwards to read any frame latched during the reset

#### Scenario: Drain MCP2515 receive buffers
**Given** the CAN controller is initialized, in any OBD-II state (IDLE or WAITING_RESPONSE), in probe mode or during an LWS zero-set
**When** the CAN reader task has queued received frames
**Then** every `update()` call SHALL take all queued frames, at most `CAN_RX_QUEUE_DEPTH` per call, before running the OBD-II or probe state machine
**And** every standard-ID `0x305` message SHALL be dispatched to the steering-angle decoder and SHALL NOT be logged as unmatched
**And** every extended-ID steering VESC message (requirement "Shared-Bus Steering VESC Frames") SHALL be dispatched to the VESC steering driver and SHALL NOT be logged as unmatched
**And** while waiting for an OBD-II or probe response, the first matching response SHALL be accepted and parsed
**And** other non-matching messages SHALL be discarded and logged as unmatched at most once per `CAN_RX_LOG_INTERVAL`
**And** the MCP2515 hardware receive buffers SHALL be read only by the CAN reader task

### Requirement: Shared-Bus Steering Angle Frame Reception
The MCP2515 SHALL accept the Bosch LWS steering-angle frame (`0x305`) through its own receive buffer, and the OBD-II response window together with the steering VESC's extended frames through the other, so that the 100 Hz angle stream, the engine replies and the VESC traffic on the one shared bus do not evict each other.

#### Scenario: Two acceptance paths
- **WHEN** the receive filters are applied (at `begin()` and after any re-initialization)
- **THEN** RXB0 SHALL use mask `0x7F8` (`init_Mask(0, 0, 0x07F80000)`), filter 0 = standard `0x7E8` (OBD-II replies `0x7E8`–`0x7EF`) and filter 1 = extended ID `0x00000000` (`init_Filt(1, 1, 0x00000000)`), which accepts every extended ID below `0x200000` and therefore every VESC frame
- **AND** RXB1 SHALL use mask `0x7FF` with filters 2–5 = standard `0x305` (steering angle only), and SHALL accept no extended frame
- **AND** no mask SHALL care about any extended-ID bit below ID bit 18, because for standard frames the MCP2515 applies those mask bits to data bytes 0–1
- **AND** RXB0-to-RXB1 rollover SHALL stay enabled so an OBD-II reply or VESC frame arriving while RXB0 is full is not lost

#### Scenario: Lost steering frames are tolerated
- **WHEN** a `0x305` frame is lost (RXB1 overflow or a slow loop iteration)
- **THEN** the loss SHALL NOT be treated as a CAN error, because every frame carries the absolute angle
- **AND** only the steering-angle frame timeout SHALL turn missing frames into a steering sensor fault

#### Scenario: OBD-II polling unaffected
- **WHEN** the steering-angle sensor transmits at 100 Hz and the steering VESC transmits its status frames and command traffic on the shared bus
- **THEN** the RPM poll cadence (`CAN_POLL_INTERVAL_RPM`, and `CAN_POLL_INTERVAL_RPM_BOOST` during a gear change) and the other PIDs SHALL be polled as without them

## ADDED Requirements
### Requirement: Shared-Bus Steering VESC Frames
The CAN controller SHALL carry the steering VESC's native CAN protocol on the shared MCP2515 bus: it SHALL dispatch the VESC's extended frames to the VESC steering driver and SHALL be the only transmitter of extended frames, through a whitelist. The bus SHALL NOT be shared with DroneCAN. Reliable reception of back-to-back VESC frames SHALL rely on interrupt/task-driven MCP2515 reception (change `move-can-rx-to-interrupt`).

#### Scenario: Dispatch VESC frames
- **WHEN** an extended data frame with ID ≤ `0xFFFF` is received
- **THEN** it SHALL be dispatched to the VESC steering driver if its low byte is `STEER_VESC_CAN_ID` and its cmd byte (ID bits 15–8) is 9, 16 or 27, or if its low byte is `CAN_CONTROLLER_VESC_SENDER_ID` and its cmd byte is 5, 7 or 8
- **AND** any other extended frame, and any remote frame, SHALL be counted and dropped
- **AND** dispatch SHALL happen in every CAN state (IDLE, WAITING_RESPONSE, probe, LWS zero-set)

#### Scenario: Whitelisted VESC transmission
- **WHEN** the VESC steering driver asks the CAN controller to transmit
- **THEN** the frame SHALL be an extended data frame with ID `(cmd << 8) | STEER_VESC_CAN_ID` and at most 8 data bytes
- **AND** cmd SHALL be `CAN_PACKET_SET_DUTY` (0) or `CAN_PACKET_PROCESS_SHORT_BUFFER` (8); any other cmd SHALL be refused without transmitting
- **AND** no code path SHALL transmit an extended frame other than through this entry point, and no frame SHALL be addressed to the VESC broadcast ID 255
- **AND** the transmission SHALL be allowed in every OBD-II, probe and zero-set state and SHALL be refused while the controller is not initialized

#### Scenario: Back-to-back bursts received
- **WHEN** the VESC sends STATUS_1, STATUS_4 and STATUS_5 back-to-back (about 260 µs apart) every 200 ms while the LWS streams at 100 Hz
- **THEN** all three frames SHALL reach the VESC steering driver, and the MCP2515 RX overflow count SHALL stay 0
- **AND** this SHALL be provided by interrupt/task-driven reception, not by the main-loop poll rate

#### Scenario: No DroneCAN on this bus
- **WHEN** the bus plan is documented
- **THEN** it SHALL state that the VESC bus is not shared with DroneCAN, because a DroneCAN message from a node whose ID equals the VESC ID with a type-ID low byte of 0–4 would drive the motor, and no ID choice removes that
