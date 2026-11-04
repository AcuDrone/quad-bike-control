## MODIFIED Requirements
### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.
The RPM poll interval SHALL be adjustable at runtime via `setRPMPollInterval(uint32_t ms)` to allow higher-frequency polling during gear changes.
The active poll table SHALL contain seven PIDs: engine RPM (`0x0C`) in the fast class, and coolant temperature (`0x05`), throttle position (`0x11`), manifold absolute pressure (`0x0B`), control module voltage (`0x42`), intake air temperature (`0x0F`) and calculated engine load (`0x04`) in the `CAN_POLL_INTERVAL_TEMP` class. Exactly one OBD-II request SHALL be in flight at any time, and the RPM poll cadence SHALL remain unchanged by the additional PIDs.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the MCP2515 shall be initialized at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM: `CAN_POLL_INTERVAL_RPM` ms; coolant, throttle position, MAP, module voltage, intake air temperature and engine load: `CAN_POLL_INTERVAL_TEMP` ms)
**And** the state machine shall be set to IDLE
**And** return `true` if successful, `false` otherwise

#### Scenario: Non-blocking OBD-II polling via state machine
**Given** the CAN controller is initialized
**When** `update()` is called
**Then** every call SHALL first take the frames the CAN reader task has queued since the previous call (requirement "Interrupt-Driven CAN Reception") and route each of them
**And** if state is IDLE and a PID is due for polling, an OBD-II request SHALL be queued into a free MCP2515 transmit buffer without waiting for it to leave, and state transitions to WAITING_RESPONSE
**And** if state is WAITING_RESPONSE and a matching response was among the routed frames, it SHALL be parsed and stored, and state transitions to IDLE
**And** a response latched for an earlier request SHALL NOT be accepted for a newer one
**And** if the response timeout (200ms) elapses, the request SHALL be marked as failed and state transitions to IDLE
**And** `update()` SHALL NOT read the MCP2515 receive buffers itself and SHALL return without blocking the main loop

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
**Given** the CAN controller is initialized
**When** the module voltage PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x42 shall be sent
**And** the response shall be parsed to extract the control module supply voltage
**And** the raw value `((A*256)+B)` SHALL be stored verbatim as millivolts in `VehicleData.moduleVoltageMv`
**And** consumers SHALL convert to volts as `((A*256)+B)/1000` (0.000–65.535 V) at the presentation edge

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
**Given** the poll table contains seven PIDs with RPM in the fast class and six PIDs at `CAN_POLL_INTERVAL_TEMP`
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
**And** the whole reset or re-initialization SHALL run under the MCP2515 SPI lock, the CAN reader task SHALL skip reading while it runs, and the receive interrupts (CANINTE = RX0IE | RX1IE) SHALL be re-asserted and read back before the lock is released
**And** frames already in the receive queue SHALL be kept, and the reader task SHALL be woken once afterwards to read any frame latched during the reset

#### Scenario: Drain MCP2515 receive buffers
**Given** the CAN controller is initialized, in any OBD-II state (IDLE or WAITING_RESPONSE), in probe mode or during an LWS zero-set
**When** the CAN reader task has queued received frames
**Then** every `update()` call SHALL take all queued frames, at most `CAN_RX_QUEUE_DEPTH` per call, before running the OBD-II or probe state machine
**And** every standard-ID `0x305` message SHALL be dispatched to the steering-angle decoder and SHALL NOT be logged as unmatched
**And** while waiting for an OBD-II or probe response, the first matching response SHALL be accepted and parsed
**And** other non-matching messages SHALL be discarded and logged as unmatched at most once per `CAN_RX_LOG_INTERVAL`
**And** the MCP2515 hardware receive buffers SHALL be read only by the CAN reader task

## ADDED Requirements
### Requirement: Interrupt-Driven CAN Reception
The CAN controller SHALL empty the MCP2515 receive buffers from a dedicated FreeRTOS reader task woken by the MCP2515 `INT` line (`PIN_CAN_INT`, GPIO38), and SHALL pass the frames to the main loop through a bounded software queue, so that neither a long main-loop iteration nor a busy web handler loses frames and back-to-back bursts of at least three frames are received.

#### Scenario: Interrupt wakes the reader task without SPI
- **WHEN** the MCP2515 pulls `INT` low because a frame arrived in RXB0 or RXB1
- **THEN** a FALLING-edge ISR (IRAM) SHALL only count the interrupt and give a binary semaphore with `xSemaphoreGiveFromISR()` and `portYIELD_FROM_ISR()`
- **AND** the ISR SHALL NOT access SPI, the MCP2515 library or the receive queue

#### Scenario: Reader task drains the chip
- **WHEN** the reader task wakes on the semaphore or after `CAN_RX_WAIT_MS` (2 ms) without one
- **THEN** it SHALL read frames while `checkReceive()` reports one, at most `CAN_RX_DRAIN_CAP` (16) frames per wake, taking the MCP2515 SPI lock for each frame read and releasing it before the frame is queued
- **AND** it SHALL send each frame to the receive queue without blocking, keeping the ID in the `readMsgBuf()` encoding (extended and remote flags in bits 31 and 30), the DLC, the data and the receive time
- **AND** it SHALL skip reading while a re-initialization is running
- **AND** it SHALL run at priority `CAN_RX_TASK_PRIORITY` (12, above the AsyncTCP task) on core `CAN_RX_TASK_CORE` (1, the Arduino loop's core) with a `CAN_RX_TASK_STACK` (4096-byte) stack, and SHALL NOT log

#### Scenario: Only receive interrupts are enabled
- **WHEN** the MCP2515 is initialized or re-initialized
- **THEN** CANINTE SHALL be `RX0IE | RX1IE` only (`CAN_CANINTE_VALUE`, 0x03), written through the controller's raw register access and read back, with a mismatch logged as an error
- **AND** error, wake-up and transmit interrupts SHALL stay disabled, and EFLG, TEC and REC SHALL be read from the main loop
- **AND** the `mcp_can` library SHALL NOT be modified for this

#### Scenario: Back-to-back burst received
- **WHEN** three frames accepted by the filters arrive back-to-back about 260 µs apart while the main loop is busy for tens of milliseconds
- **THEN** all three SHALL reach the receive queue, and the MCP2515 RX overflow count SHALL NOT increase

#### Scenario: Queue full
- **WHEN** the receive queue (`CAN_RX_QUEUE_DEPTH`, 32 frames) is full
- **THEN** the newest frame SHALL be dropped and counted in `can_rx_queue_drops`, and the reader task SHALL NOT block

#### Scenario: Dead INT line degrades to polling
- **WHEN** `INT` never produces an edge (open, stuck, or the test build flag `CAN_RX_TEST_NO_INT`)
- **THEN** the reader task SHALL still drain the chip at least every `CAN_RX_WAIT_MS`
- **AND** the steering-angle frames SHALL stay fresh

#### Scenario: Initialization failed
- **WHEN** `begin()` fails
- **THEN** no reader task, interrupt, queue or SPI mutex SHALL be created, and the steering-angle sensor SHALL report `can_down` as before

### Requirement: MCP2515 SPI Access Serialization
Every access to the MCP2515 SHALL be serialized by one FreeRTOS mutex with bounded waits, and no holder SHALL keep it longer than its own SPI sequence, so that the reader task and the main loop never interleave on the chip and neither blocks the other for long.

#### Scenario: One lock around every chip access
- **WHEN** any code reads or writes the MCP2515 (library calls, raw `readRegister()`/`modifyRegister()`, transmit, error-flag reads, initialization, re-initialization)
- **THEN** it SHALL hold the single SPI mutex, taken only at the outermost entry point and never taken twice by the same holder
- **AND** no log output SHALL be produced while the mutex is held

#### Scenario: Bounded wait
- **WHEN** the mutex is not obtained within `CAN_SPI_LOCK_TIMEOUT_MS` (10 ms)
- **THEN** the waiting operation SHALL fail without touching the chip and the lock timeout SHALL be counted
- **AND** in the main loop a failed OBD-II send SHALL be rescheduled without incrementing the consecutive send-failure count
- **AND** no wait on the mutex or on the receive queue SHALL be unbounded

#### Scenario: Non-blocking OBD-II transmit
- **WHEN** an OBD-II Mode 01 or Mode 03 request is sent
- **THEN** it SHALL be written into a free MCP2515 transmit buffer and requested to send under one short lock hold, without waiting for the frame to be acknowledged
- **AND** if all three transmit buffers are busy, the send SHALL fail and SHALL count toward the existing consecutive send-failure handling (abort and ABAT clear)
- **AND** the LWS zero-set frames SHALL still be sent with the library's confirmed transmit

### Requirement: CAN Receive Path Diagnostics
The CAN controller SHALL count and report the health of the interrupt-driven receive path, and SHALL detect an inactive `INT` line, using rate-limited log lines only.

#### Scenario: Inactive INT detected
- **WHEN** frames were read in a `CAN_INT_CHECK_MS` (5000 ms) window but no interrupt was counted in it
- **THEN** the controller SHALL log `[CAN] INT inactive, polling fallback` once and report `can_int_ok` false
- **WHEN** interrupts are counted again in a later window
- **THEN** it SHALL log `[CAN] INT active` once and report `can_int_ok` true
- **AND** a window with no frames SHALL leave the state unchanged

#### Scenario: Receive health line
- **WHEN** `CAN_HEALTH_LOG_INTERVAL` (5 s) elapses while the controller is initialized
- **THEN** one `[CAN] rx:` line SHALL report the interrupt count, the frames read by the task, the queue drops, the queue high-water mark, the per-wake cap hits, the SPI lock timeouts, the cumulative RX overflow count, the maximum queue age, the task stack high-water mark and the INT state
- **AND** this line SHALL be printed whether or not OBD-II data is valid

#### Scenario: RX overflow monitored in every state
- **WHEN** `update()` runs in any OBD-II, probe or zero-set state
- **THEN** EFLG RX0OVR/RX1OVR SHALL be checked at least every `CAN_EFLG_CHECK_MS` (100 ms), cleared when set, added to a cumulative overflow count, and logged at most once per `CAN_OVERFLOW_LOG_INTERVAL`
