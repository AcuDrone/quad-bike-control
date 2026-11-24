## MODIFIED Requirements
### Requirement: Shared-Bus Steering Angle Frame Reception
The MCP2515 SHALL accept the Bosch LWS steering-angle frame (`0x305`) and the three Bosch iBooster
status frames (`0x38E`, `0x38F`, `0x39D`) through receive buffer RXB1, and the OBD-II response window
through RXB0, so that the 100 Hz angle stream, the brake status and the engine replies on the one
shared bus do not evict each other.

#### Scenario: Two acceptance paths
- **WHEN** the receive filters are applied (at `begin()` and after any re-initialization)
- **THEN** RXB0 SHALL use mask `0x7F8` with filters 0 and 1 = `0x7E8` (OBD-II replies `0x7E8`–`0x7EF`)
- **AND** RXB1 SHALL use mask `0x7FF` with filter 2 = `0x305`, filter 3 = `0x38E`, filter 4 =
  `0x38F` and filter 5 = `0x39D` (exact matches)
- **AND** extended-ID frames SHALL match no filter
- **AND** RXB0-to-RXB1 rollover SHALL stay enabled so an OBD-II reply arriving while RXB0 is full is not lost

#### Scenario: Lost steering frames are tolerated
- **WHEN** a `0x305` frame is lost (RXB1 overflow or a slow loop iteration)
- **THEN** the loss SHALL NOT be treated as a CAN error, because every frame carries the absolute angle
- **AND** only the steering-angle frame timeout SHALL turn missing frames into a steering sensor fault

#### Scenario: Lost brake status frames are tolerated
- **WHEN** single `0x38E`, `0x38F` or `0x39D` frames are lost
- **THEN** the loss SHALL NOT be treated as a CAN error
- **AND** only the `0x39D` staleness limit (250 ms) SHALL turn missing frames into a lost brake unit

#### Scenario: OBD-II polling unaffected
- **WHEN** the steering-angle sensor transmits at 100 Hz and the iBooster at 175 frames/s on the
  shared bus
- **THEN** the RPM poll cadence (`CAN_POLL_INTERVAL_RPM`, and `CAN_POLL_INTERVAL_RPM_BOOST` during a gear change) and the other PIDs SHALL be polled as without them

### Requirement: Queued CAN Transmit
Every frame the main loop sends on the CAN bus SHALL be queued in a transmit ring of `CAN_TX_RING` (8)
frames and loaded into the MCP2515 by the CAN task, without any busy-wait in the main loop. General
frames SHALL use transmit buffer TXB0 only, in FIFO order, at TXP 1. TXB1 and TXB2 SHALL be used only
by the CAN task's periodic iBooster schedule (see "iBooster Periodic Frames").

#### Scenario: Queue a frame
- **WHEN** the main loop queues a frame
- **THEN** the call SHALL return at once, `true` if the frame was queued and `false` if the ring is full (counted)
- **AND** the task SHALL be notified

#### Scenario: Frame sent
- **WHEN** TXB0 is free and a frame is queued
- **THEN** the task SHALL write it into TXB0 and request transmission
- **AND** when TXREQ clears, the frame SHALL count as sent

#### Scenario: Frame not sent in time
- **WHEN** a frame has not left TXB0 within `CAN_TX_TIMEOUT_MS` (20 ms)
- **THEN** the task SHALL abort that buffer only, by clearing its TXREQ
- **AND** once TXREQ reads clear, the frame SHALL count as a transmit failure if TXB0CTRL.ABTF is set, and as sent otherwise (an attempt already on the wire completed)
- **AND** if TXREQ is still set `CAN_TX_TIMEOUT_MS` after the abort, the frame SHALL count as a transmit failure
- **AND** no other buffer and no other sender's frame SHALL be aborted
- **AND** the CANCTRL abort-all bit SHALL NOT be used

#### Scenario: Sender asks for the result
- **WHEN** a frame was queued with a result request (the LWS zero-set frames)
- **THEN** the task SHALL report sent or failed to the main loop
- **AND** a frame queued while the chip is down (a failed initialization or re-initialization) SHALL be reported as failed at once
- **AND** a frame still pending in the transmit buffer when a re-initialization or filter change starts SHALL be aborted in that buffer only and reported as sent or failed by the same ABTF rule
- **AND** a frame whose requester has stopped waiting for its result SHALL NOT be loaded into the chip afterwards

#### Scenario: Steering sensor zero-set frames
- **WHEN** the zero-set queues `[05 00]`
- **THEN** the reset frame SHALL be reported as possibly sent from that moment, so the steering erases its Left / Right limits whatever the transmit result (only a frame that could not be queued leaves them)
- **AND** the sequence SHALL continue only after a confirmed send of `[05 00]`, and SHALL complete only after a confirmed send of `[53 30]`
- **AND** a failed result, or no result within `CAN_TX_RESULT_WAIT_MS` (1000 ms), for either frame SHALL abort the sequence with `tx_failed`, and a frame without a result SHALL be withdrawn so that it is never sent later

#### Scenario: OBD-II send failure
- **WHEN** an OBD-II request cannot be queued
- **THEN** the PID SHALL be rescheduled and its retry counter incremented, as for any send failure
- **AND** a request that was queued but never sent SHALL be handled by the 200 ms response timeout

#### Scenario: General frames never delay the brake frames
- **WHEN** a general frame is retransmitting in TXB0 while brake frames are loaded
- **THEN** the brake frames SHALL leave first, because TXB1 / TXB2 have a higher TXP

### Requirement: CAN Receive Queue
Received frames SHALL pass from the CAN task to the main loop through a bounded ring of `CAN_RX_RING`
(64) frames, each holding the ID in the `readMsgBuf()` encoding (bit 31 extended, bit 30 remote), the
DLC and up to 8 data bytes. The exception SHALL be the iBooster status frames (`0x38E`, `0x38F`,
`0x39D`): they SHALL be consumed by the CAN task's brake client and SHALL enter the ring only while the
CAN sniffer window is open.

#### Scenario: Ring full
- **WHEN** the task has a frame and the ring is full
- **THEN** the oldest frame in the ring SHALL be discarded, the new one stored, and the drop counted
- **AND** the task SHALL NOT block

#### Scenario: Consumers stay in the main loop
- **WHEN** `update()` takes frames from the ring
- **THEN** steering-angle frames SHALL be handed to the LWS decoder with the main-loop time of dispatch
- **AND** the LWS decoder, the OBD-II and probe state machines, the sniffer table and the CAN watchdogs SHALL run only in the main loop

#### Scenario: Brake frames stay in the task
- **WHEN** the task reads a `0x38E`, `0x38F` or `0x39D` frame and the sniffer window is closed
- **THEN** the frame SHALL be decoded by the brake client in the task and SHALL NOT be put in the ring
- **AND** while the sniffer window is open the frame SHALL be both decoded and put in the ring

#### Scenario: Extended frames pass through
- **WHEN** a frame with an extended ID is accepted by the acceptance filters
- **THEN** it SHALL reach the main loop with bit 31 of its ID set

## ADDED Requirements
### Requirement: iBooster Periodic Frames
While frames to the iBooster are enabled (see "iBooster Unit State Machine"), the CAN task SHALL
send `0x38B` (DLC 4), `0x38C` (DLC 4) and `0x38D` (DLC 7) once per `IB_SLOT_MS` (10 ms) slot, built by
`include/IboosterProto.h` exactly as on the bench: byte 0 = CRC-8 SAE J1850 (polynomial `0x1D`, init
`0xFF`, xorout `0xFF`) over bytes 1..end; the low nibble of byte 1 = one 4-bit counter shared by the
three frames of a slot and incremented in every slot whose frames are sent.

#### Scenario: Frame content
- **WHEN** a slot is built
- **THEN** `0x38B` SHALL be `[CRC] [Q|cnt] [speed] 05`, `0x38C` SHALL be `[CRC] [Q|cnt] [flow_lo]
  [flow_hi]` and `0x38D` SHALL be `[CRC] [cnt] 00 00 00 50 47`
- **AND** Q SHALL be `0x40` while the brake request is set and `0x00` otherwise; `0x80` SHALL never
  be set
- **AND** the flow SHALL be `0x7E00` whenever the request is not set

#### Scenario: Reproduces the reference trace
- **WHEN** the builder is given counter n (0–15), the request, flow `0x8300` and speed 0
- **THEN** the three frames SHALL equal the 48 frames of the reference trace byte for byte

#### Scenario: Buffers and order
- **WHEN** a slot starts
- **THEN** a brake buffer whose TXREQ is still set from the previous slot SHALL be aborted alone (its
  own TXREQ cleared) and counted as a transmit miss
- **AND** `0x38D` SHALL be loaded into TXB2 (TXP 3) and `0x38C` into TXB1 (TXP 2) and both requested
- **AND** `0x38B` SHALL be loaded into TXB1 on the first task cycle that finds it free, the task
  waiting at most one tick between checks while it is pending
- **AND** the TXP bits SHALL be written again after every (re-)initialization
- **AND** CANINTE SHALL stay as the initialization sets it

#### Scenario: Slot timing
- **WHEN** the task waits
- **THEN** its wait SHALL end no later than the next slot
- **AND** a late slot SHALL be sent once, without a catch-up burst, and the gap between two sent
  slots SHALL be measured

#### Scenario: Across a re-initialization or filter switch
- **WHEN** the CAN task starts any command (re-initialization, filter switch)
- **THEN** it SHALL first clear TXREQ on TXB1 and TXB2 as well as TXB0, each buffer alone, and count
  the aborted brake frames as transmit misses
- **AND** no brake frame SHALL be loaded until the command has ended and, on success, the TXP bits
  have been written again
- **AND** the next slot SHALL carry frames built from the current command; no brake frame SHALL be
  kept from before the command
- **AND** the brake SHALL be re-commanded by the rod loop with no operator action if the unit dropped
  the request during the gap

#### Scenario: Nothing sent while asleep
- **WHEN** the brake state is `ASLEEP`, the chip is down, or a re-initialization is running
- **THEN** no brake frame SHALL be loaded

### Requirement: iBooster Frame Reception
The CAN task SHALL check and decode `0x38E`, `0x38F` and `0x39D` as they are drained from the chip,
and SHALL publish the result to the main loop as one snapshot record.

#### Scenario: Checksums
- **WHEN** a `0x38E` or `0x38F` frame arrives
- **THEN** its byte 0 SHALL be checked as CRC-8 SAE J1850 over bytes 1..end
- **WHEN** a `0x39D` frame arrives
- **THEN** its byte 0 SHALL be checked as (`0x03` + `0x9D` + bytes 1..4) & `0xFF`
- **AND** a frame failing its check SHALL be counted and discarded

#### Scenario: Decoded fields
- **WHEN** a valid `0x39D` arrives
- **THEN** status (byte 1 bits 4–6), counter (byte 1 low nibble), driver_brake_apply (byte 2 bits
  0–1), internal state (byte 2 bits 2–4) and rod position (12 bits from byte 2 bit 5, mm = raw / 64 −
  5) SHALL be stored with the arrival time
- **WHEN** a valid `0x38E` arrives
- **THEN** the external-request status (byte 1 high nibble) and the two 12-bit rod fields (bytes 2–3
  and 4–5, mm = raw / 64 − 5) SHALL be stored
- **WHEN** a valid `0x38F` arrives
- **THEN** the available pressure (byte 3, bar) and the partner flag (byte 4: `0x40` sees us, `0x80`
  does not) SHALL be stored
- **AND** the last raw bytes of each of the three frames SHALL be kept for the journal

#### Scenario: Real bench frames decode
- **WHEN** the `0x39D` frame `F8 50 E1 27 00` is decoded
- **THEN** status SHALL be 5, counter 0, driver_brake_apply 1, internal state 0 and the rod raw 319

#### Scenario: Implausible rod
- **WHEN** a checksum-valid `0x39D` carries a rod outside −1.00 … 40.00 mm (the 12-bit field spans
  −5 … 59 mm; the unit's mechanical full stroke is ~38 mm)
- **THEN** the frame SHALL be counted as bad and discarded

#### Scenario: Frame held in the chip through a task stall
- **WHEN** a unit frame is drained more than `IB_RX_TRUST_MS` (25 ms) after the last slot (a flash
  erase or another stall held the task while the frame waited in a receive buffer)
- **THEN** its arrival time is unknown: it SHALL NOT be used as a fresh frame (status, rod, partner
  and their times stay as they were), only its raw bytes SHALL be kept for the journal, and it SHALL
  be counted

### Requirement: Brake Command Hand-Off to the CAN Task
The main loop and the CAN task SHALL exchange brake information only through three bounded,
non-blocking channels, each with one writer: a command record (loop → task, every loop: time stamp,
rod target, apply and release flow limits, vehicle-speed byte, ignition-on, cranking), a snapshot
record (task → loop) and a brake event ring of 8 entries (task → loop, oldest dropped and counted
when full). The task-side brake code SHALL never journal, log, allocate or touch NVS.

#### Scenario: Command published every loop
- **WHEN** `IServiceBrake::update()` runs
- **THEN** a new command record with the current time stamp SHALL be copied for the task under a
  short critical section

#### Scenario: Stale command
- **WHEN** the newest command is older than `IB_CMD_STALE_MS` (500 ms)
- **THEN** the task SHALL keep using it, flag it stale and record the episode length for the journal

#### Scenario: Events reach the journal
- **WHEN** the task changes the unit state, sees a unit drop, a slot gap above 50 ms or the end of a
  stale-command episode
- **THEN** it SHALL put an event with the state, the unit fields, the target, the request bit, the
  flow and the last raw frames into the event ring, and the main loop SHALL journal it

### Requirement: CAN Maintenance During a Brake Request
While a brake request is active (`BRAKING` or `RELEASING`) the CAN controller SHALL NOT start work
that interrupts the periodic brake frames when it can be postponed.

#### Scenario: Watchdog re-initializations postponed
- **WHEN** the LWS silence watchdog or the OBD-II no-data watchdog would trigger a re-initialization
  during a brake request
- **THEN** the main loop SHALL NOT post the re-initialization during the request
- **AND** the watchdog condition SHALL be evaluated again on the first `update()` after the request
  ends and, if it still holds, the re-initialization SHALL be posted then, with the existing
  spacing, retry budget and journal lines (a condition that cleared during the request posts
  nothing)
- **AND** further attempts of a re-initialization sequence already in flight when the request
  started SHALL continue

#### Scenario: Bus-off still recovered
- **WHEN** bus-off is detected during a brake request
- **THEN** the recovery SHALL start at once, as before, because no frame leaves the chip anyway

#### Scenario: Sniff already open
- **WHEN** a brake request starts while a sniff window is open
- **THEN** the window SHALL run to its end and its closing filter switch SHALL NOT be postponed

#### Scenario: Sniffer refused
- **WHEN** the CAN sniffer is requested during a brake request
- **THEN** it SHALL be refused with the reason "brake" and the filters SHALL stay unchanged
