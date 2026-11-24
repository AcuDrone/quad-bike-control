## ADDED Requirements
### Requirement: CAN Bus Sniffer
The CAN controller SHALL provide a one-shot sniffer that, for a bounded window, accepts every
standard and extended CAN ID and records a per-ID summary, then restores the normal acceptance
filters and resumes OBD-II polling.

#### Scenario: Start the sniffer
- **WHEN** `startSniff(ms)` is called while the controller is initialised, no ECU probe and no LWS
  zero-set is running, and no gear change is active
- **THEN** OBD-II polling SHALL pause (an in-flight request is abandoned)
- **AND** both receive-buffer masks SHALL be set to 0 with one standard and one extended filter per
  buffer, so every standard and extended ID is accepted, with RXB0→RXB1 rollover kept
- **AND** the previous results SHALL be cleared and the window SHALL start

#### Scenario: Refuse while busy
- **WHEN** `startSniff()` is called while the ECU probe, the LWS zero-set or a sniff is running, or
  a gear change is active
- **THEN** it SHALL be refused and the bus configuration SHALL not change
- **WHEN** an ECU probe or an LWS zero-set is requested while the sniffer runs
- **THEN** that request SHALL be refused as busy

#### Scenario: Record frames
- **WHEN** a frame is received during the window
- **THEN** up to 32 frames SHALL be read per `update()` call
- **AND** the total frame count SHALL increase
- **AND** for an ID already in the table its count, DLC and last 8 data bytes SHALL be updated, and
  each data byte that differs from the previous frame of that ID SHALL set its bit in the ID's
  changed mask (the bit stays set)
- **AND** a new ID SHALL get a row while fewer than 32 rows exist
- **AND** a new ID that does not fit SHALL be counted in a distinct "IDs beyond 32" counter
- **AND** each RX0OVR / RX1OVR overflow flag seen SHALL be counted and cleared
- **AND** a standard 0x305 frame SHALL still be handed to the LWS decoder

#### Scenario: Window ends
- **WHEN** the window elapses
- **THEN** the normal masks and filters SHALL be re-programmed by the same routine used at start-up
  (a full chip re-initialisation if that fails)
- **AND** OBD-II polling SHALL resume from IDLE without an immediate forced re-initialisation
- **AND** the results SHALL be marked complete with the completion time

#### Scenario: Steering sensor during the window
- **WHEN** LWS frames are lost during the window because the receive buffers overflow
- **THEN** the steering controller SHALL take its normal sensor-unhealthy path
- **AND** it SHALL recover by itself once valid frames arrive again, without a reboot
