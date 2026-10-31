## MODIFIED Requirements
### Requirement: Bosch LWS Steering Angle Sensor
The steering position SHALL be read from a Bosch LWS steering-angle sensor 0 265 005 486 (PSA 9650236180) mounted directly on the steering shaft, which transmits standard CAN frame `0x305` (DLC 6, 100 Hz, 500 kbit/s) by itself after 12 V power-up from the ignition key. The decoder SHALL be free of Arduino dependencies and SHALL reproduce the bench-verified `lws305.h` logic.

#### Scenario: Decode an accepted frame
- **WHEN** a standard-ID `0x305` frame with DLC ≥ 6 is received
- **AND** the high nibble of byte 4 equals the XOR of the eight nibbles of bytes 0..3 and the counter (low nibble of byte 4)
- **AND** the counter differs from that of the last accepted frame
- **THEN** the frame SHALL be accepted
- **AND** the angle SHALL be bytes 0–1 as big-endian int16 in 0.1°, with `0x7FFF` meaning invalid
- **AND** the rotation speed SHALL be byte 2 × 4 °/s, with `0xFF` meaning invalid, and the direction SHALL be byte 3 bit 7 (1 = clockwise)
- **AND** the status SHALL be byte 3: bits 6..3 fault code (0 = none), bit 2 adjusted, bit 1 calibrated, bit 0 sensor OK
- **AND** the firmware position SHALL be the negated angle for a normal mount (sensor positive = left; firmware positive = right), or the angle unchanged when the sensor-direction flag says the sensor is mounted mirrored (see "Steering Sensor Direction Inference" in vehicle-systems)
- **AND** the signed rotation rate SHALL follow the same rule, so that a positive rate always means turning right
- **AND** the decoder itself SHALL stay sign-neutral: the raw angle and direction bit SHALL remain available in the sensor's own sign for logs

#### Scenario: Reject corrupt or repeated frames
- **WHEN** a `0x305` frame fails the checksum
- **THEN** it SHALL be dropped and a checksum-error counter incremented
- **WHEN** a `0x305` frame carries the same counter as the last accepted frame
- **THEN** it SHALL be dropped and a counter-error counter incremented
- **WHEN** the counter advanced by more than one since the last accepted frame
- **THEN** the frame SHALL be accepted and the number of missed frames added to a miss counter, since every frame is absolute

#### Scenario: Sensor health
- **WHEN** the steering controller evaluates the sensor
- **THEN** it SHALL report exactly one fault token, in this priority order: `zeroing` (zero-set sequence running), `can_down` (CAN controller not initialized), `timeout` (no accepted frame for `STEER_LWS_TIMEOUT_MS`), `range_latched` (fault code 9), `sensor_fault` (OK bit clear or any other nonzero fault code), `not_calibrated` (calibrated bit clear), `angle_invalid` (angle `0x7FFF`), otherwise `none`
- **AND** a fault SHALL take effect on the first frame or timeout that shows it
- **AND** the sensor SHALL be healthy only when the token is `none` for at least `STEER_LWS_RECOVER_FRAMES` consecutive accepted frames

#### Scenario: Latched out-of-range fault
- **WHEN** the sensor reports fault code 9 (it was turned beyond its ~±600° range)
- **THEN** the token SHALL be `range_latched` and telemetry SHALL tell the operator to restart the vehicle
- **AND** the firmware SHALL take no other action to clear it (the sensor is powered from the ignition key and clears only on a power cycle)

#### Scenario: Range guard for closed-loop moves only
- **WHEN** the sensor is healthy and a closed-loop position move driven by a percent command (MAVLink, web slider or test buttons, boot centering, failsafe centre) would reach or pass `STEER_LWS_SAFE_RANGE` (550.0°)
- **THEN** its target SHALL be clamped to ±`STEER_LWS_SAFE_RANGE`, and the move SHALL be stopped rather than drive further outward at or beyond it
- **AND** driving back toward zero SHALL remain allowed
- **AND** jog, nudge and limit captures SHALL NOT be limited by the range guard
