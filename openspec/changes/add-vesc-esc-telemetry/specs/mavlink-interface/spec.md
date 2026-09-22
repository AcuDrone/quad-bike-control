## ADDED Requirements

### Requirement: Steering ESC Electrical Telemetry via ESC_STATUS
The system SHALL report the steering VESC's live electrical telemetry to the MAVLink network using
the standard `ESC_STATUS` message (id 291) sent from this component, so the ground station sees the
steering driver's voltage and current in the fields that name those quantities. The vehicle has
exactly one ESC, which SHALL occupy slot index 0; the remaining slots SHALL be marked as carrying
no data. No value SHALL be reported in a field that names a different quantity, except that the
`rpm` slot — whose named quantity is permanently unmeasurable on this brushed, unsensored drive —
SHALL carry the measured position of the axis that same ESC drives. No `NAMED_VALUE_FLOAT` SHALL be
used for any of these values.

#### Scenario: Report steering ESC voltage and current
- **WHEN** the engine/state report interval elapses (`MAVLINK_REPORT_TX_MS`, default 200 ms / 5 Hz)
- **THEN** an `ESC_STATUS` message SHALL be sent from the ESP32's own system and component id
  (system 1, component 25), in the same interval tick that carries `EFI_STATUS`
- **AND** `index` SHALL be 0, identifying the single steering ESC
- **AND** `voltage[0]` SHALL carry the input voltage measured by the VESC itself — the 24 V boost
  rail that powers the steering driver — in volts
- **AND** `current[0]` SHALL carry the VESC's average MOTOR current in amperes, which is the
  steering load and stall diagnostic
- **AND** the VESC's average INPUT current SHALL NOT be reported in the same slot, because one
  physical ESC has exactly one current field and only one of the two quantities can occupy it
  honestly

#### Scenario: The reported ESC voltage is not the ECU module voltage
- **WHEN** both `ESC_STATUS` and `EFI_STATUS` are sent in the same interval tick
- **THEN** `ESC_STATUS.voltage[0]` SHALL be the VESC-measured steering supply rail
- **AND** `EFI_STATUS.ignition_voltage` SHALL remain the ECU control module supply voltage
  (OBD-II PID `0x42`)
- **AND** the two SHALL NOT be treated as the same measurement, because they are different physical
  quantities on different rails, each already carried in the field that names it

#### Scenario: Report the measured steering position in the rpm slot
- **WHEN** an `ESC_STATUS` message is packed
- **AND** the steering position sensor is healthy and the steering is calibrated
- **THEN** `rpm[0]` SHALL carry the MEASURED steering position in centi-percent of the calibrated
  lock-to-lock range — the steering-percent reading scaled by 100 and rounded to a 32-bit integer,
  spanning −10000 to +10000
- **AND** a negative value SHALL mean LEFT of centre and a positive value SHALL mean RIGHT of
  centre, matching the sign convention of the steering-percent accessor and the reverse-rotation
  sense the `rpm` field's own description assigns to negative values
- **AND** the value SHALL be percent of the calibrated travel, NOT degrees, because the firmware
  holds no counts-to-degrees calibration
- **AND** the VESC's own ERPM field SHALL NOT be decoded or forwarded, because the driver runs in
  brushed-DC mode with no motor sensor, so the quantity the field names is permanently unmeasurable
  on this vehicle — the same permanent-absence test that governs every other repurposed field
- **AND** the steering shaft angle SHALL be documented as the position of the axis this very ESC
  drives, measured downstream of its gearbox, which is why it occupies this slot rather than a
  field in another message

#### Scenario: Signal an unknown steering position with an out-of-range sentinel
- **WHEN** an `ESC_STATUS` message is packed
- **AND** the steering position sensor is not healthy, OR the steering is not calibrated
- **THEN** `rpm[0]` SHALL be `INT32_MIN`
- **AND** `rpm[0]` SHALL NOT be 0 in this case, because 0 is a valid reading meaning exactly
  straight-ahead, and the steering-percent accessor itself returns zero when uncalibrated, so a
  bare zero would report "centred" for a vehicle whose steering position is entirely unknown
- **AND** the sentinel SHALL be a value no real reading can produce, because the field is an
  integer type with no not-a-number encoding
- **AND** the sentinel SHALL be documented so a consumer renders "--" rather than plotting it as a
  position

#### Scenario: Steering position validity is independent of the ESC driver link
- **WHEN** the steering driver link is unhealthy — the VESC is unplugged, unpowered or silent —
  and the steering position sensor is healthy and calibrated
- **THEN** `rpm[0]` SHALL carry the live measured position
- **AND** `voltage[0]` and `current[0]` SHALL be not-a-number in that same message
- **WHEN** the steering driver link is healthy and the steering position sensor is unhealthy or
  uncalibrated
- **THEN** `rpm[0]` SHALL be `INT32_MIN` while `voltage[0]` and `current[0]` carry live readings
- **AND** the two validity gates SHALL therefore be documented as separate, so a consumer
  attributes each missing reading to the correct sensor

#### Scenario: Mark the unused ESC slots as carrying no data
- **WHEN** an `ESC_STATUS` message is packed
- **THEN** `voltage[1..3]` and `current[1..3]` SHALL be not-a-number
- **AND** `rpm[1..3]` SHALL be 0
- **AND** the companion `ESC_INFO` message SHALL declare `count = 1`, which is what tells a
  consumer that those slots are not data, including that the zeros in `rpm[1..3]` are not
  positions

#### Scenario: Timestamp from a monotonic 64-bit clock
- **WHEN** an `ESC_STATUS` message is packed
- **THEN** `time_usec` SHALL be taken from a 64-bit monotonic microsecond boot clock
  (`esp_timer_get_time()`)
- **AND** a 32-bit microsecond source SHALL NOT be used, because its wrap (about every 71 minutes)
  would present a fresh measurement as a very old one

### Requirement: Steering ESC Health and Fault Reporting via ESC_INFO
The system SHALL report the steering VESC's slow-changing health data — connection type, online
state, received-reply counter, power-stage temperature, failure flags and an error count — using
the standard `ESC_INFO` message (id 290) sent from this component on its own schedule, so a fault
or an over-temperature is visible at the ground station rather than only on the vehicle's web
portal and serial console.

#### Scenario: Emit ESC_INFO on its own interval
- **WHEN** the ESC-info interval elapses (`MAVLINK_ESC_INFO_TX_MS`, default 1000 ms / 1 Hz)
- **THEN** an `ESC_INFO` message SHALL be sent from the ESP32's own system and component id
- **AND** `index` SHALL be 0 and `count` SHALL be 1
- **AND** `connection_type` SHALL be `ESC_CONNECTION_TYPE_SERIAL`, because the driver is reached
  over a dedicated UART
- **AND** the interval SHALL be independent of the `MAVLINK_REPORT_TX_MS` tick, because the data it
  carries changes slowly and repeating it at 5 Hz would spend link budget for no information

#### Scenario: Report the ESC online state and reply counter
- **WHEN** an `ESC_INFO` message is packed
- **THEN** bit 0 of `info` SHALL be set if and only if the steering driver link is healthy — a
  valid telemetry reply was received within `STEER_VESC_COMM_TIMEOUT_MS`
- **AND** `counter` SHALL be the number of valid telemetry replies received from the VESC since
  boot, as a 16-bit value that is permitted to wrap
- **AND** the counter SHALL be incremented in the driver at the single point where a reply is
  accepted, not sampled by the reporting layer, so that no reply is missed or double-counted by
  the difference between the poll rate and the transmit rate

#### Scenario: Report power-stage temperature in centi-degrees
- **WHEN** an `ESC_INFO` message is packed
- **AND** the steering driver link is healthy
- **THEN** `temperature[0]` SHALL carry the VESC FET temperature in centi-degrees Celsius,
  rounded and clamped so that a real temperature can never equal the "not supplied" sentinel
- **AND** `temperature[1..3]` SHALL be `INT16_MAX`, the sentinel the message definition assigns to
  "data not supplied by ESC"

#### Scenario: Map the VESC fault code onto the standard failure bitmask
- **WHEN** an `ESC_INFO` message is packed
- **AND** the steering driver link is healthy
- **THEN** `failure_flags[0]` SHALL carry the VESC fault code mapped onto `ESC_FAILURE_FLAGS`:
  over-voltage and under-voltage SHALL both map to `ESC_FAILURE_OVER_VOLTAGE`, the gate-driver
  fault SHALL map to `ESC_FAILURE_GENERIC`, absolute over-current SHALL map to
  `ESC_FAILURE_OVER_CURRENT`, both over-temperature faults SHALL map to
  `ESC_FAILURE_OVER_TEMPERATURE`, no fault SHALL map to 0, and any other non-zero code SHALL map to
  `ESC_FAILURE_GENERIC`
- **AND** the under-voltage mapping SHALL be documented as lossy, because the `ESC_FAILURE_FLAGS`
  enumeration has no under-voltage bit: the category is preserved and the direction is not
- **AND** the numeric fault codes SHALL be documented as VESC-firmware-version dependent and SHALL
  be bench-verified against the flashed firmware before the mapping is trusted
- **AND** an unrecognised code SHALL degrade to `ESC_FAILURE_GENERIC` rather than to 0, so a fault
  is never reported as health
- **AND** `failure_flags[1..3]` SHALL be 0

#### Scenario: Count fault episodes since boot
- **WHEN** the decoded VESC fault code transitions from zero to non-zero
- **THEN** a fault-event counter SHALL be incremented once for that episode
- **AND** `error_count[0]` SHALL carry that counter, sent unconditionally because it is cumulative
  history rather than a live reading
- **AND** it SHALL NOT be reset when the fault clears, when the link recovers, or by any command
- **AND** `error_count[1..3]` SHALL be 0

### Requirement: Steering ESC Telemetry Validity Signalling
The system SHALL signal an unavailable steering-driver reading positively rather than by silence or
by a stale value, using the not-a-number and sentinel encodings the ESC messages define. The
electrical and health values SHALL be gated by the steering driver's own link health, and the
steering position SHALL be gated by its own sensor's health and calibration; both gates SHALL be
independent of each other, of CAN validity, of the speed sensor's validity, and of the MAVLink link
state.

#### Scenario: Report unknown rather than stale while the VESC is silent
- **WHEN** the steering driver link is unhealthy — no valid telemetry reply within
  `STEER_VESC_COMM_TIMEOUT_MS`, or no reply ever received since boot
- **THEN** `ESC_STATUS.voltage[0]` and `ESC_STATUS.current[0]` SHALL be not-a-number
- **AND** `ESC_INFO.temperature[0]` SHALL be `INT16_MAX`
- **AND** bit 0 of `ESC_INFO.info` SHALL be clear
- **AND** `ESC_INFO.failure_flags[0]` SHALL be 0, because a fault code read from a dead link is not
  a fault observation
- **AND** the last values received before the link went silent SHALL NOT be re-sent, so a consumer
  shows "--" instead of a number that is no longer true
- **AND** `ESC_STATUS.rpm[0]` SHALL be unaffected by this gate, because its source is the steering
  position sensor rather than the driver link: a dead ESC does not blind the shaft sensor

#### Scenario: Keep sending both messages while the VESC is down
- **WHEN** the steering driver link is unhealthy
- **THEN** `ESC_STATUS` and `ESC_INFO` SHALL continue to be sent on their normal intervals
- **AND** the messages SHALL NOT be suppressed, because suppression would make "this component is
  alive and its ESC is down" indistinguishable from "this component is gone", and those require
  different responses from the operator
- **AND** this SHALL be contrasted with `VISION_POSITION_DELTA`, which is suppressed instead:
  that message is a fusable measurement feeding the autopilot's estimator, where silence is the
  only safe degradation, whereas these two are display values where a positively signalled
  "unknown" is better than silence

#### Scenario: Validity is the driver's own, independent of CAN and of the speed sensor
- **WHEN** CAN `VehicleData` is invalid or the hall speed reading is invalid
- **AND** the steering driver link is healthy
- **THEN** `ESC_STATUS` and `ESC_INFO` SHALL still carry live values
- **WHEN** the steering driver link is unhealthy and CAN data is valid
- **THEN** the `EFI_STATUS` fields SHALL still carry live values while the ESC fields read as
  unknown
- **AND** the steering position in `ESC_STATUS.rpm[0]` SHALL follow its own sensor's gate in every
  one of these cases, neither forced unknown by an unhealthy driver link nor forced valid by a
  healthy one
- **AND** a consumer SHALL therefore be able to attribute a missing reading to the correct
  subsystem

#### Scenario: ESC telemetry is a reporting path only
- **WHEN** any ESC telemetry value is unavailable, out of range, or carries a fault flag
- **THEN** no steering stop, stall latch, interlock or fail-safe SHALL be triggered by the
  reporting path
- **AND** the existing fault-code stop and communication fail-safe in the steering controller
  SHALL remain the only consumers of that data for control purposes
