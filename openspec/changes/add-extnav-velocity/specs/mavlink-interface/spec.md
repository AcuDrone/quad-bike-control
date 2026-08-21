## ADDED Requirements

### Requirement: Autopilot Attitude Subscription
The system SHALL subscribe to the autopilot's `ATTITUDE` message and maintain a fresh yaw value,
so that a scalar wheel speed can be rotated into the earth NED frame. Yaw SHALL be accepted only
from the learned autopilot, and SHALL be treated as unusable once stale.

#### Scenario: Request the attitude stream
- **WHEN** the autopilot addressing is known and the link is up
- **AND** the measured `ATTITUDE` rate is below `MAVLINK_ATTITUDE_MIN_RATE_HZ`
- **THEN** the ESP32 SHALL send `REQUEST_DATA_STREAM` for `MAV_DATA_STREAM_EXTRA1` and
  `MAV_CMD_SET_MESSAGE_INTERVAL` for `ATTITUDE`, both at `MAVLINK_ATTITUDE_RATE_HZ` (default
  10 Hz), addressed to the learned autopilot system/component id
- **AND** the request SHALL be repeated no more often than `MAVLINK_STREAM_REREQUEST_MS`, so a
  lost or rejected request at boot and an autopilot reboot both recover automatically

#### Scenario: Accept yaw only from the autopilot
- **WHEN** an `ATTITUDE` message is received
- **AND** its `system_id` equals the learned autopilot system id **AND** its `component_id` is
  `MAV_COMP_ID_AUTOPILOT1`
- **THEN** the `yaw` field (radians, NED) SHALL be stored as the current heading and its arrival
  timestamp recorded
- **WHEN** an `ATTITUDE` message arrives from any other system or component (companion computer,
  gimbal, ground-station tool)
- **THEN** it SHALL be ignored and SHALL NOT update the stored yaw or its timestamp

#### Scenario: Detect stale yaw
- **WHEN** no accepted `ATTITUDE` message has arrived for `MAVLINK_ATTITUDE_TIMEOUT_MS`
  (default 500 ms, about five missed frames at the requested rate)
- **THEN** the yaw SHALL be reported as not fresh
- **AND** the yaw age in milliseconds SHALL be available to callers
- **AND** the yaw SHALL likewise be not fresh before the first `ATTITUDE` message has ever been
  received, rather than defaulting to a heading of zero

#### Scenario: Track command and attitude stream rates independently
- **WHEN** the rolling one-second rate window elapses
- **THEN** the command (`SERVO_OUTPUT_RAW`) rate and the `ATTITUDE` rate SHALL both be updated
  from that same window
- **AND** each stream SHALL be re-requested according to its own rate threshold, so a healthy
  command stream cannot suppress re-requesting a dead attitude stream, nor the reverse

### Requirement: External Navigation Velocity Reporting
The system SHALL report the hall-effect wheel speed to the autopilot as a fusable earth-frame
velocity using the `VISION_SPEED_ESTIMATE` message, so the EKF3 can use it as an external
navigation velocity source on a vehicle without reliable GPS. The measurement SHALL be suppressed
entirely rather than reported as zero whenever its validity, its heading rotation, or its sign is
in doubt.

#### Scenario: Report wheel speed as an earth-frame NED velocity
- **WHEN** the report interval elapses (`MAVLINK_REPORT_TX_MS`, default 200 ms / 5 Hz)
- **AND** the link is up, the speed reading is valid, and the yaw is fresh
- **AND** the travel direction is known, or the vehicle is not rolling in neutral
- **THEN** a `VISION_SPEED_ESTIMATE` message SHALL be sent from the ESP32's component id
- **AND** the signed longitudinal speed SHALL be the sensor speed in metres per second multiplied
  by the travel direction (`speedKmh / 3.6 · travelDirection`)
- **AND** the velocity SHALL be reported in the earth NED frame as `x = v · cos(yaw)`,
  `y = v · sin(yaw)`, `z = 0`, using the yaw obtained from the autopilot's `ATTITUDE`
- **AND** the vehicle-frame speed SHALL NOT be sent unrotated, because the autopilot's
  `VISO_ORIENT` parameter is not applied to the velocity path

#### Scenario: Timestamp with a monotonic microsecond clock
- **WHEN** a `VISION_SPEED_ESTIMATE` is packed
- **THEN** `usec` SHALL be taken from a 64-bit monotonic microsecond clock
  (`esp_timer_get_time()`)
- **AND** a 32-bit microsecond source SHALL NOT be used, because its wrap (about every 71
  minutes) would present a fresh measurement as a very old one
- **AND** no attempt SHALL be made to convert to autopilot time — the autopilot corrects the boot
  offset itself

#### Scenario: Declare unknown covariance and no frame reset
- **WHEN** a `VISION_SPEED_ESTIMATE` is packed
- **THEN** `covariance[0]` SHALL be `NaN`, so the autopilot applies its own configured velocity
  noise (`VISO_VEL_M_NSE`) instead of a fabricated per-sample value
- **AND** `reset_counter` SHALL be 0 and SHALL never be incremented, since a wheel sensor has no
  estimator frame that can reset

#### Scenario: Derive travel direction from the physical gear
- **WHEN** the travel direction is determined
- **THEN** it SHALL be derived from the physically sensed gear (the opto gear switches), mapping
  REVERSE to `-1`, LOW and HIGH to `+1`, and NEUTRAL or UNKNOWN to `0`
- **AND** it SHALL NOT be derived from the assumed or commanded gear, because the transmission
  command path is sensorless and an assumption must never sign a fused measurement
- **AND** a gear reading that is briefly UNKNOWN during a shift SHALL yield direction `0`, which
  suppresses the sample rather than guessing its sign

#### Scenario: Suppress the message when the link is down or the speed is invalid
- **WHEN** the report interval elapses
- **AND** the autopilot link is down, **OR** the hall speed reading is invalid or stale
- **THEN** no `VISION_SPEED_ESTIMATE` SHALL be sent for that tick
- **AND** a zero velocity SHALL NOT be substituted, because an invalid reading includes the
  sensor's wire-fault (suspicious) latch, where a disconnected sensor decays to zero while the
  vehicle is still moving — fusing that as "stopped" would corrupt the state estimate
- **AND** the other outbound messages (`HEARTBEAT`, `EFI_STATUS`, `VFR_HUD`) SHALL continue to be
  sent unaffected

#### Scenario: Suppress the message when yaw is stale
- **WHEN** the report interval elapses
- **AND** the yaw from the autopilot is not fresh
- **THEN** no `VISION_SPEED_ESTIMATE` SHALL be sent for that tick
- **AND** the last known yaw SHALL NOT be reused, because a velocity rotated by an outdated
  heading is indistinguishable to the autopilot from a genuine change of direction

#### Scenario: Suppress the message when rolling in neutral
- **WHEN** the report interval elapses
- **AND** the travel direction is `0` (neutral, or gear unknown)
- **AND** the speed reading exceeds `MAVLINK_VISO_NEUTRAL_ZERO_KMH`
- **THEN** no `VISION_SPEED_ESTIMATE` SHALL be sent for that tick, because the vehicle is moving
  with no recoverable direction sign and a wrong sign would inject an error of twice the speed

#### Scenario: Send healthy zero-velocity updates
- **WHEN** the report interval elapses
- **AND** the link is up, the speed reading is valid and the yaw is fresh
- **AND** the vehicle is stationary — including stationary in neutral, with a speed at or below
  `MAVLINK_VISO_NEUTRAL_ZERO_KMH`
- **THEN** a `VISION_SPEED_ESTIMATE` carrying a zero velocity SHALL be sent
- **AND** it SHALL NOT be suppressed, because with no reliable GPS a zero-velocity update is the
  strongest available constraint on estimator drift while the vehicle is parked or idling

#### Scenario: Expose the gating state on the debug log
- **WHEN** the periodic MAVLink debug line is emitted (1 Hz, `DebugFeature::MAVLINK`)
- **THEN** it SHALL include the count of `VISION_SPEED_ESTIMATE` messages sent, the age or
  staleness of the yaw, and the measured `ATTITUDE` rate
- **AND** an operator SHALL therefore be able to tell, from serial output alone and without a
  ground station, whether the message is flowing and which gate is suppressing it if not
