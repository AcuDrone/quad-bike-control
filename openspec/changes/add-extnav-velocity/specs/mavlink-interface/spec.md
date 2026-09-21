## ADDED Requirements

### Requirement: External Navigation Odometry Reporting
The system SHALL report the hall-effect wheel speed to the autopilot as a fusable body-frame
distance increment using the `VISION_POSITION_DELTA` message, so the EKF3 can use it as a
body-odometry aiding source on a vehicle without reliable GPS. The measurement SHALL be suppressed
entirely rather than reported as zero whenever its validity or its sign is in doubt, and a
suppressed interval SHALL NOT be folded into a later increment.

#### Scenario: Report wheel travel as a body-frame position delta
- **WHEN** the report interval elapses (`MAVLINK_REPORT_TX_MS`, default 200 ms / 5 Hz)
- **AND** the link is up and the speed reading is valid
- **AND** the travel direction is known, or the vehicle is not rolling in neutral
- **AND** a valid integration baseline exists (see the re-baseline scenario)
- **THEN** a `VISION_POSITION_DELTA` message SHALL be sent from the ESP32's component id
- **AND** the signed longitudinal speed SHALL be the sensor speed in metres per second multiplied
  by the travel direction (`speedMs · travelDirection`)
- **AND** `position_delta` SHALL be that signed speed multiplied by the integration interval,
  placed on the body-frame forward axis, with the lateral and vertical components exactly zero
- **AND** the velocity SHALL NOT be rotated into the earth frame, and no heading SHALL be required
  to send the message, because the autopilot rotates the body-frame increment with its own
  attitude

#### Scenario: Declare no measured rotation
- **WHEN** a `VISION_POSITION_DELTA` is packed
- **THEN** `angle_delta` SHALL be `{0, 0, 0}`
- **AND** a rotation SHALL NOT be fabricated from any other source, because the wheel sensor
  measures no rotation and the autopilot's own gyros own that channel

#### Scenario: Report the actually measured integration interval
- **WHEN** a `VISION_POSITION_DELTA` is packed
- **THEN** `time_delta_usec` SHALL be the measured elapsed time since the previous transmitted
  delta, taken from the same monotonic microsecond clock
- **AND** a nominal or assumed interval SHALL NOT be substituted, because the autopilot divides
  the position delta by exactly this value to recover a velocity, so an assumed interval would
  inject a proportional velocity error
- **AND** `time_usec` SHALL be taken from a 64-bit monotonic microsecond clock
  (`esp_timer_get_time()`), never a 32-bit microsecond source whose wrap (about every 71 minutes)
  would present a fresh measurement as a very old one
- **AND** no attempt SHALL be made to convert to autopilot time — the autopilot corrects the boot
  offset itself

#### Scenario: Re-baseline the interval after a break
- **WHEN** the message has been suppressed by any gate, or no delta has ever been sent, or the
  elapsed time since the previous delta exceeds `MAVLINK_VISO_MAX_DT_US`
- **THEN** the next opportunity SHALL NOT send a message
- **AND** it SHALL instead re-establish the integration baseline from the current clock, so that
  exactly one interval is discarded and the following message is integrated over an interval that
  was actually observed
- **AND** the current speed SHALL NOT be multiplied by the elapsed time across the break, because
  a delta is an integral and reporting travel over an unobserved interval would inject a large
  spurious position step

#### Scenario: Declare full confidence when the message is sent at all
- **WHEN** a `VISION_POSITION_DELTA` is packed
- **THEN** `confidence` SHALL be `MAVLINK_VISO_CONFIDENCE` (100)
- **AND** a reduced confidence SHALL NOT be used to signal degraded health, because the health
  policy is suppression: a message that is sent has passed every gate
- **AND** this SHALL keep the autopilot's derived velocity error at the tight end of its
  configured range, satisfying the autopilot's requirement that the error be below 1 m/s before
  body-odometry aiding may begin

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
- **THEN** no `VISION_POSITION_DELTA` SHALL be sent for that tick
- **AND** the integration baseline SHALL be invalidated
- **AND** a zero delta SHALL NOT be substituted, because an invalid reading includes the sensor's
  wire-fault (suspicious) latch, where a disconnected sensor decays to zero while the vehicle is
  still moving — fusing that as "stopped" would corrupt the state estimate
- **AND** the other outbound messages (`HEARTBEAT`, `EFI_STATUS`, `VFR_HUD`) SHALL continue to be
  sent unaffected

#### Scenario: Suppress the message when rolling in neutral
- **WHEN** the report interval elapses
- **AND** the travel direction is `0` (neutral, or gear unknown)
- **AND** the speed reading exceeds `MAVLINK_VISO_NEUTRAL_ZERO_MS`
- **THEN** no `VISION_POSITION_DELTA` SHALL be sent for that tick, because the vehicle is moving
  with no recoverable direction sign and a wrong sign would inject an error of twice the speed
- **AND** the integration baseline SHALL be invalidated

#### Scenario: Send healthy zero-motion updates
- **WHEN** the report interval elapses
- **AND** the link is up and the speed reading is valid
- **AND** the vehicle is stationary — including stationary in neutral, with a speed at or below
  `MAVLINK_VISO_NEUTRAL_ZERO_MS`
- **THEN** a `VISION_POSITION_DELTA` carrying a zero position delta SHALL be sent
- **AND** it SHALL NOT be suppressed, because with no reliable GPS a zero-motion update is the
  strongest available constraint on estimator drift while the vehicle is parked or idling

#### Scenario: Require no inbound subscription
- **WHEN** the external navigation odometry feature is active
- **THEN** the interface SHALL NOT subscribe to, request, or depend upon the autopilot's
  `ATTITUDE` message or any other inbound message beyond the existing `HEARTBEAT` and
  `SERVO_OUTPUT_RAW`
- **AND** the stream re-request logic SHALL therefore gate on the command stream rate alone

#### Scenario: Expose the gating state on the debug log
- **WHEN** the periodic MAVLink debug line is emitted (1 Hz, `DebugFeature::MAVLINK`)
- **THEN** it SHALL include the count of `VISION_POSITION_DELTA` messages sent and the last
  transmitted integration interval
- **AND** an operator SHALL therefore be able to tell, from serial output alone and without a
  ground station, whether the message is flowing and whether the baseline is being repeatedly
  re-established by a flapping gate
