## ADDED Requirements

### Requirement: Autopilot Flight-Mode Awareness
The system SHALL decode the learned autopilot's `HEARTBEAT` and expose its flight mode, so that
behaviour which is only correct in MANUAL can be gated on the mode the autopilot is actually in.
The mode SHALL be observed only; the system SHALL NOT command, request or change it.

#### Scenario: Store the mode from the autopilot's heartbeat
- **WHEN** a `HEARTBEAT` arrives from the learned autopilot's system and component id
- **THEN** its `base_mode` and `custom_mode` fields SHALL be decoded and stored together with the
  time of receipt
- **AND** the heartbeat SHALL continue to serve its existing purposes — learning the autopilot and
  driving the link-loss timer — unchanged

#### Scenario: Ignore a heartbeat from any other component
- **WHEN** a `HEARTBEAT` arrives whose system or component id is not the learned autopilot's
- **THEN** the stored mode SHALL be left unchanged
- **AND** a ground station or any other component sharing the link SHALL therefore be unable to
  make the vehicle believe it is in a different mode

#### Scenario: Expose the mode and its availability
- **WHEN** a consumer asks for the autopilot's flight mode
- **THEN** the system SHALL report the stored `custom_mode`, and separately whether that value is
  currently trustworthy
- **AND** the value SHALL be reported as untrustworthy when no heartbeat has been received since
  initialisation, or when the link is down
- **AND** for an ArduPilot Rover a `custom_mode` of zero SHALL mean MANUAL

#### Scenario: Reset the mode on re-initialisation
- **WHEN** the MAVLink interface is initialised
- **THEN** the stored mode SHALL be cleared to "never received"
- **AND** consumers SHALL see the mode as unavailable until the next heartbeat from the autopilot

### Requirement: Autopilot Steering Speed-Scaling Base Parameter Subscription
The system SHALL subscribe to the autopilot's `MOT_SPD_SCA_BASE` parameter over MAVLink, as the
fallback source of the steering speed-scaling base. The subscription SHALL reuse the existing
read-only parameter-subscription mechanism rather than duplicating it, and SHALL NOT change the
behaviour of the `SPEED_MAX` subscription that mechanism already serves. The subscription SHALL be
read-only: the system SHALL NOT send `PARAM_SET` or otherwise write any autopilot parameter.

#### Scenario: Two parameters share one subscription mechanism
- **WHEN** the parameter subscription runs
- **THEN** `SPEED_MAX` and `MOT_SPD_SCA_BASE` SHALL both be subscribed through the same poll,
  the same `PARAM_VALUE` handler and the same value-hygiene rules
- **AND** each parameter SHALL keep its own stored value, its own receipt time, its own poll
  timestamp and its own upper bound
- **AND** the existing `SPEED_MAX` accessors SHALL keep their current semantics exactly, so no
  existing consumer of the speed limit changes behaviour

#### Scenario: Poll the parameter periodically
- **WHEN** the autopilot's system and component ids have been learned and the link is up
- **THEN** a `PARAM_REQUEST_READ` for `MOT_SPD_SCA_BASE` SHALL be sent with a parameter index of
  `-1` (look up by name), addressed to the learned autopilot
- **AND** the first request SHALL be sent `MAVLINK_PARAM_FIRST_DELAY_MS` after the autopilot was
  learned and subsequent requests every `MAVLINK_PARAM_POLL_MS`, as for `SPEED_MAX`
- **AND** the requests for the two parameters SHALL NOT be emitted in the same loop iteration, so
  two `PARAM_REQUEST_READ` frames do not contend for the same window on a 115200-baud link
- **AND** polling SHALL NOT stop once a value has been received, because the poll — not any
  broadcast — is what detects a change made from the ground station

#### Scenario: Apply the same value hygiene
- **WHEN** a `PARAM_VALUE` for `MOT_SPD_SCA_BASE` arrives
- **THEN** it SHALL be accepted only from the learned autopilot's system and component id
- **AND** the 16-byte `param_id` SHALL be copied into a 17-byte buffer and terminated before any
  string comparison
- **AND** a value that is not-a-number, infinite, negative, or greater than
  `MAVLINK_PARAM_SPD_SCA_BASE_MAX` SHALL be rejected with a log and SHALL NOT be stored
- **AND** a rejected value SHALL leave the previously stored value and its timestamp unchanged, so
  one corrupt frame cannot invalidate a good subscription

#### Scenario: Store and expose an accepted value
- **WHEN** a `PARAM_VALUE` for `MOT_SPD_SCA_BASE` passes every check
- **THEN** the value SHALL be stored in metres per second together with the time of receipt
- **AND** the system SHALL expose it in metres per second together with the age of the reading
- **AND** a change SHALL be logged only when the new value differs from the previous one by more
  than `MAVLINK_PARAM_EPSILON_MS`, so a five-second poll of an unchanged parameter does not fill
  the console

#### Scenario: Report the parameter as unavailable when it cannot be trusted
- **WHEN** the vehicle layer asks whether a `MOT_SPD_SCA_BASE` value is available
- **THEN** the answer SHALL be true only if a value has been received, the value is greater than
  zero, the link is up, and the reading is younger than `MAVLINK_PARAM_STALE_MS`
- **AND** a value of zero SHALL be reported as unavailable, because a non-positive base means "no
  scaling" both to ArduPilot and to this vehicle
- **AND** "unavailable" SHALL result in the vehicle layer falling back to its own stored base, or
  to no scaling at all when it has none

#### Scenario: Surface the subscription state on the diagnostic line
- **WHEN** the periodic (1 Hz) MAVLink debug line is emitted
- **THEN** it SHALL include the last received `MOT_SPD_SCA_BASE`, the age of that reading and
  whether the value is currently considered available
- **AND** it SHALL also include the autopilot's stored flight mode, so "not scaling because the
  autopilot is not in MANUAL" can be distinguished from "not scaling because there is no base" on
  the serial console alone

### Requirement: Steering Scale Telemetry via NAMED_VALUE_FLOAT
The system SHALL report the steering speed-scaling factor it is currently applying to the MAVLink
network as a `NAMED_VALUE_FLOAT` (id 251) named `STEER_SCA`, sent from this component. This name
SHALL be admitted under the same scoped exception that admits the steering VESC values, and the
exception SHALL remain closed to every other value: engine, gear, digital-flag, odometer and trip
values SHALL continue to be carried in distinct fields of the single `EFI_STATUS` message.
`STEER_SCA` SHALL NOT be a control input.

#### Scenario: Report the applied scale at 1 Hz
- **WHEN** the slow steering telemetry interval elapses (`MAVLINK_STEER_SLOW_TX_MS`, default
  1000 ms)
- **THEN** a `NAMED_VALUE_FLOAT` named `STEER_SCA` SHALL be sent from the ESP32's own system and
  component id (system 1, component 25), alongside `VESC_TEMP` and `VESC_OK`
- **AND** its value SHALL be the scale actually applied to the steering command after rate
  limiting, in the range 0 to 1 inclusive
- **AND** its `time_boot_ms` SHALL carry the milliseconds-since-boot value already computed for
  that tick

#### Scenario: One means "not scaling", and it is never NaN
- **WHEN** the steering scaling is inactive for any reason — no base value, an invalid speed
  reading, the autopilot not in MANUAL, or a speed at or below the base
- **THEN** `STEER_SCA` SHALL be reported as `1.0`
- **AND** it SHALL NEVER be `NaN`, because "not scaling" is a definite answer and a ground station
  should plot it as a flat line at one rather than as a gap in the record

#### Scenario: The value reflects the command path that is actually in use
- **WHEN** the active command source is not the autopilot
- **THEN** `STEER_SCA` SHALL be `1.0`, matching the fact that the web steering path is never scaled
- **AND** the message SHALL keep being sent regardless of the command source, so the record has no
  holes

## MODIFIED Requirements

### Requirement: Steering VESC Telemetry via NAMED_VALUE_FLOAT
The system SHALL report the steering VESC's electrical and thermal telemetry, its link health, the
measured position of the steering axis that VESC drives, and the steering speed-scale being applied
to that axis's commands, to the MAVLink network as **exactly six** `NAMED_VALUE_FLOAT` messages
(id 251) sent from this component. These six named values SHALL
be the ONLY `NAMED_VALUE_FLOAT` this component ever sends: they are a **scoped exception**, granted
because the standard ESC telemetry messages are absent from the dialect the ground station decodes
with and are therefore discarded before any consumer can read them. The exception SHALL NOT extend
to any engine, gear, digital-flag, odometer or trip value, all of which SHALL continue to be carried
in distinct fields of the single `EFI_STATUS` message. Every reported value SHALL be gated by the
validity of its OWN source, SHALL be reported as `NaN` rather than as a stale or zero reading when
that source is unhealthy, and SHALL keep being sent regardless of source health — except the link
flag and the steering scale, which SHALL never be `NaN`. None of these values SHALL be a control
input.

#### Scenario: Five named values from this component
- **WHEN** the interface is reporting vehicle state
- **THEN** it SHALL send exactly six `NAMED_VALUE_FLOAT` names from the ESP32's own system and
  component id (system 1, component 25): `STEER_POS`, `STEER_A`, `VESC_V`, `VESC_TEMP`,
  `VESC_OK` and `STEER_SCA`
- **AND** `STEER_POS` SHALL carry the MEASURED steering position as a percentage of the CALIBRATED
  lock-to-lock range, −100 at the left limit, 0 at centre, +100 at the right limit — percent of
  calibrated travel, NOT degrees, because the firmware holds no counts-to-degrees calibration
- **AND** a negative `STEER_POS` SHALL mean LEFT of centre and a positive value SHALL mean RIGHT of
  centre, matching the sign convention of the steering-percent accessor
- **AND** `STEER_A` SHALL carry the VESC's average MOTOR current in amperes, which is the steering
  load and stall diagnostic; the VESC's average INPUT current SHALL NOT be reported under this name
- **AND** `VESC_V` SHALL carry the input voltage measured by the VESC itself — the 24 V boost rail
  that powers the steering driver — in volts
- **AND** `VESC_TEMP` SHALL carry the VESC's power-stage (FET) temperature in degrees Celsius
- **AND** `VESC_OK` SHALL carry the steering driver's link health as `1.0` or `0.0`
- **AND** `STEER_SCA` SHALL carry the steering speed-scale the vehicle layer is applying to
  autopilot steering commands after rate limiting, in the range 0 to 1 inclusive, `1.0` meaning
  "not scaling"; it is sourced from the vehicle layer rather than from the VESC and SHALL NOT be
  gated by the steering driver's link health
- **AND** the VESC's raw fault code and any boot-cumulative reply or fault counters SHALL NOT be
  reported on this link; they remain available on the web portal and the serial console

#### Scenario: Three names on the report tick and two on the slow timer
- **WHEN** the engine/state report interval elapses (`MAVLINK_REPORT_TX_MS`, default 200 ms / 5 Hz)
- **THEN** `STEER_POS`, `STEER_A` and `VESC_V` SHALL be sent in that same tick, after `VFR_HUD`, so
  a ground-station log lines the live values up with `EFI_STATUS` without interpolation
- **AND** `VESC_TEMP`, `VESC_OK` and `STEER_SCA` SHALL be sent on their own slower interval
  (`MAVLINK_STEER_SLOW_TX_MS`, default 1000 ms / 1 Hz), because a power stage's thermal time
  constant is seconds, a link flag needs no faster rate, and the steering scale moves at the pace
  the vehicle accelerates
- **AND** the `time_boot_ms` field of all six SHALL carry the milliseconds-since-boot value already
  computed for that report tick; a microsecond clock SHALL NOT be used, and the 49.7-day wrap of
  that field is its defined behaviour and SHALL NOT be worked around

#### Scenario: Electrical and thermal values are NaN while the VESC is silent
- **WHEN** no valid VESC reply has arrived within the steering driver's communication timeout, or
  no reply has ever arrived since boot
- **THEN** `STEER_A`, `VESC_V` and `VESC_TEMP` SHALL each be reported as `NaN`
- **AND** the last value seen SHALL NOT be repeated, and zero SHALL NOT be substituted, because a
  stale or zero electrical reading is indistinguishable from a real one at the ground station
- **AND** a consumer SHALL render `NaN` as "no data" rather than as a number

#### Scenario: VESC_OK is the validity flag and is never NaN
- **WHEN** any `VESC_OK` message is sent
- **THEN** its value SHALL be `1.0` when a valid VESC reply arrived within the steering driver's
  communication timeout and `0.0` when it did not
- **AND** it SHALL NEVER be `NaN`, because it is the value that TELLS a consumer why the other
  VESC-sourced names went `NaN`, and an "unknown" there would defeat its only purpose
- **AND** `VESC_OK` = `0.0` SHALL be read as "the steering driver is offline", not as "the
  peripheral is offline"
- **AND** `STEER_SCA` SHALL likewise NEVER be `NaN`: every condition that disables the scaling
  resolves to `1.0`, because "not scaling" is a definite answer and a ground station should plot it
  as a flat line at one rather than as a gap in the record

#### Scenario: Steering position validity is the sensor's, independent of the driver link
- **WHEN** `STEER_POS` is sent
- **THEN** its validity gate SHALL be the position sensor's own — the sensor reading healthy AND the
  steering calibrated — and SHALL NOT be the steering driver's link health
- **AND** `STEER_POS` SHALL carry a live position while `STEER_A`, `VESC_V` and `VESC_TEMP` are
  `NaN` and `VESC_OK` is `0.0`, because a silent driver does not blind the shaft sensor
- **AND** `STEER_POS` SHALL be `NaN` while those same values are live, whenever the sensor is
  unhealthy or the steering is uncalibrated
- **AND** `0.0` SHALL be reported as a REAL reading meaning "straight ahead"; an unknown position
  SHALL NEVER be reported as `0.0`
- **AND** a consumer SHALL NOT infer the state of either subsystem from the other

#### Scenario: Keep sending while the VESC is down
- **WHEN** the steering driver is offline for any length of time, including from boot
- **THEN** all six names SHALL keep being sent at their normal rates, carrying `NaN` and
  `VESC_OK` = `0.0` as appropriate, with `STEER_SCA` unaffected because its source is the vehicle
  layer and not the VESC
- **AND** transmission SHALL NOT be suppressed, because silence would make "this component is alive
  and its driver is down" indistinguishable from "this component is gone", and the two require
  different operator responses
- **AND** this SHALL remain the deliberate opposite of the external-navigation policy, where a
  fusable measurement goes silent rather than sending a doubtful value

#### Scenario: Names are unique within the ten-character field
- **WHEN** a named value is packed
- **THEN** the name SHALL be zero-padded into a ten-byte buffer before packing, because the wire
  field is a fixed ten bytes and is copied in full
- **AND** every name SHALL be a compile-time literal of at most ten characters, checked at compile
  time; names SHALL NOT be constructed at runtime, because truncation of an over-long name would be
  silent
- **AND** all six names SHALL be distinct within their first ten characters
- **AND** a consumer SHALL trim the name by length rather than by a terminator, because a
  ten-character name carries no terminator

#### Scenario: Scoped exception to the single-message rule
- **WHEN** vehicle telemetry is reported
- **THEN** engine RPM, coolant temperature, intake air temperature, manifold pressure, ECU load,
  measured and commanded throttle, module voltage, both gear values, the digital-output bitmask, the
  total odometer and the trip distance SHALL continue to be carried in distinct fields of the single
  `EFI_STATUS` message and SHALL NOT be sent as `NAMED_VALUE_FLOAT`, because those messages share one
  message id and collide by name in any name-agnostic store
- **AND** the six steering VESC, steering-position and steering-scale values named above SHALL be
  the ONLY exception, and the ONLY `NAMED_VALUE_FLOAT` this component sends
- **AND** the exception SHALL be justified by the standard ESC telemetry messages being undecodable
  at the ground station rather than by preference, and SHALL be documented at the point where the
  messages are packed
- **AND** a consumer SHALL filter on this component's id, because message id 251 may be sent by
  other components on the same link

#### Scenario: The reported voltage is not the ECU module voltage
- **WHEN** both `VESC_V` and `EFI_STATUS` are sent in the same interval tick
- **THEN** `VESC_V` SHALL be the VESC-measured steering supply rail
- **AND** `EFI_STATUS.ignition_voltage` SHALL remain the ECU control module supply voltage
  (OBD-II PID `0x42`)
- **AND** the two SHALL NOT be treated as the same measurement, because they are different physical
  quantities on different rails

#### Scenario: Telemetry is a reporting path only
- **WHEN** any of the six named values is at any value, including `NaN` or `VESC_OK` = `0.0`
- **THEN** no steering stop, stall latch, interlock, fail-safe or other control action SHALL be
  triggered by the act of reporting it
- **AND** the existing fault-code stop and communication fail-safe inside the steering controller
  SHALL remain the only consumers of that data for control purposes
- **AND** removing or changing this reporting path SHALL NOT alter any control behaviour
