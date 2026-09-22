## ADDED Requirements

### Requirement: Steering VESC Telemetry via NAMED_VALUE_FLOAT
The system SHALL report the steering VESC's electrical and thermal telemetry, its link health, and
the measured position of the steering axis that VESC drives, to the MAVLink network as **exactly
five** `NAMED_VALUE_FLOAT` messages (id 251) sent from this component. These five named values SHALL
be the ONLY `NAMED_VALUE_FLOAT` this component ever sends: they are a **scoped exception**, granted
because the standard ESC telemetry messages are absent from the dialect the ground station decodes
with and are therefore discarded before any consumer can read them. The exception SHALL NOT extend
to any engine, gear, digital-flag, odometer or trip value, all of which SHALL continue to be carried
in distinct fields of the single `EFI_STATUS` message. Every reported value SHALL be gated by the
validity of its OWN source, SHALL be reported as `NaN` rather than as a stale or zero reading when
that source is unhealthy, and SHALL keep being sent regardless of source health — except the link
flag, which SHALL never be `NaN`. None of these values SHALL be a control input.

#### Scenario: Five named values from this component
- **WHEN** the interface is reporting vehicle state
- **THEN** it SHALL send exactly five `NAMED_VALUE_FLOAT` names from the ESP32's own system and
  component id (system 1, component 25): `STEER_POS`, `STEER_A`, `VESC_V`, `VESC_TEMP` and
  `VESC_OK`
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
- **AND** the VESC's raw fault code and any boot-cumulative reply or fault counters SHALL NOT be
  reported on this link; they remain available on the web portal and the serial console

#### Scenario: Three names on the report tick and two on the slow timer
- **WHEN** the engine/state report interval elapses (`MAVLINK_REPORT_TX_MS`, default 200 ms / 5 Hz)
- **THEN** `STEER_POS`, `STEER_A` and `VESC_V` SHALL be sent in that same tick, after `VFR_HUD`, so
  a ground-station log lines the live values up with `EFI_STATUS` without interpolation
- **AND** `VESC_TEMP` and `VESC_OK` SHALL be sent on their own slower interval
  (`MAVLINK_STEER_SLOW_TX_MS`, default 1000 ms / 1 Hz), because a power stage's thermal time
  constant is seconds and a link flag needs no faster rate
- **AND** the `time_boot_ms` field of all five SHALL carry the milliseconds-since-boot value already
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
- **THEN** all five names SHALL keep being sent at their normal rates, carrying `NaN` and
  `VESC_OK` = `0.0` as appropriate
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
- **AND** all five names SHALL be distinct within their first ten characters
- **AND** a consumer SHALL trim the name by length rather than by a terminator, because a
  ten-character name carries no terminator

#### Scenario: Scoped exception to the single-message rule
- **WHEN** vehicle telemetry is reported
- **THEN** engine RPM, coolant temperature, intake air temperature, manifold pressure, ECU load,
  measured and commanded throttle, module voltage, both gear values, the digital-output bitmask, the
  total odometer and the trip distance SHALL continue to be carried in distinct fields of the single
  `EFI_STATUS` message and SHALL NOT be sent as `NAMED_VALUE_FLOAT`, because those messages share one
  message id and collide by name in any name-agnostic store
- **AND** the five steering VESC and steering-position values named above SHALL be the ONLY
  exception, and the ONLY `NAMED_VALUE_FLOAT` this component sends
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
- **WHEN** any of the five named values is at any value, including `NaN` or `VESC_OK` = `0.0`
- **THEN** no steering stop, stall latch, interlock, fail-safe or other control action SHALL be
  triggered by the act of reporting it
- **AND** the existing fault-code stop and communication fail-safe inside the steering controller
  SHALL remain the only consumers of that data for control purposes
- **AND** removing or changing this reporting path SHALL NOT alter any control behaviour
