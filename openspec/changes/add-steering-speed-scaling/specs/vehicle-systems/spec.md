## ADDED Requirements

### Requirement: Speed-Scaled Steering Authority
The system SHALL reduce the steering authority of autopilot-sourced steering commands as road
speed rises, using ArduPilot's own formula and the locally measured, locally validated wheel
speed. The scaling SHALL be applied on the ESP32 rather than on the autopilot, because the
autopilot's speed source silently falls back to raw GPS ground speed when its velocity estimate is
unavailable — which is precisely when this vehicle's speed source has failed. The scaling SHALL be
treated as a comfort and assist feature: **every fault condition SHALL result in no scaling at
all**, never in a reduced, held or substituted authority.

#### Scenario: Scale the deviation from centre
- **WHEN** the scaling is active and a steering command is received from the autopilot
- **THEN** the scale SHALL be computed as `min(1, base / speed)`, where `speed` is the measured
  road speed in metres per second and `base` is the speed-scaling base value in metres per second
- **AND** the commanded steering percentage SHALL be multiplied by that scale
- **AND** because the steering percentage is already signed about a centre of zero, this SHALL
  scale the deviation from centre and SHALL leave a centred command centred

#### Scenario: At or below the base speed there is no reduction
- **WHEN** the measured speed is at or below the base value
- **THEN** the scale SHALL be 1
- **AND** the steering command SHALL pass through unmodified
- **AND** a stationary vehicle and a reading that has decayed to zero SHALL therefore need no
  special case, because both satisfy this condition

#### Scenario: A non-positive base disables the scaling
- **WHEN** the base value is zero or negative
- **THEN** the scale SHALL be 1
- **AND** this SHALL match ArduPilot's own reading of a non-positive `MOT_SPD_SCA_BASE`

#### Scenario: An invalid speed reading disables the scaling entirely
- **WHEN** the speed sensor reports that its reading is not valid — for any reason, including
  never having pulsed since boot, a latched implausible-loss flag, or the sensor not being
  initialised
- **THEN** the scale SHALL be 1 and the steering command SHALL pass through unmodified
- **AND** the system SHALL NOT hold the last computed scale, SHALL NOT substitute a conservative
  or last-known speed, and SHALL NOT apply any minimum-scale floor
- **AND** a warning SHALL be logged no more often than `STEER_SCALE_WARN_MS`
- **AND** this SHALL be the case even while the vehicle is moving fast, because a fault must never
  be able to leave the driver with restricted steering

#### Scenario: Scaling applies only in the autopilot's MANUAL mode
- **WHEN** the learned autopilot reports a Rover flight mode other than MANUAL
- **THEN** the scale SHALL be 1
- **AND** the steering command SHALL pass through unmodified, because in those modes the autopilot
  computes steering from speed itself and a second, invisible reduction on top of it would be
  double limiting

#### Scenario: An unknown or stale mode is treated as MANUAL
- **WHEN** no flight mode has been received from the autopilot, or the heartbeat carrying it is
  stale
- **THEN** the mode SHALL be treated as MANUAL and the scaling SHALL apply
- **AND** this SHALL be the single place where an unavailable input does not disable the scaling,
  because MANUAL is the mode this vehicle is driven in and the cost of guessing wrong is only that
  a mode which would have been scaled by the autopilot is scaled here instead

#### Scenario: The web steering path is never scaled
- **WHEN** a steering command arrives from the web portal rather than from the autopilot
- **THEN** it SHALL be passed to the steering controller unscaled
- **AND** the scaling SHALL remain a property of the autopilot command path only, because the web
  path is a bench and maintenance control with no road speed behind it

#### Scenario: The applied scale is rate-limited in both directions
- **WHEN** the computed target scale differs from the scale currently applied
- **THEN** the applied scale SHALL move toward the target by at most `STEER_SCALE_SLEW_PER_S` per
  second, whether rising or falling
- **AND** the rate limit SHALL be applied to the scale and not to the steering command, so the
  driver's own steering movements are never slowed by the assist
- **AND** the elapsed time used SHALL be capped at one second, and the first evaluation after the
  scaling becomes active SHALL adopt the target directly, so a stalled or restarted loop cannot
  produce a step change through an unbounded interval

#### Scenario: Log a scale change once
- **WHEN** the applied scale changes by more than `STEER_SCALE_LOG_EPSILON`, or changes between
  "scaling" and "not scaling"
- **THEN** the new scale SHALL be logged once, together with the speed and the base that produced
  it and the source of that base
- **AND** a further log SHALL be suppressed until at least `STEER_SCALE_LOG_MIN_MS` has passed
- **AND** while a log is suppressed the remembered scale SHALL NOT be updated, so the settled value
  is logged on the next opportunity rather than lost

#### Scenario: The autopilot's own scaling must be switched off by the operator
- **WHEN** this firmware is deployed
- **THEN** the documentation SHALL require `MANUAL_OPTIONS = 0` on the autopilot as a commissioning
  step, so the steering is not scaled twice
- **AND** the firmware SHALL NOT write that parameter, or any other autopilot parameter, because
  the firmware never sends `PARAM_SET`

### Requirement: Steering Speed-Scaling Base Value
The speed-scaling base SHALL be taken from a locally stored value first and from the autopilot's
`MOT_SPD_SCA_BASE` second, and SHALL be absent — meaning no scaling — when neither is available.
The local value SHALL be persisted in non-volatile storage and SHALL be settable from the web
portal without a reflash.

#### Scenario: The locally stored value takes priority
- **WHEN** a base value is needed and the non-volatile key `steer_sca_base` holds a positive value
  in metres per second
- **THEN** that value SHALL be used
- **AND** the autopilot's `MOT_SPD_SCA_BASE` SHALL NOT be consulted, so a vehicle can be
  commissioned and driven with the scaling working before its autopilot has been touched

#### Scenario: Fall back to the autopilot parameter
- **WHEN** the locally stored value is absent or zero
- **AND** the MAVLink interface reports a usable `MOT_SPD_SCA_BASE`
- **THEN** that value SHALL be used as the base
- **AND** it SHALL be used in metres per second, unconverted, because it is already the firmware's
  internal speed unit

#### Scenario: Neither source means no scaling
- **WHEN** the locally stored value is absent or zero
- **AND** the MAVLink interface reports no usable `MOT_SPD_SCA_BASE` — never received, zero,
  rejected, stale, or the link is down
- **THEN** the base SHALL be reported as zero and the scale SHALL be 1
- **AND** a warning SHALL be logged no more often than `STEER_SCALE_WARN_MS`
- **AND** the system SHALL NOT substitute a compile-time default, because a base nobody chose
  would silently restrict the steering of an unconfigured vehicle

#### Scenario: Load and validate the stored base on startup
- **WHEN** the vehicle controller initialises
- **THEN** `steer_sca_base` SHALL be read from non-volatile storage
- **AND** a stored value that is not-a-number, negative, or greater than
  `STEER_SCALE_BASE_MAX_MS` SHALL be ignored and treated as absent
- **AND** an absent key SHALL be treated as zero, that is, as "not set"

#### Scenario: Set the base at runtime
- **WHEN** a `set_steer_sca_base` web command is received with a value between zero and
  `STEER_SCALE_BASE_MAX_MS` inclusive
- **THEN** the value SHALL be applied immediately and persisted to non-volatile storage so it
  survives a reboot
- **AND** a value of zero SHALL be accepted and SHALL mean "use the autopilot's value, or none"
- **AND** the command SHALL succeed regardless of the active input source, at the same privilege
  level as the existing calibration setters

#### Scenario: Reject an out-of-range base
- **WHEN** a `set_steer_sca_base` command carries a value that is not-a-number, negative, or
  greater than `STEER_SCALE_BASE_MAX_MS`
- **THEN** the command SHALL be rejected with an error response
- **AND** the stored and applied base SHALL be left unchanged

### Requirement: Steering Speed-Scaling Bench Override
The system SHALL provide a bench test override that substitutes a speed for the steering
speed-scaling calculation ONLY, so the feature can be exercised on a stationary, disarmed vehicle.
The override SHALL be volatile, SHALL expire on its own, and SHALL be visible while it is live.

#### Scenario: Substitute a speed for the scaling calculation
- **WHEN** a `set_test_speed` web command is received with a value between zero and
  `STEER_SCALE_TEST_SPEED_MAX_MS`
- **THEN** the steering speed-scaling calculation SHALL use that value in place of the measured
  speed
- **AND** the speed-sensor validity gate SHALL still apply, so the override cannot be used to scale
  steering while the sensor is unhealthy

#### Scenario: The override reaches nothing but the steering scale
- **WHEN** a test speed override is active
- **THEN** the wheel odometry reported to the autopilot, the reported ground speed, the
  maximum-speed throttle limiter, the transmission speed interlock and the odometer SHALL all
  continue to use the real measured speed
- **AND** the bench configuration SHALL therefore be structurally incapable of becoming a driving
  configuration

#### Scenario: The override is volatile and expires
- **WHEN** a test speed override is set
- **THEN** it SHALL be held in RAM only and SHALL NOT be persisted
- **AND** it SHALL clear itself automatically after `STEER_SCALE_TEST_SPEED_MS`
- **AND** it SHALL be cleared by a reboot
- **AND** setting it to zero SHALL clear it immediately

#### Scenario: The override is announced while it is live
- **WHEN** a test speed override is active
- **THEN** the steering scale log line SHALL mark the speed as `TEST`
- **AND** the telemetry stream SHALL report that an override is in force and what its value is
