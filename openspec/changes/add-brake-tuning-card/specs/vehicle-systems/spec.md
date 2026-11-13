## MODIFIED Requirements
### Requirement: Brake Control System
The system SHALL control braking force through a linear actuator-driven brake mechanism. The
"fully released" endstop SHALL be read from the brake limit sensor on `PIN_BRAKE_SENSOR`
(DevKitC-1: direct `digitalRead()`, GPIO14). The brake position SHALL be an open-loop
estimate advanced by time while the actuator is driven, at a rate derived from the configured full
travel time (see "Configurable Brake Travel Time and Hold Drive"), and a held non-zero position
SHALL be maintained with the configured hold drive.

#### Scenario: Set brake position by percentage
- **WHEN** BrakeSystem.setPosition() is called with value from 0 to 100
- **THEN** brake actuator extends proportionally
- **AND** 0% maps to fully released brakes
- **AND** 100% maps to maximum braking force

#### Scenario: Release brakes
- **WHEN** BrakeSystem.release() is called
- **THEN** brake actuator retracts to 0% position
- **AND** brakes are confirmed released

#### Scenario: Detect the released endstop
- **WHEN** `VehicleController::isBrakeReleased()` is called
- **THEN** it SHALL read `PIN_BRAKE_SENSOR` directly with `digitalRead()`
- **AND** a HIGH level SHALL mean "released" (no pressure)

#### Scenario: Emergency braking
- **WHEN** BrakeSystem.emergencyStop() is called
- **THEN** brakes are immediately applied to 100%
- **AND** throttle is forced to idle
- **AND** transmission remains in current gear
- **AND** emergency braking SHALL NOT depend on the input expander being healthy

#### Scenario: Brake hold on startup
- **WHEN** system powers on or resets
- **THEN** brakes are automatically applied to 30% (parking brake)
- **AND** brakes remain engaged until explicitly released by user command

#### Scenario: Position estimate follows the configured travel time
- **WHEN** the actuator is driven at full drive (±255) toward the target
- **THEN** the position estimate SHALL move toward the target by
  `brakeRatePctPerMs(travel_ms) × |speed| / 255 × dt` per control-loop iteration, `dt` capped at
  100 ms, and SHALL be clamped to 0–100 %
- **AND** with the default travel time of 1500 ms the estimate SHALL go from 0 to 100 % in 1.5 s

#### Scenario: Held position uses the configured hold drive
- **WHEN** the target is non-zero and the estimate is within `BRAKE_TOLERANCE` of it
- **THEN** the actuator SHALL be driven in the apply direction with the configured hold drive
  (`hold_pwm`, default 30 on the 0–255 scale)
- **AND** a hold drive of 0 SHALL stop the actuator (coast) instead of pushing

#### Scenario: Endstop logic unchanged
- **WHEN** the target is 0 and the released endstop is reached
- **THEN** the actuator SHALL keep retracting for `BRAKE_SENSOR_OVERRUN_TIME`, then stop and sync
  the estimate to 0 %, exactly as before this change

## ADDED Requirements
### Requirement: Configurable Brake Travel Time and Hold Drive
The brake full travel time and the hold drive SHALL be persisted, operator-set, per-vehicle values
owned by `VehicleController`, stored in the NVS namespace `"brake"` under the keys `travel_ms`
(whole ms, accepted range `BRAKE_TRAVEL_MS_MIN`–`BRAKE_TRAVEL_MS_MAX`, 1000–3000, default
`BRAKE_FULL_TRAVEL_TIME`, 1500) and `hold_pwm` (0–255 PWM scale, accepted range
`BRAKE_HOLD_PWM_MIN`–`BRAKE_HOLD_PWM_MAX`, 0–100, default `BRAKE_HOLD_SPEED`, 30). The rate and the
range checks SHALL be implemented by the pure header `include/BrakeLaw.h`, covered by the native
test suite `test_brake_law`.

#### Scenario: Rate from travel time
- **WHEN** `brakeRatePctPerMs(travelMs)` is called
- **THEN** it SHALL return `100 / travelMs` %/ms (0.0667 %/ms at 1500 ms, 0.1 at 1000 ms, 0.0333 at
  3000 ms)

#### Scenario: Defaults on a unit without stored values
- **WHEN** the system starts and the `"brake"` namespace or a key is missing
- **THEN** the travel time SHALL be 1500 ms and the hold drive 30, the brake behaving exactly as
  before this change

#### Scenario: Stored values are restored
- **WHEN** the system starts and `travel_ms` / `hold_pwm` hold values within their ranges
- **THEN** those values SHALL be used

#### Scenario: Invalid stored value is ignored
- **WHEN** the system starts and a stored value lies outside its range
- **THEN** that value SHALL be ignored and logged, and the default SHALL be used
- **AND** the stored value SHALL NOT be rewritten

#### Scenario: Change applied immediately
- **WHEN** a valid new travel time is set
- **THEN** it SHALL be persisted first and, once the write succeeded, used for the estimate's rate
  from the next control-loop iteration, without resetting or rescaling the current estimate
- **AND** a valid new hold drive SHALL be persisted first and used at the next hold cycle
- **AND** if the NVS write fails, the value in use SHALL stay unchanged

#### Scenario: Out-of-range value refused
- **WHEN** a travel time outside 1000–3000 ms or a hold drive outside 0–100 is requested
- **THEN** it SHALL be refused, and neither the stored nor the in-use value SHALL change
