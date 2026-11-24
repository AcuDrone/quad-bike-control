## ADDED Requirements
### Requirement: iBooster Vehicle Speed Floor
On aztec, the vehicle-speed byte sent to the iBooster (`0x38B` byte 2) SHALL be round(v / 0.4),
clamped to 0–255, where v = max(true wheel speed in km/h, `IB_LAW_SPEED_FLOOR_KMH` = 2.0 km/h). An
invalid, NaN or non-positive wheel speed SHALL give the floor (5 counts), never 0. The floor SHALL be
applied in `ibSpeedByte()` (`include/IboosterLaw.h`), the single place the byte is computed, and SHALL
apply to every `0x38B` frame the firmware sends, with or without a brake request. The floored value
SHALL NOT be used anywhere else: the speed limiter, the speed governor, telemetry, MAVLink and the web
page SHALL keep the true speed. This requirement supersedes the "0 while it is invalid" clause of the
scenario "Vehicle speed to the unit" in "iBooster Rod Control". `0x38B` byte 1 (the request /
speed-qualifier bits, `0x40` only with a request), byte 3 (`0x05`), `0x38C` and `0x38D` SHALL stay
unchanged.

#### Scenario: Standing still
- **WHEN** the wheel speed is valid and 0, or valid and below 2 km/h (e.g. 1 km/h)
- **THEN** `0x38B` byte 2 SHALL be 5

#### Scenario: Speed sensor invalid
- **WHEN** the wheel speed is invalid, NaN or negative, at any true vehicle speed
- **THEN** `0x38B` byte 2 SHALL be 5

#### Scenario: At and above the floor
- **WHEN** the wheel speed is valid and exactly 2 km/h, 3 km/h or 40 km/h
- **THEN** `0x38B` byte 2 SHALL be 5, 8 and 100 respectively

#### Scenario: Clamp
- **WHEN** the wheel speed is valid and 102 km/h or more, or infinite
- **THEN** `0x38B` byte 2 SHALL be 255

#### Scenario: Rest of the firmware unaffected
- **WHEN** the buggy stands still with a valid speed of 0, or the speed sensor is invalid
- **THEN** the speed limiter, the governor, telemetry, MAVLink and the web page SHALL see the true
  speed (0, or invalid), not 2 km/h

#### Scenario: Qualifier bits unchanged
- **WHEN** frames are sent without a brake request
- **THEN** `0x38B` byte 1 SHALL carry no `0x40` (speed qualifier 0) as in firmware 1.6.0, the speed
  byte still floored; whether the unit uses the speed byte from such frames is not known
