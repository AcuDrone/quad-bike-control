## ADDED Requirements
### Requirement: Parking Brake State in the EFI_STATUS Digital-Output Bitmask
The `EFI_STATUS.pt_compensation` digital-output bitmask SHALL report the aztec parking-brake state
in two new bits, next to bit0 `EFI_DIGITAL_FLAG_WHEEL_LOCK` and bit1 `EFI_DIGITAL_FLAG_FRONT_LIGHT`:
- bit2 `EFI_DIGITAL_FLAG_PARK_BRAKE` (0x04) SHALL be set when the state is `APPLIED`;
- bit3 `EFI_DIGITAL_FLAG_PARK_BRAKE_UNSETTLED` (0x08) SHALL be set when the state is `APPLYING`,
  `RELEASING` or `UNKNOWN`.

Both bits clear SHALL mean `RELEASED`. Bits 0 and 1 SHALL keep their meaning, and bits 4–7 SHALL
stay 0. The bitmask SHALL be assembled with the other flags in `src/main.cpp`, and
`MAVLINK_SETUP.md` SHALL document the new bits and the value range 0–15.

#### Scenario: Applied
- **WHEN** the park brake is `APPLIED`, the lock is released and the light is off
- **THEN** `pt_compensation` SHALL be 4.0

#### Scenario: Running or unknown
- **WHEN** the park brake is `APPLYING`, `RELEASING` or `UNKNOWN`
- **THEN** bit3 SHALL be 1 and bit2 SHALL be 0

#### Scenario: Throttle passing during a release
- **WHEN** a release run has passed its throttle release point and the actuator is still running
- **THEN** bit3 SHALL still be 1 and bit2 SHALL be 0, because the state is `RELEASING` until the
  run ends

#### Scenario: Released
- **WHEN** the park brake is `RELEASED`
- **THEN** bits 2 and 3 SHALL both be 0

#### Scenario: Existing bits unchanged
- **WHEN** the lock is engaged and the park brake is `APPLIED`
- **THEN** bit0 and bit2 SHALL both be 1 (`pt_compensation` = 5.0 with the light off)
