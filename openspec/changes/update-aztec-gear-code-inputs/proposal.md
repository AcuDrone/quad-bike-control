# Change: Aztec physical gear sensing as a 3-bit code on three opto inputs

## Why
The aztec buggy's gear selector does not drive one opto line per gear like the quad bike. It drives
three lines as a binary code, and the fourth line (port bit 7, the quad bike's HIGH) is not wired.
The one-hot decode reads every aztec position as UNKNOWN or as the wrong gear.

## What Changes
- Port bits 0, 1 and 6 of PCA9557 U10 (the quad bike's NEUTRAL, REVERSE and LOW lines, same X16
  connector, same engaged-HIGH polarity) are read as a 3-bit gear code `[bit0 bit1 bit6]`:
  L = `101`, H = `001`, N = `011`, R = `010`. Every other code (`000`, `100`, `110`, `111`) is
  `GEAR_UNKNOWN`. Bit 7 is ignored completely.
- `BOARD_IN_BIT_GEAR_NEUTRAL/REVERSE/LOW/HIGH` are replaced by `BOARD_IN_BIT_GEAR_CODE_A/B/C`
  (0, 1, 6) and `GEAR_CODE_LOW/HIGH/NEUTRAL/REVERSE`, with `static_assert`s that the codes are
  distinct and that `000`/`111` stay unassigned.
- `BoardInputs::Snapshot` publishes the raw `gearCode` instead of four per-gear booleans. The code is
  decoded in one table-driven place, `TransmissionController::decodeGearCode()`.
- `getPhysicalGear()` retries (consumes the next snapshot, up to `TRANS_GEAR_READ_RETRY_COUNT`) while
  the code decodes to UNKNOWN and the servo is idle. This replaces the one-hot "none or several
  active" condition. A debug line `[TRANS] gear code=0bXYZ -> L/H/N/R/UNKNOWN` is logged on every
  code change.
- Telemetry adds `gear_code` (0..7, -1 while the inputs are stale) and `gear_sensed` (instant
  decode). The web UI shows them under the current gear, en + uk.
- **Web gear code calibration**: the code table becomes a runtime table in `TransmissionController`
  (`GEAR_CODE_*` stay the compile-time defaults). A new web card shows the three live bits
  (A = bit 0, B = bit 1, C = bit 6), the packed code and its decode, "Save as R/N/L/H" buttons that
  store the CURRENT live code for that gear, "Reset to defaults", and the active table marked
  default/saved. Commands `set_gear_code` (`value` = `"R"|"N"|"L"|"H"`) and `reset_gear_codes`;
  NVS keys `code_r/code_n/code_h/code_l` in the existing `"transmission"` namespace, loaded at
  `begin()`. Captures of `000`/`111`, of a stale snapshot, during a gear change, or with the engine
  running are rejected. Capturing a code owned by another gear takes it over and leaves that gear
  UNSET (never decodes, reads UNKNOWN) until it is captured. Invalid NVS content falls back to the
  defaults. Telemetry adds `gear_codes` (`-1` = unset) and `gear_codes_custom`.
- The transmission actuator (servo, overshoot/return, sequencing, RC gear ranges) is unchanged.
- **BREAKING (aztec only)**: the quad-bike one-hot assignment no longer applies on this branch. It
  stays on `master`.

## Impact
- Affected specs: `gpio-expander`, `vehicle-systems`, `web-telemetry`, `web-control`
- Affected code: `include/Constants.h`, `include/BoardInputs.h`, `src/BoardInputs.cpp`,
  `include/TransmissionController.h`, `src/TransmissionController.cpp`, `include/VehicleController.h`,
  `src/VehicleController.cpp`, `include/WebPortal.h`,
  `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`, `include/Changelog.h`,
  `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §5
