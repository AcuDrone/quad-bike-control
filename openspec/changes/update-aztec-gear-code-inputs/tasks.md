## 1. Constants and input reader
- [x] 1.1 Replace `BOARD_IN_BIT_GEAR_NEUTRAL/REVERSE/LOW/HIGH` with `BOARD_IN_BIT_GEAR_CODE_A/B/C` = 0, 1, 6 and add `GEAR_CODE_LOW/HIGH/NEUTRAL/REVERSE` (`0b101/0b001/0b011/0b010`) with `static_assert`s (distinct, 3-bit, `000`/`111` unassigned) in `include/Constants.h`
- [x] 1.2 `BoardInputs::Snapshot`: replace the four per-gear bools with `uint8_t gearCode`; add `BoardInputs::extractGearCode()`; bit 7 is never read; update logs and the header comment

## 2. Transmission
- [x] 2.1 Add table-driven `TransmissionController::decodeGearCode()` (the only code→gear table) plus `getGearCode()` and `getGearLetter()`
- [x] 2.2 `getPhysicalGear()`: decode the snapshot code; retry (consume the next snapshot) while the code is UNKNOWN and the servo is idle; drop the one-hot multiple/none-active branches
- [x] 2.3 Log `[TRANS] gear code=0bXYZ -> L/H/N/R/UNKNOWN` on every code change; update the `initGearSensors()` bind log

## 3. Telemetry, UI, docs
- [x] 3.1 Telemetry `gear_code` (0..7, -1 when stale) and `gear_sensed` (instant decode) in `WebPortal::Telemetry`, `TelemetryManager`, and the JSON
- [x] 3.2 `data/index.html`: show "sensor code XYZ → gear" under the current gear, i18n keys in `en` and `uk`
- [x] 3.3 `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §5: aztec code table, bit 7 not wired, engaged-HIGH note, fail-safe note, pointer to `master` for the quad-bike one-hot map
- [x] 3.4 Add a uk + en line to the 1.2.0 entry in `include/Changelog.h` (no version bump)
- [x] 3.4a Docs only (no TRANS_SERVO_* value change): name the aztec transmission servo GXservo HS-1005SGT 150KG in `Constants.h`, `TransmissionController.h` and the pinout doc (16.8 V supply limit, never on the 24V rail); name the vehicle ODES JUNGLECROSS 800 EFI/EPS (aztec) vs quad bike (`master`) in `openspec/project.md`, the pinout doc header and the 1.2.0 changelog
- [x] 3.5 `pio run` builds with no new project warnings
- [x] 3.6 `openspec validate update-aztec-gear-code-inputs --strict` passes

## 3b. Web gear code calibration
- [x] 3b.1 Runtime code table `gearCodes_` in `TransmissionController` (defaults `GEAR_CODE_*`); `decodeGearCode()` becomes a `const` instance method; `getGearCodeFor()`, `isGearCodeDefault()`
- [x] 3b.2 NVS `"transmission"` keys `code_r/code_n/code_h/code_l`: `loadGearCodes()` at `begin()` with whole-table fallback to defaults on invalid content (logged); `resetGearCodes()` removes the keys
- [x] 3b.3 `captureGearCode()`: reject unknown gear, gear change in progress, stale snapshot, `000`/`111`; a code owned by another gear is taken over and that gear becomes UNSET (`0xFF`, never decodes); persist, invalidate the gear cache, TRANSMISSION log
- [x] 3b.4 Web commands `set_gear_code` (`value` R/N/L/H) and `reset_gear_codes` via `VehicleController::processGearCodeCommand()`, refused while the engine runs
- [x] 3b.5 Telemetry `gear_codes {R,N,H,L}` (`-1` = unset) and `gear_codes_custom` bitmask
- [x] 3b.6 `data/index.html`: "Gear Sensor Codes" card — live A/B/C bit indicators + `0bABC` → gear, Save as R/N/L/H, Reset to defaults (confirm), active table marked default/saved/not set (warning colour), help text on take-over, status line; en + uk
- [x] 3b.7 Changelog 1.2.0: one uk + en line
- [x] 3b.8 `pio run` clean, `openspec validate --strict` passes

## 4. Bench verification
- [ ] 4.1 Sweep the lever: verify `gear_code` per position (L `101`, H `001`, N `011`, R `010`) and that `gear` / `gear_sensed` match
- [ ] 4.2 Between positions, confirm the code reads UNKNOWN (or settles within one retry window) and never latches a wrong gear
- [ ] 4.3 Disconnect X16 (opto feed lost) → `gear_code` = 7, `gear` = `?` (UNKNOWN), throttle capped
- [ ] 4.4 Confirm bit 7 floating HIGH or LOW does not change the decoded gear
- [ ] 4.5 Capture each gear on the vehicle (lever in R/N/L/H → "Save as"), reboot, verify the table persists and `gear` matches the lever in every position
- [ ] 4.6 Verify a capture is rejected at `000`/`111` (between positions / X16 unplugged), and with the engine running; capturing a code owned by another gear takes it over and that gear shows "not set" and reads `?` until captured; "Reset to defaults" restores `R 010, N 011, H 001, L 101`
