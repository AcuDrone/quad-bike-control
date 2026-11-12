## 0. Preconditions
- [x] 0.1 Open questions Q1–Q4 answered by the owner on 2026-11-13 and folded into design.md (D-F … D-I) and the specs.
- [x] 0.1a Proposal approved by the owner (2026-11-13, including the WEB → MAVLINK re-sync with auto-crank).
- [x] 0.2 Work starts on `master` (tasks 1–6). `aztec` gets this change only by merging `master` (task 7) plus the aztec-only handbrake (task 8). Never cherry-pick the parking brake into `master`.

## 1. Constants and pure law (`master`)
- [x] 1.1 `include/Constants.h`:
  - `VCMD_PROTOCOL_VERSION` 1, `VCMD_CAPS_TX_MS` 10000;
  - capability bits `VCMD_CAP_GEARS_RNL` 0x01, `VCMD_CAP_GEAR_H` 0x02, `VCMD_CAP_IGNITION` 0x04,
    `VCMD_CAP_LIGHT` 0x08, `VCMD_CAP_LOCK` 0x10, `VCMD_CAP_HANDBRAKE` 0x20;
  - reason codes 1 engine not running, 2 speed, 3 failsafe / not under MAVLink control, 4 handbrake
    apply speed;
  - `BOARD_REVISION_CODE` 0 (`Control_v0`), `VEHICLE_TYPE_CODE` 1 (quad; aztec keeps 2 after the
    merge);
  - `RC_CHANNEL_VALID_MIN_US` 880, `RC_CHANNEL_VALID_MAX_US` 2160 (absent outside);
  - comment the `MAV_CMD_USER_2` protocol next to the `MAV_CMD_USER_1` block.
- [x] 1.2 `include/VehicleCommandLaw.h` (header-only, no Arduino):
  - `VehicleCmdFunction`, `VehicleCmdResult` (values = `MAV_RESULT_*`), `VehicleCmdOutcome {result, reason}`;
  - `vehicleCmdDecode(param1, param2, capsMask)` per design D3 (epsilon, NaN, integer check, table, capability bits, H vs bit1, query `param2` 0/1/2);
  - interlock functions per design D5 (gear: 3 → 1 → already-at-target → 2 → accept; ignition OFF / ACC / ON; light; lock; relay-unavailable → FAILED);
  - absent-aware RC decoders (gear R/N/L, ignition OFF/ACC/ON, switch > 1520) and the `RcEdge<T>` tracker (`step()` → apply / no change, `reset()`);
  - `vehicleCapsMask()`, `qbCapsValue()`, constexpr `firmwareVersionCode()`, `hardwareVersionCode()`, `ignStateCode()`;
  - `VehicleCommandSink` abstract interface (no mavlink types).

## 2. Transport (`master`)
- [x] 2.1 `MavlinkInterface::handleCommandLong()` takes `param2`; `MAVLINK_MSG_ID_COMMAND_LONG` passes `cl.param2`.
- [x] 2.2 `MAV_CMD_USER_2` branch: decode → query answered from the registered mask / versions (`progress` = 1 for caps), function → sink → ACK with `result_param2` = reason; no sink → `UNSUPPORTED`. Trip reset, strict addressing and `UNSUPPORTED` for other commands unchanged. ACK helper takes result, progress and `result_param2`. `static_assert` the `VehicleCmdResult` values against `MAV_RESULT_*`.
- [x] 2.3 `setVehicleCommandSink(sink, capsMask, hwVersion)`; one `[MAV] USER_2 …` debug line per command (function, value, result, reason, requester).
- [x] 2.4 `StateReport` gains `ignState` (the caps mask comes from the sink registration, fixed at boot); `report()` sends `QB_CAPS` every 10 s (first on the first report) and `IGN_STATE` on the 1 Hz slow tick and immediately on change; name literals with the 10-byte `static_assert`; update the "only NAMED_VALUE_INT" comments.

## 3. Vehicle layer (`master`)
- [x] 3.1 `VehicleController` implements `VehicleCommandSink::executeVehicleCommand()` per design D4/D5: "not under MAVLink control" = `currentInputSource_ != MAVLINK || !mavlink_.isSignalValid()`; gear via the public speed-interlock query + `setGear()`, clears the pending RC gear on accept; ignition OFF persists counters on a transition into OFF; ON with the engine running sets IGNITION without a crank; ON otherwise `requestCrank()`; light; lock; FAILED when the relay board is unavailable (D-G).
- [x] 3.2 `TransmissionController`: expose the speed interlock as a public const query (`canChangeGear()` or a wrapper); no behaviour change.
- [x] 3.3 `RelayController`: public `isCrankPending()` (crank armed, pre-crank dwell); no behaviour change.
- [x] 3.4 `processMavlinkCommands()`: replace the every-loop gear / ignition / light / lock re-apply with the `RcEdge` trackers and the pending RC gear (design D6); steering, throttle and brake unchanged. Remove `previousIgnitionState_` / `lastCommandedGear_`.
- [x] 3.5 Re-sync (D-A, D-H): in every loop whose source is not MAVLink (`FAILSAFE` or `WEB`) reset the four RC trackers and the pending RC gear, so the first valid frame after any return to MAVLink control re-applies all four channels; remove the `previousIgnitionState_` / `lastCommandedGear_` reset from `applyFailsafe()`, everything else there unchanged.
- [x] 3.6 Capability mask built once at boot (bit5 never on `master`); `getIgnStateCode()` getter for the report.
- [x] 3.7 `src/main.cpp`: register the sink after `vehicleController.begin()` with the mask and the hardware version (board revision, `VEHICLE_TYPE_CODE`, detected `RelayBoardKind`); fill `report.capsMask` / `report.ignState`.

## 4. Tests (`master`)
- [x] 4.1 `test/test_vehicle_cmd_law/test_main.cpp` + an indented `test_filter` line in `platformio.ini`:
  - decode: addressing is not in the law; param1 0…5 accepted, 6, −1, 2.5, NaN → UNSUPPORTED; param2 per table, 0.005 off → accepted, 0.5 off / NaN / out of table → DENIED; H with bit1 clear → UNSUPPORTED; function 5 with bit5 clear → UNSUPPORTED; query `param2` 3 → DENIED;
  - ACK per interlock: gear in fail-safe / WEB → reason 3; engine off → reason 1; speed refused → reason 2; N at speed → ACCEPTED (D-F), R/H/L at speed → reason 2; already at target → ACCEPTED; ignition OFF in fail-safe / WEB / cranking → ACCEPTED; ACC/ON not under MAVLink control → reason 3; ON with the engine running → ACCEPTED, no crank; headlight in fail-safe → ACCEPTED; lock in WEB → reason 3; relays unavailable → FAILED;
  - last change wins: command then constant RC → command holds; RC edge → RC wins; RC absent (0 µs, 870, 2170) → no change; same value after absent → no change; first value after `reset()` → applied, for each re-sync cause: boot, fail-safe exit, and WEB → MAVLINK (web control released; RC back with a browser connected) — a value set by a GCS / web command before the return is overwritten; pending RC gear cleared by an accepted GCS gear, kept by a rejected one, replaced by a later RC edge;
  - QB_CAPS: `0x1F | 1<<24` = 16777247, `0x3F | 1<<24` = 16777279; bits 16–23 zero;
  - versions: "1.5.28" → 10528, "1.5.30" → 10530, "2.10.3" → 21003; hardware version (board 0, vehicle 1, relay board 2) → 0x00020100;
  - IGN_STATE: OFF 0, ACC 1, ACC + crank pending 3, CRANKING 3, IGNITION 2.

## 5. Docs and release (`master`)
- [x] 5.1 `MAVLINK_SETUP.md`: section "Vehicle commands (MAV_CMD_USER_2)" — table, ACK results and reason codes, queries, QB_CAPS / IGN_STATE, last-change-wins, extension rules; note that function 5 is aztec-only.
- [x] 5.2 `FIRMWARE_VERSION` → "1.5.30"; `include/Changelog.h` 1.5.30 entry, uk + en, operator wording: gear / ignition / headlight / diff lock from the ground station, interlocks, last change wins, an out-of-band RC channel no longer forces N / OFF; releasing web control (or RC returning while a browser is connected) re-applies the RC switches, including the auto-crank with the ignition switch in ON.
- [x] 5.3 Fix the stale "not fitted on aztec" wording in the `RelayController.h` comment and the `processMavlinkCommands()` lock comment.
- [x] 5.4 `pio test -e native` green; `pio run -e esp32-s3-devkitc-1` succeeds.
- [x] 5.5 `openspec validate add-gcs-vehicle-commands --strict` passes.

## 6. Bench — quad (`master` 1.5.30)
- [ ] 6.1 RC only, GCS sends nothing: gear, ignition (incl. auto-crank), light and lock behave as on 1.5.28; pull the RC stream → fail-safe; restore → state re-applied, re-crank if RC is ON.
- [ ] 6.1a Web control: switch the ignition off and the light on from the web with RC CH4 in ON and CH6 OFF, then release web control → the light goes off and the auto-crank starts (D-H); the same when the RC stream returns while a browser is connected.
- [ ] 6.2 USER_2 to comp 0 → no ACK; unknown `param1` → UNSUPPORTED; `MAV_CMD_USER_1` trip reset unchanged.
- [ ] 6.3 Queries: caps ACCEPTED, `result_param2` 31 (0x1F), `progress` 1; firmware 10530; hardware 0x000X0100 (X = relay board detected at boot: 1 PCA9557, 2 XL9535; D-I). `QB_CAPS` = 16777247 every 10 s.
- [ ] 6.4 Ignition from the GCS: ON → IGN_STATE 3 for the dwell and the starter, then 2; OFF during the dwell / cranking → 0; ACC/ON with no RC stream → reason 3; OFF with no RC stream → accepted.
- [ ] 6.5 Gear from the GCS: engine off → reason 1; wheels above 1.4 m/s → reason 2 for R/H/L, N accepted (D-F); H accepted; an RC gear left constant does not pull the gear back; moving the RC switch does.
- [ ] 6.6 Headlight / diff lock from the GCS hold against a constant RC channel; lock in web control → reason 3.

## 7. Merge into `aztec`
- [x] 7.1 Merge `master` into `aztec`; resolve `Constants.h` (version 1.5.30, `VEHICLE_TYPE_CODE` 2, `RELAY_WHEEL_LOCK_FITTED` 1), `Changelog.h` (1.5.30 above the aztec 1.5.29 entry), `processMavlinkCommands()` (keep `applyOperatorThrottle()`), `platformio.ini` (`test_filter` keeps `test_park_brake_law`).
- [x] 7.2 `pio test -e native` and `pio run -e esp32-s3-devkitc-1` on `aztec`.

## 8. Handbrake — `aztec` only
- [x] 8.1 `include/VehicleCommandLaw.h` (aztec): `vehicleCmdParkBrakeAck(ParkBrakeResult)` — STARTED / STARTED_SPEED_UNKNOWN / REVERSING / ALREADY_RUNNING → ACCEPTED; REFUSED_MOVING → TEMPORARILY_REJECTED reason 4; REFUSED_DRIVER / REFUSED_NVS → FAILED.
- [x] 8.2 `VehicleController::executeVehicleCommand()` function 5 → `commandParkBrake(value == 1 ? APPLY : RELEASE, ParkBrakeSource::GCS)`; no source / ignition / fail-safe check (never reason 3).
- [x] 8.3 Capability bit5 = `parkBrake_.isDriverOk()` at boot (mask 0x3F, or 0x1F without the X6 driver); `EFI_STATUS` bits 2 / 3, drive-away release and throttle hold unchanged.
- [x] 8.4 Native tests (aztec cases in `test_vehicle_cmd_law`): every `ParkBrakeResult` → ACK; function 5 with bit5 clear → UNSUPPORTED; QB_CAPS 16777279.
- [x] 8.5 Changelog 1.5.30 (aztec): add the handbrake-from-GCS line (uk + en, "багі").
- [x] 8.6 `pio test -e native`, `pio run -e esp32-s3-devkitc-1`, `openspec validate add-gcs-vehicle-commands --strict` on `aztec`.

## 9. Bench — buggy (`aztec` 1.5.30)
- [ ] 9.1 Repeat 6.1–6.6 on the buggy; caps 63 (0x3F) with the X6 driver, `QB_CAPS` 16777279; hardware 0x000X0200; diff lock = Relay_4 + Relay_5 + Relay_6.
- [ ] 9.2 Gear H from the GCS reaches H (the RC cannot select it) — confirms bit1 on the buggy.
- [ ] 9.3 Handbrake from the GCS: apply at standstill → ACCEPTED, `[PBRAKE] Apply accepted (gcs)`, `pt_compensation` bit3 then bit2; apply above 3 km/h → reason 4; release → ACCEPTED in fail-safe and with the ignition off; drive-away release and throttle hold behave as with a web apply.
- [ ] 9.4 X6 driver absent (disconnect / failed init at boot): caps 0x1F, function 5 → UNSUPPORTED.
