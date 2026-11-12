# Change: GCS vehicle commands — gear, ignition, headlight, diff lock and handbrake over MAV_CMD_USER_2

## Why
The pilot keeps only two joystick axes (steering, throttle/brake). Gear, ignition, headlight, diff
lock and handbrake move to the ground-station UI (Mission Planner plugin v1.21 and the QGC fork)
and to steering-wheel buttons. Today the firmware takes these functions only from RC servo
channels (`SERVO_OUTPUT_RAW` servo3/4/6/7), re-applies them every control loop, and answers every
`COMMAND_LONG` other than the trip reset with `MAV_RESULT_UNSUPPORTED`. A command from the ground
station would be overwritten by the RC re-apply in the next loop, and the clients have no way to
learn which functions a vehicle has.

The ground station sends **commands** to the ESP32 (sys 1 / comp 25). The ESP32 stays the arbiter
of every interlock and reports the result in a `COMMAND_ACK` and in telemetry. ARM/DISARM goes to
the autopilot (comp 1) and is not part of this change.

Both branches: `master` (quad bike, now 1.5.28) and `aztec` (buggy, now 1.5.29). The work is done
on `master` first, then `master` is merged into `aztec`, and the aztec-only handbrake function
(`param1 = 5`) is added on `aztec` on top of the merge.

## What Changes
- **New command `MAV_CMD_USER_2` (31011), control-protocol version 1.** Addressed exactly to
  sys 1 / comp 25 (strict addressing unchanged). `param1` = function, `param2` = value,
  `param3..7` ignored. Floats are compared with `MAVLINK_CMD_PARAM_EPSILON`, never `==`; NaN is
  invalid.

  | param1 | Function | param2 | Capability bit |
  |---|---|---|---|
  | 0 | Query | 0 caps, 1 firmware version, 2 hardware version | — |
  | 1 | Gear | R −1, N 0, H 1, L 2 | bit0 R/N/L, bit1 H |
  | 2 | Ignition | OFF 0, ACC 1, ON 2 (ON = auto-crank) | bit2 |
  | 3 | Headlight | OFF 0, ON 1 | bit3 |
  | 4 | Diff lock | OFF 0, ON 1 | bit4 |
  | 5 | Handbrake | OFF 0 (release), ON 1 (apply) — **aztec only** | bit5 |

- **COMMAND_ACK results.** `ACCEPTED` (also when already in the requested state);
  `TEMPORARILY_REJECTED` with a reason in `result_param2` — 1 engine not running, 2 speed
  > 1.4 m/s, 3 failsafe / not under MAVLink control, 4 speed > 3 km/h (handbrake apply);
  `DENIED` for an invalid `param2`; `UNSUPPORTED` for an unknown function or a function whose
  capability bit is clear; `FAILED` when the vehicle could not execute it (handbrake
  `REFUSED_DRIVER` / `REFUSED_NVS`, relay board unavailable). `progress` is 0 except for the caps
  query (= protocol version 1). The ACK goes back to the requester, as today.
- **Interlocks (firmware is the arbiter).**
  - Gear: reason 3 when the control source is not MAVLink (fail-safe or web portal), reason 1
    when the engine is not running (not remembered), reason 2 when the transmission's speed
    interlock refuses — R / H / L only; N is accepted at any speed, as the transmission does
    today (D-F, a deliberate deviation from the brief). H is allowed over MAVLink (RC still cannot select it). Allowed with the
    handbrake on.
  - Ignition: OFF always accepted (engine kill, also in fail-safe, web and cranking). ACC / ON:
    reason 3 when the source is not MAVLink. ON while the engine runs → `ACCEPTED`, no re-crank. ON
    otherwise → `requestCrank()` (2 s pre-crank dwell, starter ≤ 5 s), like a fresh RC transition.
  - Headlight: always accepted.
  - Diff lock: reason 3 when the source is not MAVLink; otherwise the same `RelayController` path
    CH7 uses (quad: Relay_5 + Relay_6; buggy: Relay_4 + Relay_5 + Relay_6).
  - Handbrake (aztec): `commandParkBrake(…, ParkBrakeSource::GCS)`; no new interlock; never
    reason 3; `REFUSED_MOVING` → reason 4.
- **Arbitration: last change wins.** RC channels CH3 gear, CH4 ignition, CH6 light, CH7 lock and
  GCS commands are equal sources. RC is applied only when its decoded value **changes**; a
  disappearing channel (no frame, fail-safe, out-of-band value) is not a change. The every-loop
  re-apply of gear / ignition / light / lock is removed. Every time RC comes back in control — at
  boot, on every exit from fail-safe (owner decision D-A) and on every return from the web portal
  to MAVLink control (D-H) — the first valid RC frame sets the state again, so RC-only operation
  behaves exactly as today, including the re-crank on recovery. An RC gear requested
  with the engine stopped stays pending and is applied once the engine runs (today's retry); a
  later GCS gear command supersedes it (D-E).
- **Queries (`param1 = 0`).** caps → bitmask (bits 0–5 as above, 6–15 reserved = 0) with
  `progress` = 1; firmware version → `major*10000 + minor*100 + patch`; hardware version → bits
  0–7 board revision (0 = `Control_v0`), bits 8–15 vehicle type (1 quad / 2 buggy), bits 16–23
  relay board detected at boot (0 none, 1 PCA9557, 2 XL9535; D-I — not the NVS setting, a
  deviation from the brief). Any other `param2` → `DENIED`.
- **Broadcasts.** `NAMED_VALUE_INT "QB_CAPS"` every 10 s = mask | (protocol version << 24).
  `NAMED_VALUE_INT "IGN_STATE"` (0 OFF, 1 ACC, 2 ON, 3 CRANKING incl. the pre-crank dwell) at 1 Hz
  and immediately on change, from the applied relay state. `EFI_STATUS` is unchanged (aztec bits 2
  / 3 for the parking brake stay exactly as committed in 1.5.29; on master they stay 0).
- **Capability masks.** Quad (`master`): `0x1F` — bit5 never. Buggy (`aztec`): `0x3F` when the
  X6 parking-brake driver initialised at boot, `0x1F` when it did not (then function 5 answers
  `UNSUPPORTED`).
- **Pure law** `include/VehicleCommandLaw.h` (header-only, no Arduino) with native tests
  `test/test_vehicle_cmd_law`: command decode / whitelist, ACK result per interlock (and, on
  aztec, per `ParkBrakeResult`), last-change-wins including the fail-safe-exit and WEB → MAVLINK re-sync, QB_CAPS,
  firmware- and hardware-version encoding, IGN_STATE code.
- **Docs.** `MAVLINK_SETUP.md`: new "Vehicle commands (MAV_CMD_USER_2)" section with the table,
  ACK codes, queries, QB_CAPS / IGN_STATE and the extension rules.
- **Version.** Both branches become **1.5.30** (owner decision D-D; `master` skips 1.5.29, which
  is the aztec-only parking brake). Changelog entries in uk + en ("багі" in Ukrainian). No web
  UI change, so no LittleFS upload is needed for this change.

Must not change: the trip reset (`MAV_CMD_USER_1`), strict addressing, `UNSUPPORTED` for unknown
commands, the `EFI_STATUS` field layout, fail-safe behaviour itself (`applyFailsafe()` still
turns all relays off on entry), RC-only operation for vehicles whose GCS never sends commands,
and the existing parking-brake controller's behaviour (drive-away release, throttle hold, apply
speed limit).

Out of scope: ARM/DISARM (autopilot); client code (`gcs` repository); a web-UI view of GCS
commands; `EventJournal` entries for GCS commands (debug log only); position feedback for the
handbrake.

## Impact
- Affected specs:
  - `mavlink-interface`: MODIFIED "Inbound Command Handling and Trip Reset", "Steering Control
    Mode Reporting" (STEER_MODE is no longer the only `NAMED_VALUE_INT`); ADDED "GCS Vehicle
    Commands via MAV_CMD_USER_2", "Vehicle Command Queries", "Vehicle Capability Broadcast",
    "Ignition State Broadcast", "Vehicle Command Protocol Extension Rules".
  - `vehicle-systems`: ADDED "GCS Vehicle Command Interlocks", "Last-Change-Wins Arbitration of
    RC Channels and GCS Commands", "GCS Handbrake Command".
- Affected code (`master`, then merged into `aztec`): `include/Constants.h`,
  `include/VehicleCommandLaw.h` (new), `include/MavlinkInterface.h`, `src/MavlinkInterface.cpp`,
  `include/VehicleController.h`, `src/VehicleController.cpp`, `include/RelayController.h`
  (crank-pending getter), `include/TransmissionController.h` (speed-interlock query made public),
  `src/main.cpp`, `include/Changelog.h`, `platformio.ini`, `test/test_vehicle_cmd_law/` (new),
  `MAVLINK_SETUP.md`.
- Affected code (`aztec` only, after the merge): the function-5 path in `VehicleController`,
  the `ParkBrakeResult` → ACK mapping in `include/VehicleCommandLaw.h`, its native tests, bit5 from
  `ParkBrakeController::isDriverOk()`.
- Behaviour after flashing, for a vehicle whose GCS sends nothing: unchanged, with one deliberate
  difference — an RC channel that goes out of band (not 880–2160 µs) no longer forces N / OFF /
  light off / unlock; the last state is kept (see design D6). Releasing web control now re-applies
  the RC switches too (D-H), including the auto-crank when the ignition switch is in ON.
- Hardware: none.
- Branches: `master` first; merge `master` → `aztec`; handbrake on `aztec` only. Never cherry-pick
  the parking brake into `master` (X6 is the quad's steering VESC).
