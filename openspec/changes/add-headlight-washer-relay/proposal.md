# Change: Headlight washer ("омивач фар") on Relay_8 — timed pulse from the web and the GCS

## Why
The owner wants a headlight washer on both vehicles (quad bike `master`, buggy `aztec` — "багі"),
switched by the spare Relay_8 (owner decisions 2026-11-25). Today Relay_7 and Relay_8 are spare on
both vehicles: `RELAY_SET_RESERVED = RELAY_N(7) | RELAY_N(8)` (`include/Constants.h:205` on
`master`, `:257` on `aztec`), and `RelayController::computeShadow()` clears every reserved bit
(`src/RelayController.cpp:541`). Relay_8 is PCA9557 relay-board IO1 / connector X11
(`RELAY_TABLE_PCA9557`, `include/RelayBoardLaw.h`) and XL9535 P0.7. Relay_7 must stay unused: the
owner reports it was faulty on some boards — likely because PCA9557 IO0 is that chip's only
open-drain output (datasheet property; not verified on these boards).

## Owner decisions (2026-11-25, final)
1. **Both vehicles**, Relay_8; **Relay_7 stays reserved**.
2. **Pulse with auto-off.** One wash = Relay_8 on for the set time, then the firmware releases it.
3. **Control: web + GCS.** Web: a button in Tests → "Vehicle Ignition & Lighting" next to light /
   lock. GCS: a new `MAV_CMD_USER_2` function. No RC / servo channel (as for the lock since 1.5.34).
4. **Duration: slider in Calibration**, new card "Омивач фар", **2–20 s**, default **3 s**, NVS per
   vehicle, with help text. (Corrected by the owner from 1–10 s to 2–20 s during drafting.)
5. **Interlock: ignition on only.** Accepted in ACC or with the engine running; refused with the
   ignition OFF and during an engine start; ignition OFF and a crank abort a running pulse.
6. **EFI status bit** `pt_compensation` bit4 `0x10` = pulse running, both vehicles.
7. **"Washer fitted" switch** («Омивач встановлено»), persisted per vehicle, **default OFF**, in the
   Calibration card above the slider. Off: the Tests button is greyed, web wash refused, GCS
   capability bit6 cleared (function 6 `UNSUPPORTED`), Relay_8 never driven by the washer; the relay
   test is unaffected.

## What Changes
- **Relay assignment (both vehicles).** New `RELAY_SET_WASHER = RELAY_N(8)`; `RELAY_SET_RESERVED`
  shrinks to `RELAY_N(7)`. Masks: washer PCA9557 `0x02` / XL9535 `0x80`. Reserved: quad `0x01` /
  `0x40`, buggy `0x01` / `0x40` (buggy lock `0x8C` / `0x38` unchanged). `RelayBoardLaw.h`
  (`relaySetsValid()`, `RelayPortMasks`, `relayBuildMasks()`) gains the washer set; the compile-time
  mask `static_assert`s in `RelayController.cpp` pin the new values on both boards.
- **Pulse engine in `RelayController`.** `requestWasher(reason)` / `stopWasher(why)` /
  `isWasherOn()` / `getWasherRemainingMs()` / RAM-only `setWasherPulseMs()`. Non-blocking,
  `millis()`-based, wrap-safe, serviced in `update()` beside the relay-test expiry. A new wash while
  running **restarts** the pulse. Released by ignition OFF, crank armed / CRANKING, `allOff()`, an
  unverified write / starter escalation, a GCS stop. Refused while the relay board is unavailable.
  Pure decisions in a new header `include/WasherLaw.h`.
- **Settings.** NVS `"washer"` / `pulse_ms` (uint16 ms, 2000–20000, default 3000) and `"washer"` /
  `fitted` (bool, missing = false), owned by `VehicleController`, loaded at boot, persisted first
  then applied (length from the next wash; the switch at once). "Fitted" is a runtime setting, not a
  compile flag: one firmware for vehicles with and without the pump.
- **GCS.** `MAV_CMD_USER_2` `param1` **6** = headlight washer (`param2` 1 wash, 0 stop), capability
  **bit6 `0x40`**, set only while the washer is fitted: quad `0x1F` (not fitted, unchanged) /
  **`0x5F`** (fitted), buggy `0x3F` / **`0x7F`** (`0x1F` / `0x5F` without the X6 driver). Bit6 is
  runtime: a toggle re-sends `QB_CAPS` on the next report (today the mask is fixed at boot,
  `src/main.cpp:306-309`, and `QB_CAPS` repeats every 10 s, `src/MavlinkInterface.cpp:1015-1019`). New reject reason **5** = ignition not on / engine start in progress. Wash only under
  MAVLink control (reason 3, like the lock); stop accepted in every source.
- **EFI status bit** (owner decision 2026-11-25): `EFI_STATUS.pt_compensation` bit4
  `EFI_DIGITAL_FLAG_WASHER` `0x10` = washer pulse running, both vehicles; bits 0–3 unchanged.
  Current MP / QGC clients decode bits individually and ignore it.
- **Web.** Command `wash_headlights` (Tests button with live remaining time, refusal text under the
  button; greyed with a hint while not fitted), `set_washer_fitted` and `set_washer_pulse_ms`
  (Calibration card "💦 Омивач фар" / "💦 Headlight washer"), telemetry `washer_fitted`, `washer_on`,
  `washer_left_ms`, `washer_pulse_ms`; relay-test label of Relay_8 becomes
  "омивач фар" / "headlight washer"; uk + en.
- **Journal.** `RELAY I washer_on` per accepted wash, `RELAY I washer_abort` on an early end.
- **Docs / version.** `MAVLINK_SETUP.md`, `../../API/README.md`, `WEB_PORTAL_SETUP.md`,
  `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6, `include/Changelog.h` (uk + en); firmware **1.6.3** on both
  branches (unused on `master` and `aztec`, checked 2026-11-25).

Out of scope: Mission Planner / QGC buttons (other
repos; the API page records "ні" until they ship), RC control, a web stop button, washing on a
timer or with the wipers, `old-board` (not maintained).

## Impact
- Affected specs (deltas written against `master`'s specs):
  - `vehicle-systems`: ADDED "Headlight Washer Pulse", "Headlight Washer Fitted Switch",
    "Configurable Headlight Washer Pulse Length";
    MODIFIED "Quad Bike Relay Assignment With Fitted Front-Wheel Lock", "GCS Vehicle Command
    Interlocks".
  - `gpio-expander`: MODIFIED "Relay Output Expander", "Relay Port Mask Construction".
  - `mavlink-interface`: ADDED "Headlight Washer State in the EFI_STATUS Digital-Output Bitmask";
    MODIFIED "Relay Controller for Ignition and Lights", "GCS Vehicle Commands
    via MAV_CMD_USER_2", "Vehicle Command Queries", "Vehicle Capability Broadcast", "Vehicle Command
    Protocol Extension Rules".
  - `web-control`: ADDED "Headlight Washer Control via Web Interface", "Headlight Washer Pulse
    Length Configuration via Web Interface"; MODIFIED "Web Relay Test Control".
  - `web-telemetry`: ADDED "Headlight Washer Telemetry".
  - `event-journal`: ADDED "Headlight Washer Journal Events".
- **`aztec` specs differ** for four of the MODIFIED requirements; they must be re-based on aztec's
  text before archiving there (design D11, task 7.3), otherwise the buggy's spec would lose its lock.
- Affected code: `include/Constants.h`, `include/RelayBoardLaw.h`, new `include/WasherLaw.h`,
  `include/RelayController.h`, `src/RelayController.cpp`, `include/VehicleCommandLaw.h`,
  `src/MavlinkInterface.cpp`, `src/main.cpp` (flag assembly, caps hand-over), `include/VehicleController.h`, `src/VehicleController.cpp`,
  `include/WebPortal.h`, `src/WebPortal.cpp` (telemetry JSON only), `src/TelemetryManager.cpp`,
  `data/index.html`, `include/Changelog.h`, `test/test_relay_board`, `test/test_vehicle_cmd_law`.
  No `platformio.ini` change (the new law is tested inside the existing `test_relay_board` suite).
- **Active change `reduce-web-heap-usage`** (1.6.2, implemented, bench pending): no conflict — it
  touches `WebPortal.cpp` WebSocket connect / broadcast gate / `mountFilesystem()` and
  `platformio.ini`; this change touches only `createTelemetryJSON()` and the command dispatch in
  `VehicleController`, and no spec requirement in common.
- Behaviour on existing units: after flashing 1.6.3 the washer is **off** (not fitted) on both
  vehicles until the owner turns the switch on where the pump is installed; until then Relay_8 stays
  off and GCS clients see the same mask as today. Fitted: bit6 appears (existing bits unchanged,
  protocol version 1).
- LittleFS image must be re-uploaded on both vehicles (new card, button, strings); as always this
  erases the event journal — download it first.
