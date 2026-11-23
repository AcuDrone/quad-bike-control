# Change: Aztec service brake becomes a Bosch iBooster Gen2 on the CAN bus

> **Known limitations accepted by the owner (decisions 3 and 4, design.md D1).**
> - The iBooster wakes from the ACC / ignition line (Relay_1). With the ignition OFF, including the
>   fail-safe `allOff()`, the unit sleeps and **the buggy ("багі") has no service brake**.
> - The fail-safe is **not** changed. Today it freezes the last brake target and switches the
>   ignition off. With the iBooster that means: **on fail-safe the service brake drops** as soon as
>   the ignition relay opens. No fail-safe braking is added.
> - The firmware cannot power-cycle the unit. A latched unit `failure` needs the operator (ignition
>   cycle — untested — or a power cycle of X150 pin 1).
> - **The buggy can be left with no brake at all — owner decision 2026-11-23 (Q1 (b)).** A new
>   service-brake press releases the parking brake as today even when the iBooster is not acting
>   (ignition OFF, asleep, lost, fault): the parking brake is then released and the service brake
>   does not hold. Only while the iBooster is available does the release wait until it has reached
>   the pressed position.

## Why
- The aztec service brake is a linear actuator on a BTS7960 (X7, GPIO7/GPIO6). It has no position
  feedback: the firmware estimates the position by time (`include/BrakeLaw.h`), uses the In5 endstop
  only on release, and pushes with a fixed hold drive. The brake force is a guess.
- The owner is replacing it with a Bosch iBooster Gen2 (Tesla Model 3/Y, P/N 1044671-00-x). The unit
  takes its commands on CAN and reports its rod position. One unit was characterised on the bench
  (`Arduino/iBooster-TEST/README.md`, «Що виміряно на стенді», 22–23.11.2026): frames, checksums,
  timeouts, the stroke limit, the field map.
- The board already has one CAN bus (MCP2515, 500 kbit/s) shared by the engine ECU (OBD-II) and the
  Bosch LWS (`0x305`). The approved prerequisite `move-can-rx-to-interrupt` gives that chip a single
  owner task, woken by INT, with two transmit buffers left free for exactly this change.
- The unit needs three frames from us every 10 ms. While a brake request is active, a gap of 200 ms
  or more makes it release by itself, and it does not re-apply. The 10 ms schedule and the rod loop
  therefore cannot live in the cooperative `loop()`.

## What Changes
- **Dependency.** Built on `move-can-rx-to-interrupt` (the `can_io` task, the RX / TX rings, TXB0
  for general frames, TXB1 / TXB2 free, design D5 / D11 there). That change is implemented on
  `master` and merged into `aztec` **before** this one is implemented.
- **BREAKING (hardware, aztec only).** The X7 BTS7960 brake actuator, its In5 "released" endstop,
  the open-loop estimate, the hold drive and the brake tuning card are removed. The service brake
  exists only through the iBooster (owner decision 1).
- **Wiring** (owner decision 3): X150 pin 1 straight from the battery (4 mm², no diode, no
  converter), pin 9 ground, pin 20 Wake from the ACC / ignition line switched by Relay_1, pins 18 / 10
  CAN-H / CAN-L to the existing bus (X13). No new relay; Relay_7/8 stay reserved. Pins 25 / 16 (the
  unit's second bus) are not connected. The bus termination is measured and corrected (task 9.1).
- **One brake abstraction** (`IServiceBrake`): the rest of the firmware sets a percent target and
  reads "applied %", "reached", "available" and a state. The iBooster implementation sits behind it.
  `applyBrake()` and its 5 % dead zone are unchanged; the RC, web and governor paths keep calling it.
- **Frames in the CAN task** (design D3). The `can_io` task gets a 10 ms slot clock and sends
  `0x38B` / `0x38C` / `0x38D` through TXB1 / TXB2 (bench-exact payloads, CRC-8 SAE J1850, shared
  4-bit counter, `0x38D` pressure fixed at 5.1 bar, qualifier 1 — no pressure sensor, owner decision
  2). It receives `0x38E` / `0x38F` / `0x39D` through the free exact-match filters RXF3–RXF5, checks
  their checksums, decodes them and runs the rod loop there. The main loop and the task exchange one
  command record and one snapshot record; no iBooster frame goes through the receive ring outside a
  sniff.
- **State machine** (design D4): `ASLEEP → STARTING → READY ⇄ BRAKING → RELEASING → READY`, plus
  `LOST` and `FAULT`. Frames are sent only while the ignition is on or the unit is still heard.
  A cranking dip is journaled with `crank=1` and does not count as a latched fault.
- **Brake command = rod stroke** (design D5): target mm = brake % × max stroke / 100, P loop on the
  `0x39D` rod position, flow clamped by an apply and a release rate, hard ceiling 16 mm. Release is
  a controlled return to 1 mm under request, then the request is cleared.
- **Settings** (Calibration section, NVS `brake`): max stroke 2–16 mm (default 16, the full
  external-request stroke), apply rate 1–40 ml/s (default 8), release rate 1–40 ml/s (default 8) —
  owner decision 2026-11-23, replacing the earlier Q5 defaults. The operator lowers the maximum
  stroke for the first vehicle test and raises it step by step. The old `travel_ms` / `hold_pwm` keys
  are removed once at boot (no migration; design D6).
- **Vehicle speed to the unit** from the hall sensor, 0 while the speed is invalid; the "faulty"
  qualifier is never sent (design D5).
- **Park brake** (owner Q1 (b)): a new service-brake press releases the parking brake under the same
  sources as today. While the iBooster is available the release waits for it to report "reached"
  (today's hand-over, with the measured rod instead of the estimate); while it is unavailable the
  release fires on the press alone, so the "any gear, any ignition" rule stays true (design D8).
- **GCS visibility** (owner Q4 (a)): one `STATUSTEXT` per brake LOST / FAULT / recovered, WARNING /
  INFO like the existing steering and governor texts, at most one per 5 s with the latest state kept
  pending (design D10).
- **Governor / limiter / slew:** "brake commanded" stays the commanded target ≥ 5 %, unchanged.
- **iBooster trouble** (owner decision 4): journaled (new source `BRAKE`: state transitions, a fault
  entry with the last raw `0x39D` / `0x38E` / `0x38F`, battery voltage, what we were commanding),
  shown in web telemetry and the Diagnostics attention box, and no brake request is sent until the
  unit is ready again.
- **Flash and OTA discipline** (design D9): journal flush and the periodic odometer / engine-hour
  saves are deferred while a brake request is active; web OTA, LittleFS upload, ArduinoOTA and the
  journal clear are refused while a request is active or the buggy is moving; CAN watchdog re-inits
  and the CAN sniffer are deferred / refused during a request.
- **Loop stall** (no task watchdog exists): the task keeps the last command, flags it stale after
  500 ms, and the episode is journaled when the loop returns (design D7).
- **Pure logic, native-tested:** `include/IboosterProto.h` (ported from the bench
  `src/ibooster_proto.h` with its 48-frame trace and real `0x39D` / `0x38E` / `0x38F` vectors) and
  `include/IboosterLaw.h` (state machine, percent → rod → flow, staleness, speed byte).
  `include/BrakeLaw.h` and `test/test_brake_law` are deleted.
- **Version** (owner Q6). Wiring change → minor bump: **1.6.0** on `aztec`, after
  `move-can-rx-to-interrupt` is merged in as 1.5.38 (design D12). `master` continues 1.5.x; its next
  minor is 1.7.0. Changelog entry uk + en ("багі"), LittleFS image required.

## Relationship to other changes
- **Must be implemented after** `move-can-rx-to-interrupt` is merged from `master` into `aztec`.
- **Must be archived after** (its MODIFIED / REMOVED headers exist only in their deltas):
  1. `add-brake-tuning-card` — "Brake Control System" (MODIFIED there, REMOVED here and replaced
     by "Service Brake Control"), "Configurable Brake Travel Time and Hold Drive", "Brake Tuning
     Card", "Brake Position and Tuning Telemetry" (REMOVED here);
  2. `add-aztec-parking-brake` — "BTS7960 Motor Driver Control", "Hardware Configuration"
     (MODIFIED here);
  3. `repurpose-x10-battery-sense` — also modifies "Hardware Configuration";
  4. `add-park-brake-release-by-service-brake` — "Parking Brake Release by Service-Brake Press",
     "Parking Brake Card Service-Brake Release Help" (MODIFIED here);
  5. `add-event-journal-and-link-recovery` — the `event-journal` capability ("Event Line Format",
     "Buffered Journal Flush" MODIFIED here);
  6. `move-can-rx-to-interrupt` — "Queued CAN Transmit", "CAN Receive Queue" (MODIFIED here).
- **`move-aztec-vesc-to-can` is stale again.** It modifies "Hardware Configuration" and
  "Shared-Bus Steering Angle Frame Reception" from texts that still have the X7 brake and RXF3–RXF5
  = `0x305`, and claims firmware 1.7.0. It must be re-based on this change (its RXF1 plan is
  compatible) before it is implemented.

## Out of scope
- A master-cylinder pressure sensor (later change; the best free ADC1 input is GPIO10 on X5 — not
  designed here).
- Any change to the fail-safe, fail-safe braking, an emergency stop, a brake on RC loss.
- A firmware-switched power or Wake relay for the unit; the unit's second bus (pins 25 / 16).
- A task watchdog (project rule, firmware has none — known gap).
- Brake telemetry on MAVLink beyond the three `STATUSTEXT` texts (no NAMED_VALUE, no new message).
- `master` and `old-board`: nothing of this change goes there (owner decision 5).

## Impact
- Affected specs:
  - `vehicle-systems`: MODIFIED Degraded Operation on Board I/O Fault, Parking Brake Release by
    Service-Brake Press; ADDED Service Brake Control, Service Brake Abstraction, iBooster Unit State
    Machine, iBooster Rod Control, Service Brake Settings, Service Brake With the Ignition OFF and
    Under Fail-Safe, Flash Writes During a Service-Brake Request; REMOVED Brake Control System,
    Configurable Brake Travel Time and Hold Drive. (Brake Control System is removed and re-added
    under a new name because the CLI refuses a MODIFIED block that drops scenarios; the other
    MODIFIED blocks keep every scenario header, rewritten where the behaviour changed.)
  - `vehicle-actuators`: MODIFIED BTS7960 Motor Driver Control, Hardware Configuration, Actuator
    Safety Limits; ADDED Bosch iBooster Gen2 Service Brake Unit.
  - `gpio-expander`: MODIFIED Opto-Isolated Input Expander, Opto Input Polling and Fault Policy.
  - `can-controller`: MODIFIED Shared-Bus Steering Angle Frame Reception, Queued CAN Transmit, CAN
    Receive Queue; ADDED iBooster Periodic Frames, iBooster Frame Reception, Brake Command Hand-Off
    to the CAN Task, CAN Maintenance During a Brake Request.
  - `web-control`: REMOVED Brake Tuning Card; ADDED Service Brake Card; MODIFIED Parking Brake Card
    Service-Brake Release Help.
  - `web-telemetry`: REMOVED Brake Position and Tuning Telemetry; ADDED Service Brake Telemetry,
    Service Brake Display and Attention.
  - `event-journal`: MODIFIED Event Line Format, Buffered Journal Flush; ADDED Service Brake Journal
    Events.
  - `ota-updates`: ADDED OTA Refused While Braking or Moving.
  - `mavlink-interface`: ADDED Service Brake Status Text.
- Affected code (aztec):
  - New: `include/IServiceBrake.h`, `include/IboosterProto.h`, `include/IboosterLaw.h`,
    `include/IboosterBrake.h`, `src/IboosterBrake.cpp`, `test/test_ibooster_proto/`,
    `test/test_ibooster_law/`.
  - Changed: `src/CANController.cpp` / `include/CANController.h` (slot clock, TXB1/TXB2, filter
    slots, task-side hooks, maintenance deferral), `src/VehicleController.cpp` /
    `include/VehicleController.h`, `include/ParkBrakeLaw.h`, `test/test_park_brake_law/`,
    `src/MavlinkInterface.cpp` / `include/MavlinkInterface.h` (brake `STATUSTEXT`),
    `include/EventJournalLaw.h` / `src/EventJournal.cpp` (`BRAKE`, flush deferral),
    `src/SpeedSensor.cpp`, `src/EngineHourMeter.cpp` (flash hold), `src/WebPortal.cpp` /
    `include/WebPortal.h`, `src/TelemetryManager.cpp`, `src/main.cpp`, `src/BoardInputs.cpp` /
    `include/BoardInputs.h`, `include/Constants.h`, `include/Changelog.h`, `data/index.html`,
    `platformio.ini`.
  - Deleted: `include/BrakeLaw.h`, `test/test_brake_law/`, the X7 `BTS7960Controller` instance.
  - Docs: `MAVLINK_SETUP.md` (brake `STATUSTEXT`s) and, per the project rule, the API compatibility
    page `UGV/API/README.md` in the parent folder; `GPIO_PINOUT_CUSTOM_BOARD_S3.md` (X7 free, In5 spare, X13 bus members and termination,
    X150 wiring), `WEB_PORTAL_SETUP.md` (brake card, tile, park-brake hand-over), `openspec/project.md`
    (aztec hardware line).
- Hardware: X7 harness and the actuator removed, iBooster fitted, X150 wired as above, bus
  termination checked.
- Safety: the brake gains real position feedback and a visible availability; it loses its function
  with the ignition OFF and on fail-safe, and a brake press can release the parking brake while the
  service brake is not acting (all accepted by the owner).
