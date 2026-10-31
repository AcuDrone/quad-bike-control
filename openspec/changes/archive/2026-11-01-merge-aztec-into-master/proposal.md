# Change: Merge the aztec branch into master, keeping the quad bike's vehicle configuration

## Why
The quad bike will get the same Bosch LWS steering-angle sensor on CAN as the aztec buggy (user
decision 2026-10-31). So everything developed on `aztec` since it branched from `master` at
`b5a2153` (firmware 1.1.0) is wanted on the quad too:
- the LWS steering;
- the sensorless rate mode and the sensor direction inference;
- the service-console web UI;
- the mirrored-servo throttle calibration;
- the changelog mechanism;
- the native tests and the MAVLink additions.

Only three hardware areas really differ between the two vehicles: the relay assignment
(with the front-wheel lock), gear sensing, and the steering VESC link.

Today the two branches diverge further with every change. All new work (speed limiter floor, CAN RX
interrupt, VESC on CAN) is planned on `aztec` only. This change makes `master` the trunk again:
master takes aztec as-is and restores the quad's configuration in those three areas. From then on,
work is done on master and merged into aztec.

This is mostly a branch operation. There is no build switch or vehicle flag yet. That is a later
change (`unify-vehicle-builds`), once both vehicles are on the bench.

## What Changes
- **Branch operation.** `master` has no commits after the merge base (`git log b5a2153..master` is
  empty), so `git merge aztec` on master is a **fast-forward**. There are no merge conflicts to
  resolve. The quad configuration is restored afterwards in reviewable follow-up commits:
  - **G, vehicle-neutral.** It goes back to aztec too:
    - `RelayController` becomes a true superset, gated by `RELAY_WHEEL_LOCK_FITTED`;
    - the web UI picks the relay labels, the relay-test gate and the visibility of the gear-code
      card from telemetry;
    - comments and docs are reworded so they are true for both vehicles.
  - **Q, quad configuration.** It is recorded on aztec with `git merge -s ours`, so it never
    overwrites aztec's config:
    - relay masks and wheel lock fitted;
    - one-line-per-gear sensing from master;
    - VESC on UART2 / X6;
    - pinout doc vehicle sections;
    - version and changelog.
- **Master keeps (quad hardware):**
  - **Relays.** Master's assignment:
    - ignition = Relay_1 + Relay_2 (paralleled);
    - starter = Relay_3;
    - front light = Relay_4;
    - front-wheel lock = Relay_5 + Relay_6 (paralleled), **fitted**, MAVLink channel 7;
    - Relay_7/8 spare.

    Aztec's web relay test stays on the quad. Its gate is extended by "wheel lock released", and it
    gets the quad relay labels.
  - **Gear sensing.** Master's one-line-per-gear decoder (NEUTRAL = bit 0, REVERSE = bit 1,
    LOW = bit 6, HIGH = bit 7, engaged-HIGH, exactly one line → gear, otherwise UNKNOWN).
    Recommended option (a), see `design.md` D3. Aztec's 3-bit code table, its NVS keys, the
    `gear_code*` telemetry and the `set_gear_code` / `reset_gear_codes` commands are not built on
    the quad. The web "Коди датчика передач" card is hidden when `gear_codes` is absent from
    telemetry.
  - **Steering VESC link.** UART2 on X6 (GPIO4 TX / GPIO5 RX via the BSS138 shifters). It does not
    use aztec's UART0 / X1 (GPIO43/44), and R63/R64 to the CH9121T stay fitted on the quad board.
    Aztec's UART0 log suppression (`uartSetDebug(NULL)`, `esp_log_set_vprintf` → USB-CDC) and
    `-DCORE_DEBUG_LEVEL=0` are kept: they are harmless on the quad, and they keep log text off the
    CH9121T. Only their comments are reworded.
- **Master takes from aztec as-is:**
  - LWS steering: `Lws305.h`, `LwsSensor`, the `SteeringController` rewrite, the CAN filters and
    the zero-set;
  - rate mode (`SteerRate.h`) and sensor direction inference (`SteerSensorDir.h`);
  - service console `data/index.html`, with the wheel-lock toggle working because
    `wheel_lock_fitted` = true;
  - mirrored-servo throttle calibration;
  - `Changelog.h` entries 1.2.0–1.5.1;
  - `WIFI_AP_IP` 192.168.10.1 (user decision for the quad too);
  - `[env:native]` tests, MAVLink `STEER_MODE` etc. and all new telemetry keys;
  - `DRONECAN_RESEARCH.md` and the doc updates;
  - the `openspec/project.md` versioning rule;
  - **all twelve** aztec `openspec/changes/*` folders. They are archived once, on master, in their
    documented order.
- **Version.** `FIRMWARE_VERSION` "1.5.1" → **"1.5.2"** on master, with a new `Changelog.h` entry
  (uk/en) written for the quad operator. It says that steering recalibration (sensor zero +
  limits) is required. Aztec stays at 1.5.1. 1.6.0 stays reserved for `move-can-rx-to-interrupt`.
- **Spec delta:** three small ADDED requirements in `vehicle-systems` that state the quad build's
  relay/wheel-lock, gear-sensing and VESC-link configuration. Everything else the quad gains
  (AP IP, web UI, LWS, rate mode) is delivered by the arriving aztec change folders and is **not**
  duplicated here.

## Impact
- Affected specs: `vehicle-systems` (ADDED "Quad Bike Relay Assignment With Fitted Front-Wheel
  Lock", "Quad Bike One-Line-Per-Gear Sensing", "Quad Bike Steering VESC on UART2").
- Arriving change folders (implemented on aztec, archived on master after this merge):
  - `add-aztec-dual-as5600-steering`, `update-aztec-vesc-uart0`, `update-aztec-gear-code-inputs`,
    `update-aztec-relay-assignment`, `replace-aztec-steering-sensor-with-bosch-lws`,
    `redesign-web-ui-service-console`, `add-aztec-steering-rate-mode`,
    `auto-detect-aztec-steering-sensor-direction`, `update-throttle-inverted-calibration`;
  - pending (proposals only): `add-speed-limiter-integral-floor`, `move-can-rx-to-interrupt`,
    `move-aztec-vesc-to-can`.
- Affected code on master after the fast-forward:
  - **G:** `include/RelayController.h`, `src/RelayController.cpp`, `data/index.html`,
    `platformio.ini` and `src/main.cpp` (comments only), `MAVLINK_SETUP.md`,
    `WEB_PORTAL_SETUP.md`, `openspec/project.md`.
  - **Q:**
    - `include/Constants.h`;
    - `include/BoardInputs.h`, `src/BoardInputs.cpp`, `include/TransmissionController.h`,
      `src/TransmissionController.cpp` (master versions);
    - `src/TelemetryManager.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`,
      `include/VehicleController.h`, `src/VehicleController.cpp` (gear-code removal);
    - `GPIO_PINOUT_CUSTOM_BOARD_S3.md`, `include/Changelog.h`.
- **Breaking for the quad operator:**
  - The AS5600 on X14 is no longer read. Steering needs the Bosch LWS on CAN. Without it, only the
    sensorless rate mode in MANUAL and web jog/nudge move the wheels.
  - The old steering calibration (`c_ang/l_ang/r_ang`) is removed on first boot, so recalibration
    is required.
  - The web address changes to 192.168.10.1.
- **Precondition for flashing the quad:** the LWS is mounted on the steering shaft and wired to the
  MCP2515 bus. The quad ECU must not use CAN ID 0x305 or 0x7C0 (bench check).
- **Branch policy afterwards:** work is done on master and merged into aztec. Commits that touch the
  three vehicle areas conflict on that merge by design; resolve them toward aztec on aztec. See
  `design.md` D6.
