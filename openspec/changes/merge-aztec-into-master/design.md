# Design: Merge aztec into master, keep the quad configuration

## Context
- `git merge-base master aztec` = `b5a2153` = the tip of `master`. `git log b5a2153..master` is
  empty, and `master..aztec` is linear (12 commits: 9 code commits and 3 proposal-only commits).
  `git merge aztec` on master is therefore a **fast-forward**. The merge produces no conflicts at
  all. The "conflict resolution toward the quad" is a deliberate edit after the fast-forward.
- `git diff --stat master aztec` shows 118 files, +14067 / −1338. Most of it is new files and the
  OpenSpec folders. Vehicle-specific code is concentrated in five places, in four commits:

  | Area | aztec commit(s) | Files |
  |---|---|---|
  | Relays / wheel lock / relay test | `cb70db1` | `Constants.h`, `RelayController.h/.cpp`, `VehicleController.*`, `WebPortal.*`, `TelemetryManager.cpp`, `index.html` |
  | Gear code | `7bed413` | `Constants.h`, `BoardInputs.h/.cpp`, `TransmissionController.h/.cpp`, `VehicleController.*`, `WebPortal.*`, `TelemetryManager.cpp`, `index.html` |
  | VESC on UART0 | `d255b8a` | `Constants.h`, `main.cpp`, `platformio.ini`, pinout doc |
  | Transmission servo name | `de4ab55`, `7bed413` | comments only (`Constants.h`, `TransmissionController.h`); values identical |

  `BoardInputs.*` and `TransmissionController.*` are touched by `7bed413` **only**.
  `RelayController.*` is touched by `cb70db1` **only**. No later aztec commit builds on them except
  through the telemetry and command plumbing listed above.
- Identity: `BOARD_NAME` is "Control_v0" on both branches. The SSID is `QuadBike-Control` on both.
  `index.html` already says "QuadBike Control Portal". The sidebar brand reads "🛠️ Service console"
  (`brand_title`, uk "Сервісна консоль") with the small hard-coded subtitle "QuadBike". The version
  label reads `1.5.2 · Control_v0`. So for the quad the identity is already right and nothing
  changes. (`WEB_PORTAL_SETUP.md` gives the wrong example "`1.4.0 · aztec`"; it is fixed in G.)

## Goals / Non-Goals
- Goal: master = aztec + the quad's relays, gears and VESC link. Every step is a small, buildable,
  reviewable commit.
- Goal: future master → aztec merges stay cheap. The vehicle-neutral fixes go back to aztec. The
  quad-config commit is recorded on aztec without its content.
- Non-goal: a vehicle build flag or a `[env:quad]` / `[env:aztec]` split (`unify-vehicle-builds`,
  later).
- Non-goal: re-tuning steering gains for the quad. Every `bench-tune` value comes from aztec and is
  checked on the bench.

## Decisions

### D1 — Fast-forward, then two follow-up commits (G, Q)
1. `git merge --ff-only aztec` on master. The untracked `openspec/changes/merge-aztec-into-master/`
   and `tree.txt` do not block it, because aztec has neither path.
2. **G — vehicle-neutral merge fixes.** It is built and verified while `Constants.h` is still the
   aztec one, so G proves itself on the aztec config. It contains:
   - `RelayController` as a superset (D2);
   - `index.html`: relay labels, the relay-test gate and the gear-code card driven by telemetry
     (D2, D3);
   - neutral comments in `platformio.ini` / `main.cpp` (D4);
   - `openspec/project.md`, `MAVLINK_SETUP.md`, `WEB_PORTAL_SETUP.md` wording for both vehicles;
   - this change folder.
3. **Q — quad configuration**:
   - `Constants.h` vehicle values (D2–D4);
   - master's gear decoder (D3) and the gear-code plumbing removal;
   - the vehicle sections of `GPIO_PINOUT_CUSTOM_BOARD_S3.md`;
   - `FIRMWARE_VERSION` 1.5.2 + the `Changelog.h` entry (D5).

A true `--no-ff` merge would only add an empty merge commit. The fast-forward + G + Q sequence is
simpler, and every quad-specific line is in one reviewable diff (Q).

### D2 — Relays: aztec's RelayController made a superset, quad masks in Constants.h
Finding: aztec's `RelayController` is **not** a superset as it stands:
- `setWheelLock()` is hard-coded to reject a lock.
- `getWheelLock()` is `return false`.
- There is no `wheelLockOn_` state.
- `computeShadow()` has no wheel-lock term.
- Three static_asserts pin aztec's single-relay layout: `relayIoMask(1) == RELAY_MASK_IGNITION`,
  `relayIoMask(2) == RELAY_MASK_STARTER`, `relayIoMask(3) == RELAY_MASK_FRONT_LIGHT`. All three
  fail with the quad masks.

Setting `RELAY_WHEEL_LOCK_FITTED 1` alone therefore neither compiles nor works.

The relay test, the verified writes, the starter escalation and the shadow logic are generic.
`RELAY_TEST_STARTER_MAX_MS` is applied through `mask & RELAY_MASK_STARTER`, so it follows the
starter to Relay_3 on the quad without a change.

**G (both vehicles, gated by the preprocessor, NOT `if constexpr`, because `RELAY_MASK_WHEEL_LOCK`
is undefined on aztec):**
- `RelayController.h`:
  - add a `bool wheelLockOn_` member (`#if RELAY_WHEEL_LOCK_FITTED`);
  - `getWheelLock()` returns `wheelLockOn_` when fitted and `false` otherwise;
  - the Doxygen header describes both assignments.
- `RelayController.cpp`:
  - `setWheelLock()`:
    - fitted → master's logic: refuse while the expander is unavailable; on a change, log
      "Front Wheel Lock: LOCKED/UNLOCKED", call `clearRelayTests("wheel lock command takes over")`
      when locking, then `updateRelays()`;
    - return `true` when applied or unchanged, `false` when the expander is unavailable;
    - not fitted → aztec's reject path, unchanged.
  - `computeShadow()`: when fitted, `shadow = withMask(shadow, RELAY_MASK_WHEEL_LOCK,
    wheelLockOn_)` before the `& ~RELAY_MASK_RESERVED` line.
  - `allOff()` keeps the wheel lock in its last state, as on master: it does not clear
    `wheelLockOn_`.
  - The starter escalation still forces the port to 0x00 (master behaviour, unchanged).
  - `setRelayTest()` ON gate: when fitted, also refuse while `wheelLockOn_`, with "Relay test
    refused - wheel lock is engaged (all relay functions must be off)".
  - static_asserts become vehicle-neutral:
    - function masks pairwise disjoint, disjoint from `RELAY_MASK_RESERVED`, and together with it
      covering `0xFF`. `RELAY_MASK_WHEEL_LOCK` counts as a function mask only when fitted.
    - The relay-test lookup assert "1..8 → all eight bits, 0 and 9 → 0" stays.
    - The three per-relay equality asserts move under `#if RELAY_WHEEL_LOCK_FITTED` / `#else`:
      - quad: `relayIoMask(1)|relayIoMask(2) == RELAY_MASK_IGNITION`, `relayIoMask(3) ==
        RELAY_MASK_STARTER`, `relayIoMask(4) == RELAY_MASK_FRONT_LIGHT`,
        `relayIoMask(5)|relayIoMask(6) == RELAY_MASK_WHEEL_LOCK`,
        `relayIoMask(7)|relayIoMask(8) == RELAY_MASK_RESERVED`;
      - aztec: the existing asserts.
- `VehicleController::setWheelLock()` / `processWheelLockCommand()` (aztec versions) already
  return and report the bool. They need no change.

**Q (quad values in `Constants.h`, taken from master):**
```
#define RELAY_MASK_IGNITION     0b00110000  // Relay_1 (IO4, X4) + Relay_2 (IO5, X5) — paralleled pair
#define RELAY_MASK_STARTER      0b01000000  // Relay_3 (IO6, X6)
#define RELAY_MASK_FRONT_LIGHT  0b10000000  // Relay_4 (IO7, X7)
#define RELAY_MASK_WHEEL_LOCK   0b00001100  // Relay_5 (IO3, X8) + Relay_6 (IO2, X9) — paralleled pair
#define RELAY_MASK_RESERVED     0b00000011  // Relay_7 (IO0) + Relay_8 (IO1) — spare
#define RELAY_WHEEL_LOCK_FITTED 1
```
`RELAY_TEST_TIMEOUT_MS` / `RELAY_TEST_STARTER_MAX_MS` are kept. Master's comment block above the
masks is restored, with the relay-test paragraph from aztec. The `ServoChannelConfig::WHEEL_LOCK`
and `EFI_DIGITAL_FLAG_WHEEL_LOCK` comments drop "not fitted on aztec" (master wording). The
`main.cpp` digitalFlags comment and the `relayController` declaration comment revert to master
wording.

**Relay test on the quad: it stays.** It is a wiring check with ignition OFF, auto-off after 30 s,
and a 2 s cap on the starter.
- On the quad, each relay of a paralleled pair is tested on its own. This checks each contact,
  which is the point of the test.
- Energizing Relay_5 or Relay_6 alone drives the wheel-lock actuator through one contact for at
  most 30 s. This is acceptable for a bench check, and it is noted in the help text.

**`index.html` (G, identical on both branches):**
- The relay labels are chosen by the existing `wheelLockFitted` flag. It is a proxy for the vehicle
  until `unify-vehicle-builds` adds a `vehicle` key.
  - New keys `relay_fn_q_1..8`, en/uk:
    - 1: "ignition (pair with 2)" / "запалювання (пара з 2)";
    - 2: "ignition (pair with 1)" / "запалювання (пара з 1)";
    - 3: "starter (max 2 s)" / "стартер (макс. 2 с)";
    - 4: "light" / "світло";
    - 5: "wheel lock (pair with 6)" / "блокування колеса (пара з 6)";
    - 6: "wheel lock (pair with 5)" / "блокування колеса (пара з 5)";
    - 7, 8: "spare" / "резерв".
  - `renderRelayTest()` uses `t((wheelLockFitted ? 'relay_fn_q_' : 'relay_fn_') + n)`.
- `relayTestAllowed` also requires `!data.wheel_lock_on`.
- `relay_test_help` / `relay_test_blocked` (en/uk) also mention "wheel lock released (if fitted)".
- The wheel-lock toggle already works when `wheel_lock_fitted` is true or absent. No change.

### D3 — Gear sensing: option (a), master's decoder; the gear-code UI is hidden by telemetry
**Option (b), the quad as a code table, was rejected.** Aztec's table cannot represent the quad:
- `extractGearCode()` reads bits 0/1/6 only, and the quad's HIGH line is bit 7.
- One-line-per-gear as a [b0 b1 b6] code gives N = 100, R = 010, L = 001 and HIGH = 000. But 000
  is reserved for UNKNOWN by a static_assert and by `isGearCodeTableValid()`.

Making it work needs a 4-bit code, and that touches a lot:
- `BoardInputs::extractGearCode`, the `GEAR_CODE_*` static_asserts (3-bit, 000/111 reserved);
- `isGearCodeTableValid`, `formatGearCode`, `decodeGearCode`'s `> 0b111` guard;
- the telemetry `gear_code` range, the web bit tiles (3 → 4) and `padStart(3)`;
- `update-aztec-gear-code-inputs`' spec text.

The aztec gear path would change too, and that path is bench-verified. It would also make the quad
operator's gear sensing re-calibratable for no benefit. This is higher risk, for both vehicles.

**Option (a), chosen.** The quad keeps master's bench-verified one-line-per-gear decoder,
unchanged. The aztec gear-code feature is simply not built on master:
- **Q, `Constants.h`:** master's gear block is restored. That is `BOARD_IN_BIT_GEAR_NEUTRAL 0`,
  `_REVERSE 1`, `_LOW 6`, `_HIGH 7`, with master's "engaged-HIGH" comment and the 2026-08-21 port
  bytes. All `BOARD_IN_BIT_GEAR_CODE_*`, `GEAR_CODE_*` and their static_asserts are deleted. The
  `TRANS_GEAR_READ_RETRY_COUNT` comment goes back to master's.
- **Q, files:** `git checkout b5a2153 -- include/BoardInputs.h src/BoardInputs.cpp
  include/TransmissionController.h src/TransmissionController.cpp`. Only `7bed413` changed them,
  so this takes nothing else from aztec. It also restores the "HappyModel Super400 Plus" servo
  comment.
- **Q, plumbing removal:**
  - `TelemetryManager.cpp`: delete the gear-code block (`gear_code`, `gear_sensed`,
    `gear_code_r/n/h/l`, `gear_codes_custom`).
  - `include/WebPortal.h`: delete those struct fields.
  - `src/WebPortal.cpp`: delete the `gear_code` / `gear_sensed` / `gear_codes` /
    `gear_codes_custom` emission, and drop the "gear_code / gear_sensed / gear_codes_custom" and
    "gear_codes" items from the capacity comment. The size stays at 4096.
  - `VehicleController.h/.cpp`: delete `processGearCodeCommand()`. The `set_gear_code` /
    `reset_gear_codes` dispatch now answers `sendResponse(false, "Gear codes not used on this
    vehicle")`, so an old cached page gets an explicit answer.
- **What the 1.4.0+ page does when the keys are missing (checked):**
  - The "under current gear" line (`telemetry-gear-code`) is already blanked when `gear_code` is
    undefined.
  - `updateGearCodes()` already skips both blocks.
  - **But the "🔢 Gear Sensor Codes" card stays visible**, with "—" and live Save buttons.
- **G, `index.html`:** the card gets `id="card-gear-codes"` and `style="display:none"`. The
  telemetry handler shows it only when `data.gear_codes` is present, and hides it when it is
  absent. The comment changes to "aztec 3-bit gear code calibration — hidden when the firmware
  sends no `gear_codes`". Aztec is unaffected, because it always sends the key.

### D4 — Steering VESC link: UART2 / X6 on the quad; the UART0 log guard stays
- **Q, `Constants.h`:** master's block is restored:
  - `PIN_VESC_TX GPIO_NUM_4`, `PIN_VESC_RX GPIO_NUM_5`, `VESC_UART_NUM 2`, with the "Connector X6
    via BSS138 shifters — 115200 only" comment;
  - the "Steering Actuator … over UART2" line;
  - the reserved-pin note: GPIO43/44 = UART0 to the CH9121T LAN chip, reserved, no console. It
    replaces aztec's "GPIO4/5 FREE" and "GPIO43/44 carry the VESC" lines.
- Everything else in that block stays from aztec, because it is LWS-related and true on the quad:
  - `PIN_I2C1_SDA/SCL` replacing `PIN_STEER_SDA/SCL`; X14 has no device and `Wire` is not started;
  - the I2C bus-map text;
  - the logging-section sentence "UART0 … must never be reclaimed for a console". It is reworded:
    UART0 = steering VESC on aztec, CH9121T link on the quad.
  - `VescMotorDriver` is unchanged between the branches. It takes `VESC_UART_NUM`.
- **`src/main.cpp` / `platformio.ini`: nothing is aztec-only in effect.**
  - `uartSetDebug(NULL)` + `esp_log_set_vprintf(usbOnlyLogVprintf)` move log output from UART0 to
    USB-CDC. On the quad, UART0 drives the CH9121T (R63/R64 fitted), so keeping log text off it is
    an improvement. ESP-IDF component logs may now also appear on the USB-CDC console. This is
    harmless.
  - `-DCORE_DEBUG_LEVEL=0` matches the Arduino-ESP32 PlatformIO default (`build.code_debug` = 0),
    so it is a no-op on the quad. Confirm this with `pio run -v` if in doubt.
  - Both stay. **G** rewords their comments: "UART0 (GPIO43/44) is the steering VESC on aztec / the
    reserved CH9121T link on the quad — never put a console or log output on it".
  - The `steeringVesc` comment in `main.cpp` becomes "over `VESC_UART_NUM` (UART2/X6 on the quad,
    UART0/X1 on aztec)".
- The rest of the `main.cpp` diff (I2C1 not started, LWS boot centring, `steerMode` report) is
  steering/LWS work and is kept.

### D5 — Version and changelog
- `FIRMWARE_VERSION` "1.5.2" on master (in Q). The code is aztec 1.5.1 plus quad configuration,
  which is a patch-level difference from 1.5.1.
- New first `Changelog.h` entry, dated the implementation date, uk/en, written for the quad
  operator (task 5.2 has the exact lines). It says what the quad gains since 1.1.0 and that
  steering recalibration is required.
- Entries 1.2.0–1.5.1 stay as history. Some lines are buggy-only (dual AS5600, gear codes, relay
  layout, "wheel lock not fitted"). This is accepted, because the 1.5.2 entry is the quad
  operator's summary.
- Aztec stays at 1.5.1. 1.6.0 stays reserved for `move-can-rx-to-interrupt`, implemented on master
  and merged into aztec.
- Consequence: the first master → aztec merge after Q conflicts on the `FIRMWARE_VERSION` line and
  on the top of `Changelog.h`. These are trivial conflicts: take the new version and keep both
  entries.

### D6 — Keeping aztec in step (branch policy)
- After Q, on aztec:
  1. `git merge --ff-only <G>`. aztec's tip is G's parent, so this fast-forwards, and aztec gets
     the superset `RelayController` and the telemetry-driven UI.
  2. `git merge -s ours <Q> -m "Record quad configuration (master) as merged; aztec keeps its
     vehicle config"`.

  From then on, master → aztec merges never re-apply Q.
- Master commits that later touch the three vehicle areas conflict on aztec by design. Resolve them
  toward aztec on aztec, and toward the quad on master.
- These are branch operations. They run only with the user's go-ahead (task 7).

### D7 — Specs: how the aztec-only requirements coexist with the quad
- All twelve arriving folders are archived on master, in the order their proposals document:
  1. `add-aztec-dual-as5600-steering`
  2. `update-aztec-vesc-uart0`
  3. `update-aztec-gear-code-inputs`
  4. `update-aztec-relay-assignment`
  5. `replace-aztec-steering-sensor-with-bosch-lws`
  6. `redesign-web-ui-service-console`
  7. `add-aztec-steering-rate-mode`
  8. `auto-detect-aztec-steering-sensor-direction`
  9. `update-throttle-inverted-calibration` (standalone, any time)
  10. **this change**
  11. (pending, later) `move-can-rx-to-interrupt` → `move-aztec-vesc-to-can`;
      `add-speed-limiter-integral-floor` (standalone).
- The three vehicle-specific folders cannot be skipped. `redesign-web-ui-service-console` MODIFIES
  "Gear Code Calibration via Web Interface" and "Web Relay Test Control", which they ADD. The
  later steering changes also build on the `Hardware Configuration` text that
  `update-aztec-vesc-uart0` modifies.
- Their requirement text already names the vehicle ("The aztec buggy SHALL…", "Relay_3, IO6, on
  the aztec buggy", "UART0 … aztec board").
- This change ADDS three requirements that state the quad build's values explicitly ("On the quad
  bike (`master`) build …"). After archiving, `openspec/specs/` therefore says, per area, what
  each vehicle does. The aztec requirement and the quad requirement sit side by side, with no
  contradiction, because each one names its vehicle.
- Merging them into single per-vehicle requirements, together with a build flag, is
  `unify-vehicle-builds` (later, out of scope).

## Risks / Trade-offs
- **Quad steering before the LWS is fitted.** With 1.5.2 and no LWS on CAN:
  - percent steering is refused;
  - the autopilot's autonomous modes get no steering ("blocked");
  - only MANUAL rate mode and web jog/nudge work.

  Mitigation: flash the quad only after the LWS is mounted and wired (precondition, task 0.4).
- **CAN ID clash.** The quad's ECU shares the bus. If it ever sends 0x305, the LWS filter would
  accept garbage. Only CRC/counter checks in `LwsSensor` would reject it. Bench check: log the bus
  with the LWS unplugged (task 8.3).
- **NVS:** the quad's v1 steering keys (`c_ang/l_ang/r_ang`) are deleted by the v4 migration, so
  recalibration is required. The `transmission` gear positions and the `throttle` endpoints are
  untouched. The quad never has `code_*` keys.
- **AP address change** 4.1 → 10.1 (user decision): bookmarks and any companion configured for
  192.168.4.1 need updating.
- **Bench-tune values** (`STEER_KP`, tolerances, `STEER_LWS_SAFE_RANGE` 550°) were tuned on the
  buggy's steering column. The quad's lock-to-lock angle may differ. `STEER_CAL_MIN_SPAN` = 20°
  must be smaller than the quad's half travel.
- **Divergent files** between the branches until `unify-vehicle-builds`: the gear files,
  `Constants.h` vehicle blocks, the gear telemetry plumbing and the pinout doc sections.
  Everything else stays identical, which keeps future merges small.

## Open Questions
None. All three were decided by the user on 2026-10-31:
1. **Flash timing — decided:** the Bosch LWS is fitted on the quad before 1.5.2 is flashed, and the
   AS5600 on X14 is physically removed (the firmware no longer reads it).
2. **Version scheme — decided:** master goes to 1.5.2; aztec stays at 1.5.1 (D5).
3. **Aztec bookkeeping — decided:** approved as in D6: fast-forward aztec to G, then
   `git merge -s ours Q`.
