## Context
The relays (ignition, starter, front light, reserved locks/spares) are driven by `RelayController`
through `PCA9557Expander` bound to `0x1F` on I2C2 (`Wire1`, header X15, 100 kHz). The PCA9557 relay
board routes Relay_N to a scrambled IO bit (`RelayController::relayIoMask()`: Relay_1=IO4 …
Relay_8=IO1), so `Constants.h` defines whole-port `RELAY_MASK_*` values and `RelayController.cpp`
static_asserts them against that table. Each write is verified by reading the **output register**
back (`readOutputRegister()`), retried `RELAY_WRITE_RETRIES` times, with a separate starter
escalation; while faulted, `update()` retries the safe-state write once per call.

The same bus carries the on-board PCA9557s U10 `0x1A` (opto inputs) and U2 `0x1C` (CD4051 mux,
never addressed). Both sit in the PCA9557 block `0x18`..`0x1F`.

New board: **XL9535-K8V5** — 8 relays (5 V coils, optocoupler + FET), XL9535 16-bit expander
(PCA9535/TCA9535-compatible).

| Item | Value |
|------|-------|
| 7-bit address | `0x20`..`0x27` via solder pads A0..A2, default `0x20` |
| Registers | `0x00`/`0x01` input P0/P1, `0x02`/`0x03` output P0/P1, `0x04`/`0x05` polarity P0/P1, `0x06`/`0x07` config P0/P1 (1 = input, 0 = output) |
| Power-on defaults | output `0xFF`, polarity `0x00`, config `0xFF` |
| Output register read | returns the latch, not the pin |
| Input register read | returns the actual pin level, also for pins configured as outputs |
| Reset | no RESET pin; internal POR only |
| Bus speed | max 400 kHz; we stay at the bus's 100 kHz |
| Relay mapping (believed) | Relay_1..Relay_8 = P0.0..P0.7, active-high (bit 1 = energized) — **not yet bench-confirmed on a K8 board** |

Electrical (hardware task, not firmware): remove jumper J1; VIN (logic + the board's on-board I2C
pull-ups) from the X15 I2C high-side rail (3.3 V side); the 5V relay terminal fed separately
(8 × ~70 mA ≈ 0.6 A); shared GND; no I2C isolation.

Settings: there is no single settings store. Every persisted setting is an NVS key written through
`Preferences`, in a namespace owned by its controller (`steering`/`inv` for `steer_sensor_inv`,
`speed`/`slew` for the limiter rate, `bat12`/`cal`, `brake`, ...). The `nvs` partition
(`0x9000`, `partitions_16mb.csv`) is outside `app0`/`app1`, so it survives OTA. This change follows
that pattern with namespace `relay`, key `board`.

## Goals / Non-Goals
- Goals: one firmware image drives either relay board; a wrong or ambiguous bus never energizes a
  relay; a silent expander reset is detected and recovered; relay numbers, web relay test and
  MAVLink behaviour unchanged.
- Non-Goals: driving P1 of the XL9535 (relays 9..16); hot switching boards; I2C isolation; a web
  reboot button; detecting welded contacts (still out of scope); any change on the quad-bike
  `master` branch beyond what the shared spec text describes.

## Decisions

### a) `IRelayPort` interface
`include/IRelayPort.h`, pure virtual, same style as `IMotorDriver`:
`isPresent()`, `initAllOff()`, `writeOutputs(uint8_t)`, `readOutputRegister(uint8_t&)`,
`readPins(uint8_t&)`, `address()`, `name()` (log string, `"PCA9557"` / `"XL9535"`), and the probe
range used by the failure scan. The init sequence is implemented once in the interface as
`initPort(seed)` (Output = seed → Polarity 0 → Config 0 → pins == seed) on top of the per-chip
register primitives (`writePolarity()`, `writeConfig()`); `initAllOff()` = `initPort(0x00)`. `PCA9557Expander` implements it (its existing register methods
stay; `readPins()` = register `0x00`). New `XL9535Expander` implements it on **P0 only** (output
`0x02`, polarity `0x04`, config `0x06`, input `0x00`); P1 stays at power-on defaults (inputs) and
is never written. Every method is at most one I2C transaction, no retry, no sleep — retry policy
stays in `RelayController`. `RelayController` owns one instance of each driver and points an
`IRelayPort*` at the detected one (no heap allocation).
*Rationale:* the controller logic (shadow byte, ignition, starter escalation, test) is chip-agnostic;
only register addresses and the pin table differ.

### b) `initAllOff()` order is mandatory
1. Output = `0x00` (latch all-off while the pins are still inputs),
2. Polarity = `0x00` (needed for a truthful pin read; the PCA9557's power-on polarity is `0xF0`),
3. Config = `0x00` (outputs enabled — they come up at the already-latched 0),
4. `readPins()` must return `0x00`.
Writing Config first would, on the XL9535, momentarily energize all eight relays (latch default
`0xFF`), including the starter. Any failed step returns `false` with the step named in the log.

### c) Verify against the pins, recover a silent reset
`writePort()` writes the latch and verifies with `readPins()` == shadow (both chips). A reset
expander (XL9535 brown-out: config back to all-inputs, relays dropped, while a latch read-back
could still match) now fails verification. On mismatch: the existing retry
(`RELAY_WRITE_RETRIES`), fault raise, test-relay drop and starter escalation, unchanged. While
faulted, `update()` keeps its existing single-attempt write per call; additionally every
`RELAY_RECOVER_MS` (1000 ms, same value as `BOARD_INPUT_RECOVER_MS`) it replaces that attempt by a
re-init seeded with the **current shadow byte** — Output = shadow, then Polarity = `0x00`, then
Config = `0x00`, then a pin read-back that must equal the shadow (`IRelayPort::initPort(shadow)`) —
followed by one verified write of the shadow (all-off while the starter escalation is latched). The
starter bit is never seeded: if the shadow holds it when recovery runs, the starter escalation
(all-off, ignition OFF, crank inhibit) applies first and the recovery seeds that all-off state. At
boot `begin()` still uses `initAllOff()` (seed `0x00`). The single-attempt rule keeps a wedged bus from costing the loop several I2C timeouts per
iteration. A detected board whose `initAllOff()` fails in `begin()` enters the same faulted
recovery (its shadow is all-off, so the recovery seed is `0x00`) (the board identity is latched, so re-initializing it is safe); a **detection** fault is
not retried.

### d) Board selection, detection once at boot
Setting `relay_board` ∈ {`auto` (0), `pca9557` (1), `xl9535` (2)}, NVS `relay`/`board` (uint8),
default `auto`; an absent key reads as `auto`, an out-of-range value is logged and treated as
`auto` (never rewritten). It is read in `RelayController::begin()` — called from `setup()` after
`Wire1` is up and before anything else writes a relay. Probing is bounded zero-length writes to
`0x1F` and `0x20`..`0x27` only (9 probes); `0x1A`/`0x1C` are never treated as a relay board.
The decision is a pure function `relayBoardDecide(ackMask, setting)` in `RelayBoardLaw.h`:

| Setting | Bus | Result |
|---------|-----|--------|
| auto | `0x1F` only | PCA9557 @`0x1F` |
| auto | exactly one of `0x20`..`0x27`, no `0x1F` | XL9535 @ that address |
| auto | `0x1F` and any of `0x20`..`0x27` | fault `both_found` |
| auto / pca9557 / xl9535 | none of `0x1F`, `0x20`..`0x27` | fault `none_found` |
| auto / xl9535 | two or more of `0x20`..`0x27` | fault `multiple_xl9535` |
| pca9557 | `0x1F` present (XL9535 addresses ignored) | PCA9557 @`0x1F` |
| pca9557 | no `0x1F`, some XL9535 address | fault `cfg_mismatch` |
| xl9535 | exactly one of `0x20`..`0x27` (`0x1F` ignored) | XL9535 @ that address |
| xl9535 | no `0x20`..`0x27`, `0x1F` present | fault `cfg_mismatch` |

A manual selection therefore resolves `both_found` (that is what the override is for), but never
the choice between two XL9535 addresses. On a detection fault nothing is written to any address,
the controller stays uninitialized (every setter a logging no-op, `io_relay_fault` = true), the
reason is logged once and published as `relay_board_error`, and the diagnostic scan of
`0x18`..`0x27` lists every ACKing address. Never guess. The result is latched until reboot.

### e) No hot switching
`set_relay_board` persists the new value (persist first, like `set_speed_limit_slew`) and replies
"…takes effect after reboot"; the running board is untouched. Telemetry carries the stored value
(`relay_board_cfg`), the active board and address (`relay_board`, `relay_board_addr`), and
`relay_board_reboot` = stored value differs from the value used at boot. The UI renders e.g.
"XL9535 @0x20 (auto)".

### f) Relay-number sets in `Constants.h`, masks built at runtime
`Constants.h` keeps the per-vehicle assignment as relay-number sets (bit0 = Relay_1 … bit7 =
Relay_8, the same encoding as the `relay_test` telemetry), e.g. via `RELAY_N(n)`:
- aztec: `RELAY_SET_IGNITION = RELAY_N(1)`, `RELAY_SET_STARTER = RELAY_N(2)`,
  `RELAY_SET_FRONT_LIGHT = RELAY_N(3)`, `RELAY_SET_RESERVED` = Relay_4..Relay_8;
- quad bike (`master`): ignition = {1, 2}, starter = {3}, light = {4}, wheel lock = {5, 6},
  reserved = {7, 8}.
`RelayBoardLaw.h` holds both Relay_N → bit tables as constexpr arrays (PCA9557: the
`Relay.PrjPcb` table now in `relayIoMask()`; XL9535: identity) and `relayPortMask(table, set)`.
`begin()` builds `maskIgnition_`, `maskStarter_`, `maskFrontLight_`, (`maskWheelLock_`),
`maskReserved_` from the detected table; `relayIoMask()` and `isStarterEnergized()` use the
runtime masks. Compile time: the sets are disjoint and cover all eight relays; each table is a
permutation of the eight port bits; the mapped masks are disjoint and OR to `0xFF` for **both**
tables. `begin()` repeats the mask checks at runtime and refuses to initialize on failure.

### g) Behaviour unchanged
Web relay test (gate, 30 s auto-off, 2 s starter cap, take-over), MAVLink wheel-lock plumbing
(channel 7, `EFI_DIGITAL_FLAG_WHEEL_LOCK`), ignition state machine, pre-crank dwell, starter
escalation and `allOff()` are unchanged; only the Relay_N → bit table behind them depends on the
board.

### h) Host-side test
`test/test_relay_board/test_main.cpp` (Unity, Arduino-free, header-only like the other suites)
covers: mask building for both tables (aztec and quad sets), disjoint/union invariants, table
permutation checks, and every row of the detection table above plus out-of-range setting values.
One new indented `test_filter` line in `[env:native]`; `build_src_filter` unchanged (header-only).

## Alternatives considered
- **Compile-time `#define RELAY_BOARD_XL9535`**: rejected — two images per vehicle, and a board
  swap in the field needs a reflash; a wrong image silently drives the wrong bits.
- **Auto-detect without override**: rejected — with both boards on the bus (bench, migration) the
  operator would have no way to pick one without unplugging hardware.
- **Hot switching on setting change**: rejected — re-targeting a live relay port mid-drive risks a
  half-written state on two boards; reboot is cheap and deterministic.
- **Keep latch read-back**: rejected — blind to the XL9535 brown-out reset (the main new failure mode).

## Risks / Trade-offs
- **Unconfirmed K8 mapping/polarity** → a bench task (Output P0 = `0x01`, then Config P0 = `0xFE`,
  Relay_1 must click and the coil LED light) gates trusting the identity table; if the board is
  active-low or permuted, only the XL9535 table (and, for active-low, an inversion in the driver)
  changes.
- **Momentary all-on if the init order is wrong** → order fixed in `initAllOff()`, documented in the
  driver header, reviewed in tasks; bench check with the starter output disconnected first.
- **Power-up before firmware runs**: with config at all-inputs the relay inputs float/pull up
  through the board's input stage; whether that energizes a relay is unknown → bench task observes
  the relays from power-on until `begin()` completes.
- **Brown-out reset** → pin verification + timed recovery. Recovery re-initializes the port seeded
  with the current shadow (output latch first, then polarity, then config), so a fault caused by a
  transient bus error without a real reset does not blip an energized ignition relay, and after a
  real reset the relays come back straight to the commanded state. The all-relays-on glitch stays
  impossible because the output latch is written before the configuration register. The starter
  bit is never seeded (escalation first). See Open Questions (resolved).
- **Mis-detection** → only `0x1F` and `0x20`..`0x27` are probed for the decision; every ambiguity is a
  fault with nothing written; the active board is shown in telemetry so a wrong board is visible.
- **VIN from 3.3 V rail**: a sagging X15 rail resets the XL9535 → covered by recovery; bench
  brown-out check.

## Migration Plan
Flash firmware and the LittleFS image. With no `relay/board` key the firmware runs `auto`, which
picks the PCA9557 board on existing vehicles exactly as before. To fit the XL9535: wire per the
hardware task, leave `auto` (or select `xl9535`), reboot, confirm "XL9535 @0x20 (auto)" in
telemetry and click through the relay test. Rollback: flash the previous firmware (ignores the key;
PCA9557 board only).

## Open Questions
- ~~Should recovery seed the output latch with the current shadow instead of `0x00` (avoids the
  ignition blip on a transient-error recovery)?~~ **Resolved (approved amendment):** yes. On fault
  recovery (not at boot) the re-init writes Output = current shadow, then Polarity = `0x00`, then
  Config = `0x00`, then verifies the pins against the shadow. A bus glitch that was not a real
  expander reset must not blip the ignition relay; the all-relays-on glitch is still impossible
  because the latch is written before the config register. `begin()` keeps the all-off sequence
  (`initAllOff()`). The starter bit is never seeded: a shadow holding it triggers the existing
  starter escalation (all-off, ignition OFF, crank inhibit) and the recovery seeds that all-off
  state.
