## Context

`Control_v0` (ESP32-S3-WROOM-1U, Altium project `Control_v0.PrjPcb`, 7 sheets) replaces the
DevKitC-1 + hand wiring. Two companion boards hang off it: `Relay.PrjPcb` (8× SRD-12VDC relays +
PCA9557, via X15 / I2C2) and `Buck Boost.PrjPcb` (TL494 12V→24V, via X10). The full wiring
reference —
including the connector table, the opto/relay assignment and the bring-up warnings — is
`GPIO_PINOUT_CUSTOM_BOARD_S3.md`; this document only covers the firmware architecture decisions the
migration forces.

Three structural changes drive everything else:

1. Functions that were direct GPIO (gear switches, brake limit sensor, four relays) are now behind
   I2C port expanders, so reads/writes become fallible and latency-bearing.
2. There are now **two** I2C buses with four devices between them, two of which the firmware must
   deliberately *not* touch in this change.
3. The console leaves UART0 for USB-CDC, which changes what is observable during early boot.

The firmware is a single cooperative `loop()` with no FreeRTOS tasks (`src/main.cpp:167`); every new
subsystem must follow the existing `begin()` + non-blocking `update()` pattern.

## Goals / Non-Goals

**Goals**

- One authoritative pin map (`include/Constants.h`) that matches `Control_v0` exactly, with no
  DevKit compatibility layer and no dead placeholder constants.
- Gear, brake-limit and relay behavior preserved from the consumer's point of view: the
  `RelayController` public API and `TransmissionController::getPhysicalGear()` semantics do not change.
- Explicit, testable failure behavior for both expanders — this is a safety-relevant vehicle, and an
  I2C bus that is now in the path of the starter relay and the gear interlock cannot fail silently.
- The 24V rail comes up automatically and its voltage is observable.

**Non-Goals**

- No dual-board support, no runtime board detection, no `#ifdef BOARD_DEVKIT`.
- No CD4051 current-sense mux driving (defines only), no wired LAN, no S-BUS (dropped for good — its
  GPIO8 / X2 6N137 opto input is reallocated to the driveline speed sensor by
  `add-hall-speed-sensor`), no speed sensor.
- No interrupt-driven expander reads (`PIN_CAN_INT` is likewise defined but unused) — polling is
  sufficient at the required rates and keeps the cooperative-loop model intact.

## Decisions

### Decision 1 — Two `TwoWire` instances, hard-assigned by role

**The assignment below is empirical** — it comes from live I2C scans on the assembled board and
overrides the schematic's port names, which suggested the opto-input expander U10 was on I2C1.

`Wire` = I2C1 (SDA GPIO1, SCL GPIO2, 100 kHz): the **external-header** bus — header X14, reserved
for the AS5600 steering sensor @ **0x36** alone.
`Wire1` = I2C2 (SDA GPIO48, SCL GPIO47, 100 kHz, BSS138 level-shifted): **both on-board PCA9557s** —
opto-input U10 @ **0x1A** and CD4051 mux-control U2 @ **0x1C** — plus the relay board @ **0x1F** on
header X15.

| Bus | Instance | SDA | SCL | Speed | Devices |
|-----|----------|-----|-----|-------|---------|
| I2C1 | `Wire` | 1 | 2 | 100 kHz | AS5600 `0x36` (X14) |
| I2C2 | `Wire1` | 48 | 47 | 100 kHz | opto-input PCA9557 U10 `0x1A`; mux PCA9557 U2 `0x1C` (**do not touch**); relay-board PCA9557 `0x1F` (X15) |

The relay board's as-shipped strap `A2A1A0 = 111` → **0x1F** is kept: it collides with neither
`0x1A` nor `0x1C`, so it is valid on either bus. On the first assembled board header X15 passed no
signals at all — a device there never answered while the same device answered immediately on X14 —
until rework in the `FB3`/`FB4` ferrite area; that is the first place to look if another board copy
shows a silent X15. Because the strap is bus-agnostic, running the relay board on X14 / `Wire`
instead is a one-argument change, kept as a contingency only.

**Both buses run at 100 kHz, not 400 kHz.** The AS5600 is not on the board — it hangs off X14 on a
long cable run to the steering column, and that cable plus the connector is the weakest link on
I2C1. 400 kHz was tried and rejected: the edges were too marginal for a bus that carries the
steering position feedback. 100 kHz buys back the signal-integrity margin at a cost that does not
matter here — the steering loop reads a handful of AS5600 registers per pass and the relay/opto
transactions are a few bytes each, so even at 100 kHz every transaction is well inside the loop
budget. Both buses share the single `I2C_BUS_FREQ_HZ` constant rather than being tuned separately:
I2C2 could run faster (short traces, BSS138-shifted), but one constant is simpler and there is no
measured need. If faster steering reads are ever wanted, the change is a hardware one first — a
stronger pull-up on the X14 run — verified on the bench before the constant moves.

**U2 @ 0x1C must never be written by any driver.** A PCA9557 driver instance is bound to one
address at construction and the relay driver is constructed with `PCA9557_ADDR_RELAYS` only; there
is no "scan and take the first expander" path. Writing 0x1C would toggle the CD4051
channel-select/enable lines — harmless today (the ADC mux is unused) but it would become a real
fault once IS monitoring lands. The relay strap only has to avoid `0x1A` and `0x1C`; `0x1F` does, so
no strap rework is needed (§8.8 of the pinout doc was updated accordingly).

*Alternatives considered.* Putting the relay board on `Wire` next to the AS5600 is rejected: the
steering loop is the most latency-sensitive I2C consumer and relay writes are bursty, and the board
physically routes X15 to GPIO47/48 anyway. It was proven to work during bring-up (while X15 was
still dead) and the cost turned out to be acceptable — relay writes are a few bytes at 100 kHz and
only happen on state changes — so it stays available as a contingency, not as the design.

*Diagnostic sweep at `begin()`.* If nothing answers at `PCA9557_ADDR_RELAYS`, the relay driver
probes the whole PCA9557 block `0x18`-`0x1F` on **its own bus** and logs which addresses ACKed. The
earlier "probe 0x1C and blame a mis-strapped relay board" heuristic was removed: 0x1C is the
on-board mux U2, which always answers whenever the driver runs on I2C2, so that message would
misattribute a plain wiring fault. The sweep is eight zero-length writes — bounded and
non-blocking.

### Decision 2 — Opto inputs: polled snapshot with staleness, not per-call I2C

`BoardInputs` owns the PCA9557 @ 0x1A, polls the input register every `BOARD_INPUT_POLL_MS` (**25 ms**
— the brake overrun logic tolerates ~1 s of latency and the gear read already caches for 100 ms via
`TRANS_GEAR_READ_INTERVAL_MS`, so 25 ms is comfortably fast for both), and publishes an immutable
snapshot:

```
struct Snapshot {
  bool gearReverse, gearNeutral, gearLow, gearHigh;  // In1..In4, de-asserted sense normalized here
  bool brakeReleased;                                // In5
  bool valid;        // false once the snapshot is stale
  uint32_t ageMs;    // ms since the last successful read
};
```

Consumers call `getSnapshot()`; they never issue I2C themselves. This keeps
`TransmissionController::getPhysicalGear()` (which today can perform up to four re-reads in a retry
loop, `src/TransmissionController.cpp:234-242`) from turning into a burst of blocking I2C
transactions inside the control loop — its retry loop becomes "wait for a newer snapshot", i.e. the
existing 100 ms cache simply consumes successive 25 ms polls.

**Failure policy (explicit):**

| Condition | Behavior |
|-----------|----------|
| Read succeeds | Snapshot replaced, `valid = true`, `ageMs = 0`, consecutive-failure counter reset |
| Single read fails | Snapshot retained (last-known), `valid` unchanged, failure counter incremented, retried on the next poll — no logging storm (rate-limited) |
| `ageMs > BOARD_INPUT_STALE_MS` (**250 ms**, = 10 missed polls) | `valid = false`, fault flag raised, error logged once per fault episode |
| Recovery | Bus re-init attempted every `BOARD_INPUT_RECOVER_MS` (1000 ms) while faulted; on the first good read the fault clears and a recovery message is logged |

The *last-known-value* choice (rather than "fail to all-inactive") is deliberate: the two consumers
already have safe interpretations of their stale state, and holding the last value avoids a
gear-flap glitch on a single NACK. Consumer degradation is specified in the `vehicle-systems` delta:

- **Gear:** when `valid == false`, `getPhysicalGear()` returns `GEAR_UNKNOWN`, which already caps
  throttle at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (5%) and blocks confirmation of a gear change. No new
  interlock is needed — the existing unknown-gear path is the correct degraded mode.
- **Brake limit:** when `valid == false`, `isBrakeReleased()` returns `false` ("not confirmed
  released"). Retraction therefore relies on `BRAKE_SENSOR_OVERRUN_TIME` / `BRAKE_FULL_TRAVEL_TIME`
  to bound travel, exactly as it does today when the endstop is not yet reached. Returning `true`
  on a fault would let the retraction stop early and is rejected.

### Decision 3 — Relay writes are verified by read-back, with starter escalation

The PCA9557 output register is readable. Every `RelayController` write therefore does
write → read-back-and-compare → retry (up to `RELAY_WRITE_RETRIES`, 3, on the same `update()` pass,
which is bounded and short even at 100 kHz — a retry is a few one-byte transactions) → on persistent mismatch raise `relayFault` and log. The
whole 8-bit output port is written from a single shadow byte, so the six used relays and the two
spares are always consistent and there is no read-modify-write race.

**The relay board's PCA9557 routing is not 1:1** (`Relay_1=IO4, Relay_2=IO5, Relay_3=IO6,
Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`, from the `Relay.PrjPcb` netlist),
and two functions drive a paralleled *pair* of relays that switch together — the ignition/ECU line
(Relay_1 + Relay_2, harness X4 + X5) and the front-wheel lock (Relay_5 + Relay_6, harness X8 + X9).
Firmware therefore names each function by a whole-port mask (`RELAY_MASK_IGNITION` `0b00110000`,
`RELAY_MASK_STARTER` `0b01000000`, `RELAY_MASK_FRONT_LIGHT` `0b10000000`, `RELAY_MASK_WHEEL_LOCK`
`0b00001100`; spares Relay_7/Relay_8 = `0b00000011`, always 0) instead of per-relay bit indices.
A mask is set or cleared in one operation, so a pair can never be left half-energized, and the
"obvious" `Relay_N = bit N` shortcut — which would fire the starter on a front-light command — has
no place to hide.

**The starter (Relay_3, IO6) is treated as more dangerous than the others.** A stuck-on starter destroys
the starter motor and can move the vehicle. Its handling:

- `RELAY_MASK_STARTER` is never set unless `RelayController` is in `CRANKING`, and the existing
  `CRANKING_TIMEOUT` / RPM-based exit is unchanged.
- A verified read-back is **required** before the starter is considered engaged; if the read-back
  after engaging does not confirm, the crank is aborted and the port is rewritten with
  `RELAY_MASK_STARTER` clear.
- On any relay-write fault while a `RELAY_MASK_STARTER` bit is set, the driver immediately attempts an
  "all-relays-off" port write, repeats it every `update()` while the fault persists, latches the
  ignition state to `OFF`, and refuses new crank requests until the expander verifies clean again.
- Any expander failure at `begin()` leaves the driver un-initialized; `begin()` returns `false`,
  every setter becomes a no-op that logs, and the ignition state reports `OFF`. The system continues
  running degraded (steering/throttle/brake are independent of this bus) rather than halting.

*Alternative considered:* trusting writes without read-back (as the current `digitalWrite`
implementation implicitly does). Rejected — with a GPIO, a write cannot fail; over I2C it can, and
this is the starter.

*Not solved by firmware:* a physically welded relay contact is invisible to the output-register
read-back. That risk stays with the hardware (the relay board's contact rating) and is called out in
the bring-up checklist.

### Decision 4 — Boot sequence ordering

```
setup()
 1. pinMode(PIN_BOOST_EN, OUTPUT); digitalWrite(PIN_BOOST_EN, HIGH)   // rail up first
 2. Serial.begin()                    // USB-CDC; no wait-for-host
 3. nvs_flash_init(), Debug::begin()
 4. Wire.begin(PIN_STEER_SDA, PIN_STEER_SCL, I2C_BUS_FREQ_HZ)   // 100 kHz
    Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ)  // 100 kHz
 5. boardInputs.begin(Wire1)          // 0x1A (U10, I2C2)
 6. throttle / VESC / steering (AS5600 on Wire, already initialized) / transmission / brake
 7. relayController.begin(Wire1)      // 0x1F on X15/I2C2; all outputs LOW before anything else is commanded
 8. CAN, web portal
```

**Boost first.** GPIO46 has a 100K pulldown (R7) so the rail is off through reset and there is no
enable glitch; the rail feeds the steering VESC, so it must be up before `steeringVesc.begin()`
expects replies. A short settle is expected — the VESC controller is already designed to stay
"driver-down" until the first valid telemetry reply, so no blocking delay is added; the rail-voltage
sample simply reads low for the first samples and the low-rail warning is suppressed for
`BOOST_STARTUP_GRACE_MS` (1000 ms) after enable.

**Wire before any I2C consumer.** `SteeringController::begin(PIN_STEER_SDA, PIN_STEER_SCL)`
currently owns the `Wire.begin()` call; with a second device on the same bus, bus setup moves to
`main.cpp` (or the steering `begin()` becomes idempotent) so that the input expander does not depend
on steering initialization succeeding.

**Relays last among the I/O.** All eight outputs are driven LOW during `relayController.begin()`
before any vehicle logic can request ignition, preserving today's safe-state-at-boot behavior.

**USB-CDC console implications.** With `ARDUINO_USB_CDC_ON_BOOT=1`, `Serial` is the native USB CDC
device. Two consequences the bring-up must expect:

- Output produced before the host enumerates the CDC endpoint (~1-2 s after power-up) is lost. The
  early `[INIT]` prints in `setup()` are therefore *not* a reliable boot diagnostic any more.
  Firmware deliberately does **not** block on `while (!Serial)` — that would hang the vehicle
  whenever no laptop is attached.
- A panic/backtrace during early boot may not reach the host at all. If a boot-time hang has to be
  debugged, the fallback is the ROM bootloader UART on the Prog header (GPIO43/44) — which is
  exactly why the LAN section must stay unpopulated (warning §8.1).
- `monitor_speed` is irrelevant for CDC but harmless; `esp32_exception_decoder` still works.

### Decision 5 — 24V rail measurement and the web toggle

`PIN_ADC_24V` (GPIO9, ADC1) is sampled every `RAIL_SAMPLE_INTERVAL_MS` (500 ms) with a small
median/rolling average (8 samples) — ESP32-S3 ADC1 is noisy and the divider is high-impedance
(200K/24K). Conversion: `V24 = Vadc × 9.33`. Published as telemetry `rail_24v` (float, 1 decimal)
plus `rail_low` when the reading is below `RAIL_24V_LOW_THRESHOLD` (**21.0 V**) for longer than the
startup grace. The threshold is a warning only — no automatic shutdown, because dropping the rail
would kill steering.

The **web toggle is included** because it drops into the existing command path with no new
machinery (`WebPortal::WebCommand` → `VehicleController::processWebCommand`, identical in shape to
`set_light` / `set_wheel_lock`, `src/VehicleController.cpp:140-147`). It is **guarded**: disabling
the boost is refused unless ignition is `OFF`, and the refusal is reported back to the UI like the
other guarded commands. Rationale: the rail powers the steering VESC, so an operator-initiated
disable while the vehicle is live would remove steering authority — but the ability to power-cycle
the rail from the bench is worth having, and re-enable is always allowed.

## Risks / Trade-offs

- **Single bus in the path of the gear interlock and the starter.** → Explicit staleness + fault
  flags (Decisions 2, 3), degraded modes that match the existing safe behaviors, and telemetry
  visibility so a marginal bus is diagnosable rather than intermittent-and-silent.
- **Hard switch means no rollback to the DevKit.** → Mitigated by tagging the last DevKit-compatible
  commit before the constants rewrite; rollback is a git checkout, not a build flag.
- **Bring-up risks carried over from `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §8** — each is a checklist item
  in `tasks.md`:
  - §8.1 GPIO43/44 are shared copper between the Prog header and the CH9121T; the LAN section must
    stay unpopulated/silent.
  - §8.2 Strapping pins: GPIO0 (boot/LED), GPIO45 (`CFG_CH9121T`), GPIO46 (boost enable — verify no
    enable glitch through reset), GPIO3 (JTAG-select strap, used as ADC input only, which is safe).
  - §8.3 The X6 (VESC) and X9 (MAVLink) UART links run through BSS138 shifters with 10K pull-ups to
    5V; verified at 115200 only — **do not raise the baud rate**, keep cables short.
  - §8.4 X9 pin 1 (5V) must **not** be wired to Pixhawk TELEM 5V (both are supply outputs).
  - §8.5 Hall inputs are divided for 5V sensors (×0.65); a 12V push-pull sensor would exceed the GPIO
    absolute maximum.
  - §8.6 I2C2 level-shifter low-side rail is jumper-selected: verify **R16 populated (3V3), R18 not**.
  - §8.8 The relay board's PCA9557 straps must avoid 0x1A (U10) and 0x1C (U2); the as-shipped A2A1A0 = 111 → 0x1F is kept, and the board runs on X15/I2C2 (that header needed FB3/FB4-area rework on the first board).
- **GPIO1/2 are the AS5600 bus now.** Anything (including the pending speed-sensor change) that
  assumed those pins were free is invalidated — handled as an explicit doc-update task.
- **ADC1 and WiFi.** GPIO3 and GPIO9 are ADC1, which is unaffected by WiFi (unlike ADC2), so the AP
  can stay up while sampling the rail. Noted because it constrains any future move of these pins.

## Migration Plan

1. Tag / branch the last DevKit-compatible commit (rollback point).
2. Rewrite `Constants.h` and add the new drivers; the build must stay green at each step
   (`pio run` is a gate in `tasks.md`).
3. Flash and bench-verify subsystem by subsystem on the `Control_v0` board with the vehicle on
   stands: I2C scan → inputs → relays → boost/rail → servos/PWM → UARTs → CAN.
4. Update `GPIO_PINOUT_CUSTOM_BOARD_S3.md` if the as-built firmware diverges from it, and update the
   `add-hall-speed-sensor` change documents.
5. Rollback: `git checkout` the tagged commit and reflash the DevKit build; no data migration is
   involved (NVS namespaces — steering calibration, throttle calibration, gear defaults, transmission
   state, debug flags — are untouched by this change and survive the switch).

## Open Questions

- Should `Wire.begin()` move out of `SteeringController::begin()` into `main.cpp` (cleaner, two
  callers) or should the steering `begin()` become idempotent and keep owning it? Implementation
  detail; both satisfy the spec.
- Is 21.0 V the right low-rail threshold for the TL494 boost under Jetson + Starlink + camera load,
  or should it be derived from a measured loaded minimum during bring-up?
- Opto input polarity: PC817 collector-low vs the PCA9557 input register bit sense must be confirmed
  on the bench before the gear-active logic is finalized. The current gear code treats switches as
  active-low (`!digitalRead(...)`); the delta specifies *behavior* ("input asserted"), leaving the
  bit polarity to be pinned down by task 8.2.
- Should the low-rail warning also gate a MAVLink `STATUSTEXT` to the GCS, or is web telemetry
  enough for now?
