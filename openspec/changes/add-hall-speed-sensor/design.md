## Context
The firmware runs on an ESP32-S3-DevKitC-1 (N16R8) under the Arduino framework with a single
cooperative `loop()` (`src/main.cpp:167-208`). Every subsystem is a class with `begin()` + `update()`
polled with `millis()` rate limiting; there are no FreeRTOS tasks. There is currently **no pulse /
interrupt / PCNT / RMT input anywhere in the project** — this is a greenfield input pattern.

Speed today is a dead value: `CANController::VehicleData.vehicleSpeed` (`include/CANController.h:22`,
`uint8_t` km/h) is never populated because the OBD-II speed PID (0x0D) read is commented out
(`src/CANController.cpp:283-287`) and the pending `add-can-pid-probe` change deliberately leaves it
disabled (the ECU's PID map is untrusted). The three would-be consumers are therefore inert:

- The gear-change interlock at `src/TransmissionController.cpp:180-199` blocks non-NEUTRAL shifts
  when `vehicleData_.vehicleSpeed > TRANS_SPEED_INTERLOCK_THRESHOLD` (5 km/h, `Constants.h:332`), but
  since speed is always 0 it never blocks.
- Web telemetry emits `vehicle_speed` only inside `if (can_status == "connected")`
  (`src/WebPortal.cpp:604-606`), gating it behind CAN health it has nothing to do with.
- MAVLink (`MavlinkInterface`) sends only `EFI_STATUS` and explicitly notes speed is unavailable
  (`include/MavlinkInterface.h:55`, `src/MavlinkInterface.cpp:262`).

A physical 3-pin **12V hall-effect** speed sensor (VCC / GND / open-collector-or-push-pull signal, no
CAN data) is being fitted. This change reads it directly and makes speed a real, independent signal.

## Goals / Non-Goals
- Goals: read hall pulses on one GPIO with a hardware pulse counter; derive km/h from
  runtime-settable calibration (pulses/rev + wheel circumference) persisted in NVS; expose speed to
  web telemetry (decoupled from the CAN gate), to MAVLink (`VFR_HUD`), and to control logic (live
  gear interlock + a configurable max-speed throttle limiter); fail safe on sensor loss.
- Non-Goals: no FreeRTOS task; no odometer / trip-distance accumulation; no wheel-slip or
  multi-wheel fusion; no closed-loop cruise control; no new UI framework (only a telemetry field and
  a few calibration/limiter commands + readouts); no re-enabling of CAN PID 0x0D (speed comes solely
  from the hall sensor). Distinguishing "stopped" from "sensor disconnected" purely in firmware is a
  Non-Goal (see Risks) — it is fundamentally ambiguous for a passive pulse sensor.

## Decisions

### Decision: Count pulses with the ESP32-S3 PCNT peripheral, not `attachInterrupt`
The signal is a pulse train whose frequency encodes speed. Two options were considered:

- **PCNT hardware pulse counter (chosen).** The ESP32-S3 has a dedicated Pulse Counter (PCNT)
  peripheral. It counts edges in hardware, includes a configurable **glitch filter** (rejects the
  short spikes typical of an automotive hall signal on a long harness), and requires no ISR. The
  `update()` method samples the accumulated count on a fixed `SPEED_SAMPLE_INTERVAL_MS` window and
  computes frequency = Δcount / Δt — a perfect fit for the existing cooperative `begin()/update()`
  polling model, with essentially zero CPU cost per pulse and no jitter from software edge handling.
- **`attachInterrupt` + volatile counter (rejected).** Works, but every edge takes an ISR; a noisy
  or high-frequency signal (high speed × high pulses/rev) can flood the CPU and starve the 5 Hz
  telemetry / control loop, and there is no hardware glitch filtering (spurious edges inflate speed —
  a safety concern for an over-speed limiter and the interlock). It also introduces the project's
  first ISR/`volatile`/critical-section surface, which is more error-prone than a counter read.

PCNT is accessed via the ESP-IDF `driver/pulse_cnt.h` (`pcnt_unit_*`) API, available under Arduino
on the ESP32-S3. The unit counts **one edge only — falling or rising, either works; pick one and
note the opto inversion** (the 6N137 output is low while the LED conducts, so an active sensor pulse
appears as a GPIO8 low; edge *count* per pulse is identical either way). The glitch filter is set
from a `SPEED_GLITCH_FILTER_NS` constant. On counter overflow the high/low limit watch-points
accumulate into a 32-bit running total so a fast signal between samples cannot lose counts.

### Decision: Input on GPIO 8 (6N137 opto input, X2), not a Hall channel
Input pin: **GPIO 8** (`PIN_SPEED_SENSOR`) = the 6N137 opto-isolated input on connector X2 of the
`Control_v0` board (`GPIO_PINOUT_CUSTOM_BOARD_S3.md` §2/§3/§8.9). Signal path:
`X2.1 / X2.2 → 470Ω R22 → 6N137 LED → open-collector output + pull-up → GPIO8`.

The fitted sensor outputs a **12V** signal. The board's Hall channels (X3/X4/X5) are scaled for 5V
sensors (ferrite bead → 1.2K/2.2K divider ×0.65 → PESD3V3L1UA TVS + 100n) and 12V × 0.65 ≈ 7.8V
would exceed the GPIO absolute maximum, so a Hall channel would need a resistor rework to be usable.
The opto input needs no such rework: it is **current-driven and galvanically isolated**, so it
handles 12V with a single external series resistor and additionally breaks the ground loop to the
driveline harness — a real benefit for a long automotive cable run. GPIO8 routes to a PCNT input and
has no strapping, USB or JTAG caveats.

Consequences:
- **Hall_1 (GPIO14 channel A / GPIO21 channel B, X3) returns to SPARE**, as do Hall_2/Hall_3
  (GPIO12/13 and GPIO10/11) — all 5V-scaled channels available for future 5V sensors.
- **S-BUS as an RC-receiver input is gone for good.** GPIO8 was earmarked for it, but no UART was
  available to serve it; the pin is now the speed-sensor pulse input.
- There is **no fallback pin**: this change originally proposed GPIO 2 with GPIO 1 as the alternate,
  but on `Control_v0` both are I2C1 (the AS5600 steering sensor and the opto-input PCA9557).

The **inversion** is a documentation matter, not a functional one: pulse counting is edge-based, so
the firmware simply picks one edge and records that the opto inverts.

The remaining firmware-visible hardware requirement is **LED current limiting** (see Risks): at 12V
the stock 470Ω `R22` alone gives ≈22 mA, over the 6N137's 20 mA absolute maximum. An extra
**560Ω–1kΩ in series in the sensor cable** (≈7–10 mA) — or reworking `R22` to 1kΩ — is required.
Wiring depends on the sensor type:
- **push-pull 12V:** sensor signal → `X2.1` (series R in line), sensor GND → `X2.2`;
- **open-collector NPN:** +12V through the series R → `X2.1`, sensor output → `X2.2` (the sensor
  sinks the LED current).

Fitting the series resistor and wiring per sensor type is a manual user task (called out in
`tasks.md`).

### Decision: Speed lives in a new independent `SpeedSensor` source, NOT in `VehicleData.vehicleSpeed`
Two placements were considered for where the derived km/h enters the data model:

- **New independent `SpeedSensor` module owning `getSpeedKmh()` + `isValid()` (chosen).** The hall
  sensor is a wholly separate physical source from the CAN bus, with its own health and staleness.
  Coupling it to `VehicleData` would entangle two unrelated validity domains: `VehicleData.dataValid`
  means "CAN/MCP2515 is healthy", and a CAN outage would then wrongly blank a perfectly good speed
  reading (or vice-versa). Because CAN speed (PID 0x0D) is disabled anyway, there is no real CAN
  speed to reconcile with. Keeping `SpeedSensor` independent lets telemetry, MAVLink, the interlock,
  and the limiter each consume `speedSensor.getSpeedKmh()` / `isValid()` on the sensor's own terms,
  and is exactly why telemetry can be decoupled from the `can_status == "connected"` gate.
- **Write into `VehicleData.vehicleSpeed` (rejected).** Zero consumer rewiring (interlock/telemetry
  already read that field), but it conflates CAN validity with sensor validity, misrepresents a
  non-CAN value as CAN data, and would fight the `add-can-pid-probe` change's intent to keep CAN
  speed disabled. Rejected in favor of an honest, separately-validated source.

Consequence: the interlock (`TransmissionController`) and telemetry are rewired to read the
`SpeedSensor`, and `vehicleSpeed` is removed from the CAN-gated telemetry JSON block.

### Decision: Calibration model = two physical parameters (pulses/rev + circumference), not one scale
The conversion needs distance-per-pulse. Options:

- **Store `pulses_per_rev` and `wheel_circumference_mm` separately (chosen).** Both are independently
  measurable physical quantities: pulses/rev is a property of the sensor/target (count the teeth or
  bench-spin one wheel revolution and read the pulse count), circumference is a tape-measure of the
  fitted tyre. The runtime derives `distance_per_pulse = circumference_mm / pulses_per_rev`, and
  `speed_kmh = (Δpulses × distance_per_pulse) / Δt` with unit scaling. Storing the two physical
  values keeps calibration intuitive (change a tyre → update one number) and each is set by its own
  web command.
- **Store a single combined scale factor (pulses→km/h) (rejected as the stored form).** Fewer NVS
  keys, but it hides the physics, is easy to get wrong, and forces a full recompute-by-hand whenever
  either underlying quantity changes. The single scale is still computed internally as the derived
  value; it is just not the *stored* representation.

Persistence: NVS namespace `"speed"` via `Preferences`, keys `ppr` and `circ_mm`, defaults from
`Constants.h` (`SPEED_DEFAULT_PULSES_PER_REV`, `SPEED_DEFAULT_WHEEL_CIRC_MM`). Set at runtime via
`speed_cal_ppr` / `speed_cal_circ` web commands routed through
`WebPortal::WebCommand` → `VehicleController::processWebCommand`, mirroring `steer_cal_*` (accepted
regardless of active input source, same diagnostic privilege as other calibration commands), and
persisted immediately so calibration survives reboots.

### Decision: Staleness / "silent sensor" behavior — 0 km/h reading plus a separate validity flag
A passive hall sensor emits **no pulses at standstill**, which is indistinguishable at the sensor
from a disconnected wire. The design separates two facts:

- **Speed value:** if no edges arrive within `SPEED_STALE_TIMEOUT_MS` (chosen from the lowest
  measurable speed — the period of one pulse at ~1 km/h), the reported speed decays to **0 km/h**.
  This is the physically correct reading for a stopped vehicle and is what the interlock wants at
  rest.
- **Validity flag (`isValid()`):** a separate health signal. It is `false` until the sensor has
  produced at least one plausible pulse since boot, and it is set *suspicious/false* if pulses cease
  faster than physically plausible after the vehicle was recently moving above the interlock
  threshold (a decel-rate plausibility check) — the fingerprint of a mid-motion wire fault, as
  opposed to a normal stop.

This split lets consumers choose their own fail-safe direction (below) rather than baking one policy
into the sensor.

### Decision: Interlock fail-safe mirrors the existing CAN-timeout philosophy
The live interlock reads `speedSensor.getSpeedKmh()` when the sensor is valid, and blocks non-NEUTRAL
shifts above `TRANS_SPEED_INTERLOCK_THRESHOLD`. Fail-safe cases:

- **Sensor valid, moving:** block the shift (correct).
- **Sensor valid, stopped (0 km/h):** allow the shift (correct).
- **Sensor never initialized / flagged suspicious:** fall back to the **existing CAN-timeout
  behavior** already in `canChangeGear()` — i.e. allow the change (fail-open), logging the reason.
  Rationale: this preserves the current, reviewed transmission behavior (a stuck-blocked gearbox is
  itself a hazard), and the ambiguity of a passive pulse sensor means firmware cannot *prove* the
  vehicle is moving when no pulses arrive. The residual risk (disconnected sensor while genuinely
  moving → reads 0 → shift permitted) is documented in Risks with the decel-plausibility heuristic as
  the partial mitigation and a wired fault line as future work.

### Decision: Max-speed throttle limiter — simplest safe design, disabled by default
A new control-logic limiter in `VehicleController`, applied after command arbitration, before the
throttle command reaches the actuator:

- Config in NVS `"speed"`: `limit_enable` (bool, default **false**) and `limit_max_kmh`
  (default `SPEED_LIMIT_MAX_KMH_DEFAULT`, a safe/high value). Set via `speed_limit_enable` /
  `speed_limit_set` web commands.
- Behavior when enabled and speed is **valid**: while `speed > limit_max_kmh`, throttle authority is
  clamped to `SPEED_LIMIT_THROTTLE_CAP_PCT` (a reduced ceiling, not a hard cut — a hard cut mid-turn
  is itself unsafe); below the limit, throttle passes through unchanged. This is a plain ceiling, not
  a PID, per the minimalism guardrail.
- **Failure mode on sensor loss** (speed invalid/stale): the limiter **does not clamp** (fails open),
  and logs a rate-limited warning. Justification: the limiter only ever *acts* above the max speed; a
  lost sensor reads 0 km/h, which is below any limit, so no phantom clamp is applied to bad data —
  clamping throttle on a false reading mid-maneuver is the more dangerous failure. Over-speed
  protection explicitly assumes a working sensor; this trade-off is recorded as an Open Question in
  case a future revision wants a fail-closed variant.
- Interaction: the gear-change throttle-boost PID (`TransmissionController` / `VehicleController`)
  already overrides throttle during shifts; the limiter applies to normal driver/MAVLink/web throttle
  only and does not fight the boost PID (limiter runs on the arbitrated command path, boost owns
  throttle while a shift is active).

### Decision: MAVLink speed via `VFR_HUD`
Speed is reported to the GCS using the standard `VFR_HUD` message's `groundspeed` field (m/s), sent
on the existing engine-telemetry schedule alongside `EFI_STATUS`. `VFR_HUD.groundspeed` is the
conventional, graphable speed field in Mission Planner for a `MAV_TYPE_GROUND_ROVER`, so no custom
`NAMED_VALUE_FLOAT` is needed. When the sensor reading is invalid/stale the field is sent as 0 (or
`VFR_HUD` is suppressed on that tick); `EFI_STATUS` continues to omit speed. The prior "speed is not
reported" note in `MavlinkInterface` is superseded.

## Risks / Trade-offs
- **Stopped vs disconnected is ambiguous for a passive pulse sensor.** No pulses = 0 km/h. A wire
  fault while moving reads 0 and could permit a gear shift or release the limiter. → Mitigations: the
  decel-rate plausibility heuristic flags implausible pulse loss as suspicious; the interlock then
  falls back to the reviewed CAN-timeout policy; a dedicated hardware sensor-fault/continuity line is
  logged as future work. Accepted because it matches the existing fail-safe philosophy and firmware
  cannot resolve the ambiguity alone.
- **Miscalibration produces wrong speed** (unknown pulses/rev at fit time). → Bench verification with
  a drill/known wheel-spin is a required task; calibration is runtime-settable so it is corrected
  without a reflash.
- **6N137 LED over-current at 12V.** The stock 470Ω `R22` alone passes ≈22 mA, slightly above the
  6N137's **20 mA absolute maximum** — the optocoupler would run out of spec and age fast. →
  Explicit spec requirement + a clearly-marked manual hardware task: add **560Ω–1kΩ in series in
  the sensor cable** (≈7–10 mA, in the 6–15 mA sweet spot) or rework `R22` to 1kΩ, before first
  power-up with the sensor connected. Too *little* current is also a failure mode (below ~5 mA the
  6N137 output may not switch reliably), which is why the target band is stated.
- **Opto output is inverted** (LED conducting → GPIO8 low). → Harmless for edge counting; recorded
  in the PCNT configuration comment so a future reader does not "fix" the polarity.
- **Signal noise on a long automotive harness** could inflate counts. → PCNT hardware glitch filter
  (`SPEED_GLITCH_FILTER_NS`); rejected `attachInterrupt` partly for lacking this. The opto input's
  current-mode drive and galvanic isolation also help here compared with a voltage-divider input.
- **Opto propagation/rise time bounds the maximum pulse rate.** The 6N137 is a fast (10 Mbit-class)
  part, so it is not a practical limit for a driveline pulse train, but the pull-up value sets the
  rising edge. → Bench-verify pulse tracking at the highest expected speed (task 9.1).
- **Telemetry contract change:** `vehicle_speed` leaves the CAN-connected JSON block and is now
  sensor-sourced. → Called out as BREAKING in the proposal; the web UI reads it independent of
  `can_status`, and `speed_valid` communicates sensor health.

## Migration Plan
Additive at the code level (new module + new NVS namespace + new web commands), with one behavioral
change to the telemetry contract (`vehicle_speed` decoupled from the CAN gate) and one to the
interlock's speed source (CAN field → hall sensor). No stored data is removed; the new NVS `"speed"`
namespace starts from `Constants.h` defaults on first boot. The speed limiter ships **disabled**, so
default drive behavior is unchanged until an operator enables it. Rollback: leaving the sensor
unwired and uncalibrated yields `isValid() == false` and 0 km/h, restoring today's effective
behavior (interlock never blocks, limiter never clamps).

## Open Questions
- Exact `SPEED_DEFAULT_PULSES_PER_REV` and `SPEED_DEFAULT_WHEEL_CIRC_MM` values — placeholder defaults
  ship in `Constants.h`; real values are captured during bench calibration and set at runtime.
- Whether the max-speed limiter should ever be **fail-closed** (clamp on sensor loss) for a specific
  deployment; the chosen fail-open default is documented so this can be revisited with field data.
- Whether a dedicated hardware sensor-fault line (to positively distinguish "stopped" from
  "disconnected") is worth adding in a later change to harden the interlock.
