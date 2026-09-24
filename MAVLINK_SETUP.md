# MAVLink Vehicle Interface — Setup and Wire Contract

The ESP32 talks to the Pixhawk 2.4.8 over a single MAVLink 2 serial link (replacing the former
SBUS-Out connection). This file is both the **setup guide** (wiring, ArduPilot parameters, servo
mapping) and the **wire contract** (every outbound message field by field, with unit and validity).

## Identity and transport

| | |
|---|---|
| System id | `1` (`MAVLINK_SYSTEM_ID`, `include/Constants.h:251`) — **the same system as the autopilot** |
| Component id | `25` = `MAV_COMP_ID_USER1` (`MAVLINK_COMPONENT_ID`, `include/Constants.h:252`) |
| Dialect | `ardupilotmega` (superset of `common`) |
| Protocol | MAVLink **2**, forced on the outbound channel (`MAVLINK_STATUS_FLAG_OUT_MAVLINK1` cleared) |
| Link | UART1, 8N1, **115200** (`MAVLINK_BAUD_RATE`), channel `MAVLINK_COMM_0` |

ArduPilot forwards **target-less** messages (`HEARTBEAT`, `EFI_STATUS`, `VFR_HUD`,
`NAMED_VALUE_FLOAT`, `STATUSTEXT`, and
`VISION_POSITION_DELTA`, which it also consumes itself) from TELEM2 to every other MAVLink port, so
they reach the GCS unchanged; `COMMAND_ACK` **is** addressed — to the requester — and rides the route
ArduPilot learned from this component's own outbound traffic. Mission Planner will **not** populate
its native `efi_*` or HUD fields from component 25 (it maps those from the autopilot,
component 1 only), so the QuadBike MP plugin reads the **raw packets** via a subscription keyed on
message id + sysid/compid. Everything below is visible to that subscription and to MP's MAVLink
Inspector, and to nothing else in MP. Verified on BlueOS + MP 1.3.83.

> ⚠ Because sysid is shared with the autopilot, a consumer **must** filter on `compid == 25`.
> `VFR_HUD` and `HEARTBEAT` exist twice on this link — once from the Pixhawk (comp 1), once from
> this board (comp 25) — and they mean different things.

## Wiring

| Pixhawk TELEM2 | ESP32 (UART1) | `Control_v0` header |
|----------------|---------------|---------------------|
| TX             | GPIO 18 (`PIN_MAVLINK_RX`) | X9 pin 3 |
| RX             | GPIO 17 (`PIN_MAVLINK_TX`) | X9 pin 2 |
| GND            | GND | X9 pin 4 |

Non-inverted, no inverter circuit (unlike SBUS).

> ⚠ The link runs through the **BSS138 level shifter** on header X9 (Pixhawk 5 V ↔ ESP32 3.3 V).
> That shifter is passive and rate-limited: **115200 is the only supported baud** — do not raise
> `SERIAL2_BAUD`.
>
> ⚠ GPIO 8 is **not** free — it is `PIN_SPEED_SENSOR` (hall wheel-speed input, header X2). The
> pre-`Control_v0` GPIO 8 / GPIO 15 MAVLink pair no longer exists.

## ArduPilot parameters

| Parameter | GPS-less (bench-proven) | With a GPS fitted | Meaning |
|-----------|-------------------------|-------------------|---------|
| `SERIAL2_PROTOCOL` | `2` (MAVLink2) | same | TELEM2 speaks MAVLink 2 |
| `SERIAL2_BAUD` | `115` (115200) | same | Match the ESP32 link baud |
| `SPEED_MAX` | m/s, `0` = no limit | same | Read-only by the ESP32; the **only** source of the throttle limiter's ceiling — see below |
| `MANUAL_OPTIONS` | **`0`** — REQUIRED | **`0`** — REQUIRED | Turns OFF ArduPilot's own speed-scaled steering. The ESP32 does that scaling now; leaving bit 0 set scales the steering **twice** — see below |
| `MOT_SPD_SCA_BASE` | m/s, `0` = no scaling | same | Read-only by the ESP32; the **fallback** source of the steering speed-scaling base (the ESP32's own `steer_sca_base` wins) — see below |
| `VISO_TYPE` | `1` (MAVLink) | `1` | Enable the visual-odometry backend. **Verify the parameter exists** — see the fmuv2 caveat |
| `EK3_SRC1_VELXY` | `6` (ExternalNav) | `6` | Fuse horizontal velocity from body odometry |
| `EK3_SRC1_POSXY` | **`0`** (None) | **`3`** (GPS) | With no GPS there is no position source to wait for |
| `GPS1_TYPE` | **`0`** (None) | `2` (or as the receiver needs) | Named `GPS_TYPE` before ArduPilot 4.7. `0` on the bench so an absent GPS cannot hold the filter waiting |
| `VISO_DELAY_MS` | `10` (default) → tune toward ~150 | same | Measurement lag. The hall speed is a window average: ~100 ms at speed, stretching toward ~1 s at a crawl. Tune against `XKF3 IVN/IVE` innovations and record the final value here |
| `VISO_POS_X` / `_Y` / `_Z` | rear-hub lever arm (m) | same | Measuring wheel relative to the IMU, body frame |
| `VISO_ORIENT` | `0` (`ROTATION_NONE`) | same | The firmware already emits body-frame-forward; `VISO_ORIENT` **is** applied on this path, so a non-zero value would silently rotate the measurement |

**Not applicable to the delta path**, verified by reading
`AP_VisualOdom_Backend::handle_vision_position_delta_msg` in ArduPilot-4.7: `VISO_QUAL_MIN` (the
delta handler does no quality gating), `VISO_VEL_M_NSE` (read only by
`handle_vision_speed_estimate`) and `VISO_SCALE` (only by `handle_pose_estimate`).

No stream-rate parameter is needed: the ESP32 requests its one inbound stream itself (see
"Requests toward the autopilot"). If the autopilot ignores both request forms, set `SRx_RC_CHAN`.

### `MANUAL_OPTIONS = 0` — required, and why

**`MANUAL_OPTIONS` bit 0 enables ArduPilot's speed-scaled steering in MANUAL**
(`AP_MotorsUGV::output_regular()`: `if (is_positive(base) && fabsf(v) > base) steering *= base /
fabsf(v);`, with `base = MOT_SPD_SCA_BASE`). The formula is fine. **Its speed is not.** `v` comes
from `AR_AttitudeControl::get_forward_speed()`, which asks the AHRS for a velocity estimate and,
when EKF3 has none, **silently returns the raw GPS ground speed instead**. There is no ArduPilot
parameter that disables that fallback while a GPS is attached.

On this vehicle EKF3's horizontal velocity source **is** the ESP32 (`VISO_TYPE = 1`,
`EK3_SRC1_VELXY = 6`, `VISION_POSITION_DELTA`), and the ESP32 deliberately goes **silent** rather
than send zeros when its hall sensor is unhealthy. So the exact moment the speed source fails is the
moment ArduPilot starts steering by GPS. **Observed 2026-09-24: a parked vehicle, GPS reporting
56 m/s, steering scaled to 0.018 — effectively no steering at all.** The failure is silent, it
happens precisely when the speed source is already unhealthy, and its result is the opposite of
fail-safe.

The scaling therefore moved to the ESP32, which measures the speed directly and knows whether to
believe it (`SpeedSensor::isValid()`). **Clear `MANUAL_OPTIONS` to `0` as a commissioning step.** A
vehicle that gets this firmware with bit 0 still set is scaled **twice** — once by the autopilot from
the untrustworthy EKF/GPS speed this exists to escape, and once by the ESP32.

> `VISO_TYPE = 1` stays exactly as it is. The wheel-odometry path is untouched by this change and
> must not be "cleaned up" along with it — the two are unrelated.

**Behaviour note.** Clearing `MANUAL_OPTIONS` also removes ArduPilot's *ground-speed-based* steering
reversal in reverse; with the bit clear that reversal keys off **throttle sign** instead. This
vehicle has a mechanical gearbox and never commands negative throttle, so the change is inert here.
No firmware behaviour depends on it.

### Steering speed scaling on the ESP32

`scale = min(1, base / speed)` — ArduPilot's own formula, unchanged, applied to the **deviation from
centre** of autopilot steering commands. It applies **in MANUAL only** (`custom_mode == 0` in the
autopilot's `HEARTBEAT`); in every other mode the autopilot computes steering from speed itself and a
second, invisible reduction would be double limiting. An **unknown or stale** mode is treated as
MANUAL — the one place where an unavailable input does not disable the feature, because MANUAL is the
mode this vehicle is driven in.

The base has two sources, in a fixed one-directional priority:

| Priority | Source | Notes |
|---|---|---|
| 1 | ESP32 NVS `steer_sca_base` (namespace `"steering"`, m/s) | Set from the web portal, no reflash. `0` or absent = "not set" |
| 2 | Autopilot `MOT_SPD_SCA_BASE` (m/s) | Read-only subscription, same mechanism as `SPEED_MAX`; unavailable when never received, `0`, rejected, stale or the link is down |
| — | neither | **No scaling at all**, plus a rate-limited warning. There is deliberately no compile-time default |

**Any fault disables the scaling — scale = 1, full steering authority.** No base, an invalid speed
reading (`SpeedSensor::isValid()` false for any reason), or an autopilot mode that is known and is
not MANUAL. There is deliberately **no hold, no last-known-good speed, no substitute speed and no
minimum-scale floor**: speed scaling is a comfort and assist feature, and every one of those would be
a way for a sensor fault to leave the driver unable to turn. The applied scale is slew-limited to
`STEER_SCALE_SLEW_PER_S` (2.0 /s, both directions) — applied to the **scale**, never to the steering
command, so the driver's own movements are never slowed.

The **web `set_steering` path is never scaled**: it is a bench and maintenance control with no road
speed behind it.

`set_test_speed <m/s>` (web command) substitutes a speed for **the scaling calculation only**, so the
feature can be exercised on a stationary, disarmed bench. It is RAM-only, clears itself after
`STEER_SCALE_TEST_SPEED_MS` (60 s), is marked `TEST` in the log and in the portal, and reaches
neither `VISION_POSITION_DELTA`, `VFR_HUD`, the `SPEED_MAX` limiter, the transmission interlock nor
the odometer.

The applied scale is reported as the `STEER_SCA` `NAMED_VALUE_FLOAT` (below), as `steer_scale` in the
web telemetry JSON with its base and source, and as a `[STEER] Speed scale: …` line on change.

## Servo function mapping

The ESP32 reads command channels from `SERVO_OUTPUT_RAW.servoN_raw`. ArduPilot's `SERVOn_FUNCTION`
outputs must align with these indices (see `ServoChannelConfig` in `include/Constants.h`):

| Servo output | Function | Value range (µs) |
|--------------|----------|------------------|
| `servo1_raw` | Steering | 1000–2000 (1500 = center) |
| `servo2_raw` | Throttle / Brake (combined) | >1500 = throttle, <1500 = brake |
| `servo3_raw` | Transmission | R <1200, N 1201–1520, L >1520 |
| `servo4_raw` | Ignition | OFF <1200, ACC 1201–1520, IGNITION >1520 |
| `servo6_raw` | Front light | >1520 = ON |
| `servo7_raw` | Front-wheel lock | >1520 = LOCKED |

(Channel 5 is intentionally unused.)

## Vehicle state reported back to the autopilot

Ten outbound message types: six periodic, one reactive acknowledgement, three requests toward
the autopilot. Scheduling is internal to `MavlinkInterface::report()`, called every loop iteration from
`src/main.cpp`; the `StateReport` snapshot it consumes is declared in `include/MavlinkInterface.h`.

| Message | Id | Rate | Purpose | Gating |
|---------|----|------|---------|--------|
| `HEARTBEAT` | 0 | 1 Hz (`MAVLINK_HEARTBEAT_TX_MS` = 1000 ms) | Component liveness + fail-safe state | none |
| `EFI_STATUS` | 225 | 5 Hz (`MAVLINK_REPORT_TX_MS` = 200 ms) | All engine/vehicle telemetry in one message | none — fields go NaN instead |
| `VFR_HUD` | 74 | 5 Hz (same tick) | Hall ground speed + throttle for a HUD | none |
| `NAMED_VALUE_FLOAT` | 251 | 5 Hz (same tick) ×3 + 1 Hz (`MAVLINK_STEER_SLOW_TX_MS` = 1000 ms) ×3 | The six steering values: `STEER_POS`, `STEER_A`, `VESC_V`, `VESC_TEMP`, `VESC_OK`, `STEER_SCA` | none — values go NaN instead (`VESC_OK` and `STEER_SCA` never do) |
| `VISION_POSITION_DELTA` | 11011 | ≤ 5 Hz (same tick) | Wheel odometry as an EKF3-fusable body-frame delta | **heavily gated** — silence over zeros |
| `STATUSTEXT` | 253 | on change, ≥ 250 ms apart | Gear / ignition / fail-safe transitions | only on an actual change |
| `COMMAND_ACK` | 77 | reactive | Answer to a `COMMAND_LONG` addressed exactly to 1/25 | broadcasts get **no** ACK |
| `REQUEST_DATA_STREAM` | 66 | ≤ 1 per 3 s | Ask for `SERVO_OUTPUT_RAW` | while the command rate is below 10 Hz |
| `COMMAND_LONG` (`SET_MESSAGE_INTERVAL`) | 76 | ≤ 1 per 3 s | Same request, modern form | same gate, sent together |
| `PARAM_REQUEST_READ` | 20 | 0.2 Hz each | Poll `SPEED_MAX` and `MOT_SPD_SCA_BASE` (one request per loop iteration, never both at once) | link up + autopilot learned |

Engine data comes from the CAN bus (`CANController::VehicleData`); ground speed, odometer and trip
come from the hall wheel-speed sensor (GPIO 8 / X2), independently of CAN health. Steering-VESC
electrical data comes from the VESC's own UART, and the measured steering position from the AS5600
on the steering shaft — three more sources, each with its own validity gate.

Value-to-name summary for the six steering `NAMED_VALUE_FLOAT` messages:

| Value | `name` | Rate | Source | Comp |
|-------|--------|------|--------|------|
| Steering position, **MEASURED** (%) | `STEER_POS` | 5 Hz | AS5600 | 25 |
| Steering motor current (A) | `STEER_A` | 5 Hz | VESC UART | 25 |
| VESC input voltage, 24 V rail (V) | `VESC_V` | 5 Hz | VESC UART | 25 |
| VESC FET temperature (°C) | `VESC_TEMP` | 1 Hz | VESC UART | 25 |
| VESC link health (`1.0` / `0.0`) | `VESC_OK` | 1 Hz | VESC UART | 25 |
| Applied steering speed-scale (`0`…`1`, `1` = not scaling) | `STEER_SCA` | 1 Hz | vehicle layer | 25 |

### `HEARTBEAT` (0) — 1 Hz

| Field | MAVLink meaning | What we send | Validity | Source |
|-------|-----------------|--------------|----------|--------|
| `type` | Vehicle/component type | `MAV_TYPE_GROUND_ROVER` (10) | always | constant |
| `autopilot` | Autopilot class | `MAV_AUTOPILOT_INVALID` (8) — correct for a component that is not a flight controller | always | constant |
| `base_mode` | Mode bitmap | `0` — no modes, never armed | always | constant |
| `custom_mode` | Autopilot-specific flags | `0` | always | constant |
| `system_status` | `MAV_STATE` | `MAV_STATE_CRITICAL` (5) in fail-safe, else `MAV_STATE_ACTIVE` (4) | always | `state.failsafe` |
| `mavlink_version` | Protocol version | filled by the library | always | library |

`state.failsafe` is `getInputSource() == InputSource::FAILSAFE` — neither MAVLink nor web control is
driving. This heartbeat is the ESP32's own liveness, **not** the autopilot's, whose heartbeat comes
the other way and drives `isLinkUp()`.

### `EFI_STATUS` (225) — 5 Hz

Every value sits in the field that **names** it, so the message is self-describing. Exactly **five**
fields are repurposed (⚑), and each passes a *permanent-absence* test: the quantity the field names
can never be produced by this vehicle's ECU. **Why one `EFI_STATUS` and not per-value
`NAMED_VALUE_FLOAT`:** all `NAMED_VALUE_FLOAT` messages share one message id and differ only by a
name string, so any name-agnostic store (MP's Inspector, the mavlink2rest cache, unmapped
`customField`s) keeps only the last one received. Distinct fields of one message cannot collide.
That reasoning governs the **ECU/EFI values** here; the six steering named floats are the
documented exception, and the reason they are one is set out in their own section below.

#### All 19 fields, in wire order

| # | Field | MAVLink meaning / unit | What we send | Validity | Source |
|---|-------|------------------------|--------------|----------|--------|
| 1 | `ecu_index` | ECU index | `0.0` | always | literal |
| 2 | `rpm` | RPM | **Engine RPM** (PID `0x0C`) | NaN if `!canValid` | `state.engineRpm` |
| 3 | ⚑ `fuel_consumed` | [cm³] Fuel consumed | **ASSUMED (commanded) gear** — repurposed: fuel PID `0x2F` absent from this ECU | **always valid**, never NaN | `gearVal` |
| 4 | ⚑ `fuel_flow` | [cm³/min] Fuel flow | **PHYSICAL (opto-sensed) gear** — same absent-PID justification | **NaN when the gear reads UNKNOWN** — sensor-driven, *not* CAN-driven | `physGearVal` |
| 5 | `engine_load` | [%] Engine load | **ECU calculated load** (PID `0x04`) | NaN if `!canValid` | `state.engineLoad` |
| 6 | `throttle_position` | [%] Throttle position | **MEASURED throttle** (PID `0x11`) | NaN if `!canValid` | `state.throttlePosition` |
| 7 | ⚑ `spark_dwell_time` | [ms] Spark dwell | **ENGINE HOURS, total, h** — repurposed: spark dwell has no standard OBD-II Mode 01 PID, so this ECU can never surface it | **always valid**, never NaN | `state.engineHours` |
| 8 | ⚑ `barometric_pressure` | [kPa] Barometric pressure | **ODOMETER, total, km** — repurposed: this ECU has no ambient-pressure sensor | **always valid**, never NaN | `state.odoKm` |
| 9 | `intake_manifold_pressure` | [kPa] MAP | **Manifold absolute pressure** (PID `0x0B`) | NaN if `!canValid` | `state.mapKpa` |
| 10 | `intake_manifold_temperature` | [°C] | **Intake air temperature** (PID `0x0F`) | NaN if `!canValid` | `state.intakeTemp` |
| 11 | `cylinder_head_temperature` | [°C] | **Coolant temperature** (PID `0x05`) | NaN if `!canValid` | `state.coolantTemp` |
| 12 | `ignition_timing` | [deg] Crank angle | `0.0` — **unused, reserved** | — | literal |
| 13 | ⚑ `injection_time` | [ms] | **TRIP ENGINE HOURS, resettable, h** — repurposed: injection time has no direct OBD-II Mode 01 PID (it is only derivable from the fuel trims), so this ECU can never surface it directly | **always valid**, never NaN; **0 is a genuine zero** | `state.engineTripHours` |
| 14 | `exhaust_gas_temperature` | [°C] | `0.0` — **unused, reserved** | — | literal |
| 15 | `throttle_out` | [%] Output throttle | **COMMANDED (arbitrated) throttle** | **always valid**, never NaN | `state.throttleCmdPct` |
| 16 | ⚑ `pt_compensation` | Pressure/temp compensation | **Digital-output bitmask** — repurposed: this ECU publishes no compensation factor | **always valid** (relay ground truth) | `state.digitalFlags` |
| 17 | `health` | EFI health (`uint8_t`) | `1` = present. **Constant — not a CAN-health flag** | always `1` | literal |
| 18 | `ignition_voltage` | [V] EFI supply voltage | **Control-module supply voltage** (PID `0x42`) | NaN if `!canValid` | `state.moduleVoltageMv / 1000` |
| 19 | ⚑ `fuel_pressure` | [kPa] Fuel pressure | **TRIP distance, resettable, km** — repurposed: fuel PIDs absent | **always valid**, never NaN; **0 is a genuine zero** | `state.tripKm` |

`EFI_STATUS` has exactly **19 fields** — the 18 floats plus the `uint8_t health` — and all 19 are
listed above in struct/wire order, which is *not* the `_pack()` argument order (`health` is the first
argument but the 17th field). The pack site writes one argument per line precisely because 18
same-typed positional floats would otherwise let a mis-ordered argument compile cleanly and fail only
on the bench.

**The two reserved fields** (`ignition_timing`, `exhaust_gas_temperature`) name quantities this ECU
**could plausibly expose later** over OBD-II — ignition timing is PID `0x0E`, exhaust gas
temperature is PID `0x78`, both standard Mode 01 — so they are deliberately not repurposed. A
consumer must treat `0.0` in these two as "not reported".

They used to be four. The test is whether the named quantity is *permanently* unavailable on this
bus, and the two that left the set are the two with **no direct Mode 01 PID**:

- **`spark_dwell_time`** — an internal coil-charging interval that OBD-II never standardised a way
  to ask for. There is no PID to enable and no plausible firmware change on either side that would
  make this ECU able to report it. It now carries the **total** engine hour meter.
- **`injection_time`** — likewise has no PID of its own; it is only *derivable*, from the fuel-trim
  PIDs (`0x06`/`0x07`) plus engine load, so it could never arrive on this bus as a field the way
  timing and EGT could. It now carries the **trip** engine hours.

The 2026-08-14 bench probe confirmed PIDs `0x2F` (fuel level) and `0x5C` (oil temperature) are
**absent** from this ECU's supported-PID bitmaps — that is what makes the fuel pair safe to
repurpose permanently. Generic tools (MP's Inspector, mavlink2rest, log analysers) label fields by
their MAVLink names and show the two gear values as fuel quantities in cm³: inherent to the
repurposing and harmless, since the values are visibly implausible as fuel. **Oil temperature** is
therefore not reported; **vehicle speed is**, but from the hall sensor, not the ECU — the OBD-II
speed PID `0x0D` is verified dead on this vehicle (always 0, even in motion).

#### Gear: assumed vs physical

Both gear fields use the **physical gear-sequence position** (`MavlinkInterface::encodeGear()`,
which switches on the first character of the gear name):

| Gear | Value | Assumed (`fuel_consumed`) moving toward it | Physical (`fuel_flow`) while moving |
|------|-------|--------------------------------------------|--------------------------------------|
| REVERSE | `-1.0` | (from N) −0.5 | NaN |
| NEUTRAL | `0.0` | (from R/H) −0.5 / 0.5 | NaN |
| HIGH | `1.0` | (from N/L) 0.5 / 1.5 | NaN |
| LOW | `2.0` | (from H) 1.5 | NaN |

- **`fuel_consumed` = assumed gear = intent** — the controller's commanded, time-based gear. While
  the servo steps between two gears (`state.gearMoving`) the value is their midpoint
  `(encodeGear(gearFrom) + encodeGear(gearTo)) * 0.5`, so R→L reads
  `-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0`. An unrecognised or null name falls back to `0.0`
  (NEUTRAL). **Always valid**, independent of both CAN and gear-switch health.
- **`fuel_flow` = physical gear = measurement** — read from the transmission's opto switches
  (`TransmissionController::getPhysicalGear()`), the same source the fail-safe and the
  `VISION_POSITION_DELTA` direction sign trust. **Never interpolated**: an integer or `NaN`.
  `encodeGear(state.gearPhysical, NAN)` passes `NAN` deliberately — "cannot tell" must never be
  reported as NEUTRAL.

> **`fuel_flow = NaN` during a shift is expected, not a fault**, and so is a brief disagreement
> between the two values. Only a **persistent** one matters: once `fuel_consumed` settles on an
> integer, `fuel_flow` should re-acquire the same integer within the settle window. A `fuel_flow`
> that stays NaN, or settles on a *different* gear, means the shift did not physically complete (or
> the gear-switch input is faulted). The firmware does not distinguish these on the wire — a
> consumer has both values and a clock.

#### `pt_compensation` bitmask

`EFI_DIGITAL_FLAG_*` in `include/Constants.h:288-289`, assembled in `src/main.cpp`, cast to float:

| Bit | Mask | Meaning |
|-----|------|---------|
| 0 | `0x01` | `EFI_DIGITAL_FLAG_WHEEL_LOCK` — front-wheel lock engaged |
| 1 | `0x02` | `EFI_DIGITAL_FLAG_FRONT_LIGHT` — front light on |
| 2–7 | — | unassigned, always 0 |

A `float` holding a small integer, so `0`, `1`, `2`, `3` are the only values on the wire today.

#### Odometer and trip distance

Both counters come from the hall wheel-speed sensor (GPIO 8 / X2), accumulated on the vehicle in
exact `uint64_t` **millimetres** from counted pulses only and persisted in NVS (so they survive a
power cycle). They are reported in **kilometres** and are **always valid**, never `NaN`:

| Counter | Field | Resettable |
|---------|-------|------------|
| **ODO** — total, vehicle lifetime | `barometric_pressure` | **no** — no command, parameter, magic value or web control zeroes or decreases it |
| **TRIP** — accumulates across ignition cycles | `fuel_pressure` | yes, via `MAV_CMD_USER_1` (see "Trip reset") |

> **`fuel_pressure = 0` is a genuine zero trip distance**, even though the MAVLink definition
> assigns zero the meaning "unknown". The sentinel it suggests (`0.0001`) is deliberately **not**
> substituted: reporting a non-zero distance right after a trip reset is the worse lie. Same for
> `ignition_voltage`, which carries the identical note — the measured voltage goes out as-is and
> **NaN**, not `0.0001`, means unknown. The consumer is this repository's own plugin, which decodes
> by field position.

A trip reset is observable on the wire as `fuel_pressure` **and** `injection_time` (the trip hours,
see below) falling to zero together, while `barometric_pressure` and `spark_dwell_time` are
unchanged. The odometer never decreases between two messages.

#### Engine hours

The engine hour meter ("мотогодини") is **two** counters — a total and a resettable trip, exactly as
the distance pair is — each carried in **hours** as a float. They come from a different source than
the distance counters (the CAN bus, not the hall sensor) and live in their own NVS namespace
(`"engine"`, keys `"hours_s"` and `"trip_s"`, exact `uint64` seconds each).

| Counter | Field | Resettable |
|---------|-------|------------|
| **ENGINE HOURS** — total, engine lifetime | `spark_dwell_time` | **no** — there is no command, parameter, magic value or web control that zeroes or decreases it. The only way back to zero is an NVS erase. |
| **TRIP HOURS** ("мотогодини місії") — since the last reset | `injection_time` | yes — by the **same** `MAV_CMD_USER_1` that clears the TRIP distance (see "Trip reset"). There is no separate command and no separate `param1` magic. |

The two hour counters advance in **lockstep**: the same whole-second amount, from the same single
sub-second carry, in the same place — they can never disagree about how long the engine ran, they
differ only in when they were last zeroed. Both are written by one NVS `persist()`, so a shutdown
cannot save one and lose the other, and `resetTrip()` writes through immediately rather than waiting
for the next interval.

> **`injection_time = 0` is a genuine zero**, the same note `fuel_pressure` carries: it means "the
> trip has just been reset", not "unknown". Nothing substitutes a sentinel.

**What makes it count.** Time accrues only while the CAN data is **valid** *and* the reported engine
RPM is at or above `ENGINE_HOURS_MIN_RPM` (300). Two consequences follow, both deliberate:

- **Idling counts.** 300 RPM sits well below this engine's idle, because idling is running time and
  wears the engine on the same schedule. This is a *different* threshold from
  `ENGINE_RUNNING_RPM_THRESHOLD` (1500), which is a gear-change and crank interlock chosen to sit
  safely above idle; sharing them would build an hour meter that ignores the yard.
- **Ignition on with the engine stopped does not count.** The ECU answers on ACC and reports
  `rpm = 0`, which is CAN-*valid* — so it is the RPM floor, not the validity flag, that stops the
  meter there.

**During a CAN outage the meter STOPS GROWING; it does not go `NaN`.** Accumulated running time is
history and stays true when the bus goes quiet. The elapsed interval is *discarded*, not banked, so
reconnecting does not credit the meter with the outage — the meter under-counts a CAN fault rather
than inventing hours through one, which is the same failure direction the odometer takes for a hall
sensor dropout. A consumer that sees a flat hour meter alongside a field of `NaN`s is looking at a
CAN outage; one that sees a flat meter alongside live sub-idle RPM is looking at a stopped engine.

The **total** never decreases between two messages; the **trip** decreases only on an accepted trip
reset, and only to zero. On first flash both NVS keys are absent, so both read `0.00` and count from
that moment: it is "hours since this firmware", not "hours since the engine was built", and there is
no seeding path. (A vehicle upgrading from the total-only firmware keeps its total and starts the
trip at zero — `"trip_s"` simply does not exist yet and reads as `0`.)

#### Trip reset

| | |
|---|---|
| Message | `COMMAND_LONG` (#76) |
| Command | `MAV_CMD_USER_1` (31010) |
| `param1` | `MAVLINK_CMD_TRIP_RESET_MAGIC` = `1.0`, matched within `MAVLINK_CMD_PARAM_EPSILON` (0.01) |
| `target_system` | `1` |
| `target_component` | `25` — **not** 0 |

**One command zeroes BOTH trip readings** — the TRIP distance (`fuel_pressure`) and the TRIP engine
hours (`injection_time`). They describe the same "since the operator last pressed reset" interval in
two units, so clearing one and leaving the other would leave the pair permanently incomparable.
There is **no second command and no second `param1` value**; the totals — ODO
(`barometric_pressure`) and total engine hours (`spark_dwell_time`) — are untouched.

The magic `param1` exists so a stray, replayed or mis-scripted `MAV_CMD_USER_1` cannot silently
destroy the operator's trip readings; it is compared with a tolerance, not float equality, so `NaN`
fails and is denied. See `COMMAND_ACK` below for the replies.

### `VFR_HUD` (74) — 5 Hz

| Field | MAVLink meaning / unit | What we send | Validity | Source |
|-------|------------------------|--------------|----------|--------|
| `airspeed` | [m/s] | `0.0` — no source on this component | — | literal |
| `groundspeed` | [m/s] | **Hall wheel-speed ground speed** | **NaN when the sensor reading is invalid**; a genuine `0.0` is sent as `0.0` | `state.speedMs` gated by `state.speedValid` |
| `heading` | [deg] compass | `0` — no source | — | literal |
| `throttle` | [%] `uint16_t`, 0–100 | **MEASURED TPS while `canValid`, COMMANDED throttle otherwise** | **always populated** — a `uint16_t` percent has no NaN encoding | arbitrated |
| `alt` | [m] MSL | `0.0` — no source | — | literal |
| `climb` | [m/s] | `0.0` — no source | — | literal |

`groundspeed`'s validity is the **sensor's**, independent of CAN: `SpeedSensor::isValid()` =
`initialized_ && everPulsed_ && !suspicious_` (`include/SpeedSensor.h`) — false until the first pulse
since boot, and false while latched suspicious after an implausible pulse loss. Sending `0.0` there
would make a severed hall lead read as a parked vehicle. There is no `> 0.0f` guard: `speedMs_` is
already clamped at zero by the decay path.

**The `throttle` fallback is never ambiguous:** when the commanded value is substituted,
`EFI_STATUS.throttle_position` is `NaN` in that *same tick* and `VFR_HUD.throttle` equals
`EFI_STATUS.throttle_out`. A HUD bar frozen at 0 while the operator holds throttle misleads in the
more dangerous direction. (`throttle_position` is what the ECU **measured**; `throttle_out` is the
**arbitrated servo output** — autopilot, web, gear-change boost, speed-limit cap or fail-safe idle —
not the raw demand of any one input.)

### Steering telemetry (`NAMED_VALUE_FLOAT`)

The **steering VESC** (Flipsky 75200) is the only smart driver on this vehicle. Its telemetry, the
measured steering position from the AS5600 on the shaft it drives, and the speed-scale the firmware
is applying to that shaft's commands ride **exactly six** `NAMED_VALUE_FLOAT` (251) messages from
component 25 — the **one scoped exception** to the "everything in the field that names it, in a
single `EFI_STATUS`" rule stated above.

| `name` | Value | Unit | Rate | Unknown | Gate |
|--------|-------|------|------|---------|------|
| `STEER_POS` | **MEASURED** steering position | % of calibrated travel, `-100` left … `0` centre … `+100` right | 5 Hz | `NaN` | `steerSensorOk && steerCalibrated` (AS5600) |
| `STEER_A` | VESC average **motor** current | A | 5 Hz | `NaN` | `steerDriverOk` |
| `VESC_V` | VESC-measured **input** voltage (24 V boost rail) | V | 5 Hz | `NaN` | `steerDriverOk` |
| `VESC_TEMP` | VESC power-stage (FET) temperature | °C | 1 Hz | `NaN` | `steerDriverOk` |
| `VESC_OK` | VESC link flag | `1.0` / `0.0` | 1 Hz | **never `NaN`** | *is* the flag |
| `STEER_SCA` | **Applied** steering speed-scale | `0`…`1` (`1.0` = not scaling) | 1 Hz | **never `NaN`** | none — every fault resolves to `1.0` |

`STEER_POS`, `STEER_A` and `VESC_V` ride the existing 5 Hz report tick beside `EFI_STATUS` and
`VFR_HUD`, so a GCS-side log lines the live values up without interpolation. `VESC_TEMP`, `VESC_OK`
and `STEER_SCA` sit on their own 1 Hz timer (`MAVLINK_STEER_SLOW_TX_MS`): a MOSFET's thermal time
constant is seconds, a link flag needs no faster, and the steering scale moves at the pace the
vehicle accelerates. The underlying VESC data refreshes at **3.3 Hz** (`STEER_VESC_TELEM_MS` =
300 ms), so the 5 Hz names repeat a sample roughly every third message.

`STEER_SCA` is not a VESC value at all — it is the scale the **vehicle layer** is applying to
autopilot steering commands after rate limiting, admitted under the same scoped exception. It is
**never `NaN`**: "not scaling" is the definite answer `1.0`, and a ground station must plot it as a
flat line at one rather than as a gap in the record. It is a reporting path only; nothing on the
MAVLink side computes or influences the scale.

The `time_boot_ms` field of all six carries the report tick's own `millis()`. It **wraps at 49.7
days** — that is what the field is *defined* as (`uint32_t` ms since boot), not a defect to fix.

#### Why `NAMED_VALUE_FLOAT` here, and only here

**Because the standard ESC messages never reach a consumer.** An earlier revision of this firmware
sent this same data as the two standard ESC telemetry messages, **ids 290 and 291**. A bench run on
Mission Planner 1.3.83 logged **zero** ESC frames in 50 s beside **277** `EFI_STATUS`: those two
message ids are absent from the ArduPilot dialect MP generates its `MAVLink.dll` from, so MP
discards the frames **before** decoding them. No plugin version can recover a packet the host parser drops. A message no consumer
can decode carries no telemetry, however well its field names fit.

**The collision argument that rules `NAMED_VALUE_FLOAT` out elsewhere does not bite here.** All
`NAMED_VALUE_FLOAT` share one message id and differ only by a name string, so a **name-agnostic**
store (MP's Inspector, the mavlink2rest cache, an unmapped `customField`) keeps only the last one
received — which is exactly why every engine value stays in one `EFI_STATUS`. But the intended
consumer, the QuadBike MP plugin, dispatches on the **(compid, name)** pair and never confuses the
six; and no name-agnostic store consumed these values in the first place. The exception is
**scoped**: the ECU values, both gears, the digital flags, the odometer and the trip stay in
`EFI_STATUS`, and these six names are the only `NAMED_VALUE_FLOAT` this component ever sends.

Also rejected:

- **`ESC_TELEMETRY_1_TO_4` (11030).** Present in the ArduPilot dialect, but its fields are **unsigned
  integers** (`centivolts`, `centiamps`, `temperature` in whole °C) with **no `NaN`**, so "unknown"
  would need in-band sentinels again, a signed steering position does not fit at all, and it is
  natively an *autopilot-to-GCS* message — sending it from component 25 invites exactly the
  "which ESC block is this?" confusion the plugin would then have to unpick.
- **More repurposed `EFI_STATUS` fields.** The fields still at `0.0` there name engine quantities
  this ECU could plausibly expose later, so they stay **RESERVED** and fail the *permanent-absence*
  test a repurposing must pass.
- **A Lua relay on the Pixhawk** re-emitting the same names with `gcs:send_named_float`. Not needed —
  the frames already reach the GCS from component 25 — but it stays available as a future option
  precisely *because* the carrier is now a message every dialect knows; it would change nothing here.

#### Field notes

- **`VESC_V` is not `EFI_STATUS.ignition_voltage`.** That field is the **ECU control-module supply**
  on the **12 V** side (PID `0x42`); this one is the **24 V boost rail** that powers the steering
  VESC, measured **by the VESC itself, at the load**. Different rails, different quantities. The
  board-side ADC on GPIO 9 measures the same rail at the *board* and keeps its existing
  web-telemetry path — a **divergence between the two under load is the sagging-rail diagnostic**,
  which is exactly why both are kept.
- **`STEER_A` is MOTOR current, not input current.** The same `COMM_GET_VALUES` payload carries both
  (`avg_motor_current` at offset 5, decoded; `avg_input_current` at offset 9, not decoded). Motor
  current is the one that moves with steering effort, so it is the one reported.
- **`STEER_POS` is percent of calibrated travel, NOT degrees.** The firmware holds no
  counts-to-degrees calibration anywhere (the AS5600's 0–4095 counts are only ever normalised
  against the stored centre and the two limits), and the two halves are scaled independently because
  the travel is asymmetric. **Negative is LEFT, positive is RIGHT** — the sign convention of
  `SteeringController::getSteeringPercent()`. **`0.0` is a real reading: straight ahead**, which is
  precisely why "unknown" must be `NaN` and never `0` (`getSteeringPercent()` itself returns `0.0f`
  when uncalibrated, so passing it through unguarded would report "wheels dead centre" for a vehicle
  whose steering position is entirely unknown — the same trap the `fuel_pressure` note above
  records, resolved the same way).
- **`STEER_POS` has the AS5600's gate, not the VESC's** — `isSensorOk() && isCalibrated()`,
  **independent** of `VESC_OK`. All four combinations are reachable and all four are meaningful:

| VESC | AS5600 | `STEER_A` / `VESC_V` / `VESC_TEMP` | `VESC_OK` | `STEER_POS` |
|------|--------|------------------------------------|-----------|-------------|
| healthy | healthy + calibrated | live | `1.0` | live position |
| **silent** | healthy + calibrated | `NaN` | `0.0` | **live position** — a dead VESC does not blind the shaft sensor |
| healthy | unhealthy or uncalibrated | live | `1.0` | **`NaN`** |
| **silent** | unhealthy or uncalibrated | `NaN` | `0.0` | `NaN` |

  This is the same sensor-driven caveat `fuel_flow` already carries: "everything is unknown, so one
  subsystem is down" does **not** hold in reverse for this value.

- **Consumer note.** The **commanded** steering already reaches the GCS as `SERVO_OUTPUT_RAW` from
  the autopilot's steering channel; `STEER_POS` adds the **measured** half, so command-vs-actual
  becomes visible for steering exactly as `throttle_out` vs `throttle_position` already makes it
  visible for throttle. A transient disagreement is the actuator slewing; a persistent one is a
  steering fault.
- **Not reported at all:** the raw VESC `mc_fault_code` and the boot-cumulative reply / fault-episode
  counters. Six names is the whole set; the fault code stays reachable on the **web portal**
  (`steer_vesc_fault`) and the serial console.

#### The `name` field is 10 bytes, and truncation would be silent

`NAMED_VALUE_FLOAT.name` is a fixed **`char[10]`**, and the library's pack helper copies **exactly
10 bytes** out of whatever pointer it is given. The firmware therefore zero-pads every name into a
local 10-byte buffer before packing (a shorter literal would be read past its terminator and put
`.rodata` on the wire), holds the six names as compile-time literals that are **never** built at
runtime, and `static_assert`s each one at ≤ 10 characters. On the receiving side: a name shorter
than 10 is NUL-padded, a name of exactly 10 carries **no terminator**, so a consumer must **trim on
length, not on NUL**.

#### Validity: "no reading" is signalled positively, and all five keep flowing

While the steering driver link is unhealthy — no valid reply within `STEER_VESC_COMM_TIMEOUT_MS`
(1000 ms), or **no reply ever since boot** — the VESC-sourced values go to `NaN` rather than
repeating the last number seen:

| `name` | While the VESC is silent | A consumer must |
|--------|--------------------------|-----------------|
| `STEER_A`, `VESC_V`, `VESC_TEMP` | `NaN` | render `--` |
| `VESC_OK` | `0.0` | read this as "VESC offline", and grey the three above |
| `STEER_POS` | **unaffected** — its gate is the AS5600's | keep reading it |

> **All five keep being sent in that state.** Suppression would make "the peripheral is alive and
> its VESC is down" look identical to "the peripheral is gone", and those need different responses
> from the operator. Three `NaN`s beside `VESC_OK` = `0.0` say precisely *"I am alive, my VESC is
> not."* This is the deliberate **opposite** of the `VISION_POSITION_DELTA` policy: that message is
> a **fusable** measurement feeding the EKF, where silence is the only safe degradation; these are
> **display** values, where a positively signalled "unknown" beats silence.

Nothing in this path is a control input: no VESC telemetry value triggers a steering stop, stall
latch, interlock or fail-safe. The existing fault-code stop and communication fail-safe inside
`SteeringController` remain the only consumers of that data for control purposes.

### `VISION_POSITION_DELTA` (11011) — ≤ 5 Hz, heavily gated

Sent behind the compile-time switch `MAVLINK_VISO_ENABLED` (`include/Constants.h:241`); the runtime
switch is the autopilot's `VISO_TYPE`.

| Field | MAVLink meaning / unit | What we send | Validity | Source |
|-------|------------------------|--------------|----------|--------|
| `time_usec` | [µs] timestamp | `esp_timer_get_time()` — 64-bit **monotonic boot µs**. Never `micros()`, whose 32-bit wrap (~71 min) would date the sample | always | timer |
| `time_delta_usec` | [µs] interval | The **actually measured** interval since the previous send, never an assumed 200 ms — ArduPilot divides by exactly this to recover velocity | always | measured |
| `angle_delta[3]` | [rad] roll, pitch, yaw | `{0, 0, 0}` — an honest "this sensor measures no rotation" | always | literal |
| `position_delta[3]` | [m] in `MAV_FRAME_BODY_FRD` | `{ v·dt, 0, 0 }` — x forward only; the wheel measures only the longitudinal axis | always | speed × dt |
| `confidence` | [%] 0–100 | `MAVLINK_VISO_CONFIDENCE` = `100.0` — picks the `EK3_VIS_VERR_MIN` end of the noise range | always | `include/Constants.h:242` |

`v`'s sign comes from `state.travelDirection`, the **physically sensed** gear (`+1` forward, `-1`
reverse, `0` neutral/unknown) — the hall sensor counts pulses and cannot tell forward from reverse.

#### External navigation odometry (EKF3 wheel-speed aiding)

This vehicle has **no reliable GPS**, so the hall wheel speed is fed to the autopilot as a fusable
measurement rather than a display value. ArduPilot routes the message `AP_VisualOdom` →
`AHRS::writeBodyFrameOdom()` → EKF3 body-frame odometry; no heading is involved, since the EKF
rotates the increment with its own attitude.

> ⚠ **Why not `VISION_SPEED_ESTIMATE`?** It was the original design and does **not** work without
> GPS: EKF3 never *starts* aiding from external-navigation velocity (`readyToUseExtNav()` is
> position-only), so with `EK3_SRC1_VELXY=6` and no position source the filter stays in
> constant-position mode and the velocity observation is given `EK3_NOAID_M_NSE` noise and discarded.
> Bench-verified 2026-08-21 (Pixhawk 2.4.8 / Rover 4.7.0): `VISION_SPEED_ESTIMATE` at 5 Hz left the
> EKF flags stuck at `0x00A7` with velocity 0, while `VISION_POSITION_DELTA` moved the filter to
> `AID_RELATIVE` (`0x012F`) with velocity tracking the injected speed. It is the only MAVLink message
> that reaches `writeBodyFrameOdom()`.

**Measurement noise comes from `confidence`, not `VISO_VEL_M_NSE`:**
`velErr = EK3_VIS_VERR_MIN + (EK3_VIS_VERR_MAX - EK3_VIS_VERR_MIN) * (1 - 0.01 * confidence)`, so at
`confidence = 100` the error is exactly `EK3_VIS_VERR_MIN` (default **0.1 m/s**; MAX default 0.9).

⚠ **Caveats, all bench-confirmed 2026-08-21:**

- **Do not raise `EK3_VIS_VERR_MAX` above 1.0.** EKF3 refuses to *start* body-odometry aiding unless
  `velErr < 1.0 m/s`, and nothing reports why aiding never began.
- **Verify `VISO_TYPE` exists.** `AP_VisualOdom` is compiled out of 1 MB-flash targets: a Pixhawk
  2.4.8 on the **fmuv2** (1 MB) build has no `VISO_*` parameters and silently ignores the message;
  the **fmuv3** (2 MB) build is required. Confirmed present on this vehicle.
- **Set the EKF origin** once per boot from the ground station (MP → "Set EKF Origin") — with
  `GPS1_TYPE = 0` the autopilot cannot learn where it is. The ESP32 deliberately sends no
  `SET_GPS_GLOBAL_ORIGIN`: with no position source, any origin it produced would be invented.
- **Body odometry gives `AID_RELATIVE`, not `AID_ABSOLUTE`** — position drift is unbounded. A large
  improvement over constant-position mode, but not GPS, and not to be trusted for navigation.
- **Keep a position source if you have one:** velocity-only ExternalNav with no position source can
  time out the EKF's position estimate (ArduPilot issue #23485).
- **Calibrate the compass** — the increment is rotated by the EKF's own attitude, so a bad heading
  steers the fused travel direction with nothing on the ESP32 side able to detect it.

#### When the message is NOT sent

Policy is **silence over zeros** — a zero the EKF fuses as "stopped" is far worse than a gap it
coasts through. The message is **skipped entirely** when:

| # | Gate |
|---|------|
| 1 | The autopilot link is down (`!isLinkUp()` — no autopilot `HEARTBEAT` for 3 s) |
| 2 | The speed reading is invalid (`!state.speedValid`) — including the sensor's **wire-fault latch**, where a cut signal wire decays to 0 and looks exactly like a standstill |
| 3 | Rolling with no recoverable sign: `travelDirection == 0` **and** `speedMs > MAVLINK_VISO_NEUTRAL_ZERO_MS` (0.14 m/s ≈ 0.5 km/h). A wrong sign injects an error of **twice** the speed |
| 4 | The integration baseline is unusable: `lastOdomTimeUs_ == 0`, `dt <= 0`, or `dt > MAVLINK_VISO_MAX_DT_US` (1 s) — the baseline is re-established and **one interval is discarded** |

A delta is an *integral*, valid only over an interval actually observed, so gates 1–3 also
**invalidate the baseline on the way out** (`lastOdomTimeUs_ = 0`): the first send after any break
falls into gate 4 and skips one more interval rather than multiplying the current speed by a
long-dead `dt`. **Healthy zeros are the deliberate exception and ARE sent** — gate 3's comparison is
strictly greater-than, so a genuine standstill, idling in neutral included, goes out as a zero-motion
update, which with no GPS is the strongest available constraint on estimator drift.

`getOdomTxCount()` and the 1 Hz `[MAV] viso:<n> dt:<ms>` debug line (`DebugFeature::MAVLINK`) make
the gates diagnosable on serial alone: a frozen count means a gate is closed, a `dt` that never
settles near 200 ms means one is flapping.

### `STATUSTEXT` (253) — on change, ≥ 250 ms apart

Emitted **outside** the 200 ms telemetry tick — the change test runs on every `report()` call.

| Field | MAVLink meaning | What we send | Validity |
|-------|-----------------|--------------|----------|
| `severity` | RFC-5424 severity | `MAV_SEVERITY_WARNING` (4) when `state.failsafe`, else `MAV_SEVERITY_INFO` (6) | always |
| `text[50]` | Status text, UTF-8, no NUL termination | `"gear=<G> ign=<I>"`, with `" FAILSAFE"` appended while in fail-safe | always |
| `id` / `chunk_seq` | Long-message reassembly | `0` / `0` — never chunked | always |

`<G>` is the **assumed** gear string (`state.gearTo`), `"?"` if null; `<I>` is
`"OFF"` / `"ACC"` / `"IGNITION"` / `"CRANKING"`, `"?"` if null. Example: `gear=N ign=IGNITION`, or
`gear=N ign=OFF FAILSAFE`.

**Trigger:** the first `report()` after boot, or any change of gear string, ignition string or the
fail-safe flag. **Rate limit:** `MAVLINK_STATUSTEXT_MIN_MS` = 250 ms; the "last sent" strings are
updated only when a message actually goes out, so a suppressed change is sent on the next call past
the window rather than lost. The `char text[50]` buffer caps the payload at 49 characters — always
within the field, never chunked.

### `COMMAND_ACK` (77) — reactive

| Field | MAVLink meaning | What we send |
|-------|-----------------|--------------|
| `command` | Command id being acknowledged | echo of the inbound `command` |
| `result` | `MAV_RESULT` | see the table below |
| `progress` | [%] when `IN_PROGRESS` | `0` — nothing here is long-running |
| `result_param2` | Command-specific extra | `0` — unused |
| `target_system` / `target_component` | The **requester** | inbound `msg.sysid` / `msg.compid` |

**Addressing is strict.** This component shares the autopilot's system id (`1`) and is distinguished
only by its component id (`25`), so a command is handled **only** when `target_system == 1` **and**
`target_component == 25`. A **broadcast** (`target_component == 0`) or a command aimed at any other
component is discarded **with no `COMMAND_ACK` and no log line**, so a normal GCS session against the
autopilot (arming, mode changes, mission upload) draws no acknowledgement from this component at all
and the ESP32 can never answer for the Pixhawk. The ACK goes back to the **requester** (typically
255/190), not to the learned autopilot.

| Inbound | `result` | Effect |
|---------|----------|--------|
| `MAV_CMD_USER_1` (31010), `param1` within `MAVLINK_CMD_PARAM_EPSILON` (0.01) of `MAVLINK_CMD_TRIP_RESET_MAGIC` (1.0) | `MAV_RESULT_ACCEPTED` (0) | **TRIP km *and* TRIP engine hours** both zeroed and persisted by the vehicle layer on its next iteration (< 40 ms at ≥ 25 Hz); **ODO and total engine hours unchanged**. On the wire: `fuel_pressure` **and** `injection_time` both fall to zero in the next `EFI_STATUS`, while `barometric_pressure` and `spark_dwell_time` are unchanged |
| `MAV_CMD_USER_1`, any other `param1` — **`NaN` included**, since `fabsf(NaN - 1.0) <= eps` is false | `MAV_RESULT_DENIED` (2) | nothing changes; `param1` and the requester are logged |
| Any other command | `MAV_RESULT_UNSUPPORTED` (3) | nothing changes; the command id and the requester are logged |

The transport only **latches** the request (`tripResetPending_`); the vehicle layer consumes it via
`consumeTripResetRequest()`, because the transport must not hold a `SpeedSensor&`. `ACCEPTED`
therefore means "accepted for execution", not "already done". Both trip counters are cleared inside
that one consumption, so they can never diverge. **The odometer and the total engine hour meter
cannot be reset by any means.**

## Requests toward the autopilot

All three are addressed to the learned autopilot (`targetSystem_`/`targetComponent_`, from the first
`HEARTBEAT` carrying `MAV_COMP_ID_AUTOPILOT1`). Apart from `HEARTBEAT`, no other **stream** is
subscribed: the wheel-odometry message the ESP32 sends is body-frame and needs no autopilot attitude.

| Message | Fields | Gate |
|---------|--------|------|
| `REQUEST_DATA_STREAM` (66) — legacy form, honored on ports where `SET_MESSAGE_INTERVAL` is denied (e.g. SERIAL4/TELEM4) | `req_stream_id` = `MAV_DATA_STREAM_RC_CHANNELS` (3), the group containing `SERVO_OUTPUT_RAW`; `req_message_rate` = `MAVLINK_SERVO_OUTPUT_RATE_HZ` = **25** Hz; `start_stop` = `1` | sent together with the next row: autopilot learned, link up, measured inbound command rate below `MAVLINK_STREAM_MIN_RATE_HZ` (10 Hz), at most once per `MAVLINK_STREAM_REREQUEST_MS` (3 s) |
| `COMMAND_LONG` (76) — `MAV_CMD_SET_MESSAGE_INTERVAL` (511), modern form | `param1` = `MAVLINK_MSG_ID_SERVO_OUTPUT_RAW` = `36`; `param2` = interval µs = `1000000 / 25` = **40000**; `confirmation` and `param3`–`param7` = `0` | same gate |
| `PARAM_REQUEST_READ` (20) — the read-only `SPEED_MAX` subscription | `param_id` = `"SPEED_MAX"` (`MAVLINK_PARAM_SPEED_MAX_ID`); `param_index` = `-1` ("look it up by name" — the index is not stable across firmware builds and must never be hard-coded) | first request `MAVLINK_PARAM_FIRST_DELAY_MS` (1 s) after the autopilot is learned, then every `MAVLINK_PARAM_POLL_MS` (5 s), **for ever** — the poll *is* the change detector |

Once the stream is healthy the requests stop; an autopilot reboot restarts them automatically. The
inbound `COMMAND_ACK` for `SET_MESSAGE_INTERVAL` is decoded and logged only.

## Autopilot speed limit (`SPEED_MAX`)

The ESP32 has its own max-speed throttle limiter, driven by the hall wheel sensor. Its ceiling has
**exactly one source: ArduPilot's `SPEED_MAX` parameter (m/s)**. There is no local ceiling, no
stored value and no enable toggle — the limiter is always armed and does nothing when no usable
`SPEED_MAX` is available. A change made from Mission Planner or MAVProxy lands within about 5 s
(the poll above, plus unsolicited `PARAM_VALUE` frames are accepted), with no reboot.

> **The parameter is never written and never persisted.** The ESP32 sends no `PARAM_SET` — it only
> ever reads, the value lives in RAM and dies with the link, and nothing about the limiter is stored
> in NVS.

### When the limit applies

| `SPEED_MAX` state | Limit enforced |
|---|---|
| fresh and > 0 | that value, in m/s |
| `0` | *none* — 0 means "no limit", exactly as ArduPilot reads it, and it is what MP's speed-limit sign writes to clear a limit |
| never received / rejected (NaN, ∞, negative, > `MAVLINK_PARAM_SPEED_MAX_MS` = 30 m/s) | *none* |
| older than `MAVLINK_PARAM_STALE_MS` (16 s ≈ 3 polls) | *none* |
| link down (no `HEARTBEAT` for 3 s) | *none* |

**The fallback is no limiting** — losing the autopilot never leaves the vehicle throttled by a stale
ceiling nobody can see or change. A rejected `PARAM_VALUE` is logged and the previously accepted
value stays in force, so one corrupt frame cannot invalidate a good subscription; one whose sender
system **or** component id is not the learned autopilot is dropped outright, so a ground station on
the same wire (typically sysid 255) cannot move the vehicle's speed ceiling. All speeds inside the
firmware are **m/s**; km/h appears only in the web JSON and in human-readable debug strings.

### What the limiter then does

The throttle ceiling is **tapered**, not stepped: 100 % at `limit − SPEED_LIMIT_TAPER_BAND_MS`
(1.4 m/s ≈ 5 km/h), falling linearly to `SPEED_LIMIT_FLOOR_PCT` (10 %) at the limit and holding that
floor above it. The ceiling is slew-limited to `SPEED_LIMIT_CEILING_SLEW_PCT_S` (200 %/s) so
engagement cannot snap the servo — the rate limit applies to the *ceiling*, never to the driver's own
throttle movements. There is no hard cut (removing all drive mid-corner is a stability event, not a
safety measure); an invalid speed reading **fails open**, and the gear-change throttle boost is
outside the limiter entirely.

This is a **throttle limiter, not a speed governor**: it only withholds throttle and **never applies
the brake**. Total reaction latency is about **0.65 s** (a 200 ms speed sample plus the 0.45 s the
ceiling needs to slew from 100 % to the floor) — roughly **5 m past the limit at 30 km/h** and
**11 m at 60 km/h**. On a descent it cannot hold the ceiling at all: gravity, not the engine, is
doing the accelerating.

The 1 Hz `[MAV]` debug line reports `spdmax:<m/s> age:<ms> valid:<Y|N>`, separating "never received"
(`nan`), "answered and fresh" (a 0 → 5000 ms age sawtooth) and "stale/link down" (`valid:N`). The
vehicle layer logs `[SPEED] Speed limit: … m/s (… km/h) from SPEED_MAX` on each change, and
`[SPEED] Speed limit: none …` when it stops limiting.

## Fail-safe

- No `SERVO_OUTPUT_RAW` for `MAVLINK_CMD_TIMEOUT_MS` (500 ms) → commands invalid → vehicle enters
  fail-safe (center steering, idle throttle, NEUTRAL, parking brake).
- No autopilot `HEARTBEAT` for `MAVLINK_HEARTBEAT_TIMEOUT_MS` (3000 ms) → link down.

## Conventions

### `NaN` means "value unknown", never "zero"

`NaN` is used wherever the float field allows it and the value is genuinely unavailable, so a
consumer renders `--` instead of a misleading number. A zero is **never** substituted for an unknown,
and the definitions' "zero means unknown" sentinels (`ignition_voltage`, `fuel_pressure`) are
deliberately **not** honored — on this vehicle a real zero is more likely than an unknown. The
inverse holds too: a field that *can* be `NaN` and is not is a real measurement, `0.0` included.

### Fields that are ALWAYS valid (never `NaN`)

| Field | Why |
|-------|-----|
| `EFI_STATUS.fuel_consumed` (assumed gear) | the controller always knows the gear it commanded |
| `EFI_STATUS.throttle_out` (commanded throttle) | the arbitrated servo output is always known locally |
| `EFI_STATUS.pt_compensation` (digital flags) | relay ground truth |
| `EFI_STATUS.barometric_pressure` (ODO) / `fuel_pressure` (TRIP) | distance already driven depends on neither CAN health nor the current speed reading |
| `EFI_STATUS.spark_dwell_time` (total engine hours) / `injection_time` (trip engine hours) | time already run is history; during a CAN outage both stop growing rather than going unknown |
| `EFI_STATUS.health` | constant `1`; **not** a health signal — do not read CAN state from it |
| `VFR_HUD.throttle` | `uint16_t` percent, no NaN encoding — falls back to commanded |
| `VESC_OK` | locally known; it **is** the VESC link's health, and an "unknown" here would defeat its only purpose |
| all `HEARTBEAT`, `COMMAND_ACK` and `VISION_POSITION_DELTA` fields | all locally known; VISO goes **silent** rather than invalid |


### Telling "CAN down" from "sensor down"

Five independent validity domains share the link. They do **not** move together:

| Domain | Gate | Fields it controls |
|--------|------|--------------------|
| **CAN / ECU** | `state.canValid` (`CANController::VehicleData::dataValid`) | `rpm`, `engine_load`, `throttle_position`, `intake_manifold_pressure`, `intake_manifold_temperature`, `cylinder_head_temperature`, `ignition_voltage` — **seven** fields, all NaN together |
| **Gear switches** | physical gear reads UNKNOWN | `fuel_flow` only |
| **Hall speed sensor** | `state.speedValid` = `SpeedSensor::isValid()` | `VFR_HUD.groundspeed` (→ `NaN`) and `VISION_POSITION_DELTA` (→ silence) |
| **Steering VESC link** | `state.steerDriverOk` = `SteeringController::isDriverOk()` | `STEER_A`, `VESC_V`, `VESC_TEMP` (→ `NaN`) and `VESC_OK` (→ `0.0`) |
| **Steering position sensor** | `steerSensorOk && steerCalibrated` (AS5600 + stored calibration) | `STEER_POS` only (→ `NaN`) |

- **All seven CAN fields NaN at once** → CAN/ECU is down (or the engine harness is off); gear,
  `throttle_out`, flags, ODO, TRIP and ground speed are unaffected.
- **`fuel_flow` alone NaN** → the gear switches are unsure (mid-shift, ambiguous pattern, faulted
  input expander). Says **nothing** about CAN — "everything is NaN so CAN is down" does not hold in
  reverse for this field.
- **`VFR_HUD.groundspeed` NaN and the VISO count frozen, CAN fields fine** → the hall sensor has
  never pulsed since boot or is latched suspicious; ODO/TRIP stop advancing but stay valid.
- **`STEER_A` / `VESC_V` / `VESC_TEMP` all `NaN` with `VESC_OK` = `0`, everything else fine** → the
  VESC is unplugged, unpowered or silent past its comm timeout. All five names **keep arriving**,
  which is how this is told apart from "the peripheral is gone"; CAN, gear, speed and `STEER_POS`
  are unaffected.
- **`STEER_POS` = `NaN` alone, with `VESC_OK` = `1`** → the AS5600 is unhealthy or the steering is
  uncalibrated. Says **nothing** about the VESC — like `fuel_flow`, this value has its own sensor,
  its own gate and its own `NaN`, and the reverse implication does not hold.
- **Nothing at all from 1/25** → the peripheral or the serial link is gone. The two `HEARTBEAT`s are
  independent: the Pixhawk's continuing while comp 25's stops is exactly this case.

### Ground-station plugin compatibility

> ⚠ **BREAKING — flash and plugin must be updated together.** The QuadBike Mission Planner plugin
> decodes these messages **by packet subscription and field position**, so any change to which field
> carries which value is a breaking wire change with **no compatible intermediate state**. Two fields
> moved in the last remap: **gear** left `engine_load` and **measured throttle** left `throttle_out`.

| Value | Old field | New field | Plugin action |
|-------|-----------|-----------|---------------|
| Gear (assumed) | `engine_load` | `fuel_consumed` | **move** the decode |
| Measured throttle (%) | `throttle_out` | `throttle_position` | **move** the decode |
| Physical gear | — | `fuel_flow` | **new** — render NaN as "--"; fall back to the assumed gear for a single gear indicator |
| ECU calculated load (%) | — | `engine_load` | **new** |
| Manifold pressure (kPa) | — | `intake_manifold_pressure` | **new** |
| Commanded throttle (%) | — | `throttle_out` | **new** — always valid, never NaN |

#### Engine hours in `spark_dwell_time` / `injection_time` — **non-breaking**

Two new read-only fields, both in slots the plugin ignores today, so **nothing it decodes moves** and
an un-updated plugin keeps working and simply does not show the values.

| Value | Field | Plugin action |
|-------|-------|---------------|
| Engine hours (total) | `EFI_STATUS.spark_dwell_time` | **new** — read-only, **hours**, render to **2 decimals**, always valid (never `NaN`), monotonically non-decreasing |
| Engine hours (trip, "мотогодини місії") | `EFI_STATUS.injection_time` | **new** — read-only, **hours**, render to **2 decimals**, always valid (never `NaN`); falls to `0.00` on a trip reset, where **0 is a genuine zero** |

Offer **no reset control** for either: the total has no reset on any interface, and the trip hours
are cleared by the **existing** trip-reset command, which needs **no change** — the same
`MAV_CMD_USER_1` / `param1 = 1` that zeroes the TRIP km now zeroes the trip hours too, in the same
`EFI_STATUS` tick. A plugin that already sends it is already correct; a plugin that shows both trip
readings should expect them to clear together. Against **legacy firmware** both fields are a literal
`0.0`, so an updated plugin degrades to `0.00 h` — a visibly absent value rather than a wrong one —
which is why this can ship ahead of or behind the flash.

#### Steering VESC named floats (251) — **non-breaking**

The plugin must add **one** packet subscription, for **msg id 251 (`NAMED_VALUE_FLOAT`)**, and
dispatch inside it on the `name` field. **Nothing it decodes today changes**, so an un-updated
plugin keeps working exactly as before and simply does not show the new data — one more unknown
message id on a link that already carries several.

Decoding rules, all mandatory:

- **Filter on `compid == 25`.** Message id 251 is shared with anything else on the link that chooses
  to send it, including a Lua script on the autopilot; only component 25 is this firmware.
- **Dispatch on the trimmed `name`.** The field is a fixed **10-byte** array. A name shorter than 10
  characters is NUL-padded; a name of exactly 10 characters carries **no terminator at all**, so
  trim on length, never on a NUL that may not be there. All six names below are shorter than 10
  today, but the decode must not depend on that.
- **Render `NaN` as `--`.** Never as `0`.
- **`VESC_OK == 0` greys the VESC values.** It is the name that says *why* `STEER_A`, `VESC_V` and
  `VESC_TEMP` went `NaN`; `STEER_POS` is **not** covered by it and keeps its own validity.

| `name` | Value | Unit | Rate |
|--------|-------|------|------|
| `STEER_POS` | measured steering position, `-100` left … `0` centre … `+100` right | % of calibrated travel | 5 Hz |
| `STEER_A` | VESC average **motor** current | A | 5 Hz |
| `VESC_V` | VESC-measured input voltage (**24 V boost rail**) | V | 5 Hz |
| `VESC_TEMP` | VESC FET temperature | °C | 1 Hz |
| `VESC_OK` | VESC link flag, `1.0` / `0.0` — **never `NaN`** | — | 1 Hz |
| `STEER_SCA` | applied steering speed-scale, `0`…`1` — `1.0` = not scaling, **never `NaN`** | — | 1 Hz |

Mission Planner will **not** populate any native field from these (it maps its own from the
autopilot, component 1 only), so the plugin reads the **raw packet**, exactly as it already does for
`EFI_STATUS`.

**Failure mode if the plugin is NOT updated:** it keeps reading gear from `engine_load`, now the
ECU's calculated load, 0–100 %. The GCS shows a "gear" ranging up to 100 that tracks engine effort —
wildly out of range rather than a plausible wrong gear, so the mismatch is obvious, but the gear
display is useless until the plugin is updated.

Rules that keep such changes rare:

1. **Every value sits in the field that names it**, and a repurposing must pass the
   *permanent-absence* test and be documented at the pack site.
2. **Reserved fields stay at `0.0`** — never repurposed (see the three above).
3. **Adding a new message id is not breaking**: an un-updated plugin ignores it (precedents:
   `VISION_POSITION_DELTA`, and the steering `NAMED_VALUE_FLOAT` set).
4. **Consumers harden by treating an out-of-range value as "--"** — a gear outside `-1..2`, say,
   turns a field-remap mismatch into a clean "no data" instead of a nonsense number.
