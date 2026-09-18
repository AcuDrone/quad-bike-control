# MAVLink Vehicle Interface — Setup

The ESP32 talks to the Pixhawk 2.4.8 over a single MAVLink 2 serial link
(replacing the former SBUS-Out connection).

## Wiring

| Pixhawk TELEM2 | ESP32 (UART1) | `Control_v0` header |
|----------------|---------------|---------------------|
| TX             | GPIO 18 (`PIN_MAVLINK_RX`) | X9 pin 3 |
| RX             | GPIO 17 (`PIN_MAVLINK_TX`) | X9 pin 2 |
| GND            | GND | X9 pin 4 |

Non-inverted, 8N1, **115200 baud**. No inverter circuit (unlike SBUS).

> ⚠ The link runs through the **BSS138 level shifter** on header X9 (Pixhawk 5 V ↔ ESP32 3.3 V).
> That shifter is passive and rate-limited: **115200 is the only supported baud** on this link —
> do not raise `SERIAL2_BAUD`.

> ⚠ GPIO 8 is **not** free — it is `PIN_SPEED_SENSOR` (hall wheel-speed input, header X2). The
> pre-`Control_v0` GPIO 8 / GPIO 15 MAVLink pair no longer exists.

## ArduPilot parameters (TELEM2)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SERIAL2_PROTOCOL` | `2` (MAVLink2) | TELEM2 speaks MAVLink 2 |
| `SERIAL2_BAUD` | `115` (115200) | Match the ESP32 link baud |

The ESP32 requests **one** inbound stream on connect — `SERVO_OUTPUT_RAW` at
**25 Hz** (`MAVLINK_SERVO_OUTPUT_RATE_HZ`, the command input) — via both
`REQUEST_DATA_STREAM` and `MAV_CMD_SET_MESSAGE_INTERVAL`, so no manual
stream-rate parameter is required. It is re-requested while its measured rate is
low, so an autopilot reboot recovers automatically. (If the autopilot ignores
both requests, set `SRx_RC_CHAN`.) Apart from `HEARTBEAT`, no other **stream** is
subscribed: the wheel-odometry message the ESP32 sends is **body-frame** and
needs no attitude from the autopilot. One **parameter**, `SPEED_MAX`, is polled
read-only — see "Autopilot speed limit" below.

## Servo function mapping

The ESP32 reads command channels from `SERVO_OUTPUT_RAW.servoN_raw`. ArduPilot's
`SERVOn_FUNCTION` outputs must align with these indices (see
`ServoChannelConfig` in `include/Constants.h`):

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

Engine data is sourced from the CAN bus (`CANController::VehicleData`); ground speed comes from
the hall wheel-speed sensor, independently of CAN health. Every value in `EFI_STATUS` sits in the
field that **names** it, so the message is self-describing; only three fields are repurposed
(marked ⚑), and each is documented below.

| Data | MAVLink message / EFI_STATUS field | Source | Component | Rate |
|------|------------------------------------|--------|-----------|------|
| Liveness + fail-safe status | `HEARTBEAT` (`MAV_TYPE_GROUND_ROVER`) | — | 25 | 1 Hz |
| Engine RPM | `rpm` | PID `0x0C` | 25 | 5 Hz¹ |
| Coolant temp (°C) | `cylinder_head_temperature` | PID `0x05` | 25 | 5 Hz¹ |
| Intake air temp (°C) | `intake_manifold_temperature` | PID `0x0F` | 25 | 5 Hz¹ |
| Manifold absolute pressure (kPa) | `intake_manifold_pressure` | PID `0x0B` | 25 | 5 Hz¹ |
| ECU calculated engine load (%) | `engine_load` | PID `0x04` | 25 | 5 Hz¹ |
| Measured throttle position (%) | `throttle_position` | PID `0x11` | 25 | 5 Hz¹ |
| Commanded (arbitrated) throttle (%) | `throttle_out` | servo output | 25 | 5 Hz |
| Module supply voltage (V) | `ignition_voltage` | PID `0x42` | 25 | 5 Hz¹ |
| ⚑ **Assumed** gear | `fuel_consumed` | commanded | 25 | 5 Hz |
| ⚑ **Physical** gear | `fuel_flow` | opto switches | 25 | 5 Hz³ |
| ⚑ Digital outputs (wheel lock, front light) | `pt_compensation` (bitmask) | relays | 25 | 5 Hz |
| Gear / ignition / fail-safe transitions | `STATUSTEXT` | — | 25 | on change |
| Ground speed (hall sensor) | `VFR_HUD.groundspeed` (m/s) | GPIO 8 / X2 | 25 | 5 Hz |
| Throttle for the HUD bar (%) | `VFR_HUD.throttle` | measured, else commanded | 25 | 5 Hz |
| Wheel odometry for the EKF | `VISION_POSITION_DELTA` (11011) | GPIO 8 / X2 | 25 | 5 Hz² |

**NaN policy.** ¹ marks the seven **CAN-gated** fields — `rpm`, `cylinder_head_temperature`,
`intake_manifold_temperature`, `intake_manifold_pressure`, `engine_load`, `throttle_position` and
`ignition_voltage`. All seven are sent as **NaN** while CAN data is invalid (`!canValid`), so the
consumer shows "--" instead of misleading zeros. Three fields are **never NaN**: `fuel_consumed`
(the controller always knows the gear it commanded), `throttle_out` (the arbitrated servo output is
always known locally) and `pt_compensation` (relay ground truth).

³ `fuel_flow` is the exception to both rules: it is `NaN` whenever the **physical gear reads
UNKNOWN**, and that validity is the **gear sensor's, independent of CAN**. So a lone `fuel_flow`
NaN with every other field populated means "the gear switches are unsure" and says nothing about
CAN health — "everything is NaN, so CAN is down" does not hold in reverse for this field.
`VFR_HUD.throttle` has no NaN encoding at all (`uint16_t` percent); see the fallback rule below.

² `VISION_POSITION_DELTA` is **gated**: it is skipped entirely (not sent as zeros) when the link
is down, the speed sensor is invalid, or the vehicle is rolling in neutral. See "External
navigation odometry" below.

> **Why one `EFI_STATUS`, not per-value `NAMED_VALUE_FLOAT`:** all `NAMED_VALUE_FLOAT` messages
> share a single message id and are distinguished only by a `name` field, so any name-agnostic
> store (MP's MAVLink Inspector, the mavlink2rest cache, unmapped `customField`s) keeps only the
> last one received — the values appear to override each other. Packing everything into
> **distinct fields of one `EFI_STATUS`** eliminates that collision entirely. Mission Planner
> won't surface a *peripheral's* EFI in its native `efi_*` fields (those map only the autopilot,
> comp 1), but the raw packet **is** delivered, so the QuadBike MP plugin reads it cleanly via a
> packet subscription. Verified against BlueOS + MP 1.3.83.

> **`throttle_position` vs `throttle_out`.** `throttle_position` is what the **ECU measured**;
> `throttle_out` is what the vehicle was **commanded** — the arbitrated servo output (autopilot,
> web, gear-change boost override, speed-limit cap or fail-safe idle), not the raw demand of any
> one input. `VFR_HUD.throttle` shows the measured value while CAN is healthy and falls back to the
> commanded one when it is not, because a `uint16_t` percent has no NaN encoding and a HUD bar
> frozen at 0 while throttle is applied misleads in the more dangerous direction. The fallback is
> never ambiguous: `throttle_position` reads NaN in exactly those ticks, and `VFR_HUD.throttle`
> then equals `throttle_out`.

### Gear: assumed vs physical

Two gear values are reported, both encoded by **physical gear-sequence position**:

| Gear | Value | Assumed (`fuel_consumed`) moving toward it | Physical (`fuel_flow`) while moving |
|------|-------|--------------------------------------------|--------------------------------------|
| REVERSE | −1.0 | (from N) −0.5 | NaN |
| NEUTRAL | 0.0 | (from R/H) −0.5 / 0.5 | NaN |
| HIGH | 1.0 | (from N/L) 0.5 / 1.5 | NaN |
| LOW | 2.0 | (from H) 1.5 | NaN |

- **`fuel_consumed` = assumed gear = intent.** The controller's commanded, time-based gear. A whole
  number = settled (or briefly dwelling) at that gear; a `.5` value = the servo is moving between
  the two adjacent gears. Example R→L: `-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0` as the box steps
  R→N→H→L. **Always valid** — never NaN, independent of both CAN health and gear-switch health.
- **`fuel_flow` = physical gear = measurement.** Read from the transmission's opto switches
  (`TransmissionController::getPhysicalGear()`), the same source the fail-safe and the
  `VISION_POSITION_DELTA` direction sign already trust. **Never interpolated**: it is an integer or
  it is `NaN`. It reads `NaN` whenever the physical gear is UNKNOWN — between detents during a
  shift, an ambiguous switch pattern, or a faulted input expander.

> **`fuel_flow = NaN` during a shift is expected, not a fault.** So is a brief disagreement between
> the two values while the box is moving. The interesting condition is a **persistent** one: once
> `fuel_consumed` settles on an integer, `fuel_flow` should re-acquire the same integer within the
> settle window. A `fuel_flow` that stays NaN, or that settles on a *different* gear, means the
> shift did not physically complete (or the gear-switch input is faulted). The firmware
> deliberately does not try to distinguish these on the wire — a consumer has both values and a
> clock, and can apply its own timeout.

### Ground-station plugin compatibility

> ⚠ **BREAKING — flash and plugin must be updated together.** The QuadBike Mission Planner plugin
> decodes `EFI_STATUS` by packet subscription. Two fields moved in this remap: **gear** left
> `engine_load` and **measured throttle** left `throttle_out`. There is no compatible intermediate
> state.

| Value | Old field | New field | Plugin action |
|-------|-----------|-----------|---------------|
| Gear (assumed) | `engine_load` | `fuel_consumed` | **move** the decode |
| Measured throttle (%) | `throttle_out` | `throttle_position` | **move** the decode |
| Physical gear | — | `fuel_flow` | **new** — render NaN as "--"; fall back to the assumed gear for a single gear indicator |
| ECU calculated load (%) | — | `engine_load` | **new** |
| Manifold pressure (kPa) | — | `intake_manifold_pressure` | **new** |
| Commanded throttle (%) | — | `throttle_out` | **new** — always valid, never NaN |

**Failure mode if the plugin is NOT updated:** it keeps reading gear from `engine_load`, which now
carries the ECU's **calculated engine load, 0–100 %**. The GCS will show a "gear" that ranges up to
100 and tracks engine effort — a wildly out-of-range value rather than a plausible wrong gear, so
the mismatch is obvious, but the gear display is useless until the plugin is updated. As hardening,
the plugin **should treat any gear value outside `-1..2` as "--"**, which turns the failure into a
clean "no data" instead of a nonsense number.

Generic tools (MP's MAVLink Inspector, mavlink2rest, log analysers) label the fields by their
MAVLink names and will therefore show the two gear values as fuel quantities in cm³ / cm³·min⁻¹.
That is inherent to the repurposing and is harmless — the values are visibly implausible as fuel.
The fuel pair is safe to repurpose permanently on this vehicle: the 2026-08-14 bench probe
confirmed PIDs `0x2F` (fuel level) and `0x5C` (oil temp) are **absent** from this ECU, so no genuine
fuel signal can ever compete for the fields.

The ESP32 uses system id `1`, component id `25` (`MAV_COMP_ID_USER1`).

> **Oil temperature** is not reported — the ECU does not provide it over CAN (PID `0x5C` is
> confirmed absent on this vehicle). **Vehicle speed IS reported**, but not from the ECU: the
> ECU's OBD-II speed PID (`0x0D`) is verified dead (always 0, even in motion — no speed input on
> this vehicle). Speed comes from the **hall wheel-speed sensor** on GPIO 8 / X2 and is sent as
> `VFR_HUD.groundspeed`, and — gated — as `VISION_POSITION_DELTA` for the EKF.

## External navigation odometry (EKF3 wheel-speed aiding)

This vehicle has **no reliable GPS**, so the hall wheel speed is fed to the autopilot as a fusable
measurement rather than a display value. The ESP32 sends `VISION_POSITION_DELTA` (11011) at 5 Hz;
ArduPilot routes it `AP_VisualOdom` → `AHRS::writeBodyFrameOdom()` → EKF3 body-frame odometry.

The message carries a **body-frame distance increment** — `position_delta = {v·dt, 0, 0}` along
the vehicle's own forward axis, `angle_delta = {0, 0, 0}`, and `time_delta_usec` set to the
**actually measured** interval since the previous message. No heading is involved: the EKF rotates
the increment with its own attitude. The sign of `v` comes from the **physically sensed** gear
(opto switches), never the commanded one.

> ⚠ **Why not `VISION_SPEED_ESTIMATE`?** It was the original design and it does **not** work on a
> GPS-less vehicle. EKF3 never *starts* aiding from external-navigation velocity: `readyToUseExtNav()`
> is position-only, so with `EK3_SRC1_VELXY=6` and no position source the filter stays in
> constant-position mode, where the velocity observation is given `EK3_NOAID_M_NSE` noise and
> discarded. Verified on the 2026-08-21 bench against a real Pixhawk 2.4.8 / Rover 4.7.0:
> `VISION_SPEED_ESTIMATE` at 5 Hz left the EKF flags stuck at `0x00A7` with velocity 0, while
> `VISION_POSITION_DELTA` moved the filter to `AID_RELATIVE` (`0x012F`) and the reported velocity
> tracked the injected speed. `VISION_POSITION_DELTA` is the only MAVLink message that reaches
> `writeBodyFrameOdom()`, which is the one path `EK3_SRC1_VELXY=6` can bootstrap from.

### ArduPilot parameters

| Parameter | GPS-less (bench-proven) | With a GPS fitted | Meaning |
|-----------|-------------------------|-------------------|---------|
| `VISO_TYPE` | `1` (MAVLink) | `1` | Enable the visual-odometry backend. **Verify this parameter exists first** — see the build caveat below. |
| `EK3_SRC1_VELXY` | `6` (ExternalNav) | `6` | Fuse horizontal velocity from body odometry. |
| `EK3_SRC1_POSXY` | **`0`** (None) | **`3`** (GPS) | With no GPS there is no position source to wait for; with one, keep it — see the position-timeout caveat below. |
| `GPS1_TYPE` | **`0`** (None) | `2` (or as the receiver needs) | Named `GPS_TYPE` before ArduPilot 4.7. Set to `0` on the bench so an absent/unhealthy GPS could not hold the filter waiting for a position that never arrives. |
| `VISO_DELAY_MS` | `10` (default) → tune toward ~150 | same | Measurement lag. The hall speed is a window average, so the true lag is ~100 ms at speed and stretches toward ~1 s at a crawl — a single scalar cannot cancel it. Tune against `XKF3 IVN/IVE` innovations and record the final value here. |
| `VISO_POS_X` / `_Y` / `_Z` | rear-hub lever arm (m) | same | Position of the measuring wheel relative to the IMU, in body frame. |
| `VISO_ORIENT` | `0` (`ROTATION_NONE`) | same | The firmware already emits body-frame-forward. Unlike the velocity path, `VISO_ORIENT` **is** applied here (to both `angle_delta` and `position_delta`), so leaving it non-zero would silently rotate the measurement. |

**Parameters that do NOT apply to this path** — verified by reading
`AP_VisualOdom_Backend::handle_vision_position_delta_msg` in ArduPilot-4.7:

| Parameter | Why not |
|-----------|---------|
| `VISO_QUAL_MIN` | The delta handler performs no quality gating at all. The `_quality >= get_quality_min()` check exists only in the pose-estimate and vision-speed paths. |
| `VISO_VEL_M_NSE` | Read only by `handle_vision_speed_estimate`. The delta path's noise comes from `EK3_VIS_VERR_MIN`/`MAX` — see below. |
| `VISO_SCALE` | Applied only in `handle_pose_estimate`. |

**Measurement noise comes from `confidence`, not `VISO_VEL_M_NSE`.** The firmware sends
`confidence = 100`, and EKF3 derives the observation error as

```
velErr = EK3_VIS_VERR_MIN + (EK3_VIS_VERR_MAX - EK3_VIS_VERR_MIN) * (1 - 0.01 * confidence)
```

so at `confidence = 100` the error is exactly `EK3_VIS_VERR_MIN` (default **0.1 m/s**; the MAX
default is 0.9 m/s).

> ⚠ **Do not raise `EK3_VIS_VERR_MAX` above 1.0.** EKF3 refuses to *start* body-odometry aiding
> unless `velErr < 1.0 m/s`, and nothing reports why aiding never began. Sending `confidence = 100`
> keeps this firmware at the `MIN` end regardless, but the interaction is worth knowing before
> touching either parameter.

`SERIAL2_PROTOCOL` / `SERIAL2_BAUD` are unchanged from the table above — this rides the existing
TELEM2 link.

> ⚠ **Set the EKF origin.** With `GPS1_TYPE = 0` the autopilot has no way to learn where it is,
> and the origin must be set **once per boot from the ground station** (Mission Planner → "Set EKF
> Origin"). The ESP32 deliberately does not send `SET_GPS_GLOBAL_ORIGIN`: it has no position source
> of its own, so any origin it produced would be invented.

> ⚠ **Body odometry gives `AID_RELATIVE`, not `AID_ABSOLUTE`.** Velocity and dead-reckoned
> position are relative to that origin, and position drift is unbounded over time. This is a large
> improvement over constant-position mode (which carried zero information), but it is not GPS and
> must not be trusted for navigation.

> ⚠ **Verify `VISO_TYPE` exists before relying on this.** `AP_VisualOdom` is compiled out of
> 1 MB-flash targets. A Pixhawk 2.4.8 running the **fmuv2** (1 MB) build has no `VISO_*`
> parameters at all and will silently ignore the message; the **fmuv3** (2 MB) build is required.
> Confirmed present on this vehicle on 2026-08-21.

> ⚠ **Keep a position source if you have one.** Velocity-only ExternalNav with no position source
> can time out the EKF's position estimate (ArduPilot issue #23485). If a GPS is fitted, restore
> `GPS1_TYPE = 2` and `EK3_SRC1_POSXY = 3` alongside `EK3_SRC1_VELXY = 6`.

> ⚠ **Calibrate the compass.** The body-frame increment is rotated by the EKF's own attitude, so a
> bad heading steers the fused travel direction with nothing on the ESP32 side able to detect it.

### When the message is NOT sent

Policy is **silence over zeros** — a zero the EKF fuses as "stopped" is far worse than a gap it
coasts through. The message is skipped entirely when:

1. the autopilot link is down;
2. the speed reading is invalid — this includes the sensor's **wire-fault latch**, where a cut
   signal wire decays to 0 km/h and looks exactly like a standstill;
3. the vehicle is rolling in **NEUTRAL** (or the gear reads UNKNOWN mid-shift) above
   `MAVLINK_VISO_NEUTRAL_ZERO_KMH` = 0.5 km/h — the direction sign is unrecoverable, and a wrong
   sign injects an error of twice the speed.

**Healthy zeros are the exception and ARE sent.** A genuine standstill — including idling in
neutral — is transmitted as a zero-motion update, which with no GPS is the strongest available
constraint on estimator drift.

**After any gap, one interval is discarded.** A delta is an *integral*, so it is only valid over
an interval that was actually observed. Every gate invalidates the integration baseline on the way
out, and any gap longer than `MAVLINK_VISO_MAX_DT_US` (1 s) does the same; the first message after
a break is therefore skipped and the baseline re-established, so the current speed is never
multiplied by the length of the outage.

The 1 Hz `[MAV]` debug line (`DebugFeature::MAVLINK`) reports `viso:<count> dt:<ms>` so all of
this is diagnosable on serial alone: a frozen `viso` count means a gate is closed (link, speed
validity, or rolling in neutral/unknown), and a `dt` that never settles near 200 ms means the
baseline keeps being re-established by a flapping gate.

## Autopilot speed limit (`SPEED_MAX`)

The ESP32 has its own max-speed throttle limiter, driven by the hall wheel sensor. Its **km/h
ceiling** can come from the autopilot: the firmware polls ArduPilot's `SPEED_MAX` (m/s) with
`PARAM_REQUEST_READ` every `MAVLINK_PARAM_POLL_MS` (5 s, first request 1 s after the autopilot's
`HEARTBEAT` is seen) and also accepts an unsolicited `PARAM_VALUE`. A change made from Mission
Planner or MAVProxy therefore lands within about 5 s, with no reboot.

> **The parameter is never written and never persisted.** The ESP32 sends no `PARAM_SET` — it
> only ever reads. The received value lives in RAM and dies with the link; the ESP32's own stored
> ceiling (NVS `speed` / `lim_kmh`) is **never** overwritten by autopilot traffic, so the value in
> the web UI's "Maximum speed" box survives every `SPEED_MAX` change and every power cycle.

### Precedence

| Web `speed_limiter` toggle | `SPEED_MAX` state | Ceiling used | Telemetry `speed_limit_src` |
|---|---|---|---|
| OFF | anything | *none — the limiter does not act* | `"off"` |
| ON | fresh and > 0 | `SPEED_MAX × 3.6` km/h, clamped into 1–200 km/h | `"mavlink"` |
| ON | `0` ("not set" in ArduPilot's own reading) | the stored web value | `"local"` |
| ON | never received / rejected | the stored web value | `"local"` |
| ON | older than `MAVLINK_PARAM_STALE_MS` (16 s ≈ 3 polls) | the stored web value | `"local"` |
| ON | link down (no `HEARTBEAT` for 3 s) | the stored web value | `"local"` |

**The web toggle is the master switch.** The autopilot supplies the ceiling's *value*, never the
decision to limit — an operator at the vehicle can always disable the limiter without a ground
station, and no autopilot traffic can re-arm it.

A value outside 1–200 km/h is **clamped, not discarded**: silently reverting a deliberate crawl
setting to a stored 60 km/h ceiling is the failure direction that hurts.

### Rejected values

A `PARAM_VALUE` is ignored (and logged) when it is NaN, infinite, negative, or above
`MAVLINK_PARAM_SPEED_MAX_MS` (30 m/s, ArduPilot's own range bound); the previously accepted value
stays in force, so one corrupt frame cannot invalidate a good subscription. A `PARAM_VALUE` whose
sender system **or** component id is not the learned autopilot is dropped outright — a ground
station on the same wire (typically sysid 255) cannot move the vehicle's speed ceiling.

### What the limiter then does

The throttle ceiling is **tapered**, not stepped: 100 % at `limit − SPEED_LIMIT_TAPER_BAND_KMH`
(5 km/h), falling linearly to `SPEED_LIMIT_FLOOR_PCT` (10 %) at the limit, and holding that floor
above it. The ceiling itself is slew-limited to `SPEED_LIMIT_CEILING_SLEW_PCT_S` (200 %/s) so
engagement cannot snap the servo — the rate limit applies to the *ceiling*, so the driver's own
throttle movements are never slowed. There is no hard cut: removing all drive mid-corner is a
stability event, not a safety measure. An invalid speed reading still **fails open** (no clamp),
and the gear-change throttle boost is outside the limiter entirely.

The 1 Hz `[MAV]` debug line (`DebugFeature::MAVLINK`) reports `spdmax:<m/s> age:<ms> valid:<Y|N>`,
which separates "never received" (`nan`), "answered and fresh" (a 0 → 5000 ms age sawtooth) and
"received but stale/link down" (`valid:N`). The vehicle layer logs `[SPEED] Limit source: … @ …
km/h` on each change. `[SPEED] Limiter maximum set to …` comes only from a web save — if it ever
appears in response to autopilot traffic, something is writing NVS that must not.

## Fail-safe

- No `SERVO_OUTPUT_RAW` for `MAVLINK_CMD_TIMEOUT_MS` (500 ms) → commands invalid →
  vehicle enters fail-safe (center steering, idle throttle, NEUTRAL, parking brake).
- No autopilot `HEARTBEAT` for `MAVLINK_HEARTBEAT_TIMEOUT_MS` (3000 ms) → link down.
