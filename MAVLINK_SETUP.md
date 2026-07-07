# MAVLink Vehicle Interface — Setup

The ESP32 talks to the Pixhawk 2.4.8 over a single MAVLink 2 serial link
(replacing the former SBUS-Out connection).

## Wiring

| Pixhawk TELEM2 | ESP32 (UART1) |
|----------------|---------------|
| TX             | GPIO 8  (`PIN_MAVLINK_RX`) |
| RX             | GPIO 15 (`PIN_MAVLINK_TX`) |
| GND            | GND |

Non-inverted, 8N1, **115200 baud**. No inverter circuit (unlike SBUS).

> ⚠ Do **not** use GPIO 9 — it is `PIN_TRANS_SERVO` (transmission servo).

## ArduPilot parameters (TELEM2)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SERIAL2_PROTOCOL` | `2` (MAVLink2) | TELEM2 speaks MAVLink 2 |
| `SERIAL2_BAUD` | `115` (115200) | Match the ESP32 link baud |

The ESP32 requests `SERVO_OUTPUT_RAW` at 50 Hz on connect via
`MAV_CMD_SET_MESSAGE_INTERVAL`, so no manual stream-rate parameter is required.
(If the autopilot ignores the request, set the relevant `SRx_*` stream rate so
`SERVO_OUTPUT_RAW` streams at ~50 Hz.)

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

(Channel 5 is intentionally unused.)

## Vehicle state reported back to the autopilot

Sourced from the CAN bus (`CANController::VehicleData`):

| Data | MAVLink message | Component | MP field | Rate |
|------|-----------------|-----------|----------|------|
| Data | EFI_STATUS field | Component | Rate |
|------|------------------|-----------|------|
| Liveness + fail-safe status | `HEARTBEAT` (`MAV_TYPE_GROUND_ROVER`) | 25 | 1 Hz |
| Gear | `engine_load` | 25 | 5 Hz |
| Engine RPM | `rpm` | 25 | 5 Hz¹ |
| Coolant temp | `cylinder_head_temperature` | 25 | 5 Hz¹ |
| Gear / ignition / fail-safe transitions | `STATUSTEXT` | 25 | on change |

¹ `rpm` and `cylinder_head_temperature` are sent as **NaN** while CAN data is invalid
(`!canValid`), so the consumer shows "--" instead of misleading zeros. The `engine_load`
(gear) field is always valid (sensorless, time-based).

> **Why one `EFI_STATUS`, not per-value `NAMED_VALUE_FLOAT`:** all `NAMED_VALUE_FLOAT` messages
> share a single message id and are distinguished only by a `name` field, so any name-agnostic
> store (MP's MAVLink Inspector, the mavlink2rest cache, unmapped `customField`s) keeps only the
> last one received — the three values appear to override each other. Packing gear/rpm/coolant
> into **distinct fields of one `EFI_STATUS`** eliminates that collision entirely. Mission Planner
> won't surface a *peripheral's* EFI in its native `efi_*` fields (those map only the autopilot,
> comp 1), but the raw packet **is** delivered, so the QuadBike MP plugin reads it cleanly via a
> packet subscription. Verified against BlueOS + MP 1.3.83.

`engine_load` carries the gear, encoded by physical gear-sequence position; while shifting it
reads the **midpoint** between the two gears in transit, so a multi-step change is an even 0.5
staircase:

| Gear | Settled value | Moving toward it (midpoint) |
|------|---------------|------------------------------|
| REVERSE | −1.0 | (from N) −0.5 |
| NEUTRAL | 0.0 | (from R/H) −0.5 / 0.5 |
| HIGH | 1.0 | (from N/L) 0.5 / 1.5 |
| LOW | 2.0 | (from H) 1.5 |

A whole number = settled (or briefly dwelling) at that gear; a `.5` value = the servo is moving
between the two adjacent gears. Example R→L: `-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0` as the
box steps R→N→H→L.

> `GEAR` is the controller's **assumed/commanded** gear — the transmission is sensorless and
> time-based, so the value reflects intent, not a measured position. Uses `NAMED_VALUE_FLOAT`
> (not the editable Params list) because gear is a live value.

The ESP32 uses system id `1`, component id `25` (`MAV_COMP_ID_USER1`).

> Vehicle speed and oil temperature are **not** reported — the ECU does not provide them
> over CAN (they always read 0). ArduPilot supplies its own `VFR_HUD` groundspeed from GPS.

## Fail-safe

- No `SERVO_OUTPUT_RAW` for `MAVLINK_CMD_TIMEOUT_MS` (500 ms) → commands invalid →
  vehicle enters fail-safe (center steering, idle throttle, NEUTRAL, parking brake).
- No autopilot `HEARTBEAT` for `MAVLINK_HEARTBEAT_TIMEOUT_MS` (3000 ms) → link down.
