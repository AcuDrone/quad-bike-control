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
| Liveness + fail-safe status | `HEARTBEAT` (`MAV_TYPE_GROUND_ROVER`) | 25 | — | 1 Hz |
| Engine RPM | `EFI_STATUS.rpm` | **1** | `efi_rpm` | 5 Hz |
| Coolant temp | `EFI_STATUS.cylinder_head_temperature` | **1** | `efi_cylinder_head_temperature` | 5 Hz |
| Gear | `EFI_STATUS.engine_load` | **1** | `efi_load` | 5 Hz |
| Gear (also) | `NAMED_VALUE_FLOAT` `GEAR` | 25 | `customFieldN` / `GEAR` in Tuning | 5 Hz |
| Gear / ignition / fail-safe transitions | `STATUSTEXT` | 25 | Messages tab | on change |

> **Why EFI_STATUS is sent as component 1:** Mission Planner only maps `EFI_STATUS` into its
> labeled `efi_*` fields from the **autopilot component** (sysid 1 / comp 1), not from a
> peripheral component. So engine telemetry is tagged comp 1 (the real autopilot sends no EFI,
> so there's no conflict), while the ESP32 keeps its own identity (comp 25) for `HEARTBEAT`/
> `STATUSTEXT`. Verified against BlueOS + MP 1.3.83: from comp 25, `efi_rpm`/`rpm1` stay 0.
>
> **Gear → `efi_load`:** gear has no standard MP field, so it rides in the otherwise-unused
> `EFI_STATUS.engine_load`, surfacing as the selectable/gaugeable `efi_load` value in MP. It is
> *also* sent as `NAMED_VALUE_FLOAT "GEAR"` (correctly labeled in the Tuning graph; `customFieldN`
> in the Status tab — MP does not show NVF names there or offer them on the Quick tab).

`GEAR` appears in Mission Planner's **Status tab** (and can be graphed). It is encoded by
physical gear-sequence position; while shifting it reads the **midpoint** between the two gears
in transit, so a multi-step change is an even 0.5 staircase:

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
