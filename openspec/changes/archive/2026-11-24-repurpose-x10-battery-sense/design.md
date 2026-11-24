## Context
X10 on the control board is the Buck-Boost connector: pin 1 = enable (GPIO46), pin 2 = GND,
pin 3 = rail sense through 200K/24K (÷9.33) into GPIO9 (ADC1). aztec has no boost board (the
steering VESC runs from a 7s LiFePO4 pack). The MT05 ECU never answers OBD PID 0x42, so the
"ECU voltage" telemetry, the attention line and `EFI_STATUS.ignition_voltage` have had no source.

## Goals / Non-Goals
- Goals: a real 12 V battery reading, user-calibratable against a multimeter; reuse the existing
  `ecu_voltage` / `ecu_voltage_low` / `ignition_voltage` consumers.
- Non-Goals: no new MAVLink message, no build macro (branch-only like the rest of aztec), no
  automatic load shedding on a low battery, no change on `master` (it keeps the boost semantics).

## Decisions
- **Pure header** `BatteryVoltage.h` with a `BatParams` struct fed from `Constants.h`, so the native
  tests compile without Arduino (same pattern as `SteerPositionLaw.h`).
- **Scaling**: `raw = Vadc × 9.33`, `V = raw × cal`. 12.6 V → ~1.35 V at the pin; 11 dB attenuation.
- **Present**: raw > 5 V; below that `bat_v_valid` = false, no warning, MAVLink NaN.
- **Low flag**: set < 11.8 V, cleared > 12.1 V (hysteresis against ADC noise / load ripple).
  Suppressed while the starter relay output bit is set (crank or relay test) and 2 s after, because
  a healthy battery sags well below 11.8 V while cranking. The starter state is read from the
  relay port shadow (`RelayController::isStarterEnergized()`), sampled every loop so a short crank
  is not missed.
- **Calibration**: `cal = measured / raw` using the CURRENT averaged raw value; accepted when measured
  ∈ [8, 16] V, raw > 5 V and cal ∈ [0.80, 1.20] (a wider factor means a typo or a wiring fault).
  Persist first (NVS `bat12`/`cal`), then apply — same pattern as `set_speed_limit_floor`. Boot
  load: absent / NaN / out of range → 1.0, never rewritten. The web command uses the existing
  `{"cmd":"cal_bat_v","value":<V>}` shape parsed by `WebPortal` (the parser reads `value`).
- **Telemetry**: `ecu_voltage` keeps its name (the UI and attention box already use it) but moves out
  of the `can_status == "connected"` block. `bat_v_raw`, `bat_v_cal`, `bat_v_valid` added;
  `rail_24v`, `boost_on`, `rail_low` removed (net +2 scalar keys, well within the 8 KiB document).
- **CAN**: PID 0x42 leaves the poll table (6 PIDs; the stagger now offsets indices 4–5), stays in
  the probe candidate list so it can be re-checked.
- **`set_boost`**: still dispatched (an old cached page cannot produce "unknown command"), changes
  nothing, replies "Boost rail not fitted".
- **Version 1.5.8** (patch, per the decision made with the user; the wiring is additive and nothing
  needs recalibrating).

## Overlap with active changes
- `move-aztec-vesc-to-can` MODIFIES `vehicle-systems` "24V Boost Rail Control and Monitoring"
  (which this change REMOVES) and `web-telemetry` "Steering VESC Input Voltage Telemetry" and
  `vehicle-actuators` "Hardware Configuration" (which this change also MODIFIES).
- `move-can-rx-to-interrupt` MODIFIES `can-controller` "CAN Controller OBD-II Data Reading" (this
  change also MODIFIES it: seven → six PIDs).
- These deltas are written against the current specs; whichever change archives second must
  rebase its delta onto the other's result (drop the boost-rail delta in `move-aztec-vesc-to-can`,
  carry the six-PID table into `move-can-rx-to-interrupt`).

## Risks / Trade-offs
- The divider is high-impedance and ADC1 is noisy → 8-sample average and hysteresis; the user
  calibration absorbs divider tolerance and ADC gain error.
- A fuse is recommended in the X10 sense lead (the divider draws ~56 µA at 12.6 V, so the lead can be
  thin).
- `MAVLINK_SETUP.md` still describes `ignition_voltage` as PID 0x42 and VESC_V as the boost rail; it is
  being edited by the user and is left for them to update.

## Migration Plan
Flash 1.5.8 firmware AND the LittleFS image (index.html). Wire battery (+) → X10 pin 3. Open the
ignition card, measure the battery, type the value, press Calibrate. Rollback: flash 1.5.7; the
`bat12` NVS key is ignored by older firmware.
