## Context

`SpeedSensor::update()` already computes, on every 200 ms sample window that saw edges, the exact
quantity an odometer needs:

```cpp
speedMs_ = (delta * distancePerPulseMm_) / (float)windowMs;
```

`delta × distancePerPulseMm_` **is** the distance travelled in that window, in millimetres. It is
discarded today. Everything else in this change is about where to keep that number, how to survive
a power cut, and how to get it to the ground station without adding a message.

Constraints that shaped the design:

- **The pulse train is the only truth.** The sensor deliberately reports a *decayed estimate*
  during silence (`speedMs_` falls toward 0 at `SPEED_MAX_PLAUSIBLE_DECEL_MS2`) and latches
  `suspicious_` when the decay is implausible. That estimate is a display value, not a
  measurement — integrating it would invent distance the wheel never turned.
- **Flash is the scarce resource, not RAM or CPU.** Two `uint64_t` and one multiply-add per sample
  window cost nothing. NVS writes cost flash endurance, so the write *schedule* is the only part of
  this design with a real budget.
- **Recalibration must not rewrite history.** `speed_cal_ppr` and `speed_cal_circ` are runtime
  commands (`specs/speed-sensor/spec.md` → *Runtime Speed Calibration*). Whatever is persisted must
  already be a physical distance.
- **`include/MavlinkInterface.h` must stay free of mavlink headers** — existing policy. Any new
  inbound handler takes decoded scalars, the `handleServoOutputRaw` / `handleParamValue` pattern.
- **This component shares the autopilot's system id.** `MAVLINK_SYSTEM_ID` is 1, the same as the
  Pixhawk; only `MAVLINK_COMPONENT_ID` (25, `MAV_COMP_ID_USER1`) distinguishes them. Any inbound
  command handling must therefore be *stricter* than a normal vehicle's, or the ESP32 will answer
  for commands meant for the autopilot.
- **The operator's cockpit is the GCS.** A control that exists only on the vehicle's WiFi portal is
  a control the operator cannot reach while driving — the lesson already recorded in
  `add-mavlink-speed-max-limit`.

## Goals / Non-Goals

- Goals:
  - A total odometer that only ever increases, and a trip meter the operator can clear from
    Mission Planner, both correct to the wheel-calibration error.
  - Survive power loss with at most one kilometre of loss, and zero loss on a normal ignition-OFF
    shutdown.
  - Reach the GCS over the link and the message that already exist.
  - Make "no speed reading" distinguishable from "stopped" on the wire, so the ground-station
    widget can render the difference.
  - Spend a negligible fraction of the NVS partition's write endurance.
- Non-Goals:
  - A trip B, a service-interval counter, engine hours, or per-session statistics.
  - Resetting ODO by any means. It is a vehicle-lifetime counter.
  - A reset control in the web portal.
  - Back-dating the odometer to the vehicle's real mileage, or any import/seed path.
  - Signed (net-displacement) distance. A car counts reverse as distance; so does this.
  - Sub-millimetre accounting. The accumulator is exact in millimetres; the *wheel calibration* is
    the accuracy limit, and it is percent-level.

## Decisions

### Decision: accumulate in `uint64_t` millimetres, inside the `delta > 0` branch

The accumulation goes immediately after the existing wrap guard, in the branch that already knows
`delta` is a plausible, non-zero pulse count:

```cpp
if (delta > SPEED_MAX_PULSES_PER_SAMPLE) {
    return;              // counter glitch — no speed, and no distance either
}
if (delta > 0) {
    const uint64_t stepMm = (uint64_t)llroundf(delta * distancePerPulseMm_);
    odoMm_  += stepMm;
    tripMm_ += stepMm;
    ...
}
```

`uint64_t` millimetres is chosen over every alternative:

- **Range**: 2⁶⁴ mm ≈ 1.8 × 10¹³ km. It cannot overflow in any physical scenario, so no wrap
  handling is needed anywhere — not in the accumulator, not in NVS, not on the wire.
- **Exactness**: integer addition, so there is no drift. A `float` accumulator loses its last
  increment entirely once the total is large: at 1 000 km expressed in metres, a float32 ULP is
  0.06 m, and a 0.03 m pulse step would add **nothing at all** — the classic "odometer that stops"
  bug.
- **Millimetres, not pulses**: pulses are only distance *through the current calibration*. Storing
  pulses and multiplying at read time means the day the operator corrects the tyre circumference,
  the whole recorded history silently changes. Storing millimetres freezes each kilometre at the
  calibration that measured it, which is what a physical odometer does.

`llroundf` rather than a truncating cast: `distancePerPulseMm_` is ~28.4 mm at the default
calibration (1990 mm / 70 ppr), and truncating every sample window would bias the odometer low by
up to 1 mm per window — about 0.02 % at a 5 Hz sample rate and highway speed, a systematic error
where rounding gives an unbiased one.

*Alternatives considered:* a `double` accumulator (exact enough, but the ESP32-S3 has no hardware
double and it buys nothing over an integer); accumulating a `float` metres total (the stall bug
above); keeping the odometer in `VehicleController` from `getSpeedMs() × dt` (integrates the
*decayed estimate*, i.e. invents distance during a sensor dropout, and double-counts the sample
window's own averaging).

### Decision: the decayed estimate and wrap-guard rejects contribute nothing

Distance is added **only** where pulses were actually counted. Two consequences are deliberate:

- During a genuine sensor dropout at speed the odometer **under**-counts. That is the correct
  failure direction: an odometer that keeps ticking on a disconnected sensor is worse than one that
  stops, because the fault is then invisible. `isSuspicious()` already tells the operator the
  reading is untrustworthy.
- A sample rejected as a counter glitch (`delta > SPEED_MAX_PULSES_PER_SAMPLE`, 20 000 pulses in
  200 ms ≈ 2 000 km/h) adds nothing, because the existing guard `return`s before the accumulation
  is reached. This is not extra code — it is a consequence of where the accumulation is placed, and
  the placement is load-bearing.

### Decision: NVS writes at 1 km of ODO growth, at ignition OFF, and on trip reset

Three triggers, chosen so that the expensive one is rare and the precise one is free:

| Trigger | Cost | What it buys |
|---|---|---|
| ODO grew ≥ `ODO_NVS_WRITE_INTERVAL_MM` (1 000 000 mm = 1 km) | 1 write/km | Bounds the loss from a *crash or power cut* to < 1 km |
| Ignition OFF transition | ~1 write/journey | **Zero** loss on a normal shutdown, which is how the vehicle is stopped almost every time |
| Accepted trip reset | 1 write/reset | The operator sees 0 and it stays 0 through a power cycle |

**Wear budget.** ESP32 NVS is a wear-levelled, log-structured store: rewriting a key does not erase
in place — it appends a new 32-byte entry and tombstones the old one, and a 4096-byte page is
erased only when it is compacted. Two `uint64` keys are two entries per write, so roughly 63 writes
fill a page and cost one erase. Against the ~100 000 erase-cycle endurance of the flash:

| Lifetime distance | NVS writes | Page erases | Fraction of endurance |
|---|---|---|---|
| 10 000 km | ~10 000 | ~160 | 0.16 % |
| 100 000 km | ~100 000 | ~1 600 | 1.6 % |
| 1 000 000 km | ~1 000 000 | ~16 000 | 16 % |

At a million kilometres the vehicle is long gone and the flash is still inside spec. For scale, the
already-shipped transmission persistence writes on **every gear change**, which on a working day
exceeds a year of odometer writes.

Holding the counters only in RAM and writing at ignition OFF alone was rejected: an ignition cut,
a brown-out on the 12 V line, or a watchdog reset would then lose the whole journey, and those are
exactly the events the vehicle experiences.

*Alternatives considered:* write every sample window (5 Hz — destroys the partition in days); write
on a wall-clock timer (writes while parked, and the loss bound is in minutes rather than in the
unit the operator cares about); keep a RAM shadow and flush only from a shutdown hook (the ESP32
has no reliable power-fail interrupt on this board).

### Decision: `barometric_pressure` ← ODO km, `fuel_pressure` ← TRIP km, in the existing `EFI_STATUS`

`EFI_STATUS` is already packed and sent at 5 Hz from component 25 and already carries eleven
values; both of these fields are currently transmitted as literal `0.0f`. Adding two floats to an
existing pack call costs **zero** extra bytes on a 115200 link shared with a 25 Hz command stream.

This amends the *"repurpose only permanently-free fields"* rule that `remap-efi-native-fields`
introduced, which named `barometric_pressure` and `fuel_pressure` among the fields to leave unused
"rather than repurposed". The rule's test is whether the named quantity is permanently unavailable
on this vehicle, and both pass it:

- **`barometric_pressure`** — the ECU has no barometric/ambient-pressure sensor. The
  `add-can-pid-probe` bench run found no ambient-pressure PID answering, and MAP (`0x0B`) is
  already carried in the field that names it.
- **`fuel_pressure`** — the same probe confirmed the fuel PIDs (`0x2F` fuel level, and no fuel-rail
  pressure PID) are absent from this ECU's supported-PID bitmaps. That is the identical
  justification already accepted for `fuel_consumed` and `fuel_flow`.

One quirk is accepted knowingly: the MAVLink definition of `fuel_pressure` says *"Zero in this
value means unknown"*. Immediately after a trip reset, TRIP **is** zero and would read as "unknown"
to a generic consumer. The consumer here is the repository's own Mission Planner plugin, which
decodes `EFI_STATUS` by field position and is updated with this change; the 0.0001 sentinel the
definition suggests is **not** used, because writing 0.1 m when the operator asked for zero is the
worse lie.

*Alternatives considered:*

- **`NAMED_VALUE_FLOAT`** (`ODO`, `TRIP`) — rejected for the same reason the gear value was moved
  out of it in `remap-efi-native-fields`: every `NAMED_VALUE_FLOAT` shares one message id and is
  distinguished only by a name field, so a name-agnostic store keeps only the last one received and
  the values appear to overwrite each other. It would also add two more messages to the link.
- **A new custom message** — needs a dialect both ends agree on, and the whole point of the
  `EFI_STATUS` approach is that it survives a stock ArduPilot and a stock MAVLink library.
- **`DISTANCE_SENSOR` / `ODOMETRY`** — both mean something else entirely (a rangefinder and a full
  6-DoF pose), and a GCS would render them as such.

### Decision: kilometres with a fraction on the wire

The transmitted value is `odo_mm / 1e6`. Precision is **not** what decides this, and the design
should not pretend otherwise: float32 carries a constant *relative* precision of 2⁻²⁴, so scaling
the unit scales the ULP with it and the absolute resolution is identical either way.

| Distance | as km, float32 ULP | as m, float32 ULP |
|---|---|---|
| 1 km | 0.12 mm | 0.12 mm |
| 100 km | 7.6 mm | 7.8 mm |
| 1 000 km | 6.1 cm | 6.1 cm |
| 10 000 km | 0.98 m | 0.98 m |
| 100 000 km | 7.8 m | 8.0 m |

What the table *does* settle is that float32 is good enough in either unit: even at 100 000 km the
quantisation is 7.8 m, or 0.008 % — two orders of magnitude below the few-percent error of the
wheel-circumference calibration that produced the number. The exact value lives in the `uint64_t`
millimetre accumulator on the device; the float is only ever a *presentation* of it, and is never
read back, re-accumulated, or used to reconstruct the counter.

Kilometres is then chosen because it is the unit the dashboard, the service schedule and the
operator all use, so the GCS field can be displayed verbatim with no divide and no ambiguity about
whether a large bare number is metres or millimetres.

### Decision: trip reset as `COMMAND_LONG` / `MAV_CMD_USER_1` with a magic `param1`

The reset needs to be operator-initiated from Mission Planner, over the link that already exists,
with a confirmation the operator can see. `COMMAND_LONG` is MAVLink's request/acknowledge
primitive, `MAV_CMD_USER_1` (31010) is reserved by the standard for exactly this, and
`COMMAND_ACK` gives the plugin something concrete to display.

`param1` carries a **magic value** (`MAVLINK_CMD_TRIP_RESET_MAGIC`, 1) rather than the command
being self-sufficient, so that a stray or malformed `MAV_CMD_USER_1` — from a mis-scripted GCS
button, a replayed log, or a future second user command — cannot silently destroy the operator's
trip reading. A wrong `param1` is answered `MAV_RESULT_DENIED` and logged, which is a diagnosable
outcome rather than a silent one. The comparison uses a tolerance
(`fabsf(param1 - MAGIC) <= MAVLINK_CMD_PARAM_EPSILON`), per the existing house rule against `==` on
floats; `NaN` fails that comparison and is therefore denied, which is correct.

**Addressing is deliberately strict.** The ESP32 shares system id 1 with the autopilot, so only
`target_system == MAVLINK_SYSTEM_ID && target_component == MAVLINK_COMPONENT_ID` is handled. A
broadcast (`target_component == 0`) is **ignored without an ACK**, because answering a broadcast
would put a second `COMMAND_ACK` on the wire for every command the GCS sends to the autopilot and
would confuse Mission Planner's command-confirmation logic. A command addressed to another
component is likewise ignored.

**The ACK goes back to `msg.sysid` / `msg.compid`**, not to the learned autopilot. The requester is
the ground station (typically 255/190), not the Pixhawk, and ArduPilot forwards it on the route it
learned from this component's own outbound `HEARTBEAT` / `EFI_STATUS` traffic — the same routing
that already delivers this component's telemetry to the GCS. No route needs to be configured and
none is assumed beyond "traffic has flowed in both directions", which the 5 Hz report guarantees.

*Alternatives considered:*

- **An autopilot parameter the ESP32 watches** (a `TRIP_RESET` the GCS sets to 1) — rejected: the
  ESP32 has no parameter server of its own, so the parameter would have to live on the *autopilot*
  and be polled, giving up to 5 s of latency, no acknowledgement, and no way to clear the flag
  without writing an autopilot parameter (which this firmware is forbidden to do — see
  *Autopilot Maximum-Speed Parameter Subscription*).
- **Implementing a real parameter server on component 25** (`PARAM_REQUEST_LIST` / `PARAM_SET`) —
  a large, stateful protocol surface and a parameter download competing with the 25 Hz command
  stream, for one write-only button.
- **A reset button in the web portal** — rejected by the operator: the portal is reachable only
  from the vehicle's own WiFi, and the operator is at the ground station. The portal shows the
  counters and offers no control over them.
- **A servo channel as a reset switch** — a level-triggered channel has no edge semantics, no
  acknowledgement, and would consume a scarce RC channel.

### Decision: `VFR_HUD.groundspeed` reports an invalid reading as `NaN`, not `0`

The whole reason `SpeedSensor` separates `getSpeedMs()` from `isValid()` is that "stopped" and
"disconnected" look identical at a passive pulse sensor. `VFR_HUD` threw that distinction away on
the wire: `src/MavlinkInterface.cpp:478-488` collapses `!speedValid` to `0.0f`, so a ground station
sees the same zero for a parked vehicle and for a severed hall lead. The field becomes
`state.speedValid ? state.speedMs : NAN` — a valid `0.0` stays `0.0`, and only the unknown case is
`NaN`. The `> 0.0f` guard in the current expression goes with it; it was only there to keep a
negative out of the field, and `speedMs_` is already clamped at zero by the decay path.

This is a *contract* change, not a formatting one, so it belongs in the spec: the ground-station
widget now reads this field from component 25 directly (companion change
`add-odometer-trip-readout` in the WindowsHelper repository) and renders "--" for `NaN`, which it
cannot do from a zero. MAVLink permits `NaN` in a float field to mean "unknown" — the same
convention this firmware already relies on for `EFI_STATUS.fuel_flow` and the CAN-gated fields.

Blast radius is deliberately small. Mission Planner does not fold component-25 packets into its own
vehicle state, so its HUD and `cs.*` are untouched; `VFR_HUD.throttle` keeps its
measured-with-commanded-fallback behaviour, because a `uint16_t` percentage has no `NaN` encoding;
and `VISION_POSITION_DELTA` is untouched, because it is a *fusable measurement* rather than a
display value and already goes silent on an invalid reading — sending `NaN` into the EKF would be a
very different and much worse idea.

*Alternatives considered:* **keep `0`** — rejected, it is the exact ambiguity the change exists to
remove, and it fails in the misleading direction (a dead sensor reads as a stationary vehicle);
**suppress the whole `VFR_HUD` message for that tick** (the other branch the old spec permitted) —
rejected, it would also blank the `throttle` field, which is still perfectly valid, and a consumer
cannot tell a suppressed message from a dropped one; **a negative sentinel such as `-1`** —
rejected, it is a number a graph will happily plot and a naive consumer will happily average.

### Decision: the transport latches the request; the vehicle layer performs it

`MavlinkInterface` decodes the command, validates it, sends the `COMMAND_ACK` and sets a pending
flag; `VehicleController::update()` consumes the flag and calls `speedSensor_.resetTrip()`. The
layering rule in `openspec/project.md` (MAVLink input → command interpreter → vehicle systems) and
the "no mavlink headers in the public header" policy both forbid `MavlinkInterface` holding a
`SpeedSensor&`.

`MAV_RESULT_ACCEPTED` is sent on decode, before the zeroing happens. That is the correct semantic —
"accepted for execution" — and the gap is under one control iteration (the loop runs at ≥ 25 Hz, so
< 40 ms); the bench gate is 200 ms end-to-end, which includes the 5 Hz report that carries the new
value back. Deferring the ACK until after the NVS write was considered and rejected: it would add a
second piece of cross-layer state for no operator-visible benefit, and an NVS write failure is not
something the operator can act on mid-drive (it is logged, and the RAM value is zero regardless).

## Risks / Trade-offs

- **A wrong wheel calibration makes a wrong odometer, permanently.** Distance is frozen in
  millimetres at the calibration in force when it was driven, so a kilometre recorded under a bad
  circumference stays wrong. → Mitigation: the bench task measures a known 100 m *before* any
  distance worth keeping accumulates; correcting the calibration early costs nothing.
- **Under-count during a sensor dropout.** Accepted deliberately (see above); `isSuspicious()` and
  `speed_valid` already expose the condition.
- **Up to 1 km lost to a brown-out or watchdog reset.** Accepted: the alternative is more flash
  wear, and the ignition-OFF write makes the normal case lossless.
- **`fuel_pressure == 0` reads as "unknown" to a generic consumer** immediately after a reset.
  Accepted (see above); the repository's own plugin decodes by field position.
- **The ESP32 now answers inbound commands on a system id it shares with the autopilot.** → The
  broadcast and foreign-component cases are ignored *without* an ACK precisely so that no extra
  `COMMAND_ACK` can ever appear on the wire for a command meant for the Pixhawk. The bench task
  checks a normal Mission Planner session (arm/disarm, mode changes) produces no ESP32 ACKs.
- **A consumer that reads component-25 `VFR_HUD.groundspeed` and does not expect `NaN`** could
  render "NaN km/h" or propagate it into an average. → The only such consumer is the repository's
  own Mission Planner widget, updated in lockstep by `add-odometer-trip-readout`; Mission Planner
  itself does not fold component-25 packets into `cs.*`, and the bench task checks the stock HUD is
  unaffected while the hall lead is out.
- **A GCS could spam `MAV_CMD_USER_1`.** Each accepted reset is one NVS write; a runaway script
  could burn writes. → The realistic exposure is an operator pressing a button, and each press is
  logged; no rate limiter is added for a hazard that does not exist in the operating model.

## Migration Plan

1. Flash firmware **and** `pio run -t uploadfs` (`data/index.html` changes).
2. On first boot both NVS keys are absent, so ODO and TRIP read 0 and the counters start from the
   moment of the flash. This is one-way — there is no seeding path.
3. The Mission Planner plugin update is **not** lockstep-critical for the counters: nothing it
   decodes today moves, so an un-updated plugin simply does not show them and cannot send a reset.
   The one thing that *does* change under an old consumer is component-25 `VFR_HUD.groundspeed`,
   which now reads `NaN` while the hall sensor is invalid instead of `0`; deploy
   `add-odometer-trip-readout` with this firmware so the widget renders it as "--".
4. Rollback: flashing the previous firmware leaves `odo_mm` / `trip_mm` untouched in NVS (old
   firmware never reads them), so re-flashing forward restores the counters intact.

## Open Questions

- None blocking. The bench tasks in `tasks.md` are the remaining verification; the operator's
  100 m measured-distance run is what pins the calibration and therefore the odometer's accuracy.
