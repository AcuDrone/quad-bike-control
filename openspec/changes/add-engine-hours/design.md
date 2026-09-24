## Context

`VehicleController::update()` already reads, once per iteration, the exact two facts an hour meter
needs:

```cpp
CANController::VehicleData canData = canController_.getVehicleData();
// canData.dataValid  — is the ECU answering at all?
// canData.engineRPM  — is the crank turning?
```

Both are discarded for this purpose today. Everything else in this change is about where to keep
the accumulated time, how to survive a power cut, and how to get it to the ground station without
adding a message.

Constraints that shaped the design:

- **CAN is the only witness to the engine.** There is no tachometer input, no oil-pressure switch
  and no alternator-sense line on the `Control_v0` board. `VehicleData.dataValid` is the sole
  statement that the RPM figure means anything: `CANController::update()` sets it true on every
  successful PID response, false when every PID has exhausted its retries, and false again on
  `isDataStale()`. So "CAN invalid" genuinely means "the engine state is unknown", not "the engine
  is off" — and those must be treated differently.
- **`ENGINE_RUNNING_RPM_THRESHOLD` is not the threshold this feature wants.** It is 1500 RPM and
  it exists to answer *safety* questions — may the transmission shift, may the starter engage, may
  the transmission state be restored. It is deliberately conservative and sits above this engine's
  idle. An hour meter built on it would silently refuse to count idling, which is a large fraction
  of what an hour meter is bought to measure.
- **Flash is the scarce resource, not RAM or CPU.** One `uint64_t`, one `uint32_t` and one add per
  loop iteration cost nothing. NVS writes cost flash endurance, so the write *schedule* is the only
  part of this design with a real budget.
- **`include/MavlinkInterface.h` must stay free of mavlink headers** — existing policy. The value
  reaches the transport as a plain `float` in `StateReport`, exactly as `odoKm` does.
- **The layering rule in `openspec/project.md`** (MAVLink input → command interpreter → vehicle
  systems) puts a counter that describes the *engine* in the vehicle layer, not in the transport
  and not in `CANController` (which is a protocol driver and owns no history).
- **One class per `.h`/`.cpp` pair**, Doxygen on the public API — project convention.

## Goals / Non-Goals

- Goals:
  - A total engine hour meter that only ever increases, counts idling, and is correct to the
    resolution of the CAN poll schedule.
  - A resettable TRIP hour meter beside it ("мотогодини місії"), counting in lockstep with the
    total and cleared by the SAME operator gesture that clears the trip distance.
  - Survive power loss with at most ten minutes of loss, and zero loss on a normal ignition-OFF
    shutdown.
  - Reach the GCS over the link and the message that already exist, at zero extra bytes.
  - Spend a negligible fraction of the NVS partition's write endurance.
  - Be unable to lie upward: no assumed engine state, no banked time, no replayed stall.
- Non-Goals:
  - A service-interval counter with its own alarms, due-dates or per-component buckets. The trip
    hours are a plain "since last reset" total, nothing more.
  - A SEPARATE reset for the trip hours: no second command, no second `param1` value, no web
    control. It rides the existing trip-distance reset or it does not exist.
  - Resetting the TOTAL by any means. It is an engine-lifetime counter.
  - A reset or seed control in the web portal, or anywhere else.
  - Back-dating either meter to the engine's real running time, or any import path.
  - Separate idle-hours / load-hours / PTO-hours buckets. One counter answers the service schedule;
    anything finer is a data-logging feature, not an hour meter.
  - Any change to `isEngineRunning()` or to the safety thresholds that depend on it.

## Decisions

### Decision: accumulate `millis()` deltas in `uint64_t` seconds + a `uint32_t` millisecond remainder

```cpp
void EngineHourMeter::update(bool engineTurning, uint32_t nowMs) {
    if (!started_) { started_ = true; lastUpdateMs_ = nowMs; return; }
    const uint32_t dtMs = nowMs - lastUpdateMs_;   // unsigned: wraps correctly
    lastUpdateMs_ = nowMs;
    if (!engineTurning) return;                    // discard, never bank
    if (dtMs > ENGINE_HOURS_MAX_DELTA_MS) return;  // implausible: a stall, not running time
    remainderMs_ += dtMs;
    if (remainderMs_ >= 1000) {
        engineSeconds_ += remainderMs_ / 1000;
        remainderMs_   %= 1000;
        dirty_ = true;
    }
}
```

Three properties are load-bearing:

- **Wall clock, not a tick count.** The loop rate is not fixed (CAN polling, NVS writes and the web
  server all perturb it), so counting iterations × an assumed period would drift with load. The
  measured delta cannot.
- **Exact seconds.** `uint64_t` seconds cannot overflow in any physical scenario (2⁶⁴ s ≈ 5.8×10¹¹
  years), so no wrap handling is needed anywhere — not in the accumulator, not in NVS, not on the
  wire. Integer addition means no drift.
- **The remainder is carried, not dropped.** At a 25–50 Hz loop rate each delta is 20–40 ms. Without
  a remainder, integer-second accumulation would discard *every* delta and the meter would never
  move; with a float accumulator it would eventually stop moving for the classic reason (at 1000 h
  in seconds, a float32 ULP is 0.25 s, and a 0.02 s increment adds nothing at all — the "odometer
  that stops" bug, one unit down).

*Alternatives considered:* **`float` hours accumulated directly** — the stall bug above, and it
arrives sooner in hours than in seconds; **`uint64_t` milliseconds** — exact and simpler (no
remainder), but it stores three digits of precision no consumer will ever read, and the NVS value
becomes harder to eyeball; **a `double`** — the ESP32-S3 has no hardware double and it buys nothing
over an integer; **counting loop iterations** — drifts with load, as above.

### Decision: `ENGINE_HOURS_MIN_RPM = 300`, a NEW constant

The meter counts while `dataValid && engineRPM >= ENGINE_HOURS_MIN_RPM`.

`ENGINE_RUNNING_RPM_THRESHOLD` (1500) is explicitly **not** reused. It is the gear-change and
crank-interlock threshold — `isEngineRunning()`, the transmission-state restore, and the "engine
already running, refuse to crank" check all hang off it — and it is chosen to be safely *above*
anything that could be mistaken for running. An hour meter wants the opposite bias: it must count
the engine idling in a yard, because idling is running time and wears the engine on the same
schedule. Sharing the constant would also couple two unrelated tuning decisions, so that anyone
raising the shift interlock silently changes the service records.

`rg -i idle` across the repository finds only *throttle-servo* idle (the calibrated servo endpoint,
`THROTTLE_DEFAULT_IDLE_US`) — nothing documents this engine's idle **RPM**, and the archived CAN
probe changes record no idle figure either. 300 RPM is therefore chosen on a physical argument
rather than a measured one: a healthy petrol engine idles at 800–1500 RPM, a starter cranks it at
roughly 200–300 RPM, and anything reading above ~300 means the crank is turning under its own power
or under the starter. Both are running time an hour meter should record — a few seconds of cranking
per start is orders of magnitude below the resolution anyone reads an hour meter at.

The threshold is far enough below idle that no idle droop, no cold-start dip and no low-RPM
lugging can drop the meter out of counting, and far enough above zero that a stationary engine with
the ignition on (`engineRPM == 0`, `dataValid == true` — the ECU answers on ACC) counts nothing.
That last case is the one that actually matters: **ignition on with the engine off must not tick**,
and it is CAN-valid, so the RPM test, not the validity test, is what stops it.

*Alternatives considered:* **reuse 1500** — does not count idling, and couples an hour meter to a
shift interlock; **use 0/any positive RPM** — a noisy or stale-but-valid zero would read as
running, and the ECU reports 0 legitimately with ignition on; **gate on the ignition relay state
instead of RPM** — that measures "key turned", not "engine running", and would count every minute
the operator sits with the ignition on and the engine stalled, which is the classic way an hour
meter loses its credibility.

### Decision: discard the interval while CAN is invalid; never bank it

While `dataValid` is false the engine state is unknown, and the elapsed time is **dropped**:
`lastUpdateMs_` still advances, so when CAN returns the meter does not receive the outage as a
lump. Two consequences are deliberate:

- During a genuine CAN outage with the engine running, the meter **under**-counts. That is the
  correct failure direction, and it is the same one the odometer chose for a hall-sensor dropout: a
  counter that keeps ticking on a dead bus hides the fault, and a service schedule that is
  pessimistic about remaining life is safer than one that is optimistic.
- The meter says nothing about CAN health itself. That is already reported: `EFI_STATUS.rpm` and
  its six CAN-gated siblings go `NaN` in the same tick, so a consumer looking at a flat hour meter
  alongside a field of `NaN`s can see exactly why it stopped.

### Decision: cap a single delta at `ENGINE_HOURS_MAX_DELTA_MS` (5 s)

The main loop is cooperative and non-blocking by design, so a delta of even 200 ms is already
unusual. Five seconds is chosen as an order of magnitude above anything the loop legitimately does
(the longest single blocking operation in the firmware is an NVS write) and still far below any
interval worth counting. A delta larger than that is a symptom — a watchdog-adjacent stall, a long
flash operation, a debugger halt — and the time it represents is not time anyone can attest the
engine was running.

It also handles the `millis()` wrap for free. `nowMs - lastUpdateMs_` on `uint32_t` is correct
across the 49.7-day wrap by modular arithmetic, so the cap is not strictly needed for that case;
but it is a cheap second line of defence against any future clock source that does not wrap
cleanly, and against a `lastUpdateMs_` left stale by a code path that forgets to call `update()`.

The cost of the cap is bounded and tiny: at most `ENGINE_HOURS_MAX_DELTA_MS` of real running time
is lost per stall event.

### Decision: `spark_dwell_time` ← engine hours, in the existing `EFI_STATUS`

`EFI_STATUS` is already packed and sent at 5 Hz from component 25 and already carries thirteen
live values; `spark_dwell_time` is currently transmitted as a literal `0.0f`. Adding one float to
an existing pack call costs **zero** extra bytes on a 115200 link shared with a 25 Hz command
stream.

This amends the *"repurpose only permanently-free fields"* rule, which currently names
`spark_dwell_time` among four reserved fields to leave unused "rather than repurposed" because they
"name quantities this ECU **could plausibly expose later** over OBD-II". The rule's test is whether
the named quantity is permanently unavailable, and `spark_dwell_time` is the one of the four that
passes it outright:

| Reserved field | Route onto this bus | Verdict |
|---|---|---|
| `ignition_timing` | **PID `0x0E`** — timing advance, standard Mode 01 | could appear later — stays reserved |
| `exhaust_gas_temperature` | **PID `0x78`** — EGT sensor bank 1, standard Mode 01 | could appear later — stays reserved |
| `injection_time` | **no PID of its own** — only *derivable*, from the fuel-trim PIDs (`0x06`/`0x07`) with engine load | **permanently free** (takes the TRIP hours — see the next decision) |
| `spark_dwell_time` | **none** — spark dwell has no standard Mode 01 PID at all | **permanently free** (takes the TOTAL hours) |

Dwell is a coil-charging interval the ECU computes internally and OBD-II never standardised a way
to ask for it. There is no PID to enable, no bitmap bit to light up, and no plausible firmware
change on either side that would make this ECU able to report it. That is a stronger justification
than the odometer's `barometric_pressure` had (which rested on a bench probe finding no
ambient-pressure PID answering *on this ECU*); this one rests on the protocol, not on one vehicle.

The value is **always valid**, never `NaN`. Unlike `rpm`, it is history: it does not stop being
true when the bus goes quiet, it merely stops growing. A consumer that sees a flat hour meter and a
field of `NaN`s is looking at a CAN outage; a consumer that sees a flat hour meter and live RPM
below idle is looking at a stopped engine. Neither needs an "unknown" encoding on this field.

After this change two reserved fields remain (`ignition_timing`, `exhaust_gas_temperature`) and the
documentation's "four reserved fields" paragraph becomes two.

*Alternatives considered:*

- **`NAMED_VALUE_FLOAT` (`ENG_HRS`)** — rejected for the same reason the gear value was moved out
  of it in `remap-efi-native-fields` and the same reason the odometer never used it: every
  `NAMED_VALUE_FLOAT` shares one message id (251) and is distinguished only by a 10-byte `name`
  field, so a name-agnostic store keeps only the last one received and the values appear to
  overwrite each other. It is the right vehicle for the steering VESC set (five values of one
  subsystem, dispatched on `(compid, name)` by a plugin written for them) and the wrong one for a
  single scalar that belongs with the engine data a consumer is already decoding by field position.
  It would also add one more message to the link for a value that fits free into one already there.
- **A second `EFI_STATUS` from a different component, or a custom message** — needs a dialect both
  ends agree on; the whole point of the `EFI_STATUS` approach is that it survives a stock ArduPilot
  and a stock MAVLink library.
- **`ONBOARD_COMPUTER_STATUS.uptime` / `SYS_STATUS`** — both mean something else (the *companion
  computer's* uptime, and the autopilot's own subsystem health), and a GCS renders them as such.
  Engine hours are not the ESP32's uptime; conflating them would be wrong the moment the engine is
  off and the controller is on, which is most of the time.
- **A new NVS-backed "hours" web command to seed the meter from the engine's real history** —
  rejected as out of scope and as a footgun: a seedable hour meter is a meter with a write path,
  and the first thing a write path attracts is a mistake. The meter reads "hours since this
  firmware", stated plainly in the documentation.

### Decision: a resettable TRIP hour meter, cleared by the EXISTING trip reset, carried in `injection_time`

`tripSeconds_` sits beside `engineSeconds_` in the same class. Three sub-decisions, each with an
alternative that was rejected.

**Why a SHARED reset, not a command of its own.** The odometer's TRIP and the hour meter's trip
describe **the same interval** — "since the operator last pressed reset" — in two different units.
A mission is one thing: how far it went and how long the engine ran are two readings of it, and an
operator who resets one and forgets the other is left with a pair that cannot be compared (an
average speed computed from them would simply be wrong). So the reset is **one gesture, one
command, one magic value**:

```cpp
if (mavlink_.consumeTripResetRequest()) {
    speedSensor_.resetTrip();
    engineHours_.resetTrip();   // TRIP hours only — the TOTAL hour meter is untouched
}
```

`EngineHourMeter::resetTrip()` has **exactly one call site**, that one. There is no new
`MAV_CMD_USER_*`, no new `param1` value beside `MAVLINK_CMD_TRIP_RESET_MAGIC`, no new
`WebPortal::WebCommand` and no new constant. `handleCommandLong()` is not touched at all — the
transport still latches one request, and the vehicle layer still performs it; the only change is
what "performing it" means, and it stays inside the vehicle layer where both counters live.

*Alternatives considered:* **a second magic `param1` (e.g. `2.0` = reset hours)** — two commands to
keep in step, two plugin buttons, and the failure mode is exactly the drift this design exists to
prevent: an operator who sends one and not the other. It also spends the `MAV_CMD_USER_1` parameter
space on a distinction nobody asked for. **A web reset button** — the trip reset is deliberately a
ground-station action (the operator drives from the GCS, not from the vehicle's WiFi), and adding
one here would give the hour meter a write path the portal does not have for the distance. **A
`param1` bitmask (`1` = km, `2` = hours, `3` = both)** — the same objection, plus it silently
redefines the existing magic value, breaking every script that already sends `1.0` expecting "reset
the trip".

**Why `injection_time`.** With `spark_dwell_time` taken by the total, three reserved fields remain,
and the rule is unchanged: repurpose only a field whose named quantity is *permanently* unavailable
on this bus. `ignition_timing` (PID `0x0E`) and `exhaust_gas_temperature` (PID `0x78`) are both
standard Mode 01 PIDs — this ECU could answer either one day, so both stay reserved. Injection time
has **no PID of its own**: OBD-II never standardised a request for it, and the only route to it is
*derivation* from the fuel-trim PIDs (`0x06`/`0x07`) plus engine load — an inference a consumer
would compute, never a field this firmware would receive and forward. That makes it free on the
same terms dwell is, one step weaker only in that the quantity is at least *computable*, which is
why it is the second choice and not the first.

Like `fuel_pressure` (the trip distance) it is **always valid, never `NaN`, and a `0` is a genuine
zero** — the operator has just reset it. The sentinel the MAVLink definition suggests for "unknown"
is deliberately not substituted, for the identical reason: reporting a non-zero figure right after a
reset is the worse lie. A trip reset is therefore visible on the wire as `fuel_pressure` **and**
`injection_time` falling to zero in the same `EFI_STATUS`, with `barometric_pressure` and
`spark_dwell_time` unchanged — a signature a consumer can check without knowing anything about
which command caused it.

**Why ONE shared remainder carry.** Both counters take the same `wholeSeconds` from the single
`remainderMs_`:

```cpp
remainderMs_ += dtMs;
if (remainderMs_ >= 1000) {
    const uint64_t wholeSeconds = remainderMs_ / 1000;
    engineSeconds_ += wholeSeconds;
    tripSeconds_   += wholeSeconds;
    remainderMs_   %= 1000;
    dirty_ = true;
}
```

A *second* remainder for the trip would round the same elapsed time twice and let the two drift
apart by a second here and there — small, but it would make "total − trip" stop being an exact
figure, and it would mean the two counters could disagree about whether the engine ran at all
during some interval. One carry makes the invariant structural rather than tested: after any
sequence of updates, `tripSeconds_` is exactly the number of counted seconds since the last
`resetTrip()`, and `engineSeconds_` is exactly that plus what came before. It also costs one
`uint64_t` of state and one add, against a second `uint32_t` and a second divide-and-modulo.

The same logic governs persistence: both keys are written by the **one** `persist()` call under the
**one** `dirty_` flag, on the same three triggers, and the write counts as successful only when
**both** `putULong64` calls returned non-zero. A half-written pair stays dirty and is retried, so
there is no shutdown that saves the total and loses the trip. `resetTrip()` additionally calls
`persist()` on the spot — the operator has just made a deliberate gesture and expects it to survive
a power cut, exactly as `SpeedSensor::resetTrip()` writes through.

*Alternatives considered:* **deriving the trip from a stored "total at last reset"** (trip = total −
mark) — one fewer counter and automatically consistent, but it makes the *total* load-bearing for
the trip reading, so an NVS erase or a total-only corruption takes both out, and it stores a value
that means nothing on its own in a dump. **A separate `TripHourMeter` class** — a second copy of the
carry, the clock latch, the write schedule and the NVS policy, to hold one extra `uint64_t`; the two
counters share every one of those, which is the argument for one class.

### Decision: hours on the wire, exact seconds on the device

The transmitted value is `engineSeconds_ / 3600.0f`. Precision is not what decides this: float32
carries a constant *relative* precision of 2⁻²⁴, so at 10 000 engine hours the ULP is ~0.001 h
(3.6 s) whatever unit is chosen — four orders of magnitude finer than anything a service schedule
cares about. Hours is chosen because it is the unit the service schedule, the dealer and the
operator all use, so the GCS field can be displayed verbatim with no divide and no ambiguity about
whether a large bare number is seconds or minutes.

The exact value lives in the `uint64_t` seconds counter on the device; the float is only ever a
*presentation* of it, and is never read back, re-accumulated, or used to reconstruct the counter.

### Decision: NVS writes every 600 s of accumulated growth, at ignition OFF, and on fail-safe entry

Three triggers, mirroring the odometer exactly, chosen so that the expensive one is rare and the
precise one is free:

| Trigger | Cost | What it buys |
|---|---|---|
| Meter grew ≥ `ENGINE_HOURS_NVS_WRITE_INTERVAL_S` (600 s = 10 min) | 6 writes/engine-hour | Bounds the loss from a crash or power cut to < 10 min |
| Ignition OFF transition (MAVLink path and web path) | ~1 write/journey | **Zero** loss on a normal shutdown, which is how the engine is stopped almost every time |
| Fail-safe entry | ~1 write/event | A fail-safe is a power-down in every respect that matters to the counter |

Note the interval is measured in **accumulated** seconds, not wall-clock seconds: a parked vehicle
with the controller powered writes nothing, because the meter is not growing. This is the direct
analogue of the odometer's "1 km of growth" rather than "every 10 minutes".

**Wear budget.** ESP32 NVS is a wear-levelled, log-structured store: rewriting a key does not erase
in place — it appends a new 32-byte entry and tombstones the old one, and a 4096-byte page is
erased only when it is compacted. One `uint64` key is one entry per write, so roughly 126 writes
fill a page and cost one erase. Against the ~100 000 erase-cycle endurance of the flash:

| Engine hours | NVS writes | Page erases | Fraction of endurance |
|---|---|---|---|
| 1 000 h | 6 000 | ~48 | 0.05 % |
| 10 000 h | 60 000 | ~480 | 0.48 % |
| 100 000 h | 600 000 | ~4 800 | 4.8 % |

A quad-bike engine is typically rebuilt somewhere between 1 000 and 2 000 hours, so the vehicle
reaches the end of several engines inside the first row. This is a cheaper budget than the
odometer's (one key rather than two, and a slower trigger), and both are dwarfed by the already
shipped transmission persistence, which writes on **every gear change**.

Writing only at ignition OFF was rejected for the same reason it was for the odometer: an ignition
cut, a brown-out on the 12 V line, or a watchdog reset would then lose the whole session, and those
are exactly the events the vehicle experiences.

*Alternatives considered:* write on every whole second (0.017 Hz sounds harmless but is 3 600
writes per engine-hour — 60× this design, and it destroys the partition in a few thousand hours);
write on a wall-clock timer (writes while parked, and the loss bound stops tracking the unit the
operator cares about); keep a RAM shadow and flush only from a shutdown hook (the ESP32 has no
reliable power-fail interrupt on this board — the same finding that shaped the odometer).

### Decision: a new NVS namespace `"engine"`, not a borrowed one

`"speed"` is owned by `SpeedSensor` and holds wheel calibration and distance; running time is
neither, and the class that owns it is not `SpeedSensor`. A namespace per owning class is the
pattern already used throughout (`"speed"`, `"boost"`, throttle calibration), and it keeps the
erase blast radius honest: clearing `"engine"` clears the hour meter and nothing else.

The key is `"hours_s"` — named for the unit it holds, so that a future reader of an NVS dump cannot
mistake seconds for hours. (NVS keys are capped at 15 characters, which `"hours_s"` is comfortably
within.)

### Decision: the meter lives in `VehicleController`, updated from `update()`

`VehicleController::update()` already fetches `CANController::VehicleData` once per iteration for
the transmission's safety checks. The meter is updated from that same snapshot, immediately after
it is read, so there is no second `getVehicleData()` call and no chance of the two consumers seeing
different data in one iteration.

`begin()` is called from a new `VehicleController::initEngineHourMeter()`, invoked from `setup()`
beside `initCAN()` — the existing pattern for a member that needs an explicit init step
(`initBoostRail()`, `initCAN()`). Putting the NVS load inside the constructor was rejected: the
controller is constructed as a global before `setup()` runs, and NVS is not ready then.

The three `persist()` calls sit beside the three existing `speedSensor_.persistDistance()` calls,
so the two counters can never disagree about what a shutdown is.

## Risks / Trade-offs

- **Under-count during a CAN outage.** Accepted deliberately (see above); the CAN-gated `NaN`
  fields already expose the condition, and a pessimistic service schedule is the safe error.
- **Up to 10 minutes lost to a brown-out or watchdog reset.** Accepted: the alternative is more
  flash wear, and the ignition-OFF write makes the normal case lossless. Ten minutes is ~0.17 h,
  below the precision any service schedule is written to.
- **The threshold is reasoned, not measured.** 300 RPM rests on a physical argument because nothing
  in the repository documents this engine's idle. → Mitigation: the bench task reads the actual
  idle RPM off the portal and confirms it is comfortably above 300; if this engine idles below
  400 RPM the constant should be revisited, and the task says so.
- **Cranking counts.** A few seconds per start are recorded as running time. Accepted: it is under
  0.002 h per start, and excluding it would need a second threshold and a state machine to
  distinguish cranking from a stumbling idle — complexity for an error far below the reading
  resolution.
- **A CAN bus that reports a plausible RPM while the engine is off would over-count.** There is no
  such failure mode on this ECU (it reports 0 with the ignition on and stops answering with the
  ignition off), and `dataValid` plus a 300 RPM floor are two independent gates against it.
- **`spark_dwell_time` and `injection_time` are no longer reserved fields**, so a future genuine
  dwell or injection-time value would have nowhere to go, and the reserved set is down to two.
  Accepted on the protocol argument above: OBD-II Mode 01 has no PID for either, so the situation
  cannot arise through the channel this firmware uses. The two fields that *could* arrive as PIDs
  (`0x0E` timing, `0x78` EGT) are exactly the two left alone.
- **The trip hours cannot be reset without also resetting the trip distance.** That is the point of
  the design, not an oversight — but an operator who wants only one of them cannot have it. Judged
  the right trade: the two readings describe one interval, and the alternative (a second command) is
  the drift this avoids.
- **A generic consumer will label the field "Spark dwell (ms)" and show hours.** Inherent to the
  repurposing and harmless — the value is visibly implausible as a dwell time (a real dwell is a
  few milliseconds and never monotonically increasing), which is the same property that makes the
  gear-in-`fuel_consumed` repurposing safe.

## Migration Plan

1. Flash firmware **and** `pio run -t uploadfs` (`data/index.html` changes).
2. On first boot both NVS keys are absent, so both meters read 0 and start from the moment of the
   flash. This is one-way — there is no seeding path. A vehicle upgrading from a build that had
   only the total keeps its total and starts the trip at zero: `"trip_s"` does not exist yet and
   reads as `0`.
3. The Mission Planner plugin update is **not** lockstep-critical: nothing it decodes today moves,
   so an un-updated plugin simply does not show the hours, and **the trip-reset command it already
   sends needs no change**. Against legacy firmware an updated plugin reads `spark_dwell_time` and
   `injection_time` as `0.0` and shows `0.00 h` — a visibly absent value, not a wrong one.
4. Rollback: flashing the previous firmware leaves `engine/hours_s` and `engine/trip_s` untouched in
   NVS (old firmware never reads them), so re-flashing forward restores both meters intact. Note
   that on the old firmware the trip reset stops clearing the hours, so the trip figure freezes
   rather than tracking the trip distance until the new firmware is back.

## Open Questions

- None blocking. The one item worth a bench reading is this engine's actual idle RPM (task 7.3): it
  is expected to be 800–1500, comfortably above the 300 RPM floor, and the floor only needs
  revisiting if it turns out to idle below ~400.
