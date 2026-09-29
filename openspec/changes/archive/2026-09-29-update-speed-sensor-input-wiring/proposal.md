# Change: Bring the speed-sensor spec in line with the confirmed opto-isolated input stage

## Why
`openspec/specs/speed-sensor/spec.md` still describes the input as it was *guessed* when
`add-hall-speed-sensor` was written: the sensor signal reaching GPIO 17 **directly**, no
opto-isolated stage, no polarity assumption, and the wiring explicitly "to be confirmed on the
bench". It also carries a scenario demanding that a 12V sensor "must be level-shifted" before the
board is powered.

The bench confirmed the wiring on **2026-09-29** and the firmware was updated to match
(commit `30c28af`: the `PIN_SPEED_SENSOR` block in `include/Constants.h`, the channel
configuration in `src/SpeedSensor.cpp`, and `GPIO_PINOUT_S3.md`). The real input stage is a
**PC817 opto-coupler**: the 12V toothed-ring pickup drives its LED through a 1 kΩ series resistor,
and its phototransistor collector goes to GPIO 17 with the emitter on GND. That stage both
**level-shifts** (nothing 12V ever reaches the pin, so the spec's level-shifting scenario is
answered by the hardware, not by a pending action) and **inverts** (the signal at the pin is
**active LOW**, and the firmware deliberately counts the **falling** edge — the start of the active
phase). An **external 1 kΩ pull-up from GPIO 17 to 3.3V is mandatory**: the first build relied on
the ESP32-S3 internal pull-up (~45 kΩ) alone, which leaves a high-impedance node between pulses
that picked up ignition-coil interference as phantom pulses and tripped the gear interlock. With
the 1 kΩ fitted the noise counters read 0 with the engine idling.

The same spec is also behind the noise hardening already merged in commit `49ba19c`: the PCNT
glitch filter is now **12.5 µs** (1000 APB cycles, the ESP32-S3 hardware maximum) rather than the
original 1 µs, the per-window wrap guard is **240 pulses per 200 ms** rather than 20000, and a set
of noise counters is exposed to the bench. None of that appears in the spec today.

Leaving the spec as it is means the written truth actively contradicts the hardware on the vehicle
and would mislead the next person into removing the pull-up or "fixing" the inverted polarity.

## What Changes
- **REMOVED** `Hall-Effect Speed Sensor Hardware Interface`, **ADDED**
  `Opto-Isolated Hall Speed Sensor Input Stage` in its place. Removed and replaced rather than
  modified because both of its scenarios are wrong at the level of their *names*
  ("Signal reaches the GPIO directly", "A 12V sensor must be level-shifted") — a MODIFIED
  requirement has to carry every existing scenario name forward, and neither of those should
  survive. The replacement states the PC817 opto between the 12V pickup and GPIO 17, the
  active-LOW signal at the pin, the falling counted edge, the mandatory external 1 kΩ pull-up to
  3.3V and its bench justification, and that the wiring is confirmed rather than pending. It
  carries forward the two facts from the old requirement that are still true: GPIO 17 is the pin
  freed when the BTS7960 steering driver was removed, and GPIO 18 stays free/reserved.
- **MODIFIED** `Hall Pulse Counting via PCNT` — the counted edge is the **falling** one (a
  documentation choice, not a correctness one, since the pulse rate is identical on either edge),
  the glitch filter is pinned to the hardware maximum against coil ringing, and the per-window
  pulse ceiling is stated as the physical ceiling it is derived from rather than a loose
  wrap guard.
- **ADDED** `Speed Sensor Noise Diagnostics` — the raw per-window pulse count, the stray-pulse
  counter, the rejected windows/pulses counters, their web telemetry fields
  (`speed_raw_pulses` / `speed_stray_pulses` / `speed_rej_windows`) and the rate-limited
  `[SPEED] window:` VEHICLE debug line. These are **counters only**: they never feed the speed or
  distance maths.

## Impact
- Affected specs: `speed-sensor` (1 MODIFIED, 2 ADDED, 1 REMOVED).
- Affected code: **none — documentation only.** The firmware, the constants and `GPIO_PINOUT_S3.md`
  already describe the confirmed hardware (commits `49ba19c` and `30c28af`); this change makes the
  spec agree with them, it does not ask for a single line of `src/` or `include/` to move.
- No behaviour change, no reflash, no field migration.

## Out of Scope
- The calibration, validity/timeout, telemetry, odometer and persistence requirements of
  `speed-sensor` are untouched.
- The `web-telemetry` and `web-control` capabilities are untouched: the three noise fields are
  described here as the diagnostics surface of the sensor that produces them, which is where the
  counters live.
- No second hardening layer (software debounce, a minimum-plausible-pulse-interval gate) is
  proposed here. The 1 kΩ pull-up made the counters read 0 at idle; anything further waits for
  evidence from the field.
