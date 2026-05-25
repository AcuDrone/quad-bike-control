---
id: refactor-transmission-servo
title: "Refactor TransmissionController: Replace Linear Actuator with PWM Servo"
type: refactor
status: proposed
created: 2026-05-24
author: Sergey Zalozniy
---

## Summary

Replace the BTS7960 linear actuator + PCNT encoder combo in `TransmissionController` with a
standard PWM servo (signal pin GPIO 9, pulse range 800–2200 µs). Gear positions are stored
and exposed as float percentages (0.0–100.0 with one decimal digit) instead of raw encoder
counts. Physical gear selector hall-effect switches are retained for gear confirmation and
safety. Overshoot-and-return motion profile is preserved for mechanical detent engagement.
Auto-home is removed entirely (servo knows its position by construction).

## Motivation

The current BTS7960 linear actuator requires an external PCNT encoder for position feedback
and a complex closed-loop controller with stall detection and auto-homing on every cold start.
A PWM servo simplifies this dramatically:

- No encoder needed — servo position is commanded directly.
- No auto-home sequence — servo moves to absolute position on any power cycle.
- No stall detection — servo holds any position indefinitely under power.
- Float-percent positions are hardware-agnostic and easier to calibrate via web UI.

## Scope

| Area | Change |
|---|---|
| `include/Constants.h` | Remove actuator/encoder pin constants; add servo pin, channel, µs range, default gear percentages |
| `include/TransmissionController.h` | Drop BTS7960 inheritance; own `ServoController`; float positions; updated API |
| `src/TransmissionController.cpp` | Full rewrite of position control; time-based overshoot; float NVS keys |
| `src/main.cpp` | Remove encoder + BTS7960 init; add servo init |
| `include/VehicleController.h` | Remove auto-home state members and method |
| `src/VehicleController.cpp` | Remove `updateAutoHome()`, auto-home command handler, and guard blocks |
| `include/WebPortal.h` | Gear default telemetry fields change from `int32_t` to `float` |
| `src/WebPortal.cpp` | Serialise float gear positions with one decimal |
| `src/TelemetryManager.cpp` | No type change needed (auto-deduced from `getGearPosition()` return type) |
| `data/index.html` | Gear position inputs accept `0.0`–`100.0`; display with one decimal |

## Out of Scope

- Brake actuator (still uses BTS7960 + no encoder).
- Steering actuator (unchanged).
- Physical gear selector switch wiring (retained as-is).
- Throttle boost PID — logic unchanged; only trigger mechanism (`needsThrottleBoost()`) adapts.

## Key Decisions

See `design.md` for full rationale on:
- Why `TransmissionController` owns `ServoController` rather than inheriting.
- Time-based overshoot phase transition.
- NVS key migration strategy.
- How `needsThrottleBoost()` is redefined without encoder position.
