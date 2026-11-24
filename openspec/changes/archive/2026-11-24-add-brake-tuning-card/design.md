# Design: add-brake-tuning-card

## Context
The brake position is an open-loop estimate: `currentBrakePosition_` (starts at 100 % at boot)
moves at `100 / BRAKE_FULL_TRAVEL_TIME × |speed| / 255` %/ms while the actuator is driven; the
only correction is the released endstop (In5), which, when the target is 0, triggers the
`BRAKE_SENSOR_OVERRUN_TIME` (1000 ms) overrun and then syncs the estimate to 0. Above 0 there is no
feedback at all, so the travel time is the whole calibration.

## Decisions
1. **Two settings only, travel time and hold drive.** These are the two vehicle-dependent numbers
   in the loop. `BRAKE_TOLERANCE` (1 %) and `BRAKE_SENSOR_OVERRUN_TIME` (1000 ms) stay constants:
   the first is a loop detail, the second only matters at the endstop and has not caused trouble.
2. **Ranges.** Travel 1000–3000 ms around the measured 1500 ms: covers a faster or a slower
   actuator on the other vehicle without allowing absurd values; step 50 ms in the UI. Hold
   0–100 on the 0–255 PWM scale: 30 works today; above ~100 the actuator is pushed hard while
   stalled against the cylinder for as long as the brake is held, heating the motor and the
   BTS7960 — so the upper bound is deliberately well below full drive. 0 is a valid value meaning
   "no hold push" (`stop()`, coast).
3. **Owner and storage.** `VehicleController` owns the brake estimate, so it owns the values and
   the NVS namespace `"brake"` (new; nothing else uses it). Keys `travel_ms` (uint16) and
   `hold_pwm` (uint8). Loaded in `begin()` with the other `VehicleController` NVS reads.
   Invalid stored values are logged and ignored, never rewritten (same rule as `open_slew`).
4. **Apply on change, no rescale.** A new travel time only changes the rate used from the next
   `updateBrakeControl()` iteration; the current estimate is kept as is. Rescaling the estimate
   would be equally wrong (we do not know the real position either way), and the next full
   release re-syncs it at the endstop. A new hold value is read every hold cycle, so it applies at
   once if the brake is being held.
5. **Pure law in `include/BrakeLaw.h`.** Only arithmetic and validation, testable natively:
   rate from travel time, range checks, or-default helpers for NVS load. The control flow of
   `updateBrakeControl()` is not moved into the law (it is tied to millis, the driver and the
   endstop), to keep this change small.
6. **Commands accepted from any input source.** They change settings, not the brake command,
   like `set_throttle_open_slew` / `set_steer_park_*`. `set_brake` itself keeps its web-control
   gate.
7. **`brake_pct` is the estimate, not the target.** The point is to watch the estimate while tuning:
   with the right travel time, `brake_pct` reaches 100 exactly when the actuator hits its end.
   The manual-control slider keeps showing the operator's command.
8. **`brake_released` is `isBrakeReleased()`.** It already returns false when the expander
   snapshot is invalid; the UI shows "ні" / "no" in that case too (the Board I/O health telemetry
   already reports an expander fault separately).
9. **Telemetry capacity.** Three new scalar keys (+ `brake_pct` already present) add 3 slots
   (≈ 48 B) to the worst case of ≈ 8.24 KB counted in 1.5.18; `StaticJsonDocument<9216>` keeps
   ≈ 0.9 KB of headroom. The capacity comment is updated, the size is not changed.
10. **UI placement.** The settings cards are one scrolling column; the brake card goes right after
    "Throttle Calibration" (next to the throttle and steering tuning cards). No reset button,
    matching the park card.
11. **Travel time is a slider, hold drive a number field (user decision 2026-11-06).** The travel
    time is a range input 1000–3000 ms, step 50, default 1500, with a live value label
    "N мс" / "N ms" beside it; it is tuned by eye against the panel's `brake_pct`, so a slider
    is quicker than typing. Wiring follows the existing sliders in `data/index.html`:
    - the label is updated on `input`, like the steering jog-speed slider (`steer-jog-speed` →
      `steer-jog-speed-val`, `oninput` sets the text only, nothing is sent while dragging);
    - the command is sent once, on `change` (release / keyboard commit), like the final send in
      the gear-definition slider (`initGearDefSlider()`, `gd-servo-slider` `change` handler);
      unlike that slider there is no rate-limited streaming during the drag, because every
      `set_brake_travel_ms` is an NVS write;
    - a "dragging" flag (set on `pointerdown` / `input`, cleared on `change` / `pointercancel`,
      as `gdDragging`) keeps telemetry from moving the thumb while the operator holds it.
    The hold drive keeps the number field (step 5, unchanged from the original design) and a Save button, because it is
    set once and its values are few.

## Risks
- A travel time set too long makes the firmware drive past the real end for longer before it
  believes it has arrived; if the actuator has no internal limit switch it stalls at full drive
  for that time (there is no current sense). The 3000 ms cap bounds this.
- A hold of 0 may let the brake bleed back on a vehicle whose hydraulics push the actuator back;
  this is visible on the bench and is the operator's choice.

## Follow-ups (out of scope)
- Closed-loop brake position (potentiometer actuator or BTS7960 current sense `IS` pins).
- Stall / overrun guard when extending (current or timeout based).
- Brake–throttle interlock (throttle forced to idle above a brake threshold), already described
  in specs but not implemented.
- Parking brake applied on start-up (spec scenario "Brake hold on startup").
- Re-syncing the estimate at 100 % from an extended endstop if one is ever fitted.
