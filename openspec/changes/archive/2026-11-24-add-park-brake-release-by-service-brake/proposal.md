# Change: Aztec parking brake — released by pressing the service brake (no protection)

> **RISK — owner decision 2026-11-17: "Без захисту" (no protection).**
> Any new service-brake press releases the parking brake. There is **no** guard against the
> automatic fail-safe brake. The owner was shown the hazard and two safer alternatives and accepted
> this consequence:
>
> 1. the RC link is lost;
> 2. the BlueOS Copilot extension brakes on its own (`RC_CHANNELS_OVERRIDE` ch2 = `brake_pwm`,
>    default 1400 µs ≈ 25 %), about 2.5–5 s after the loss;
> 3. that brake is a new press, so **the parking brake releases**;
> 4. after `brake_hold_s` (default 15 s) the Copilot lets the service brake go;
> 5. **the buggy is then held by nothing** — on a slope or a trailer ramp it rolls.
>
> Offered and declined: a 70 % single-press threshold (the Copilot brakes at 25 %); a double press;
> a web switch "release by service brake" (default OFF); a "Copilot braking" flag sent by the
> Copilot. A guard can be added later without redesign (design.md, "Risks").

## Why
The owner (2026-11-17): "the parking brake must be released by the brake. It's needed for unloading
from a trailer, because at 10 % throttle it will simply shoot out." Today the parking brake is
released only in two ways:
- **Drive-away.** Gear L/H/R, ignition on, and the operator's throttle rising above 10 %
  (`PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT`). The throttle is held at idle until `thr_pass` % of the
  run has elapsed.
- **An explicit Release** from the web card or the GCS (`MAV_CMD_USER_2` handbrake).

On a trailer ramp the drive-away release puts throttle into an 800 cc buggy as the brake lets go.
The operator wants to hold the vehicle on the **service brake**, let the parking brake go, then ease
off the service brake and roll down under control, with no throttle at all.

## What Changes
- **New release trigger (aztec only).** While the parking brake is `APPLIED` or `UNKNOWN` (UNKNOWN is
  treated as applied, as everywhere else), a **new service-brake press** starts the normal release
  run (`parkBrake_.request(RELEASE, OK)`, the drive-away path).
  - **New press** = the commanded service-brake target (`currentBrakeTarget_`, written only by
    `applyBrake()`, which turns anything below 5 % into 0) goes from 0 to non-zero. Every source that
    goes through `applyBrake()` counts: the RC handset and the GCS joystick (servo channel 2 below
    centre, input source MAVLINK) and the web brake command (`set_brake`, input source WEB).
  - **Rising edge, not level.** A brake already held when the parking brake becomes APPLIED does not
    release it; the target has to go back to 0 and then rise again. A target frozen under the
    firmware's own FAILSAFE source is not a press. An input-source change (MAVLINK ↔ WEB takeover,
    FAILSAFE ↔ either) produces no edge and cancels a pending hand-over.
  - **Hand-over without rolling (assumption A1, owner can overrule).** The run starts only when the
    service-brake position estimate has reached the commanded target (within `BRAKE_TOLERANCE`,
    1 %). If the command drops to 0 first, nothing happens and a new press is needed. From a
    released service brake the delay is about (target − 1 %) × `brake`/`travel_ms`:

    | `travel_ms` | 25 % press | 50 % press | 100 % press |
    |---|---|---|---|
    | 1000 ms | 0.24 s | 0.49 s | 0.99 s |
    | 1500 ms (default) | 0.36 s | 0.74 s | 1.49 s |
    | 3000 ms | 0.72 s | 1.47 s | 2.97 s |

    Simpler alternative (not recommended): the run starts immediately on the press.
  - No gear, ignition, flight-mode, heartbeat or RC-alive condition.
- **After the run starts — unchanged.** The run completes even if the brake is let go; nothing
  re-applies; a brake press never APPLIES the parking brake and does nothing while it is RELEASED,
  RELEASING or APPLYING; the firmware never moves the service brake by itself; the throttle hold
  during the release run, the drive-away release, the manual web / GCS commands, the 3 km/h apply
  interlock and the speed governor are unchanged.
- **Pure law.** `include/ParkBrakeLaw.h` gains a small service-brake release step (previous target,
  pending flag → next state + fire), with native tests in `test/test_park_brake_law`.
- **Web UI.** No new setting, switch or telemetry key. The Parking brake card's help (ⓘ) says, in uk
  and en, that pressing the service brake releases the parking brake — **including the automatic
  brake on RC loss**.
- **Docs.** `WEB_PORTAL_SETUP.md`, `MAVLINK_SETUP.md`, `../../API/README.md`, Changelog 1.5.37 (uk +
  en, "багі").
- **Version.** Firmware **1.5.37**, after `add-crank-time-limit-setting` (1.5.36) is merged into
  `aztec`. The LittleFS image must be uploaded (help text).

## Impact
- Affected specs (all ADDED; the parking-brake requirements live in the unarchived
  `add-aztec-parking-brake`):
  - `vehicle-systems`: ADDED "Parking Brake Release by Service-Brake Press". For this one trigger it
    supersedes the scenario "Service brake independent" of "Aztec Parking Brake Actuator"
    (tasks 0.4).
  - `web-control`: ADDED "Parking Brake Card Service-Brake Release Help".
- Pending-change overlap:
  - `add-aztec-parking-brake` (implemented, fw 1.5.29, not archived) must be archived first; its
    "Service brake independent" scenario is then reworded (tasks 0.4).
  - `add-crank-time-limit-setting` (1.5.36) goes first. This change adds no telemetry key, so it does
    not touch the telemetry document size.
- Affected code (aztec): `include/ParkBrakeLaw.h`, `include/VehicleController.h`,
  `src/VehicleController.cpp`, `data/index.html`, `include/Changelog.h`, `include/Constants.h`
  (version only), `test/test_park_brake_law/test_main.cpp`.
- Hardware: none.
- Branches: aztec only, like the parking brake. Never cherry-picked into `master`.
- Other repositories: none.
