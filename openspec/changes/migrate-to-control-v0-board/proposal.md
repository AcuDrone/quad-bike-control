# Change: Migrate firmware to the custom `Control_v0` control board

## Why

The vehicle is moving off the ESP32-S3-DevKitC-1 plus hand wiring onto a single custom control PCB
(`Control_v0`, ESP32-S3-WROOM-1U). The board re-pins nearly every peripheral, moves the gear
switches / brake limit sensor / relays off the ESP32's own GPIO onto I2C port expanders, adds a
switchable 24V boost rail, and moves the debug console from UART0 to USB-CDC. The authoritative,
user-approved wiring reference is [`GPIO_PINOUT_CUSTOM_BOARD_S3.md`](../../../GPIO_PINOUT_CUSTOM_BOARD_S3.md)
(§9 lists every constant change, §8 the bring-up warnings).

This is a **HARD SWITCH**: the DevKit wiring is dropped, not kept behind a build flag. Maintaining
two pin maps for a one-off vehicle would double the bring-up surface for a board that is being
replaced permanently, and several old assignments (GPIO19/20 gear inputs) are now physically the
USB data lines, so they cannot coexist.

## What Changes

- **BREAKING — pin map rewrite** in `include/Constants.h`, exactly per `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §9:
  throttle PWM 3→15, transmission servo 9→16, MAVLink TX 15→17 / RX 8→18, CAN CS/SCK/MISO/MOSI
  10/12/13/11→39/40/41/42, AS5600 SDA/SCL 41/42→1/2. VESC (4/5) and brake (6/7) are unchanged.
  New: `PIN_CAN_INT` (GPIO38, defined but unused — the CAN driver keeps polling), `PIN_BOOST_EN`
  (GPIO46), `PIN_ADC_24V` (GPIO9), `PIN_ADC_IS_MUX` (GPIO3, defined for the future current-sense mux).
- **BREAKING — stale steering placeholders deleted.** `PIN_STEER_RPWM` / `PIN_STEER_LPWM` (GPIO17/18)
  and `LEDC_CH_STEER_RPWM` / `LEDC_CH_STEER_LPWM` are removed, not kept "reserved": GPIO17/18 are now
  the MAVLink UART. The current spec statement that those GPIOs and LEDC channels stay reserved is
  superseded.
- **BREAKING — gear switches and brake limit sensor move off GPIO.** `PIN_GEAR_REVERSE/NEUTRAL/LOW/HIGH`
  and `PIN_BRAKE_SENSOR` are deleted. A new PCA9557 input driver reads them from the on-board opto
  expander **U10 @ 0x1A on I2C2 (`Wire1`, SDA GPIO48 / SCL GPIO47 — scan-verified; the schematic's
  port names suggested I2C1, the copper says I2C2)**:
  In1=REVERSE, In2=NEUTRAL, In3=LOW, In4=HIGH, In5=brake limit ("released" endstop), In6-8 spare.
  Inputs are polled non-blocking from `loop()`; consumers (`TransmissionController::getPhysicalGear()`,
  `VehicleController::isBrakeReleased()`) read the cached snapshot instead of `digitalRead()`.
- **BREAKING — `RelayController` moves off GPIO.** `PIN_RELAY1-3` and `PIN_WHEEL_LOCK` are deleted;
  the relays are driven over I2C on the external relay board's PCA9557 **@ 0x1F** (as-shipped
  A2A1A0 = 111 strap, kept) on header X15, driven on **`Wire1` (I2C2)** — its intended home, reached
  after rework of the X15 signal path in the `FB3`/`FB4` ferrite area.
  The board's IO routing is **not** 1:1 (`Relay_1=IO4 … Relay_8=IO1`),
  and two functions drive a paralleled pair, so firmware uses port masks: ignition/ECU line =
  Relay_1 + Relay_2 (IO4+IO5), starter = Relay_3 (IO6), front light = Relay_4 (IO7), front-wheel
  lock = Relay_5 + Relay_6 (IO3+IO2), Relay_7/Relay_8 (IO0/IO1) spare. The public
  `RelayController` API (`begin`, `setIgnitionState`, `requestCrank`, `setFrontLight`,
  `setWheelLock`, `update`, `allOff`, getters) is unchanged, so callers do not move.
- **New — expander fault policy.** Input reads that fail hold the last-known snapshot, mark it stale
  after a bounded window, and raise a fault flag; relay writes are read back from the PCA9557 output
  register and retried, with a starter-specific escalation (the starter must never be left latched).
  Both faults surface in telemetry.
- **New — 24V boost rail.** `PIN_BOOST_EN` (GPIO46) is driven high early in `setup()` (the rail feeds
  the steering VESC, Jetson, Starlink and cameras); `PIN_ADC_24V` (GPIO9) is sampled periodically
  through the board's 200K/24K divider (×9.33) and published as a new telemetry field `rail_24v`,
  with a low-rail warning threshold. A guarded web command `set_boost` is added following the
  existing `set_light` / `set_wheel_lock` pattern.
- **New — console over USB-CDC.** `ARDUINO_USB_CDC_ON_BOOT=1` is added to `platformio.ini`
  `build_flags`. UART0 (GPIO43/44) is reserved for the future CH9121T wired-LAN chip and is not used
  by this change.
- **Cross-change dependency:** the pending, un-implemented change `add-hall-speed-sensor` proposes
  `PIN_SPEED_SENSOR` on GPIO2 (fallback GPIO1). On `Control_v0` both pins are I2C1. Its documents
  must be updated to **GPIO8** — the 6N137 opto-isolated input on connector X2 — because the fitted
  driveline sensor outputs **12V**: the Hall channels (X3/X4/X5) are scaled for 5V via a ×0.65
  divider, while the opto input is current-driven and galvanically isolated and takes 12V with a
  single external series resistor (**560Ω–1kΩ added in the sensor cable**, since the stock 470Ω
  `R22` alone would push ≈22 mA through the 6N137's 20 mA-max LED). The opto output is inverted,
  which is harmless for pulse counting. Its external level-shifter hardware task is dropped, and
  Hall_1 (GPIO14/GPIO21, X3) stays spare. That doc update is a task in this change; whichever change
  lands second must be re-read against the other.

## Out of Scope

Stated explicitly so reviewers do not expect them here:

- **Speed-sensor implementation** — remains in the separate pending `add-hall-speed-sensor` change;
  this change only corrects that change's pin and hardware assumptions.
- **CD4051 IS current monitoring** — `PIN_ADC_IS_MUX` (GPIO3) is defined, but the mux-control PCA9557
  U2 @ 0x1C on I2C2 is **not** driven and BTS7960 current sensing is not implemented.
- **Wired LAN (CH9121T / RJ45 / X18)** — UART0 stays idle and GPIO45 (`CFG_CH9121T`) is untouched.
- **S-BUS input** — dropped for good (no UART was available for it). GPIO8 / the X2 6N137 opto input
  is reallocated to the driveline speed sensor, but that pin is allocated and used by the separate
  `add-hall-speed-sensor` change, not here; this change leaves GPIO8 unused.
- **Relay_7/Relay_8 and opto In6-8** — wired to the connectors, no firmware function assigned.
- **Hall_2 / Hall_3 channels (GPIO10-13), GPIO21, GPIO0 system LED** — left spare.

## Impact

- **Affected specs**
  - `gpio-expander` — **REMOVED** the three MCP23017 requirements (that expander was specified but
    never fitted or implemented; no `MCP23017` symbol exists in `src/` or `include/`) and **ADDED**
    the dual-bus PCA9557 topology, the opto-input reader, the relay output driver, their fault
    policies, and the reserved on-board mux expander.
  - `vehicle-actuators` — **MODIFIED** `Hardware Configuration` (Control_v0 pin map, ESP32-S3 GPIO
    validation, functions that no longer have a GPIO) and `BTS7960 Motor Driver Control` (the freed
    steering GPIO17/18 + LEDC 6/7 are no longer "reserved"; the GPIOs are reallocated to MAVLink).
  - `vehicle-systems` — **ADDED** 24V boost rail control/monitoring and degraded-operation behavior
    on board I/O expander fault; **MODIFIED** `Brake Control System` (brake retraction endstop now
    comes from opto In5 through the expander).
  - `mavlink-interface` — **MODIFIED** `Relay Controller for Ignition and Lights` (expander-backed
    relays, corrected relay truth table keyed on `RELAY_MASK_*`, paired ignition and wheel-lock
    relays).
  - `web-telemetry` — **ADDED** 24V rail voltage telemetry and board I/O health telemetry.
  - `web-control` — **ADDED** guarded 24V boost rail control via the web interface.
  - `debug-logging` — **ADDED** the USB-CDC serial console requirement (UART0 reserved).
- **Affected code**
  - `include/Constants.h` — full pin-map rewrite, new boost/expander/I2C constants, deleted constants.
  - `include/PCA9557Expander.h` + `src/PCA9557Expander.cpp` — new minimal PCA9557 driver (register
    read/write on a caller-supplied `TwoWire`).
  - `include/BoardInputs.h` + `src/BoardInputs.cpp` — new polled opto-input reader (gears + brake
    limit + fault state) on `Wire`.
  - `include/RelayController.h` + `src/RelayController.cpp` — rewritten onto the expander on `Wire1`;
    public API unchanged.
  - `src/TransmissionController.cpp` — `initGearSensors()` / `getPhysicalGear()` read the input
    snapshot instead of `digitalRead()`.
  - `include/VehicleController.h` (`isBrakeReleased()` at `:132`), `src/VehicleController.cpp` —
    brake endstop from the snapshot; `set_boost` web command dispatch; boost/rail state exposure.
  - `src/main.cpp` — boost enable early in `setup()`, `Wire`/`Wire1` init ordering, construct and
    poll the new subsystems in `loop()`, delete the direct `pinMode(PIN_BRAKE_SENSOR, ...)` block.
  - `include/WebPortal.h` + `src/WebPortal.cpp` — `rail_24v`, `boost_on`, board-I/O health fields;
    `set_boost` command parsing.
  - `src/TelemetryManager.cpp` — populate the new fields.
  - `data/index.html` — 24V rail readout, boost toggle, I/O fault indicator, `en` + `uk` i18n keys
    (needs `pio run -t uploadfs`, not just a firmware flash).
  - `platformio.ini` — `build_flags = -DARDUINO_USB_CDC_ON_BOOT=1`.
  - `GPIO_PINOUT_CUSTOM_BOARD_S3.md` — cross-checked against the final `Constants.h`; corrected if
    they disagree (the document is the authority for wiring, the header for firmware).
  - `openspec/changes/add-hall-speed-sensor/{proposal,design,tasks}.md` + its `specs/speed-sensor/spec.md`
    — re-pin to GPIO8 (X2 6N137 opto input), replace the external level-shifter task with the
    LED series-resistor / wiring task.
