# ESP32-S3 GPIO Pinout — Custom Control Board "Control_v0"

**Hardware:** custom control PCB `Control_v0` — ESP32-S3-WROOM-1U (external antenna), `qio_qspi` mode
**Schematic:** `Control_v0.PrjPcb` (7 sheets, Altium) — Alexandr Tulusha, 14.06.2026
**Companion boards:** `Buck Boost.PrjPcb` (→ X10), `Relay.PrjPcb` (→ **X15 / I2C2**, bench-verified)
**Pin Definitions:** `include/Constants.h`
**Supersedes:** [`GPIO_PINOUT_S3.md`](GPIO_PINOUT_S3.md) (ESP32-S3-DevKitC-1 + hand wiring)

---

## Contents

1. [Board Overview](#1-board-overview)
2. [Complete Pin Assignment — All GPIO](#2-complete-pin-assignment--all-gpio)
3. [Connector Reference](#3-connector-reference)
4. [Peripheral / Bus Summary](#4-peripheral--bus-summary)
5. [Opto-Isolated Input Assignment](#5-opto-isolated-input-assignment)
6. [Relay Board Assignment](#6-relay-board-assignment)
7. [24V Boost Subsystem](#7-24v-boost-subsystem)
8. [⚠ Warnings & Bring-Up Checklist](#8--warnings--bring-up-checklist)
9. [Firmware Migration Summary](#9-firmware-migration-summary)

---

## 1. Board Overview

`Control_v0` replaces the ESP32-S3-DevKitC-1 plus hand wiring with a single control PCB.

| Item | Detail |
|------|--------|
| Module | ESP32-S3-WROOM-1U (u.FL / external antenna) |
| Programming & console | USB-C, native USB (GPIO19/20), USB-CDC console |
| Analog current sense | 8× BTS7960 IS channels → CD4051 8:1 mux → GPIO3 (ADC1) |
| Digital inputs | 8× 5V opto-isolated (PC817) → PCA9557 U10 on **I2C2** (scan-verified, §4) |
| Relays | off-board `Relay` PCB — 8× SRD-12VDC + PCA9557 @ 0x1F on **X15 / I2C2** (scan-verified, all relay functions bench-tested) |
| 24V rail | off-board `Buck Boost` PCB — TL494 two-phase 12V→24V, 0-26V / 0-15A, INA180 current sense (X10) |
| CAN | MCP2515 + TJA1051T/3, **8 MHz** crystal |
| Wired LAN (future) | CH9121T + RJ45 (X18) on UART0 |

**Firmware strategy — HARD SWITCH.** `include/Constants.h` will be rewritten to this map; DevKit wiring
is no longer supported on this branch. The implementation lands as a separate OpenSpec change — this
document is the wiring / pin reference only.

---

## 2. Complete Pin Assignment — All GPIO

| GPIO | Board net / connector | New firmware function | Old (DevKit) function | Status |
|------|-----------------------|-----------------------|-----------------------|--------|
| **0** | BOOT button, Prog X1 pin 6; LED3 via BSS138 VT3 (active high) | Boot strap + system LED (optional) | FREE | NEW ⚠ strapping |
| **1** | I2C1 SDA — X14 header (AS5600 0x36 only) | `PIN_STEER_SDA` | FREE | MOVED (was 41) |
| **2** | I2C1 SCL — same bus | `PIN_STEER_SCL` | FREE | MOVED (was 42) |
| **3** | CD4051 8:1 mux common out (Analog_1..8 = BTS7960 IS) | `PIN_ADC_IS_MUX` — ADC1 | `PIN_THROTTLE_PWM` | NEW (input only) |
| **4** | PWM_1 → X6 pin 2 (L_PWM) | `PIN_VESC_TX` — UART2 TX | same | UNCHANGED |
| **5** | PWM_2 → X6 pin 3 (R_PWM) | `PIN_VESC_RX` — UART2 RX | same | UNCHANGED |
| **6** | PWM_3 → X7 pin 2 (L_PWM) | `PIN_BRAKE_LPWM` — LEDC ch5 | same | UNCHANGED |
| **7** | PWM_4 → X7 pin 3 (R_PWM) | `PIN_BRAKE_RPWM` — LEDC ch4 | same | UNCHANGED |
| **8** | Opto-isolated input via 6N137 (X2, ex-"S-BUS") | `PIN_SPEED_SENSOR` — driveline speed (PCNT) ⚠ inverted, see §3/§8.9 | `PIN_MAVLINK_RX` | RE-PINNED |
| **9** | 24V rail via 200K/24K divider | `PIN_ADC_24V` — ADC1 | `PIN_TRANS_SERVO` | NEW feature |
| **10** | Hall_3 channel A (X5) | spare | `PIN_CAN_CS` | SPARE |
| **11** | Hall_3 channel B (X5) | spare | `PIN_CAN_MOSI` | SPARE |
| **12** | Hall_2 channel A (X4) | spare | `PIN_CAN_SCK` | SPARE |
| **13** | Hall_2 channel B (X4) | spare | `PIN_CAN_MISO` | SPARE |
| **14** | Hall_1 channel A (X3) | spare — 5V hall channel (×0.65 divider) | `PIN_BRAKE_SENSOR` | SPARE |
| **15** | PWM_5 → X8 pin 2 (L_PWM) | `PIN_THROTTLE_PWM` — LEDC ch1, 50 Hz | `PIN_MAVLINK_TX` | MOVED (was 3) |
| **16** | PWM_6 → X8 pin 3 (R_PWM) | `PIN_TRANS_SERVO` — LEDC ch2, 50 Hz | FREE | MOVED (was 9) |
| **17** | PWM_7 → X9 pin 2 (L_PWM) | `PIN_MAVLINK_TX` — UART1 TX → Pixhawk RX | `PIN_STEER_RPWM` (freed) | MOVED (was 15) |
| **18** | PWM_8 → X9 pin 3 (R_PWM) | `PIN_MAVLINK_RX` — UART1 RX ← Pixhawk TX | `PIN_STEER_LPWM` (freed) | MOVED (was 8) |
| **19** | USB D− → USB-C | Native USB / USB-CDC console | `PIN_GEAR_REVERSE` | NEW (USB) |
| **20** | USB D+ → USB-C | Native USB / USB-CDC console | `PIN_GEAR_NEUTRAL` | NEW (USB) |
| **21** | Hall_1 channel B (X3) | spare — 5V hall channel (×0.65 divider) | `PIN_GEAR_LOW` | SPARE |
| **35** | not connected | — | FREE | N/C |
| **36** | not connected | — | `PIN_RELAY1` | N/C (→ relay board) |
| **37** | not connected | — | `PIN_RELAY2` | N/C (→ relay board) |
| **38** | MCP2515 INT | `PIN_CAN_INT` — optional/future (firmware polls) | `PIN_RELAY3` | NEW |
| **39** | MCP2515 SPI CS | `PIN_CAN_CS` | `PIN_WHEEL_LOCK` | MOVED (was 10) |
| **40** | MCP2515 SPI CLK | `PIN_CAN_SCK` | FREE | MOVED (was 12) |
| **41** | MCP2515 SPI MISO | `PIN_CAN_MISO` | `PIN_STEER_SDA` | MOVED (was 13) |
| **42** | MCP2515 SPI MOSI | `PIN_CAN_MOSI` | `PIN_STEER_SCL` | MOVED (was 11) |
| **43** | UART0 TX → Prog X1 (510Ω R13) **and** CH9121T RXD1 (100Ω R63) | RESERVED — future wired LAN | console TX | RESERVED ⚠ shared |
| **44** | UART0 RX ← Prog X1 (510Ω R14) **and** CH9121T TXD1 (100Ω R64) | RESERVED — future wired LAN | console RX | RESERVED ⚠ shared |
| **45** | `CFG_CH9121T` via BSS138 | LAN config-mode toggle — unused until LAN enabled | FREE | RESERVED ⚠ strapping |
| **46** | `Bust_On/OFF` → X10 pin 1 | `PIN_BOOST_EN` — boost enable output | FREE | NEW ⚠ strapping |
| **47** | I2C2 SCL (BSS138 level shift) | On-board U10/U2 + X15 bus SCL | `PIN_GEAR_HIGH` | NEW |
| **48** | I2C2 SDA (BSS138 level shift) | On-board U10/U2 + X15 bus SDA | FREE | NEW |

> GPIO2 was the pin proposed by the pending `add-hall-speed-sensor` change. On this board GPIO2 is
> I2C1 SCL, and Hall_1 (GPIO14) is scaled for **5V** sensors — the fitted driveline sensor outputs
> **12V**, so the speed sensor lands on **GPIO8**, the 6N137 opto-isolated input on X2. The opto path
> is current-driven and galvanically isolated, so 12V is handled with one external series resistor.
> See warning 9 and §3 (X2). S-BUS as an RC-receiver input is dropped for good.

---

## 3. Connector Reference

| Connector | Purpose | Pinout |
|-----------|---------|--------|
| **X1** | Prog / programming header | 3V3, RESET, GND, TxD (GPIO43), RxD (GPIO44), BOOT |
| **X2** | Driveline speed sensor (12V), opto-isolated | 1 = In+, 2 = In− → 470Ω `R22` → 6N137 LED → open-collector output + pull-up → **GPIO8**. ⚠ **Inverted** (LED on → GPIO8 low) and ⚠ **needs an extra series resistor** — see below |
| **X3** | Hall 1 — spare | 1=5V, 2=Hall A (GPIO14), 3=Hall B (GPIO21), 4=GND |
| **X4** | Hall 2 — spare | 1=5V, 2=Hall A (GPIO12), 3=Hall B (GPIO13), 4=GND |
| **X5** | Hall 3 — spare | 1=5V, 2=Hall A (GPIO10), 3=Hall B (GPIO11), 4=GND |
| **X6** | VESC steering (UART) | 1=5V(fused), 2=L_PWM → **VESC RX** (GPIO4 TX), 3=R_PWM → **VESC TX** (GPIO5 RX), 4/5=IS n/c, 6=GND |
| **X7** | Brake BTS7960 | 1=5V(fused), 2=L_PWM (GPIO6), 3=R_PWM (GPIO7), 4=L_IS, 5=R_IS, 6=GND |
| **X8** | Servos | 1=5V(fused, servo supply), 2=throttle PWM (GPIO15), 3=transmission PWM (GPIO16), 4/5=IS, 6=GND |
| **X9** | MAVLink → Pixhawk TELEM | 2 → Pixhawk RX (GPIO17 TX), 3 ← Pixhawk TX (GPIO18 RX), 6=GND. ⚠ **Do NOT connect pin 1 (5V)**; leave 4/5 unconnected |
| **X10** | Buck Boost board | 1 = Boost On/Off (GPIO46), 2 = GND, 3 = +24V measure → divider → GPIO9 |
| **X11** | Power in | 3× +12V, 3× GND (main 12V input) |
| **X12** | LAN UART2 | CH9121T's second UART — LAN-side, **not** an ESP UART |
| **X13** | CAN | CANL, GND, CANH — TJA1051T/3, split termination 2×60Ω + 47n, common-mode choke |
| **X14** | I2C1 header — **AS5600 steering angle sensor only** (reserved for it; the sensor is not yet connected) | VCC 3V3, SDA (GPIO1), SCL (GPIO2), GND |
| **X15** | I2C2 header — relay board @ 0x1F, **working / bench-verified** | VCC, SDA (GPIO48), SCL (GPIO47), GND. ⚠ *History:* on the first assembled board this connector passed no signals until rework in the `FB3`/`FB4` ferrite area — **check that area first on any board copy that shows a silent X15** |
| **X16** | Opto inputs | In1..In4 (In_Plus / In_Minus pairs) + GND |
| **X17** | Opto inputs | In5..In8 (In_Plus / In_Minus pairs) + GND |
| **X18** | Ethernet | RJ45 (CH9121T) |
| **USB-C** | Flashing + console | Native USB (GPIO19/20), USB-CDC |

**Hall input conditioning (X3/X4/X5):** ferrite bead (1 kΩ @ 100 MHz) → 1.2K / 2.2K divider (×0.65)
→ PESD3V3L1UA TVS + 100n. Designed for **5V** sensors. All three channels are **spare** — the
driveline speed sensor is a 12V type and uses the X2 opto input instead.

**Speed-sensor input conditioning (X2, 6N137):** `X2.1 / X2.2` → 470Ω `R22` → 6N137 LED →
open-collector output with pull-up → **GPIO8**. Current-driven and galvanically isolated, so a 12V
sensor is safe with the series resistor below. The output is **inverted** (LED conducting pulls GPIO8
low) — irrelevant for pulse counting, but PCNT must be configured on one chosen edge with the
inversion noted.

> ⚠ **REQUIRED — extra series resistance for a 12V sensor.** At 12V the stock 470Ω `R22` alone gives
> ≈22 mA of LED current, slightly **over the 6N137's 20 mA absolute maximum**. Add **560Ω–1kΩ in
> series in the sensor cable** (preferred; ≈7–10 mA, inside the 6–15 mA sweet spot), or rework `R22`
> to 1kΩ on the board.
>
> Wiring by sensor type:
> - **Push-pull 12V sensor:** sensor signal → `X2.1` (series R in line), sensor GND → `X2.2`.
> - **Open-collector (NPN) sensor:** +12V through the series R → `X2.1`, sensor output → `X2.2`
>   (the sensor sinks the LED current).

**BTS7960 IS conditioning (X7/X8 pins 4/5 and all Analog_1..8):** 4.7K series + 10K to GND +
BAV199 clamp + 100n, into the CD4051 mux.

---

## 4. Peripheral / Bus Summary

### UART Allocation

| UART | RX | TX | Baud | Purpose |
|------|----|----|------|---------|
| UART0 | 44 | 43 | — | **RESERVED** for CH9121T wired LAN (future). Console moved to USB-CDC |
| UART1 | 18 | 17 | 115200 | MAVLink 2 ↔ Pixhawk TELEM (via X9) |
| UART2 | 5 | 4 | 115200 | VESC steering driver (Flipsky 75200) via X6 — unchanged |
| — | — | — | — | No UART is free for S-BUS; **S-BUS is dropped for good** and GPIO8 (X2 opto) is now the speed-sensor pulse input |

### I2C Allocation

| Bus | SDA | SCL | Speed | Devices |
|-----|-----|-----|-------|---------|
| I2C1 (`Wire`) | 1 | 2 | 100 kHz | **X14 header only**, reserved for the AS5600 @ 0x36 (not yet connected). No other device belongs here |
| I2C2 (`Wire1`) | 48 | 47 | 100 kHz | **Both on-board PCA9557s plus the relay board.** Opto-input **U10 @ 0x1A** (A2A1A0 = 010); mux-control **U2 @ 0x1C** (A2A1A0 = 100); external relay board **@ 0x1F** on header X15 |

> ⚠ **EMPIRICAL — established by live `i2cdetect`-style scans on the assembled board, and it
> overrides the schematic reading.** The schematic's port names suggested the opto-input expander
> **U10 was on I2C1**; the scans show **both** on-board expanders (U10 @ 0x1A *and* U2 @ 0x1C)
> answering on **I2C2 (GPIO47/48)**, and nothing on-board answering on I2C1. Trust the copper.
>
> The final, bench-verified I2C2 scan is **0x1A, 0x1C, 0x1F** — the two on-board expanders plus the
> relay board on X15. `RelayController` is constructed with `Wire1` in `src/main.cpp`, and the relay
> board keeps its as-shipped **0x1F** strap (it collides with neither 0x1A nor 0x1C).
>
> ⚠ *History (relevant to any board copy):* on the first assembled board X15 passed no signals at
> all — a device on X15 never answered while the same device answered immediately on X14, twice.
> It was fixed by rework in the **`FB3`/`FB4` ferrite area**; check there first if X15 is silent on
> another board. If the connector is ever unusable, the relay board also runs on X14 → I2C1 (change
> the `RelayController` bus argument to `Wire` and borrow X14 from the AS5600) — a contingency, not
> the current wiring.

> Both buses run at **100 kHz** (`I2C_BUS_FREQ_HZ` in `include/Constants.h`, shared by `Wire` and
> `Wire1`). The AS5600 is at the far end of a long cable run to the steering column via X14, so
> signal-integrity margin beats bus speed; 400 kHz was tried and rejected. Going faster is a
> hardware change first (stronger pull-up on the X14 run), verified on the bench.

### SPI (CAN)

| Signal | GPIO |
|--------|------|
| CS | 39 |
| CLK | 40 |
| MISO | 41 |
| MOSI | 42 |
| INT | 38 (optional/future — firmware currently polls) |

The MCP2515 crystal on this board is **8 MHz**; firmware already uses `MCP_8MHZ`
(`src/CANController.cpp`) — no clock change needed.

### LEDC PWM Channels

| Channel | GPIO | Function | Frequency | Resolution |
|---------|------|----------|-----------|------------|
| 1 | **15** | Throttle servo | 50 Hz | 14-bit |
| 2 | **16** | Transmission servo | 50 Hz | 14-bit |
| 3 | — | FREE | — | — |
| 4 | 7 | Brake RPWM | 10 kHz | 8-bit |
| 5 | 6 | Brake LPWM | 10 kHz | 8-bit |
| 6 / 7 | — | Freed (former steering PWM) — **released**, GPIO17/18 now MAVLink | — | — |

Channel numbers are unchanged; only the throttle/transmission pins move to 15/16.

### ADC

| GPIO | Source | Scaling |
|------|--------|---------|
| 3 | BTS7960 IS current sense, 8 channels via CD4051 (channel select A/B/C + enable on PCA9557 U2, I2C2) | per-channel 4.7K/10K + clamp |
| 9 | 24V boost rail | 200K/24K → V24 = Vadc × 224/24 ≈ **Vadc × 9.33** (24V → ~2.57V) |

### PCNT

| GPIO | Source |
|------|--------|
| 8 | Driveline speed sensor via the 6N137 opto input (X2) — re-pinned from GPIO2, then from GPIO14. Signal is **inverted** by the optocoupler; count on one edge (falling or rising — either works) |

---

## 5. Opto-Isolated Input Assignment

All inputs are 5V, PC817-isolated, each an `In_Plus` / `In_Minus` pair. They are read through
**PCA9557 U10 @ 0x1A on I2C2 (`Wire1`, GPIO47/48)** — not directly by GPIO. (Scan-verified: the
schematic port names implied I2C1, the copper says I2C2 — see §4.)

| Input | Connector | Function |
|-------|-----------|----------|
| In1 | X16 | Gear REVERSE |
| In2 | X16 | Gear NEUTRAL |
| In3 | X16 | Gear LOW |
| In4 | X16 | Gear HIGH |
| In5 | X17 | Brake limit sensor ("released" endstop — 1000 ms overrun logic, I2C latency acceptable) |
| In6-In8 | X17 | Spare |

---

## 6. Relay Board Assignment

`Relay.PrjPcb` — 8× SRD-12VDC relays (NO / COM / NC per relay), board powered from 12V, driven by its
own PCA9557 **@ 0x1F** (as-shipped straps A2A1A0 = 111 — kept, see warning 8). It sits on its
intended home **X15 / I2C2** and answers at 0x1F; every relay function below has been exercised on
the bench. **X4..X11 below are the RELAY BOARD's own connectors** — not the main board's X4/X5 Hall
headers in §3.

| Relay | PCA9557 IO | Port bit | Connector | Function |
|-------|-----------|----------|-----------|----------|
| Relay_1 | **IO4** | `0b00010000` | X4 | Ignition / ECU line — **pair with Relay_2** |
| Relay_2 | **IO5** | `0b00100000` | X5 | Ignition / ECU line — **pair with Relay_1** |
| Relay_3 | **IO6** | `0b01000000` | X6 | Starter solenoid |
| Relay_4 | **IO7** | `0b10000000` | X7 | Front light |
| Relay_5 | **IO3** | `0b00001000` | X8 | Front-wheel lock — **pair with Relay_6** |
| Relay_6 | **IO2** | `0b00000100` | X9 | Front-wheel lock — **pair with Relay_5** |
| Relay_7 | **IO0** | `0b00000001` | X10 | Spare — always driven off |
| Relay_8 | **IO1** | `0b00000010` | X11 | Spare — always driven off |

> ⚠ **The PCA9557 IO routing is NOT 1:1 with the relay numbers.** Verified from the `Relay.PrjPcb`
> schematic netlist: `Relay_1=IO4, Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2,
> Relay_7=IO0, Relay_8=IO1`. Do **not** "fix" firmware back to `Relay_N = bit N` — that mapping is
> wrong and would fire the starter when the front light is commanded. The firmware encodes this in
> `RELAY_MASK_*` in `include/Constants.h`, never in per-relay bit indices.

**Duplicate (paralleled) relays are intentional.** Two functions are split across two relay contacts
that always energize together:

| Function | Mask (`include/Constants.h`) | Value | Relays / IO |
|----------|------------------------------|-------|-------------|
| Ignition / ECU line | `RELAY_MASK_IGNITION` | `0b00110000` | Relay_1 (IO4) + Relay_2 (IO5) |
| Starter | `RELAY_MASK_STARTER` | `0b01000000` | Relay_3 (IO6) |
| Front light | `RELAY_MASK_FRONT_LIGHT` | `0b10000000` | Relay_4 (IO7) |
| Front-wheel lock | `RELAY_MASK_WHEEL_LOCK` | `0b00001100` | Relay_5 (IO3) + Relay_6 (IO2) |
| Spare | — | `0b00000011` | Relay_7 (IO0) + Relay_8 (IO1), always 0 |

**Harness wiring note.** The ignition/ECU load is wired to **X4 + X5**, the front-wheel lock to
**X8 + X9**, the starter solenoid to **X6**, and the front light to **X7**. Nothing is landed on X10
or X11. The whole 8-bit port is written from a single shadow byte and read back for verification,
so a paired relay can never be left half-energized by firmware.

---

## 7. 24V Boost Subsystem

`Buck Boost.PrjPcb` — TL494 two-phase converter, 12V → 24V, adjustable 0-26V / 0-15A, INA180 current
sense.

| Aspect | Detail |
|--------|--------|
| Enable | GPIO46 → X10 pin 1 (`Bust_On/OFF`), 100K pulldown R7 on the control board |
| Measurement | X10 pin 3 → 200K/24K divider → GPIO9 (÷9.33) |
| Loads | Steering VESC (Flipsky 75200), Starlink, Nvidia Jetson, cameras |
| Firmware to add | Enable control (on at boot), rail-voltage telemetry, correlation with steering driver-ok |

A separate 7s LiFePO4 battery also exists in the vehicle power system.

---

## 8. ⚠ Warnings & Bring-Up Checklist

1. **GPIO43/44 are shared copper** between the Prog connector and the CH9121T. A device on the Prog
   connector and an active LAN chip are mutually exclusive — CH9121T TXD1 through 100Ω wins over the
   Prog header through 510Ω. Keep the LAN section unpopulated / held silent until UART0 is truly free.
2. **Strapping pins:** GPIO0 (boot / LED), GPIO45 (`CFG_CH9121T`), GPIO46 (boost enable — 100K pulldown
   keeps the boost off through reset; verify there is no enable glitch at boot), GPIO3 (JTAG source
   select strap — used as an ADC **input only**, which is fine).
3. **PWM-header UART links (X6 VESC, X9 MAVLink)** run through BSS138 bidirectional level shifters with
   10K pull-ups to 5V, 1K series and 100pF. Verified workable at 115200: a logic low arrives at ~0.5V at
   the ESP, highs reach ~5V through ~11K into the peripheral (Pixhawk TELEM and VESC STM32 pins are
   5V-tolerant). **Do not raise the baud rate** (rise time ~1 µs) and keep cables short.
4. **X9 pin 1 (5V) must NOT be wired to Pixhawk TELEM 5V** — both ends are supply outputs.
5. **Hall inputs (X3/X4/X5) are scaled for 5V sensors** (×0.65 divider) and are all **spare**. A 12V
   push-pull signal would exceed the GPIO absolute maximum — this is why the 12V driveline speed
   sensor uses the X2 opto input instead (warning 9).
   - 12V open-collector sensor: power the sensor from 12V and pull the signal up to the connector's 5V
     pin with 4.7-10K.
   - True 12V push-pull: change that channel's 1.2K series resistor to ~6.8K — the channel then becomes
     12V-only.
6. **I2C2 level-shifter low-side rail** is jumper-selected by 0Ω `R16` (3V3) / `R18` (1V8). With the
   WROOM-1 in `qio_qspi` mode GPIO47/48 are 3.3V IOs — verify **R16 populated, R18 not**.
7. **Debug console is USB-CDC.** Add the `ARDUINO_USB_CDC_ON_BOOT=1` build flag (`platformio.ini`
   currently has no `build_flags` for it); use the serial monitor over USB-C.
8. **Relay-board PCA9557 address straps: as-shipped A2A1A0 = 111 → 0x1F, KEPT.** Any strap **except
   0x1A (U10) and 0x1C (U2)** is acceptable — those two are the on-board expanders on I2C2. The
   board ships at **0x1F**, which collides with neither, so there is **no rework to do**; the earlier
   "must be strapped 0x18" instruction is withdrawn. `PCA9557_ADDR_RELAYS` is 0x1F.
   - The board lives on **X15 (I2C2)** and the firmware constructs `RelayController` with `Wire1`.
     A boot scan of I2C2 shows **0x1A, 0x1C, 0x1F**.
   - ⚠ **X15 needed rework on the first assembled board.** As delivered it passed no I2C signals at
     all (a device on X15 never answered, while the same device answered immediately on X14). The
     fix was in the **`FB3`/`FB4` ferrite area** of the X15 SDA/SCL run — **check there first** on
     any board copy whose X15 is silent (continuity check, bridge/populate). If a connector is ever
     beyond repair, the relay board also works on X14 (I2C1) with `RelayController` constructed on
     `Wire` — contingency only.
   - ⚠ **The relay board's IO routing is NOT 1:1** (§6): `Relay_1=IO4, Relay_2=IO5, Relay_3=IO6,
     Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`, verified from the
     `Relay.PrjPcb` netlist. Never "simplify" the firmware back to `Relay_N = bit N` — that would
     command the starter when the front light is requested. Firmware uses `RELAY_MASK_*`.
   - Two loads are deliberately split across **paired** relays that switch together: the
     ignition/ECU line on **X4 + X5** (Relay_1 + Relay_2) and the front-wheel lock on **X8 + X9**
     (Relay_5 + Relay_6). Starter is **X6** (Relay_3), front light **X7** (Relay_4). X10/X11
     (Relay_7/Relay_8) stay unconnected and are always written off.
9. **Driveline speed sensor is on GPIO8 — the X2 6N137 opto input, not a Hall channel.** The fitted
   sensor outputs **12V**; the Hall channels are scaled for 5V (warning 5), while the opto input is
   current-driven and galvanically isolated, so it takes 12V with a single external series resistor.
   The pending OpenSpec change `add-hall-speed-sensor` uses `PIN_SPEED_SENSOR` = **GPIO8**.
   - ⚠ **20 mA caveat:** at 12V the stock 470Ω `R22` alone draws ≈22 mA through the 6N137 LED —
     over its **20 mA absolute maximum**. Add **560Ω–1kΩ in series in the sensor cable** (preferred;
     ≈7–10 mA, in the 6–15 mA sweet spot) or rework `R22` to 1kΩ. Do this **before** first power-up
     with the sensor connected.
   - Wiring: push-pull 12V → signal to `X2.1`, sensor GND to `X2.2` (series R in line);
     open-collector NPN → +12V through the series R to `X2.1`, sensor output to `X2.2`.
   - The opto **inverts** the signal (LED on → GPIO8 low). Harmless for pulse counting; the PCNT
     config just picks one edge and records the inversion.
   - Consequence: **S-BUS as an RC-receiver input is gone for good** (no UART was available anyway),
     and Hall_1 (GPIO14 A / GPIO21 B, X3) returns to spare.

---

## 9. Firmware Migration Summary

### `include/Constants.h` — changed pins

| Constant | Old | New |
|----------|-----|-----|
| `PIN_THROTTLE_PWM` | GPIO 3 | **GPIO 15** |
| `PIN_TRANS_SERVO` | GPIO 9 | **GPIO 16** |
| `PIN_MAVLINK_TX` | GPIO 15 | **GPIO 17** |
| `PIN_MAVLINK_RX` | GPIO 8 | **GPIO 18** |
| `PIN_CAN_CS` | GPIO 10 | **GPIO 39** |
| `PIN_CAN_SCK` | GPIO 12 | **GPIO 40** |
| `PIN_CAN_MISO` | GPIO 13 | **GPIO 41** |
| `PIN_CAN_MOSI` | GPIO 11 | **GPIO 42** |
| `PIN_STEER_SDA` | GPIO 41 | **GPIO 1** |
| `PIN_STEER_SCL` | GPIO 42 | **GPIO 2** |
| `PIN_VESC_TX` / `PIN_VESC_RX` | GPIO 4 / 5 | unchanged |
| `PIN_BRAKE_LPWM` / `PIN_BRAKE_RPWM` | GPIO 6 / 7 | unchanged |

### New constants

| Constant | GPIO | Purpose |
|----------|------|---------|
| `PIN_CAN_INT` | 38 | MCP2515 interrupt (optional — firmware polls today) |
| `PIN_BOOST_EN` | 46 | 24V boost enable → X10 |
| `PIN_ADC_24V` | 9 | 24V rail measurement (÷9.33) |
| `PIN_ADC_IS_MUX` | 3 | CD4051 common output — BTS7960 IS channels |
| `PIN_SPEED_SENSOR` | 8 | Driveline speed sensor (PCNT) via the X2 6N137 opto input, inverted — from the pending speed-sensor change |

### Removed constants (function moved off-GPIO)

| Constant | Old GPIO | New location |
|----------|----------|--------------|
| `PIN_BRAKE_SENSOR` | 14 | Opto input **In5** (PCA9557 U10, I2C2) |
| `PIN_GEAR_REVERSE` | 19 | Opto input **In1** |
| `PIN_GEAR_NEUTRAL` | 20 | Opto input **In2** |
| `PIN_GEAR_LOW` | 21 | Opto input **In3** |
| `PIN_GEAR_HIGH` | 47 | Opto input **In4** |
| `PIN_RELAY1` | 36 | Relay board ignition/ECU line — **Relay_1 + Relay_2** (IO4+IO5, `RELAY_MASK_IGNITION`) |
| `PIN_RELAY2` | 37 | Relay board starter — **Relay_3** (IO6, `RELAY_MASK_STARTER`) |
| `PIN_RELAY3` | 38 | Relay board front light — **Relay_4** (IO7, `RELAY_MASK_FRONT_LIGHT`) |
| `PIN_WHEEL_LOCK` | 39 | Relay board front-wheel lock — **Relay_5 + Relay_6** (IO3+IO2, `RELAY_MASK_WHEEL_LOCK`) |
| `PIN_STEER_RPWM` / `PIN_STEER_LPWM` | 17 / 18 | Fully released — GPIO17/18 are now MAVLink UART1. The "RESERVED/FREED" placeholders (and `LEDC_CH_STEER_RPWM`/`LEDC_CH_STEER_LPWM`) must be deleted, not kept reserved |

### New / reworked drivers

| Item | Work |
|------|------|
| PCA9557 input reader | New driver on **I2C2** @ 0x1A (U10) — supplies gear switches and the brake limit sensor to `TransmissionController` / brake logic (replaces `digitalRead`) |
| `RelayController` | Rewrite from direct GPIO to the relay-board PCA9557 @ 0x1F — bus passed in by `main.cpp` (**`Wire1`/I2C2** via X15) |
| Boost control / telemetry | `PIN_BOOST_EN` on at boot; `PIN_ADC_24V` rail voltage into telemetry |
| Console | Add `ARDUINO_USB_CDC_ON_BOOT=1` build flag (USB-CDC over USB-C) |
| CD4051 IS mux (future) | PCA9557 U2 @ 0x1C drives A/B/C + enable; read on `PIN_ADC_IS_MUX` — brake-motor current monitoring |

**Source files touched by the removed constants:** `include/Constants.h`, `include/VehicleController.h`,
`src/main.cpp`, `src/RelayController.cpp`, `src/TransmissionController.cpp`.
