# ESP32-S3 GPIO Pinout — Quad Bike Control System

**Hardware:** ESP32-S3-DevKitC-1 v1.1 N16R8, hand-wired (OPI PSRAM not used — `qio_qspi` mode; GPIO 33-37 free)
**Reference:** [Official User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)
**Pin definitions:** `include/Constants.h` — this document and that header must agree; if they
disagree, the divergence is resolved before the firmware is run on the vehicle.

On this board every relay is driven **directly from a GPIO** and the gear switches and brake limit
sensor are read **directly with `digitalRead()`**. There are no I2C port expanders and no 24V boost
rail — those belong to the custom PCB, which this branch does not target.

---

## Functional Pin Map

Every `PIN_*` define in `include/Constants.h`, in GPIO order, plus the two console pins.

| GPIO | Define | Dir | Peripheral | Connects to |
|------|--------|-----|------------|-------------|
| **3** | `PIN_THROTTLE_PWM` | out | LEDC ch 1, 50 Hz | Throttle servo signal |
| **4** | `PIN_VESC_TX` | out | UART2 TX, 115200 | Flipsky 75200 VESC RX (steering, brushed-DC mode) |
| **5** | `PIN_VESC_RX` | in | UART2 RX, 115200 | Flipsky 75200 VESC TX |
| **6** | `PIN_BRAKE_LPWM` | out | LEDC ch 5 | BTS7960 LPWM — brake actuator retract |
| **7** | `PIN_BRAKE_RPWM` | out | LEDC ch 4 | BTS7960 RPWM — brake actuator extend |
| **8** | `PIN_MAVLINK_RX` | in | UART1 RX, 115200 | Pixhawk TELEM2 **TX** (non-inverted, no inverter) |
| **9** | `PIN_TRANS_SERVO` | out | LEDC ch 2, 50 Hz | Transmission (gear selector) servo signal |
| **10** | `PIN_CAN_CS` | out | SPI CS | MCP2515 CAN controller chip select |
| **11** | `PIN_CAN_MOSI` | out | SPI MOSI | MCP2515 SI |
| **12** | `PIN_CAN_SCK` | out | SPI SCK | MCP2515 SCK |
| **13** | `PIN_CAN_MISO` | in | SPI MISO | MCP2515 SO |
| **14** | `PIN_BRAKE_SENSOR` | in | plain GPIO (`INPUT`) | Brake "fully released" limit switch — **HIGH = released** |
| **15** | `PIN_MAVLINK_TX` | out | UART1 TX, 115200 | Pixhawk TELEM2 **RX** (non-inverted) |
| **17** | `PIN_SPEED_SENSOR` | in | PCNT unit, 1 µs glitch filter | Hall driveline speed pickup — **NEW**, ⚠ wiring TBD on the bench |
| **18** | — | — | — | **FREE / RESERVED** (former `PIN_STEER_LPWM`) |
| **19** | `PIN_GEAR_REVERSE` | in | plain GPIO (floating, external pull-up) | Gear switch **R** — active LOW |
| **20** | `PIN_GEAR_NEUTRAL` | in | plain GPIO (floating, external pull-up) | Gear switch **N** — active LOW |
| **21** | `PIN_GEAR_LOW` | in | plain GPIO (floating, external pull-up) | Gear switch **L** — active LOW |
| **36** | `PIN_RELAY1` | out | plain GPIO | Relay 1 — ignition / ECU line (active HIGH) |
| **37** | `PIN_RELAY2` | out | plain GPIO | Relay 2 — starter solenoid (active HIGH) |
| **38** | `PIN_RELAY3` | out | plain GPIO | Relay 3 — front light (active HIGH) |
| **39** | `PIN_WHEEL_LOCK` | out | plain GPIO | Relay 4 — front-wheel lock (active HIGH) |
| **41** | `PIN_STEER_SDA` | i/o | I2C (`Wire`), 100 kHz | AS5600 steering angle sensor, addr `0x36` |
| **42** | `PIN_STEER_SCL` | out | I2C (`Wire`), 100 kHz | AS5600 steering angle sensor, addr `0x36` |
| **43** | — (`Serial`) | out | UART0 TX, 115200 | **Debug console** — on-board USB-UART bridge, "COM" port |
| **44** | — (`Serial`) | in | UART0 RX, 115200 | **Debug console** — on-board USB-UART bridge, "COM" port |
| **47** | `PIN_GEAR_HIGH` | in | plain GPIO (floating, external pull-up) | Gear switch **H** — active LOW |

### Notes on the new speed sensor (GPIO 17)

GPIO 17 and GPIO 18 were freed when the BTS7960 steering driver was replaced by the VESC (they were
`PIN_STEER_RPWM` / `PIN_STEER_LPWM`, LEDC channels 6/7). **GPIO 17 now carries the hall driveline
speed sensor**, counted in hardware by a PCNT unit.

⚠ There is **no opto isolation on the DevKit** — the sensor drives the pin directly, so the wiring is
still to be confirmed on the bench. A 12V-output sensor **must** be level-shifted to 3.3V before this
pin is connected; GPIO 17 is not 12V tolerant. The firmware counts one edge only; the pulse rate is
the same on either edge, so polarity is a documentation matter, not a correctness one.

GPIO 18 stays **free/reserved** and LEDC channels 6 and 7 stay unallocated.

### Input electrical conventions

- **Gear switches (19/20/21/47):** configured `GPIO_MODE_INPUT` with `GPIO_FLOATING` — the pull-ups
  are **external**, on the harness, not internal. A selected gear pulls its pin LOW; the firmware
  reads `!digitalRead(pin)` and requires exactly one active switch, retrying up to
  `TRANS_GEAR_READ_RETRY_COUNT` times while the servo is idle. Zero or more than one active reads as
  `GEAR_UNKNOWN`, which caps throttle at `TRANS_UNKNOWN_GEAR_THROTTLE_MAX`.
- **Brake limit sensor (14):** `pinMode(INPUT)`, no internal pull-up. **HIGH = brake released.**
- **Relays (36/37/38/39):** `pinMode(OUTPUT)`, driven LOW at `begin()`; **HIGH energizes**.

---

## Console, Flashing and Filesystem

The debug console is **UART0 — GPIO 43 (TX) / GPIO 44 (RX)** — reached through the DevKitC-1's
on-board USB-UART bridge, the **"COM"/"UART" USB-C port**. `Serial` is UART0 at `SERIAL_BAUD_RATE`
(115200) and `setup()` never waits for a host: the vehicle must boot with no laptop attached.
Flashing and the serial monitor both go over that "COM" port.

> ⛔ **Do not enable the native USB CDC console on this board.** The native USB D−/D+ lines are
> GPIO 19 and GPIO 20, and here those pins are `PIN_GEAR_REVERSE` and `PIN_GEAR_NEUTRAL`. Adding
> `-DARDUINO_USB_CDC_ON_BOOT=1` to `platformio.ini` would take the two gear switches away from the
> transmission interlock. GPIO 43/44 are therefore **in use** by the console, not free.

**Flash layout** — `partitions_16mb.csv`, 16 MB flash:

| Partition | Type | Offset | Size |
|-----------|------|--------|------|
| `nvs` | data/nvs | `0x9000` | 20 KB |
| `otadata` | data/ota | `0xE000` | 8 KB |
| `app0` | app/ota_0 | `0x10000` | 4 MB |
| `app1` | app/ota_1 | `0x410000` | 4 MB |
| `spiffs` (LittleFS) | data/spiffs | `0x810000` | **1.5 MB** — web UI assets in `data/` |
| `coredump` | data/coredump | `0x990000` | 64 KB |

Flash beyond `0x9A0000` is intentionally unallocated: a large filesystem partition makes `uploadfs`
erase megabytes in one stretch, which is slow and fragile, and 1.5 MB is ample for the web UI in
`data/`.

---

## Free Pins

| GPIO | Header | Notes |
|------|--------|-------|
| **0** | J3-14 | ⚠ Strapping: HIGH = normal boot, LOW = download mode |
| **1** | J3-4 | General I/O, ADC1 |
| **2** | J3-5 | General I/O, ADC1 |
| **16** | J1-9 | General I/O, ADC2, 32 kHz crystal capable |
| **18** | J1-11 | Reserved — former `PIN_STEER_LPWM` (LEDC ch 7) |
| **35** | J3-13 | Was OPI PSRAM SPIIO6 — free with `qio_qspi` |
| **40** | J3-8 | JTAG TDO |
| **45** | J3-15 | ⚠ Strapping: VDD_SPI voltage level |
| **46** | J1-14 | ⚠ Strapping: ROM log verbosity |
| **48** | J3-16 | General I/O |

---

## Power & Non-GPIO Pins

| Pin | Header | Type | Notes |
|-----|--------|------|-------|
| 3V3 | J1-1, J1-2 | Power out | 3.3 V |
| 5V | J1-21 | Power in/out | 5 V |
| RST/EN | J1-3 | Input | Active-low chip enable |
| GND | J1-22, J3-1, J3-21, J3-22 | Ground | |

---

## LEDC PWM Channels

| Channel | GPIO | Function | Frequency | Resolution |
|---------|------|----------|-----------|------------|
| 1 | 3 | Throttle servo | 50 Hz | 14-bit |
| 2 | 9 | Transmission servo (gear selector) | 50 Hz | 14-bit |
| 3 | — | FREE | — | — |
| 4 | 7 | Brake RPWM | 10 kHz | 8-bit |
| 5 | 6 | Brake LPWM | 10 kHz | 8-bit |
| 6 | — | RESERVED/FREED (`LEDC_CH_STEER_RPWM`, was steering RPWM) | — | — |
| 7 | — | RESERVED/FREED (`LEDC_CH_STEER_LPWM`, was steering LPWM) | — | — |

---

## UART Allocation

| UART | RX | TX | Baud | Purpose |
|------|----|----|------|---------|
| UART0 | 44 | 43 | 115200 | **Debug console** via the on-board USB-UART bridge ("COM" port) |
| UART1 | 8 | 15 | 115200 | MAVLink 2 ↔ Pixhawk TELEM2 (non-inverted) |
| UART2 | 5 | 4 | 115200 | VESC steering driver (Flipsky 75200, brushed-DC mode) |

---

## Other Peripherals

| Peripheral | Pins | Details |
|------------|------|---------|
| I2C `Wire` | SDA 41 / SCL 42 | 100 kHz (`I2C_BUS_FREQ_HZ`). AS5600 `0x36` is the **only** device. Opened once in `main.cpp`; no driver re-opens it |
| SPI (CAN) | MOSI 11 / MISO 13 / SCK 12 / CS 10 | MCP2515 + transceiver, 8 MHz crystal → `MCP_8MHZ` |
| PCNT | 17 | Hall speed sensor, single edge, 1 µs hardware glitch filter, hardware overflow accumulation |

---

## ⚠ Warnings

| GPIO | Issue |
|------|-------|
| **0** | Strapping pin — must be HIGH at boot for normal operation |
| **19, 20** | Native USB D−/D+ pins, used here as plain GPIO for the R/N gear switches — the native USB CDC console **MUST NOT** be enabled (never add `ARDUINO_USB_CDC_ON_BOOT`) |
| **38** | Also drives the onboard WS2812B RGB LED — it will flicker when relay 3 toggles |
| **39** | Also JTAG MTCK — hardware JTAG unused here |
| **41, 42** | JTAG TDI/TMS — hardware JTAG unavailable while these are the AS5600 I2C bus |
| **43, 44** | UART0 debug console — in use, do not reassign |
| **45, 46** | Strapping pins — read at reset; safe after boot but initialize carefully |
