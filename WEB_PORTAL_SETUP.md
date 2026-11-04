# Web Portal Setup Guide

## Overview
The web portal provides a WiFi-based interface for monitoring telemetry, manual control, and OTA firmware updates for the QuadBike control system.

## Features
✅ Real-time telemetry display (5 Hz updates)
✅ Manual control when S-bus inactive (gear selection, steering, throttle)
✅ OTA firmware updates
✅ Input source priority: S-bus > Web > Fail-safe
✅ Mobile-responsive web interface

## Quick Start

### 1. Build and Upload Firmware

```bash
# Build the project (this will download all libraries)
pio run

# Upload firmware to the ESP32-S3
pio run -t upload

# Monitor serial output
pio run -t monitor
```

### 2. Upload Filesystem (Web Interface)

The web interface HTML file must be uploaded to the ESP32's LittleFS filesystem:

```bash
# Upload filesystem image containing data/index.html
pio run -t uploadfs
```

**Important:** You must upload the filesystem after any changes to `data/index.html`.

### 3. Connect to WiFi

After the ESP32 boots:

1. **WiFi Network:**
   - SSID: `QuadBike-Control`
   - Password: None (open network)
   - IP Address: `192.168.10.1`

2. **Connect your device:**
   - On your laptop/phone, connect to the `QuadBike-Control` WiFi network
   - Open a web browser and navigate to: `http://192.168.10.1`

### 4. Use the Web Portal

The portal is a **service console** for the engineer (setup, calibration, diagnostics). The
operator drives through the autopilot and does not need it.

**Layout.** A sidebar on the left holds the brand "Сервісна консоль" / "Service console", four
section buttons and, at the bottom, two status items: the WebSocket state (Connected /
Disconnected) and the firmware version with the board name (e.g. `1.5.2 · Control_v0`; click it to open
"What's new"). Exactly one section is shown at a time; the page remembers the last one in the
browser (`localStorage` key `ui_section`) and opens on **Diagnostics** the first time, or when the
browser blocks storage (private window). All cards keep updating in the background, so switching
sections never shows stale values. The language toggle (`УК` / `EN`) sits in the header next to
the section title.

**Help (ⓘ).** Long instructions are not printed in the cards any more. Cards that have them show a
small ⓘ button in the title; it opens a help window with that card's instructions. Close it with ✕,
a click outside, or Escape. One-line hints and warnings that depend on the current state (MAVLink
control active, steering jog without a sensor, a steering limit beyond ±550°) stay in the cards.

**Phone / narrow screen (≤ 768 px).** The sidebar turns into a top bar with a ☰ button, a round
connection dot (green = WebSocket connected, red = disconnected) and the current section name.
☰ opens the section list together with the WebSocket and firmware items; choosing a section closes
it. Cards are shown in one column.

#### Section by section

**Діагностика / Diagnostics** (read-only; the only button is "Probe ECU"):
- **Увага / Attention** box, shown only while something is wrong: steering sensor unhealthy (jog
  without stall protection), steering VESC down or faulted, a steering limit beyond ±550°, MAVLink
  inactive, board I/O fault, low steering VESC input voltage, low 12 V battery (X10 sense; shown
  whether or not the ECU link is up, only while a battery is seen), no ECU link.
  The voltage thresholds are firmware constants (`STEER_VIN_LOW_V`, `BAT12_LOW_V` /
  `BAT12_LOW_CLEAR_V` in `include/Constants.h`); the page only shows the flags the firmware sends.
- **Telemetry**: gear and its sensor code, servo position, brake, throttle (µs), steering, speed,
  MAVLink link.
- **Кермо: стан / Steering: state**: tiles for angle, rate, left/right limits, sensor state (with
  the "mirrored" marker for a mirrored sensor mount), calibration, VESC driver, sensor zero, motor current, FET temperature and VESC input voltage, plus
  one line of raw LWS status bits and error counters.
- **Vehicle Data (CAN)**: engine data from the ECU, engine hours and trip hours, CAN status, and
  "Probe ECU" with its results. The **Engine RPM** tile shows the EFFECTIVE RPM (`engine_rpm`,
  tacho when fitted, CAN otherwise) whether or not CAN is connected, with a small **TACHO** / **CAN**
  badge (`rpm_source`); "—" when no source is valid.
- **MAVLink Command Channels**: link rate/age and channels 1–8.
- **Швидкість / Speed**: speed and sensor state, speed limit and limiting, steering authority and
  scaling base, the bench test-speed state, odometer and trip.

**Калібрування / Calibration**:
- **Steering calibration**: jog / nudge / jog speed, Set Left, Set Right, Set sensor zero, the
  angle, the limits and one sensor | calibration status line. The sensor direction (normal or
  mirrored mount) is detected from the first limit capture and shown as "дзеркально" / "mirrored"
  next to the sensor state; right after that capture check that Jog ▶ makes the angle grow. After
  re-mounting the sensor: jog straight ahead, Set sensor zero, then capture both limits.
- **Throttle calibration**: the idle and full sliders always show the saved endpoints and are
  locked; "Start Calibration" unlocks them (engine off), then Save or Cancel.
- **Gear position defaults**: one servo slider (0–100 %). Press **Start** to unlock it; dragging
  moves the transmission servo (at most 10 commands per second). **Save as R/N/H/L** stores the
  slider value for that gear (always available, it does not move the servo); the button of the
  gear at the current position is highlighted. The ▶ R / N / H / L row moves the servo to a saved
  position. **Done**, or a lost WebSocket connection, locks the slider again. The RPM boost target
  is set in the same card.
- **Gear sensor codes** (aztec only — shown only when the firmware sends `gear_codes`; hidden on the
  quad bike, which senses one line per gear): live bits A/B/C and the decoded gear; **Save as
  R/N/H/L** assigns the live code to a gear (each button shows the stored code, "(default)", or
  "not set"); "Reset to defaults".
- **Speed sensor**: pulses per revolution (with the measurement block, see below) and wheel
  circumference.
- **Speed limit and steering**: speed-limiter floor, speed-limit readout, steering authority,
  steering scaling base and bench test speed.
- **Оберти (TACHO) / Tacho (RPM) calibration** (aztec): live frequency, pulses per revolution,
  tacho RPM and RPM source; reference-RPM calibration, a manual ppr select and Reset (see below).

**Тести / Tests** (bench checks):
- **Manual control**: Take / Release control, gear buttons, steering, throttle and brake sliders.
  Disabled while MAVLink control is active.
- **Ignition & lighting**: ignition OFF / ACC / START, front light, wheel lock (if fitted), the
  **12 V battery** tile and its calibration, and their telemetry. (aztec has no boost board: the
  24 V rail button and reading are gone; `set_boost` is still accepted but only replies "Boost rail
  not fitted".)
- **Relay test**: eight buttons "Relay 1…8" with the relay function (quad bike labels when the
  wheel lock is fitted, aztec labels otherwise); an energised relay is highlighted. Switching a
  relay on needs ignition OFF, light off, the wheel lock released (if fitted) and a healthy relay
  board.

**Система / System**:
- **System maintenance**: debug output on/off, RPM boost test, firmware upload (OTA), "What's new".
- **Мережа / Network**: the access point SSID and IP address as reported by the firmware
  (read-only; network settings will be added later).

**Odometer / trip (read-only):** the Speed card **displays** the total odometer (`odo_km`) and the
trip distance (`trip_km`), both in km to 3 decimals and both shown whatever the sensor's health —
distance already driven is not invalidated by the sensor going quiet. The portal offers **no reset
control**: clearing the trip is a **ground-station action only** (`MAV_CMD_USER_1`, see
`MAVLINK_SETUP.md` → "Trip reset"), because the operator drives from the GCS and not from the
vehicle's own WiFi. The odometer cannot be reset at all.

**Engine hours (read-only):** the Vehicle Data (CAN) card **displays** the engine hour meter
(`engine_hours`) in hours to 2 decimals, shown whatever the CAN status — accumulated running time is
not invalidated by the bus going quiet, it merely stops growing — and the portal offers **no reset**,
because the meter has no reset on any interface (see `MAVLINK_SETUP.md` → "Engine hours"). The trip
hours ("мотогодини місії", `engine_trip_hours`) are displayed the same way directly beneath it, also
read-only: they are zeroed by the **ground-station** trip reset, the same `MAV_CMD_USER_1` that
clears the trip distance.

**Speed limit (read-only):** the throttle limiter's ceiling comes **only** from the autopilot's
`SPEED_MAX` parameter (`speed_limit_kmh` in telemetry; `0` / "none" means no limit). The web
portal has no maximum-speed box and no limiter on/off button.
See `MAVLINK_SETUP.md` → "Autopilot speed limit (`SPEED_MAX`)".

**Limiter floor (Speed limit and steering card):** the one limiter setting — the lowest throttle the speed
limiter may leave at and above the limit, 0–20 % (default **0 %** = the calibrated idle position,
which holds the limit most tightly). A higher floor softens engine braking at the cost of a limit
that may not hold. Saved with `set_speed_limit_floor` to NVS `"speed"` / `thr_floor`, reported as
`speed_limit_floor`; out-of-range values are refused by the page and by the firmware. The floor
alone never limits: with no `SPEED_MAX` the throttle passes through. The limiter row ("Limiting
X %", with "at floor" when the ceiling sits on the floor) turns into a warning badge **"Limiter at
floor, speed above limit"** when the ceiling has been at the floor for 2 s while the speed stays
above the limit (`speed_limit_sat`), and reads "Limiter was at floor…" for 10 s after it clears
(`speed_limit_sat_held`, held by the firmware so every client shows it). It means the floor is too
high or the vehicle is going downhill — the limiter does not brake.

**12 V battery (Ignition & lighting card):** the battery (+) is wired to X10 pin 3 and measured on
GPIO9 through the 200K/24K divider (÷9.33); it is the "ECU voltage" (`ecu_voltage`, V, calibrated)
because the Delphi MT05 ECU never answers OBD PID 0x42, and it is reported **whether or not CAN is
connected**. Telemetry: `ecu_voltage`, `ecu_voltage_low`, `bat_v_raw` (V before calibration),
`bat_v_cal` (factor), `bat_v_valid` (false = nothing on X10, raw ≤ 5 V; the tile then reads "not
connected"). The tile shows the calibrated voltage and "raw X.XX V × cal 1.000". **Calibrate:**
measure the battery terminals with a multimeter, type the reading (8–16 V, step 0.01) and press
**Calibrate** — the page sends `cal_bat_v` with `value` = the measured volts; the firmware computes
`cal = measured / raw` from its current averaged raw reading, refuses a factor outside 0.80–1.20 or a
missing battery, saves it to NVS `"bat12"` / `cal` and applies it at once. **Reset** sends
`reset_bat_v_cal` (factor 1.0, saved). The "Attention" box shows "12 V voltage low" below 11.8 V
(cleared above 12.1 V) only while `bat_v_valid`, never while the starter relay is on and for 2 s
after.

**Tacho (RPM) calibration card:** engine RPM comes from the ECU tachometer wire (grey J1 pin 6)
through a PC817 on X3 / GPIO14 (wiring: `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §7a); CAN RPM is the
fallback. The card shows `tacho_hz` (one decimal), `tacho_ppr`, `tacho_rpm` and `rpm_source`, and
"немає імпульсів" / "no pulses" while `tacho_hz` is 0. **Calibrate:** with the engine running, type
the reference RPM (300–8000) from the dash — or from the crank sensor, RPM = crank Hz × 60 / 23 —
and press **Calibrate**: the page sends `cal_tacho_ppr` with `value` = the reference; the firmware
computes `raw = Hz × 60 / reference` from its current averaged frequency, snaps it to the nearest of
0.5, 1, 2, 3, 4 and replies e.g. "Tacho: 26.1 Hz × 60 / 1550 = 1.01 → ppr 1". It refuses (old ppr
kept) with no pulses, a reference outside 300–8000, or `raw` more than ±25 % from the snapped value.
**Apply** sends `set_tacho_ppr` with the selected ppr (only 0.5/1/2/3/4 accepted). **Reset** sends
`reset_tacho_ppr`: the stored key is removed and ppr goes back to 1. The ppr lives in NVS `"tacho"` /
`ppr`; storing it also marks the tacho fitted from boot (a stopped engine then reads 0 from the tacho
instead of CAN). All three commands work from any input source; replies start with "Tacho" and are
shown in the card's status line. Telemetry (always emitted, not gated on CAN): `engine_rpm`
(effective), `rpm_source` ("TACHO" / "CAN" / "NONE"), `tacho_hz`, `tacho_ppr`, `tacho_rpm`.
Since fw 1.5.10 the RPM is the **median** of the last 8 periods (in 1.5.10 only, edges closer than
0.5 × that median to the last accepted edge were also rejected — removed in 1.5.11, see below). A monospace **Діагностика / Diagnostics**
block under the tiles shows the raw signal for remote diagnosis — telemetry: `tacho_periods` (string,
the last 8 accepted periods in ms with one decimal, oldest first, e.g. `"42.9 43.1 21.4 21.6 42.8"`,
`""` before the first period; kept after the engine stops), `tacho_min_ms` / `tacho_max_ms` (float,
shortest / longest accepted period in the last 1 s window, 0 = none), `tacho_acc` / `tacho_rej`
(integers, accepted / rejected edges per second over that window) and `tacho_rej_total` (rejected
edges since boot). Healthy signal: alike periods and `tacho_rej` ≈ 0; halves (21 ms among 43 ms) or a
high `tacho_rej` mean noise on the TACHO wire — see `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §7a.
Since fw 1.5.11 the lock window is gone: the ISR sees both edges and a rising edge counts only when
the LOW before it and the HIGH after it each last ≥ 2 ms (`TACHO_MIN_LEVEL_US`); bounce on an edge,
glitches inside a level and narrow spikes are counted in `tacho_rej`. The block gains a line
**high / low, ms: 10.7 / 10.9 (high min/max 10.2 / 11.1)** — telemetry `tacho_hi_ms` (float, the last
high span), `tacho_lo_ms` (float, the last validated low), `tacho_hi_min_ms` / `tacho_hi_max_ms`
(float, shortest / longest high span in the last 1 s window); all ms with one decimal, 0 = none yet.
**Reading the ppr from it** (the ECU TACHO is a ≈50 % duty square wave): high ≈ low ≈ **10.7 ms**
with periods ≈21.4 ms (≈46 Hz at a 1400 RPM idle) means the ECU gives **2 ppr** (the 1.5.10 doubles
were missed pulses); high ≈ low ≈ **21.5 ms** with periods ≈43 ms (≈23 Hz) means **1 ppr** (the
1.5.10 halves were a spurious edge at the slow falling edge, now rejected). Then calibrate against
the dash. A `tacho_hi_min_ms` well below `tacho_hi_ms` means glitches inside the high pulse.

**Measuring pulses per revolution (Speed sensor card):** lift the wheel off the ground, put the
transmission in neutral and put a chalk mark on the tyre. Enter the number of turns (1–20, default
5), press **Start**, turn the wheel by hand exactly that many turns in one direction, wait until the
live count stops changing and press **Stop**. The page computes the result from the firmware's
running pulse total (`speed_pulse_total` in telemetry; the firmware keeps no measurement state) and
shows "N / turns = x.x → R". A fractional part of 0.2 or more is flagged: the turns were inexact or
pulses were missed, so repeat. The sensor has no direction channel, so turning back also **adds**
pulses: on an overshoot, Cancel and start again. The hand-turned distance is added to the odometer
and trip like any pushed distance (about 2 m per turn). Nothing is stored until **Save as
pulses/rev**, which sends the usual `speed_cal_ppr`; a disconnect or a page reload drops the
measurement.

**OTA Firmware Update** (Система → System maintenance):
1. Build new firmware: `pio run`
2. Locate firmware binary: `.pio/build/esp32-s3-devkitc-1/firmware.bin`
3. Click "Click to select firmware (.bin)" on web portal
4. Select the firmware.bin file
5. Click "Upload Firmware"
6. Vehicle will enter safe state, update, and reboot automatically

## Input Source Priority

The system enforces this priority hierarchy:

1. **S-bus (Highest)** - When S-bus signal is active, web control is disabled
2. **Web** - When S-bus inactive, web commands are accepted
3. **Fail-safe (Lowest)** - When both sources inactive, vehicle enters safe state:
   - Steering: Center (90°)
   - Throttle: Idle (0%)
   - Brakes: Applied
   - Transmission: Neutral

## Configuration

All settings are in [include/Constants.h](include/Constants.h):

```cpp
// WiFi Configuration
#define WIFI_AP_SSID          "QuadBike-Control"
#define WIFI_AP_PASSWORD      ""  // Open network
#define WIFI_AP_IP            IPAddress(192, 168, 10, 1)

// Telemetry Rate
#define TELEMETRY_INTERVAL    200  // ms (5 Hz)

// OTA Configuration
#define OTA_HOSTNAME          "quadbike-control"
#define OTA_PASSWORD          ""   // No password
```

## Troubleshooting

### WiFi AP not appearing
- Check serial monitor for "Web portal initialized" message
- Verify ESP32 has booted successfully
- Check if WiFi is disabled in your region's frequency band

### Cannot access web interface (http://192.168.10.1)
- Ensure you're connected to the `QuadBike-Control` WiFi network
- Check that filesystem was uploaded: `pio run -t uploadfs`
- Check serial monitor for "SPIFFS mounted" and "index.html" found messages
- Try accessing by IP: `http://192.168.10.1/index.html`

### Telemetry not updating
- Check WebSocket connection indicator (should show "Connected")
- Open browser developer console (F12) and check for WebSocket errors
- Verify control loop is running (serial monitor should show activity)

### Web control not working
- Check input source indicator - must show "Web" (not "S-bus" or "Fail-safe")
- If S-bus is active, disconnect S-bus to enable web control
- Check that manual control buttons/sliders are enabled (not greyed out)

### OTA update fails
- Ensure firmware file is valid `.bin` file from PlatformIO build
- Check that device has enough free space for new firmware
- Try OTA update via Arduino IDE OTA (hostname: `quadbike-control`)
- If update fails, device will automatically rollback to previous firmware

### Build errors
If you see library-related errors:

```bash
# Clean and rebuild
pio run -t clean
pio run
```

Common missing libraries are installed automatically by PlatformIO:
- ESPAsyncWebServer
- AsyncTCP
- ArduinoJson
- SPIFFS (built-in)

## Serial Commands

Serial commands still work alongside web portal:

**Actuator Control:**
- `s<angle>` - Set steering (0-180°)
- `t<angle>` - Set throttle (0-180°)
- `g<speed>` - Transmission actuator (-255 to +255)
- `b<speed>` - Brake actuator (-255 to +255)

**Gear Selection:**
- `R` - Reverse gear
- `N` - Neutral gear
- `L` - Low gear
- `H` - High gear
- `G` - Display current gear

**Safety:**
- `x` - Emergency stop
- `c` - Safe position (center/idle/stop)

## Architecture

```
ESP32-S3
├── WiFi AP (192.168.10.1)
├── AsyncWebServer (port 80)
│   ├── HTTP: Serves /index.html
│   └── WebSocket (/ws): Real-time telemetry & commands
├── ArduinoOTA (port 3232)
└── Control Loop (100 Hz)
    ├── Input source priority
    ├── Telemetry broadcasting (5 Hz)
    └── Web command processing
```

## Files

**Implementation:**
- [include/WebPortal.h](include/WebPortal.h) - WebPortal class declaration
- [src/WebPortal.cpp](src/WebPortal.cpp) - Web server, WebSocket, OTA
- [data/index.html](data/index.html) - Web interface (HTML/CSS/JS)
- [src/main.cpp](src/main.cpp) - Integration and control loop

**Configuration:**
- [include/Constants.h](include/Constants.h) - WiFi and web server constants
- [platformio.ini](platformio.ini) - Library dependencies and build config

## Safety Notes

⚠️ **Important Safety Considerations:**

1. **Web control is secondary** - S-bus always has priority when active
2. **Open WiFi network** - Anyone within range can connect (suitable for single operator)
3. **Manual control testing** - Test in safe environment before field use
4. **OTA updates** - Vehicle enters safe state during updates (brakes applied, neutral)
5. **Fail-safe behavior** - System defaults to safe state when no control source active
6. **Control loop timing** - Web portal designed not to interfere with 100 Hz control loop

## Next Steps

1. **Test WiFi AP** - Verify you can connect and access web interface
2. **Test telemetry** - Check that all vehicle data displays correctly
3. **Test manual control** - With S-bus disconnected, test gear/steering/throttle
4. **Test S-bus priority** - Connect S-bus and verify web control is disabled
5. **Test OTA update** - Perform test firmware update in safe environment
6. **Load testing** - Connect multiple clients and verify performance

## Support

For issues or questions:
- Check serial monitor output for detailed logs
- Review OpenSpec documentation: `openspec/changes/add-web-portal/`
- Submit issues to project repository

---

**Access URL:** http://192.168.10.1 (after connecting to QuadBike-Control WiFi)
