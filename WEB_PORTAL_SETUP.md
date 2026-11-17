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

> ⚠ **`uploadfs` erases the event journal.** The journal files (`/events.log`, `/events.1`) live in
> the same LittleFS partition, so `pio run -t uploadfs` — and a web LittleFS update
> (`/update?type=littlefs`) — wipe them. **Download the journal first** (System → Event journal →
> Download, or `GET /api/events/download`). A normal firmware (application) update keeps it.

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
  inactive, board I/O fault, low steering VESC input voltage, low 12 V (ECU) voltage, no ECU link.
  The voltage thresholds are firmware constants (`STEER_VIN_LOW_V`, `ECU_VOLTAGE_LOW_V` in
  `include/Constants.h`); the page only shows the flags the firmware sends.
- **Telemetry**: gear and its sensor code, servo position, brake, throttle (µs), steering, speed,
  MAVLink link.
- **Кермо: стан / Steering: state**: tiles for angle, rate, left/right limits, sensor state (with
  the "mirrored" marker for a mirrored sensor mount), calibration, VESC driver, sensor zero, motor current, FET temperature and VESC input voltage, plus
  one line of raw LWS status bits and error counters.
- **Vehicle Data (CAN)**: engine data from the ECU, engine hours and trip hours, CAN status, and
  "Probe ECU" with its results.
- **MAVLink Command Channels**: link rate/age and channels 1–8. Ch5, Ch7 and Ch8 show their raw
  value without a function (Ch7 was the wheel lock until 1.5.33).
- **Швидкість / Speed**: speed and sensor state, speed limit and limiting, the speed-governor row
  (with the no-speed banner and the at-idle badge), steering authority and scaling base, the bench
  test-speed state, odometer and trip.

**Калібрування / Calibration**:
- **Steering calibration**: jog / nudge / jog speed, Set Left, Set Right, Set sensor zero, the
  angle, the limits and one sensor | calibration status line. The sensor direction (normal or
  mirrored mount) is detected from the first limit capture and shown as "дзеркально" / "mirrored"
  next to the sensor state; right after that capture check that Jog ▶ makes the angle grow. After
  re-mounting the sensor: jog straight ahead, Set sensor zero, then capture both limits.
- **Throttle calibration**: the idle and full sliders always show the saved endpoints and are
  locked; "Start Calibration" unlocks them (engine off), then Save or Cancel.
- **Engine start** ("Запуск двигуна", since 1.5.33): the pre-crank delay slider ("Pre-crank
  delay", 1–8 s, step 0.5, value on the right, default 3 s; saved on release, no Save button) —
  how long the ignition is on before the starter on an RC or GCS auto-start; the START button in
  Tests → Ignition & lighting has no delay; `set_crank_dwell_ms`, saved to NVS. Under it, since
  1.5.36, the max starter time slider ("Max starter time" / «Макс. час роботи стартера», 2–10 s,
  step 0.5, value on the right, default 5 s; saved on release, no Save button) — the longest the
  starter turns in one start attempt from the RC switch, the GCS or the START button (it stops
  earlier once the engine RPM shows it running); `set_crank_max_ms`, saved to NVS, applies from
  the next attempt; telemetry `crank_max_ms`. No caution line; the ⓘ explains the risk.
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
- **Speed limit and steering**: speed-limiter floor, the throttle rate ("Плавність газу (обмежувач і
  регулятор)" / "Throttle rate (limiter and governor)", `thr_slew`, relabelled in 1.5.35 — it is
  also the speed governor's output and hand-over rate; command and key unchanged), the throttle
  opening rate, speed-limit readout with the limiter and governor rows, steering authority, steering
  scaling base and bench test speed.
- **Speed governor** ("🎯 Регулятор швидкості", since 1.5.35, right after "Speed limit and
  steering"): the governor ON/OFF button and its five settings — see "Speed governor" below.

**Тести / Tests** (bench checks):
- **Manual control**: Take / Release control, gear buttons, steering, throttle and brake sliders.
  Disabled while MAVLink control is active.
- **Ignition & lighting**: ignition OFF / ACC / START, front light, wheel lock (if fitted), 24 V
  boost rail, and their telemetry. Since 1.5.34 the wheel lock is set only here or from the ground
  station, not from the RC transmitter; it stays as set until changed here or from the GCS and is
  released after a reboot.
- **Relay test**: eight buttons "Relay 1…8" with the relay function (quad bike labels when the
  wheel lock is fitted, aztec labels otherwise); an energised relay is highlighted. Switching a
  relay on needs ignition OFF, light off, the wheel lock released (if fitted) and a healthy relay
  board. The subtitle shows the active relay board ("Board: XL9535 @0x20 (auto)"); relay numbers
  mean the same relays on either board.

**Система / System**:
- **System maintenance**: debug output on/off, RPM boost test, firmware upload (OTA), "What's new".
- **Плата реле / Relay board**: Auto / PCA9557 / XL9535 selector, the active board line, a
  "Reboot to apply" note and the detection error (see "Relay board selection" below).
- **Мережа / Network**: the access point SSID and IP address as reported by the firmware
  (read-only; network settings will be added later).
- **Журнал подій / Event journal** (after Network): the last 100 journal lines (monospace, newest
  at the bottom) with a source filter (All, SYS, SRC, STEER, VESC, CAN, I2C, RELAY, SNAP, RTC),
  Refresh, Download and Clear (with a confirm). It refreshes on open and every 5 s while the System
  section is visible. The header shows the self-recovery counters VESC / CAN / I2C (`vesc_resync`,
  `can_reinit`, `i2c_unlock` since boot) and the journal size (`journal_bytes`); when the journal
  cannot write to flash (`journal_bytes` = −1) a red note says so. A **«Запис у flash» / "Write to
  flash"** toggle (default **OFF**, from telemetry `journal_fs`) switches the journal's flash writes;
  the "Storage" row shows "RAM only" / "flash" and the number of lines held in RAM. See "Event
  journal" below.

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

**Speed governor (Calibration → "🎯 Регулятор швидкості" / "Speed governor", since 1.5.35):** with
the switch **ON** and an effective speed limit (`SPEED_MAX` from the autopilot; on the buggy also the
gear ceiling L 45 / H 75 / R 20 km/h) the throttle input — RC channel 2 above centre or the web
throttle slider in Tests → Manual control — is a **speed setpoint**: ≤ 5 % = stop (idle at once),
then from ≈ 5 km/h (at most half the limit) up to the limit at 100 %. A closed loop on the wheel
speed sets the throttle; the ceiling limiter stands down meanwhile. With the switch **OFF**
(default), or ON without a limit, the throttle path is exactly as before.

- **It never brakes.** Downhill it closes to idle; the governor row then shows the badge
  **"Регулятор на холостому, швидкість вища за задану — гальмуйте" / "Governor at idle, speed above
  setpoint — brake"** (after 2 s, kept 10 s after it clears; web only).
- **No speed signal = direct throttle (fail-open).** With an invalid speed reading — also from boot
  until the wheels have turned once — the governor does not act and the throttle moves at the
  throttle rate to the direct value: **full stick = full throttle**. Both speed cards show the red
  banner **"Регулятор не діє: немає сигналу швидкості — газ напряму (повний газ = повний газ)" /
  "Governor not acting: no speed signal — direct throttle (full stick = full throttle)"**, and the
  ground station gets `STATUSTEXT` "Gov: NO SPEED - direct throttle" (after 2 s with the stick
  above 5 %, then every 10 s), and "Gov: speed OK - governing" when it governs again.
- **Bypass:** in N / unknown gear, during a gear change (boost or idle hold), during throttle
  calibration and in fail-safe the governor is bypassed; RC brake and a web brake ≥ 5 % close the
  throttle at once while it governs. The autopilot mode is not consulted (use it in MANUAL).
- **Governor row** (Diagnostics "Speed" and Calibration "Speed limit and steering"): "вимкнено" /
  "off", "немає ліміту — газ напряму" / "no limit — direct throttle", "обхід (N / перемикання)" /
  "bypassed (N / shift)", "немає швидкості — газ напряму" / "no speed — direct throttle", or
  "активний: NN км/год, газ NN %" / "active: NN km/h, throttle NN %". While it is active the
  limiter's "Limiting" row is hidden, and the web throttle slider shows "→ NN км/год" (the setpoint).

Settings (NVS `"speed"`, per vehicle, saved first and applied after a successful write; a
compiled default applies **only until a value is saved** — a stored value is never replaced, also
not by a later firmware with other defaults):

| Setting | Command | NVS key / telemetry | Range | Default, buggy (`aztec`) | Default, quad bike (`master`, `old-board`) |
|---|---|---|---|---|---|
| governor on/off | `set_gov_enabled` (bool) | `gov_en` | — | off | off |
| Kp, % per m/s | `set_gov_kp` | `gov_kp` | 0.2–10 (step 0.1) | 1.0 | 3.0 |
| Ki, %/s per m/s | `set_gov_ki` | `gov_ki` | 0–5 (step 0.1) | 0.35 | 1.0 |
| setpoint acceleration, m/s² | `set_gov_acc` | `gov_acc` | 0.3–3.0 (step 0.1) | 1.0 | 1.0 |
| setpoint deceleration, m/s² | `set_gov_dec` | `gov_dec` | 0.5–5.0 (step 0.1) | 2.5 | 2.5 |
| start throttle limit ("Стартовий газ, макс."), % | `set_gov_launch` | `gov_launch` | 1–40 (step 0.5) | 4 | 10 |

Each setting is a slider: the label with the value in use on the right, then "−", the slider and
"+", with the range ends written under the slider. Releasing the slider sends ONE command (nothing
is sent while dragging); each "−" / "+" tap moves one step and sends one command — use them for the
0.1 steps, which are too fine for a thumb on a phone. Telemetry does not move a slider while it is
held or until it confirms the sent value (≤ 3 s); a refusal, no connection or a disconnect puts the
slider back on the value in use. The sliders stay disabled ("--") until telemetry carries the
values. The page names no default number (it is shared by both vehicles; the sliders show the values
in use). The switch and each setting have their own small ⓘ icon beside the label: it opens a short
help for that one setting — what it is, what "more" and "less" do, and which symptom calls for
changing it, by how much.
Replies: "Speed governor ON/OFF", "Governor Kp set to 1.50", "Governor Kp out of range (0.2-10)",
"Failed to save the governor setting" (same shape for Ki, acceleration, deceleration, start throttle
limit). Accepted from any input source. Telemetry: `gov_en`, `gov_state` (`off` / `nolimit` /
`bypass` / `nospeed` / `active`), `gov_sp` (setpoint, km/h, 0 unless active), `gov_out` (%, 0 unless
active), `gov_sat` and the five settings.

**Field tuning** (web page only; start from this vehicle's defaults — buggy Kp 1.0 / Ki 0.35 / start
4 %, quad bike Kp 3.0 / Ki 1.0 / start 10 %, acc 1.0 / dec 2.5 on both; check on the card that these
are the values in use). Tune at the **10 km/h** limit first, then only check 20 and 30 km/h; one
value at a time, at most five runs per value:
1. *Preparation (governor OFF)*: flat ground, speed sensor calibrated; on the buggy use L. Note the
   throttle % at which the vehicle creeps, the steady throttle at 10 / 20 / 30 km/h, the response to
   a +1 % step at 10 km/h, and the stored throttle calibration (idle / full µs).
2. *Launch*: `SPEED_MAX` 2.78 m/s (10 km/h), governor ON, stick to 100 % from rest. Does not move
   within ≈ 2 s on a slope → raise the start throttle limit by 1 %; lurches when the clutch takes up
   → lower it by 1 %.
3. *Hold 30 s*: **hunts** (±1 km/h or more, period ≈ 2–4 s, throttle swinging with it) → lower Kp by
   ≈ 20 % and set Ki = Kp / 3 (buggy 1.0 / 0.35 → 0.8 / 0.27 → 0.65 / 0.22; quad 3.0 / 1.0 → 2.4 /
   0.8 → 1.9 / 0.65); never raise the throttle rate to cure it. A throttle flicker of ± 0.5 % with a
   calm speed is normal on the quad. **Slow swing** (period ≥ 5 s, throttle in single ≈ 0.3 % steps)
   → the throttle servo resolution limit: lower Ki by 0.1 and note it (a linkage using more servo
   travel is the fix). **Sluggish** (a 1 km/h gap takes > 8 s, a small slope costs > 5 km/h) → raise
   Kp by ≈ 20 % with Ki = Kp / 3; at the first sign of hunting go back one step.
4. *Steps* 30 → 100 → 30 % at 10 km/h: overshoot > 1.5 km/h → acceleration −0.2 m/s²; closing too
   abruptly → deceleration −0.5 m/s².
5. *Hill*: up — a dip of a few km/h recovering within ≈ 10 s is expected; down — the at-idle badge,
   brake.
6. *20 and 30 km/h* with the 10 km/h values; if they hunt while 10 km/h does not, lower Kp until all
   three are calm. The 10 km/h tune wins.
7. *Buggy only*: repeat in L, once in H at 10 km/h (rougher around the clutch take-up — expected) and
   in R (R ceiling 20 km/h).
Record the final values; a default changes in code only for that vehicle's branch.

**Relay board selection (System → Relay board card):** two relay boards are supported on header
X15 / I2C2, one at a time: the custom **PCA9557** board at `0x1F` and the **XL9535-K8V5** at
`0x20`–`0x27` (pads A0–A2, default `0x20`). The firmware picks the board **once at boot**, before any
relay is written, from the stored setting — NVS `"relay"` / `board` (uint8: 0 = `auto`, default;
1 = `pca9557`; 2 = `xl9535`; an absent or out-of-range value means `auto` and is not rewritten). `auto`
uses whichever board answers; with **both** boards, **no** board or **several** XL9535 addresses on
the bus nothing is guessed: no relay is switched on, every relay command is ignored, `io_relay_fault`
is true and the reason is shown. A manual `pca9557` / `xl9535` selects that board when both are
connected (it never picks between two XL9535 addresses) and is a fault if the chosen board is
missing. The selector sends `{"cmd":"set_relay_board","value":"auto|pca9557|xl9535"}` (any input
source, any ignition state); the firmware saves it first and replies "Relay board set to … — takes
effect after reboot" — **the running board never changes until a reboot**. Errors: "Relay board must
be auto, pca9557 or xl9535", "Failed to save the relay board setting". Telemetry (every frame):
`relay_board` (`"pca9557"` / `"xl9535"` / `"none"`), `relay_board_addr` (7-bit address as an
integer, `null` when `"none"`), `relay_board_cfg` (stored setting), `relay_board_reboot` (stored
setting differs from the one used at boot) and `relay_board_error` (`null` or `both_found`,
`none_found`, `multiple_xl9535`, `cfg_mismatch`, `init_failed`). Every relay write is verified on the
expander's pins; after a relay-board reset (e.g. an XL9535 VIN glitch) the firmware re-initializes
the board every second until the relays verify again.

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

**Event journal (System → Event journal):** a persistent post-mortem record kept on LittleFS as two
plain-text files, `/events.log` (active) and `/events.1` (previous), 128 KB each (≈ 1500 lines,
≈ 17–20 powered hours per file at the idle rate). When the active file is full it is renamed to
`/events.1` (the old `.1` is deleted). Lines are buffered in RAM and written to flash at most once per
second, never while the steering motor is being driven (deferred up to 10 s), and just before a
controlled reboot (OTA). The last 16 unflushed lines also sit in RTC memory and are replayed after a
panic / watchdog / software reset behind an `RTC I replay` line; up to ≈ 1 s of events is lost on a
power cut. A boot counter (NVS `journal`/`boot`) numbers the boots.

**Flash-write switch (default OFF).** Everything above about files applies only while «Запис у
flash» / "Write to flash" is on (NVS `journal`/`fs`, uint8, default 0; survives a reboot). While it is
**off** (the default, so an untested vehicle never sees a flash-erase stall from the journal) the
journal does no LittleFS writes at all — no flush, no rotation, no file access at boot — and runs in
RAM: the newest ≈ 4 KB of lines (`JOURNAL_RAM_BUF_BYTES`, ≈ 45–55 lines; oldest lines dropped) plus
the RTC ring (last 16 lines across a reset). Dedupe, SNAP every 60 s, counters and replay work as
usual; `journal_bytes` reports the RAM bytes, `jflush_max` stays 0, the pre-OTA flush is skipped.
**Everything is lost at power-off.** Files from an earlier period with writes on are still served
(read-only) by tail and download, before the RAM lines; with writes off each part is preceded by a
label line `# --- flash: … ---` / `# --- RAM only: … ---`. Switching is journaled so gaps are
explainable: on → `SYS I journal_fs on=1 ram_dropped=N` and the RAM lines are written to flash at
once; off → `SYS I journal_fs on=0`, one final flush, then the files are left alone (both flushes
still wait for the steering motor, at most 10 s). Clear deletes the files in either mode.

Line format (ASCII, ≤ 120 bytes): `<boot> <uptime_s.mmm> <SRC> <LVL> <event> k=v k=v ...`, levels
`I`/`W`/`E`, sources `SYS SRC STEER VESC CAN I2C RELAY SNAP RTC`; each boot starts with
`# boot=N fw=… reset=…`. The same (source, event) is written at most once per 2 s; repeats are counted
as `sup=N` on the next line of that pair. Examples:

```
47 0.412 SYS I boot reset=POWERON fw=1.5.26 heap=131560 prev_heap_min=?
47 612.447 VESC W resync state=PAYLOAD idle=287 bytes_since_good=1412 drop=1
47 980.113 CAN W lws_silent ms=1200 obd_alive=1 -> reinit
47 980.120 CAN I reinit reason=lws_silent result=ok attempt=1
47 2210.551 I2C W bus_stuck sda=0 scl=1 clocks=3 released=1 reinit=1 by=relay err=5 runs=1
47 60.001 SNAP I heap=126840 min=118204 loop_max=4 jflush_max=38 vin=24.0 fet=40.1
47 60.001 SNAP I lws_fps=100 vesc_good=198 drop=0 resync=0 can_reinit=0 i2c_unlock=0
```

The VESC link lines (`link_lost`, `link_up`, `link_down_snap`) use compact keys to stay within the
line limit: `st` parser state, `bsg` bytes since the last good packet, `good`, `crc` / `stop` dropped
packets, `rs` resyncs, `vin`, `fet`, `fault`, `down_ms`. The snapshot is written every 60 s as two
`SNAP` lines.

HTTP API (text/plain, streamed; the main loop never waits for a reader):
- `GET /api/events?tail=N` — the last N lines (default 100, max 500), RAM lines included, oldest
  first. Headers: `X-Journal-Fs` (`0`/`1`) and `X-Journal-Ram-Lines` (lines held in RAM).
- `GET /api/events/download` — `/events.1` then `/events.log`, then the RAM lines, as one attachment
  (`<SSID>-events-boot<N>.log`). Rotation is postponed while it streams.
- `GET /api/events/config` → `{"fs":false}` — the flash-write switch.
- `POST /api/events/config` with `{"fs":true}` / `{"fs":false}` → `{"success":true,"fs":…}`
  (400 `{"success":false,"message":…}` for invalid JSON or a missing boolean `fs`). Applied by the
  main loop on its next iteration (persisted to NVS, `journal_fs on=…` line, flush).
- `POST /api/events/clear` — deletes both files and the RAM buffer (executed by the main loop) and
  writes `SYS I journal_cleared`; answers `{"ok":true}`. The boot counter is kept.

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
- A filesystem (LittleFS) update erases the event journal — download it first

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
