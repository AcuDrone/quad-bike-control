# Change: Redesign the web portal as a sectioned service console

## Why
The web portal (`data/index.html`) has grown to a single long page of ~14 cards in one grid:
read-only diagnostics, calibration actions, bench tests and maintenance are interleaved, long
instruction paragraphs push the controls down, and the steering card mixes the jog controls with
a dozen diagnostic rows. The page is used only by the engineer during setup, calibration and
diagnostics — the operator drives through ArduPilot and never opens it — so it should be organised
as a **service console**, not as an operator dashboard.

The approved design is the clickable mockup `ui-mockups/C-sidebar.html` (generated from the current
`data/index.html` by `build_c.py`; both in the design session scratchpad). This change specifies it.

## What Changes
- **Shell**: a left sidebar with the brand "Сервісна консоль" / "Service console", four section
  buttons, and the WebSocket and firmware status items at the bottom of the sidebar. The status
  bar and the "QuadBike Control Portal" header are replaced. The language toggle stays, in the
  content header next to the section title.
- **Exactly four sections, one visible at a time**: Діагностика / Diagnostics (default),
  Калібрування / Calibration, Тести / Tests, Система / System. The last section is remembered in
  `localStorage` (guarded by try/catch; an unknown or missing value falls back to Diagnostics).
- **Діагностика (read-only)**: an "Увага" / "Attention" box listing active problems (sensorless
  steering, VESC down/fault, range warning, MAVLink inactive, board I/O fault, low VESC input
  voltage, low 12 V ECU voltage, no ECU/CAN link; hidden when there are none); Telemetry; a new
  "Кермо: стан" / "Steering: state" card as a 2-column tile grid (including a "Напруга VESC" tile)
  plus one monospace line of raw LWS bits and counters; Vehicle Data (CAN, incl. the ECU probe);
  MAVLink Command Channels; a read-only "Швидкість" / "Speed" card.
- **Калібрування**: steering calibration (jog, nudge, jog speed, Set left/right, Set sensor zero,
  angle, limits, one status line — the diagnostic rows move to Diagnostics); throttle calibration
  with the idle/full sliders always visible but locked until "Почати"; a simplified gear-position
  card (one servo slider + four always-enabled "Зберегти як R/N/H/L" buttons + "▶ R/N/H/L" row +
  boost RPM target; the slider and ▶ row unlock with Почати and re-lock on Готово or WebSocket loss);
  gear sensor codes with the same 2×2 button style; the speed sensor settings card.
- **Тести**: a one-line bench note, Manual control, Ignition & lighting, and the relay test as
  eight equal gear-style buttons (energised = highlighted), 2 columns, 1 column on mobile.
- **Система**: System maintenance (debug, boost test, OTA, "Що нового" → existing changelog modal)
  and a read-only "Мережа" / "Network" placeholder card (SSID and IP from firmware telemetry,
  "settings later").
  No separate "What's new" card.
- **Help**: long instruction paragraphs leave the cards. Each affected card gets an ⓘ button in its
  title that opens one shared help modal (same styles as the changelog modal), rendering the
  existing i18n help strings. Short one-line hints and state-dependent warning boxes stay.
- **Mobile (≤768 px)**: the sidebar becomes a sticky top bar with a ☰ menu and a green/red
  WebSocket connection dot; cards are one column.
- **Firmware** (no control behaviour change, no new WebSocket command):
  - `wifi_ssid` / `wifi_ip` telemetry keys from `WIFI_AP_SSID` / `WIFI_AP_IP` for the Network card
  - `steer_vin` telemetry key: steering VESC input voltage from `SteeringController::getInputVoltage()`
  - `steer_vin_low` / `ecu_voltage_low` telemetry flags computed against two new constants
  - new constants `STEER_VIN_LOW_V` (default 21.5 V, adjustable per pack build) and
    `ECU_VOLTAGE_LOW_V` (default 12.0 V, lead-acid starter battery) in `Constants.h`
  - `FIRMWARE_VERSION` 1.3.0 → 1.4.0 and a `Changelog.h` entry (uk + en, "no recalibration
    required"), as `project.md` requires

## Relationship to other pending changes (archive order matters)
This change modifies requirements that exist **only as deltas in other unarchived changes**, and
its MODIFIED blocks are written against the state those deltas produce:

| Requirement modified here | Introduced by (pending) |
|---|---|
| `web-control` / Steering Calibration and Sensor Zero via Web Interface | `replace-aztec-steering-sensor-with-bosch-lws` |
| `web-telemetry` / Steering Angle Sensor Telemetry | `replace-aztec-steering-sensor-with-bosch-lws` |
| `web-control` / Gear Code Calibration via Web Interface | `update-aztec-gear-code-inputs` |
| `web-control` / Web Relay Test Control | `update-aztec-relay-assignment` |

MODIFIED blocks keep every scenario name of the requirement they replace (scenario bodies are
updated), so the archiver does not drop scenarios. `openspec validate --strict` can check that
only against `openspec/specs/`, so for the four requirements above it becomes meaningful only once
their prerequisite change is archived: re-run it after each prerequisite archive.

It must therefore be archived **after** `replace-aztec-steering-sensor-with-bosch-lws` (itself
archived after `add-aztec-dual-as5600-steering` and `update-aztec-vesc-uart0`),
`update-aztec-gear-code-inputs` and `update-aztec-relay-assignment`. Firmware 1.3.0 (the LWS
change) is already committed (`e466566`); this change ships as 1.4.0 on top of it.

## Impact
- Affected specs:
  - `web-control`:
    - ADDED: Service Console Navigation, Service Console Section Layout, Card Help Modal,
      Throttle Calibration Card
    - MODIFIED: Gear Position Default Configuration via Web Interface, Steering Calibration and
      Sensor Zero via Web Interface, Gear Code Calibration via Web Interface, Web Relay Test Control
  - `web-telemetry`:
    - ADDED: Diagnostics Attention Summary, Speed State Card, Network Identity Telemetry,
      Steering VESC Input Voltage Telemetry
    - MODIFIED: Steering Angle Sensor Telemetry, Firmware Version Display, Gear Default Positions
      in Telemetry, Speed-Limit and Limiter-Ceiling Telemetry
  - `web-server`:
    - MODIFIED: Language Toggle Control
- Affected code:
  - `data/index.html` (markup, CSS, JS, `en`/`uk` dictionaries) — the bulk of the change
  - Firmware, one item each:
    1. `include/Constants.h`: `FIRMWARE_VERSION` "1.3.0" → "1.4.0"
    2. `include/Changelog.h`: new 1.4.0 entry (uk + en)
    3. `include/Constants.h`: new `STEER_VIN_LOW_V` (21.5 V) and `ECU_VOLTAGE_LOW_V` (12.0 V)
    4. `include/WebPortal.h` telemetry struct + `src/TelemetryManager.cpp` + `src/WebPortal.cpp`:
       `wifi_ssid`, `wifi_ip` (next to `firmware_version`)
    5. same files: `steer_vin` (from `SteeringController::getInputVoltage()`, next to the other
       steering VESC fields) and `steer_vin_low`
    6. same files: `ecu_voltage_low`, inside the CAN-connected block next to `ecu_voltage`
  - `WEB_PORTAL_SETUP.md` (portal description: sections, help buttons, 192.168.10.1)
- Spec text corrections with no firmware change: `Gear Position Default Configuration via Web
  Interface` and `Gear Default Positions in Telemetry` described gear defaults as encoder counts
  (0–10 000) and a `{"ok","gear","position"}` response; the firmware has used servo percent
  (0.0–100.0 %) and a plain success message for some time, and the MODIFIED text now says so.
- Not affected: all other `src/` code, the WebSocket commands, the existing telemetry keys, NVS layout,
  calibration data. No recalibration is required. The LittleFS image and the firmware must both
  be flashed (the version and changelog live in the firmware).
