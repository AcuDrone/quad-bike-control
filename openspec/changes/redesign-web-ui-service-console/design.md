## Context
`data/index.html` is a single self-contained page (inline CSS, JS and the `en`/`uk` dictionaries)
served from LittleFS. It talks to the firmware over one WebSocket (5 Hz telemetry + JSON commands)
and `GET /api/changelog`. The audience is the service engineer on the bench; the operator never
opens it. The approved layout is the mockup `ui-mockups/C-sidebar.html`, produced from the current
page by `build_c.py`, which documents every structural transformation (card moves, removed help
paragraphs, new i18n keys, new JS helpers). The mockup is a demo: it runs on canned telemetry with
`mockMode` shortcuts and mirrors duplicated readouts by copying `-cal` suffixed ids. The
implementation SHALL NOT copy those demo shortcuts (see D7).

Baseline: the state produced by the pending changes `replace-aztec-steering-sensor-with-bosch-lws`
(firmware 1.3.0, committed as `e466566`), `update-aztec-gear-code-inputs` and
`update-aztec-relay-assignment`.

## Goals / Non-Goals
- Goals: four task-oriented sections; read-only diagnostics separated from actions; shorter cards
  with help on demand; consistent big-button patterns for "save as gear" and relays; usable at
  375 px; no firmware behaviour change.
- Non-Goals: an operator dashboard or overview tiles; network settings (only a placeholder card);
  new WebSocket commands, or telemetry keys beyond those listed in D12a/D12b; changing any firmware gate, timeout or validation;
  splitting `index.html` into several files or adding a framework/CDN.

## Decisions

### D1. Section model and routing
- Four `<section class="section" data-section="diag|calib|tests|system">` blocks inside one
  `<main>`; exactly one has `.active` (`display:block`), the rest are `display:none`. All cards
  remain in the DOM, so the existing `updateTelemetry()` keeps updating hidden cards and switching
  sections never shows stale data.
- `showSection(id)`: validates `id` against `SECTIONS = ['diag','calib','tests','system']`
  (unknown → `diag`), toggles `.active` on sections and nav buttons, writes
  `localStorage['ui_section']` inside try/catch, updates the content-header title and the mobile
  top-bar title from `t('nav_' + id)`, closes the mobile menu and scrolls to top.
- No URL hash routing (kept simple; one engineer, one page). Alternative considered: `#calib`
  hashes — rejected for now, trivial to add later.
- Start-up order: `applyLanguage(...)` → `showSection(saved || 'diag')` → `connectWebSocket()`.

### D2. Read-only vs action split
- **Діагностика** contains no control that sends a command, except the existing "Probe ECU" button
  inside the CAN card (a diagnostic read, kept with its results panel).
- Every command-sending control lives in Калібрування, Тести or Система.
- Card → section map (authoritative for the implementation):
  - diag: Attention box, Telemetry, Кермо: стан, Vehicle Data (CAN), MAVLink Command Channels,
    Швидкість (read-only)
  - calib: Steering calibration, Throttle calibration, Gear position defaults, Gear sensor codes,
    Speed sensor settings
  - tests: bench note, Manual control, Ignition & lighting, Relay test
  - system: System maintenance, Мережа (placeholder)

### D3. Shared help modal
- One modal `#help-modal` reusing `.modal-overlay` / `.modal` / `.modal-header` / `.modal-close`
  from the changelog modal. `openHelp(keys, titleKey)` stores `{keys, titleKey}`; `renderHelp()`
  sets the title from `t(titleKey)` and the body to one `<p>` per key with `innerHTML = t(key)`
  (the dictionary strings are trusted, compiled into the page, and some already carry `<b>`).
- Closes on ✕, overlay click and Escape. `applyLanguage()` calls `refreshHelp()`, which re-renders
  an open modal and re-localises the ⓘ buttons' `aria-label`.
- ⓘ button: `<button type="button" class="help-btn">` at the right end of the card `<h2>`, with a
  localized `aria-label`/`title` (`help_label`).
- Card → help keys: steering cal `steer_cal_help`; throttle cal `throttle_cal_help`; gear defaults
  `gear_defaults_help`; gear codes `gear_codes_help`; speed sensor `speed_sensor_help`,
  `speed_limiter_hint`, `steer_scale_hint`, `steer_test_speed_hint` (one modal, four paragraphs);
  manual control `web_control_hint`; ignition `boost_rail_hint`; relay test `relay_test_help`;
  system maintenance `fw_update_warning`.
- Stays in the cards: one-line field labels/hints, the sensorless-jog warning in steering cal,
  the "MAVLink control is active" warning in manual control, status lines, the ⚠ range notes.

### D4. Throttle calibration pattern
- The idle/full sliders are always rendered, showing `throttle_idle_us` / `throttle_full_us` from
  telemetry, `disabled` and visually muted (`.is-locked`) while `throttle_calibrating` is false.
- "Почати" sends the existing `throttle_cal_begin`; while `throttle_calibrating` is true the
  sliders are enabled and "Зберегти" / "Скасувати" replace "Почати". UI state is derived from
  telemetry (`throttle_calibrating`), not from the click, exactly as today. Firmware session
  logic (engine-off gate, live servo follow, save/cancel) is unchanged.
- `throttle_cal_help` is shortened to the essentials (endpoints in µs, engine OFF, Почати → drag →
  Зберегти, full may be below idle).

### D5. Gear position defaults (simplified)
- One slider "Положення серво" 0–100 % (step 0.1). While not being dragged it follows telemetry
  `hall_position`.
- Locked (disabled, muted) until "Почати"; "Готово" locks it again. This active state is
  **UI-only** (page variable), no firmware command; reloading the page starts locked.
- While unlocked, dragging sends the existing `move_to_position` command, **rate-limited to at most
  one per 100 ms** with the final value always sent on release (`change`). The mockup uses 60 ms
  and no trailing send; the implementation follows the 10 Hz web-control rule instead.
- Four big buttons "Зберегти як R/N/H/L" (2×2, order R N H L; class `relay-btn`) send the existing
  `set_gear_default` with the slider value. Second line: the saved % from `gear_defaults`, or `--`.
  The button whose saved value is within ±0.3 % of the current slider value gets `.active`.
- Row "▶ R ▶ N ▶ H ▶ L" calls the existing `moveToGearPosition(g)` (sends `move_to_position` with
  the saved default) and is enabled only while unlocked.
- Removed: the four number inputs, 📍 capture, per-row Перемістити / Зберегти buttons. The saved
  values are held in a JS object fed from telemetry (not hidden inputs as in the mockup).
- Boost RPM target input + Зберегти stay unchanged in this card.
- The "Зберегти як" buttons are **always enabled**, whether or not Почати was pressed, because
  saving does not move the servo (decided, Q3). Only the slider and the ▶ row are gated by Почати.
- When the WebSocket disconnects, the card locks itself: slider and ▶ row disabled, Почати shown
  again (decided, Q4). Reconnecting does not unlock it; the engineer presses Почати again.

### D6. Gear sensor code and relay button patterns
- **Gear codes**: bit indicators A/B/C and the "code → gear" line stay. The four "Зберегти як"
  buttons use the same `relay-btn` 2×2 grid (order R N H L). Second line: the saved code as three
  binary digits plus " (типовий)" when the gear's `gear_codes_custom` bit is clear, or "не задано"
  in the warning colour when the code is `-1`. The button whose saved code equals the live
  `gear_code` (≥ 0) gets `.active`. The old text table `#gear-codes-table` is removed (its content
  is now on the buttons). "Скинути до типових" stays. Commands unchanged (`set_gear_code`,
  `reset_gear_codes`).
- **Relays**: `renderRelayTest()` builds eight `relay-btn` buttons: first line "Реле N"
  (`relay_test_relay`), second line the existing `relay_fn_N` description. Energised (bit set in
  `relay_test`) → `.active` (same highlight as an active gear button) and `aria-pressed="true"`.
  No ВИМК/УВІМК text; the `title` tooltip keeps "Реле N — опис: УВІМК/ВИМК". Enable/disable rules
  are unchanged from `update-aztec-relay-assignment`. Grid: 2 columns, equal height; 1 column at
  ≤768 px.
- `relay-btn` is a class distinct from `.gear-btn` so `updateControlsAvailability()` (which
  disables every `.gear-btn` while MAVLink is active) does not touch calibration or relay buttons.

### D7. Data binding for readouts shown in two sections
- The mockup duplicates a few readouts (steering angle, L/R limits and their ⚠ notes, range note,
  sensor/calibration status; speed limit, limiting row, steering authority, scaling base, reason,
  test-speed row) as `id="...-cal"` copies refreshed by `mirrorCalCopies()`. **Do not ship this.**
- Implementation: each duplicated readout carries `data-bind="<name>"` (no id needed), and
  `updateTelemetry()` renders it through one helper, e.g.
  `setBound(name, text, {color, show})` → `document.querySelectorAll('[data-bind="'+name+'"]')`
  and applies text / colour / visibility to every match. Single-instance readouts may keep their
  ids. This removes the second source of truth and makes adding a third placement free.
- Cost: `querySelectorAll` on ~15 names at 5 Hz is negligible; optionally cache the NodeLists once
  after DOMContentLoaded (the DOM is static).

### D8. Diagnostics "Увага" box
- Rebuilt on every telemetry frame from existing keys; hidden when empty. Items:
  - `steer_sensor_ok === false` → sensorless steering (error)
  - `steer_driver_ok === false` → VESC driver down (error); else `steer_vesc_fault` ≠ 0 → VESC
    fault with the code (error)
  - `steer_range_warning === true` → limit beyond ±550° (warning)
  - `mav_active` false → MAVLink inactive (warning)
  - `io_input_fault` / `io_relay_fault` → board I/O fault naming the expander(s) (error)
  - `steer_vin_low` true and `steer_driver_ok` true → "Напруга VESC низька (xx.x В)" with
    `steer_vin` (warning); ignored while the driver is down, because there is no valid VESC data
  - `can_status` ≠ `connected` → "Немає зв'язку з ЕБК" (warning)
  - `can_status` = `connected` and `ecu_voltage_low` true → "Напруга 12 В низька (xx.x В)" with
    `ecu_voltage` (warning)
- The board boost-rail fields `rail_24v` / `rail_low` stay in telemetry and in the Ignition card
  but are **not** used by the box: the 24 V warning comes from the steering VESC's own input
  voltage, which is what actually feeds the steering motor (decided, Q5).
- Items are built with `textContent` (no HTML injection), error items in the danger colour.
- On WebSocket disconnect the box keeps the last list (the sidebar WS status and the mobile
  connection dot show the disconnect).

### D9. Mobile (≤768 px)
- `.app` becomes one column; the sidebar is hidden and a sticky top bar (☰ + current section
  title) appears. Next to ☰ a small round connection dot shows the WebSocket state: green when
  connected, red when disconnected (decided, Q2); it is driven by the same code that updates the
  sidebar WS status item and carries a localized `title`/`aria-label`. ☰ toggles the sidebar as a drop-down panel under the top bar; choosing a section
  closes it. The WS/firmware status items are inside that panel.
- All grids collapse to one column (`main-grid`, relay grid); the gear 2×2 grids stay 2×2 (big
  enough at 375 px). No horizontal scroll at 375 px.

### D10. localStorage
- One new key `ui_section` (values `diag|calib|tests|system`), next to the existing language key.
  Every read and write is wrapped in try/catch; failure → default `diag`, page works normally.

### D11. i18n keys
- New (both `en` and `uk`, parity required): `brand_title`, `help_label`, `nav_diag`, `nav_calib`,
  `nav_tests`, `nav_system`, `card_steer_state`, `sd_angle`, `sd_rate`, `sd_rate_unit`, `sd_left`,
  `sd_right`, `sd_sensor`, `sd_cal`, `sd_driver`, `sd_zero`, `sd_motor`, `sd_fet`, `sd_code`,
  `card_speed_state`, `attention_title`, `att_sensorless`, `att_range`, `att_vesc`, `att_mav`,
  `att_io`, `att_steer_vin`, `att_ecu_volt`, `att_can`, `sd_vin`, `conn_dot_on`, `conn_dot_off`,
  `tests_note`, `card_network`, `lbl_ssid`, `lbl_ip`, `network_later`, `gd_start`,
  `gd_done`, `gc_unset`.
- Changed text: `gear_defaults_help` (slider + Зберегти як + ▶), `throttle_cal_help` (shortened).
- Removed when no longer referenced: `btn_move`, `title_use_current_servo`, and any `lbl_*` used
  only by the deleted steering-cal diagnostic rows. A parity check (same key set in `en` and `uk`)
  is part of the tasks.

### D12a. Network identity telemetry (decided Q1)
- The "Мережа" card shows the SSID and IP from the firmware constants, never from `index.html`
  text or `location.hostname` (`project.md`: the UI reads such values from telemetry).
- The firmware adds two static telemetry keys, delivered exactly like `firmware_version` / `board`:
  - `wifi_ssid`: `WIFI_AP_SSID` (e.g. `"QuadBike-Control"`)
  - `wifi_ip`: `WIFI_AP_IP` as a dotted string (e.g. `"192.168.10.1"`, via `IPAddress::toString()`)
- Implementation: two `String` fields next to `firmware_version` in the telemetry struct
  (`include/WebPortal.h`), set in `src/TelemetryManager.cpp` beside `FIRMWARE_VERSION`/`BOARD_NAME`,
  serialized in `src/WebPortal.cpp` beside `doc["firmware_version"]`. About 50 bytes per frame;
  stays within the telemetry document capacity.
- The card shows `--` until the first frame with the keys; a missing key keeps the previous value
  (same as the firmware version), so an older firmware never causes a JavaScript error.
- Together with D12b these are the only firmware changes besides the version bump and changelog.

### D12b. Voltage telemetry and thresholds (decided Q5)
- New telemetry key `steer_vin`: steering VESC input voltage in V, one decimal, from
  `SteeringController::getInputVoltage()` (→ `VescMotorDriver::inputVoltageV()`, already polled via
  `COMM_GET_VALUES`; today it only reaches the MAVLink report in `main.cpp`). Emitted every frame;
  the UI treats it as meaningless while `steer_driver_ok` is false.
- New compile-time constants in `include/Constants.h`:
  - `STEER_VIN_LOW_V` = **21.5 V** (default). The steering pack is 6s NMC or 7s LiFePO4 depending
    on the vehicle build: 21.5 V ≈ 3.58 V/cell on 6s NMC and ≈ 3.07 V/cell on 7s LiFePO4 — low but
    not yet harmful for either. **The user may adjust this value per build.**
  - `ECU_VOLTAGE_LOW_V` = **12.0 V** (default), applied to the ECU module voltage (`ecu_voltage`).
    The 12 V battery is an ordinary lead-acid starter battery: at rest 12.0 V is about 50 % state
    of charge (11.8 V about 40 %), so 12.0 V warns early enough to recharge before cranking fails.
    With the engine running the alternator should hold 13.5–14.4 V, so the fixed 12.0 V threshold
    only catches a badly failing charging system while running (see "Later").
- How the thresholds reach the page: the page cannot read `Constants.h`, and `project.md` forbids
  copying firmware values into `index.html`. So the firmware compares and emits two booleans,
  `steer_vin_low` (`steer_driver_ok && steer_vin < STEER_VIN_LOW_V`) and `ecu_voltage_low`
  (CAN connected && module voltage < `ECU_VOLTAGE_LOW_V`, emitted inside the CAN-connected block
  like `ecu_voltage`), following the existing `rail_low` pattern. The UI shows the measured value
  in the message text. Alternative considered: emit the two thresholds as telemetry keys and compare
  in the page — rejected, it sends constants every frame and duplicates logic in JS. No hysteresis
  (the attention box is informational and re-evaluated each frame).
- "Кермо: стан" gets an extra tile "Напруга VESC" / "VESC voltage" showing `steer_vin` (`--` while
  `steer_driver_ok` is false or the key is missing), coloured warning while `steer_vin_low`.

### D12. What stays unchanged in firmware
- All of `src/`, `include/` except `FIRMWARE_VERSION` (1.3.0 → 1.4.0), a new 1.4.0 entry at the
  top of `include/Changelog.h` (uk + en, plain language, states "no recalibration required") and
  the explicit list below:
  1. `FIRMWARE_VERSION` 1.3.0 → 1.4.0 (`include/Constants.h`)
  2. 1.4.0 changelog entry (`include/Changelog.h`)
  3. `wifi_ssid` / `wifi_ip` telemetry keys (D12a)
  4. `steer_vin` telemetry key (D12b)
  5. `steer_vin_low` / `ecu_voltage_low` telemetry flags (D12b)
  6. `STEER_VIN_LOW_V` and `ECU_VOLTAGE_LOW_V` constants (D12b)
- Existing telemetry JSON keys, WebSocket commands and their gates (input source, engine-running, ignition,
  relay-test gate), `/api/changelog`, `/api/debug`, OTA endpoint, NVS layout, AP SSID and IP.
- Commands used by the new UI are all existing: `move_to_position`, `set_gear_default`,
  `set_gear_code`, `reset_gear_codes`, `set_relay_test`, `throttle_cal_begin|save|cancel`, `steer_jog`, `steer_nudge`, `steer_lws_zero`, `steer_cal_left|right`, `set_boost_rpm` and the existing `set_*` commands.

### Later (out of scope)
- Charging-system warning: with `engine_rpm` > 0, warn when the ECU voltage stays below 13.0 V
  (alternator not charging). Needs a second constant and an rpm-gated flag; not part of this change.

## Risks / Trade-offs
- Hidden sections hide state changes (e.g. a fault while on Тести) → the Attention box and the
  sidebar status are the safety net; manual-control warnings stay in their card.
- The gear-default slider sends `move_to_position`, which the firmware accepts regardless of the
  input source (unchanged behaviour). A slider is easier to drag than typing a number → the
  Почати/Готово lock and the 100 ms rate limit mitigate; no firmware gate is added (out of scope).
- Each `move_to_position` produces a response and a debug line; at 10 Hz this is the same load as
  the existing manual-control sliders.
- One big HTML file grows by the shell CSS/JS but loses the duplicated help paragraphs and rows;
  net size should stay within a few KB of today (check LittleFS image size in tasks).

## Migration Plan
- Flash firmware 1.4.0 and the LittleFS image (`pio run -t upload`, `pio run -t uploadfs`), in any
  order. No NVS migration; no recalibration.
- An old LittleFS page with new firmware (or the reverse) keeps working, because the protocol is
  unchanged. Rollback = reflash the previous LittleFS image.

## Open Questions
- None. Q1–Q5 were decided by the user (see D5, D8, D9, D12a, D12b). The only value flagged for
  adjustment is the default of `STEER_VIN_LOW_V` (D12b).
