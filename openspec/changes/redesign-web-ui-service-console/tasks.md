## 0. Preconditions
- [x] 0.1 Proposal approved; all open questions decided (see design.md D5, D8, D9, D12a, D12b)
- [x] 0.2 Confirm the baseline `data/index.html` is the 1.3.0 page from `e466566` (LWS steering card,
      gear codes, relay test present)

## 1. Shell and navigation (`data/index.html`)
- [x] 1.1 Replace the header + status bar with the `.app` grid: sidebar (brand `brand_title`, four
      nav buttons, WS + firmware status items at the bottom) and `<main class="content">` with the
      content header (section title + existing language toggle)
- [x] 1.2 Wrap the cards in four `<section data-section="diag|calib|tests|system">` blocks per the
      card → section map in design D2; add the Тести bench note (`tests_note`)
- [x] 1.3 Implement `showSection()` / `toggleNav()` and start-up restore from `localStorage['ui_section']`
      with try/catch and fallback to `diag` (D1, D10)
- [x] 1.4 Re-render the section title / top-bar title on language change
- [x] 1.5 Mobile CSS (≤768 px): sticky top bar with ☰, drop-down sidebar, one-column grids (D9)
- [x] 1.5a Mobile top-bar connection dot next to ☰: green connected / red disconnected, updated by
      the same function as the sidebar WS status, localized title/aria-label
- [x] 1.6 Network placeholder card in Система: SSID and IP read-only from telemetry `wifi_ssid` /
      `wifi_ip` (`--` until received, previous value kept when a key is missing), `network_later` note
- [x] 1.7 Remove the separate "What's new" button from anywhere except System maintenance; keep the
      firmware status item opening the changelog modal

## 2. Diagnostics section
- [x] 2.1 "Увага" box: `updateAttention(data)` built with `textContent`, hidden when empty, with all
      items of D8 incl. low VESC voltage (`steer_vin_low`, ignored while `steer_driver_ok` false),
      low 12 V (`ecu_voltage_low` while CAN connected) and no ECU link (`can_status` ≠ connected);
      `rail_24v`/`rail_low` not used
- [x] 2.2 "Кермо: стан" card: 2-column tile grid (Кут, Швидкість, Межа Л, Межа П with ⚠ "далі 550°",
      Датчик + fault label, Калібрування, Драйвер VESC, Нуль датчика + error, Струм мотора, Темп. FET,
      Напруга VESC from `steer_vin`),
      full-width range note, full-width monospace LWS raw line
- [x] 2.3 Read-only "Швидкість" card: speed, sensor state, limit, limiting row, steering authority,
      scaling base, reason, test-speed row, odometer, trip — no inputs
- [x] 2.4 Verify Telemetry, CAN (with Probe ECU) and MAVLink cards moved unchanged

## 3. Calibration section
- [x] 3.1 Steering calibration: keep jog/nudge/jog speed/Set left/Set right/Set sensor zero, the
      sensorless note, angle, L/R limits with ⚠ notes, one sensor|calibration status line, the
      status message; delete the diagnostic rows now in "Кермо: стан"
- [x] 3.2 Introduce `data-bind` + one render helper for readouts shown in two sections (D7); no
      `-cal` id mirroring
- [x] 3.3 Throttle calibration: sliders always visible, locked until `throttle_calibrating`;
      Зберегти/Скасувати replace Почати while active; shorten `throttle_cal_help` (D4)
- [x] 3.4 Gear position defaults: servo slider following `hall_position`, Почати/Готово UI lock,
      `move_to_position` at ≤10 Hz with trailing send, 2×2 "Зберегти як" buttons with saved % and
      ±0.3 % highlight, "Зберегти як" always enabled, "▶ R/N/H/L" row enabled only while active,
      auto-lock (Почати shown again) on WebSocket disconnect, boost RPM target kept; remove
      number inputs, 📍, per-row Перемістити/Зберегти; new `gear_defaults_help` text (D5)
- [x] 3.5 Gear sensor codes: 2×2 `relay-btn` buttons (R N H L) with saved code / "(типовий)" /
      "не задано", live-code highlight; remove the text table; keep bits, live line, reset (D6)
- [x] 3.6 Speed sensor settings card: pulses/rev, circumference, limit readout, scaling base
      readout + input, bench test speed input, status line

## 4. Tests section
- [x] 4.1 Manual control and Ignition & lighting moved unchanged (minus help paragraphs)
- [x] 4.2 Relay test: eight equal `relay-btn` buttons ("Реле N" + description), energised =
      `.active` + `aria-pressed`, tooltip with state, 2 columns / 1 on mobile; enable rules unchanged

## 5. Help modal
- [x] 5.1 Shared `#help-modal` reusing the changelog modal styles; `openHelp`/`closeHelp`/`refreshHelp`;
      close on ✕, overlay, Escape; re-render on language switch (D3)
- [x] 5.2 Add ⓘ buttons to the nine cards listed in D3 and remove their long paragraphs
- [x] 5.3 Confirm one-line hints and state-dependent warnings remain in place

## 6. i18n
- [x] 6.1 Add the new keys of D11 to `en` and `uk`; update `gear_defaults_help`, `throttle_cal_help`
- [x] 6.2 Remove keys no longer referenced; run a parity check (identical key sets in `en`/`uk`,
      e.g. a short node/python script over the page) and fix any gap

## 7. Firmware version and changelog (per `project.md`)
- [x] 7.1 `include/Constants.h`: `FIRMWARE_VERSION` "1.3.0" → "1.4.0"
- [x] 7.2 `include/Changelog.h`: new top entry `{"version":"1.4.0", date, uk:[...], en:[...]}` —
      e.g. uk "Веб-панель перероблена на сервісну консоль: розділи Діагностика, Калібрування, Тести,
      Система; довідка за кнопкою ⓘ; повторне калібрування не потрібне", with the matching en line
      ("Web portal redesigned as a service console: Diagnostics, Calibration, Tests, System sections;
      help behind the ⓘ button; no recalibration required"); optionally one line each for the gear
      slider and the relay buttons
- [x] 7.3 `include/Constants.h`: add `STEER_VIN_LOW_V` 21.5f (comment: 6s NMC ≈3.58 V/cell,
      7s LiFePO4 ≈3.07 V/cell; adjust per build) and `ECU_VOLTAGE_LOW_V` 12.0f (lead-acid at rest ≈50 % SOC)
- [x] 7.4 Telemetry `wifi_ssid` / `wifi_ip`: String fields in the struct in `include/WebPortal.h`, set
      in `src/TelemetryManager.cpp` from `WIFI_AP_SSID` / `WIFI_AP_IP.toString()` next to
      `FIRMWARE_VERSION`, serialized in `src/WebPortal.cpp` next to `doc["firmware_version"]`
- [x] 7.5 Telemetry `steer_vin` (float, 1 decimal) from `SteeringController::getInputVoltage()`, next
      to `steer_motor_current`/`steer_fet_temp`; and `steer_vin_low` = `steer_driver_ok &&
      steer_vin < STEER_VIN_LOW_V`
- [x] 7.6 Telemetry `ecu_voltage_low` = module voltage < `ECU_VOLTAGE_LOW_V`, emitted inside the
      CAN-connected block next to `ecu_voltage`
- [x] 7.7 Check the telemetry JSON document capacity with all new keys and the peak ECU-probe case
- [x] 7.8 Confirm no other file under `src/` or `include/` changed (`git diff --stat`)

## 8. Documentation
- [x] 8.1 `WEB_PORTAL_SETUP.md`: rewrite "Use the Web Portal" for the four sections, the ⓘ help,
      the mobile menu; fix the stale `WIFI_AP_IP` snippet (192.168.10.1); "ESP32-C6" → "ESP32-S3"
      where it describes this board
- [x] 8.2 Add or refresh screenshots of the four sections (desktop) and one mobile view, if the
      document carries screenshots; otherwise add a short section-by-section description

## 9. Build and verification
- [x] 9.1 `pio run` builds without new warnings
- [x] 9.2 `pio run -t buildfs` succeeds; note the LittleFS image size vs 1.3.0
- [x] 9.3 Open `data/index.html` from disk in a browser: no console errors with no WebSocket;
      all four sections switch; reload restores the last section; private window (no storage) still
      works and starts on Діагностика
- [ ] 9.4 [USER/MANUAL] Flash firmware 1.4.0 (`pio run -t upload`) and `pio run -t uploadfs`; connect
      to `http://192.168.10.1`; sidebar shows WebSocket connected and "1.4.0 · aztec"; changelog modal
      shows the 1.4.0 entry
- [ ] 9.5 [USER/MANUAL] Діагностика: tiles update at 5 Hz; unplug the LWS CAN → "Увага" lists
      sensorless steering and the tile shows the fault; reconnect → item clears; MAVLink inactive item
      present on the bench and gone with the autopilot streaming
- [ ] 9.6 [USER/MANUAL] Калібрування / steering: jog, nudge, Set left/right, Set sensor zero behave as
      in 1.3.0; angle and limits match the Діагностика tiles
- [ ] 9.7 [USER/MANUAL] Калібрування / throttle: sliders show saved values and are locked; Почати →
      sliders move the servo; Зберегти persists (reload shows new values); Скасувати restores
- [ ] 9.8 [USER/MANUAL] Калібрування / gear defaults: slider locked until Почати; dragging moves the
      transmission servo smoothly; "Зберегти як N" stores the position and highlights N; ▶ buttons
      move to saved positions and are disabled after Готово
- [ ] 9.9 [USER/MANUAL] Калібрування / gear codes: put the lever in each gear, "Зберегти як" assigns
      the code, the matching button highlights; take-over shows "не задано" on the other gear;
      reset restores "(типовий)"
- [ ] 9.10 [USER/MANUAL] Калібрування / speed sensor: save PPR and circumference, scaling base,
      bench test speed; the read-only Швидкість card reflects them
- [ ] 9.11 [USER/MANUAL] Тести: manual control, ignition/light/boost rail, relay buttons energise
      and highlight; button disabled with ignition on; auto-off clears the highlight
- [ ] 9.12 [USER/MANUAL] Система: debug toggle, boost test, OTA upload of the same 1.4.0 binary,
      "Що нового" opens the changelog
- [ ] 9.12a [USER/MANUAL] Telemetry frame (browser devtools → WS messages) contains
      `"wifi_ssid":"QuadBike-Control"` and `"wifi_ip":"192.168.10.1"`; the Мережа card shows the same
      values read-only; the telemetry document does not overflow (no missing trailing keys)
- [ ] 9.12b [USER/MANUAL] Voltages: `steer_vin` matches a multimeter on the VESC input (±0.2 V) and
      the "Напруга VESC" tile shows it; lower the bench supply below 21.5 V (or temporarily build with
      a higher `STEER_VIN_LOW_V`) → "Напруга VESC низька" appears and clears when restored; with the
      VESC unplugged no voltage item appears (driver-down item only)
- [ ] 9.12c [USER/MANUAL] ECU: unplug CAN → "Немає зв'язку з ЕБК" appears, 12 V item absent; with
      CAN connected and the battery below 12.0 V (bench supply) → "Напруга 12 В низька" appears
- [ ] 9.12d [USER/MANUAL] Gear defaults: press Почати, then stop the WebSocket (switch WiFi off) →
      slider and ▶ row lock and Почати returns; "Зберегти як" stays enabled while connected and locked
- [ ] 9.12e [USER/MANUAL] Mobile dot: green when connected, turns red within a few seconds of WiFi loss
- [ ] 9.13 [USER/MANUAL] Help: every ⓘ opens the right text; Escape/overlay/✕ close; switch language
      with the modal open → text re-renders
- [ ] 9.14 [USER/MANUAL] Mobile at 375 px (phone or devtools): top bar + ☰, no horizontal scroll,
      relay grid one column, gear 2×2 grids usable, modals fit the screen
- [x] 9.15 `openspec validate redesign-web-ui-service-console --strict` passes
