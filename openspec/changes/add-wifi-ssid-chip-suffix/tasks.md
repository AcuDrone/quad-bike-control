## 1. Helper
- [x] 1.1 `include/DeviceId.h` (header-only, no Arduino): `formatChipSuffix(const uint8_t mac[6], char out[5])` (octets 4–5, `%02X%02X`), `formatChipSuffixFromEfuse(uint64_t, char out[5])` (octet 4 = bits 32–39, octet 5 = bits 40–47, byte-order assumption from `ESP.getEfuseMac()` documented in a comment), `buildSuffixedName(base, suffix, lowercase, out, cap)` truncating the base so the result fits `cap - 1`.
- [x] 1.2 `test/test_device_id/test_main.cpp`: MAC `3C:DC:75:FC:5E:CC` → `"5ECC"` via the array form; `0x0000CC5EFC75DC3C` → `"5ECC"` via the uint64 form; leading zeros kept (`..:0A:05` → `"0A05"`); `buildSuffixedName("QuadBike-Control","5ECC",false,…,33)` → `"QuadBike-Control-5ECC"`; lowercase hostname → `"quadbike-control-5ecc"`; a 40-char base is truncated so the result is exactly 32 chars and ends with `-5ECC`. Add `test_device_id` as its own indented line in `platformio.ini` `[env:native]` `test_filter` and to the header comment.

## 2. Firmware
- [x] 2.1 `WebPortal`: static `apSsid_[33]` / `otaHostname_[33]` filled once in `begin()` from `esp_efuse_mac_get_default()` before `setupWiFiAP()`; `getApSsid()` accessor.
- [x] 2.2 `WebPortal::setupWiFiAP()`: `WiFi.softAP(apSsid_, …)` in both the password and the open branch; debug print shows the suffixed SSID. Password, channel, IP, max clients unchanged.
- [x] 2.3 `WebPortal::setupOTA()`: `ArduinoOTA.setHostname(otaHostname_)` (lowercase suffix).
- [x] 2.4 `TelemetryManager.cpp`: `telemetry.wifi_ssid` = the suffixed SSID (static storage, pointer stays valid).
- [x] 2.5 `Constants.h`: restore `WIFI_AP_SSID` to `"QuadBike-Control"` (drop the local `-ask` edit), comment that a chip suffix is appended at boot; `FIRMWARE_VERSION` → `"1.5.21"`.
- [x] 2.6 `Changelog.h`: 1.5.21 entry uk + en (SSID now `QuadBike-Control-XXXX`, join the new network once; OTA hostname suffixed; IP unchanged).

## 3. Verification
- [x] 3.1 `pio test -e native` green (all suites incl. `test_device_id`); `pio run -e esp32-s3-devkitc-1` builds.
- [ ] 3.2 Bench: boot log prints `SSID: QuadBike-Control-XXXX`; the phone sees that SSID, joins it, `192.168.10.1` opens the panel.
- [ ] 3.3 Bench: the System → Network card shows the same suffixed SSID; reboot → same suffix.
- [ ] 3.4 Bench (if two boards): the two SSIDs differ.

## 4. Merge
- [ ] 4.1 Merge into `aztec`: keep aztec's own `WIFI_AP_SSID` / `OTA_HOSTNAME` strings in `Constants.h`; the suffix logic applies unchanged.
