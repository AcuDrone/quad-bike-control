# Design: WiFi SSID chip suffix

## Source of the suffix
- Use the factory **base MAC** from eFuse (`esp_efuse_mac_get_default()` / Arduino
  `ESP.getEfuseMac()`), not the SoftAP MAC. The base MAC is the same regardless of which
  interface is up and of any `esp_base_mac_addr_set()` call. ESP-IDF derives the SoftAP MAC as
  base + 1 in the last octet, so the AP BSSID a phone's WiFi analyser shows may differ from the
  suffix in the last hex digit(s); this is accepted and documented in the Changelog note.
- Suffix = MAC octets 4 and 5 (the last two in the usual `AA:BB:CC:DD:EE:FF` printing), formatted
  `%02X%02X` → 4 uppercase hex characters. 16 bits give 65 536 values; collisions between the few
  boards in one fleet are negligible.

## Byte-order assumption (documented in the header)
`ESP.getEfuseMac()` (arduino-esp32) does `esp_efuse_mac_get_default((uint8_t*)&mac)` into a
`uint64_t`, i.e. it memcpy's the 6 octets in transmission order into a little-endian integer:
octet 0 is bits 0–7, octet 5 is bits 40–47. For MAC `3C:DC:75:FC:5E:CC` the value is
`0x0000CC5EFC75DC3C`. Therefore:
- `formatChipSuffix(const uint8_t mac[6], char out[5])` — primary, unambiguous: `mac[4]`, `mac[5]`.
- `formatChipSuffixFromEfuse(uint64_t efuse, char out[5])` — takes `(efuse >> 32) & 0xFF` as
  octet 4 and `(efuse >> 40) & 0xFF` as octet 5 → `"5ECC"` for the example above.
The firmware calls the array form with `esp_efuse_mac_get_default(mac)` (or `esp_read_mac(mac,
ESP_MAC_WIFI_STA)`, which equals the base MAC); the uint64 form exists so code using
`ESP.getEfuseMac()` gets the same answer, and the native test pins both to the same result.

## Helper shape (`include/DeviceId.h`, header-only, no Arduino includes)
- `formatChipSuffix(...)` as above, writes a NUL-terminated 4-char string.
- `buildSuffixedName(const char* base, const char* suffix, bool lowercase, char* out, size_t cap)`
  → `base + "-" + suffix` (suffix lower-cased for the hostname), truncating the **base** so the
  result fits `cap - 1`; for the SSID `cap = 33` (≤ 32 octets per 802.11).
  `QuadBike-Control-5ECC` is 21 characters; the aztec prefix is also well below the limit.
- `WebPortal` owns two `static char` buffers (`apSsid_[33]`, `otaHostname_[33]`) filled once in
  `begin()` before `setupWiFiAP()` and exposes `const char* getApSsid()`. Static storage is required
  because `Telemetry::wifi_ssid` is a `const char*` stored by pointer (no String churn at 5 Hz).

## Call sites
- `WebPortal::setupWiFiAP()` — `WiFi.softAP(apSsid_, ...)` in both password branches; the debug
  print shows `apSsid_`.
- `WebPortal::setupOTA()` — `ArduinoOTA.setHostname(otaHostname_)`.
- `TelemetryManager` — `telemetry.wifi_ssid = webPortal.getApSsid()` (or the shared static getter
  from `DeviceId`), replacing the compile-time `WIFI_AP_SSID`.

## Alternatives rejected
- NVS / web-editable name: more UI and a recovery problem if mistyped; not requested.
- Full 6-byte MAC: SSID gets long and unreadable; 2 bytes are enough to tell boards apart.
- SoftAP MAC: matches BSSID but changes if the base MAC is ever overridden; base MAC preferred per
  user decision.

## Merge note (aztec)
Merging into `aztec` keeps aztec's own `WIFI_AP_SSID` / `OTA_HOSTNAME` values in `Constants.h`
(resolve the conflict in favour of aztec's strings); the suffix logic applies unchanged, giving
`<aztec prefix>-XXXX`.
