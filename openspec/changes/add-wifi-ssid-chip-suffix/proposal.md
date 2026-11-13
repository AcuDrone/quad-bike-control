# Change: Append a per-chip suffix to the WiFi AP SSID and OTA hostname

## Why
Every board broadcasts the same SSID `QuadBike-Control` (and the aztec branch its own fixed
prefix), so with two or more vehicles or bench boards powered up nearby the phone cannot tell the
access points apart and may join the wrong vehicle. The OTA hostname `quadbike-control` collides
the same way. A local uncommitted workaround (`"QuadBike-Control-ask"` in `Constants.h`) shows the
need; it has to be hand-edited per board and is not flashed consistently.

User decision 2026-11-06: append a unique, stable chip suffix computed at boot.

## What Changes
- The AP SSID becomes `<WIFI_AP_SSID>-XXXX`, where `XXXX` is the last two bytes of the factory
  (eFuse base) MAC as 4 uppercase hex characters, e.g. `QuadBike-Control-5ECC`.
- The OTA hostname becomes `<OTA_HOSTNAME>-xxxx` with the same suffix in lowercase
  (e.g. `quadbike-control-5ecc`).
- Telemetry `wifi_ssid` carries the actual broadcast SSID (with suffix), so the web Network card
  shows what the phone sees.
- The AP password / open-network setting, channel, IP, max clients are unchanged.
- No NVS key, no web setting: the suffix is derived from hardware at every boot.
- A small header-only helper `include/DeviceId.h` (pure functions, native-tested in
  `test/test_device_id`).
- Firmware version 1.5.21 (1.5.20 is the concurrent steering change); Changelog uk + en.
- The base `WIFI_AP_SSID` stays a per-branch constant (`QuadBike-Control` on master; aztec keeps
  its own prefix on merge). The local `"-ask"` edit is reverted to `"QuadBike-Control"`.

## Impact
- Affected specs: `web-server` (WiFi Access Point Operation), `ota-updates` (Over-The-Air
  Firmware Update), `web-telemetry` (Network Identity Telemetry).
- Affected code: new `include/DeviceId.h`; `src/WebPortal.cpp` (`setupWiFiAP`, `setupOTA`, debug
  print), `src/TelemetryManager.cpp` (`wifi_ssid`), `include/Constants.h` (version, SSID base
  restored), `include/Changelog.h`, `platformio.ini` (native `test_filter`), new
  `test/test_device_id/test_main.cpp`.
- User-visible: after flashing, the phone must join the new SSID once (old saved network
  `QuadBike-Control` no longer appears). Arduino OTA / mDNS target name changes to the suffixed
  hostname; the fixed IP `192.168.10.1` is unchanged, so the web panel and HTTP OTA upload work as
  before.
