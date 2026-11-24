## MODIFIED Requirements
### Requirement: Network Identity Telemetry
The telemetry broadcast SHALL include the WiFi access point identity actually in use, delivered the
same way as `firmware_version`, and the System section's "Мережа" / "Network" card SHALL display it
read-only. The web page SHALL NOT hard-code these values or derive them from the browser location.

#### Scenario: Include the network identity in telemetry JSON
- **WHEN** telemetry data is formatted as JSON
- **THEN** it SHALL include `wifi_ssid` equal to the broadcast SSID including the chip suffix
  (`<WIFI_AP_SSID>-XXXX`, e.g. `"QuadBike-Control-5ECC"`), not the bare compiled `WIFI_AP_SSID`
- **AND** `wifi_ip` equal to the compiled `WIFI_AP_IP` as a dotted-quad string (e.g. `"192.168.10.1"`)
- **AND** both SHALL be emitted unconditionally, independent of CAN or MAVLink status, and SHALL
  keep the telemetry document within its configured capacity
- **AND** the SSID string SHALL live in static storage computed once at boot (no per-frame
  allocation)

#### Scenario: Network card shows the values read-only
- **WHEN** a telemetry message containing `wifi_ssid` and `wifi_ip` is received
- **THEN** the Network card SHALL show the SSID and the IP address as read-only text, with a note
  that the settings will come later
- **AND** the SSID shown SHALL match the network name the phone sees
- **AND** the card SHALL offer no input or command

#### Scenario: Values not yet received
- **WHEN** no telemetry with these keys has been received, or a frame lacks them
- **THEN** the card SHALL show `--` until the first value arrives and otherwise keep the previous
  value
- **AND** no JavaScript error SHALL be thrown
