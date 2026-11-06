## MODIFIED Requirements
### Requirement: WiFi Access Point Operation
The system SHALL operate as a WiFi Access Point to provide network connectivity for the web portal.
The broadcast SSID SHALL be the compiled per-branch base name `WIFI_AP_SSID` followed by `-` and a
4-character chip suffix, so that several boards powered up nearby can be told apart. The suffix
SHALL be the last two octets of the factory eFuse base MAC (not the SoftAP MAC) as uppercase
hexadecimal, computed at every boot without NVS storage or a web setting.

#### Scenario: Initialize WiFi AP on startup
- **WHEN** system powers on or resets
- **THEN** ESP32 creates WiFi Access Point with SSID `<WIFI_AP_SSID>-XXXX` (e.g.
  "QuadBike-Control-5ECC" on master)
- **AND** AP uses open authentication (no password), unchanged by the suffix
- **AND** ESP32 IP address is 192.168.10.1
- **AND** DHCP server is enabled for client devices
- **AND** the startup debug log prints the suffixed SSID

#### Scenario: Chip suffix derived from the eFuse base MAC
- **WHEN** the factory base MAC is `3C:DC:75:FC:5E:CC`
- **THEN** the suffix SHALL be `5ECC` (octets 4 and 5, uppercase, leading zeros kept)
- **AND** the same board SHALL produce the same suffix on every boot
- **AND** two boards with different base MACs in their last two octets SHALL broadcast different SSIDs

#### Scenario: SSID length limit
- **WHEN** the base name plus `-XXXX` would exceed 32 octets
- **THEN** the base name SHALL be truncated so the SSID is at most 32 octets and still ends with
  `-XXXX`

#### Scenario: Maintain AP during operation
- **WHEN** WiFi AP is active
- **THEN** AP remains available continuously during system operation
- **AND** multiple clients can connect simultaneously
- **AND** client connections do not interrupt vehicle control operations

#### Scenario: Query AP status
- **WHEN** AP status is requested
- **THEN** number of connected clients is returned
- **AND** AP IP address and the suffixed SSID are available
