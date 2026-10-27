## MODIFIED Requirements
### Requirement: WiFi Access Point Operation
The system SHALL operate as a WiFi Access Point to provide network connectivity for the web portal.

#### Scenario: Initialize WiFi AP on startup
- **WHEN** system powers on or resets
- **THEN** ESP32 creates WiFi Access Point with SSID "QuadBike-Control"
- **AND** AP uses open authentication (no password)
- **AND** ESP32 IP address is 192.168.10.1
- **AND** DHCP server is enabled for client devices

#### Scenario: Maintain AP during operation
- **WHEN** WiFi AP is active
- **THEN** AP remains available continuously during system operation
- **AND** multiple clients can connect simultaneously
- **AND** client connections do not interrupt vehicle control operations

#### Scenario: Query AP status
- **WHEN** AP status is requested
- **THEN** number of connected clients is returned
- **AND** AP IP address and SSID are available

