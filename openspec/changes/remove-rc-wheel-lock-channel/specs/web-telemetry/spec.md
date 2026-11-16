## ADDED Requirements
### Requirement: Unassigned Servo Channel Labels
The web page's raw MAVLink channel rows (Ch1–Ch8, values from telemetry `mav_channels`) SHALL name
a function only for a channel the firmware maps to one. Servo channel 7 drives nothing (see
`mavlink-interface` "Servo Channel to Vehicle Command Mapping"), so its row SHALL keep showing the
raw value under the plain label "Ch7" / "Кан7", like the unused channels 5 and 8, and SHALL NOT
mention the lock.

#### Scenario: Channel 7 row without a function
- **WHEN** the web page shows the MAVLink channel rows in English or in Ukrainian
- **THEN** the channel 7 row SHALL be labelled "Ch7" / "Кан7" and SHALL show `mav_channels[6]`
- **AND** no channel row SHALL be labelled with the lock
