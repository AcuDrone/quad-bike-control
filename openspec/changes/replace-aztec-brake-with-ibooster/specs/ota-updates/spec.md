## ADDED Requirements
### Requirement: OTA Refused While Braking or Moving
On the aztec buggy a firmware or filesystem update (web `/update`, ArduinoOTA) and the event-journal
clear SHALL NOT start while a service brake is commanded (target above 0) or its request is active
(`BRAKING` or `RELEASING`), or while the wheel-speed signal is valid and above `OTA_MAX_SPEED_KMH`
(1.0 km/h), because flash erasing stalls the CAN task that keeps the brake applied. An invalid speed
signal SHALL count as standing. A web upload already running when that condition starts SHALL be
aborted.

#### Scenario: Web upload refused
- **WHEN** a web firmware or LittleFS upload starts while the brake is requested or the buggy is moving
- **THEN** the upload SHALL be refused before `Update.begin()` with the reply "OTA refused: brake
  applied or vehicle moving"
- **AND** no flash write SHALL occur

#### Scenario: ArduinoOTA held off
- **WHEN** the same condition holds
- **THEN** `ArduinoOTA.handle()` SHALL NOT be called, so an upload invitation times out
- **AND** one log line SHALL be written per episode

#### Scenario: Web upload aborted
- **WHEN** a brake is commanded or applied, or the buggy starts moving, while a web firmware or
  LittleFS upload is being written
- **THEN** the upload SHALL be aborted before its next write (`Update.abort()`), answered "OTA
  aborted: brake applied or vehicle moving" and shown on the page with the reason
- **AND** an aborted firmware upload SHALL leave the running firmware in place; an aborted LittleFS
  upload leaves the filesystem partly written, so the web page needs a new LittleFS upload (USB or
  ArduinoOTA)

#### Scenario: ArduinoOTA keeps the brake command frozen
- **WHEN** an ArduinoOTA upload runs (it blocks the main loop for the whole upload)
- **THEN** no brake request can begin: the upload started only with no brake commanded and no
  request active, the brake command the CAN task holds is frozen at that value (target 0) and no
  input reaches it until the upload ends

#### Scenario: Journal clear refused
- **WHEN** a journal clear is requested under the same condition
- **THEN** it SHALL be refused with an error reply

#### Scenario: Standing, brake idle
- **WHEN** no brake request is active and the speed is invalid or at most 1.0 km/h
- **THEN** OTA and the journal clear SHALL work as before
