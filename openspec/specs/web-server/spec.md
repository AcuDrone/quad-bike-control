# web-server Specification

## Purpose
TBD - created by archiving change add-web-portal. Update Purpose after archive.
## Requirements
### Requirement: WiFi Access Point Operation
The system SHALL operate as a WiFi Access Point to provide network connectivity for the web portal.

#### Scenario: Initialize WiFi AP on startup
- **WHEN** system powers on or resets
- **THEN** ESP32 creates WiFi Access Point with SSID "QuadBike-Control"
- **AND** AP uses open authentication (no password)
- **AND** ESP32 IP address is 192.168.4.1
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

### Requirement: Asynchronous Web Server
The system SHALL provide an asynchronous HTTP web server for hosting the control interface.

#### Scenario: Initialize web server on port 80
- **WHEN** WebPortal.begin() is called
- **THEN** AsyncWebServer is initialized on port 80
- **AND** web server starts listening for HTTP requests
- **AND** server operates asynchronously (non-blocking)

#### Scenario: Serve static HTML page
- **WHEN** HTTP GET request is made to root URL (/)
- **THEN** index.html is served from SPIFFS/LittleFS filesystem
- **AND** content-type header is set to text/html
- **AND** page loads successfully in web browser

#### Scenario: Handle 404 errors
- **WHEN** HTTP request is made to non-existent resource
- **THEN** 404 error response is returned
- **AND** error message is logged

### Requirement: WebSocket Communication
The system SHALL provide WebSocket endpoint for bidirectional real-time communication between server and clients.

#### Scenario: Accept WebSocket connection
- **WHEN** client connects to WebSocket endpoint (/ws)
- **THEN** connection is accepted
- **AND** client is added to active connection list
- **AND** connection confirmation is sent to client
- **AND** connection event is logged

#### Scenario: Receive WebSocket message from client
- **WHEN** JSON message is received from WebSocket client
- **THEN** message is parsed and validated
- **AND** appropriate command handler is invoked
- **AND** response is sent back to client

#### Scenario: Broadcast message to all connected clients
- **WHEN** broadcastMessage() is called with data
- **THEN** message is sent to all active WebSocket clients
- **AND** failed sends to disconnected clients are handled gracefully

#### Scenario: Handle WebSocket disconnection
- **WHEN** client disconnects from WebSocket
- **THEN** client is removed from active connection list
- **AND** any associated command control is released
- **AND** disconnection event is logged

### Requirement: SPIFFS/LittleFS Filesystem Integration
The system SHALL use SPIFFS or LittleFS filesystem to store and serve web interface files.

#### Scenario: Mount filesystem on startup
- **WHEN** WebPortal.begin() is called
- **THEN** SPIFFS/LittleFS filesystem is mounted
- **AND** mount success is verified
- **AND** error is logged if mount fails

#### Scenario: Serve files from filesystem
- **WHEN** HTTP request is made for static resource
- **THEN** file is read from SPIFFS/LittleFS
- **AND** appropriate content-type header is set based on file extension
- **AND** file contents are returned in HTTP response

### Requirement: Web Server Performance
The system SHALL ensure web server operations do not interfere with real-time vehicle control.

#### Scenario: Non-blocking web operations
- **WHEN** web server is handling HTTP requests or WebSocket messages
- **THEN** main control loop is not blocked
- **AND** control loop timing remains <10ms average
- **AND** vehicle control commands are processed without delay

#### Scenario: Handle multiple concurrent clients
- **WHEN** multiple web clients are connected (up to 5)
- **THEN** all clients receive telemetry broadcasts
- **AND** system performance remains stable
- **AND** control loop timing remains <10ms average

### Requirement: Web UI Visual Theme
The served control interface (`data/index.html`) SHALL present a single, coherent dark visual theme based on the "Sage Garden" palette, with all colors defined as CSS custom properties in a `:root` block and referenced via `var()` throughout the page. The theme SHALL NOT depend on any external network resource (no CDN, no remote fonts). Safety-relevant status colors SHALL remain clearly distinguishable.

#### Scenario: Dark theme applied as the single theme
- **WHEN** the page loads in a client browser
- **THEN** the page renders using the Sage Garden dark palette (dark background, light foreground, sage-green primary)
- **AND** no light-mode variant and no `prefers-color-scheme` switching is present
- **AND** colors are sourced from CSS custom properties defined in `:root`

#### Scenario: Distinguishable safety status colors
- **WHEN** the UI shows connection, control-source, and fault states
- **THEN** OK / connected states render in a green tone distinct from warning and error
- **AND** failsafe / warning states render in an amber tone distinct from OK and error
- **AND** sensor-fault / disconnected / error states render in a red tone distinct from OK and warning

#### Scenario: No external font or network dependency
- **WHEN** the page is served from LittleFS while the device has no internet access
- **THEN** the UI styles and fonts load entirely from the served page with no requests to Google Fonts or any CDN
- **AND** text renders using a system font stack fallback

### Requirement: Web UI Localization
The served control interface (`data/index.html`) SHALL provide its user-facing text in two
languages — English (`en`) and Ukrainian (`uk`) — using a translation dictionary and lookup
helper embedded inline in the single served page. The localization SHALL NOT introduce any
external network resource, additional file, or runtime fetch (no CDN, no remote i18n file),
preserving the offline operation of the portal. English SHALL be the fallback for any
translation key that is missing in the active language.

#### Scenario: Static text is translated
- **WHEN** the interface is displayed in a selected language
- **THEN** all static markup text (page title/header, status-bar labels, card headings,
  telemetry labels, button captions, help paragraphs, and warnings) renders in that language
- **AND** the text is sourced from an inline dictionary via `data-i18n` attributes

#### Scenario: Dynamic JavaScript strings are translated
- **WHEN** the interface generates text at runtime (connection state, MAVLink active/inactive
  indicator, gear-switching indicator, calibration/save status lines, `alert()`/`confirm()`
  dialogs, debug/boost toggles, and the ECU probe results panel)
- **THEN** those strings render in the active language via a `t(key)` lookup helper
- **AND** values embedded in a string (percentages, gear letters, counts) are substituted
  into the localized template

#### Scenario: Missing key falls back to English
- **WHEN** a translation key is absent from the active language's dictionary
- **THEN** the English value for that key is displayed
- **AND** no untranslated placeholder or blank text is shown

#### Scenario: No external dependency added by localization
- **WHEN** the localized page is served from LittleFS while the device has no internet access
- **THEN** all languages and the lookup logic load entirely from the served page
- **AND** no request is made to any CDN, remote font, or external i18n resource

### Requirement: Language Toggle Control
The interface SHALL present a language toggle control in the page header that switches the
entire UI between English and Ukrainian instantly, without reloading the page. When the
language changes, the `<html lang>` attribute SHALL be updated to the active language code.

#### Scenario: Toggle switches language without reload
- **WHEN** the operator activates the language toggle
- **THEN** all currently displayed static and dynamic text updates to the other language
- **AND** the page is not reloaded and current telemetry/control state is preserved

#### Scenario: Document language attribute reflects selection
- **WHEN** the active language is English
- **THEN** the `<html lang>` attribute is `en`
- **AND** when the active language is Ukrainian, the `<html lang>` attribute is `uk`

### Requirement: Language Persistence and Auto-Detection
The interface SHALL persist the selected language in `localStorage` and reapply it on
subsequent loads. On the first visit (no stored preference), the interface SHALL auto-detect
the language from the browser's `navigator.language` / `navigator.languages`, selecting
Ukrainian when a `uk` language prefix is present and English otherwise.

#### Scenario: Selection persists across reloads
- **WHEN** the operator selects a language and later reloads the page
- **THEN** the previously selected language is applied automatically from `localStorage`

#### Scenario: First-visit auto-detection
- **WHEN** the page loads with no stored language preference
- **AND** the browser's preferred language begins with `uk`
- **THEN** the interface displays in Ukrainian
- **AND** for any non-`uk` preferred language, the interface displays in English

### Requirement: Localized Display of Firmware Status Tokens
The interface SHALL map stable enum/status tokens received in WebSocket telemetry to
localized display labels on the client side, without changing the firmware or the
WebSocket/HTTP protocol. Free-form firmware response `message` strings, which the client
routes by matching their English wording, SHALL continue to be displayed as received
(English) and SHALL NOT be re-keyed, so the protocol contract remains unchanged.

#### Scenario: Enum tokens are shown in the active language
- **WHEN** telemetry reports a stable status token (input source `MAVLINK`/`WEB`/`FAILSAFE`,
  CAN status connected/disconnected, MAVLink active/inactive, steering sensor OK/fault,
  steering calibrated/not-calibrated, ignition state, cranking yes/no, or ECU probe status)
- **THEN** the corresponding label is displayed translated into the active language
- **AND** the underlying telemetry JSON field values are not altered

#### Scenario: Protocol is unchanged and free-form messages stay English
- **WHEN** the firmware sends a free-form `sendResponse` `message` string
- **THEN** the client displays that message as received without re-keying it
- **AND** no change is made to the firmware, WebSocket messages, or HTTP endpoints to
  support localization

### Requirement: Localized Unit Labels
The interface SHALL translate unit labels for which a Ukrainian convention exists while
leaving language-neutral units and numeric formatting unchanged.

#### Scenario: Locale-specific unit label
- **WHEN** the interface displays vehicle speed in Ukrainian
- **THEN** the unit label is `км/год`
- **AND** in English the unit label is `km/h`

#### Scenario: Language-neutral units are preserved
- **WHEN** the interface displays temperature, pressure, servo pulse width, frequency, or
  percentage values
- **THEN** the unit symbols `°C`, `kPa`, `µs`, `Hz`, and `%` are shown unchanged in both
  languages
- **AND** numeric values and decimal separators are not reformatted

