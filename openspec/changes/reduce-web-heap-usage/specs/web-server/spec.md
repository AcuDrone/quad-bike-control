## MODIFIED Requirements
### Requirement: SPIFFS/LittleFS Filesystem Integration
The system SHALL use SPIFFS or LittleFS filesystem to store and serve web interface files. The web
page SHALL be stored either as `/index.html.gz` (gzip-compressed, the form produced by the build) or
as `/index.html` (plain); at boot the system SHALL accept either form and SHALL warn only when
neither is present.

#### Scenario: Mount filesystem on startup
- **WHEN** WebPortal.begin() is called
- **THEN** SPIFFS/LittleFS filesystem is mounted
- **AND** mount success is verified
- **AND** error is logged if mount fails

#### Scenario: Compressed page found at boot
- **WHEN** the filesystem is mounted and `/index.html.gz` exists
- **THEN** the console SHALL report the page as found, with its name and size
- **AND** no "page not found" warning SHALL be printed

#### Scenario: Plain page found at boot
- **WHEN** the filesystem is mounted, `/index.html.gz` does not exist and `/index.html` exists
- **THEN** the console SHALL report `/index.html` as found, with its size
- **AND** no "page not found" warning SHALL be printed

#### Scenario: Page missing at boot
- **WHEN** the filesystem is mounted and neither `/index.html.gz` nor `/index.html` exists
- **THEN** the console SHALL print a warning naming both files and the `pio run -t uploadfs` hint
- **AND** the web server and WebSocket SHALL still start

#### Scenario: Serve files from filesystem
- **WHEN** HTTP request is made for static resource
- **THEN** file is read from SPIFFS/LittleFS
- **AND** appropriate content-type header is set based on file extension
- **AND** file contents are returned in HTTP response

## ADDED Requirements
### Requirement: Compressed Web Page Delivery
The filesystem image SHALL carry the web page gzip-compressed, produced at image build time from
`data/index.html`, which SHALL remain the only source of the page in the repository; no generated
file SHALL be committed. Building the image SHALL keep working with the unchanged commands
`pio run -t buildfs`, `pio run -t uploadfs` and `pio run -t uploadfsota`, and the image produced by
`buildfs` SHALL remain usable for the web LittleFS update. The page SHALL be sent to the browser
compressed with `Content-Encoding: gzip` and `Content-Type: text/html`.

#### Scenario: Image built from the plain source
- **WHEN** `pio run -t buildfs` or `pio run -t uploadfs` is run for the firmware environment
- **THEN** the image SHALL contain `/index.html.gz` (gzip of `data/index.html`, level 9) and SHALL
  NOT contain `/index.html`
- **AND** every other file in `data/` SHALL be packed unchanged
- **AND** the staging directory SHALL lie under `.pio/` (git-ignored) and `data/` SHALL be left
  unchanged
- **AND** the build output SHALL show the original and compressed sizes

#### Scenario: Other build targets unaffected
- **WHEN** `pio run`, `pio run -t upload` or `pio test -e native` is run
- **THEN** no staging or compression step SHALL run

#### Scenario: Staging failure stops the image build
- **WHEN** `data/` is missing or empty, or a staged file cannot be written
- **THEN** the image build SHALL fail with an error instead of producing an image without the page

#### Scenario: Page requested at the root URL
- **WHEN** a browser requests `/` (directly or after a captive-portal redirect) and the image holds
  `/index.html.gz`
- **THEN** the response SHALL be the compressed file with `Content-Encoding: gzip`,
  `Content-Type: text/html` and a Content-Length
- **AND** the browser SHALL render the same page as from the plain file

#### Scenario: Page requested by file name
- **WHEN** a browser requests `/index.html` and the image holds `/index.html.gz`
- **THEN** the compressed file SHALL be served with `Content-Encoding: gzip` and
  `Content-Type: text/html`

#### Scenario: Older plain image
- **WHEN** the image holds only a plain `/index.html`
- **THEN** it SHALL be served uncompressed as before

### Requirement: Bounded WebSocket Send Queue
The firmware SHALL limit the WebSocket send queue to 4 messages per client
(`WS_MAX_QUEUED_MESSAGES` = 4, set as a build flag for the library and the firmware), counting a
message until the client has acknowledged it. A full queue SHALL NOT close the client's connection:
a message that does not fit SHALL be discarded for that client only. The periodic telemetry
broadcast SHALL be skipped, for all clients, while any connected client has 3 or more messages
queued, so that one slot stays free for command replies.

#### Scenario: Slow client does not grow the heap
- **WHEN** a client stops acknowledging data (weak Wi-Fi) while telemetry is due every 200 ms
- **THEN** no more than 4 messages SHALL be queued for that client
- **AND** the heap held by the WebSocket queues SHALL stay bounded by about 4 telemetry messages

#### Scenario: Telemetry paused while a client lags
- **WHEN** any connected client has 3 or more messages queued at a telemetry tick
- **THEN** that telemetry frame SHALL be skipped for all clients
- **AND** the broadcast SHALL resume at the next tick at which every client has fewer than 3
  messages queued

#### Scenario: Command reply to a lagging client
- **WHEN** a command reply is sent while a client has 3 messages queued
- **THEN** the reply SHALL be queued for that client in the free slot

#### Scenario: Overflow does not disconnect
- **WHEN** a message is sent while a client already has 4 messages queued
- **THEN** that message SHALL be discarded for that client only
- **AND** the client's connection SHALL stay open
- **AND** the other clients SHALL receive the message

#### Scenario: Dead client still released
- **WHEN** a client's link is lost and its data stays unacknowledged
- **THEN** the connection SHALL be closed by the TCP acknowledge timeout and the client removed,
  releasing its queue
