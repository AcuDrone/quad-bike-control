# Change: Bound the WebSocket send queue and serve the web page gzip-compressed

## Why
The owner suspects the web UI eats RAM. Verified on `master` 1.5.38 (design "Audit"):
- **The WebSocket send queue can hold ≈ 145 KB.** Telemetry goes out on `/ws` every
  `TELEMETRY_INTERVAL` = 200 ms as one JSON text of ≈ 4–5 KB (up to ≈ 8–9 KB while the ECU probe /
  CAN sniffer blocks are present). `ESPAsyncWebServer` 3.6.0 keeps up to `WS_MAX_QUEUED_MESSAGES`
  = 32 unacknowledged messages per client on ESP32 (`AsyncWebSocket.h:28-29`). The broadcast only
  stops when a client's queue is full (`ws.availableForWriteAll()`, `src/WebPortal.cpp:115`), so a
  phone on a weak Wi-Fi link lets the queue grow to 32 × ≈ 4.5 KB ≈ 145 KB (≈ 280 KB with probe /
  sniff) — about all of the free internal heap (no PSRAM is used). The same backlog also means that
  client sees telemetry up to 32 × 200 ms ≈ 6.4 s old.
- **The page is served uncompressed.** `data/index.html` (≈ 391 KB on `master`, ≈ 443 KB on
  `aztec`) is sent as is; `gzip -9` makes it ≈ 81 KB (≈ 92 KB on `aztec`). Five times more airtime
  per page load on the same AP that carries telemetry, and a longer window in which the HTTP
  response and the WebSocket queue hold buffers at the same time.

## What Changes
- **WebSocket queue cap (build flag).** `-DWS_MAX_QUEUED_MESSAGES=4` in the `[env]` `build_flags`
  of `platformio.ini` (the only firmware env, `esp32-s3-devkitc-1`, inherits it; `native` overrides
  `build_flags` and does not build the web portal). The library's `#ifndef` guard takes it.
- **No disconnect on a full queue.** The library's default is to *close* a client whose queue is
  full when one more message is queued (`closeWhenFull = true`, `AsyncWebSocket.cpp:409-439`).
  Command replies (`WebPortal::sendResponse()` → `ws.textAll()`) are not gated today, so with a cap
  of 4 a reply sent while a slow client's queue is full would drop that client's connection and
  start a 3 s reconnect loop in the page. On `WS_EVT_CONNECT` the firmware SHALL call
  `client->setCloseClientOnQueueFull(false)`: an overflowing message is discarded for that client
  only. Dead links are still closed by the AsyncTCP acknowledge timeout (5 s).
- **One slot kept for command replies.** The telemetry gate stops broadcasting while any connected
  client has 3 or more messages queued (`WS_MAX_QUEUED_MESSAGES - 1`), instead of 4, so a command
  reply normally still finds a free slot on every client. Telemetry frames are skipped for **all**
  clients while one client is behind (as today, but after ≈ 0.6 s of backlog instead of ≈ 6.4 s).
- **Gzip-compressed page.** A PlatformIO pre-script `tools/fs_gzip.py` (in `extra_scripts` of
  `[env:esp32-s3-devkitc-1]`) runs only when a filesystem image is built (`buildfs`, `uploadfs`,
  `uploadfsota`). It stages `data/` into `.pio/build/<env>/fsdata/`, writing every `*.html` as
  `<name>.html.gz` (gzip level 9, fixed mtime) and copying any other file unchanged, and points
  `PROJECT_DATA_DIR` at the staging directory. `data/index.html` stays the single editable source
  in git; nothing generated is committed (`.pio` is already git-ignored). The owner's commands
  (`pio run -t uploadfs`, `pio run -t buildfs` + web LittleFS update with `littlefs.bin`) are
  unchanged.
- **Serving needs no handler change.** The library already serves `<path>.gz` with
  `Content-Encoding: gzip` when the plain file is absent, both for the explicit `/` handler
  (`AsyncFileResponse`, `WebResponses.cpp:642-648`) and for `serveStatic` (`WebHandlers.cpp:148-176`,
  `_tryGzipFirst = true`). Content type stays `text/html` (`/` passes it explicitly; `serveStatic`
  derives it from the requested name `index.html`).
- **Boot check.** `mountFilesystem()` accepts `/index.html.gz` or `/index.html`, prints which one
  and its size on the console, and warns only when neither exists (with the `uploadfs` hint).
- **Docs and release.** `WEB_PORTAL_SETUP.md` (upload section, troubleshooting), `platformio.ini`
  comments, firmware version bump and `include/Changelog.h` entry (uk + en).

## Out of scope
Cache-Control / favicon handling, telemetry rate or content, a client-count limit, a free-heap
field in telemetry, PSRAM use. Reducing the telemetry JSON size or the static 9216-byte document.

## Version (decided by the owner 2026-11-25)
- Both branches go to **1.6.2**, so the same change carries the same version on both vehicles
  (design D8). 1.6.2 is unused on `master` and `aztec` (checked 2026-11-25).
- `master` jumps 1.5.38 → **1.6.2**, skipping 1.5.39, 1.6.0 and 1.6.1 (1.6.0 / 1.6.1 were
  buggy-only iBooster releases that never existed on the quad); its changelog entry says so.
- `aztec` goes 1.6.1 → **1.6.2** through the plain merge `master` → `aztec`: it keeps its own
  1.6.0 / 1.6.1 changelog entries and ends with a single 1.6.2 entry.

## Impact
- Affected specs: `web-server`
  - MODIFIED: SPIFFS/LittleFS Filesystem Integration (boot check accepts the compressed page).
  - ADDED: Compressed Web Page Delivery, Bounded WebSocket Send Queue.
- Affected code: `platformio.ini`, new `tools/fs_gzip.py`, `src/WebPortal.cpp`
  (`broadcastTelemetry()` gate, `WS_EVT_CONNECT`, `mountFilesystem()`), `include/Constants.h`
  (version), `include/Changelog.h`, `WEB_PORTAL_SETUP.md`.
- No change to `data/index.html`, the telemetry JSON, the command protocol or any vehicle logic.
- Flashing order does not matter: older firmware already serves `index.html.gz` (same library), it
  only prints a false "index.html not found" warning at boot; new firmware still serves a plain
  `index.html` image.
- Upload the new LittleFS image (`pio run -t uploadfs` or the web LittleFS update); as always this
  erases the event journal — download it first.
