# Design: bounded WebSocket queue and gzip-compressed page

## Context
The web portal (`src/WebPortal.cpp`) runs on `mathieucarbou/ESPAsyncWebServer` 3.6.0 + AsyncTCP
(`platformio.ini` `lib_deps`: `mathieucarbou/ESPAsyncWebServer@^3.4.6`, resolved to 3.6.0 in
`.pio/libdeps/esp32-s3-devkitc-1`). The board runs without PSRAM; the journal already watches the
heap: `SNAP I … heap=<free> min=<minimum> …` every `JOURNAL_SNAP_MS` (60 s,
`src/EventJournal.cpp:394`), `SYS W heap_low` below 40 KB and `SYS I heap_ok` above 48 KB
(`src/EventJournal.cpp:418-422`, `include/Constants.h` `JOURNAL_HEAP_WARN_BYTES` /
`JOURNAL_HEAP_HYST_BYTES`). Line references below are `master` 1.5.38; `aztec` has the same code at
other line numbers.

## Audit of `master` 1.5.38 (verified)
- Telemetry: `WebPortal::broadcastTelemetry()` (`src/WebPortal.cpp:101-121`) every
  `TELEMETRY_INTERVAL` = 200 ms (`include/Constants.h:930`), skipped while `ws.count() == 0`, sent
  only if `ws.availableForWriteAll()`; the JSON is built into a `String` from a
  `static StaticJsonDocument<9216>` (`src/WebPortal.cpp:696`), ≈ 2.2 KB of document / ≈ 4–5 KB of
  JSON in steady state, ≈ 8.7 KB worst case with `probe` + `sniff`.
- Command replies: `WebPortal::sendResponse()` (`src/WebPortal.cpp:123-126`) →
  `ws.textAll(json)`, **not gated**; ≈ 60 call sites in `src/VehicleController.cpp`, plus
  "Invalid command format" (`src/WebPortal.cpp:593`) and the ArduinoOTA start / error notices
  (`src/WebPortal.cpp:495`, `:528`). There is no per-client `ws.text(id)` / `client->text()` in the
  firmware — every reply goes to every client.
- Page: `server.on("/")` → `request->send(FILESYSTEM, "/index.html", "text/html")`
  (`src/WebPortal.cpp:173-175`), `server.serveStatic("/", FILESYSTEM, "/")` (`:442`), captive-portal
  probes and `onNotFound` redirect to `/`. Boot check `FILESYSTEM.exists("/index.html")` with a
  console-only warning (`src/WebPortal.cpp:556-559`). No `.gz`, no build step, no Cache-Control.
- Filesystem build: `board_build.filesystem = littlefs`, default `data_dir` (`data/`), no
  `extra_scripts`. `data/` holds only `index.html` (390 686 B on `master`; `gzip -9` → 80 813 B).
  Upload paths in use: `pio run -t uploadfs` (`WEB_PORTAL_SETUP.md` §2), and the web LittleFS
  update (`POST /update?type=littlefs` or a file named `*littlefs*`, `src/WebPortal.cpp:357-372`),
  fed with `.pio/build/<env>/littlefs.bin` from `pio run -t buildfs`. ArduinoOTA also accepts a
  filesystem image (`src/WebPortal.cpp:485-490`, `uploadfsota` when `upload_protocol = espota`).
- Native tests: nothing under `test/` touches `index.html`, the filesystem or the WebSocket queue.

## Library behaviour (ESPAsyncWebServer 3.6.0, verified in `.pio/libdeps`)
- `WS_MAX_QUEUED_MESSAGES` is guarded: `#ifndef WS_MAX_QUEUED_MESSAGES / #define … 32` for ESP32
  (`src/AsyncWebSocket.h:28-29`), so a `-D` build flag replaces it. PlatformIO's global
  `build_flags` apply to library sources too, so the library and the firmware see the same value.
- The queue counts a message until it is fully acknowledged: `_onAck()` → `_clearQueue()` removes
  only finished messages (`src/AsyncWebSocket.cpp:310-338`), so in-flight data is inside the cap.
- `client.queueIsFull()` = `size >= WS_MAX_QUEUED_MESSAGES || status != WS_CONNECTED`
  (`src/AsyncWebSocket.cpp:372-377`); `availableForWriteAll()` = no client full
  (`src/AsyncWebSocket.cpp:787-789`).
- `textAll()` makes **one** shared buffer and queues it on every connected client
  (`src/AsyncWebSocket.cpp:899-950`): the heap held by the queues is "distinct messages queued on
  the slowest client" × size, not × number of clients.
- On overflow `_queueMessage()` (`src/AsyncWebSocket.cpp:409-439`): if `closeWhenFull` (default
  **true**, `src/AsyncWebSocket.h:160`) the client is marked disconnected and its TCP connection
  closed; otherwise the new message is discarded for that client. Either way a `log_e` is printed,
  which `CORE_DEBUG_LEVEL=0` compiles out — the overflow is silent in this firmware.
  `setCloseClientOnQueueFull(bool)` (`src/AsyncWebSocket.h:206`) changes it per client; the header
  recommends setting it in `WS_EVT_CONNECT`.
- Dead links are closed independently by AsyncTCP's acknowledge timeout
  (`CONFIG_ASYNC_TCP_MAX_ACK_TIME` 5000 ms, `AsyncTCP.h:75-76`, checked in `AsyncTCP.cpp:1119-1122`).
- Gzip: `AsyncFileResponse(FS&, path, …)` serves `path + ".gz"` with `content-encoding: gzip` and
  a Content-Length when `path` does not exist and `path.gz` does, unless it is a download
  (`src/WebResponses.cpp:642-648`); the given content type is kept. `serveStatic`'s
  `_searchFile()` tries `path.gz` first (`_tryGzipFirst = true`, `src/WebHandlerImpl.h:50`;
  `src/WebHandlers.cpp:148-176`) and `AsyncFileResponse(File, path, …)` adds the encoding header
  when the opened file ends in `.gz` and the requested path does not
  (`src/WebResponses.cpp:676-680`); the content type comes from the requested name (`index.html`
  → `text/html`). The `Accept-Encoding` request header is not checked (every browser in use
  accepts gzip).

## Goals / Non-Goals
- Goals: bound the heap the WebSocket queues can take; keep command replies reaching the page and
  never disconnect a client because of the cap; cut the page transfer ≈ 5×; keep `data/index.html`
  the only source and the owner's upload commands unchanged; make the result measurable with the
  journal.
- Non-Goals: Cache-Control / favicon, telemetry rate or content, client-count limit, a free-heap
  telemetry field, PSRAM, shrinking the telemetry JSON or the static JSON document.

## Decisions

### D1 — Cap = 4 messages, set as a global build flag
- `-DWS_MAX_QUEUED_MESSAGES=4` in `[env]` `build_flags` with a comment (why, and that the firmware
  reads the macro too). Only `esp32-s3-devkitc-1` builds the portal; `native` replaces
  `build_flags` entirely, so it is unaffected; a future firmware env inherits it.
- Bound: 4 × ≈ 4.5 KB ≈ 18 KB steady (≈ 35 KB worst with `probe` + `sniff`) instead of
  ≈ 145 KB / ≈ 280 KB. 4 = 800 ms of telemetry, enough to ride out a normal Wi-Fi retry burst
  without skipping frames on a good link.
- Alternatives: 2 (frames skipped on ordinary jitter), 8 (≈ 36 / 70 KB, still a large share of the
  heap). Patching the library: rejected, the flag is the supported knob.

### D2 — Never close a client on overflow
With the cap at 4 the queue of a slow client sits at the limit most of the time (the telemetry gate
fills it exactly to the gate level). With the library default, the next **command reply**
(`ws.textAll`, ungated) would close that client; the page then shows "Disconnected", reverts
pending values (`govRevertAll()` etc.) and reconnects after 3 s (`data/index.html:2938-2948`) —
on a weak link that repeats on every command, and every reconnect re-allocates. Today the same
can happen at 32, but rarely.
- `onWebSocketEvent(WS_EVT_CONNECT)`: `client->setCloseClientOnQueueFull(false)`. An overflowing
  message is then dropped for that client only; other clients still get it.
- Consequence, stated honestly: a discarded **reply** means that client's card status line is not
  updated for that command. The page's state does not depend on replies — values are confirmed
  from telemetry (e.g. `govPending` cleared when telemetry agrees, `data/index.html:4536-4544`) —
  so nothing is left inconsistent. A discarded **telemetry** frame cannot happen (D3 gates it).
- Hung or vanished clients are still closed by the AsyncTCP 5 s acknowledge timeout and removed by
  `ws.cleanupClients()` (`src/WebPortal.cpp:90`).

### D3 — Telemetry leaves one slot for replies
- `broadcastTelemetry()` replaces `ws.availableForWriteAll()` by a check that every connected
  client has `queueLen() < WS_MAX_QUEUED_MESSAGES - 1` (iterate `ws.getClients()`, skip clients not
  `WS_CONNECTED`), i.e. telemetry fills at most 3 of the 4 slots. A reply then still fits on a
  lagging client unless two replies arrive within the same backlog.
- Behaviour: while any client has ≥ 3 messages not yet acknowledged, the 200 ms telemetry tick is
  skipped for **all** clients (as today's gate, only earlier); nothing queues beyond 4 per client.
  A slow phone therefore lowers the telemetry rate seen by a second, healthy client; with a single
  service laptop/phone (the normal case) this is irrelevant. The slow client's data is at most
  ≈ 0.6 s old instead of up to ≈ 6.4 s.
- The iteration has the same thread-safety as the library's own `availableForWriteAll()` (it walks
  the same list from the same `loop()` context, `src/AsyncWebSocket.cpp:787-789`).
- Alternative: keep `availableForWriteAll()` and rely on D2 only — simpler by ≈ 6 lines, but on a
  lagging client almost every reply would be discarded. **Decided by the owner 2026-11-25
  (former Q1): option (a), the gate at 3 of 4.**

### D4 — Gzip at filesystem-image build time, staged under `.pio`
- `tools/fs_gzip.py`, `extra_scripts = pre:tools/fs_gzip.py` in `[env:esp32-s3-devkitc-1]`.
- It acts only when `{"buildfs", "uploadfs", "uploadfsota"} ∩ COMMAND_LINE_TARGETS` is non-empty,
  so `pio run`, `upload`, `monitor` and `pio test -e native` are untouched.
- Staging dir: `$BUILD_DIR/fsdata` (`.pio/build/esp32-s3-devkitc-1/fsdata/`). It is wiped and
  rebuilt on every run; every file of `data/` is copied, every `*.html` is written instead as
  `<name>.html.gz` with `gzip.GzipFile(…, compresslevel=9, mtime=0)`, and every staged file keeps
  the source file's mtime (`shutil.copystat` / `copy2`) — found while implementing: the image
  builder stores each file's mtime in the LittleFS image, so a fresh mtime on every staging made
  two `buildfs` runs differ; with the source mtime the same `data/` gives a byte-identical
  `littlefs.bin`. Then `env.Replace(PROJECT_DATA_DIR=<staging>)`. Under `.pio`, so already git-ignored
  (`.gitignore:1`) — no `.gitignore` change, nothing generated in `data/`.
- Why a `pre:` script and `PROJECT_DATA_DIR`: the platform builder creates the image node as
  `env.DataToBin(…, "$PROJECT_DATA_DIR")` when its `main.py` runs
  (`~/.platformio/platforms/espressif32/builder/main.py:797-803`); `pre:` scripts run before it, so
  the replaced value is the one packed (the current pioarduino platform packs it with its built-in
  `build_fs_router` / Python littlefs, not the `mklittlefs` binary — verified with `-v`; the
  `PROJECT_DATA_DIR` hand-off is the same). The staging must exist at script time (not as a
  SCons action) because the source directory is scanned when the node is created. Overriding
  `data_dir` in `platformio.ini` instead would make the plain `data/` unusable and need a
  committed or hand-made output folder — rejected.
- The script prints one line: `fs_gzip: index.html 390686 B -> 80813 B (staged in …)` and fails the
  build (raises) if `data/` is missing, empty, or a write fails, so an image without the page is
  never produced silently.
- Unchanged commands: `pio run -t uploadfs`; `pio run -t buildfs` then the web LittleFS update with
  `.pio/build/esp32-s3-devkitc-1/littlefs.bin`; ArduinoOTA filesystem upload. The image is smaller,
  so the web / OTA filesystem update is faster too.

### D5 — Serving: no handler change
The library already prefers / falls back to `.gz` (Library behaviour above). The `/` handler keeps
`request->send(FILESYSTEM, "/index.html", "text/html")`; `/index.html` via `serveStatic` and every
captive-portal redirect to `/` get the compressed file with `content-encoding: gzip` and
`content-type: text/html`. If an image still contains the plain `index.html` (older image), it is
served as today.

### D6 — Boot check and messages
`mountFilesystem()`: look for `/index.html.gz`, then `/index.html`; console
`  ✓ Web page: /index.html.gz (80813 B, gzip)` or `  ✓ Web page: /index.html (… B)`; if neither,
`  ⚠ Web page not found (/index.html.gz or /index.html) — run 'pio run -t uploadfs'`.
No journal line is added: the page that shows the journal is exactly what is missing, the console
message is where the owner looks after an upload, and the journal is attached to the filesystem
elsewhere (`EventJournal::attachFs`) — keeping this change out of the journal's format.

### D7 — Compatibility of firmware and image versions
Firmware and LittleFS image are flashed independently. Old firmware + new image: served correctly
(same library behaviour), only the old boot warning "/index.html not found" is printed (harmless).
New firmware + old image: plain file served, boot line shows `/index.html`. No ordering
constraint.

### D8 — Version (decided by the owner 2026-11-25, former Q2)
- `openspec/project.md`: fixes / no recalibration / no wiring = patch. `aztec` already shipped
  1.6.0 and 1.6.1 (aztec-only, iBooster) on top of 1.5.38, so **both branches go to 1.6.2** and the
  same change carries the same version on both vehicles. 1.6.2 is unused on `master` and `aztec`
  (git grep, 2026-11-25).
- `master`: `include/Constants.h` `FIRMWARE_VERSION` 1.5.38 → **1.6.2**, skipping 1.5.39, 1.6.0 and
  1.6.1. `include/Changelog.h` gets ONE entry 1.6.2 (uk + en) on top of 1.5.38; its last line notes
  that 1.6.0 and 1.6.1 were buggy-only releases, which explains the gap in the quad's list.
- `aztec`: 1.6.1 → **1.6.2** through the plain merge `master` → `aztec`. `FIRMWARE_VERSION`
  conflicts textually (1.6.1 vs 1.6.2) and resolves to 1.6.2. In `include/Changelog.h` both sides
  insert above 1.5.38; the resolution is 1.6.2 (newest first), then aztec's existing 1.6.1 and
  1.6.0, then 1.5.38 — a single 1.6.2 entry, no duplicate, no separate 1.5.39 entry. The gap note
  is dropped on aztec (1.6.0 / 1.6.1 are listed there); "багі" wording where the vehicle is named.
- From now on the next free version on both branches is **1.6.3**, unless a branch-only change
  intervenes.
- Considered: (a) master 1.5.39 and aztec 1.6.2 after the merge — one change under two numbers;
  (c) aztec keeps 1.6.1 — the buggy would show an unchanged version for changed firmware.
- `old-board` is not maintained (owner decision 2026-11-24): no merge.

## Risks / Trade-offs
- A healthy client's telemetry rate drops while another client lags (D3) → accepted; it is today's
  behaviour with a 8× shorter backlog; one service device is the norm.
- A command reply can still be discarded for a lagging client (two replies inside one backlog) →
  the status line misses one message; state comes from telemetry.
- Overflows are silent (`log_e` compiled out) → bench measures the effect through the journal heap
  figures; adding a counter is out of scope.
- Heap saving from gzip is smaller than from the queue cap: the file is streamed in chunks either
  way; gzip mainly shortens the transfer (≈ 5×) and therefore the time HTTP buffers coexist with
  the WebSocket queue, and the airtime shared with telemetry.
- PlatformIO / pioarduino builder internals (`PROJECT_DATA_DIR`, target names) could change on a
  platform update → the script fails loudly rather than packing the wrong directory (it checks
  that the staged file exists after staging and prints the directory used).

## Migration Plan
1. Implement on `master`, build, flash firmware, `pio run -t uploadfs` (download the journal first).
2. Bench (tasks §6). 3. Plain merge `master` → `aztec`, version per D8, repeat the buggy bench.
Rollback: flash the previous firmware; the gzip image keeps working with it (D7). To roll back the
image, remove the `extra_scripts` line and run `uploadfs`.

## Decided Questions (owner, 2026-11-25)
- **Reply slot (D3), former Q1: (a)** telemetry gate at 3 of 4 (`WS_MAX_QUEUED_MESSAGES - 1`),
  replies keep a free slot. Not chosen: (b) keep `availableForWriteAll()` and rely only on "no
  close on overflow" (D2) — replies on a lagging client mostly discarded.
- **Versions (D8), former Q2: (b)** both branches 1.6.2. Not chosen: (a) master 1.5.39 / aztec
  1.6.2; (c) aztec keeps 1.6.1.
