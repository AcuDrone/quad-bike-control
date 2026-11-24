## 0. Preconditions
- [x] 0.1 Proposal approved by the owner. Decided 2026-11-25: reply slot = telemetry gate at 3 of 4 (design D3); versions = 1.6.2 on both `master` and `aztec` (design D8).
- [x] 0.2 No other active change in `openspec/changes/` touches `src/WebPortal.cpp` WebSocket / filesystem code or `platformio.ini` `build_flags` / `extra_scripts`; `1.6.2` is still unclaimed on both `master` and `aztec` at implementation time (`git grep -n '1\.6\.2' master` / `aztec`; unused on both when checked 2026-11-25, `master` at 1.5.38, `aztec` at 1.6.1).
- [x] 0.3 Work starts on `master`; `aztec` gets it only by a plain merge of `master` (§5). `old-board` is not maintained — no merge.
- [ ] 0.4 Baseline for the bench, before flashing (each vehicle, current firmware): download the event journal; note SNAP `heap=` / `min=` over ≥ 5 min with the page open on a good link and on a weak link (§6.2), and the page load time (§6.4).

## 1. WebSocket queue (`master`)
- [x] 1.1 `platformio.ini` `[env]` `build_flags`: add `-DWS_MAX_QUEUED_MESSAGES=4` with a comment (library default 32 ≈ 145 KB on a slow client; the firmware reads the macro for its telemetry gate; change `reduce-web-heap-usage`).
- [x] 1.2 `src/WebPortal.cpp` `onWebSocketEvent()` `WS_EVT_CONNECT`: `client->setCloseClientOnQueueFull(false)` with a one-line reason (design D2).
- [x] 1.3 `src/WebPortal.cpp` `broadcastTelemetry()`: replace `ws.availableForWriteAll()` by a helper that returns false when any `WS_CONNECTED` client in `ws.getClients()` has `queueLen() >= WS_MAX_QUEUED_MESSAGES - 1` (design D3; owner decision 2026-11-25). Add a `static_assert(WS_MAX_QUEUED_MESSAGES >= 2, …)`.
- [x] 1.4 Check: `sendResponse()` and every other WebSocket send stay unchanged (`ws.textAll`); grep confirms no `ws.text(`/`client->text(` was added.

## 2. Gzip image (`master`)
- [x] 2.1 `tools/fs_gzip.py` (PlatformIO `pre:` script, design D4): acts only when `buildfs` / `uploadfs` / `uploadfsota` is in `COMMAND_LINE_TARGETS`; rebuilds `$BUILD_DIR/fsdata`; copies `data/`, writing each `*.html` as `*.html.gz` (`compresslevel=9`, `mtime=0`), staged files keeping the source mtime (byte-identical image, design D4); `env.Replace(PROJECT_DATA_DIR=…)`; prints `fs_gzip: <name> <orig> B -> <gz> B (staged in <dir>)`; raises on missing / empty `data/` or a failed write and checks the staged files exist.
- [x] 2.2 `platformio.ini` `[env:esp32-s3-devkitc-1]`: `extra_scripts = pre:tools/fs_gzip.py` with a comment (data/index.html stays the source; staging under `.pio`).
- [x] 2.3 `src/WebPortal.cpp` `mountFilesystem()`: page check per design D6 (`/index.html.gz`, then `/index.html`, size on the console; warning only when neither exists). No handler change (design D5).
- [x] 2.4 Host checks: `pio run -t buildfs` prints the `fs_gzip` line; the staged dir holds only `index.html.gz`; `gunzip -c .pio/build/esp32-s3-devkitc-1/fsdata/index.html.gz | cmp - data/index.html`; running `buildfs` twice gives byte-identical `littlefs.bin`; `git status` shows nothing new under `data/`; `pio run` (no fs target) prints no `fs_gzip` line.

## 3. Docs, changelog, version (`master`)
- [x] 3.1 `include/Constants.h` `FIRMWARE_VERSION` `1.5.38` → `1.6.2` (design D8; skips 1.5.39, 1.6.0, 1.6.1).
- [x] 3.2 `include/Changelog.h`: ONE entry 1.6.2 on top of 1.5.38 (uk + en, operator wording): the web page loads about five times faster (it is sent compressed); a phone on a weak Wi-Fi link no longer eats the board's memory and its readings are no more than a second old; the page is no longer disconnected after a command on a weak link; no recalibration; upload the updated LittleFS image (download the journal first). Last line: versions 1.6.0 and 1.6.1 were buggy-only releases (so the gap in the quad's list is explained).
- [x] 3.3 `WEB_PORTAL_SETUP.md`: §2 — the image packs `index.html.gz` built from `data/index.html` (edit only `data/index.html`; never commit a `.gz`); web LittleFS update uses `.pio/build/esp32-s3-devkitc-1/littlefs.bin` from `pio run -t buildfs`; Troubleshooting — the boot console line names `/index.html.gz`; `/index.html` URL still works.
- [x] 3.4 `pio run -e esp32-s3-devkitc-1` builds without new warnings; `pio test -e native` green (no test touches this code; suite list unchanged).
- [x] 3.5 `openspec validate reduce-web-heap-usage --strict --no-interactive` passes.

## 4. Firmware check on the bench PC (`master`)
- [x] 4.1 `strings`/map or a temporary console print confirms the library was built with 4 (e.g. log `WS_MAX_QUEUED_MESSAGES` once in `setupWebSocket()`). Done without a print: disassembly of the library's `AsyncWebSocketClient::queueIsFull()` in `AsyncWebSocket.cpp.o` compares the queue size with `bgeui a10, 4`.

## 5. Merge
- [ ] 5.1 Plain merge `master` → `aztec`: `FIRMWARE_VERSION` conflicts textually (`1.6.1` vs `1.6.2`) and resolves to `1.6.2`; `include/Changelog.h` conflict resolves to the single 1.6.2 entry on top, then aztec's existing 1.6.1 and 1.6.0, then 1.5.38 — no duplicate, no 1.5.39 entry; the buggy-only gap note is dropped on aztec and "багі" wording is used where the vehicle is named (design D8); `aztec`'s `platformio.ini` keeps its own envs plus the flag and `extra_scripts`; `pio run`, `pio run -t buildfs`, `pio test -e native` on `aztec`.

## 6. Bench — both vehicles (code and bench are separate: nothing here is done in §1–5)
Measure with the event journal: SNAP `heap=` / `min=` every 60 s, `heap_low` / `heap_ok` lines; compare with the 0.4 baseline over the same duration.
- [ ] 6.1 Flash firmware, then `pio run -t uploadfs` (journal downloaded first). Boot console shows `Web page: /index.html.gz (… B, gzip)`. Browser dev tools on `/`: `content-encoding: gzip`, `content-type: text/html`, transfer ≈ 81 KB (≈ 92 KB on aztec); page and both languages render; `/index.html` also loads; captive-portal popup on a phone opens the page.
- [ ] 6.2 Heap, good link: page open on Diagnostics for 10 min → `min=` not lower than the baseline. Weak link: phone at the edge of Wi-Fi range (or behind a wall / in a metal box) for 10 min, page open → baseline vs new `min=`; expected: the baseline drop of up to ≈ 100–145 KB is gone (new drop ≤ ≈ 20–35 KB), no `heap_low`.
- [ ] 6.3 Weak link behaviour: the page stays "Connected"; telemetry freezes and resumes rather than disconnecting; readings visibly current within ≈ 1 s after the link recovers; send ≥ 10 commands (e.g. calibration sliders) while the link is weak → no disconnect, card status lines update (an occasional missing status line is acceptable, values must match telemetry).
- [ ] 6.4 Page load time (dev tools "Load", hard reload, cache disabled), good link and weak link, 3 runs each, baseline vs new.
- [ ] 6.5 Two clients: laptop on a good link + phone on a weak link → laptop telemetry may slow while the phone lags (expected, design D3) but never stops for more than a few seconds; walking the phone out of range entirely → it is dropped within ≈ 5–10 s and the laptop rate returns to 5 Hz.
- [ ] 6.6 Web LittleFS update with `littlefs.bin` from `pio run -t buildfs`, and a web firmware update: both complete and reboot; page served gzip afterwards.
- [ ] 6.7 Compatibility (one vehicle): previous firmware + new image → page loads (boot warning about `/index.html` expected); new firmware + previous plain image → page loads, boot line names `/index.html`.

## 7. Archive
- [ ] 7.1 Archive once the bench checks on both vehicles pass; `openspec validate --strict` after archiving.
