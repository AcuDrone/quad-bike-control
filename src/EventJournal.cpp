#include "EventJournal.h"
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdarg.h>

// ============================================================================
// State
// ============================================================================

static const char* const ACTIVE_PATH = "/events.log";
static const char* const ROTATED_PATH = "/events.1";

static_assert(sizeof(JRtcRing) <= 2304, "RTC journal ring must stay within its 2304 B budget");

// RTC slow memory, not initialized at boot: survives panic / watchdog / software reset.
RTC_NOINIT_ATTR JRtcRing s_ring;   // external linkage: the section attribute is applied as documented
static JRtcRing s_prevRing;                 // copy taken at boot for the replay

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;   // RAM buffer, ring, dedupe
static SemaphoreHandle_t s_fileMutex = nullptr;             // file operations + scratch buffer

static char s_ram[JOURNAL_RAM_BUF_BYTES];
static size_t s_ramLen = 0;
static uint32_t s_frontDropBytes = 0;     // bytes dropped from the RAM front (overflow), monotonic
static uint32_t s_dropPending = 0;        // lines dropped since the last journal_drop line
static char s_scratch[JOURNAL_RAM_BUF_BYTES];   // flush copy / tail scan (file mutex held)

static JDedupe<24> s_dedupe(JOURNAL_DEDUPE_MS);

static bool s_begun = false;
static uint32_t s_boot = 0;
static bool s_fsMounted = false;
static bool s_fsDisabled = false;
static volatile bool s_fsSuspended = false;
static uint8_t s_failCount = 0;
static uint32_t s_activeSize = 0;
static uint32_t s_rotatedSize = 0;
static int s_streams = 0;            // open HTTP streams (written under s_mux)
static volatile bool s_clearReq = false;

static uint32_t s_lastFlushMs = 0;
static uint32_t s_dueSinceMs = 0;         // 0 = no flush pending
static uint32_t s_loopMaxUs = 0;
static uint32_t s_jflushMaxUs = 0;
static uint32_t s_lastSnapMs = 0;
static uint32_t s_lastHeapCheckMs = 0;
static bool s_heapLow = false;

static EventJournal::SnapFn s_snapFn = nullptr;

// ============================================================================
// Helpers
// ============================================================================

static bool fileOutputOn() {
    return s_fsMounted && !s_fsDisabled && !s_fsSuspended;
}

static const char* resetToken(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        case ESP_RST_USB:       return "USB";
        case ESP_RST_JTAG:      return "JTAG";
        default:                return "UNKNOWN";
    }
}

// Append one complete line (with '\n') to the RAM buffer and the RTC ring. Drops the oldest
// whole lines on overflow (counted; the next logged line is preceded by journal_drop).
static void appendLine(const char* line, size_t len) {
    if (len == 0 || len > JOURNAL_LINE_MAX) return;
    portENTER_CRITICAL(&s_mux);
    if (s_ramLen + len > sizeof(s_ram)) {
        const size_t need = s_ramLen + len - sizeof(s_ram);
        size_t cut = 0;
        uint32_t lines = 0;
        while (cut < s_ramLen && cut < need) {
            const char* nl = (const char*)memchr(s_ram + cut, '\n', s_ramLen - cut);
            cut = nl ? (size_t)(nl - s_ram) + 1 : s_ramLen;
            lines++;
        }
        memmove(s_ram, s_ram + cut, s_ramLen - cut);
        s_ramLen -= cut;
        s_frontDropBytes += cut;
        s_dropPending += lines;
        if (s_begun) jRingConsume(s_ring, lines);   // lost, never reaching flash
    }
    memcpy(s_ram + s_ramLen, line, len);
    s_ramLen += len;
    if (s_begun) jRingPush(s_ring, line);
    portEXIT_CRITICAL(&s_mux);
}

static void appendFormatted(JSrc src, JLvl lvl, const char* event, const char* fields, uint32_t sup) {
    char line[JOURNAL_LINE_MAX + 1];
    const size_t n = jFormatLine(line, s_boot, millis(), src, lvl, event, fields, sup);
    appendLine(line, n);
}

// Drop the first `written` bytes of a flush snapshot from the RAM buffer (those taken by an
// overflow drop in the meantime are already gone) and release their ring slots.
static void commitFlushed(size_t written, uint32_t dropBase) {
    portENTER_CRITICAL(&s_mux);
    const uint32_t removed = s_frontDropBytes - dropBase;
    if (written > removed) {
        size_t k = written - removed;
        if (k > s_ramLen) k = s_ramLen;
        uint32_t lines = 0;
        for (size_t i = 0; i < k; i++) if (s_ram[i] == '\n') lines++;
        memmove(s_ram, s_ram + k, s_ramLen - k);
        s_ramLen -= k;
        jRingConsume(s_ring, lines);
    }
    portEXIT_CRITICAL(&s_mux);
}

static uint32_t fileSize(const char* path) {
    if (!LittleFS.exists(path)) return 0;
    File f = LittleFS.open(path, "r");
    if (!f) return 0;
    const uint32_t sz = (uint32_t)f.size();
    f.close();
    return sz;
}

// One flush attempt. File mutex held by the caller. Returns true on success (or nothing to do).
static bool doFlush() {
    size_t n;
    uint32_t dropBase;
    portENTER_CRITICAL(&s_mux);
    n = s_ramLen;
    memcpy(s_scratch, s_ram, n);
    dropBase = s_frontDropBytes;
    portEXIT_CRITICAL(&s_mux);
    if (n == 0) return true;

    // Rotation by rename (atomic on LittleFS); postponed while a download / tail streams.
    if (jShouldRotate(s_activeSize, n, JOURNAL_FILE_MAX_BYTES) && s_streams == 0) {
        LittleFS.remove(ROTATED_PATH);   // may not exist
        if (!LittleFS.rename(ACTIVE_PATH, ROTATED_PATH)) {
            return false;
        }
        s_rotatedSize = s_activeSize;
        s_activeSize = 0;
    }

    File f = LittleFS.open(ACTIVE_PATH, FILE_APPEND);
    if (!f) return false;
    const size_t w = f.write(reinterpret_cast<const uint8_t*>(s_scratch), n);
    f.close();   // sync
    if (w > 0) {
        s_activeSize += (uint32_t)w;
        commitFlushed(w, dropBase);
    }
    return w == n;
}

static void flushLocked() {
    const uint32_t t0 = micros();
    const bool ok = doFlush();
    const uint32_t dt = micros() - t0;
    if (dt > s_jflushMaxUs) s_jflushMaxUs = dt;
    if (ok) {
        s_failCount = 0;
        return;
    }
    if (++s_failCount >= 3) {
        s_fsDisabled = true;
        appendFormatted(JSrc::SYS, JLvl::E, "journal_off", "why=write_fail", 0);
    }
}

// Offset where the last `need` lines of `path` (size `size`) start; `found` = lines available
// (≤ need). Reads backwards in ≤ 1 KB chunks into s_scratch. File mutex held.
static uint32_t findTailOffset(const char* path, uint32_t size, uint32_t need, uint32_t& found) {
    found = 0;
    if (size == 0 || need == 0) return size;
    File f = LittleFS.open(path, "r");
    if (!f) return size;
    uint32_t pos = size;
    uint32_t count = 0;
    while (pos > 0) {
        const uint32_t chunk = pos > 1024 ? 1024 : pos;
        const uint32_t start = pos - chunk;
        if (!f.seek(start)) break;
        const size_t r = f.read(reinterpret_cast<uint8_t*>(s_scratch), chunk);
        if (r != chunk) break;
        for (int32_t i = (int32_t)chunk - 1; i >= 0; i--) {
            const uint32_t abs = start + (uint32_t)i;
            if (abs == size - 1) continue;   // the final newline closes the last line
            if (s_scratch[i] == '\n') {
                if (++count == need) {
                    f.close();
                    found = need;
                    return abs + 1;
                }
            }
        }
        pos = start;
    }
    f.close();
    found = count + 1;   // the first line has no newline before it
    if (found > need) found = need;
    return 0;
}

// ============================================================================
// Public API
// ============================================================================

void EventJournal::begin() {
    if (s_begun) return;
    s_fileMutex = xSemaphoreCreateMutex();

    const esp_reset_reason_t rr = esp_reset_reason();
    {
        Preferences prefs;
        if (prefs.begin("journal", false)) {
            s_boot = prefs.getUInt("boot", 0) + 1;
            prefs.putUInt("boot", s_boot);
            prefs.end();
        }
    }

    // The ring is meaningful only after a reset that kept RTC memory (never after power-on).
    const bool replay = (rr != ESP_RST_POWERON) && jRingValid(s_ring);
    if (replay) s_prevRing = s_ring;
    jRingReset(s_ring, s_boot);
    s_begun = true;

    char hdr[JOURNAL_LINE_MAX + 1];
    int n = snprintf(hdr, sizeof(hdr), "# boot=%lu fw=%s reset=%s\n",
                     (unsigned long)s_boot, FIRMWARE_VERSION, resetToken(rr));
    if (n > 0) appendLine(hdr, (size_t)n < sizeof(hdr) ? (size_t)n : sizeof(hdr) - 1);

    if (replay && s_prevRing.pending > 0) {
        uint32_t lost = 0;
        const size_t k = jRingReplay(s_prevRing, lost, [](const char*) {});
        char f[64];
        snprintf(f, sizeof(f), "n=%u lost=%lu from_boot=%lu", (unsigned)k, (unsigned long)lost,
                 (unsigned long)s_prevRing.boot);
        appendFormatted(JSrc::RTC, JLvl::I, "replay", f, 0);
        jRingReplay(s_prevRing, lost, [](const char* l) { appendLine(l, strlen(l)); });
    }

    char prevMin[16];
    if (replay && s_prevRing.prevHeapMin != 0) {
        snprintf(prevMin, sizeof(prevMin), "%lu", (unsigned long)s_prevRing.prevHeapMin);
    } else {
        strcpy(prevMin, "?");
    }
    log(JSrc::SYS, JLvl::I, "boot", "reset=%s fw=%s heap=%u prev_heap_min=%s",
        resetToken(rr), FIRMWARE_VERSION, (unsigned)ESP.getFreeHeap(), prevMin);
    s_lastSnapMs = millis();
}

void EventJournal::attachFs(bool mounted) {
    if (!mounted) {
        s_fsMounted = false;
        s_fsDisabled = true;
        log(JSrc::SYS, JLvl::E, "journal_off", "why=mount");
        return;
    }
    if (s_fileMutex && xSemaphoreTake(s_fileMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        s_activeSize = fileSize(ACTIVE_PATH);
        s_rotatedSize = fileSize(ROTATED_PATH);
        s_fsMounted = true;
        xSemaphoreGive(s_fileMutex);
    }
}

void EventJournal::log(JSrc src, JLvl lvl, const char* event, const char* fmt, ...) {
    char fields[160];
    fields[0] = '\0';
    if (fmt != nullptr) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(fields, sizeof(fields), fmt, ap);
        va_end(ap);
    }
    const uint32_t now = millis();
    uint32_t sup = 0;
    uint32_t drops = 0;
    portENTER_CRITICAL(&s_mux);
    const bool admit = s_dedupe.admit(src, event ? event : "", now, sup);
    if (admit) {
        drops = s_dropPending;
        s_dropPending = 0;
    }
    portEXIT_CRITICAL(&s_mux);
    if (!admit) return;
    if (drops > 0) {
        char f[24];
        snprintf(f, sizeof(f), "n=%lu", (unsigned long)drops);
        appendFormatted(JSrc::SYS, JLvl::W, "journal_drop", f, 0);
    }
    appendFormatted(src, lvl, event, fields, sup);
}

void EventJournal::noteLoop(uint32_t totalUs, const char* phase, uint32_t phaseUs) {
    if (totalUs > s_loopMaxUs) s_loopMaxUs = totalUs;
    if (totalUs > (uint32_t)JOURNAL_LOOP_OVERRUN_MS * 1000UL && phase != nullptr &&
        strcmp(phase, "journal") != 0) {
        log(JSrc::SYS, JLvl::W, "loop_overrun", "ms=%lu phase=%s phase_ms=%lu",
            (unsigned long)(totalUs / 1000UL), phase, (unsigned long)(phaseUs / 1000UL));
    }
}

static void writeSnap(uint32_t now) {
    // Suppressed counts still pending after 60 s go out right before the snapshot.
    struct Pend { JSrc src; char ev[JDedupe<24>::EV_MAX]; uint32_t n; };
    Pend pend[24];
    size_t np = 0;
    portENTER_CRITICAL(&s_mux);
    s_dedupe.drainPending(now, JOURNAL_SNAP_MS, [&](JSrc s, const char* ev, uint32_t n) {
        if (np < 24) {
            pend[np].src = s;
            strncpy(pend[np].ev, ev, sizeof(pend[np].ev) - 1);
            pend[np].ev[sizeof(pend[np].ev) - 1] = '\0';
            pend[np].n = n;
            np++;
        }
    });
    portEXIT_CRITICAL(&s_mux);
    for (size_t i = 0; i < np; i++) {
        EventJournal::log(JSrc::SYS, JLvl::I, "suppressed", "src=%s ev=%s n=%lu",
                          jSrcToken(pend[i].src), pend[i].ev, (unsigned long)pend[i].n);
    }

    const uint32_t heap = ESP.getFreeHeap();
    const uint32_t minHeap = ESP.getMinFreeHeap();
    portENTER_CRITICAL(&s_mux);
    jRingSetHeapMin(s_ring, minHeap);
    portEXIT_CRITICAL(&s_mux);

    char a[64] = "";
    char b[96] = "";
    if (s_snapFn) s_snapFn(a, sizeof(a), b, sizeof(b));

    char f1[160];
    snprintf(f1, sizeof(f1), "heap=%lu min=%lu loop_max=%lu jflush_max=%lu%s%s",
             (unsigned long)heap, (unsigned long)minHeap,
             (unsigned long)((s_loopMaxUs + 500) / 1000), (unsigned long)((s_jflushMaxUs + 500) / 1000),
             a[0] ? " " : "", a);
    appendFormatted(JSrc::SNAP, JLvl::I, "", f1, 0);
    if (b[0]) appendFormatted(JSrc::SNAP, JLvl::I, "", b, 0);
    s_loopMaxUs = 0;
    s_jflushMaxUs = 0;
}

void EventJournal::setSnapProvider(SnapFn fn) {
    s_snapFn = fn;
}

void EventJournal::service(bool steeringDriving) {
    if (!s_begun) return;
    const uint32_t now = millis();

    // Heap watch, once per second, with hysteresis.
    if (now - s_lastHeapCheckMs >= 1000) {
        s_lastHeapCheckMs = now;
        const uint32_t freeHeap = ESP.getFreeHeap();
        if (!s_heapLow && freeHeap < JOURNAL_HEAP_WARN_BYTES) {
            s_heapLow = true;
            log(JSrc::SYS, JLvl::W, "heap_low", "free=%lu min=%lu",
                (unsigned long)freeHeap, (unsigned long)ESP.getMinFreeHeap());
        } else if (s_heapLow && freeHeap > (uint32_t)JOURNAL_HEAP_WARN_BYTES + JOURNAL_HEAP_HYST_BYTES) {
            s_heapLow = false;
            log(JSrc::SYS, JLvl::I, "heap_ok", "free=%lu min=%lu",
                (unsigned long)freeHeap, (unsigned long)ESP.getMinFreeHeap());
        }
    }

    // Clear (requested over HTTP): executed here, never while a stream is open.
    if (s_clearReq && s_streams == 0 && xSemaphoreTake(s_fileMutex, 0) == pdTRUE) {
        if (fileOutputOn()) {
            LittleFS.remove(ROTATED_PATH);
            LittleFS.remove(ACTIVE_PATH);
        }
        s_activeSize = 0;
        s_rotatedSize = 0;
        portENTER_CRITICAL(&s_mux);
        s_ramLen = 0;
        s_dropPending = 0;
        jRingMarkFlushed(s_ring);
        portEXIT_CRITICAL(&s_mux);
        xSemaphoreGive(s_fileMutex);
        s_clearReq = false;
        log(JSrc::SYS, JLvl::I, "journal_cleared", "boot=%lu", (unsigned long)s_boot);
    }

    if (now - s_lastSnapMs >= JOURNAL_SNAP_MS) {
        s_lastSnapMs = now;
        writeSnap(now);
    }

    // Flush decision.
    if (s_ramLen == 0 || !fileOutputOn()) {
        s_dueSinceMs = 0;
        return;
    }
    const bool intervalDue = (now - s_lastFlushMs) >= JOURNAL_FLUSH_MS;
    const bool fillDue = s_ramLen >= (sizeof(s_ram) * 3) / 4 && s_failCount == 0;
    if (!intervalDue && !fillDue) return;
    if (s_dueSinceMs == 0) s_dueSinceMs = now | 1u;
    // Decision 2: no flash erase while the steering motor is driven (a sector erase stalls both
    // cores for 30-50 ms; the LWS timeout is 100 ms) — at most JOURNAL_FLUSH_DEFER_MAX_MS.
    if (steeringDriving && (now - s_dueSinceMs) < JOURNAL_FLUSH_DEFER_MAX_MS) return;
    if (xSemaphoreTake(s_fileMutex, 0) != pdTRUE) {
        s_lastFlushMs = now;   // an HTTP reader holds it: retry one interval later, never wait
        return;
    }
    flushLocked();
    xSemaphoreGive(s_fileMutex);
    s_lastFlushMs = millis();
    s_dueSinceMs = 0;
}

void EventJournal::flushNow(const char* why) {
    log(JSrc::SYS, JLvl::I, "reboot", "why=%s", why ? why : "?");
    if (!fileOutputOn() || s_fileMutex == nullptr) return;
    if (xSemaphoreTake(s_fileMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return;
    flushLocked();
    xSemaphoreGive(s_fileMutex);
}

void EventJournal::suspendFs() {
    bool taken = s_fileMutex && xSemaphoreTake(s_fileMutex, pdMS_TO_TICKS(500)) == pdTRUE;
    s_fsSuspended = true;
    if (taken) xSemaphoreGive(s_fileMutex);
}

void EventJournal::requestClear() {
    s_clearReq = true;
}

int32_t EventJournal::journalBytes() {
    if (!fileOutputOn()) return -1;
    return (int32_t)(s_activeSize + s_rotatedSize + s_ramLen);
}

uint32_t EventJournal::bootNumber() {
    return s_boot;
}

// ============================================================================
// Streams (HTTP tail / download)
// ============================================================================

std::shared_ptr<EventJournal::Stream> EventJournal::openStream(size_t tailLines) {
    std::shared_ptr<Stream> st(new Stream());

    // Unflushed RAM lines (copied: the loop may flush them while the stream runs).
    portENTER_CRITICAL(&s_mux);
    const size_t n = s_ramLen;
    portEXIT_CRITICAL(&s_mux);
    if (n > 0) {
        st->ram_ = (char*)malloc(n);
        if (st->ram_ != nullptr) {
            portENTER_CRITICAL(&s_mux);
            const size_t m = s_ramLen < n ? s_ramLen : n;
            memcpy(st->ram_, s_ram, m);
            portEXIT_CRITICAL(&s_mux);
            st->ramLen_ = m;
        }
    }
    const size_t ramLines = st->ram_ ? jCountLines(st->ram_, st->ramLen_) : 0;
    if (tailLines > 0 && ramLines >= tailLines) {
        st->ramPos_ = jTailStart(st->ram_, st->ramLen_, tailLines);
        return st;   // the RAM lines alone are enough
    }

    if (!fileOutputOn() || s_fileMutex == nullptr ||
        xSemaphoreTake(s_fileMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return st;   // RAM lines only
    }
    portENTER_CRITICAL(&s_mux);
    s_streams++;
    portEXIT_CRITICAL(&s_mux);
    st->counted_ = true;
    const uint32_t rotated = s_rotatedSize;
    const uint32_t active = s_activeSize;
    if (tailLines == 0) {
        st->seg_[0] = {0, 0, rotated};
        st->seg_[1] = {1, 0, active};
    } else {
        const uint32_t need = (uint32_t)(tailLines - ramLines);
        uint32_t found = 0;
        const uint32_t offA = findTailOffset(ACTIVE_PATH, active, need, found);
        st->seg_[1] = {1, offA, active};
        if (found < need) {
            uint32_t found0 = 0;
            const uint32_t offR = findTailOffset(ROTATED_PATH, rotated, need - found, found0);
            st->seg_[0] = {0, offR, rotated};
        } else {
            st->seg_[0] = {0, rotated, rotated};
        }
    }
    xSemaphoreGive(s_fileMutex);
    return st;
}

EventJournal::Stream::~Stream() {
    free(ram_);
    if (counted_) {
        portENTER_CRITICAL(&s_mux);
        s_streams--;
        portEXIT_CRITICAL(&s_mux);
    }
}

size_t EventJournal::Stream::read(uint8_t* buf, size_t maxLen) {
    if (maxLen == 0) return TRY_AGAIN;
    while (segIdx_ < 2) {
        Seg& seg = seg_[segIdx_];
        if (seg.pos >= seg.end || !fileOutputOn()) {
            segIdx_++;
            continue;
        }
        if (xSemaphoreTake(s_fileMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
            return TRY_AGAIN;   // a flush is running: come back on the next poll
        }
        size_t r = 0;
        File f = LittleFS.open(seg.file ? ACTIVE_PATH : ROTATED_PATH, "r");
        if (f) {
            if (f.seek(seg.pos)) {
                size_t want = seg.end - seg.pos;
                if (want > maxLen) want = maxLen;
                if (want > 1024) want = 1024;
                r = f.read(buf, want);
            }
            f.close();
        }
        xSemaphoreGive(s_fileMutex);
        if (r == 0) {
            seg.pos = seg.end;   // unreadable: skip the rest of this file
            continue;
        }
        seg.pos += (uint32_t)r;
        return r;
    }
    if (ram_ != nullptr && ramPos_ < ramLen_) {
        size_t k = ramLen_ - ramPos_;
        if (k > maxLen) k = maxLen;
        memcpy(buf, ram_ + ramPos_, k);
        ramPos_ += k;
        return k;
    }
    return 0;
}
