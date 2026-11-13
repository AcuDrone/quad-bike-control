#ifndef EVENT_JOURNAL_LAW_H
#define EVENT_JOURNAL_LAW_H

/**
 * @file EventJournalLaw.h
 * @brief Pure (Arduino-free, header-only) logic of the persistent event journal: source / level
 * tokens, the line formatter with truncation, the (source, event) dedupe table, the rotation
 * decision, the tail finder, the RAM line buffer (oldest lines dropped) and the RTC_NOINIT replay
 * ring. Native-tested in test/test_event_journal. The flash / NVS / mutex side lives in src/EventJournal.cpp.
 *
 * Line format (design D2), ASCII, at most `JOURNAL_LINE_MAX` bytes including the '\n':
 *   <boot> <uptime_s.mmm> <SRC> <LVL> <event> k=v k=v ...
 * A longer line is cut and ends with '~' before the newline. Values never contain spaces.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifndef JOURNAL_LINE_MAX
#define JOURNAL_LINE_MAX 120
#endif
#ifndef JOURNAL_RTC_RING
#define JOURNAL_RTC_RING 16
#endif

/** @brief Event source (the <SRC> column) */
enum class JSrc : uint8_t { SYS, SRC, STEER, VESC, CAN, I2C, RELAY, SNAP, RTC, COUNT };

/** @brief Level (the <LVL> column) */
enum class JLvl : uint8_t { I, W, E };

inline const char* jSrcToken(JSrc s) {
    switch (s) {
        case JSrc::SYS:   return "SYS";
        case JSrc::SRC:   return "SRC";
        case JSrc::STEER: return "STEER";
        case JSrc::VESC:  return "VESC";
        case JSrc::CAN:   return "CAN";
        case JSrc::I2C:   return "I2C";
        case JSrc::RELAY: return "RELAY";
        case JSrc::SNAP:  return "SNAP";
        case JSrc::RTC:   return "RTC";
        default:          return "?";
    }
}

inline char jLvlChar(JLvl l) {
    return l == JLvl::E ? 'E' : (l == JLvl::W ? 'W' : 'I');
}

/**
 * @brief Format one journal line.
 * @param out     destination, at least `JOURNAL_LINE_MAX + 1` bytes (NUL-terminated)
 * @param boot    boot counter
 * @param uptimeMs ms since boot
 * @param event   event token (no spaces); empty / nullptr for SNAP
 * @param fields  "k=v k=v" (may be nullptr / empty)
 * @param sup     suppressed repeats to append as " sup=N" (0 = none)
 * @return line length in bytes including the trailing '\n' (≤ JOURNAL_LINE_MAX)
 */
inline size_t jFormatLine(char* out, uint32_t boot, uint32_t uptimeMs, JSrc src, JLvl lvl,
                          const char* event, const char* fields, uint32_t sup) {
    char tmp[256];
    int n = snprintf(tmp, sizeof(tmp), "%lu %lu.%03lu %s %c", (unsigned long)boot,
                     (unsigned long)(uptimeMs / 1000UL), (unsigned long)(uptimeMs % 1000UL),
                     jSrcToken(src), jLvlChar(lvl));
    if (n < 0) n = 0;
    size_t len = (size_t)n < sizeof(tmp) ? (size_t)n : sizeof(tmp) - 1;
    if (event && *event) {
        int k = snprintf(tmp + len, sizeof(tmp) - len, " %s", event);
        if (k > 0) len += (size_t)k < sizeof(tmp) - len ? (size_t)k : sizeof(tmp) - len - 1;
    }
    if (fields && *fields) {
        int k = snprintf(tmp + len, sizeof(tmp) - len, " %s", fields);
        if (k > 0) len += (size_t)k < sizeof(tmp) - len ? (size_t)k : sizeof(tmp) - len - 1;
    }
    if (sup > 0) {
        int k = snprintf(tmp + len, sizeof(tmp) - len, " sup=%lu", (unsigned long)sup);
        if (k > 0) len += (size_t)k < sizeof(tmp) - len ? (size_t)k : sizeof(tmp) - len - 1;
    }
    // Never let a stray newline split a record.
    for (size_t i = 0; i < len; i++) {
        if (tmp[i] == '\n' || tmp[i] == '\r') tmp[i] = '_';
    }
    const size_t maxBody = (size_t)JOURNAL_LINE_MAX - 1;   // room for '\n'
    if (len > maxBody) {
        memcpy(out, tmp, maxBody - 1);
        out[maxBody - 1] = '~';
        len = maxBody;
    } else {
        memcpy(out, tmp, len);
    }
    out[len++] = '\n';
    out[len] = '\0';
    return len;
}

/** @brief Lines never suppressed by the dedupe (design D7) */
inline bool jDedupeExempt(JSrc src, const char* event) {
    if (src == JSrc::SNAP || src == JSrc::RTC) return true;
    if (event == nullptr) return true;
    if (src == JSrc::SYS) {
        if (strcmp(event, "boot") == 0 || strcmp(event, "reboot") == 0 ||
            strcmp(event, "suppressed") == 0 || strncmp(event, "journal_", 8) == 0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief (source, event) dedupe table, LRU (design D7). Parameters are not part of the key.
 */
template <size_t N = 24>
class JDedupe {
public:
    static constexpr size_t EV_MAX = 24;

    struct Slot {
        bool used = false;
        JSrc src = JSrc::SYS;
        char ev[EV_MAX] = {0};
        uint32_t lastWriteMs = 0;   // last time a line of this key was written
        uint32_t lastUseMs = 0;     // LRU stamp (write or suppress)
        uint32_t sup = 0;           // suppressed since the last written line
    };

    explicit JDedupe(uint32_t windowMs = 2000) : windowMs_(windowMs) {}

    /**
     * @brief Decide whether a line may be written now.
     * @param supOut out: suppressed count to append (" sup=N") when the line is written
     * @return true = write the line, false = suppressed (counted)
     */
    bool admit(JSrc src, const char* event, uint32_t nowMs, uint32_t& supOut) {
        supOut = 0;
        if (jDedupeExempt(src, event)) return true;
        Slot* s = find(src, event);
        if (s != nullptr) {
            s->lastUseMs = nowMs;
            if ((uint32_t)(nowMs - s->lastWriteMs) < windowMs_) {
                s->sup++;
                return false;
            }
            supOut = s->sup;
            s->sup = 0;
            s->lastWriteMs = nowMs;
            return true;
        }
        s = evictSlot();
        s->used = true;
        s->src = src;
        strncpy(s->ev, event, EV_MAX - 1);
        s->ev[EV_MAX - 1] = '\0';
        s->lastWriteMs = nowMs;
        s->lastUseMs = nowMs;
        s->sup = 0;
        return true;
    }

    /**
     * @brief Pending suppressed counts older than `ageMs`: calls fn(src, event, n) and clears them.
     * @return number of entries reported
     */
    template <typename Fn>
    size_t drainPending(uint32_t nowMs, uint32_t ageMs, Fn fn) {
        size_t k = 0;
        for (size_t i = 0; i < N; i++) {
            Slot& s = slots_[i];
            if (s.used && s.sup > 0 && (uint32_t)(nowMs - s.lastWriteMs) >= ageMs) {
                fn(s.src, (const char*)s.ev, s.sup);
                s.sup = 0;
                k++;
            }
        }
        return k;
    }

    /** @brief Suppressed counts lost by LRU eviction since construction */
    uint32_t evictedSuppressed() const { return evictedSup_; }

    void clear() {
        for (size_t i = 0; i < N; i++) slots_[i] = Slot();
    }

private:
    Slot* find(JSrc src, const char* event) {
        for (size_t i = 0; i < N; i++) {
            if (slots_[i].used && slots_[i].src == src &&
                strncmp(slots_[i].ev, event, EV_MAX - 1) == 0) {
                return &slots_[i];
            }
        }
        return nullptr;
    }
    Slot* evictSlot() {
        Slot* best = nullptr;
        for (size_t i = 0; i < N; i++) {
            if (!slots_[i].used) return &slots_[i];
            if (best == nullptr || (int32_t)(slots_[i].lastUseMs - best->lastUseMs) < 0) best = &slots_[i];
        }
        evictedSup_ += best->sup;
        return best;
    }

    Slot slots_[N];
    uint32_t windowMs_;
    uint32_t evictedSup_ = 0;
};

/**
 * @brief Rotation decision (design D1): rotate before a flush that would push a non-empty active
 * file past `maxBytes`.
 */
inline bool jShouldRotate(size_t activeSize, size_t pendingBytes, size_t maxBytes) {
    return activeSize > 0 && activeSize + pendingBytes > maxBytes;
}

/**
 * @brief Offset at which the last `n` complete lines of `buf[0..len)` start (a trailing partial
 * line without '\n' counts as a line). n == 0 → len.
 */
inline size_t jTailStart(const char* buf, size_t len, size_t n) {
    if (n == 0 || len == 0) return len;
    size_t i = len;
    if (buf[i - 1] == '\n') i--;      // the final newline closes the last line
    size_t lines = 0;
    while (i > 0) {
        if (buf[i - 1] == '\n') {
            lines++;
            if (lines == n) return i;
        }
        i--;
    }
    return 0;
}

/** @brief What jLineBufAppend() had to drop from the front of the buffer */
struct JBufCut {
    size_t bytes;     // bytes removed from the front (whole lines)
    uint32_t lines;   // lines removed
};

/**
 * @brief Append one line to a line buffer of capacity `cap`, dropping the oldest WHOLE lines from
 * the front until it fits (the RAM buffer / RAM tail of the journal). `len` is updated.
 * A line longer than `cap` (or empty) is not appended and nothing is dropped.
 * @return what was dropped (bytes + lines), {0, 0} when the line fitted as is
 */
inline JBufCut jLineBufAppend(char* buf, size_t cap, size_t& len, const char* line, size_t n) {
    JBufCut cut = {0, 0};
    if (n == 0 || n > cap) return cut;
    if (len + n > cap) {
        const size_t need = len + n - cap;
        while (cut.bytes < len && cut.bytes < need) {
            const char* nl = (const char*)memchr(buf + cut.bytes, '\n', len - cut.bytes);
            cut.bytes = nl ? (size_t)(nl - buf) + 1 : len;
            cut.lines++;
        }
        memmove(buf, buf + cut.bytes, len - cut.bytes);
        len -= cut.bytes;
    }
    memcpy(buf + len, line, n);
    len += n;
    return cut;
}

/** @brief Count '\n'-terminated lines (plus a trailing partial line) */
inline size_t jCountLines(const char* buf, size_t len) {
    size_t k = 0;
    for (size_t i = 0; i < len; i++) if (buf[i] == '\n') k++;
    if (len > 0 && buf[len - 1] != '\n') k++;
    return k;
}

// ─── RTC_NOINIT replay ring (design D8) ───────────────────────────────────────

/** @brief CRC-32 (IEEE, reflected 0xEDB88320) */
inline uint32_t jCrc32(const void* data, size_t len, uint32_t crc = 0) {
    const uint8_t* p = (const uint8_t*)data;
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static constexpr uint32_t J_RING_MAGIC = 0x4A524E4Cu;   // "JRNL"
static constexpr uint32_t J_RING_VERSION = 1;

/**
 * @brief The ring kept in RTC slow memory. The header is CRC-protected; lines are validated one
 * by one on replay (printable ASCII ending in '\n'), so a push never recomputes a CRC over 2 KB.
 * `head` counts every line pushed (slot = head % JOURNAL_RTC_RING); `pending` counts the newest
 * lines not yet on flash (may exceed the ring size; the excess is reported as `lost`). A flush of
 * K lines lowers `pending` by K without moving the slots.
 */
struct JRtcRing {
    uint32_t magic;
    uint32_t version;
    uint32_t boot;          // boot number that wrote the lines
    uint32_t prevHeapMin;   // min free heap of that session (updated at every SNAP)
    uint32_t head;          // lines pushed in total (next slot = head % JOURNAL_RTC_RING)
    uint32_t pending;       // newest lines not yet on flash
    uint32_t crc;           // CRC-32 of the six fields above
    char lines[JOURNAL_RTC_RING][JOURNAL_LINE_MAX + 1];
};

inline uint32_t jRingHeaderCrc(const JRtcRing& r) {
    uint32_t h[6] = { r.magic, r.version, r.boot, r.prevHeapMin, r.head, r.pending };
    return jCrc32(h, sizeof(h));
}

inline void jRingSeal(JRtcRing& r) { r.crc = jRingHeaderCrc(r); }

inline bool jRingValid(const JRtcRing& r) {
    return r.magic == J_RING_MAGIC && r.version == J_RING_VERSION && r.crc == jRingHeaderCrc(r);
}

/** @brief Start a fresh, empty ring for boot `boot` */
inline void jRingReset(JRtcRing& r, uint32_t boot) {
    r.magic = J_RING_MAGIC;
    r.version = J_RING_VERSION;
    r.boot = boot;
    r.prevHeapMin = 0;
    r.head = 0;
    r.pending = 0;
    for (size_t i = 0; i < JOURNAL_RTC_RING; i++) r.lines[i][0] = '\0';
    jRingSeal(r);
}

/** @brief Append one formatted line (with its '\n') */
inline void jRingPush(JRtcRing& r, const char* line) {
    char* dst = r.lines[r.head % JOURNAL_RTC_RING];
    strncpy(dst, line, JOURNAL_LINE_MAX);
    dst[JOURNAL_LINE_MAX] = '\0';
    r.head++;
    r.pending++;
    jRingSeal(r);
}

/** @brief Everything pushed so far is on flash */
inline void jRingMarkFlushed(JRtcRing& r) {
    r.pending = 0;
    jRingSeal(r);
}

/** @brief The oldest `lines` pending lines reached flash (or were dropped) */
inline void jRingConsume(JRtcRing& r, uint32_t lines) {
    r.pending = lines >= r.pending ? 0 : r.pending - lines;
    jRingSeal(r);
}

inline void jRingSetHeapMin(JRtcRing& r, uint32_t heapMin) {
    r.prevHeapMin = heapMin;
    jRingSeal(r);
}

/** @brief A replayable line: printable ASCII, ends with '\n', fits */
inline bool jRingLineOk(const char* s) {
    size_t n = strnlen(s, JOURNAL_LINE_MAX + 1);
    if (n < 2 || n > JOURNAL_LINE_MAX || s[n - 1] != '\n') return false;
    for (size_t i = 0; i + 1 < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c < 0x20 || c > 0x7E) return false;
    }
    return true;
}

/**
 * @brief Replay the unflushed lines oldest first: fn(line) for each valid one.
 * @param lostOut lines that did not fit in the ring (+ invalid slots)
 * @return lines replayed
 */
template <typename Fn>
size_t jRingReplay(const JRtcRing& r, uint32_t& lostOut, Fn fn) {
    const uint32_t n = r.pending < JOURNAL_RTC_RING ? r.pending : (uint32_t)JOURNAL_RTC_RING;
    lostOut = r.pending - n;
    size_t done = 0;
    for (uint32_t k = 0; k < n; k++) {
        const char* line = r.lines[(r.head - n + k) % JOURNAL_RTC_RING];
        if (jRingLineOk(line)) {
            fn(line);
            done++;
        } else {
            lostOut++;
        }
    }
    return done;
}

#endif // EVENT_JOURNAL_LAW_H
