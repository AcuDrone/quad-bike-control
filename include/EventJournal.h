#ifndef EVENT_JOURNAL_H
#define EVENT_JOURNAL_H

#include <Arduino.h>
#include <memory>
#include "Constants.h"
#include "EventJournalLaw.h"

/**
 * @brief Persistent event journal (change add-event-journal-and-link-recovery).
 *
 * Two plain-text files on LittleFS: /events.log (active) and /events.1 (rotated), each at most
 * JOURNAL_FILE_MAX_BYTES. Lines are `<boot> <uptime_s.mmm> <SRC> <LVL> <event> k=v ...`
 * (EventJournalLaw.h). Static API, like Debug.
 *
 * - log() never touches flash: dedupe → format → append to a RAM buffer
 *   (JOURNAL_RAM_BUF_BYTES) and to an RTC_NOINIT ring that is replayed after a reset. Callable
 *   from any task (a spinlock guards the buffer).
 * - service() (last step of loop()): heap watch, SNAP every JOURNAL_SNAP_MS, pending clear, and
 *   the flash flush — at most once per JOURNAL_FLUSH_MS and only with data (earlier at ≥ 75 %),
 *   DEFERRED while the steering motor is driven, at most JOURNAL_FLUSH_DEFER_MAX_MS. The loop
 *   only try-takes the file mutex: a busy mutex postpones the flush, never blocks the loop.
 * - Write errors degrade the journal only: after 3 consecutive failed flushes (or a failed
 *   mount) file output is off for the session (journalBytes() = -1); RAM + RTC keep working.
 * - Flash-write switch (NVS `journal`/`fs`, default OFF, design D13): while off the journal never
 *   writes LittleFS (no flush, no rotation, no file access at boot); the RAM buffer is a RAM tail
 *   of the newest JOURNAL_RAM_BUF_BYTES (oldest lines dropped), the RTC ring, dedupe and SNAP keep
 *   working, and tail / download serve any existing files read-only followed by the RAM tail.
 *   Switching is journaled (`SYS I journal_fs on=1|0`); on → flush the RAM tail at once, off → one
 *   final flush, then no more file writes.
 */
class EventJournal {
public:
    /** @brief Boot: NVS boot counter, RTC replay, `#` header, `SYS I boot` (RAM only, no FS yet) */
    static void begin();

    /** @brief LittleFS mount result (true = mounted): enables / disables file output */
    static void attachFs(bool mounted);

    /** @brief Journal one event. fmt (printf-style) builds the "k=v k=v" fields; may be nullptr. */
    static void log(JSrc src, JLvl lvl, const char* event, const char* fmt = nullptr, ...)
        __attribute__((format(printf, 4, 5)));

    /**
     * @brief Main-loop service (call as the last step of loop()).
     * @param steeringDriving true while the steering motor is driven: the flush is deferred, at
     *        most JOURNAL_FLUSH_DEFER_MAX_MS from the moment it became due
     */
    static void service(bool steeringDriving);

    /**
     * @brief Flash-write switch (HTTP). Takes effect on the next service(): persisted in NVS
     * (`journal`/`fs`), journaled, flushed as described above.
     */
    static void setFlashWrites(bool on);

    /** @brief The flash-write setting (persisted value; telemetry `journal_fs`) */
    static bool flashWritesEnabled();

    /**
     * @brief Controlled reboot: log `SYS I reboot why=<why>` and flush now (never deferred).
     * No-op beyond the RAM / RTC line while flash writes are off.
     */
    static void flushNow(const char* why);

    /** @brief Filesystem OTA started (LittleFS about to be unmounted): no more file I/O */
    static void suspendFs();

    /** @brief Clear request (HTTP): executed by the loop (both files + RAM, `journal_cleared`) */
    static void requestClear();

    /** @brief Loop timing: one iteration took totalUs, slowest phase `phase` took phaseUs */
    static void noteLoop(uint32_t totalUs, const char* phase, uint32_t phaseUs);

    /**
     * @brief SNAP field provider (vehicle side, set by main). The snapshot is written as two lines
     * to respect JOURNAL_LINE_MAX: line A = "heap min loop_max jflush_max" + `a` ("vin fet"),
     * line B = `b` ("lws_fps vesc_good drop resync can_reinit"; master adds i2c_unlock,
     * the DevKitC-1 has no I2C bus).
     */
    typedef void (*SnapFn)(char* a, size_t capA, char* b, size_t capB);
    static void setSnapProvider(SnapFn fn);

    /**
     * @brief Flash writes on: both files + unflushed RAM bytes, -1 when file output is unavailable.
     * Flash writes off: the RAM tail bytes.
     */
    static int32_t journalBytes();

    static uint32_t bootNumber();

    /**
     * @brief A read stream over the journal (`.1`, active file, RAM lines), for the HTTP handlers.
     * Rotation and clear are postponed while any stream is open. With flash writes off the file
     * part (if any) and the RAM tail are each preceded by a `# --- ... ---` label line.
     */
    class Stream {
    public:
        static constexpr size_t TRY_AGAIN = (size_t)-1;   // == RESPONSE_TRY_AGAIN on the ESP32
        ~Stream();
        /** @brief Fill up to maxLen bytes; 0 = end of stream, TRY_AGAIN = journal busy, call again */
        size_t read(uint8_t* buf, size_t maxLen);
        /** @brief Lines held in RAM when the stream was opened (all of them, not just the tail) */
        size_t ramLines() const { return ramLines_; }
    private:
        friend class EventJournal;
        Stream() = default;
        struct Seg { uint8_t file; uint32_t pos; uint32_t end; };   // file: 0 = .1, 1 = active
        Seg seg_[2] = {{0, 0, 0}, {1, 0, 0}};
        uint8_t segIdx_ = 0;
        char* ram_ = nullptr;
        size_t ramLen_ = 0;
        size_t ramPos_ = 0;
        size_t ramLines_ = 0;
        const char* lblFiles_ = nullptr;   // label before the file part (writes off)
        size_t lblFilesPos_ = 0;
        const char* lblRam_ = nullptr;     // label before the RAM lines (writes off)
        size_t lblRamPos_ = 0;
        bool counted_ = false;
    };

    /**
     * @brief Open a stream. tailLines = 0 → everything (download); else the last tailLines lines
     * (the unflushed RAM lines included).
     */
    static std::shared_ptr<Stream> openStream(size_t tailLines);
};

#endif // EVENT_JOURNAL_H
