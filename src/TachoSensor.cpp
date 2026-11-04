#include "TachoSensor.h"
#include "Debug.h"
#include <Preferences.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

// ---- ISR-shared state -------------------------------------------------------
// Written by the ISR, read by update() ONLY inside the spinlock below (the ESP32-S3 is
// dual-core, so noInterrupts() alone would not order against the other core).
static constexpr uint8_t TACHO_RING = TACHO_MEDIAN_PERIODS + 1;   // N periods need N + 1 edges
static volatile uint32_t s_edgesUs[TACHO_RING];
static volatile uint8_t  s_head = 0;        // next write slot
static volatile uint8_t  s_count = 0;       // valid edges in the ring (0..TACHO_RING)
static volatile uint32_t s_lastEdgeUs = 0;  // last ACCEPTED edge (filter reference)
static volatile uint32_t s_edgeTotal = 0;   // accepted edges since boot
static volatile uint32_t s_rejected = 0;    // edges rejected by the floor / lock window since boot
// Lock threshold (µs) = TACHO_LOCK_FRACTION × the median period, written by update(); 0 = no
// lock (floor only). Stored pre-multiplied so the ISR does integer compares only (no FPU in ISR).
static volatile uint32_t s_lockMinUs = 0;
static portMUX_TYPE s_tachoMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR tachoIsr() {
    const uint32_t nowUs = (uint32_t)esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_tachoMux);
    const uint32_t sinceUs = (uint32_t)(nowUs - s_lastEdgeUs);
    bool accept;
    if (s_count == 0) {
        accept = true;                       // first edge after boot / a stop
    } else if (sinceUs >= (uint32_t)TACHO_TIMEOUT_MS * 1000u) {
        // First edge after a silence: start a fresh buffer and drop the lock even if update()
        // has not cleared them yet, so a period spanning the silence is never used.
        s_count = 0;
        s_lockMinUs = 0;
        accept = true;
    } else {
        // Absolute floor (ignition noise: ppr 4 at 10 000 RPM is only 667 Hz) and, while
        // locked, the lock window: an edge closer than half the median period is not a pulse.
        accept = tachoEdgeAcceptedMin(sinceUs, s_lockMinUs, (uint32_t)TACHO_MIN_PERIOD_US);
    }
    if (accept) {
        s_edgesUs[s_head] = nowUs;
        s_head = (uint8_t)((s_head + 1) % TACHO_RING);
        if (s_count < TACHO_RING) s_count = (uint8_t)(s_count + 1);
        s_lastEdgeUs = nowUs;
        s_edgeTotal = s_edgeTotal + 1;
    } else {
        s_rejected = s_rejected + 1;
    }
    portEXIT_CRITICAL_ISR(&s_tachoMux);
}

TachoSensor::TachoSensor()
    : initialized_(false),
      ppr_(TACHO_PPR_DEFAULT),
      pprStored_(false),
      fitted_(false),
      live_(false),
      cleared_(true),
      hz_(0.0f),
      rpm_(0),
      edgeTotal_(0),
      lastEdgeMs_(0),
      medianUs_(0),
      rejTotal_(0),
      diagUs_{},
      diagN_(0),
      winStartMs_(0),
      winAccStart_(0),
      winRejStart_(0),
      winMinUs_(0),
      winMaxUs_(0),
      winMinMs_(0.0f),
      winMaxMs_(0.0f),
      accPerSec_(0),
      rejPerSec_(0) {
}

void TachoSensor::begin() {
    // ppr from NVS. An absent key, NaN or a value outside {0.5,1,2,3,4} is ignored (default
    // used, NVS not rewritten). A PRESENT key marks the tacho fitted from boot, so a stopped
    // engine reads 0 from the tacho rather than falling back to CAN.
    Preferences prefs;
    if (prefs.begin(TACHO_NVS_NS, true)) {
        if (prefs.isKey(TACHO_NVS_KEY)) {
            pprStored_ = true;
            const float stored = prefs.getFloat(TACHO_NVS_KEY, TACHO_PPR_DEFAULT);
            if (tachoPprAllowed(stored)) {
                ppr_ = tachoSnapPpr(stored);
            } else {
                Debug::printfFeature(DebugFeature::VEHICLE,
                    "[TACHO] Stored ppr %.3f not allowed — using %.1f\n", stored, TACHO_PPR_DEFAULT);
            }
        }
        prefs.end();
    }
    fitted_ = pprStored_;

    pinMode(PIN_TACHO, INPUT);   // plain input: the board divider is the emitter load
    attachInterrupt(digitalPinToInterrupt(PIN_TACHO), tachoIsr, RISING);
    winStartMs_ = millis();
    initialized_ = true;

    Debug::printfFeature(DebugFeature::VEHICLE,
        "[TACHO] GPIO%d rising-edge ISR (floor %d us, lock %.2f x median of %d), ppr %.1f (%s), %s\n",
        (int)PIN_TACHO, (int)TACHO_MIN_PERIOD_US, (double)TACHO_LOCK_FRACTION, (int)TACHO_MEDIAN_PERIODS, ppr_, pprStored_ ? "stored" : "default",
        fitted_ ? "fitted (ppr stored)" : "not fitted until the first pulse");
}

void TachoSensor::pushDiagPeriod(uint32_t us) {
    if (diagN_ < TACHO_DIAG_PERIODS) {
        diagUs_[diagN_++] = us;
    } else {
        for (uint8_t i = 1; i < TACHO_DIAG_PERIODS; i++) diagUs_[i - 1] = diagUs_[i];
        diagUs_[TACHO_DIAG_PERIODS - 1] = us;
    }
    if (winMinUs_ == 0 || us < winMinUs_) winMinUs_ = us;
    if (us > winMaxUs_) winMaxUs_ = us;
}

void TachoSensor::closeDiagWindow(uint32_t nowMs) {
    const uint32_t elapsedMs = nowMs - winStartMs_;
    if (elapsedMs < (uint32_t)TACHO_DIAG_WINDOW_MS) {
        return;
    }
    // Normalised to edges per second (the loop may close the window a little late).
    const float scale = 1000.0f / (float)elapsedMs;
    const float acc = (float)(edgeTotal_ - winAccStart_) * scale;
    const float rej = (float)(rejTotal_ - winRejStart_) * scale;
    accPerSec_ = (uint16_t)lroundf(acc > 65535.0f ? 65535.0f : acc);
    rejPerSec_ = (uint16_t)lroundf(rej > 65535.0f ? 65535.0f : rej);
    winMinMs_ = (float)winMinUs_ / 1000.0f;
    winMaxMs_ = (float)winMaxUs_ / 1000.0f;

    if (live_) {
        // Rate-limited field log: once per window, only while pulses arrive.
        char periods[8 * 7 + 1];
        formatDiagPeriods(periods, sizeof(periods));
        Debug::printfFeature(DebugFeature::VEHICLE,
            "[TACHO] hz=%.2f rpm=%u med=%luus acc=%u rej=%u periods=[%s]\n",
            hz_, (unsigned)rpm_, (unsigned long)medianUs_, (unsigned)accPerSec_,
            (unsigned)rejPerSec_, periods);
    }

    winStartMs_ = nowMs;
    winAccStart_ = edgeTotal_;
    winRejStart_ = rejTotal_;
    winMinUs_ = 0;
    winMaxUs_ = 0;
}

void TachoSensor::update() {
    if (!initialized_) {
        return;
    }

    // Short critical section: copy the ring and the counters, nothing else.
    uint32_t edges[TACHO_RING];
    uint8_t head, count;
    uint32_t total, lastUs, rejected;
    portENTER_CRITICAL(&s_tachoMux);
    for (uint8_t i = 0; i < TACHO_RING; i++) edges[i] = s_edgesUs[i];
    head = s_head;
    count = s_count;
    total = s_edgeTotal;
    lastUs = s_lastEdgeUs;
    rejected = s_rejected;
    portEXIT_CRITICAL(&s_tachoMux);

    // Chronological copy (oldest first) for the pure helpers.
    uint32_t ordered[TACHO_RING];
    const uint8_t oldest = (uint8_t)((head + TACHO_RING - count) % TACHO_RING);
    for (uint8_t i = 0; i < count; i++) ordered[i] = edges[(oldest + i) % TACHO_RING];

    const uint32_t nowMs = millis();
    rejTotal_ = rejected;
    if (total != edgeTotal_) {
        // Diagnostics: the raw periods ending at the new edge(s), oldest first.
        const uint32_t newEdges = total - edgeTotal_;
        uint8_t newPeriods = count > 1 ? (uint8_t)(count - 1) : 0;
        if (newEdges < newPeriods) newPeriods = (uint8_t)newEdges;
        for (uint8_t i = (uint8_t)(count - newPeriods); i < count; i++) {
            pushDiagPeriod(ordered[i] - ordered[i - 1]);
        }
        // New edge(s): place the last one on the millis() axis for the timeout / sample stamp.
        const uint32_t ageUs = (uint32_t)esp_timer_get_time() - lastUs;
        lastEdgeMs_ = nowMs - ageUs / 1000u;
        if (!fitted_) {
            Debug::printlnFeature(DebugFeature::VEHICLE, "[TACHO] First pulse — tacho fitted");
        }
        fitted_ = true;
        edgeTotal_ = total;
        cleared_ = false;
    }

    if (edgeTotal_ == 0 || tachoTimedOut(nowMs, lastEdgeMs_, TACHO_TIMEOUT_MS)) {
        live_ = false;
        hz_ = 0.0f;
        rpm_ = 0;
        medianUs_ = 0;
        if (!cleared_) {
            // Clear the buffer and the lock so the first period after a restart is not mixed
            // with periods from before the stop. An edge racing this clear simply starts fresh.
            portENTER_CRITICAL(&s_tachoMux);
            if (s_edgeTotal == edgeTotal_) s_count = 0;
            s_lockMinUs = 0;
            portEXIT_CRITICAL(&s_tachoMux);
            cleared_ = true;
        }
        closeDiagWindow(nowMs);
        return;
    }
    live_ = true;

    if (count < 2) {
        closeDiagWindow(nowMs);
        return;   // one edge after a stop: no period yet, keep 0 until the second one
    }

    const float periodUs = tachoMedianPeriod(ordered, count, TACHO_MEDIAN_PERIODS);
    medianUs_ = (uint32_t)lroundf(periodUs);
    // Lock window only once the median rests on >= 3 periods; until then the floor alone.
    const uint32_t lockMinUs = (count >= 4) ? tachoLockMinUs(medianUs_, TACHO_LOCK_FRACTION) : 0u;
    portENTER_CRITICAL(&s_tachoMux);
    if (s_edgeTotal == total) s_lockMinUs = lockMinUs;   // a newer edge (maybe a restart): next pass
    portEXIT_CRITICAL(&s_tachoMux);

    const float hz = tachoHzFromPeriodUs(periodUs);
    const float rpm = tachoRpm(hz, ppr_);
    if (tachoPlausible(rpm, (float)TACHO_MAX_RPM)) {
        hz_ = hz;
        rpm_ = (uint16_t)lroundf(rpm);
    }   // implausible: keep the previous reading
    closeDiagWindow(nowMs);
}

bool TachoSensor::setPpr(float ppr) {
    if (!tachoPprAllowed(ppr)) {
        return false;
    }
    const float snapped = tachoSnapPpr(ppr);
    // Persist first: if the write fails the previous ppr stays in effect.
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(TACHO_NVS_NS, false)) {
        ok = prefs.putFloat(TACHO_NVS_KEY, snapped) == sizeof(float);
        prefs.end();
    }
    if (!ok) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[TACHO] NVS write failed — ppr unchanged");
        return false;
    }
    ppr_ = snapped;
    pprStored_ = true;
    fitted_ = true;   // RPM follows on the next update()
    Debug::printfFeature(DebugFeature::VEHICLE, "[TACHO] ppr set to %.1f\n", ppr_);
    return true;
}

bool TachoSensor::resetPpr() {
    Preferences prefs;
    if (!prefs.begin(TACHO_NVS_NS, false)) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[TACHO] NVS open failed — ppr not reset");
        return false;
    }
    bool ok = !prefs.isKey(TACHO_NVS_KEY) || prefs.remove(TACHO_NVS_KEY);
    prefs.end();
    if (!ok) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[TACHO] NVS remove failed — ppr not reset");
        return false;
    }
    ppr_ = TACHO_PPR_DEFAULT;
    pprStored_ = false;   // fitted_ stays latched for this boot; RPM follows on the next update()
    Debug::printfFeature(DebugFeature::VEHICLE, "[TACHO] ppr reset to %.1f (key removed)\n", ppr_);
    return true;
}
