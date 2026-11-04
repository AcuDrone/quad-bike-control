#include "TachoSensor.h"
#include "Debug.h"
#include <Preferences.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

// ---- ISR-shared state -------------------------------------------------------
// Written by the ISR, read by update() ONLY inside the spinlock below (the ESP32-S3 is
// dual-core, so noInterrupts() alone would not order against the other core).
static constexpr uint8_t TACHO_RING = TACHO_AVG_PERIODS + 1;   // N periods need N + 1 edges
static volatile uint32_t s_edgesUs[TACHO_RING];
static volatile uint8_t  s_head = 0;        // next write slot
static volatile uint8_t  s_count = 0;       // valid edges in the ring (0..TACHO_RING)
static volatile uint32_t s_lastEdgeUs = 0;  // last ACCEPTED edge (glitch filter reference)
static volatile uint32_t s_edgeTotal = 0;   // accepted edges since boot
static portMUX_TYPE s_tachoMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR tachoIsr() {
    const uint32_t nowUs = (uint32_t)esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_tachoMux);
    // Software glitch filter: an edge closer than TACHO_MIN_PERIOD_US to the last accepted
    // one is ignition noise, not a tacho pulse (ppr 4 at 10 000 RPM is only 667 Hz).
    const uint32_t sinceUs = (uint32_t)(nowUs - s_lastEdgeUs);
    if (s_count == 0 || sinceUs >= (uint32_t)TACHO_MIN_PERIOD_US) {
        // First edge after a stop: start a fresh buffer even if update() has not cleared it
        // yet, so a period spanning the silence is never averaged in.
        if (s_count > 0 && sinceUs >= (uint32_t)TACHO_TIMEOUT_MS * 1000u) s_count = 0;
        s_edgesUs[s_head] = nowUs;
        s_head = (uint8_t)((s_head + 1) % TACHO_RING);
        if (s_count < TACHO_RING) s_count = (uint8_t)(s_count + 1);
        s_lastEdgeUs = nowUs;
        s_edgeTotal = s_edgeTotal + 1;
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
      lastEdgeMs_(0) {
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
    initialized_ = true;

    Debug::printfFeature(DebugFeature::VEHICLE,
        "[TACHO] GPIO%d rising-edge ISR, ppr %.1f (%s), %s\n",
        (int)PIN_TACHO, ppr_, pprStored_ ? "stored" : "default",
        fitted_ ? "fitted (ppr stored)" : "not fitted until the first pulse");
}

void TachoSensor::update() {
    if (!initialized_) {
        return;
    }

    // Short critical section: copy the ring and the counters, nothing else.
    uint32_t edges[TACHO_RING];
    uint8_t head, count;
    uint32_t total, lastUs;
    portENTER_CRITICAL(&s_tachoMux);
    for (uint8_t i = 0; i < TACHO_RING; i++) edges[i] = s_edgesUs[i];
    head = s_head;
    count = s_count;
    total = s_edgeTotal;
    lastUs = s_lastEdgeUs;
    portEXIT_CRITICAL(&s_tachoMux);

    const uint32_t nowMs = millis();
    if (total != edgeTotal_) {
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
        if (!cleared_) {
            // Clear the buffer so the first period after a restart is not averaged with
            // periods from before the stop. An edge racing this clear simply starts fresh.
            portENTER_CRITICAL(&s_tachoMux);
            if (s_edgeTotal == edgeTotal_) s_count = 0;
            portEXIT_CRITICAL(&s_tachoMux);
            cleared_ = true;
        }
        return;
    }
    live_ = true;

    if (count < 2) {
        return;   // one edge after a stop: no period yet, keep 0 until the second one
    }
    // Chronological copy (oldest first) for the pure averaging helper.
    uint32_t ordered[TACHO_RING];
    const uint8_t oldest = (uint8_t)((head + TACHO_RING - count) % TACHO_RING);
    for (uint8_t i = 0; i < count; i++) ordered[i] = edges[(oldest + i) % TACHO_RING];

    const float meanUs = tachoMeanPeriod(ordered, count, TACHO_AVG_PERIODS);
    const float hz = tachoHzFromPeriodUs(meanUs);
    const float rpm = tachoRpm(hz, ppr_);
    if (!tachoPlausible(rpm, (float)TACHO_MAX_RPM)) {
        return;   // implausible: keep the previous reading
    }
    hz_ = hz;
    rpm_ = (uint16_t)lroundf(rpm);
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
