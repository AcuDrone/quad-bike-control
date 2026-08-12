#include "SpeedSensor.h"
#include "Debug.h"
#include <Preferences.h>

static const char* NVS_NAMESPACE = "speed";

SpeedSensor::SpeedSensor()
    : unit_(nullptr),
      channel_(nullptr),
      initialized_(false),
      pulsesPerRev_(SPEED_DEFAULT_PULSES_PER_REV),
      wheelCircumferenceMm_(SPEED_DEFAULT_WHEEL_CIRCUMFERENCE_MM),
      distancePerPulseMm_(0.0f),
      limiterEnabled_(SPEED_LIMIT_ENABLE_DEFAULT),
      limitMaxKmh_(SPEED_LIMIT_MAX_KMH_DEFAULT),
      lastSampleMs_(0),
      lastPulseMs_(0),
      lastTotal_(0),
      pulseTotal_(0),
      speedKmh_(0.0f),
      lastMovingSpeedKmh_(0.0f),
      everPulsed_(false),
      suspicious_(false),
      staleHandled_(true) {
    recomputeDistancePerPulse();
}

void SpeedSensor::recomputeDistancePerPulse() {
    distancePerPulseMm_ = (pulsesPerRev_ > 0)
        ? (wheelCircumferenceMm_ / (float)pulsesPerRev_)
        : 0.0f;
}

bool SpeedSensor::begin() {
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, true)) {
        pulsesPerRev_         = (uint16_t)prefs.getInt("ppr", SPEED_DEFAULT_PULSES_PER_REV);
        wheelCircumferenceMm_ = prefs.getFloat("circ_mm", SPEED_DEFAULT_WHEEL_CIRCUMFERENCE_MM);
        limiterEnabled_       = prefs.getBool("lim_on", SPEED_LIMIT_ENABLE_DEFAULT);
        limitMaxKmh_          = prefs.getFloat("lim_kmh", SPEED_LIMIT_MAX_KMH_DEFAULT);
        prefs.end();
    }
    recomputeDistancePerPulse();

    pcnt_unit_config_t unitConfig = {};
    unitConfig.low_limit  = SPEED_PCNT_LOW_LIMIT;
    unitConfig.high_limit = SPEED_PCNT_HIGH_LIMIT;
    // Let the peripheral accumulate its own overflows: the count returned by
    // pcnt_unit_get_count() then keeps rising past the high limit, so a fast pulse
    // train between two samples cannot lose counts and no user ISR is needed.
    unitConfig.flags.accum_count = 1;

    esp_err_t err = pcnt_new_unit(&unitConfig, &unit_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] PCNT unit alloc failed (%d)\n", (int)err);
        unit_ = nullptr;
        return false;
    }

    // accum_count only fires at watch points, so the limits must be watched.
    pcnt_unit_add_watch_point(unit_, SPEED_PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(unit_, SPEED_PCNT_LOW_LIMIT);

    pcnt_glitch_filter_config_t filterConfig = {};
    filterConfig.max_glitch_ns = SPEED_GLITCH_FILTER_NS;
    pcnt_unit_set_glitch_filter(unit_, &filterConfig);

    pcnt_chan_config_t chanConfig = {};
    chanConfig.edge_gpio_num  = PIN_SPEED_SENSOR;
    chanConfig.level_gpio_num = -1;   // no direction/level input: the sensor is unidirectional
    err = pcnt_new_channel(unit_, &chanConfig, &channel_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] PCNT channel alloc failed (%d)\n", (int)err);
        pcnt_del_unit(unit_);
        unit_ = nullptr;
        channel_ = nullptr;
        return false;
    }

    // ONE edge only. The 6N137 inverts the sensor signal (LED conducting -> GPIO8 LOW),
    // so an active sensor pulse arrives here as a falling edge — but the pulse RATE is
    // identical on either edge, so this is a documentation matter, not a polarity fix.
    pcnt_channel_set_edge_action(channel_,
                                 PCNT_CHANNEL_EDGE_ACTION_HOLD,      // rising  (opto releasing)
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE); // falling (opto conducting)
    pcnt_channel_set_level_action(channel_,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    pcnt_unit_enable(unit_);
    pcnt_unit_clear_count(unit_);
    pcnt_unit_start(unit_);

    initialized_  = true;
    lastSampleMs_ = millis();
    lastPulseMs_  = lastSampleMs_;

    Debug::printfFeature(DebugFeature::VEHICLE,
        "[SPEED] PCNT on GPIO%d (opto-inverted, falling edge), filter %d ns, %u pulses/rev, %.0f mm circumference\n",
        (int)PIN_SPEED_SENSOR, (int)SPEED_GLITCH_FILTER_NS,
        (unsigned)pulsesPerRev_, wheelCircumferenceMm_);
    return true;
}

void SpeedSensor::update() {
    if (!initialized_) {
        return;
    }

    uint32_t now = millis();
    uint32_t dtMs = now - lastSampleMs_;
    if (dtMs < SPEED_SAMPLE_INTERVAL_MS) {
        return;
    }
    lastSampleMs_ = now;

    int rawCount = 0;
    if (pcnt_unit_get_count(unit_, &rawCount) != ESP_OK) {
        return;   // transient read failure: keep the previous reading
    }

    // accum_count makes the counter monotonic for a unidirectional signal; the
    // unsigned subtraction stays correct across an int32 wrap (~400 days at the
    // highest expected pulse rate).
    uint32_t total = (uint32_t)rawCount;
    uint32_t delta = total - lastTotal_;
    lastTotal_  = total;
    pulseTotal_ = total;

    if (delta > 0) {
        // Measure over the interval since the last window that SAW edges, not since the
        // last sample: below ~5 km/h one pulse period spans several sample windows, and
        // dividing by the sample interval would multiply the reading (a 1 km/h crawl would
        // read ~8 km/h and trip the interlock). Clamped so the first pulse after a long
        // standstill cannot be divided by an arbitrarily large gap.
        uint32_t windowMs = now - lastPulseMs_;
        if (windowMs > SPEED_STALE_TIMEOUT_MS) windowMs = SPEED_STALE_TIMEOUT_MS;
        if (windowMs == 0) windowMs = dtMs;
        // mm/ms is metres per second by definition; ×3.6 gives km/h.
        speedKmh_ = ((delta * distancePerPulseMm_) / (float)windowMs) * 3.6f;
        lastPulseMs_        = now;
        lastMovingSpeedKmh_ = speedKmh_;
        staleHandled_       = false;
        suspicious_         = false;   // a live pulse train clears a latched fault
        if (!everPulsed_) {
            everPulsed_ = true;
            Debug::printlnFeature(DebugFeature::VEHICLE, "[SPEED] First pulses counted — reading is now valid");
        }
        return;
    }

    // No edges in this window. Hold the last reading until the stale timeout: at low
    // speed one pulse period can span several sample windows.
    uint32_t silenceMs = now - lastPulseMs_;
    if (silenceMs < SPEED_STALE_TIMEOUT_MS || staleHandled_) {
        return;
    }
    staleHandled_ = true;
    speedKmh_     = 0.0f;

    // Could the vehicle physically have stopped inside this silence? If bleeding off
    // the last observed speed needs longer than the silence, the pulses vanished faster
    // than any real deceleration — treat the 0 as unknown, not as "stopped".
    if (lastMovingSpeedKmh_ > (float)TRANS_SPEED_INTERLOCK_THRESHOLD) {
        uint32_t minStopMs = (uint32_t)((lastMovingSpeedKmh_ / SPEED_MAX_PLAUSIBLE_DECEL_KMH_S) * 1000.0f);
        if (minStopMs > silenceMs) {
            suspicious_ = true;
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[SPEED] WARNING: pulses stopped implausibly fast from %.1f km/h (%lu ms) — reading marked invalid\n",
                lastMovingSpeedKmh_, (unsigned long)silenceMs);
        }
    }
    lastMovingSpeedKmh_ = 0.0f;
}

bool SpeedSensor::setPulsesPerRev(int32_t ppr) {
    if (ppr < SPEED_PPR_MIN || ppr > SPEED_PPR_MAX) {
        return false;
    }
    pulsesPerRev_ = (uint16_t)ppr;
    recomputeDistancePerPulse();

    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putInt("ppr", (int32_t)pulsesPerRev_);
        prefs.end();
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] Pulses/rev set to %u (%.1f mm/pulse)\n",
                         (unsigned)pulsesPerRev_, distancePerPulseMm_);
    return true;
}

bool SpeedSensor::setWheelCircumferenceMm(float mm) {
    if (!(mm >= SPEED_CIRC_MIN_MM) || mm > SPEED_CIRC_MAX_MM) {
        return false;   // the negated form also rejects NaN
    }
    wheelCircumferenceMm_ = mm;
    recomputeDistancePerPulse();

    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putFloat("circ_mm", wheelCircumferenceMm_);
        prefs.end();
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] Wheel circumference set to %.0f mm (%.1f mm/pulse)\n",
                         wheelCircumferenceMm_, distancePerPulseMm_);
    return true;
}

void SpeedSensor::setLimiterEnabled(bool enabled) {
    limiterEnabled_ = enabled;
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putBool("lim_on", enabled);
        prefs.end();
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] Max-speed limiter %s (%.0f km/h)\n",
                         enabled ? "ENABLED" : "DISABLED", limitMaxKmh_);
}

bool SpeedSensor::setLimitMaxKmh(float kmh) {
    if (!(kmh >= SPEED_LIMIT_MIN_KMH) || kmh > SPEED_LIMIT_MAX_KMH) {
        return false;
    }
    limitMaxKmh_ = kmh;
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putFloat("lim_kmh", limitMaxKmh_);
        prefs.end();
    }
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] Limiter maximum set to %.1f km/h\n", limitMaxKmh_);
    return true;
}
