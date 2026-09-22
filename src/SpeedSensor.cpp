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
      lastSampleMs_(0),
      lastPulseMs_(0),
      lastTotal_(0),
      pulseTotal_(0),
      speedMs_(0.0f),
      lastMovingSpeedMs_(0.0f),
      odoMm_(0),
      tripMm_(0),
      lastOdoWriteMm_(0),
      distanceDirty_(false),
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
        int32_t ppr  = prefs.getInt("ppr", SPEED_DEFAULT_PULSES_PER_REV);
        float   circ = prefs.getFloat("circ_mm", SPEED_DEFAULT_WHEEL_CIRCUMFERENCE_MM);
        // A key that has never been written reads as the 0 default — the correct starting
        // odometer for a virgin NVS. There is no seeding path.
        odoMm_  = prefs.getULong64("odo_mm", 0);
        tripMm_ = prefs.getULong64("trip_mm", 0);
        prefs.end();
        if (ppr >= SPEED_PPR_MIN && ppr <= SPEED_PPR_MAX) pulsesPerRev_ = (uint16_t)ppr;
        if (circ >= SPEED_CIRC_MIN_MM && circ <= SPEED_CIRC_MAX_MM) wheelCircumferenceMm_ = circ;
        Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] ODO %.3f km, TRIP %.3f km restored\n",
                             getOdoKm(), getTripKm());
    } else {
        // Virgin NVS (the namespace does not exist yet) or a read failure: nothing was restored,
        // so say so rather than reporting a "restored" 0.000 that was never stored.
        Debug::printlnFeature(DebugFeature::VEHICLE,
            "[SPEED] NVS read failed or namespace absent — using defaults (ODO 0.000 km, TRIP 0.000 km)");
    }
    lastOdoWriteMm_ = odoMm_;
    distanceDirty_  = false;   // RAM matches NVS (or both are the 0 default): nothing to write
    // One-off cleanup of the retired local-limiter keys — the ceiling now comes only from the
    // autopilot's SPEED_MAX. remove() on an absent key is a no-op, so after the first boot on
    // new firmware this costs nothing.
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.remove("lim_on");
        prefs.remove("lim_kmh");
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

    pcnt_glitch_filter_config_t filterConfig = {};
    filterConfig.max_glitch_ns = SPEED_GLITCH_FILTER_NS;

    pcnt_chan_config_t chanConfig = {};
    chanConfig.edge_gpio_num  = PIN_SPEED_SENSOR;
    chanConfig.level_gpio_num = -1;   // no direction/level input: the sensor is unidirectional

    esp_err_t err = pcnt_new_unit(&unitConfig, &unit_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] PCNT unit alloc failed (%d)\n", (int)err);
        unit_ = nullptr;
        return false;
    }

    // accum_count only fires at watch points, so the limits must be watched.
    err = pcnt_unit_add_watch_point(unit_, SPEED_PCNT_HIGH_LIMIT);
    if (err == ESP_OK) err = pcnt_unit_add_watch_point(unit_, SPEED_PCNT_LOW_LIMIT);
    if (err == ESP_OK) err = pcnt_unit_set_glitch_filter(unit_, &filterConfig);
    if (err == ESP_OK) err = pcnt_new_channel(unit_, &chanConfig, &channel_);
    // ONE edge only. The 6N137 inverts the sensor signal (LED conducting -> GPIO8 LOW),
    // so an active sensor pulse arrives here as a falling edge — but the pulse RATE is
    // identical on either edge, so this is a documentation matter, not a polarity fix.
    if (err == ESP_OK) err = pcnt_channel_set_edge_action(channel_,
                                 PCNT_CHANNEL_EDGE_ACTION_HOLD,      // rising  (opto releasing)
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE); // falling (opto conducting)
    if (err == ESP_OK) err = pcnt_channel_set_level_action(channel_,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (err == ESP_OK) err = pcnt_unit_enable(unit_);
    if (err == ESP_OK) err = pcnt_unit_clear_count(unit_);
    if (err == ESP_OK) err = pcnt_unit_start(unit_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] PCNT setup failed (%s)\n", esp_err_to_name(err));
        if (channel_) { pcnt_del_channel(channel_); channel_ = nullptr; }
        pcnt_unit_disable(unit_);
        pcnt_del_unit(unit_);
        unit_ = nullptr;
        return false;
    }

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

    if (delta > SPEED_MAX_PULSES_PER_SAMPLE) {
        return;
    }

    if (delta > 0) {
        // Distance is added HERE and nowhere else: after the wrap guard's early return, inside
        // the branch that knows the wheel actually turned. The decay/suspicious path below adds
        // nothing — integrating a decayed estimate would invent distance the wheel never turned.
        // llroundf, not a truncating cast: truncating ~28 mm/pulse every window would bias the
        // odometer low systematically, where rounding is unbiased.
        const uint64_t stepMm = (uint64_t)llroundf(delta * distancePerPulseMm_);
        odoMm_  += stepMm;
        tripMm_ += stepMm;
        distanceDirty_ = true;
        // Unsigned and monotonic — odoMm_ never decreases, so no wrap handling is needed.
        if (odoMm_ - lastOdoWriteMm_ >= ODO_NVS_WRITE_INTERVAL_MM) {
            persistDistance();
        }

        // Measure over the interval since the last window that SAW edges, not since the
        // last sample: below ~5 km/h one pulse period spans several sample windows, and
        // dividing by the sample interval would multiply the reading (a 1 km/h crawl would
        // read ~8 km/h and trip the interlock). After a stale zero every pulse in this delta
        // arrived within the last sample, so the window is dtMs.
        uint32_t windowMs = now - lastPulseMs_;
        if (staleHandled_ || windowMs == 0) windowMs = dtMs;
        // mm/ms IS metres per second by definition — no conversion, and none wanted:
        // m/s is the firmware's internal speed unit everywhere.
        speedMs_ = (delta * distancePerPulseMm_) / (float)windowMs;
        lastPulseMs_       = now;
        lastMovingSpeedMs_ = speedMs_;
        staleHandled_       = false;
        suspicious_         = false;   // a live pulse train clears a latched fault
        if (!everPulsed_) {
            everPulsed_ = true;
            Debug::printlnFeature(DebugFeature::VEHICLE, "[SPEED] First pulses counted — reading is now valid");
        }
        return;
    }

    if (staleHandled_) {
        return;
    }
    uint32_t silenceMs = now - lastPulseMs_;

    float plausibleMs = lastMovingSpeedMs_
                      - SPEED_MAX_PLAUSIBLE_DECEL_MS2 * (silenceMs * 0.001f);
    if (plausibleMs < 0.0f) plausibleMs = 0.0f;
    if (speedMs_ > plausibleMs) speedMs_ = plausibleMs;

    if (!suspicious_ && lastMovingSpeedMs_ > TRANS_SPEED_INTERLOCK_THRESHOLD_MS) {
        float impliedMaxMs = distancePerPulseMm_ / (float)silenceMs;
        if (plausibleMs > impliedMaxMs) {
            suspicious_ = true;
            Debug::printfFeature(DebugFeature::VEHICLE,
                "[SPEED] WARNING: pulses stopped implausibly fast from %.2f m/s (%.1f km/h, %lu ms) — reading marked invalid\n",
                lastMovingSpeedMs_, lastMovingSpeedMs_ * MS_TO_KMH, (unsigned long)silenceMs);
        }
    }

    if (silenceMs >= SPEED_STALE_TIMEOUT_MS) {
        staleHandled_      = true;
        speedMs_           = 0.0f;
        lastMovingSpeedMs_ = 0.0f;
    }
}

void SpeedSensor::persistDistance() {
    // The dirty flag is the whole write budget. Several triggers repeat freely — applyFailsafe()
    // fires on the first loop of EVERY boot (the input source is FAILSAFE until MAVLink or a
    // browser appears) and again on every link flap, and getIgnitionState() has no hysteresis, so
    // a servo channel jittering at a band edge can produce one "OFF transition" per inbound frame
    // at 25 Hz. Without this guard each of those would rewrite values NVS already holds.
    // A bare odoMm_ == lastOdoWriteMm_ test would not do: resetTrip() must write even when the
    // vehicle has not moved a millimetre.
    if (!distanceDirty_) {
        return;                  // NVS already holds these values — no open, no log, no wear
    }

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        // Never block and never retry in a loop. Advancing the mark here is deliberate: it
        // disarms the 1 km trigger, which would otherwise stay true and re-attempt (and re-log)
        // at the 5 Hz sample rate for as long as the vehicle keeps moving. distanceDirty_ stays
        // set, so the next kilometre — or the next ignition-OFF / trip reset — tries again.
        Debug::printlnFeature(DebugFeature::VEHICLE, "[SPEED] NVS open failed — distance not persisted");
        lastOdoWriteMm_ = odoMm_;
        return;
    }
    // Preferences returns the number of bytes written, 0 on failure. A partial write leaves the
    // counters dirty so a later trigger retries, but the mark still advances for the same
    // no-spinning reason as above.
    size_t wroteOdo  = prefs.putULong64("odo_mm", odoMm_);
    size_t wroteTrip = prefs.putULong64("trip_mm", tripMm_);
    prefs.end();
    lastOdoWriteMm_ = odoMm_;
    if (wroteOdo == 0 || wroteTrip == 0) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[SPEED] NVS write failed");
        return;                  // still dirty — retry at the next trigger
    }
    distanceDirty_ = false;
}

void SpeedSensor::resetTrip() {
    tripMm_ = 0;                 // odoMm_ is a vehicle-lifetime counter and is NEVER touched here
    distanceDirty_ = true;       // a reset must reach NVS even if the vehicle never moved
    persistDistance();
    Debug::printfFeature(DebugFeature::VEHICLE, "[SPEED] TRIP reset to 0 (ODO %.3f km unchanged)\n",
                         getOdoKm());
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
