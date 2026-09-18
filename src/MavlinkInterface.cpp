#include "MavlinkInterface.h"
#include "Debug.h"

// MAVLink C library (header-only). ArduPilotMega dialect (superset of common).
#include <ardupilotmega/mavlink.h>
#include <math.h>   // NAN
#include <esp_timer.h>  // esp_timer_get_time() — 64-bit monotonic µs (micros() wraps every ~71 min)

MavlinkInterface::MavlinkInterface()
    : serial_(nullptr),
      lastCmdTime_(0),
      totalCommands_(0),
      lastHeartbeatTime_(0),
      heartbeatSeen_(false),
      rateWindowStart_(0),
      rateWindowCount_(0),
      lastCmdRate_(0.0f),
      lastOdomTimeUs_(0),
      lastOdomDtMs_(0),
      odomTxCount_(0),
      targetSystem_(0),
      targetComponent_(0),
      targetKnown_(false),
      lastStreamRequestTime_(0),
      targetLearnedMs_(0),
      speedMaxMs_(NAN),
      speedMaxRxMs_(0),
      lastParamRequestMs_(0),
      lastHeartbeatTx_(0),
      lastReportTx_(0),
      lastStatustextTx_(0),
      lastFailsafe_(false),
      stateInitialized_(false) {
    for (uint8_t i = 0; i < 16; i++) {
        channels_[i] = RC_US_CENTER;
    }
    lastGear_[0] = '\0';
    lastIgnition_[0] = '\0';
}

bool MavlinkInterface::begin(uint8_t rxPin, uint8_t txPin, uint8_t uartNum, uint32_t baud) {
    // UART0 (GPIO43/44) is RESERVED for the future CH9121T wired LAN and is never
    // opened by the firmware; the console is USB-CDC. MAVLink lives on UART1 (X9).
    if (uartNum == UART_NUM_1) {
        serial_ = &Serial1;
    } else {
        Debug::printlnFeature(DebugFeature::MAVLINK,
            "[MAV] ERROR: Invalid UART number (only UART1 is available for MAVLink)");
        return false;
    }

    // Non-inverted, 8N1, RX + TX
    serial_->begin(baud, SERIAL_8N1, rxPin, txPin);

    // Force MAVLink 2 on the outbound channel
    mavlink_status_t* chan = mavlink_get_channel_status(MAVLINK_COMM_0);
    if (chan) {
        chan->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }

    uint32_t now = millis();
    lastCmdTime_ = 0;          // no command yet → signal invalid until first frame
    lastHeartbeatTime_ = 0;
    rateWindowStart_ = now;
    rateWindowCount_ = 0;
    lastOdomTimeUs_ = 0;       // no odometry baseline yet → the first send re-baselines
    speedMaxMs_ = NAN;         // no SPEED_MAX yet → the vehicle layer uses its stored ceiling
    speedMaxRxMs_ = 0;
    lastParamRequestMs_ = 0;   // first poll is scheduled off targetLearnedMs_

    Debug::printfFeature(DebugFeature::MAVLINK,
        "[MAV] Initialized on UART%d RX=%d TX=%d @ %lu baud (MAVLink2)\n",
        uartNum, rxPin, txPin, (unsigned long)baud);
    return true;
}

void MavlinkInterface::update() {
    if (!serial_) {
        return;
    }

    mavlink_message_t msg;
    mavlink_status_t status;

    // Drain the RX buffer (non-blocking)
    while (serial_->available() > 0) {
        uint8_t c = (uint8_t)serial_->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    handleHeartbeat(msg.sysid, msg.compid);
                    break;
                case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
                    mavlink_servo_output_raw_t so;
                    mavlink_msg_servo_output_raw_decode(&msg, &so);
                    const uint16_t servoUs[16] = {
                        so.servo1_raw,  so.servo2_raw,  so.servo3_raw,  so.servo4_raw,
                        so.servo5_raw,  so.servo6_raw,  so.servo7_raw,  so.servo8_raw,
                        so.servo9_raw,  so.servo10_raw, so.servo11_raw, so.servo12_raw,
                        so.servo13_raw, so.servo14_raw, so.servo15_raw, so.servo16_raw
                    };
                    handleServoOutputRaw(servoUs);
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t pv;
                    mavlink_msg_param_value_decode(&msg, &pv);
                    handleParamValue(msg.sysid, msg.compid, pv.param_id, pv.param_value);
                    break;
                }
                case MAVLINK_MSG_ID_COMMAND_ACK: {
                    mavlink_command_ack_t ack;
                    mavlink_msg_command_ack_decode(&msg, &ack);
                    if (ack.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
                        const char* r =
                            (ack.result == MAV_RESULT_ACCEPTED) ? "ACCEPTED" :
                            (ack.result == MAV_RESULT_TEMPORARILY_REJECTED) ? "TEMP_REJECTED" :
                            (ack.result == MAV_RESULT_DENIED) ? "DENIED" :
                            (ack.result == MAV_RESULT_UNSUPPORTED) ? "UNSUPPORTED" :
                            (ack.result == MAV_RESULT_FAILED) ? "FAILED" : "OTHER";
                        Debug::printfFeature(DebugFeature::MAVLINK,
                            "[MAV] SET_MESSAGE_INTERVAL ack: %s (%u)\n", r, ack.result);
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }

    uint32_t now = millis();

    // Request — and periodically re-request — the command stream until it is flowing
    // at a healthy rate. The one-shot request can be lost or rejected at boot;
    // re-requesting also recovers after an autopilot reboot. SERVO_OUTPUT_RAW is the only
    // inbound STREAM: the outbound odometry is body-frame and needs no yaw. (The SPEED_MAX
    // parameter below is polled per-value, not streamed.)
    if (targetKnown_ && isLinkUp() && lastCmdRate_ < MAVLINK_STREAM_MIN_RATE_HZ &&
        (lastStreamRequestTime_ == 0 || now - lastStreamRequestTime_ >= MAVLINK_STREAM_REREQUEST_MS)) {
        requestServoOutputStream();
        lastStreamRequestTime_ = now;
    }

    // Poll the autopilot's SPEED_MAX. The poll NEVER stops once a value has been received —
    // it IS the change detector. ArduPilot does broadcast a PARAM_VALUE after a PARAM_SET, but
    // whether that broadcast reaches a peripheral component depends on the firmware version and
    // the routing between ports, so the unsolicited path (handled in the RX switch) is treated as
    // an optimisation and this poll as the guarantee. One 20-byte request every 5 s is negligible
    // beside the 25 Hz command stream on the same 115200 link.
    if (targetKnown_ && isLinkUp() &&
        (lastParamRequestMs_ == 0
            ? (now - targetLearnedMs_) >= MAVLINK_PARAM_FIRST_DELAY_MS
            : (now - lastParamRequestMs_) >= MAVLINK_PARAM_POLL_MS)) {
        requestSpeedMaxParam();
        lastParamRequestMs_ = now;
    }

    // Rolling command-rate estimate
    if (now - rateWindowStart_ >= 1000) {
        float windowMs = (float)(now - rateWindowStart_);
        lastCmdRate_ = (float)rateWindowCount_ * 1000.0f / windowMs;
        rateWindowStart_ = now;
        rateWindowCount_ = 0;
    }

    // Periodic link log
    static uint32_t lastLog = 0;
    if (now - lastLog >= 1000) {
        lastLog = now;
        Debug::printfFeature(DebugFeature::MAVLINK,
            "[MAV] cmds:%lu rate:%.1fHz age:%lums valid:%s hbAge:%lums link:%s\n",
            (unsigned long)totalCommands_, lastCmdRate_, (unsigned long)getSignalAge(),
            isSignalValid() ? "Y" : "N",
            (unsigned long)(heartbeatSeen_ ? now - lastHeartbeatTime_ : 0),
            isLinkUp() ? "Y" : "N");
        // Decoded command channels (µs) — steering / throttle-brake / gear / ignition / light
        Debug::printfFeature(DebugFeature::MAVLINK,
            "[MAV] ch S:%u T:%u G:%u I:%u L:%u\n",
            getChannel(ServoChannelConfig::STEERING),
            getChannel(ServoChannelConfig::THROTTLE),
            getChannel(ServoChannelConfig::TRANSMISSION),
            getChannel(ServoChannelConfig::IGNITION),
            getChannel(ServoChannelConfig::FRONT_LIGHT));
        // Wheel-odometry health: the TX count and the last integration interval. A frozen
        // viso count means a gate is closed (link, speed validity, or rolling in
        // neutral/unknown); a dt far from MAVLINK_REPORT_TX_MS means the baseline keeps
        // being re-established, i.e. a gate is flapping.
        // spdmax/age/valid make the SPEED_MAX subscription diagnosable on serial alone:
        // "nan" = never received, a sawtooth age 0→MAVLINK_PARAM_POLL_MS = the poll is answered,
        // and valid:N with a real value = stale or link down (the vehicle layer has fallen back).
        Debug::printfFeature(DebugFeature::MAVLINK,
            "[MAV] viso:%lu dt:%lums spdmax:%.2fm/s age:%lums valid:%s\n",
            (unsigned long)odomTxCount_, (unsigned long)lastOdomDtMs_,
            speedMaxMs_, (unsigned long)getSpeedMaxAgeMs(),
            hasSpeedMaxParam() ? "Y" : "N");
    }
}

void MavlinkInterface::handleHeartbeat(uint8_t sysid, uint8_t compid) {
    // Accept the autopilot's heartbeat as the command target.
    if (compid == MAV_COMP_ID_AUTOPILOT1) {
        if (!targetKnown_) {
            targetSystem_ = sysid;
            targetComponent_ = compid;
            targetKnown_ = true;
            targetLearnedMs_ = millis();   // schedules the first SPEED_MAX request
            Debug::printfFeature(DebugFeature::MAVLINK,
                "[MAV] Autopilot learned: sys=%u comp=%u\n", sysid, compid);
        }
        lastHeartbeatTime_ = millis();
        heartbeatSeen_ = true;
    }
}

void MavlinkInterface::handleServoOutputRaw(const uint16_t* servoUs) {
    for (uint8_t i = 0; i < 16; i++) {
        channels_[i] = clampUs(servoUs[i]);
    }
    lastCmdTime_ = millis();
    totalCommands_++;
    rateWindowCount_++;
}

void MavlinkInterface::requestServoOutputStream() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    // 1) Legacy REQUEST_DATA_STREAM — sets the rate of the whole RC_CHANNELS
    //    stream group (which contains SERVO_OUTPUT_RAW). Honored by ArduPilot on
    //    ports where SET_MESSAGE_INTERVAL is denied (e.g. SERIAL4/TELEM4).
    mavlink_msg_request_data_stream_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        targetSystem_, targetComponent_,
        MAV_DATA_STREAM_RC_CHANNELS,            // stream group containing SERVO_OUTPUT_RAW
        MAVLINK_SERVO_OUTPUT_RATE_HZ,           // requested rate (Hz)
        1);                                     // start
    len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_->write(buf, len);

    // 2) Modern SET_MESSAGE_INTERVAL — kept for autopilots/ports that honor it.
    const float intervalUs = 1000000.0f / (float)MAVLINK_SERVO_OUTPUT_RATE_HZ;
    mavlink_msg_command_long_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        targetSystem_, targetComponent_,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        (float)MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,  // param1: message id
        intervalUs,                              // param2: interval (µs)
        0, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_->write(buf, len);

    Debug::printfFeature(DebugFeature::MAVLINK,
        "[MAV] Requested SERVO_OUTPUT_RAW @ %d Hz (REQUEST_DATA_STREAM + SET_MESSAGE_INTERVAL)\n",
        MAVLINK_SERVO_OUTPUT_RATE_HZ);
}

// ============================================================================
// AUTOPILOT PARAMETER SUBSCRIPTION (SPEED_MAX) — READ-ONLY
// ============================================================================

void MavlinkInterface::requestSpeedMaxParam() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // param_index = -1 means "look the parameter up by name" — the index is not stable
    // across firmware builds and must never be hard-coded.
    mavlink_msg_param_request_read_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        targetSystem_, targetComponent_,
        MAVLINK_PARAM_SPEED_MAX_ID,
        -1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_->write(buf, len);
}

void MavlinkInterface::handleParamValue(uint8_t sysid, uint8_t compid,
                                        const char* paramId, float value) {
    // Only the LEARNED autopilot may move this vehicle's speed ceiling. A ground station on the
    // same wire (typically sysid 255) must not be able to, and neither may any other component.
    if (!targetKnown_ || sysid != targetSystem_ || compid != targetComponent_) {
        return;
    }

    // param_id is a 16-byte field that MAVLink does NOT NUL-terminate when the name fills it.
    // Copy into a 17-byte buffer and terminate before any string comparison.
    char id[17];
    memcpy(id, paramId, 16);
    id[16] = '\0';
    if (strcmp(id, MAVLINK_PARAM_SPEED_MAX_ID) != 0) {
        return;
    }

    // Reject anything that is not a usable speed, and leave the previous value standing: one
    // corrupt frame must not invalidate a good subscription. NaN matters most — it makes every
    // "speed <= limit" comparison false, which would pin the throttle ceiling at the floor.
    if (isnan(value) || isinf(value) || value < 0.0f || value > MAVLINK_PARAM_SPEED_MAX_MS) {
        Debug::printfFeature(DebugFeature::MAVLINK,
            "[MAV] SPEED_MAX rejected: %.3f (expect 0..%.1f m/s)\n",
            value, MAVLINK_PARAM_SPEED_MAX_MS);
        return;
    }

    bool changed = isnan(speedMaxMs_) || fabsf(value - speedMaxMs_) > MAVLINK_PARAM_EPSILON_MS;
    speedMaxMs_ = value;
    speedMaxRxMs_ = millis();

    // Log on change only — a 5 s poll of an unchanged parameter would otherwise fill the console.
    if (changed) {
        Debug::printfFeature(DebugFeature::MAVLINK,
            "[MAV] SPEED_MAX = %.2f m/s (%.1f km/h)%s\n",
            value, value * MAVLINK_MS_TO_KMH,
            (value <= 0.0f) ? " — 0 means \"not set\", limiter falls back to the stored ceiling" : "");
    }
}

bool MavlinkInterface::hasSpeedMaxParam() const {
    if (isnan(speedMaxMs_) || speedMaxMs_ <= 0.0f) {
        return false;   // never received, or ArduPilot's "not set" zero
    }
    if (!isLinkUp()) {
        return false;   // cable out / autopilot down — fall back within the heartbeat timeout
    }
    return (millis() - speedMaxRxMs_) < MAVLINK_PARAM_STALE_MS;
}

float MavlinkInterface::getSpeedMaxKmh() const {
    if (isnan(speedMaxMs_)) {
        return 0.0f;
    }
    return speedMaxMs_ * MAVLINK_MS_TO_KMH;
}

float MavlinkInterface::getSpeedMaxRawMs() const {
    return speedMaxMs_;
}

uint32_t MavlinkInterface::getSpeedMaxAgeMs() const {
    if (speedMaxRxMs_ == 0) {
        return MAVLINK_PARAM_STALE_MS;   // never received — treat as fully stale
    }
    return millis() - speedMaxRxMs_;
}

// ============================================================================
// STATE REPORTING
// ============================================================================

void MavlinkInterface::report(const StateReport& state) {
    if (!serial_) {
        return;
    }

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint32_t now = millis();

    // HEARTBEAT (1 Hz)
    if (now - lastHeartbeatTx_ >= MAVLINK_HEARTBEAT_TX_MS) {
        lastHeartbeatTx_ = now;
        uint8_t systemStatus = state.failsafe ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_pack(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
            MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_INVALID,
            0 /*base_mode*/, 0 /*custom_mode*/, systemStatus);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        serial_->write(buf, len);
    }

    // All vehicle telemetry in ONE EFI_STATUS message from this component (25), so the values
    // can never override one another (unlike same-named NAMED_VALUE_FLOAT, which share a single
    // message id and collide in name-agnostic stores). Read by the QuadBike MP plugin via a
    // packet subscription.
    //
    // PRINCIPLE: every value sits in the field that NAMES it, so the message is self-describing.
    // Only three fields are repurposed, and each is documented below.
    //
    //   rpm                         -> engine RPM (PID 0x0C)                       NaN if !canValid
    //   cylinder_head_temperature   -> coolant temperature (°C, PID 0x05)          NaN if !canValid
    //   intake_manifold_temperature -> intake air temperature (°C, PID 0x0F)       NaN if !canValid
    //   intake_manifold_pressure    -> manifold absolute pressure (kPa, PID 0x0B)  NaN if !canValid
    //   engine_load                 -> ECU CALCULATED load (%, PID 0x04)           NaN if !canValid
    //   throttle_position           -> MEASURED throttle (%, PID 0x11)             NaN if !canValid
    //   throttle_out                -> COMMANDED/arbitrated throttle (%)           always valid
    //   ignition_voltage            -> module supply voltage (V, PID 0x42)         NaN if !canValid
    //   fuel_consumed               -> ASSUMED gear   (repurposed)                 always valid
    //   fuel_flow                   -> PHYSICAL gear  (repurposed)                 NaN if UNKNOWN
    //   pt_compensation             -> digital-output bitmask (repurposed)         always valid
    //
    // Both gear values use the PHYSICAL SEQUENCE encoding [R, N, H, L] = [-1, 0, 1, 2].
    // fuel_consumed is the controller's commanded gear: while the servo moves between two gears it
    // is their MIDPOINT (0.5 staircase), settling on the integer. fuel_flow is the opto-switch
    // measurement — never interpolated, NaN whenever the physical gear reads UNKNOWN.
    //
    // Why the fuel pair is safe to repurpose FOREVER on this vehicle: the 2026-08-14 bench probe
    // confirmed PIDs 0x2F (fuel level) and 0x5C (oil temp) are absent from this ECU's supported-PID
    // bitmaps and do not answer, so no genuine fuel quantity or flow can ever be displaced. Fields
    // naming quantities this ECU could plausibly expose later (spark_dwell_time, ignition_timing,
    // injection_time, exhaust_gas_temperature, barometric_pressure, fuel_pressure) are left at zero
    // rather than repurposed.
    //
    // CONSUMER NOTE: fuel_consumed != fuel_flow, or fuel_flow == NaN, is the NORMAL signature of a
    // gear in motion — not a fault. Only a persistent disagreement after the assumed gear settles
    // on an integer (or a persistent NaN) indicates a real gear-sensor problem. fuel_flow is also
    // the ONE field whose NaN is gear-sensor-driven rather than CAN-driven, so "everything NaN
    // means CAN is down" does not hold in reverse for it.
    //
    // BREAKING vs the pre-remap mapping: gear moved engine_load -> fuel_consumed and measured TPS
    // moved throttle_out -> throttle_position. The MP plugin must be updated in lockstep with the
    // flash (see MAVLINK_SETUP.md -> "Ground-station plugin compatibility").
    //
    // ArduPilot won't surface a peripheral's EFI in MP's native efi_* fields, but the raw packet
    // IS delivered, so the plugin's subscription receives it cleanly. The CAN-gated fields are NaN
    // while CAN data is invalid (so the plugin shows "--" instead of misleading zeros).
    if (now - lastReportTx_ >= MAVLINK_REPORT_TX_MS) {
        lastReportTx_ = now;

        // ASSUMED gear: the controller always knows what it commanded, so this is never NaN.
        float gearVal = state.gearMoving
            ? (encodeGear(state.gearFrom) + encodeGear(state.gearTo)) * 0.5f
            : encodeGear(state.gearTo);
        // PHYSICAL gear from the opto switches — NaN when they read UNKNOWN (mid-shift,
        // ambiguous pattern, or a faulted input expander). Its validity is the SENSOR's,
        // deliberately independent of CAN health.
        float physGearVal = encodeGear(state.gearPhysical, NAN);
        float rpmVal = state.canValid ? (float)state.engineRpm  : NAN;
        float chtVal = state.canValid ? (float)state.coolantTemp : NAN;
        float iatVal = state.canValid ? (float)state.intakeTemp : NAN;
        float mapVal = state.canValid ? (float)state.mapKpa : NAN;
        float loadVal = state.canValid ? (float)state.engineLoad : NAN;
        float voltVal = state.canValid ? (state.moduleVoltageMv / 1000.0f) : NAN;
        // MEASURED throttle (ECU truth) and COMMANDED throttle (arbitrated servo output) are two
        // different values and get two different fields, so a consumer can compare them directly.
        // Only the measured one can go missing; the commanded one is always known locally.
        float tpsVal = state.canValid ? (float)state.throttlePosition : NAN;
        float thrOutVal = (float)state.throttleCmdPct;
        // Digital output states packed as a bitmask (see EFI_DIGITAL_FLAG_* in Constants.h):
        // bit0 = wheel lock, bit1 = front light. Always valid (relay ground-truth), never NaN.
        float flagsVal = (float)state.digitalFlags;

        // One argument per line: 19 same-typed positional floats, so a mis-ordered argument
        // compiles cleanly and only fails on the bench. Order verified against
        // .pio/libdeps/esp32-s3-devkitc-1/c_library_v2/common/mavlink_msg_efi_status.h.
        mavlink_msg_efi_status_pack(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
            1,             // health (1 = present)
            0.0f,          // ecu_index
            rpmVal,        // rpm                         <- ENGINE RPM (0x0C)
            gearVal,       // fuel_consumed               <- ASSUMED GEAR (repurposed)
            physGearVal,   // fuel_flow                   <- PHYSICAL GEAR (repurposed)
            loadVal,       // engine_load                 <- ECU CALCULATED LOAD (0x04)
            tpsVal,        // throttle_position           <- MEASURED TPS (0x11)
            0.0f,          // spark_dwell_time            (unused)
            0.0f,          // barometric_pressure         (unused)
            mapVal,        // intake_manifold_pressure    <- MAP (0x0B)
            iatVal,        // intake_manifold_temperature <- INTAKE AIR TEMP (0x0F)
            chtVal,        // cylinder_head_temperature   <- COOLANT (0x05)
            0.0f,          // ignition_timing             (unused)
            0.0f,          // injection_time              (unused)
            0.0f,          // exhaust_gas_temperature     (unused)
            thrOutVal,     // throttle_out                <- COMMANDED THROTTLE
            flagsVal,      // pt_compensation             <- DIGITAL FLAGS bitmask (repurposed)
            voltVal,       // ignition_voltage            <- MODULE VOLTAGE (0x42)
            0.0f);         // fuel_pressure               (unused)
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        serial_->write(buf, len);

        // Ground speed from the hall speed sensor, in the standard VFR_HUD field the GCS
        // already graphs for a MAV_TYPE_GROUND_ROVER (no custom NAMED_VALUE_FLOAT needed).
        // Its validity is the SENSOR's, independent of CAN health; a stale/invalid reading
        // is reported as 0 rather than presented as genuine motion.
        //
        // throttle is MEASURED TPS while CAN is valid, falling back to the COMMANDED
        // (arbitrated servo) percent when it is not: the field is a uint16_t percent with no
        // NaN or "unknown" encoding, and a HUD bar frozen at 0 while the operator is holding
        // throttle misleads in the more dangerous direction. The substitution is never
        // ambiguous — EFI_STATUS.throttle_position reads NaN in this same tick exactly when the
        // fallback is in use, so a consumer can always tell measured from commanded (and the
        // value shown here then equals EFI_STATUS.throttle_out, which always carries commanded).
        // airspeed, heading, alt and climb remain zero: no source on this component.
        float groundSpeedMs = (state.speedValid && state.speedKmh > 0.0f)
            ? (state.speedKmh / 3.6f)
            : 0.0f;
        uint16_t throttlePct = state.canValid ? state.throttlePosition : state.throttleCmdPct;
        mavlink_msg_vfr_hud_pack(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
            0.0f,           // airspeed
            groundSpeedMs,  // groundspeed (m/s)  <- HALL SPEED SENSOR
            0,              // heading
            throttlePct,    // throttle (%)  <- MEASURED TPS, else COMMANDED
            0.0f, 0.0f);    // alt, climb
        len = mavlink_msg_to_send_buffer(buf, &msg);
        serial_->write(buf, len);

#if MAVLINK_VISO_ENABLED
        // The same wheel speed again, but as a FUSABLE measurement rather than a display
        // value (VFR_HUD above is display-only). Heavily gated — see the function.
        sendVisionPositionDelta(state);
#endif
    }

    // STATUSTEXT on gear / ignition / fail-safe transitions (rate-limited).
    // Uses the current step / assumed gear (gearTo).
    const char* gear = state.gearTo ? state.gearTo : "?";
    const char* ign  = state.ignition ? state.ignition : "?";
    bool changed = !stateInitialized_ ||
                   strncmp(gear, lastGear_, sizeof(lastGear_)) != 0 ||
                   strncmp(ign, lastIgnition_, sizeof(lastIgnition_)) != 0 ||
                   state.failsafe != lastFailsafe_;

    if (changed && (now - lastStatustextTx_ >= MAVLINK_STATUSTEXT_MIN_MS)) {
        lastStatustextTx_ = now;
        char text[50];
        snprintf(text, sizeof(text), "gear=%s ign=%s%s",
                 gear, ign, state.failsafe ? " FAILSAFE" : "");
        uint8_t severity = state.failsafe ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO;
        sendStatusText(severity, text);

        strncpy(lastGear_, gear, sizeof(lastGear_) - 1);
        lastGear_[sizeof(lastGear_) - 1] = '\0';
        strncpy(lastIgnition_, ign, sizeof(lastIgnition_) - 1);
        lastIgnition_[sizeof(lastIgnition_) - 1] = '\0';
        lastFailsafe_ = state.failsafe;
        stateInitialized_ = true;
    }
}

// Wheel speed as EKF3-fusable BODY-FRAME odometry (VISION_POSITION_DELTA, msg 11011).
//
// WHY THIS MESSAGE AND NOT VISION_SPEED_ESTIMATE: on 2026-08-21 the earth-frame velocity path was
// falsified in source and on the bench. EKF3 never STARTS aiding from extnav velocity —
// readyToUseExtNav() is position-only, and while the filter sits in constant-position mode the
// velocity observation carries EK3_NOAID_M_NSE noise and is discarded. VISION_POSITION_DELTA is
// the one MAVLink message that feeds writeBodyFrameOdom(), which readyToUseBodyOdm() accepts with
// EK3_SRC1_VELXY=6 — that is what actually moved the filter to AID_RELATIVE on the bench. Full
// citations in openspec/changes/add-extnav-velocity/design.md.
//
// The delta is measured along the vehicle's OWN longitudinal axis, so there is no rotation to do
// and no yaw to go stale: EKF3 rotates it with its own attitude. The sign still comes from the
// PHYSICAL gear (state.travelDirection) — the sensor counts pulses and cannot tell forward from
// reverse.
//
// Policy is SILENCE OVER ZEROS: whenever validity or sign is in doubt the message is skipped
// entirely, because a zero the autopilot fuses as "stopped" is far worse than a gap it simply
// coasts through. The one deliberate exception is a HEALTHY zero — a genuine standstill
// (including idling in neutral) is sent, since with no reliable GPS a zero-motion update is the
// strongest constraint on estimator drift available to this vehicle.
void MavlinkInterface::sendVisionPositionDelta(const StateReport& state) {
    // Gate 1: nobody is listening.
    if (!isLinkUp()) {
        lastOdomTimeUs_ = 0;
        return;
    }
    // Gate 2: sensor health. This covers the speed sensor's wire-fault (suspicious) latch —
    // a cut signal wire decays to 0 km/h and looks exactly like "stopped" while the vehicle
    // is still rolling. That is the single most dangerous zero in the system.
    if (!state.speedValid) {
        lastOdomTimeUs_ = 0;
        return;
    }
    // Gate 3: rolling with no recoverable sign (neutral, or gear unknown mid-shift). A wrong
    // sign injects an error of TWICE the speed; a skipped sample costs nothing. Note the
    // comparison lets a genuine standstill through as a zero-motion update.
    if (state.travelDirection == 0 && state.speedKmh > MAVLINK_VISO_NEUTRAL_ZERO_KMH) {
        lastOdomTimeUs_ = 0;
        return;
    }

    // A DELTA is an integral, so it is only meaningful over an interval we actually observed.
    // Every gate above invalidates the baseline on the way out, and a stall of the report loop
    // trips MAVLINK_VISO_MAX_DT_US, so the first send after any break re-baselines and skips one
    // interval rather than multiplying the current speed by a long-dead dt — which would present
    // as one enormous jump in distance and blow the EKF's innovation gate.
    const int64_t nowUs = esp_timer_get_time();
    const int64_t dtUs = nowUs - lastOdomTimeUs_;
    if (lastOdomTimeUs_ == 0 || dtUs <= 0 || dtUs > MAVLINK_VISO_MAX_DT_US) {
        lastOdomTimeUs_ = nowUs;
        return;
    }
    lastOdomTimeUs_ = nowUs;
    lastOdomDtMs_ = (uint32_t)(dtUs / 1000);

    // Signed longitudinal speed (m/s) integrated over the MEASURED interval — never an assumed
    // 200 ms, because the cooperative loop's tick jitters and ArduPilot divides by exactly the
    // time_delta_usec we send here to recover the velocity.
    const float dtSec = (float)dtUs * 1.0e-6f;
    const float speedMs = (state.speedKmh / 3.6f) * (float)state.travelDirection;

    // Body frame, x forward. y (right) and z (down) are exactly 0: the wheel measures only the
    // longitudinal axis, and a ground rover has no body-frame lateral or vertical travel to
    // report. angle_delta is likewise 0 — an honest "this sensor measures no rotation". The
    // vehicle's rotation is the autopilot's own gyros' job; there is no independent rotation
    // sensor here to report instead.
    float positionDelta[3] = { speedMs * dtSec, 0.0f, 0.0f };
    float angleDelta[3]    = { 0.0f, 0.0f, 0.0f };

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_vision_position_delta_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        (uint64_t)nowUs,            // time_usec — 64-bit monotonic boot µs. NEVER micros():
                                    // its 32-bit wrap (~71 min) would date the sample.
        (uint64_t)dtUs,             // time_delta_usec — the interval the delta was integrated over
        angleDelta,
        positionDelta,
        MAVLINK_VISO_CONFIDENCE);   // confidence 0-100 → EKF3 velErr = EK3_VIS_VERR_MIN..MAX
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_->write(buf, len);
    odomTxCount_++;
}

void MavlinkInterface::sendStatusText(uint8_t severity, const char* text) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_statustext_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        severity, text, 0 /*id*/, 0 /*chunk_seq*/);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_->write(buf, len);
}

float MavlinkInterface::encodeGear(const char* gear, float unknownValue) {
    // Physical sequence position [R, N, H, L] = [-1, 0, 1, 2].
    // A null or unrecognised name (notably "?" = UNKNOWN) yields unknownValue: 0.0f
    // for the ASSUMED gear (its historical behaviour), NAN for the PHYSICAL gear —
    // "cannot tell" must never be reported as NEUTRAL.
    if (!gear) return unknownValue;
    switch (gear[0]) {
        case 'R': return -1.0f;
        case 'N': return  0.0f;
        case 'H': return  1.0f;
        case 'L': return  2.0f;
        default:  return unknownValue;
    }
}

// ============================================================================
// RAW CHANNEL ACCESS
// ============================================================================

uint16_t MavlinkInterface::getChannel(uint8_t channel) const {
    if (channel < 1 || channel > 16) {
        return 0;
    }
    return channels_[channel - 1];
}

void MavlinkInterface::getRawChannels(uint16_t* channels) const {
    for (uint8_t i = 0; i < 16; i++) {
        channels[i] = channels_[i];
    }
}

// ============================================================================
// MAPPED VEHICLE COMMANDS  (identical mapping to the former S-bus path)
// ============================================================================

float MavlinkInterface::getSteering() const {
    uint16_t valueUs = getChannel(ServoChannelConfig::STEERING);
    float percentage = mapToPercentageBidirectional(valueUs, RC_US_MIN, RC_US_CENTER, RC_US_MAX);
    return applyDeadband(percentage, 0.0f, RC_STEERING_DEADBAND);
}

float MavlinkInterface::getThrottle() const {
    uint16_t valueUs = getChannel(ServoChannelConfig::THROTTLE);
    if (valueUs <= RC_US_CENTER) {
        return 0.0f;
    }
    float percentage = mapToPercentage(valueUs, RC_US_CENTER, RC_US_MAX);
    if (percentage < RC_THROTTLE_DEADBAND) {
        percentage = 0.0f;
    }
    return percentage;
}

TransmissionController::Gear MavlinkInterface::getGear() const {
    uint16_t valueUs = getChannel(ServoChannelConfig::TRANSMISSION);
    if (valueUs >= RC_GEAR_REVERSE_MIN && valueUs <= RC_GEAR_REVERSE_MAX) {
        return TransmissionController::Gear::GEAR_REVERSE;
    } else if (valueUs >= RC_GEAR_NEUTRAL_MIN && valueUs <= RC_GEAR_NEUTRAL_MAX) {
        return TransmissionController::Gear::GEAR_NEUTRAL;
    } else if (valueUs >= RC_GEAR_LOW_MIN && valueUs <= RC_GEAR_LOW_MAX) {
        return TransmissionController::Gear::GEAR_LOW;
    }
    return TransmissionController::Gear::GEAR_NEUTRAL;
}

float MavlinkInterface::getBrake() const {
    uint16_t valueUs = getChannel(ServoChannelConfig::THROTTLE);
    if (valueUs >= RC_US_CENTER) {
        return 0.0f;
    }
    // Invert: center→0%, min→100%
    uint16_t inverted = RC_US_CENTER - (valueUs - RC_US_MIN);
    return mapToPercentage(inverted, RC_US_MIN, RC_US_CENTER);
}

MavlinkInterface::IgnitionState MavlinkInterface::getIgnitionState() const {
    uint16_t valueUs = getChannel(ServoChannelConfig::IGNITION);
    if (valueUs >= RC_IGNITION_OFF_MIN && valueUs <= RC_IGNITION_OFF_MAX) {
        return IgnitionState::OFF;
    } else if (valueUs >= RC_IGNITION_ACC_MIN && valueUs <= RC_IGNITION_ACC_MAX) {
        return IgnitionState::ACC;
    } else if (valueUs >= RC_IGNITION_ON_MIN && valueUs <= RC_IGNITION_ON_MAX) {
        return IgnitionState::IGNITION;
    }
    return IgnitionState::OFF;  // safe default
}

bool MavlinkInterface::getFrontLight() const {
    uint16_t valueUs = getChannel(ServoChannelConfig::FRONT_LIGHT);
    return (valueUs > RC_FRONT_LIGHT_THRESHOLD);
}

bool MavlinkInterface::getWheelLock() const {
    uint16_t valueUs = getChannel(ServoChannelConfig::WHEEL_LOCK);
    return (valueUs > RC_WHEEL_LOCK_THRESHOLD);
}

// ============================================================================
// LINK MONITORING
// ============================================================================

bool MavlinkInterface::isSignalValid() const {
    if (lastCmdTime_ == 0) {
        return false;  // no command frame yet
    }
    return (millis() - lastCmdTime_) < MAVLINK_CMD_TIMEOUT_MS;
}

uint32_t MavlinkInterface::getSignalAge() const {
    if (lastCmdTime_ == 0) {
        return MAVLINK_CMD_TIMEOUT_MS;  // treat as stale
    }
    return millis() - lastCmdTime_;
}

bool MavlinkInterface::isLinkUp() const {
    if (!heartbeatSeen_) {
        return false;
    }
    return (millis() - lastHeartbeatTime_) < MAVLINK_HEARTBEAT_TIMEOUT_MS;
}

MavlinkInterface::LinkQuality MavlinkInterface::getLinkQuality() const {
    LinkQuality q;
    q.commandRate = lastCmdRate_;
    q.signalAge = getSignalAge();
    q.heartbeatAge = heartbeatSeen_ ? (millis() - lastHeartbeatTime_) : 0;
    return q;
}

// ============================================================================
// MAPPING HELPERS
// ============================================================================

uint16_t MavlinkInterface::clampUs(uint16_t valueUs) const {
    if (valueUs < RC_US_MIN) return RC_US_MIN;
    if (valueUs > RC_US_MAX) return RC_US_MAX;
    return valueUs;
}

float MavlinkInterface::applyDeadband(float value, float center, float deadband) const {
    if (abs(value - center) < deadband) {
        return center;
    }
    return value;
}

float MavlinkInterface::mapToPercentage(uint16_t valueUs, uint16_t minUs, uint16_t maxUs) const {
    if (valueUs < minUs) valueUs = minUs;
    if (valueUs > maxUs) valueUs = maxUs;
    float percentage = (float)(valueUs - minUs) / (float)(maxUs - minUs) * 100.0f;
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;
    return percentage;
}

float MavlinkInterface::mapToPercentageBidirectional(uint16_t valueUs, uint16_t minUs,
                                                     uint16_t centerUs, uint16_t maxUs) const {
    if (valueUs < minUs) valueUs = minUs;
    if (valueUs > maxUs) valueUs = maxUs;
    float percentage;
    if (valueUs < centerUs) {
        percentage = (float)(valueUs - centerUs) / (float)(centerUs - minUs) * 100.0f;
    } else {
        percentage = (float)(valueUs - centerUs) / (float)(maxUs - centerUs) * 100.0f;
    }
    if (percentage < -100.0f) percentage = -100.0f;
    if (percentage > 100.0f) percentage = 100.0f;
    return percentage;
}
