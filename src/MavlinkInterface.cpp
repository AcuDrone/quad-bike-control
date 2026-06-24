#include "MavlinkInterface.h"
#include "Debug.h"

// MAVLink C library (header-only). ArduPilotMega dialect (superset of common).
#include <ardupilotmega/mavlink.h>

MavlinkInterface::MavlinkInterface()
    : serial_(nullptr),
      lastCmdTime_(0),
      totalCommands_(0),
      lastHeartbeatTime_(0),
      heartbeatSeen_(false),
      rateWindowStart_(0),
      rateWindowCount_(0),
      lastCmdRate_(0.0f),
      targetSystem_(0),
      targetComponent_(0),
      targetKnown_(false),
      lastStreamRequestTime_(0),
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
    if (uartNum == UART_NUM_1) {
        serial_ = &Serial1;
    } else if (uartNum == UART_NUM_0) {
        serial_ = &Serial;
    } else {
        Debug::printlnFeature(DebugFeature::MAVLINK,
            "[MAV] ERROR: Invalid UART number");
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

    // Request — and periodically re-request — the command stream until it is
    // flowing at a healthy rate. The one-shot request can be lost or rejected at
    // boot; re-requesting also recovers after an autopilot reboot.
    if (targetKnown_ && isLinkUp() &&
        lastCmdRate_ < MAVLINK_STREAM_MIN_RATE_HZ &&
        (lastStreamRequestTime_ == 0 || now - lastStreamRequestTime_ >= MAVLINK_STREAM_REREQUEST_MS)) {
        requestServoOutputStream();
        lastStreamRequestTime_ = now;
    }

    // Rolling command-rate estimate (1 s window)
    if (now - rateWindowStart_ >= 1000) {
        lastCmdRate_ = (float)rateWindowCount_ * 1000.0f / (float)(now - rateWindowStart_);
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
    }
}

void MavlinkInterface::handleHeartbeat(uint8_t sysid, uint8_t compid) {
    // Accept the autopilot's heartbeat as the command target.
    if (compid == MAV_COMP_ID_AUTOPILOT1) {
        if (!targetKnown_) {
            targetSystem_ = sysid;
            targetComponent_ = compid;
            targetKnown_ = true;
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

    // Engine telemetry (5 Hz)
    if (now - lastReportTx_ >= MAVLINK_REPORT_TX_MS) {
        lastReportTx_ = now;

        // Gear value, encoded by PHYSICAL SEQUENCE position [R, N, H, L] = [-1, 0, 1, 2].
        // While the servo is moving between two gears the value is their MIDPOINT, so a
        // multi-step change renders as an even 0.5 staircase (e.g. -0.5 = between R and N);
        // when settled it is the integer gear value. This is the controller's ASSUMED
        // (commanded) gear — the transmission is sensorless and time-based.
        float gearVal;
        if (state.gearMoving) {
            gearVal = (encodeGear(state.gearFrom) + encodeGear(state.gearTo)) * 0.5f;
        } else {
            gearVal = encodeGear(state.gearTo);
        }

        // EFI_STATUS — engine RPM, coolant temperature, and gear.
        // Sent as the AUTOPILOT component (MAVLINK_EFI_COMPONENT_ID) because Mission
        // Planner only maps EFI_STATUS into its labeled efi_* fields from that component.
        // Gear is carried in the otherwise-unused engine_load field so it appears as a
        // selectable/gaugeable "efi_load" value in MP. Throttle is intentionally not sent:
        // MP 1.3.83 does not surface EFI_STATUS.throttle_position, and the autopilot already
        // knows the commanded throttle. Vehicle speed and oil temperature are unavailable
        // from the ECU, so they are left zero.
        mavlink_efi_status_t efi;
        memset(&efi, 0, sizeof(efi));
        efi.health = state.canValid ? 1 : 0;
        efi.rpm = state.engineRpm;
        efi.cylinder_head_temperature = (float)state.coolantTemp;
        efi.engine_load = gearVal;   // gear -> MP "efi_load"
        mavlink_msg_efi_status_encode(MAVLINK_SYSTEM_ID, MAVLINK_EFI_COMPONENT_ID, &msg, &efi);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        serial_->write(buf, len);

        // GEAR also as a NAMED_VALUE_FLOAT — correctly labeled "GEAR" in the MP Tuning
        // graph (and as a customField in the Status tab).
        sendNamedValueFloat("GEAR", gearVal);
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

void MavlinkInterface::sendStatusText(uint8_t severity, const char* text) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_statustext_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        severity, text, 0 /*id*/, 0 /*chunk_seq*/);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_->write(buf, len);
}

void MavlinkInterface::sendNamedValueFloat(const char* name, float value) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // name is truncated to 10 chars by the message definition
    mavlink_msg_named_value_float_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        millis(), name, value);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_->write(buf, len);
}

float MavlinkInterface::encodeGear(const char* gear) {
    // Physical sequence position [R, N, H, L] = [-1, 0, 1, 2]
    if (!gear) return 0.0f;
    switch (gear[0]) {
        case 'R': return -1.0f;
        case 'N': return  0.0f;
        case 'H': return  1.0f;
        case 'L': return  2.0f;
        default:  return  0.0f;
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
    q.totalCommands = totalCommands_;
    q.commandRate = lastCmdRate_;
    q.signalAge = getSignalAge();
    q.heartbeatAge = heartbeatSeen_ ? (millis() - lastHeartbeatTime_) : 0;
    q.isValid = isSignalValid();
    q.linkUp = isLinkUp();
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
