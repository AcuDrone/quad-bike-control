#ifndef TELEMETRY_MANAGER_H
#define TELEMETRY_MANAGER_H

#include <Arduino.h>
#include "Constants.h"
#include "VehicleController.h"
#include "EncoderCounter.h"
#include "WebPortal.h"
#include "SBusInput.h"

/**
 * @brief Telemetry collection and broadcasting manager
 *
 * Collects vehicle telemetry data from various sources and broadcasts
 * to connected clients via WebSocket. Supports multiple output channels
 * (web portal, logging, CAN bus, etc.)
 */
class TelemetryManager {
public:
    TelemetryManager(VehicleController& vehicleController,
                     EncoderCounter& transmissionEncoder,
                     WebPortal& webPortal,
                     SBusInput& sbusInput);

    /**
     * @brief Update telemetry broadcast - call every loop iteration
     *
     * Collects telemetry data and broadcasts to all active channels
     * (rate-limited to configured interval)
     */
    void update();

    /**
     * @brief Set telemetry broadcast interval
     * @param intervalMs Interval in milliseconds (default: 200ms = 5Hz)
     */
    void setInterval(uint32_t intervalMs) { broadcastInterval_ = intervalMs; }

    /**
     * @brief Get current broadcast interval
     * @return Interval in milliseconds
     */
    uint32_t getInterval() const { return broadcastInterval_; }

    /**
     * @brief Force immediate telemetry broadcast (bypasses rate limit)
     */
    void forceBroadcast();

    /**
     * @brief Determine current input source based on priority: SBUS > WEB > FAILSAFE
     * @return Current input source
     */
    InputSource determineInputSource();

private:
    // Component references
    VehicleController& vehicleController_;
    EncoderCounter& transmissionEncoder_;
    WebPortal& webPortal_;
    SBusInput& sbusInput_;

    // Timing
    uint32_t lastBroadcast_;
    uint32_t broadcastInterval_;

    /**
     * @brief Collect telemetry data from all sources
     * @return Telemetry structure
     */
    WebPortal::Telemetry collectTelemetry();

    /**
     * @brief Broadcast telemetry to web portal
     * @param telemetry Telemetry data to broadcast
     */
    void broadcastToWeb(const WebPortal::Telemetry& telemetry);

    // Future expansion points:
    // void broadcastToCAN(const Telemetry& telemetry);
    // void logToSD(const Telemetry& telemetry);
    // void sendToSerial(const Telemetry& telemetry);
};

/**
 * @brief Get input source name as string
 * @param source Input source enum
 * @return Input source name string
 */
inline const char* getInputSourceName(InputSource source) {
    switch (source) {
        case InputSource::SBUS: return INPUT_SOURCE_NAME_SBUS;
        case InputSource::WEB: return INPUT_SOURCE_NAME_WEB;
        case InputSource::FAILSAFE: return INPUT_SOURCE_NAME_FAILSAFE;
        default: return "UNKNOWN";
    }
}

#endif // TELEMETRY_MANAGER_H
