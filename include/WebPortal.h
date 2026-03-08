#ifndef WEBPORTAL_H
#define WEBPORTAL_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "Constants.h"

// Use SPIFFS for ESP32-C6 (LittleFS support may vary by framework version)
#define FILESYSTEM SPIFFS

// Forward declarations for vehicle controllers
class ServoController;
class BTS7960Controller;
class TransmissionController;

/**
 * WebPortal class - Manages WiFi AP, web server, WebSocket telemetry, and OTA updates
 *
 * This class provides a web-based interface for:
 * - Real-time telemetry display (gear, sensors, actuators)
 * - Manual control when S-bus is inactive
 * - Over-the-air firmware updates
 */
class WebPortal {
public:
    /**
     * Web command structure for JSON parsing
     */
    struct WebCommand {
        String cmd;        // Command type: "set_gear", "set_steering", "set_throttle"
        String strValue;   // String value (for gear: "R", "N", "L", "H")
        float floatValue;  // Float value (for steering: -100 to +100, throttle: 0 to 100)
        bool hasCommand;   // True if valid command received
    };

    /**
     * Vehicle telemetry structure
     */
    struct Telemetry {
        uint32_t timestamp;       // System time in milliseconds
        String gear;              // Current gear: "R", "N", "L", "H"
        int32_t hall_position;    // Transmission encoder position
        float brake_pct;          // Brake position percentage (0-100)
        int throttle_angle;       // Throttle servo angle (degrees)
        int steering_angle;       // Steering servo angle (degrees)
        String input_source;      // Current input source: "SBUS", "WEB", "FAILSAFE"
        bool sbus_active;         // True if S-bus signal is valid
    };

    WebPortal();
    ~WebPortal();

    /**
     * Initialize WiFi AP, web server, WebSocket, and OTA
     * @return true if initialization successful, false otherwise
     */
    bool begin();

    /**
     * Update loop - must be called periodically (e.g., in main loop)
     * Handles OTA updates and checks timeouts
     */
    void update();

    /**
     * Check if web control has active commands
     * @return true if web control commands received within timeout period
     */
    bool hasActiveControl();

    /**
     * Get the most recent web command (if any)
     * @return WebCommand structure (hasCommand = false if no command)
     */
    WebCommand getCommand();

    /**
     * Clear the current web command (after processing)
     */
    void clearCommand();

    /**
     * Broadcast telemetry data to all connected WebSocket clients
     * @param telemetry Telemetry structure with current vehicle state
     */
    void broadcastTelemetry(const Telemetry& telemetry);

    /**
     * Send response back to web client
     * @param success True if command succeeded, false if error
     * @param message Response message
     */
    void sendResponse(bool success, const String& message);

    /**
     * Get number of connected WebSocket clients
     * @return Number of active clients
     */
    uint8_t getClientCount();

    /**
     * Check if WiFi AP is active
     * @return true if AP is running
     */
    bool isAPActive();

    /**
     * Get WiFi AP IP address
     * @return IP address as string
     */
    String getAPIP();

private:
    AsyncWebServer server;           // Async HTTP web server
    AsyncWebSocket ws;               // WebSocket for real-time communication
    DNSServer dnsServer;             // DNS server for captive portal

    WebCommand currentCommand;       // Most recent web command
    uint32_t lastCommandTime;        // Time of last web command (for timeout)
    uint32_t lastTelemetryTime;      // Time of last telemetry broadcast

    bool otaInProgress;              // True during OTA update
    uint32_t lastActivityTime;       // Last client activity timestamp

    /**
     * Setup WiFi Access Point
     * @return true if AP started successfully
     */
    bool setupWiFiAP();

    /**
     * Setup DNS server for captive portal
     */
    void setupDNS();

    /**
     * Setup web server routes and handlers
     */
    void setupWebServer();

    /**
     * Setup WebSocket endpoint and event handlers
     */
    void setupWebSocket();

    /**
     * Setup OTA (Over-The-Air) update handlers
     */
    void setupOTA();

    /**
     * Mount LittleFS filesystem
     * @return true if filesystem mounted successfully
     */
    bool mountFilesystem();

    /**
     * WebSocket event handler
     * @param server WebSocket server instance
     * @param client WebSocket client instance
     * @param type Event type
     * @param arg Event argument
     * @param data Event data
     * @param len Data length
     */
    void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                          AwsEventType type, void* arg, uint8_t* data, size_t len);

    /**
     * Parse JSON command from WebSocket message
     * @param data Message data
     * @param len Message length
     * @return true if command parsed successfully
     */
    bool parseWebCommand(uint8_t* data, size_t len);

    /**
     * Validate command based on input source priority
     * @param cmd Command to validate
     * @param inputSource Current input source
     * @return true if command is allowed
     */
    bool validateCommand(const WebCommand& cmd, InputSource inputSource);

    /**
     * Create JSON telemetry message
     * @param telemetry Telemetry data
     * @return JSON string
     */
    String createTelemetryJSON(const Telemetry& telemetry);

    /**
     * Create JSON response message
     * @param success Success flag
     * @param message Response message
     * @return JSON string
     */
    String createResponseJSON(bool success, const String& message);
};

#endif // WEBPORTAL_H
