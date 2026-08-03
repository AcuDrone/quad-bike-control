#ifndef WEBPORTAL_H
#define WEBPORTAL_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <DNSServer.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "Constants.h"
#include "CANController.h"

#define FILESYSTEM LittleFS

// Forward declarations for vehicle controllers
class ServoController;
class BTS7960Controller;
class TransmissionController;

/**
 * WebPortal class - Manages WiFi AP, web server, WebSocket telemetry, and OTA updates
 *
 * This class provides a web-based interface for:
 * - Real-time telemetry display (gear, sensors, actuators)
 * - Manual control when the MAVLink command stream is inactive
 * - Over-the-air firmware updates
 */
class WebPortal {
public:
    /**
     * Web command structure for JSON parsing
     */
    struct WebCommand {
        String cmd;        // Command type: "set_gear", "set_steering", "set_throttle", "set_ignition", "set_light"
        String strValue;   // String value (for gear: "R", "N", "L", "H"; ignition: "OFF", "ACC", "IGNITION", "START")
        float floatValue;  // Float value (for steering: -100 to +100, throttle: 0 to 100, brake: 0 to 100)
        bool boolValue;    // Bool value (for light: true/false)
        bool hasCommand;   // True if valid command received
    };

    /**
     * Vehicle telemetry structure
     */
    struct Telemetry {
        uint32_t timestamp;       // System time in milliseconds
        String gear;              // Current gear: "R", "N", "L", "H"
        float hall_position;      // Transmission servo position (0.0–100.0 %)
        float brake_pct;          // Brake position percentage (0-100)
        uint16_t throttle_us;         // Throttle servo pulse width (µs)
        uint16_t throttle_idle_us;    // Calibrated idle endpoint (µs)
        uint16_t throttle_full_us;    // Calibrated full-throttle endpoint (µs)
        bool throttle_calibrating;    // True while a throttle calibration session is active
        int steering_pct;         // Steering percentage (-100 to +100)
        int32_t steer_center;     // Calibrated steering center (AS5600 raw counts, -1 if unset)
        int32_t steer_left;       // Calibrated left limit (AS5600 raw counts, -1 if unset)
        int32_t steer_right;      // Calibrated right limit (AS5600 raw counts, -1 if unset)
        int32_t steer_position;   // Current AS5600 raw angle (0-4095)
        bool steer_sensor_ok;     // True if the AS5600 reads OK and the magnet is detected
        bool steer_calibrated;    // True when center + both limits are valid

        // Steering VESC driver telemetry
        float steer_motor_current; // Steering motor current (A) from the VESC
        float steer_fet_temp;      // VESC FET temperature (°C)
        uint8_t steer_vesc_fault;  // VESC fault code (0 = no fault)
        bool steer_driver_ok;      // True if the VESC telemetry link is healthy
        String input_source;      // Current input source: "MAVLINK", "WEB", "FAILSAFE"
        bool mav_active;          // True if MAVLink command stream is valid
        bool web_control;         // True if latched web-control override is engaged

        // CAN bus vehicle data
        uint16_t engine_rpm;      // Engine RPM (0-16383)
        uint8_t vehicle_speed;    // Vehicle speed km/h (0-255)
        int8_t coolant_temp;      // Coolant temperature °C (-40 to +215)
        int8_t oil_temp;          // Oil temperature °C (-40 to +215)
        uint8_t throttle_position; // Throttle position % (0-100)
        uint8_t fuel_level;       // Fuel tank level % (0-100)
        uint8_t map_kpa;          // Manifold absolute pressure, kPa (0-255)
        String can_status;        // CAN status: "connected", "disconnected"

        // MAVLink command channel + link data
        uint16_t mav_channels[16]; // Decoded command channel values in microseconds
        float mav_cmd_rate;        // SERVO_OUTPUT_RAW command rate in Hz
        uint32_t mav_link_age;     // Time since last command frame (ms)
        uint32_t mav_heartbeat_age; // Time since last autopilot heartbeat (ms)

        // Gear transition state
        bool gear_switching;      // True if gear change in progress

        // Ignition and lighting state
        String ignition_state;    // Ignition state: "OFF", "ACC", "IGNITION", "CRANKING"
        bool is_cranking;         // True if starter motor is cranking
        bool front_light_on;      // True if front light is on
        bool wheel_lock_on;       // True if front-wheel lock is engaged

        // Firmware information
        String firmware_version;  // Firmware version string (e.g., "1.0.0")

        // Gear default positions (user-configurable NVS values, 0.0–100.0 %)
        float gear_default_r;     // Default servo % for REVERSE
        float gear_default_n;     // Default servo % for NEUTRAL
        float gear_default_l;     // Default servo % for LOW
        float gear_default_h;     // Default servo % for HIGH

        // Boost PID configuration
        int32_t boost_target_rpm; // RPM target for gear boost PID

        // ECU capability probe results (emitted transiently while fresh)
        bool probe_present;                 // true when the probe object should be serialized
        CANController::ProbeResults probe;  // latest probe snapshot
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
    int otaUpdateType;               // Current OTA update type (U_FLASH or U_SPIFFS)
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
