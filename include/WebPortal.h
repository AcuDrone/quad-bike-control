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
        int32_t relay;     // Relay number for set_relay_test (JSON "relay", 1..8); 0 if absent
        bool hasCommand;   // True if valid command received
    };

    /**
     * Vehicle telemetry structure
     */
    struct Telemetry {
        uint32_t timestamp;       // System time in milliseconds
        String gear;              // Current gear: "R", "N", "L", "H"
        int8_t gear_code;         // Raw 3-bit gear code [bit0 bit1 bit6] 0..7, -1 = inputs stale
        const char* gear_sensed;  // gear_code decoded now (no retry cache): "R"/"N"/"H"/"L"/"?"
        int8_t gear_code_r;       // Active gear code table (web-calibrated or Constants.h default), -1 = unset
        int8_t gear_code_n;
        int8_t gear_code_h;
        int8_t gear_code_l;
        uint8_t gear_codes_custom; // Bitmask of table entries differing from the default: bit0 R, bit1 N, bit2 H, bit3 L
        float hall_position;      // Transmission servo position (0.0–100.0 %)
        float brake_pct;          // Brake position ESTIMATE (open loop, 0-100 %)
        uint16_t brake_travel_ms; // Brake full travel time in use (ms)
        uint8_t brake_hold_pwm;   // Brake hold drive in use (0-100 PWM, 0 = no push)
        bool brake_released;      // Released endstop (In5) asserted in a valid expander snapshot
        uint16_t throttle_us;         // Throttle servo pulse width (µs)
        uint16_t throttle_idle_us;    // Calibrated idle endpoint (µs)
        uint16_t throttle_full_us;    // Calibrated full-throttle endpoint (µs)
        bool throttle_calibrating;    // True while a throttle calibration session is active
        int steering_pct;         // Steering percentage (-100 to +100)
        // Steering angle (Bosch LWS on CAN). Degrees, firmware sign (+ = right), relative to
        // the sensor's stored zero. Every "_valid"/"_set"/"_seen" false is serialised as JSON
        // null — never NaN, never a sentinel number.
        float steer_angle;        // Current angle, deg (1 decimal), valid only if steer_angle_valid
        bool steer_angle_valid;   // Fresh accepted frame with a valid angle
        float steer_rate;         // Signed rotation rate, deg/s (+ = turning right), valid only if steer_rate_valid
        bool steer_rate_valid;
        float steer_left;         // Stored left limit, deg, valid only if steer_left_set
        bool steer_left_set;
        float steer_right;        // Stored right limit, deg, valid only if steer_right_set
        bool steer_right_set;
        bool steer_lws_seen;      // A frame has been accepted since boot (else the four below are null)
        bool steer_lws_ok;        // Last accepted frame: sensor OK bit
        bool steer_lws_cal;       // Last accepted frame: calibrated bit
        bool steer_lws_adj;       // Last accepted frame: adjusted bit
        uint8_t steer_lws_fault_code; // Last accepted frame: fault code 0-15 (9 = range latch)
        const char* steer_sensor_fault; // none | zeroing | can_down | timeout | range_latched | sensor_fault | not_calibrated | angle_invalid
        const char* steer_mode;   // position | rate | blocked (static token, SteerRate.h)
        bool steer_sensor_ok;     // Sensor healthy (token none for STEER_LWS_RECOVER_FRAMES frames)
        bool steer_calibrated;    // True when both limits are stored and valid
        bool steer_range_warning; // Either stored limit beyond ±STEER_LWS_SAFE_RANGE (near the ~600° latch)
        bool steer_sensor_inv;    // Sensor-direction flag: true = sensor mounted mirrored (inferred at limit capture)
        uint16_t steer_lws_fps;   // Accepted frames in the last second (~100 when healthy)
        uint32_t steer_lws_crc_err; // Cumulative checksum errors since boot
        uint32_t steer_lws_cnt_err; // Cumulative repeated-counter errors since boot
        uint32_t steer_lws_miss;    // Cumulative missed frames (counter jumps) since boot
        const char* steer_zero_state; // idle | running | ok | failed
        const char* steer_zero_error; // Reason token when failed, nullptr = null

        // Steering VESC driver telemetry
        float steer_motor_current; // Steering motor current (A) from the VESC
        float steer_fet_temp;      // VESC FET temperature (°C)
        uint8_t steer_vesc_fault;  // VESC fault code (0 = no fault)
        bool steer_driver_ok;      // True if the VESC telemetry link is healthy
        float steer_vin;           // VESC input voltage (V) — meaningless while !steer_driver_ok
        bool steer_vin_low;        // steer_driver_ok && steer_vin < STEER_VIN_LOW_V (warning only)
        // Steering park correction (settings + last result, RAM only — SteeringController)
        bool steer_park_enabled;
        uint8_t steer_park_d[3];   // rung duty, rungs 1..3
        uint16_t steer_park_t[3];  // rung length (ms), rungs 1..3
        float steer_park_tol;      // done tolerance (deg) — 1 decimal in JSON
        uint8_t steer_park_max;    // max pulses per parking
        const char* steer_park_last_outcome; // none | done | gave_up | cancelled (static token)
        uint8_t steer_park_last_pulses;
        float steer_park_last_residual;      // deg, signed (target − position) — null for "none"
        uint8_t steer_park_last_duty;        // last pulse duty × ms
        uint16_t steer_park_last_ms;
        String input_source;      // Current input source: "MAVLINK", "WEB", "FAILSAFE"
        bool mav_active;          // True if MAVLink command stream is valid
        bool web_control;         // True if latched web-control override is engaged

        // Hall speed sensor — emitted independently of CAN health (not part of the
        // CAN-gated block below), because it is a wholly separate physical source.
        float vehicle_speed_ms;   // Vehicle speed (m/s) from the hall sensor — serialised as km/h
        bool speed_valid;         // Sensor health: false = reading unknown, NOT "stopped"
        uint16_t speed_ppr;       // Calibration: pulses per revolution
        uint32_t speed_pulse_total; // Running pulse total since boot (web ppr measurement; counts up either way)
        float speed_circ_mm;      // Calibration: wheel circumference (mm)
        float speed_limit_ms;     // Enforced ceiling (m/s) from SPEED_MAX, 0 = no limit — km/h in JSON
        float speed_limit_ceil;   // Throttle ceiling the limiter is applying (taper + trim, %, 100 = inactive)
        float speed_limit_floor;  // Limiter throttle floor in use (%, 0 = calibrated idle)
        float speed_limit_slew;   // Limiter ceiling slew in use (%/s, both directions)
        uint16_t throttle_open_slew; // Throttle opening rate limit in use (%/s, 0 = off)
        bool speed_limit_sat;     // Limiter saturated: ceiling at the floor, speed still above the limit
        bool speed_limit_sat_held; // speed_limit_sat, or within SPEED_LIMIT_SAT_UI_HOLD_MS after it cleared
        // Steering speed scaling — so "the steering feels weak" is diagnosable from the portal.
        float steer_scale;        // Applied scale after rate limiting, 0..1 (1 = not scaling)
        float steer_sca_base_ms;  // Base in use (m/s), 0 = none — serialised in m/s, not km/h
        uint8_t steer_sca_src;    // Base source: 0 = none, 1 = local NVS, 2 = MOT_SPD_SCA_BASE
        float steer_test_speed_ms; // Live bench override (m/s), 0 = none in force
        float odo_km;             // Total odometer (km) — always valid, NOT gated on speed_valid
        float trip_km;            // Resettable trip distance (km) — reset is a GCS action only
        float engine_hours;       // Engine hour meter (h) — always valid, NOT gated on can_status
        float engine_trip_hours;  // Trip engine hours (h) — same, zeroed by the GCS trip reset

        // Effective engine RPM + ECU tacho (NOT gated on CAN)
        uint16_t engine_rpm;      // EFFECTIVE engine RPM (0-16383), 0 when rpm_source is NONE
        const char* rpm_source;   // "TACHO" / "CAN" / "NONE" (static string)
        float tacho_hz;           // averaged tacho edge frequency, Hz (0 when timed out)
        float tacho_ppr;          // tacho pulses per revolution in effect
        uint16_t tacho_rpm;       // the tacho's own RPM (0 when timed out)
        // Tacho signal diagnostics (fw 1.5.10) — what the field tech reads instead of a scope
        char tacho_periods[64];   // last 8 raw accepted periods, ms 1 decimal, oldest first ("" = none)
        float tacho_min_ms;       // shortest / longest accepted period in the last 1 s window (0 = none)
        float tacho_max_ms;
        uint16_t tacho_acc;       // accepted edges per second (last 1 s window)
        uint16_t tacho_rej;       // rejected edges per second (pulse-width check + floor)
        uint32_t tacho_rej_total; // rejected edges since boot
        float tacho_hi_ms;        // fw 1.5.11: last high span / last validated low time, ms (0 = none)
        float tacho_lo_ms;
        float tacho_hi_min_ms;    // shortest / longest high span in the last 1 s window (0 = none)
        float tacho_hi_max_ms;

        // CAN bus vehicle data
        int8_t coolant_temp;      // Coolant temperature °C (-40 to +215)
        int8_t oil_temp;          // Oil temperature °C (-40 to +215)
        uint8_t throttle_position; // Throttle position % (0-100)
        uint8_t fuel_level;       // Fuel tank level % (0-100)
        uint8_t map_kpa;          // Manifold absolute pressure, kPa (0-255)
        int8_t intake_temp;       // Intake air temperature °C (-40 to +215)
        uint8_t engine_load;      // Calculated engine load % (0-100)
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
        bool wheel_lock_on;       // True if the lock is engaged (aztec: Relay_4+5+6; always false when not fitted)
        bool wheel_lock_fitted;   // True on aztec (RELAY_WHEEL_LOCK_FITTED); false = no relay behind the function
        uint8_t relay_test;       // Web relay test bits, bit0 = Relay_1 … bit7 = Relay_8

        // 12 V battery (X10 / GPIO9 sense; aztec has no boost board). Not gated on CAN.
        uint16_t module_voltage_mv; // CALIBRATED battery voltage, mV → JSON "ecu_voltage" (V)
        bool ecu_voltage_low;     // battery low (hysteresis 11.8/12.1 V, crank-suppressed), only with bat_v_valid
        float bat_v_raw;          // battery voltage before the user calibration factor, V
        float bat_v_cal;          // user calibration factor (1.0 = uncalibrated)
        bool bat_v_valid;         // something is wired to X10 (raw > BAT12_PRESENT_MIN_V)

        // Board I/O expander health
        bool io_input_fault;      // Opto-input expander U10 (0x1A, I2C2) unreachable/stale
        uint8_t io_raw;           // Raw U10 input port byte of the last good read
        bool io_relay_fault;      // Relay board (I2C2 via X15) not detected / unreachable / unverified
        // Relay board selection (all static strings, stored by pointer)
        const char* relay_board;       // active board "pca9557" / "xl9535" / "none"
        uint8_t relay_board_addr;      // active board 7-bit address; 0 → JSON null (relay_board "none")
        const char* relay_board_cfg;   // stored setting "auto" / "pca9557" / "xl9535"
        bool relay_board_reboot;       // stored setting differs from the one used at boot
        const char* relay_board_error; // nullptr (JSON null) or both_found / none_found /
                                       // multiple_xl9535 / cfg_mismatch / init_failed

        // Link recovery session totals + event journal size (change add-event-journal-and-link-recovery)
        uint32_t vesc_resync;     // VESC UART parser resyncs since boot
        uint32_t can_reinit;      // verified MCP2515 re-initializations of any cause since boot
        uint32_t i2c_unlock;      // I2C2 bus recoveries (Wire1 restarts) since boot
        int32_t journal_bytes;    // writes on: both journal files + unflushed bytes (-1 = cannot
                                  // write to flash); writes off: RAM tail bytes
        bool journal_fs;          // journal flash-write switch (NVS journal/fs, default off)

        // Aztec parking brake on X6 (change add-aztec-parking-brake). Tokens are static strings.
        const char* pbrake_state;      // "unknown" / "released" / "applied" / "applying" / "releasing"
        uint32_t pbrake_remaining_ms;  // ms left in the run (dwell + travel + margin), 0 when settled
        bool pbrake_thr_hold;          // the park brake holds the operator throttle at idle now
        const char* pbrake_da_wait;    // "none" / "ign_off" / "rearm" — why a throttle press did not release
        uint16_t pbrake_travel_ms;     // travel time in use, ms
        uint8_t pbrake_thr_pass_pct;   // throttle release point in use, % of travel_ms
        bool pbrake_invert;            // drive direction inverted

        // Firmware information
        String firmware_version;  // Firmware version string (e.g., "1.0.0")
        String board;             // Target board name (BOARD_NAME, e.g. "Control_v0")
        const char* wifi_ssid;    // broadcast SSID WIFI_AP_SSID-XXXX (static string, stored by pointer)
        char wifi_ip[16];         // WIFI_AP_IP as a dotted string, e.g. "192.168.10.1"

        // Gear default positions (user-configurable NVS values, 0.0–100.0 %)
        float gear_default_r;     // Default servo % for REVERSE
        float gear_default_n;     // Default servo % for NEUTRAL
        float gear_default_l;     // Default servo % for LOW
        float gear_default_h;     // Default servo % for HIGH

        // Boost PID configuration
        int32_t boost_target_rpm; // RPM target for gear boost PID
        bool boost_enabled;       // Gear-change RPM boost switch in use (NVS "boost"/enabled)

        // ECU capability probe results (emitted transiently while fresh)
        bool probe_present;                 // true when the probe object should be serialized
        CANController::ProbeResults probe;  // latest probe snapshot

        // CAN sniffer results: points into the CAN controller (which outlives the broadcast)
        // while running or fresh, nullptr otherwise. A pointer, not a copy: the table is
        // ~800 bytes and this struct lives on the loop stack.
        const CANController::SniffResults* sniff;
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

    /**
     * Broadcast AP SSID with the chip suffix (WIFI_AP_SSID-XXXX). Static storage, filled
     * once in begin(); valid for the program's lifetime (empty before begin()).
     */
    static const char* getApSsid() { return apSsid_; }

private:
    static char apSsid_[33];         // WIFI_AP_SSID-XXXX (DeviceId::NAME_CAP)
    static char otaHostname_[33];    // OTA_HOSTNAME-xxxx

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
