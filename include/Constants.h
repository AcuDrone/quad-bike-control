#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// ============================================================================
// GPIO PIN ASSIGNMENTS
// ============================================================================

// MAVLink telemetry link (UART to Pixhawk TELEM2)
#define PIN_MAVLINK_RX      GPIO_NUM_8   // UART1 RX (non-inverted)
#define PIN_MAVLINK_TX      GPIO_NUM_15  // UART1 TX (non-inverted) — confirmed free; do NOT use GPIO 9 (PIN_TRANS_SERVO)
#define MAVLINK_UART_NUM    UART_NUM_1   // UART1 for MAVLink
#define MAVLINK_BAUD_RATE   115200       // Pixhawk TELEM2 baud

// Transmission Servo (HappyModel Super400 Plus)
#define PIN_TRANS_SERVO     GPIO_NUM_9   // LEDC Channel 2

// Steering Actuator (BTS7960 — full speed, no PWM needed, uses GPIO digital write)
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_STEER_RPWM      GPIO_NUM_17    // Digital out — move right
#define PIN_STEER_LPWM      GPIO_NUM_18   // Digital out — move left

// Steering Hall Sensor (Incremental Encoder - Quadrature)
#define PIN_STEER_ENCODER_A GPIO_NUM_42   // Hall sensor channel A (PCNT)
#define PIN_STEER_ENCODER_B GPIO_NUM_41      // Hall sensor channel B (PCNT)
#define PCNT_UNIT_STEER     1             // PCNT unit ID for steering encoder

// Throttle Servo (PWM via LEDC)
#define PIN_THROTTLE_PWM    GPIO_NUM_3   // LEDC Channel 1
#define LEDC_CH_THROTTLE    1

// Brake Linear Actuator (BTS7960)
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_BRAKE_RPWM      GPIO_NUM_7   // LEDC Channel 4
#define PIN_BRAKE_LPWM      GPIO_NUM_6   // LEDC Channel 5
#define LEDC_CH_BRAKE_RPWM  4
#define LEDC_CH_BRAKE_LPWM  5

// ============================================================================
// FUTURE EXPANSION - RESERVED GPIO PINS
// ============================================================================

// SPI CAN Controller (for vehicle CAN bus communication)
#define PIN_CAN_MOSI        GPIO_NUM_11   // SPI MOSI
#define PIN_CAN_MISO        GPIO_NUM_13   // SPI MISO
#define PIN_CAN_SCK         GPIO_NUM_12  // SPI SCK
#define PIN_CAN_CS          GPIO_NUM_10  // SPI Chip Select

#define PIN_RELAY1      GPIO_NUM_36
#define PIN_RELAY2      GPIO_NUM_37
#define PIN_RELAY3      GPIO_NUM_38
#define PIN_WHEEL_LOCK  GPIO_NUM_39   // Relay 4 — front-wheel lock (also JTAG MTCK; JTAG unused here)

#define PIN_GEAR_REVERSE  GPIO_NUM_19
#define PIN_GEAR_NEUTRAL  GPIO_NUM_20
#define PIN_GEAR_LOW      GPIO_NUM_21
#define PIN_GEAR_HIGH     GPIO_NUM_47
#define PIN_BRAKE_SENSOR  GPIO_NUM_14

///////

// Cranking Parameters
#define CRANKING_TIMEOUT           2000  // ms - maximum cranking duration
#define ACC_PRECRANK_DWELL_MS      2000  // ms - ignition/ECU line (R1) must be powered this long before the starter (R2) engages
#define ENGINE_RUNNING_RPM_THRESHOLD 1500  // RPM - engine considered running above this

// ============================================================================
// MAVLINK / COMMAND CHANNEL CONFIGURATION
// ============================================================================

// Servo-output channel assignments (SERVO_OUTPUT_RAW servoN_raw indices, 1-16).
// Mirrors the function map previously used for S-bus. ArduPilot SERVOn_FUNCTION
// outputs must align with these indices.
struct ServoChannelConfig {
    static constexpr uint8_t STEERING = 1;      // Steering (-100% to +100%)
    static constexpr uint8_t THROTTLE = 2;      // Throttle/Brake combined (above center=throttle, below=brake)
    static constexpr uint8_t TRANSMISSION = 3;  // Gear selector (3 positions: R/N/L)
    static constexpr uint8_t IGNITION = 4;      // Ignition state (OFF/ACC/IGNITION)
    static constexpr uint8_t FRONT_LIGHT = 6;   // Front light (on/off)
    static constexpr uint8_t WHEEL_LOCK  = 7;   // Front-wheel lock (on/off)
};

// MAVLink transport / link parameters
#define MAVLINK_SERVO_OUTPUT_RATE_HZ  25     // Requested SERVO_OUTPUT_RAW stream rate
#define MAVLINK_STREAM_REREQUEST_MS   3000   // ms between SET_MESSAGE_INTERVAL retries until the stream is healthy
#define MAVLINK_STREAM_MIN_RATE_HZ    10.0f  // re-request the stream while command rate is below this
#define MAVLINK_CMD_TIMEOUT_MS        500    // ms without a command frame before fail-safe
#define MAVLINK_HEARTBEAT_TIMEOUT_MS  3000   // ms without an autopilot heartbeat before link is "down"
#define MAVLINK_HEARTBEAT_TX_MS       1000   // ms between outbound HEARTBEAT messages (1 Hz)
#define MAVLINK_REPORT_TX_MS          200    // ms between outbound engine/state reports (5 Hz)
#define MAVLINK_STATUSTEXT_MIN_MS     250    // ms minimum spacing between STATUSTEXT messages

// ESP32 MAVLink identity (distinct component on the vehicle's system)
#define MAVLINK_SYSTEM_ID             1      // Same system as the autopilot
#define MAVLINK_COMPONENT_ID          25     // MAV_COMP_ID_USER1 (peripheral component)

// Command channel value range (microseconds) — SERVO_OUTPUT_RAW carries µs directly
#define RC_US_MIN     1000   // Minimum command microseconds
#define RC_US_MAX     2000   // Maximum command microseconds
#define RC_US_CENTER  1500   // Center point (split between throttle and brake)

// Deadband Configuration
#define RC_STEERING_DEADBAND  2.0f   // % center deadband for steering
#define RC_THROTTLE_DEADBAND  2.0f   // % idle deadband for throttle

// Gear Selection Ranges (in microseconds) - 3-position switch: R/N/L
#define RC_GEAR_REVERSE_MIN   880
#define RC_GEAR_REVERSE_MAX   1200
#define RC_GEAR_NEUTRAL_MIN   1201
#define RC_GEAR_NEUTRAL_MAX   1520
#define RC_GEAR_LOW_MIN       1521
#define RC_GEAR_LOW_MAX       2160

// Ignition State Ranges (in microseconds) - 3-position switch: OFF/ACC/IGNITION
// Note: IGNITION automatically triggers cranking (auto-stops when engine starts)
#define RC_IGNITION_OFF_MIN   880
#define RC_IGNITION_OFF_MAX   1200
#define RC_IGNITION_ACC_MIN   1201
#define RC_IGNITION_ACC_MAX   1520
#define RC_IGNITION_ON_MIN    1521
#define RC_IGNITION_ON_MAX    2160

// Front Light Threshold (in microseconds)
#define RC_FRONT_LIGHT_THRESHOLD 1520  // >1520 = ON, <=1520 = OFF
#define RC_WHEEL_LOCK_THRESHOLD  1520  // >1520 = LOCKED, <=1520 = UNLOCKED

// Digital output states reported back to the GCS as a bitmask carried in
// EFI_STATUS.pt_compensation (float). MUST stay in sync with the MP plugin decode.
// bit2+ reserved for future digital outputs.
#define EFI_DIGITAL_FLAG_WHEEL_LOCK  0x01  // bit0 — front-wheel lock engaged
#define EFI_DIGITAL_FLAG_FRONT_LIGHT 0x02  // bit1 — front light on

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

// Servo PWM Parameters
#define SERVO_PWM_FREQ        50     // Hz (20ms period)

// Steering Actuator Parameters
// Center (straight-ahead) is runtime-calibratable via the web UI and stored in NVS
// ("steering"/"center"); STEER_DEFAULT_CENTER is only the first-boot fallback. The right
// travel limit is derived as 2*center (symmetric about center; left home = 0).
#define STEER_DEFAULT_CENTER      1775   // Encoder counts — default center position (from left home)
#define STEER_POSITION_TOLERANCE  15    // +/- encoder counts for position match
#define STEER_HOMING_TIMEOUT      30000 // ms - maximum time for auto-home
#define STEER_MOVE_TIMEOUT        15000 // ms - maximum time for any movement
#define STEER_STALL_THRESHOLD     3     // Encoder counts - stall detection threshold
#define STEER_STALL_TIMEOUT       500   // ms - time without encoder change = stall

// Throttle Servo Parameters
#define THROTTLE_SERVO_MIN_US    800   // Minimum servo pulse width (µs) — mechanical range floor / slider bound
#define THROTTLE_SERVO_MAX_US    2200  // Maximum servo pulse width (µs) — mechanical range ceiling / slider bound
// Throttle calibration: 0..100% maps onto [idle_us, full_us]. These endpoints are runtime-editable
// via the web calibration flow and persisted in NVS ("throttle" namespace); the values below are
// only the defaults used on first boot / when stored values are missing or invalid.
#define THROTTLE_DEFAULT_IDLE_US 901   // idle pulse width (previously 13° over the 800–2200µs range)
#define THROTTLE_DEFAULT_FULL_US 1344  // full-throttle pulse width (previously 70° over the 800–2200µs range)
#define THROTTLE_MIN_SPAN_US     100   // minimum accepted full_us - idle_us separation during calibration

// ============================================================================
// BTS7960 MOTOR DRIVER CONFIGURATION
// ============================================================================

// Motor PWM Parameters
#define MOTOR_PWM_FREQ        10000  // Hz (10kHz to avoid audible noise)

// Motor Speed Limits (for safety)
#define MOTOR_MAX_FORWARD     255    // Maximum forward speed
#define MOTOR_MAX_REVERSE     -255   // Maximum reverse speed

// ============================================================================
// TRANSMISSION SYSTEM CONFIGURATION
// ============================================================================

// Transmission Servo Parameters (HappyModel Super400 Plus @ 24V, 180° range)
#define LEDC_CH_TRANS_SERVO                 2
#define TRANS_SERVO_MIN_US                  800
#define TRANS_SERVO_MAX_US                  2200
#define TRANS_SERVO_MS_PER_PCT              35      // ms per 1% travel (180° @ 24V: 1110ms/100%)
#define TRANS_SERVO_SETTLE_MIN_MS           300     // minimum settle time for short moves
#define TRANS_GEAR_OVERSHOOT_R_PCT          4.5f
#define TRANS_GEAR_OVERSHOOT_N_PCT          1.0f
#define TRANS_GEAR_OVERSHOOT_L_PCT          5.0f
#define TRANS_GEAR_OVERSHOOT_H_PCT          5.0f

#define TRANS_OVERSHOOT_DWELL_MS            2000    // ms to hold at overshoot position before assuming gear engaged
#define TRANS_SEQUENCE_STEP_DWELL_MS        1000    // ms to dwell after a step completes before starting the next step

// Default gear positions (percent: 0.0 = 800µs, 100.0 = 2200µs)
#define TRANS_GEAR_DEFAULT_REVERSE_PCT      40.0f
#define TRANS_GEAR_DEFAULT_NEUTRAL_PCT      48.0f
#define TRANS_GEAR_DEFAULT_LOW_PCT          68.0f
#define TRANS_GEAR_DEFAULT_HIGH_PCT         58.0f

// Safety limits
#define TRANS_UNKNOWN_GEAR_THROTTLE_MAX     (float)5   // % - max throttle when physical gear UNKNOWN
#define TRANS_GEAR_CHECK_INTERVAL           500        // ms - physical gear verification period
#define TRANS_GEAR_READ_INTERVAL_MS         100        // ms - GPIO debounce cache for getPhysicalGear()
#define TRANS_GEAR_READ_RETRY_COUNT         3          // extra reads when all switches read inactive while servo is idle

// ============================================================================
// BRAKE SYSTEM CONFIGURATION
// ============================================================================

// Brake Movement Parameters
#define BRAKE_FULL_TRAVEL_TIME 1500  // ms - estimated time for 0-100% brake travel
#define BRAKE_TOLERANCE        1  // % - position tolerance for "at target"
#define BRAKE_SENSOR_OVERRUN_TIME 1000  // ms - continue moving after sensor triggers (full retraction)
#define BRAKE_HOLD_SPEED          30    // PWM (0-255) applied against spring return when at target position

// ============================================================================
// SERIAL DEBUG
// ============================================================================

#define SERIAL_BAUD_RATE      115200 // Serial monitor baud rate
#define DEBUG_ENABLED         true   // Default debug output state (runtime-toggleable via Debug utility or web portal)

// Feature-specific debug flags (code-only control, persisted to NVS)
// Two-tier logging: Master debug (DEBUG_ENABLED) AND feature flag must both be ON
// Features: TRANSMISSION, CAN, MAVLINK, SERVO, BRAKE, RELAY, WEB, VEHICLE, TELEMETRY
// Enable programmatically: Debug::setFeatureEnabled(DebugFeature::TRANSMISSION, true)
// Or via NVS: preferences.putBool("feat_trans", true) in "debug" namespace
#define DEBUG_FEATURE_DEFAULT_STATE  false  // All features default to OFF

// ============================================================================
// WiFi ACCESS POINT CONFIGURATION
// ============================================================================

#define WIFI_AP_SSID          "QuadBike-Control"  // WiFi AP SSID
#define WIFI_AP_PASSWORD      ""                  // No password (open network)
#define WIFI_AP_CHANNEL       1                   // WiFi channel (1-13)
#define WIFI_AP_MAX_CLIENTS   5                   // Maximum simultaneous clients
#define WIFI_AP_IP            IPAddress(192, 168, 4, 1)     // ESP32 IP address
#define WIFI_AP_GATEWAY       IPAddress(192, 168, 4, 1)     // Gateway IP
#define WIFI_AP_SUBNET        IPAddress(255, 255, 255, 0)   // Subnet mask

// ============================================================================
// WEB SERVER CONFIGURATION
// ============================================================================

#define WEB_SERVER_PORT       80                  // HTTP server port
#define WEBSOCKET_PATH        "/ws"               // WebSocket endpoint path
#define TELEMETRY_INTERVAL    200                 // ms - telemetry broadcast interval (5 Hz)
#define WEB_COMMAND_TIMEOUT   10000               // ms - web control session timeout

// ============================================================================
// OTA UPDATE CONFIGURATION
// ============================================================================

#define OTA_HOSTNAME          "quadbike-control"  // OTA hostname for identification
#define OTA_PASSWORD          ""                  // OTA password (empty = no password)

// ============================================================================
// INPUT SOURCE PRIORITY
// ============================================================================

// Input source priority: MAVLINK > WEB > FAILSAFE
enum class InputSource {
    MAVLINK,    // MAVLink command stream active (highest priority)
    WEB,        // Web portal control active
    FAILSAFE    // No control source active (safe state)
};

// Input source names for telemetry/debugging
#define INPUT_SOURCE_NAME_MAVLINK   "MAVLINK"
#define INPUT_SOURCE_NAME_WEB       "WEB"
#define INPUT_SOURCE_NAME_FAILSAFE  "FAILSAFE"

// ============================================================================
// CAN CONTROLLER CONFIGURATION
// ============================================================================

// CAN Polling Intervals
#define CAN_POLL_INTERVAL_RPM           500   // ms - RPM polling rate (normal)
#define CAN_POLL_INTERVAL_RPM_BOOST     50    // ms - RPM polling rate during gear change
#define CAN_POLL_INTERVAL_TEMP          2000  // ms - Temperature polling rate

// CAN Timeouts
#define CAN_RESPONSE_TIMEOUT      200   // ms - OBD-II response timeout (non-blocking, healthy ECU responds in ~50ms)
#define CAN_DATA_STALE_TIMEOUT    5000  // ms - Mark data invalid if not updated
#define CAN_RETRY_ATTEMPTS        3     // Number of retry attempts on error

// Transmission Safety (CAN-based)
#define TRANS_SPEED_INTERLOCK_THRESHOLD  5     // km/h - Block gear changes above this speed
#define TRANS_CAN_TIMEOUT                5000  // ms - Allow gear change if CAN fails this long

// PID-Controlled RPM Boost During Gear Changes
#define TRANS_GEAR_BOOST_TARGET_RPM      2100  // RPM target to hold during gear change
#define TRANS_GEAR_BOOST_MAX_PCT         27.0f // % - max throttle PID can command (safety cap)
#define TRANS_GEAR_BOOST_TIMEOUT         5000  // ms - max boost duration (safety escape)
#define TRANS_GEAR_BOOST_PID_KP          0.015f // proportional gain (tunable)
#define TRANS_GEAR_BOOST_PID_KI          0.01f // integral gain (tunable)
#define TRANS_GEAR_BOOST_PID_KD          0.0f  // derivative gain (0 = disabled; noisy on RPM signal)
#define TRANS_GEAR_BOOST_SLEW_RATE_US    16    // µs per CAN update

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

// Firmware version string (semantic versioning: MAJOR.MINOR.PATCH)
// Update this constant when releasing new firmware versions
#define FIRMWARE_VERSION "1.0.4"

#endif // CONSTANTS_H
