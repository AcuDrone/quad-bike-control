#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// ============================================================================
// GPIO PIN ASSIGNMENTS
// ============================================================================

// S-bus Input (UART)
#define PIN_SBUS_RX         GPIO_NUM_20  // UART1 RX with inverted signal
#define SBUS_UART_NUM       UART_NUM_1   // UART1 for S-bus

// Transmission Hall Sensor (Incremental Encoder - Quadrature)
#define PIN_TRANS_ENCODER_A GPIO_NUM_3   // Hall sensor channel A (PCNT) - SWAPPED
#define PIN_TRANS_ENCODER_B GPIO_NUM_2   // Hall sensor channel B (PCNT) - SWAPPED
#define PCNT_UNIT_TRANS     0            // PCNT unit ID for transmission encoder

// Transmission Linear Actuator (BTS7960)
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_TRANS_RPWM      GPIO_NUM_4   // LEDC Channel 2
#define PIN_TRANS_LPWM      GPIO_NUM_5   // LEDC Channel 3
#define LEDC_CH_TRANS_RPWM  2
#define LEDC_CH_TRANS_LPWM  3

// Steering Actuator (BTS7960 — full speed, no PWM needed, uses GPIO digital write)
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_STEER_RPWM      GPIO_NUM_9    // Digital out — move right
#define PIN_STEER_LPWM      GPIO_NUM_10   // Digital out — move left

// Steering Hall Sensor (Incremental Encoder - Quadrature)
#define PIN_STEER_ENCODER_A GPIO_NUM_11   // Hall sensor channel A (PCNT)
#define PIN_STEER_ENCODER_B GPIO_NUM_12   // Hall sensor channel B (PCNT)
#define PCNT_UNIT_STEER     1             // PCNT unit ID for steering encoder

// Throttle Servo (PWM via LEDC)
#define PIN_THROTTLE_PWM    GPIO_NUM_1   // LEDC Channel 1
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
#define PIN_CAN_MOSI        GPIO_NUM_8   // SPI MOSI (using freed NeoPixel pin)
#define PIN_CAN_MISO        GPIO_NUM_0   // SPI MISO (using freed boot pin - safe during runtime)
#define PIN_CAN_SCK         GPIO_NUM_23  // SPI SCK
#define PIN_CAN_CS          GPIO_NUM_22  // SPI Chip Select

// ============================================================================
// MCP23017 I2C GPIO EXPANDER
// ============================================================================

// I2C pins for MCP23017 (TODO: assign final GPIO pins)
#define PIN_I2C_SDA         GPIO_NUM_18  // I2C SDA
#define PIN_I2C_SCL         GPIO_NUM_19  // I2C SCL
#define MCP23017_ADDRESS    0x20         // Default address (A0=A1=A2=GND)

// MCP23017 Port A - Outputs (relays)
#define MCP_PIN_RELAY1      0   // GPA0 - Relay 1 (main power control)
#define MCP_PIN_RELAY2      1   // GPA1 - Relay 2 (accessory power)
#define MCP_PIN_RELAY3      2   // GPA2 - Relay 3 (front light / safety)

// MCP23017 Port B - Inputs (sensors, active-low with pull-ups)
#define MCP_PIN_GEAR_REVERSE  8   // GPB0 - Gear selector REVERSE
#define MCP_PIN_GEAR_NEUTRAL  9   // GPB1 - Gear selector NEUTRAL
#define MCP_PIN_GEAR_LOW      10  // GPB2 - Gear selector LOW
#define MCP_PIN_GEAR_HIGH     11  // GPB3 - Gear selector HIGH
#define MCP_PIN_BRAKE_SENSOR  12  // GPB4 - Brake position sensor

// Bit masks for bulk Port B reading
#define MCP_PORTB_GEAR_MASK   0x0F  // GPB0-GPB3 (bits 0-3)
#define MCP_PORTB_BRAKE_BIT   4     // GPB4 (bit 4)

// Cranking Parameters
#define CRANKING_TIMEOUT           5000  // ms - maximum cranking duration
#define ENGINE_RUNNING_RPM_THRESHOLD 1100  // RPM - engine considered running above this

// Note: GPIO 14 does not exist on ESP32-C6 (valid pins are 0-23 excluding 14)
// Note: GPIO 0 (boot pin) and GPIO 8 (NeoPixel) are now used for CAN controller (safe during runtime)
// Note: GPIO 18 (SDA) and GPIO 19 (SCL) are now used for MCP23017 I2C bus

// ============================================================================
// S-BUS CONFIGURATION
// ============================================================================

// S-bus Channel Assignments (1-16)
struct SBusChannelConfig {
    static constexpr uint8_t STEERING = 1;      // Channel 1: Steering (-100% to +100%)
    static constexpr uint8_t THROTTLE = 2;      // Channel 2: Throttle/Brake combined (above center=throttle, below=brake)
    static constexpr uint8_t TRANSMISSION = 3;  // Channel 3: Gear selector (3 positions: R/N/L)
    static constexpr uint8_t IGNITION = 4;      // Channel 4: Ignition state (OFF/ACC/IGNITION)
    static constexpr uint8_t FRONT_LIGHT = 5;   // Channel 5: Front light (on/off)
};

// S-bus Protocol Parameters
#define SBUS_RAW_MIN        201   // Minimum S-bus raw value (ArduPilot: (1000-875)*1600/1000+1)
#define SBUS_RAW_MAX        1801  // Maximum S-bus raw value (ArduPilot: (2000-875)*1600/1000+1)
#define SBUS_SIGNAL_TIMEOUT   500   // ms before fail-safe activates
#define SBUS_MIN_RECOVERY_FRAMES 3  // Consecutive good frames to exit fail-safe

// S-bus to microseconds conversion (for compatibility)
#define SBUS_US_MIN           1000   // Microseconds equivalent
#define SBUS_US_MAX           2000  // Microseconds equivalent
#define SBUS_US_CENTER        1500  // Microseconds center point (updated to match SBUS standard)

// Deadband Configuration
#define SBUS_STEERING_DEADBAND    2.0f   // % center deadband for steering
#define SBUS_THROTTLE_DEADBAND    2.0f   // % idle deadband for throttle

// Gear Selection Ranges (in microseconds) - 3-position switch: R/N/L
#define SBUS_GEAR_REVERSE_MIN     880
#define SBUS_GEAR_REVERSE_MAX     1200
#define SBUS_GEAR_NEUTRAL_MIN     1201
#define SBUS_GEAR_NEUTRAL_MAX     1520
#define SBUS_GEAR_LOW_MIN         1521
#define SBUS_GEAR_LOW_MAX         2160

// Ignition State Ranges (in microseconds) - 3-position switch: OFF/ACC/IGNITION
// Note: IGNITION automatically triggers cranking (max 5s, auto-stops when engine starts)
#define SBUS_IGNITION_OFF_MIN     880
#define SBUS_IGNITION_OFF_MAX     1200
#define SBUS_IGNITION_ACC_MIN     1201
#define SBUS_IGNITION_ACC_MAX     1520
#define SBUS_IGNITION_ON_MIN      1521
#define SBUS_IGNITION_ON_MAX      2160

// Front Light Threshold (in microseconds)
#define SBUS_FRONT_LIGHT_THRESHOLD 1520  // >1520 = ON, <=1520 = OFF

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

// Servo PWM Parameters
#define SERVO_PWM_FREQ        50     // Hz (20ms period)
#define SERVO_PWM_RESOLUTION  16     // bits (0-65535)

// Steering Actuator Parameters
#define STEER_CENTER_POSITION     500   // Encoder counts — center position (from left home)
#define STEER_RIGHT_LIMIT         1000  // Encoder counts — maximum right travel (from left home)
#define STEER_POSITION_TOLERANCE  15    // +/- encoder counts for position match
#define STEER_HOMING_TIMEOUT      30000 // ms - maximum time for auto-home
#define STEER_MOVE_TIMEOUT        15000 // ms - maximum time for any movement
#define STEER_STALL_THRESHOLD     3     // Encoder counts - stall detection threshold
#define STEER_STALL_TIMEOUT       500   // ms - time without encoder change = stall

// Throttle Servo Parameters
#define THROTTLE_SERVO_MIN_US    800   // Minimum pulse width (microseconds)
#define THROTTLE_SERVO_MAX_US    2200  // Maximum pulse width (microseconds)
#define THROTTLE_MIN_ANGLE       23    // degrees (idle) - measured 13% of full range
#define THROTTLE_MAX_ANGLE       67    // degrees (full throttle) - measured 37% of full range
#define THROTTLE_IDLE_ANGLE      23    // degrees (idle position)

// ============================================================================
// BTS7960 MOTOR DRIVER CONFIGURATION
// ============================================================================

// Motor PWM Parameters
#define MOTOR_PWM_FREQ        10000  // Hz (10kHz to avoid audible noise)
#define MOTOR_PWM_RESOLUTION  8      // bits (0-255)
#define MOTOR_MIN_SPEED       0      // Minimum PWM value
#define MOTOR_MAX_SPEED       255    // Maximum PWM value

// Motor Speed Limits (for safety)
#define MOTOR_MAX_FORWARD     255    // Maximum forward speed
#define MOTOR_MAX_REVERSE     -255   // Maximum reverse speed

// ============================================================================
// TRANSMISSION SYSTEM CONFIGURATION
// ============================================================================

// Transmission Encoder Parameters
#define TRANS_ENCODER_PULSES_PER_REV  400    // Pulses per revolution (adjust based on sensor)
#define TRANS_ENCODER_MAX_COUNT       32767  // Maximum encoder count (PCNT 16-bit signed)
#define TRANS_ENCODER_MIN_COUNT      -32768  // Minimum encoder count
#define TRANS_POSITION_TOLERANCE      10     // +/- encoder counts for position match (±2.5% of revolution)

// Transmission Gear Positions (encoder counts - default values, overridden by calibration)
#define TRANS_POSITION_REVERSE   1400      // Encoder count for REVERSE (default)
#define TRANS_POSITION_NEUTRAL   3200    // Encoder count for NEUTRAL (default)
#define TRANS_POSITION_LOW       6900    // Encoder count for LOW (default)
#define TRANS_POSITION_HIGH      5400    // Encoder count for HIGH (default)

// Transmission Movement Parameters
#define TRANS_MOVE_TIMEOUT    15000   // ms - maximum time for gear change
#define TRANS_SETTLE_TIME     500    // ms - dwell time in NEUTRAL during transitions
#define TRANS_CALIBRATION_SPEED 100  // PWM value during calibration
#define TRANS_HOMING_SPEED      255  // PWM value during auto-homing (full speed)
#define TRANS_HOMING_TIMEOUT    60000 // ms - maximum time for homing
#define TRANS_STALL_THRESHOLD   3    // Encoder counts - if no change for this time, assume stall
#define TRANS_STALL_TIMEOUT     1000 // ms - time without encoder change = stall detected
#define TRANS_GEAR_CHECK_INTERVAL 500 // ms - interval for physical gear position verification

// Proportional Speed Control (implemented in BTS7960Controller::update())
// Speed ramp thresholds for smooth positioning:
//   > 200 counts: 100% speed (180 PWM)
//   > 100 counts: 85% speed
//   > 50 counts:  65% speed
//   > 30 counts:  45% speed
//   > 15 counts:  30% speed
//   ≤ 15 counts:  20% speed (minimum 30 PWM for reliable movement)

// ============================================================================
// BRAKE SYSTEM CONFIGURATION
// ============================================================================

// Brake Positions
#define BRAKE_RELEASED        0      // 0% - fully released
#define BRAKE_PARKING         30     // 30% - parking brake on startup
#define BRAKE_FULL            100    // 100% - full braking
#define BRAKE_EMERGENCY       100    // 100% - emergency stop

// Brake Movement Parameters
#define BRAKE_FULL_TRAVEL_TIME 3000  // ms - estimated time for 0-100% brake travel
#define BRAKE_TOLERANCE        5  // % - position tolerance for "at target"
#define BRAKE_SENSOR_OVERRUN_TIME 1000  // ms - continue moving after sensor triggers (full retraction)

// ============================================================================
// SAFETY PARAMETERS
// ============================================================================

// Watchdog Timer
#define WATCHDOG_TIMEOUT      3000   // ms - watchdog timeout period

// Command Deadbands
#define STEERING_DEADBAND     2.0f   // % - center deadband
#define THROTTLE_DEADBAND     2.0f   // % - idle deadband
#define BRAKE_DEADBAND        1.0f   // % - release deadband

// Safety Thresholds
#define THROTTLE_IDLE_THRESHOLD  5.0f    // % - below this is considered idle
#define BRAKE_OVERRIDE_THRESHOLD 50.0f   // % - brake overrides throttle above this
#define ACTUATOR_TIMEOUT        10000    // ms - actuator movement timeout

// Fail-safe Values
#define FAILSAFE_STEERING     0.0f   // % - center steering
#define FAILSAFE_THROTTLE     0.0f   // % - idle throttle
#define FAILSAFE_BRAKE        30.0f  // % - parking brake
// FAILSAFE_GEAR is NEUTRAL (defined in enum)

// ============================================================================
// LOOP TIMING
// ============================================================================

#define CONTROL_LOOP_FREQ     100    // Hz - main control loop frequency
#define CONTROL_LOOP_DT       10     // ms - control loop period

// ============================================================================
// SERIAL DEBUG
// ============================================================================

#define SERIAL_BAUD_RATE      115200 // Serial monitor baud rate
#define DEBUG_ENABLED         true   // Default debug output state (runtime-toggleable via Debug utility or web portal)

// Feature-specific debug flags (code-only control, persisted to NVS)
// Two-tier logging: Master debug (DEBUG_ENABLED) AND feature flag must both be ON
// Features: TRANSMISSION, CAN, SBUS, SERVO, BRAKE, RELAY, WEB, VEHICLE, TELEMETRY
// Enable programmatically: Debug::setFeatureEnabled(DebugFeature::TRANSMISSION, true)
// Or via NVS: preferences.putBool("feat_trans", true) in "debug" namespace
#define DEBUG_FEATURE_DEFAULT_STATE  false  // All features default to OFF

// ============================================================================
// NVS (Non-Volatile Storage) KEYS
// ============================================================================

#define NVS_NAMESPACE         "vehicle"
#define NVS_KEY_CALIBRATED    "calibrated"
#define NVS_KEY_ENCODER_POS   "encoder_pos"  // Current encoder position
#define NVS_KEY_TRANS_R       "trans_r"      // REVERSE gear encoder position
#define NVS_KEY_TRANS_N       "trans_n"      // NEUTRAL gear encoder position
#define NVS_KEY_TRANS_H       "trans_h"      // HIGH gear encoder position
#define NVS_KEY_TRANS_L       "trans_l"      // LOW gear encoder position
#define NVS_KEY_BRAKE_MIN     "brake_min"
#define NVS_KEY_BRAKE_MAX     "brake_max"

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
#define WEB_COMMAND_RATE_LIMIT 100                // ms - minimum time between commands (10 Hz max)

// ============================================================================
// OTA UPDATE CONFIGURATION
// ============================================================================

#define OTA_HOSTNAME          "quadbike-control"  // OTA hostname for identification
#define OTA_PASSWORD          ""                  // OTA password (empty = no password)
#define OTA_PORT              3232                // OTA port (default Arduino OTA port)

// ============================================================================
// INPUT SOURCE PRIORITY
// ============================================================================

// Input source priority: SBUS > WEB > FAILSAFE
enum class InputSource {
    SBUS,       // S-bus control active (highest priority)
    WEB,        // Web portal control active
    FAILSAFE    // No control source active (safe state)
};

// Input source names for telemetry/debugging
#define INPUT_SOURCE_NAME_SBUS      "SBUS"
#define INPUT_SOURCE_NAME_WEB       "WEB"
#define INPUT_SOURCE_NAME_FAILSAFE  "FAILSAFE"

// ============================================================================
// CAN CONTROLLER CONFIGURATION
// ============================================================================

// CAN Controller Speed
#define CAN_SPEED_500KBPS      0    // 500 kbps (standard automotive)

// CAN Polling Intervals
#define CAN_POLL_INTERVAL_RPM     200   // ms - RPM and speed polling rate
#define CAN_POLL_INTERVAL_TEMP    1000  // ms - Temperature polling rate

// CAN Timeouts
#define CAN_RESPONSE_TIMEOUT      200   // ms - OBD-II response timeout (non-blocking, healthy ECU responds in ~50ms)
#define CAN_DATA_STALE_TIMEOUT    5000  // ms - Mark data invalid if not updated
#define CAN_RETRY_ATTEMPTS        3     // Number of retry attempts on error

// Transmission Safety (CAN-based)
#define TRANS_SPEED_INTERLOCK_THRESHOLD  5     // km/h - Block gear changes above this speed
#define TRANS_CAN_TIMEOUT                5000  // ms - Allow gear change if CAN fails this long

// Throttle Boost During Gear Changes
#define TRANS_THROTTLE_BOOST_PERCENT     10    // % - Throttle increase during gear change
#define TRANS_THROTTLE_BOOST_DURATION    5000   // ms - Maximum boost duration

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

// Firmware version string (semantic versioning: MAJOR.MINOR.PATCH)
// Update this constant when releasing new firmware versions
#define FIRMWARE_VERSION "1.0.4"

#endif // CONSTANTS_H
