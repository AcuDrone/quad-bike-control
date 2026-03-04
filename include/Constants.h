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

// Steering Servo (PWM via LEDC)
#define PIN_STEERING_PWM    GPIO_NUM_8   // LEDC Channel 0
#define LEDC_CH_STEERING    0

// Throttle Servo (PWM via LEDC)
#define PIN_THROTTLE_PWM    GPIO_NUM_9   // LEDC Channel 1
#define LEDC_CH_THROTTLE    1

// Brake Linear Actuator (BTS7960)
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_BRAKE_RPWM      GPIO_NUM_6   // LEDC Channel 4
#define PIN_BRAKE_LPWM      GPIO_NUM_7   // LEDC Channel 5
#define LEDC_CH_BRAKE_RPWM  4
#define LEDC_CH_BRAKE_LPWM  5

// ============================================================================
// S-BUS CONFIGURATION
// ============================================================================

// S-bus Channel Assignments (1-16)
#define SBUS_CH_STEERING      1   // Channel 1: Steering (-100% to +100%)
#define SBUS_CH_THROTTLE      2   // Channel 2: Throttle (0% to 100%)
#define SBUS_CH_TRANSMISSION  3   // Channel 3: Gear selector (4 positions)
#define SBUS_CH_BRAKE         4   // Channel 4: Brake (0% to 100%)

// S-bus Protocol Parameters
#define SBUS_MIN_VALUE        172   // Minimum S-bus raw value
#define SBUS_MAX_VALUE        1811  // Maximum S-bus raw value
#define SBUS_CENTER_VALUE     992   // Center point for bidirectional channels
#define SBUS_SIGNAL_TIMEOUT   500   // ms before fail-safe activates
#define SBUS_MIN_RECOVERY_FRAMES 3  // Consecutive good frames to exit fail-safe

// S-bus to microseconds conversion (for compatibility)
#define SBUS_US_MIN           880   // Microseconds equivalent
#define SBUS_US_MAX           2160  // Microseconds equivalent
#define SBUS_US_CENTER        1500  // Microseconds center point

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

// Servo PWM Parameters
#define SERVO_PWM_FREQ        50     // Hz (20ms period)
#define SERVO_PWM_RESOLUTION  16     // bits (0-65535)
#define SERVO_MIN_US          1000   // Minimum pulse width (microseconds)
#define SERVO_MAX_US          2000   // Maximum pulse width (microseconds)
#define SERVO_CENTER_US       1500   // Center pulse width

// Steering Servo Limits
#define STEERING_MIN_ANGLE    0      // degrees (full left)
#define STEERING_MAX_ANGLE    180    // degrees (full right)
#define STEERING_CENTER_ANGLE 90     // degrees (center)
#define STEERING_MAX_RATE     200    // degrees per second (rate limiting)

// Throttle Servo Limits
#define THROTTLE_MIN_ANGLE    0      // degrees (idle)
#define THROTTLE_MAX_ANGLE    180    // degrees (full throttle)
#define THROTTLE_IDLE_ANGLE   0      // degrees (idle position)

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

// Transmission Gear Positions (encoder counts - calibrated)
#define TRANS_POSITION_HIGH      0      // Encoder count for HIGH (home position)
#define TRANS_POSITION_LOW       -1300  // Encoder count for LOW
#define TRANS_POSITION_NEUTRAL   -5000  // Encoder count for NEUTRAL
#define TRANS_POSITION_REVERSE   -10000 // Encoder count for REVERSE

// Transmission Movement Parameters
#define TRANS_MOVE_TIMEOUT    15000   // ms - maximum time for gear change
#define TRANS_SETTLE_TIME     500    // ms - dwell time in NEUTRAL during transitions
#define TRANS_CALIBRATION_SPEED 100  // PWM value during calibration
#define TRANS_HOMING_SPEED      255  // PWM value during auto-homing (full speed)
#define TRANS_HOMING_TIMEOUT    60000 // ms - maximum time for homing
#define TRANS_STALL_THRESHOLD   3    // Encoder counts - if no change for this time, assume stall
#define TRANS_STALL_TIMEOUT     1000 // ms - time without encoder change = stall detected

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
#define BRAKE_MOVE_TIMEOUT    3000   // ms - maximum time for brake actuation
#define BRAKE_APPLY_SPEED     150    // PWM value when applying brakes
#define BRAKE_RELEASE_SPEED   100    // PWM value when releasing brakes

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
#define DEBUG_ENABLED         true   // Enable debug output

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
#define NVS_KEY_SERVO_MIN     "servo_min"
#define NVS_KEY_SERVO_MAX     "servo_max"
#define NVS_KEY_BRAKE_MIN     "brake_min"
#define NVS_KEY_BRAKE_MAX     "brake_max"

#endif // CONSTANTS_H
