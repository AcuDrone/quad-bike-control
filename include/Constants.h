#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// ============================================================================
// GPIO PIN ASSIGNMENTS — custom control board "Control_v0" (ESP32-S3-WROOM-1U)
// ============================================================================
//
// WIRING AUTHORITY: GPIO_PINOUT_CUSTOM_BOARD_S3.md (§2 full pin table, §9 the
// migration summary this header was written from). If this header and that
// document disagree about a pin, the document describes what the copper does —
// resolve the divergence before running the firmware on the vehicle.
//
// This is a HARD SWITCH: the ESP32-S3-DevKitC-1 hand-wired map (GPIO_PINOUT_S3.md)
// is no longer supported and there is no board-selection build flag.
//
// Pins that deliberately have NO firmware function:
//   GPIO19/20 — native USB D-/D+ (USB-CDC console, flashing). Never assign.
//   GPIO43/44 — UART0, shared copper between the Prog header and the future
//               CH9121T wired-LAN chip. RESERVED: no console, no UART is opened
//               on them (the console is USB-CDC, ARDUINO_USB_CDC_ON_BOOT=1).
//   GPIO45    — CFG_CH9121T (LAN config strap), untouched until LAN is enabled.
//   GPIO0/21, GPIO10-13 — spare (system LED, Hall_1 B, Hall_2/3).

// MAVLink telemetry link (UART1 to Pixhawk TELEM, connector X9 via BSS138 shifters)
// ⚠ Verified at 115200 only — do NOT raise the baud rate (pinout §8.3).
#define PIN_MAVLINK_RX      GPIO_NUM_18  // UART1 RX <- Pixhawk TX (X9 pin 3)
#define PIN_MAVLINK_TX      GPIO_NUM_17  // UART1 TX -> Pixhawk RX (X9 pin 2)
#define MAVLINK_UART_NUM    UART_NUM_1   // UART1 for MAVLink
#define MAVLINK_BAUD_RATE   115200       // Pixhawk TELEM baud

// Transmission Servo (HappyModel Super400 Plus) — X8 pin 3
#define PIN_TRANS_SERVO     GPIO_NUM_16  // LEDC Channel 2

// Steering Actuator — driven by a Flipsky 75200 VESC over UART2 (brushed-DC mode).
// See "VESC STEERING DRIVER CONFIGURATION" below. The former BTS7960 steering PWM
// pins (GPIO17/18) and their LEDC channels (6/7) are fully released: GPIO17/18 are
// now the MAVLink UART and LEDC 6/7 are free for future use.

// VESC steering driver UART (UART_NUM_2; UART0 reserved for LAN, UART1 = MAVLink)
// Connector X6 via BSS138 shifters — 115200 only (pinout §8.3).
#define PIN_VESC_TX         GPIO_NUM_4    // ESP TX -> VESC RX (X6 pin 2)
#define PIN_VESC_RX         GPIO_NUM_5    // ESP RX <- VESC TX (X6 pin 3)
#define VESC_UART_NUM       2             // UART_NUM_2
#define VESC_UART_BAUD      115200        // VESC app default baud

// I2C1 ("Wire") — the EXTERNAL header bus: AS5600 steering angle sensor @ 0x36
// on X14 is the only device on it (see the bus map below).
// The bus is opened once in main.cpp; no driver on this bus may re-open it with
// different parameters.
#define PIN_STEER_SDA       GPIO_NUM_1    // I2C1 SDA
#define PIN_STEER_SCL       GPIO_NUM_2    // I2C1 SCL

// Throttle Servo (PWM via LEDC) — X8 pin 2
#define PIN_THROTTLE_PWM    GPIO_NUM_15  // LEDC Channel 1
#define LEDC_CH_THROTTLE    1

// Brake Linear Actuator (BTS7960) — X7
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_BRAKE_RPWM      GPIO_NUM_7   // LEDC Channel 4 (X7 pin 3)
#define PIN_BRAKE_LPWM      GPIO_NUM_6   // LEDC Channel 5 (X7 pin 2)
#define LEDC_CH_BRAKE_RPWM  4
#define LEDC_CH_BRAKE_LPWM  5

// SPI CAN Controller (MCP2515 + TJA1051T/3, 8 MHz crystal → MCP_8MHZ)
#define PIN_CAN_MOSI        GPIO_NUM_42  // SPI MOSI
#define PIN_CAN_MISO        GPIO_NUM_41  // SPI MISO
#define PIN_CAN_SCK         GPIO_NUM_40  // SPI SCK
#define PIN_CAN_CS          GPIO_NUM_39  // SPI Chip Select
#define PIN_CAN_INT         GPIO_NUM_38  // MCP2515 INT — DEFINED ONLY, the driver polls

// 24V boost rail (Buck Boost board via X10)
#define PIN_BOOST_EN        GPIO_NUM_46  // Boost enable (X10 pin 1). 100K pulldown R7 keeps it off through reset
#define PIN_ADC_24V         GPIO_NUM_9   // Rail measurement (X10 pin 3) through a 200K/24K divider, ADC1

// Driveline speed sensor (12V hall) pulse input — connector X2, PCNT counted.
// Signal path: X2.1/X2.2 -> 470R R22 -> 6N137 LED -> open-collector + pull-up -> GPIO8.
// ⚠ The optocoupler INVERTS the signal (LED conducting = GPIO8 LOW), so the firmware
// counts the FALLING edge and does not "fix" the polarity — pulse rate is identical on
// either edge.
// ⚠ HARDWARE: a 12V sensor needs 560R-1k extra in series in the cable (or R22 reworked to
// 1k); the stock 470R alone passes ≈22 mA, over the 6N137's 20 mA absolute maximum.
#define PIN_SPEED_SENSOR    GPIO_NUM_8

// CD4051 8:1 current-sense mux common output (BTS7960 IS channels), ADC1.
// DEFINED ONLY — the mux-control PCA9557 U2 @ 0x1C is not driven by this firmware,
// so this pin is not sampled. GPIO3 is also the JTAG-source strap; input-only use is safe.
#define PIN_ADC_IS_MUX      GPIO_NUM_3

// ============================================================================
// I2C BUSES AND PORT EXPANDERS
// ============================================================================
//
// Functions that used to be direct GPIO (gear switches, brake limit sensor,
// four relays) now live behind PCA9557 port expanders, so reads and writes are
// fallible. Every expander driver instance is bound to exactly one bus and one
// address at construction — there is no bus scanning.
//
// BUS MAP — scan-verified on the assembled board. It overrides the schematic
// port names, which suggested U10 was on I2C1:
//
//   I2C1 "Wire"  (GPIO1/2)   : external headers only —
//                              AS5600 0x36 (X14, reserved for it)
//   I2C2 "Wire1" (GPIO48/47) : BOTH on-board PCA9557s —
//                              opto-input U10 0x1A, CD4051 mux U2 0x1C
//                              + external relay board 0x1F on header X15
//
// ⚠ The I2C2 level shifter low side is jumper-selected: R16 populated (3V3),
//   R18 not (pinout §8.6).

#define PIN_RELAY_SDA       GPIO_NUM_48  // I2C2 SDA (on-board U10/U2 + X15, BSS138 level shifted)
#define PIN_RELAY_SCL       GPIO_NUM_47  // I2C2 SCL (on-board U10/U2 + X15, BSS138 level shifted)
// Both buses run at 100 kHz. The AS5600 (X14) sits at the end of a long cable run to the
// steering column, so signal integrity wins over speed — user decision. 400 kHz was tried and
// rejected: not enough margin on that cable. Raising it again would need a stronger pull-up.
#define I2C_BUS_FREQ_HZ     100000       // Both buses run at 100 kHz (see note above)

#define PCA9557_ADDR_INPUTS        0x1A  // On-board opto-input expander U10 (I2C2)
// External relay board on X15 → I2C2, as shipped strapped A2A1A0 = 111 → 0x1F.
// The strap is kept: 0x1F collides with neither on-board expander (0x1A / 0x1C).
#define PCA9557_ADDR_RELAYS        0x1F  // External relay board
#define PCA9557_ADDR_MUX_RESERVED  0x1C  // On-board CD4051 mux control U2 (I2C2) — NEVER addressed

// Opto-isolated input bit indices on PCA9557 U10.
//
// ⚠ The In1..In8 = bits 0..7 assumption taken from the schematic is WRONG for the
//   gear channels — the routing is not 1:1, same story as the relay board above.
//   Bench-verified 2026-08-21 by sweeping the gear lever with a live raw dump:
//     NEUTRAL = bit 0   REVERSE = bit 1   LOW = bit 6   HIGH = bit 7
//   Observed port bytes: between positions 0x1C, R 0x1E, N 0x1D, L 0x5C, H 0x9C.
//   Bits 2, 3 and 4 sat HIGH and bit 5 LOW throughout — none of them ever moved.
//
// ⚠ The gear channels are engaged-HIGH: every wired gear opto CONDUCTS at rest
//   (pin LOW) and selecting a position OPENS exactly that one channel, so its pin
//   goes HIGH. BOARD_INPUT_ACTIVE_LOW below therefore describes the brake input
//   only — do NOT apply it to the four gear bits.
#define BOARD_IN_BIT_GEAR_NEUTRAL  0     // X16 — engaged = HIGH
#define BOARD_IN_BIT_GEAR_REVERSE  1     // X16 — engaged = HIGH
#define BOARD_IN_BIT_GEAR_LOW      6     // X16 — engaged = HIGH
#define BOARD_IN_BIT_GEAR_HIGH     7     // X16 — engaged = HIGH
#define BOARD_IN_BIT_BRAKE_LIMIT   4     // In5 (X17) — brake "released" endstop

// Brake input bit polarity. A PC817 that is conducting pulls its PCA9557 input
// LOW, so an asserted input reads 0 — hence active-low. NOT bench-verified: bit 4
// never moved during the 2026-08-21 sweep, so this is still the schematic's sense.
#define BOARD_INPUT_ACTIVE_LOW     1     // 1 = asserted input reads 0, 0 = asserted reads 1

// Relay port masks on the relay board's PCA9557.
//
// ⚠ The relay board's IO routing is NOT 1:1 — do NOT "simplify" these back to
//   Relay_N = bit N. Verified against the Relay.PrjPcb schematic netlist:
//     Relay_1=IO4  Relay_2=IO5  Relay_3=IO6  Relay_4=IO7
//     Relay_5=IO3  Relay_6=IO2  Relay_7=IO0  Relay_8=IO1
//
// Two functions drive a PAIR of relays that always energize together (the load is
// split across two contacts in the harness), so each function is a whole-port MASK,
// not a single bit.
#define RELAY_MASK_IGNITION     0b00110000  // Relay_1 (IO4, X4) + Relay_2 (IO5, X5) — ignition/ECU line, paralleled pair
#define RELAY_MASK_STARTER      0b01000000  // Relay_3 (IO6, X6) — starter solenoid
#define RELAY_MASK_FRONT_LIGHT  0b10000000  // Relay_4 (IO7, X7) — front light
#define RELAY_MASK_WHEEL_LOCK   0b00001100  // Relay_5 (IO3, X8) + Relay_6 (IO2, X9) — front-wheel lock, paralleled pair
// Relay_7 (IO0, X10) and Relay_8 (IO1, X11) are spare: mask 0b00000011 is always
// written 0. Connectors X4..X11 above are the RELAY BOARD's connectors, not the
// main board's X4/X5 Hall headers.

// Expander timing / fault policy
#define BOARD_INPUT_POLL_MS        25    // ms between opto-input polls
#define BOARD_INPUT_STALE_MS       250   // ms without a good read before the snapshot is invalid
#define BOARD_INPUT_RECOVER_MS     1000  // ms between re-init attempts while faulted
#define RELAY_WRITE_RETRIES        3     // read-back verification retries per port write

// ============================================================================
// 24V BOOST RAIL
// ============================================================================

#define RAIL_SAMPLE_INTERVAL_MS    500    // ms between rail voltage samples
#define RAIL_ADC_SAMPLE_COUNT      8      // readings averaged per sample (ADC1 is noisy)
#define RAIL_24V_DIVIDER_RATIO     9.33f  // 200K/24K divider: V24 = Vadc × 224/24
// Low-rail warning threshold. ⚠ PROVISIONAL — must be tuned on the bench against
// the measured loaded minimum of the TL494 boost under Jetson + Starlink + camera
// load before it is trusted (task 11.1). Warning only: the rail is never dropped
// automatically because that would remove steering authority.
#define RAIL_24V_LOW_THRESHOLD     21.0f  // V
#define BOOST_STARTUP_GRACE_MS     1000   // ms after enable during which low-rail warnings are suppressed

// Cranking Parameters
#define CRANKING_TIMEOUT           2000  // ms - maximum cranking duration
#define ACC_PRECRANK_DWELL_MS      2000  // ms - ignition/ECU line (Relay_1+Relay_2) must be powered this long before the starter (Relay_3) engages
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

// Gear Selection Ranges (in microseconds) - 3-position switch: L/N/R
// Full span 880-2160 divided into three equal ~427µs bands, LOW at low PWM
#define RC_GEAR_LOW_MIN       880
#define RC_GEAR_LOW_MAX       1306
#define RC_GEAR_NEUTRAL_MIN   1307
#define RC_GEAR_NEUTRAL_MAX   1733
#define RC_GEAR_REVERSE_MIN   1734
#define RC_GEAR_REVERSE_MAX   2160

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
// Position feedback is an AS5600 absolute angle sensor: all positions are raw
// 12-bit counts (0-4095, ~0.088°/LSB). Center and left/right travel limits are
// calibrated via the web UI (capture current angle) and stored in NVS
// ("steering"/"c_ang","l_ang","r_ang"). Uncalibrated => percent commands rejected.
#define STEER_POSITION_TOLERANCE  10    // +/- AS5600 counts for position match (~0.9°)
#define STEER_MOVE_TIMEOUT        7500 // ms - maximum time for any movement
#define STEER_STALL_THRESHOLD     5     // AS5600 counts - stall detection threshold
#define STEER_STALL_TIMEOUT       500   // ms - time without position change = stall
#define STEER_CAL_MIN_SPAN        50    // AS5600 counts - min |limit - center| accepted at calibration (~4.4°)
#define STEER_SENSOR_FAULT_DEBOUNCE 5   // consecutive bad samples before declaring a sensor fault (EMI glitch filter)

// Proportional PWM control: duty = clamp(|error| * STEER_KP + boost, MIN, MAX)
#define STEER_KP                  1.0f  // PWM duty (0-255) per AS5600 count of error
#define STEER_PWM_MIN_DUTY        140   // enough to overcome static friction (bench: no movement at 70)
#define STEER_PWM_MAX_DUTY        255   // full speed for large errors

// Anti-stall boost: steering load rises toward the locks; when commanded motion
// makes no progress, duty escalates above the P term until movement resumes.
// (Bench: closed-loop stalled ~72 counts short of the left lock at MIN duty.)
#define STEER_BOOST_CHECK_MS      100   // ms - progress check period
#define STEER_BOOST_MIN_PROGRESS  3     // counts per check period considered "moving"
#define STEER_BOOST_STEP          30    // duty added (stalled) or removed (moving) per check

// Calibration jog (momentary open-loop drive from the web UI)
#define STEER_JOG_DUTY            180   // PWM duty while jogging (default when the UI sends no speed; min working duty is ~140)
#define STEER_JOG_TIMEOUT_MS      500   // ms - auto-stop if the jog command is not refreshed
#define STEER_NUDGE_DUTY          150   // PWM duty for a precision nudge pulse (just above friction)
#define STEER_NUDGE_MS            70    // ms - nudge pulse length (finer: ~1-2 counts per tap)

// Re-command guard + stall latch (MAVLink streams steering at ~25 Hz; without
// these the stall/move timers never elapse — the fixed re-command bug).
#define STEER_RETARGET_TOLERANCE  10    // AS5600 counts — re-command within this of the current target is a no-op (timers keep running)
#define STEER_STALL_COOLDOWN_MS   1500  // ms — same-direction moves refused after a stall; opposite direction always allowed

// VESC steering driver telemetry monitor + failsafe
#define STEER_VESC_TELEM_MS         300   // ms — COMM_GET_VALUES poll period (~3 Hz)
#define STEER_VESC_OVERCURRENT_A    18.0f // A — motor current above this (sustained) trips a stall-stop (below the VESC hardware limit)
#define STEER_VESC_OVERCURRENT_MS   400   // ms — over-current must persist this long before tripping
#define STEER_VESC_COMM_TIMEOUT_MS  1000  // ms — no valid GET_VALUES reply for this long -> driver fault (stop, reject moves)

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
// VEHICLE SPEED SENSOR (12V toothed-ring pickup, PCNT on PIN_SPEED_SENSOR)
// ============================================================================

// Sampling / signal conditioning
#define SPEED_SAMPLE_INTERVAL_MS      200    // ms between PCNT samples (5 Hz, matches telemetry)
// No edges for this long => the reported speed decays to 0 km/h. With the measured
// calibration one pulse at ~1 km/h takes only ~0.10 s (28.8 mm/pulse ÷ 278 mm/s), so
// 2 s keeps a wide margin: it only trips on a genuinely stopped or disconnected sensor,
// never on a crawling vehicle.
#define SPEED_STALE_TIMEOUT_MS        2000   // ms
// PCNT hardware glitch filter. Highest expected pulse rate with the measured calibration
// is ≈960 Hz at 100 km/h (period ~1.04 ms), so a 1 µs filter rejects harness noise while
// consuming <0.2% of the shortest half-period. Hardware ceiling is ~12.7 µs (1023 APB cycles).
#define SPEED_GLITCH_FILTER_NS        1000   // ns
// PCNT counter limits. The hardware zeroes the counter at these watch points and the
// unit accumulates the overflow itself (accum_count), so update() polls a running total.
#define SPEED_PCNT_HIGH_LIMIT         10000
#define SPEED_PCNT_LOW_LIMIT          (-1)

// Calibration defaults (NVS namespace "speed", keys "ppr" / "circ_mm"), both
// bench-measured on the vehicle and both settable at runtime from the web UI
// (speed_cal_ppr / speed_cal_circ) — no reflash needed to change them.
// The fitted pickup reads a 70-tooth ring, NOT discrete magnets.
#define SPEED_DEFAULT_PULSES_PER_REV       70      // pulses per wheel revolution (toothed ring)
#define SPEED_DEFAULT_WHEEL_CIRCUMFERENCE_MM 1990.0f  // mm (25" ATV tyre, measured roll-out)

// Accepted calibration ranges (web command validation)
#define SPEED_PPR_MIN                 1
#define SPEED_PPR_MAX                 1000
#define SPEED_CIRC_MIN_MM             100.0f
#define SPEED_CIRC_MAX_MM             10000.0f

// Health heuristic: the fingerprint of a mid-motion wire fault is pulses ceasing
// faster than the vehicle could physically have stopped. If the last observed speed
// would need more than the stale timeout to bleed off at this deceleration, the
// silence is implausible and the reading is latched suspicious (isValid() false)
// until pulses resume. A normal stop decelerates through the samples instead.
#define SPEED_MAX_PLAUSIBLE_DECEL_KMH_S  8.0f   // km/h per second

// Maximum-speed throttle limiter (NVS "speed", keys "lim_on" / "lim_kmh").
// Ships DISABLED: default drive behavior is unchanged until an operator enables it.
#define SPEED_LIMIT_ENABLE_DEFAULT    false
#define SPEED_LIMIT_MAX_KMH_DEFAULT   60.0f  // km/h - safe/high default ceiling
#define SPEED_LIMIT_THROTTLE_CAP_PCT  20.0f  // % - reduced throttle ceiling above the limit (not a hard cut)
#define SPEED_LIMIT_MIN_KMH           1.0f   // accepted range for speed_limit_set
#define SPEED_LIMIT_MAX_KMH           200.0f
#define SPEED_LIMIT_WARN_MS           5000   // ms between "limiter armed but speed invalid" warnings

// ============================================================================
// SERIAL DEBUG
// ============================================================================

// The console is the ESP32-S3 native USB CDC device on the board's USB-C connector
// (build flag ARDUINO_USB_CDC_ON_BOOT=1). UART0 (GPIO43/44) is RESERVED for the
// future CH9121T wired LAN and must not be reclaimed for a console.
// Baud is irrelevant for CDC but harmless; the firmware never waits for a host.
#define SERIAL_BAUD_RATE      115200 // Serial monitor baud rate (ignored by USB-CDC)
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

// CAN Diagnostics / Recovery (bring-up aids, all non-blocking)
#define CAN_MAX_CONSEC_SEND_FAILURES 3     // Consecutive sendMsgBuf() failures before abortTX()
#define CAN_RX_LOG_INTERVAL          1000  // ms - rate limit for unmatched-RX-frame logging
#define CAN_HEALTH_LOG_INTERVAL      5000  // ms - rate limit for TEC/REC/EFLG health logging
#define CAN_OVERFLOW_LOG_INTERVAL    5000  // ms - rate limit for RXnOVR overflow logging
#define CAN_REINIT_NO_DATA_MS        15000 // ms - full chip re-init when polling but no valid OBD response for this long (heals a wedged TX / latched ABAT once the ECU powers up)
#define CAN_REINIT_MIN_INTERVAL_MS   15000 // ms - minimum spacing between forced re-inits

// MCP2515 hardware acceptance filtering (standard 11-bit IDs only).
// The ECU floods the bus with broadcast frames (Delphi MT05 sends 0x301 etc.) which
// overrun the two RX buffers faster than the loop can drain them. Accept only the
// OBD-II response window 0x7E8-0x7EF: (id & 0x7F8) == 0x7E8.
// The coryjfowler MCP_CAN library expects standard mask/filter IDs shifted left by 16
// (mcp2515_write_mf() takes bits 26:16 as the 11-bit ID when ext == 0).
#define CAN_RX_ID_MASK            0x7F8   // 11-bit acceptance mask (care bits 10:3)
#define CAN_RX_ID_FILTER          0x7E8   // 11-bit acceptance filter (OBD-II ECU responses)
#define CAN_RX_ID_MASK_REG   (((uint32_t)CAN_RX_ID_MASK) << 16)    // 0x07F80000
#define CAN_RX_ID_FILTER_REG (((uint32_t)CAN_RX_ID_FILTER) << 16)  // 0x07E80000

// ECU Capability Probe (on-demand diagnostic sweep)
#define CAN_PROBE_RETRY_ATTEMPTS  1     // Retries per probe request (keeps worst-case sweep duration bounded)
#define PROBE_RESULT_TTL          8000  // ms - window during which completed probe results are embedded in telemetry

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
