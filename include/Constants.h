#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include "RelayBoardLaw.h"   // RELAY_N() for the relay-number sets below

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
//               CH9121T wired-LAN chip (R63/R64 fitted on the quad board). RESERVED:
//               no console, no UART and no log output on them (the console is
//               USB-CDC, ARDUINO_USB_CDC_ON_BOOT=1).
//   GPIO45    — CFG_CH9121T (LAN config strap), untouched until LAN is enabled.
//   GPIO0/21, GPIO10-13 — spare (system LED, Hall_1 B, Hall_2/3).
// GPIO4/5 are NOT free: they carry the steering VESC on X6 (UART2, see PIN_VESC_TX).

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
// (The aztec buggy runs its VESC on UART0 / X1 instead — aztec branch.)
#define PIN_VESC_TX         GPIO_NUM_4    // ESP TX -> VESC RX (X6 pin 2)
#define PIN_VESC_RX         GPIO_NUM_5    // ESP RX <- VESC TX (X6 pin 3); internal pull-up set in VescMotorDriver::begin()
#define VESC_UART_NUM       2             // UART_NUM_2
#define VESC_UART_BAUD      115200        // VESC app default baud

// I2C1 ("Wire") — the EXTERNAL header bus on X14: NO device fitted (the steering
// angle sensor is a Bosch LWS on the CAN bus), so `Wire` is not started. DEFINED
// ONLY — kept for the documented contingency of moving the relay board to X14
// (see the bus map below).
#define PIN_I2C1_SDA        GPIO_NUM_1    // I2C1 SDA (X14) — defined, unused
#define PIN_I2C1_SCL        GPIO_NUM_2    // I2C1 SCL (X14) — defined, unused

// Throttle Servo (PWM via LEDC) — X8 pin 2
#define PIN_THROTTLE_PWM    GPIO_NUM_15  // LEDC Channel 1
#define LEDC_CH_THROTTLE    1

// Brake Linear Actuator (BTS7960) — X7
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_BRAKE_RPWM      GPIO_NUM_7   // LEDC Channel 4 (X7 pin 3)
#define PIN_BRAKE_LPWM      GPIO_NUM_6   // LEDC Channel 5 (X7 pin 2)
#define LEDC_CH_BRAKE_RPWM  4
#define LEDC_CH_BRAKE_LPWM  5

// SPI CAN Controller (MCP2515 + TJA1051T/3, 8 MHz crystal → MCP_8MHZ). One shared
// 500 kbit/s bus: the engine ECU (OBD-II 0x7DF/0x7E8) AND the Bosch LWS steering
// angle sensor (0x305, 100 Hz). See "CAN CONTROLLER CONFIGURATION" for the filter plan.
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
//   I2C1 "Wire"  (GPIO1/2)   : external header X14 only — NO device fitted, not started
//   I2C2 "Wire1" (GPIO48/47) : BOTH on-board PCA9557s —
//                              opto-input U10 0x1A, CD4051 mux U2 0x1C
//                              + ONE external relay board on header X15:
//                                the PCA9557 relay board 0x1F, or
//                                the XL9535-K8V5 relay board 0x20..0x27 (default 0x20)
//
// Expected boot scan: I2C2 = 0x1A 0x1C + 0x1F (PCA9557 relay board) or one of
// 0x20..0x27 (XL9535 relay board). The relay board is selected once at boot
// (RelayController::begin(), setting NVS "relay"/"board"). No expander driver restarts
// Wire1; after setup() only the main-owned I2cBusRecovery routine does (SCL unlock +
// Wire1.end()/begin() on bus evidence, 1.5.26). Each expander keeps its own fail-safe
// (BoardInputs re-configures itself, RelayController verifies every write on the pins and
// re-inits the relay port while faulted).
// (Steering feedback is on CAN since firmware 1.3.0 — the former AS5600 pair on
// X14/X15 is gone.)
//
// ⚠ The I2C2 level shifter low side is jumper-selected: R16 populated (3V3),
//   R18 not (pinout §8.6).

#define PIN_RELAY_SDA       GPIO_NUM_48  // I2C2 SDA (on-board U10/U2 + X15 relay board, BSS138 level shifted)
#define PIN_RELAY_SCL       GPIO_NUM_47  // I2C2 SCL (on-board U10/U2 + X15 relay board, BSS138 level shifted)
// I2C runs at 100 kHz (the relay board sits at the end of a cable run on X15); 400 kHz was
// tried and rejected for lack of margin. Raising it again would need a stronger pull-up.
#define I2C_BUS_FREQ_HZ     100000       // 100 kHz (see note above)
// I2C2 timeout + bus unlock (I2cBusRecovery, change add-event-journal-and-link-recovery D11).
// Wire1.setTimeOut(I2C_TIMEOUT_MS) after every Wire1.begin(): a dead bus can no longer hold a
// transaction for the core's 50 ms default. Only I2cBusRecovery restarts Wire1 after setup().
#define I2C_TIMEOUT_MS             20    // ms — max duration of one Wire1 transaction
#define I2C_UNLOCK_CLOCKS          9     // SCL pulses bit-banged to free a slave holding SDA low
#define I2C_RECOVER_MIN_MS         1000  // ms — min spacing between bus recoveries (all callers)
#define BOARD_INPUT_UNLOCK_FAILS   8     // consecutive failed opto-input polls (~200 ms) before a bus recovery

#define PCA9557_ADDR_INPUTS        0x1A  // On-board opto-input expander U10 (I2C2)
// PCA9557 relay board on X15 → I2C2, as shipped strapped A2A1A0 = 111 → 0x1F.
// The strap is kept: 0x1F collides with neither on-board expander (0x1A / 0x1C).
#define PCA9557_ADDR_RELAYS        0x1F  // External PCA9557 relay board
// XL9535-K8V5 relay board (PCA9535-compatible, port P0 only): pads A0..A2 → 0x20..0x27,
// default 0x20. Collides with nothing on I2C2. Hardware: J1 removed, VIN from the X15
// 3.3 V side, the 5V relay terminal fed separately (≈0.6 A), shared GND.
#define XL9535_ADDR_MIN            0x20  // XL9535 relay board, pads A2A1A0 = 000 (default)
#define XL9535_ADDR_MAX            0x27  // XL9535 relay board, pads A2A1A0 = 111
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
// Quad bike: one line per gear (the aztec buggy's 3-bit gear code lives on aztec).
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

// Relay function assignment, as RELAY-NUMBER sets (bit0 = Relay_1 … bit7 = Relay_8 — the
// same encoding as the `relay_test` telemetry), NOT port masks. RelayController::begin()
// builds the whole-port masks from these through the DETECTED board's Relay_N → bit table
// (include/RelayBoardLaw.h):
//   PCA9557 relay board (0x1F) — ⚠ NOT 1:1, verified against the Relay.PrjPcb netlist;
//     do NOT "simplify" it to Relay_N = bit N-1 (that would crank on a light request):
//       Relay_1=IO4  Relay_2=IO5  Relay_3=IO6  Relay_4=IO7
//       Relay_5=IO3  Relay_6=IO2  Relay_7=IO0  Relay_8=IO1
//   XL9535-K8V5 relay board (0x20..0x27) — identity, Relay_N = P0.(N-1), active-high
//     (bench-confirmed 2026-11-07 on an XL9535-K8V5 @0x20, tasks.md 1.4).
// Compile-time checks (RelayController.cpp): the sets are disjoint and cover all eight
// relays, both tables are permutations, and the mapped masks are disjoint and OR to 0xFF
// on BOTH boards; begin() repeats the mask check at runtime.
//
// QUAD BIKE assignment (master; the aztec buggy's single-relay assignment lives on
// aztec). Two functions drive a PAIR of relays that always energize together (the
// load is split across two contacts in the harness), so each function is a SET of
// relays, not a single relay.
// Resulting masks — PCA9557 / XL9535: ignition 0x30 / 0x03, starter 0x40 / 0x04,
// front light 0x80 / 0x08, wheel lock 0x0C / 0x30, reserved 0x03 / 0xC0.
#define RELAY_SET_IGNITION     (RELAY_N(1) | RELAY_N(2))  // Relay_1 + Relay_2 (PCA9557 IO4+IO5, X4+X5 / XL9535 P0.0+P0.1) — ignition/ECU line, paralleled pair
#define RELAY_SET_STARTER      RELAY_N(3)                 // Relay_3 (PCA9557 IO6, X6 / XL9535 P0.2) — starter solenoid
#define RELAY_SET_FRONT_LIGHT  RELAY_N(4)                 // Relay_4 (PCA9557 IO7, X7 / XL9535 P0.3) — front light
#define RELAY_SET_WHEEL_LOCK   (RELAY_N(5) | RELAY_N(6))  // Relay_5 + Relay_6 (PCA9557 IO3+IO2, X8+X9 / XL9535 P0.4+P0.5) — front-wheel lock, paralleled pair
// Relay_7 and Relay_8 are spare: outside the web relay test (below) a reserved relay is
// never set by the driver — every port write leaves those bits 0. Connectors X4..X11
// here are the PCA9557 RELAY BOARD's connectors, not the main board's X4/X5 Hall headers.
#define RELAY_SET_RESERVED     (RELAY_N(7) | RELAY_N(8))  // Relay_7 | Relay_8 — spare

// Web relay test (bench wiring check). Any of the eight relays can be toggled from the
// web UI, but ONLY while the ignition is OFF and no real function relay is energized
// (light off, wheel lock released); the per-relay number -> port bit lookup is the active
// board's table (RelayController::relayIoMask()). A test relay is the only way a reserved
// relay is ever energized, and each relay of a paralleled pair is tested on its own.
// Every test bit auto-releases after RELAY_TEST_TIMEOUT_MS; the starter relay
// (Relay_3) is capped much shorter so a test cannot crank the engine for long.
#define RELAY_TEST_TIMEOUT_MS      30000UL  // ms a test relay stays on before auto-off
#define RELAY_TEST_STARTER_MAX_MS  2000UL   // ms cap for a starter-relay (Relay_3) test

// The front-wheel lock is fitted on the quad bike (RELAY_SET_WHEEL_LOCK above);
// RelayController drives it from MAVLink channel 7 and the web `set_wheel_lock`.
#define RELAY_WHEEL_LOCK_FITTED 1           // 1 = relays behind the wheel-lock function

// Expander timing / fault policy
#define BOARD_INPUT_POLL_MS        25    // ms between opto-input polls
#define BOARD_INPUT_STALE_MS       250   // ms without a good read before the snapshot is invalid
#define BOARD_INPUT_RECOVER_MS     1000  // ms between re-init attempts while faulted
#define RELAY_WRITE_RETRIES        3     // pin read-back verification retries per port write
#define RELAY_RECOVER_MS           1000  // ms between relay-port re-inits (initPort + shadow write) while faulted

// Relay board selection (persisted; read once in RelayController::begin(), applied at boot).
// uint8: 0 = auto (default, also for an absent or out-of-range value), 1 = pca9557, 2 = xl9535.
#define RELAY_BOARD_NVS_NS         "relay"
#define RELAY_BOARD_NVS_KEY        "board"

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
#define CRANKING_TIMEOUT           5000  // ms - maximum cranking duration per start attempt; the starter is released earlier as soon as engineRpm ≥ ENGINE_RUNNING_RPM_THRESHOLD (2026-11-06: 2 s was too short)
#define ACC_PRECRANK_DWELL_MS      2000  // ms - ignition/ECU line (Relay_1+Relay_2) must be powered this long before the starter (Relay_3) engages
#define ENGINE_RUNNING_RPM_THRESHOLD 1500  // RPM - engine considered running above this

// ----------------------------------------------------------------------------
// ENGINE HOUR METER ("мотогодини") — NVS namespace "engine", key "hours_s"
// ----------------------------------------------------------------------------
// Deliberately NOT ENGINE_RUNNING_RPM_THRESHOLD (1500). That one is a gear-change /
// crank-interlock SAFETY threshold, chosen to sit safely ABOVE idle; an hour meter
// built on it would refuse to count idling, which is exactly the running time an
// hour meter exists to record. Nothing in this repo documents this engine's idle
// RPM, so the floor rests on a physical argument instead: a starter cranks at
// ~200-300 RPM and a petrol engine idles at 800-1500, so anything above ~300 means
// the crank is turning under its own power or under the starter — both are running
// time. Below it, "ignition on, engine stopped" (CAN valid, rpm 0) counts nothing.
#define ENGINE_HOURS_MIN_RPM          300    // RPM - at or above this the crank is turning

// Single-update cap. The main loop is cooperative and non-blocking, so a delta of
// even 200 ms is already unusual; 5 s is an order of magnitude above anything the
// loop legitimately does. A longer delta is a symptom (a stall, a long flash
// operation, a debugger halt), not attested running time, and is DISCARDED whole —
// at most this much real running time is lost per stall event.
#define ENGINE_HOURS_MAX_DELTA_MS     5000   // ms - a longer single update adds nothing

// Hour-meter persistence interval, in ACCUMULATED seconds of growth (not wall clock:
// a parked vehicle with the controller powered writes nothing). Plus one write on every
// ignition-OFF transition and on fail-safe entry, which makes a normal shutdown lossless
// and bounds a power-cut loss to 10 minutes. The budget: NVS is wear-levelled and
// log-structured, so ~126 writes of the one uint64 key fill a 4096-byte page and cost
// one erase — 6 writes per engine-hour means ≈480 erases over 10 000 engine hours,
// ≈0.48 % of the flash's ~100 000-cycle endurance. A quad engine is rebuilt long before.
#define ENGINE_HOURS_NVS_WRITE_INTERVAL_S  600ULL  // s of accumulated growth between NVS writes

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
#define MAVLINK_STEER_SLOW_TX_MS      1000   // ms between the SLOW steering-VESC named floats
                                             // (VESC_TEMP, VESC_OK) — 1 Hz. The other three
                                             // (STEER_POS, STEER_A, VESC_V) ride the
                                             // MAVLINK_REPORT_TX_MS tick at 5 Hz
#define MAVLINK_STATUSTEXT_MIN_MS     250    // ms minimum spacing between STATUSTEXT messages

// Autopilot parameter subscription (READ-ONLY — the firmware never sends PARAM_SET).
// TWO parameters are subscribed, both read-only, through ONE shared mechanism (one poll, one
// PARAM_VALUE handler, one set of value-hygiene rules — see MavlinkInterface's param table):
//   SPEED_MAX        (m/s) — the ONLY source of the firmware's max-speed limiter ceiling.
//   MOT_SPD_SCA_BASE (m/s) — the FALLBACK source of the steering speed-scaling base; the
//                            ESP32's own NVS value (steer_sca_base) takes priority over it.
// Both an unsolicited PARAM_VALUE and a periodic PARAM_REQUEST_READ are honored: ArduPilot's
// broadcast-on-set behaviour is version- and routing-dependent, so the POLL is the change
// detector and the broadcast only makes it sooner.
#define MAVLINK_PARAM_SPEED_MAX_ID    "SPEED_MAX"  // ArduPilot cruise-speed ceiling (m/s); 0 = "no limit"
#define MAVLINK_PARAM_SPEED_MAX_MS    30.0f  // m/s — ArduPilot's own range bound; above this → rejected
#define MAVLINK_PARAM_SPD_SCA_BASE_ID "MOT_SPD_SCA_BASE"  // ArduPilot steering speed-scaling base (m/s); 0 = "no scaling"
#define MAVLINK_PARAM_SPD_SCA_BASE_MAX 10.0f // m/s — ArduPilot's own range bound for that parameter
#define MAVLINK_PARAM_POLL_MS         5000   // ms between PARAM_REQUEST_READ polls (never stops)
#define MAVLINK_PARAM_FIRST_DELAY_MS  1000   // ms after the autopilot is learned before the first request
#define MAVLINK_PARAM_STALE_MS        16000  // ms without a PARAM_VALUE before the value is unusable (~3 polls)
#define MAVLINK_PARAM_EPSILON_MS      0.005f // m/s — smaller differences are "unchanged" (never compare floats with ==)

// Inbound COMMAND_LONG handling. The only command implemented is a trip reset, carried as
// MAV_CMD_USER_1 with a magic param1 so that a stray, replayed or mis-scripted MAV_CMD_USER_1
// cannot silently destroy the operator's trip reading.
#define MAVLINK_CMD_TRIP_RESET_MAGIC  1.0f   // param1 magic for MAV_CMD_USER_1 (trip reset)
#define MAVLINK_CMD_PARAM_EPSILON     0.01f  // param1 match tolerance (never compare floats with ==)

// Body-frame wheel odometry (VISION_POSITION_DELTA — wheel speed into the autopilot's EKF3).
// The delta is measured along the vehicle's OWN longitudinal axis, so no heading is needed and
// nothing inbound is subscribed: EKF3 rotates it with its own attitude. VISION_SPEED_ESTIMATE
// (earth-frame) was falsified on the 2026-08-21 bench — see
// openspec/changes/add-extnav-velocity/design.md.
#define MAVLINK_VISO_ENABLED           1     // compile-time switch; runtime switch is the autopilot's VISO_TYPE
#define MAVLINK_VISO_CONFIDENCE        100.0f  // 0-100 quality → EKF3 velErr = EK3_VIS_VERR_MIN..MAX; 100 picks
                                             // the MIN end (0.1 m/s default), well under the 1.0 aiding gate
#define MAVLINK_VISO_MAX_DT_US         1000000 // µs: a longer gap since the last send is treated as a break —
                                             // re-baseline (skip one interval) rather than integrate a stale dt
#define MAVLINK_VISO_NEUTRAL_ZERO_MS   0.14f // m/s — in NEUTRAL only readings below this are sent (as a
                                             // zero-motion update); above it the direction sign is
                                             // unrecoverable → stay silent

// ESP32 MAVLink identity (distinct component on the vehicle's system)
#define MAVLINK_SYSTEM_ID             1      // Same system as the autopilot
#define MAVLINK_COMPONENT_ID          25     // MAV_COMP_ID_USER1 (peripheral component)

// Command channel value range (microseconds) — SERVO_OUTPUT_RAW carries µs directly
#define RC_US_MIN     1100   // Minimum command microseconds
#define RC_US_MAX     1900   // Maximum command microseconds
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
// Position feedback: Bosch LWS steering-angle sensor 0 265 005 486 (PSA 9650236180)
// mounted DIRECTLY on the steering shaft, read over the shared MCP2515 CAN bus
// (frame 0x305, DLC 6, 100 Hz; decoder include/Lws305.h, health src/LwsSensor.cpp).
// It is absolute over several turns (measured range ≈ ±600°, beyond which it LATCHES
// fault code 9 until its 12 V — the ignition key — is power-cycled) and stores its own zero.
//  - SIGN: on a normal mount the sensor's angle is positive = LEFT (counter-clockwise) and
//    the firmware position is pos = −angle; on a mirrored mount (NVS flag `inv` = true) it is
//    pos = +angle. Either way positive = RIGHT, like IMotorDriver::setSpeed() and steering_pct.
//    The flag is inferred at limit capture (include/SteerSensorDir.h), never set by hand.
//  - UNITS: every steering position, limit, tolerance and gain below is in 0.1° of
//    steering-shaft angle (int32).
//  - CENTER = the sensor's own stored zero (angle 0.0), set with the web "Set sensor zero"
//    (0x7C0 [05 00], ~100 ms, [53 30] — see CANController). There is no centre capture.
//  - NVS namespace "steering" (shared with steer_sca_base — NEVER clear() it), layout v4:
//    cal_ver (=4), lim_l, lim_r (0.1°, firmware sign), inv (bool, true = mirrored). v3 limits
//    are kept with inv = false. Older keys (v2 fc_raw/cc_raw/fl_pos/cl_raw/fr_pos/cr_raw,
//    v1 c_ang/l_ang/r_ang) are removed one by one on first boot -> recalibration required.
//    Calibrated = both limits stored and valid.
//  - Percent commands need a healthy sensor + both limits; jog/nudge only need the VESC link
//    (sensorless open loop is allowed, with no stall backstop).
// The values marked bench-tune were converted from AS5600 counts (1 count ≈ 0.88 × 0.1°).
#define STEER_POSITION_TOLERANCE  2     // 0.1° (0.2°) - park window: driving stops (coast, the worm holds) within this;
                                        //      measured 2026-11-05 on the quad: coast after cut ≈ 0, a 0.4° window parked
                                        //      0.3–0.4° short on the approach side (fix-steering-jerk-and-centre-undershoot).
                                        //      If it overshoots / hunts: lower STEER_PWM_TRACK_MIN_DUTY first, then try 3
#define STEER_RESTART_TOLERANCE   10    // 0.1° (1.0°) - a parked loop drives again only beyond this (stop hysteresis);
                                        //      measured 2026-11-05 on the quad: no overshoot / no restarts from ±9.3°, 3.0° blocked small moves
#define STEER_MOVE_TIMEOUT        7500  // ms - max time continuously behind the target by more than STEER_RESTART_TOLERANCE
                                        //      (re-armed whenever within it) — bench-check a full lock-to-lock move
#define STEER_STALL_THRESHOLD     5     // 0.1° (0.5°) - progress below this over STEER_STALL_TIMEOUT = stall — bench-tune
#define STEER_STALL_TIMEOUT       500   // ms - time without position change = stall
#define STEER_CAL_MIN_SPAN        200   // 0.1° (20.0°) - min |limit| from the sensor zero accepted at capture, also the min |angle| that may toggle the sensor direction — bench-tune
#define STEER_BOOT_CENTER_TIMEOUT_MS  5000  // ms - a pending boot centering (waits for sensor + VESC link) is dropped after this

// Bosch LWS sensor health, range guard and zero-set
#define STEER_LWS_TIMEOUT_MS      100   // ms - no ACCEPTED 0x305 frame for this long = fault "timeout" (frames every 10 ms)
#define STEER_LWS_RECOVER_FRAMES  5     // consecutive good frames (≈50 ms) before the sensor is healthy again
#define STEER_LWS_SAFE_RANGE      5500  // 0.1° (550.0°) - closed-loop (percent) targets are clamped to ±this and
                                        // stopped driving outward beyond it, keeping clear of the ~600° latch.
                                        // NOT applied to jog/nudge/limit captures (operator's decision)
#define STEER_LWS_ZERO_GAP_MS     100   // ms - pause between 0x7C0 [05 00] and [53 30] (non-blocking)
#define STEER_LWS_ZERO_CONFIRM_MS 1000  // ms - after [53 30], wait this long for OK=1 fault=0 cal=1 |angle| ≤ tolerance
#define STEER_LWS_ZERO_TOLERANCE  10    // 0.1° (1.0°) - |angle| accepted as "zero confirmed"

// Proportional PWM control (include/SteerPositionLaw.h):
//   duty = clamp(max(|error| * STEER_KP, floor) + boost, floor, MAX), signed toward the target;
// floor = STEER_PWM_MIN_DUTY (break-away) from rest / after a reversal until 0.3° of progress
// (STEER_BOOST_MIN_PROGRESS), then STEER_PWM_TRACK_MIN_DUTY while tracking. The anti-stall boost
// adds on top of the floor (never hidden under it). Measured 2026-11-05 on the quad (USB diag,
// goto ±10): at a tracking floor of 70 a 10° move had 5–6 stop/go plateaus (17–27 A standing,
// 2.0–2.5 s per move), each broke free at duty 89–107; parked 0.3–0.4° short at a 0.4° window.
#define STEER_KP                  1.15f // PWM duty (0-255) per 0.1° of error (saturates at ≈22°) — bench-tune
#define STEER_PWM_MIN_DUTY        85     // break-away floor (≥ STEER_PWM_TRACK_MIN_DUTY): overcomes the worm's static friction (measured 2026-11-05 on the quad:
                                        // sustained break-away L 46–50 / R 42–44, 140 was ~3× too high);
                                        // used from rest until 0.3° of progress; also the rate-mode and jog floor
#define STEER_PWM_TRACK_MIN_DUTY  85     // floor while driving after break-away; must exceed the running friction incl. tyre scrub
                                        // (measured 2026-11-05 on the quad: plateaus at 70, broke free at 89–107) — bench-tune
                                        // 90 → 85 → 80 if the wheel overshoots the 0.2° park window
#define STEER_PWM_MAX_DUTY        255   // full speed for large errors
static_assert(STEER_PWM_MIN_DUTY >= STEER_PWM_TRACK_MIN_DUTY, "break-away floor must not be below the tracking floor");

// Anti-stall boost: steering load rises toward the locks; when commanded motion
// makes no progress, duty escalates above the P term until movement resumes.
// (Bench: closed-loop stalled ~72 counts short of the left lock at MIN duty.)
#define STEER_BOOST_CHECK_MS      100   // ms - progress check period
#define STEER_BOOST_MIN_PROGRESS  3     // 0.1° (0.3°) per check period considered "moving" — bench-tune
#define STEER_BOOST_STEP          30    // duty added per check when stalled (on top of the floor)
#define STEER_BOOST_DECAY         30    // duty removed per check when moving inside STEER_BOOST_HOLD_ERR (never below 0): fast shed inside 2° (10 let a held +30 reach centre: 0.7° overshoot, 2026-11-06)
// Boost hold band: while the wheel moves and |error| > this, the boost is held (no decay); a stalled
// check still adds STEER_BOOST_STEP anywhere. Measured 2026-11-06 on the loaded quad (~200 kg, dry
// steering, floors 70–100): outward 10° moves stalled at 35–44 A 2–3 times — +30 broke the wheel
// free, the −10/100 ms decay removed it within ≈300 ms and it stalled again (duty 103–150).
#define STEER_BOOST_HOLD_ERR      20    // 0.1° (2.0°) - bench-tune

// Calibration jog (momentary open-loop drive from the web UI)
#define STEER_JOG_DUTY            120   // PWM duty while jogging (default when the UI sends no speed; measured 2026-11-05 on the quad:
                                        // break-away 42–50, so 120 is a controllable jog)
#define STEER_JOG_TIMEOUT_MS      500   // ms - auto-stop if the jog command is not refreshed
#define STEER_NUDGE_DUTY          100   // PWM duty for a precision nudge pulse (measured 2026-11-05 on the quad: ≤ 100 × 70 ms does nothing)
#define STEER_NUDGE_MS            100   // ms - nudge pulse length (measured 2026-11-05 on the quad: 100 duty × 100 ms = 0.4–0.6° per tap)

// Sensorless rate mode (include/SteerRate.h): with the LWS unhealthy, the source MAVLink
// and the autopilot in MANUAL/unknown, the stick is a rotation-rate command. It adds NO tunable:
// it reuses RC_STEERING_DEADBAND (stop zone), STEER_PWM_MIN_DUTY / STEER_PWM_MAX_DUTY (duty =
// MIN + (MAX − MIN)·d²) and STEER_JOG_TIMEOUT_MS (stop without a fresh SERVO_OUTPUT_RAW).

// Continuous tracking + stall latch (MAVLink streams steering at ~25 Hz). While the loop is
// driving or parked, every percent command only replaces the target: the move timer, the stall
// window, the progress baseline, the boost and break-away are reset only by a start from rest
// (idle → driving, or parked → driving beyond STEER_RESTART_TOLERANCE). The stall window also
// restarts (boost → 0) on a drive-direction flip; the move timer is re-armed while the error is
// within STEER_RESTART_TOLERANCE. The stall latch is checked only on a command from idle.
#define STEER_STALL_COOLDOWN_MS   700   // ms — same-direction moves refused after a stall; opposite direction always allowed

// Park correction (include/SteerParkLaw.h, change add-steering-park-correction): a parking that
// ended between STEER_PARK_TRIGGER_TOL (0.4°) and STEER_RESTART_TOLERANCE (1.0°) off target is
// finished with a ladder of short open-loop pulses (setSpeed(dir × duty) then stop(), no
// STEER_PWM_MIN_DUTY floor), escalating only when the wheel did not move. Measured 2026-11-05 on
// the quad (USB steering diagnostics): sustained break-away L 46–50 / R 42–44, coast after the cut
// ≈ 0 (self-locking 1:40 worm), ≤ 100 × 70 ms moves nothing at standstill, 100 × 100 ms moves
// 0.4–0.6°, a closed-loop goto stops 0.2–0.4° short. Rolling tyres need less, hence rung 1 below
// the standstill threshold.
// Compile-time only (NOT web settings): these five decide WHEN the firmware moves the wheel by
// itself and how a move is judged. A short settle, a small gap or a move threshold below the
// sensor's 0.1° resolution gives pulses while driving or a buzzing wheel, which only the USB
// diagnostics can show — change them only with a reflash after a diagnostics run.
#define STEER_PARK_TRIGGER_TOL    4     // 0.1° (0.4°) — no correction within max(this, done tolerance); separate from the
                                        //      0.2° park window STEER_POSITION_TOLERANCE (fix-steering-jerk-and-centre-undershoot D5)
#define STEER_PARK_SETTLE_MS      1000  // ms — parked with a stable target this long before the first pulse
#define STEER_PARK_TARGET_STABLE  3     // 0.1° (0.3°) — a target move beyond this cancels and restarts the settle clock
#define STEER_PARK_PULSE_GAP_MS   500   // ms — wait after a pulse's end before its result is judged
#define STEER_PARK_MOVE_MIN       1     // 0.1° (0.1°) — smaller position change = "did not move" (escalate)
// Web settings (NVS "steering", owned by SteeringController): defaults and bounds
#define STEER_PARK_ENABLED_DEFAULT     true
#define STEER_PARK_DUTY1_DEFAULT       70    // rung 1 duty (0-255)
#define STEER_PARK_DUTY2_DEFAULT       85
#define STEER_PARK_DUTY3_DEFAULT       100
#define STEER_PARK_MS1_DEFAULT         80    // rung 1 length, ms
#define STEER_PARK_MS2_DEFAULT         90
#define STEER_PARK_MS3_DEFAULT         100
#define STEER_PARK_DONE_TOL_DEFAULT    2     // 0.1° (0.2°) — a correction ends within this
#define STEER_PARK_MAX_PULSES_DEFAULT  5     // pulses per parking (a done does not refill the budget)
#define STEER_PARK_DUTY_MIN            40
#define STEER_PARK_DUTY_MAX            150
#define STEER_PARK_MS_MIN              40    // ms
#define STEER_PARK_MS_MAX              200   // ms
#define STEER_PARK_DONE_TOL_MIN        1     // 0.1° (0.1°)
#define STEER_PARK_DONE_TOL_MAX        5     // 0.1° (0.5°)
#define STEER_PARK_MAX_PULSES_MIN      1
#define STEER_PARK_MAX_PULSES_MAX      10
// NVS keys in the EXISTING "steering" namespace (beside lim_l / lim_r / inv / cal_ver /
// steer_sca_base). Never clear() it; the v4 migration and eraseLimits() leave these keys alone.
#define STEER_PARK_EN_NVS_KEY     "park_en"    // bool
#define STEER_PARK_D1_NVS_KEY     "park_d1"    // uint8 duty, rung 1..3
#define STEER_PARK_D2_NVS_KEY     "park_d2"
#define STEER_PARK_D3_NVS_KEY     "park_d3"
#define STEER_PARK_T1_NVS_KEY     "park_t1"    // uint16 ms, rung 1..3
#define STEER_PARK_T2_NVS_KEY     "park_t2"
#define STEER_PARK_T3_NVS_KEY     "park_t3"
#define STEER_PARK_TOL_NVS_KEY    "park_tol"   // int32, 0.1°
#define STEER_PARK_MAX_NVS_KEY    "park_max"   // uint8

// VESC steering driver telemetry monitor + failsafe
// (Over-current is left to the VESC itself: Motor Current Max clamp, Absolute Max
//  Current fault — surfaced here as a fault code — and MOSFET temperature limiting.
//  Mechanical jams are caught by the steering-angle stall detector, STEER_STALL_TIMEOUT,
//  while the sensor is healthy; sensorless jog relies on the VESC current clamp + fault stop.)
#define STEER_VESC_TELEM_MS         300   // ms — COMM_GET_VALUES poll period (~3 Hz)
#define STEER_VESC_COMM_TIMEOUT_MS  1000  // ms — no valid GET_VALUES reply for this long -> driver fault (stop, reject moves)
#define VESC_RX_GAP_MS              5     // ms — byte gap that ends an incomplete frame (VescRxParser resync)
#define VESC_LINK_DOWN_SNAP_MS      10000 // ms — journal `VESC W link_down_snap` period while the link is down
// Steering VESC input voltage below this = "low" (telemetry steer_vin_low → web "Увага" warning only,
// no control action). 21.5 V ≈ 3.58 V/cell on a 6s NMC pack, ≈ 3.07 V/cell on a 7s LiFePO4 pack —
// low but not yet harmful for either. Adjust per vehicle build.
#define STEER_VIN_LOW_V             21.5f // V

// ----------------------------------------------------------------------------
// STEERING DIAGNOSTICS (USB serial) — include/SteerDiag.h, include/SteerDiagLaw.h,
// host script tools/steer_diag.py (change add-steering-serial-diagnostics).
// Line commands on the USB-CDC `Serial` console only (never UART0 / UART2). While
// `diag on` the console owns the steering: external commands are refused.
// ----------------------------------------------------------------------------
#define STEER_DIAG_LINE_MAX          96     // bytes — longest accepted command line (longer lines are dropped)
#define STEER_DIAG_LOG_MS            20     // ms — D-line period (50 Hz) while an experiment runs
#define STEER_DIAG_KEEPALIVE_MS        100    // ms — `K` line period inside the keepalive window (HWCDC stuck-TX workaround)
#define STEER_DIAG_KEEPALIVE_WINDOW_MS 30000  // ms — keepalive while diag on and this long after the last command line
#define STEER_DIAG_REFRESH_MS        100    // ms — open-loop duty re-sent to the VESC at least this often
#define STEER_DIAG_VESC_TELEM_MS     50     // ms — COMM_GET_VALUES period while diag is on (normal: STEER_VESC_TELEM_MS)
#define STEER_DIAG_IDLE_TIMEOUT_MS   60000  // ms — automatic `diag off` (motor stopped) without any command line
#define STEER_DIAG_HOST_LOST_MS      500    // ms — `(bool)Serial` false this long (after it was true once since diag on) = USB host lost
#define STEER_DIAG_MAX_SPEED_KMH     0.5f   // km/h — diag on / experiments refused, running ones aborted, above this
#define STEER_DIAG_LIMIT_MARGIN      20     // 0.1° (2.0°) — abort when driving to within this of a captured limit
#define STEER_DIAG_MOVE_POS          5      // 0.1° (0.5°) — ramp: |angle − start| at which the wheel counts as moving
#define STEER_DIAG_MOVE_RATE         5      // °/s — ramp: LWS rate (toward the drive) at which the wheel counts as moving
#define STEER_DIAG_SETTLE_MS         600    // ms — logging after the duty is cut (coast) / after the loop stops (goto)
#define STEER_DIAG_RAMP_TIMEOUT_MS   15000  // ms — ramp hard timeout
#define STEER_DIAG_RAMP_MAXHOLD_MS   1000   // ms — ramp ends "no_move" after this long at its max duty without movement
#define STEER_DIAG_GOTO_TIMEOUT_MS   12000  // ms — goto hard timeout (longer than STEER_MOVE_TIMEOUT)
#define STEER_DIAG_PULSE_MAX_MS      1000   // ms — longest `pulse`
#define STEER_DIAG_HOLD_MAX_MS       1500   // ms — longest `hold`
#define STEER_DIAG_GOTO_MAX_DEG      60.0f  // °  — |goto| target bound (the targets are also clamped to the limits)

// Throttle Servo Parameters
#define THROTTLE_SERVO_MIN_US    800   // Minimum servo pulse width (µs) — mechanical range floor / slider bound
#define THROTTLE_SERVO_MAX_US    2200  // Maximum servo pulse width (µs) — mechanical range ceiling / slider bound
// Throttle calibration: 0..100% maps linearly from idle_us to full_us (either order: full_us < idle_us
// on a mirrored servo mount, e.g. the aztec buggy 2200 -> 800). These endpoints are runtime-editable
// via the web calibration flow and persisted in NVS ("throttle" namespace); the values below are
// only the defaults used on first boot / when stored values are missing or invalid.
#define THROTTLE_DEFAULT_IDLE_US 901   // idle pulse width (previously 13° over the 800–2200µs range)
#define THROTTLE_DEFAULT_FULL_US 1344  // full-throttle pulse width (previously 70° over the 800–2200µs range)
#define THROTTLE_MIN_SPAN_US     100   // minimum accepted |full_us - idle_us| separation (full may be below idle on a mirrored servo)
// Throttle OPENING rate limit (anti-stall; include/ThrottleSlewLaw.h, applied in
// ThrottleController::setThrottlePercent). Snapping the throttle open from idle stalls the engine
// (air before fuel, the CVT takes load before the RPM rises). Closing is never rate-limited.
// Range: the engine needs ≈0.5–1 s from idle before it can take load, so a 1–2 s idle→full ramp
// is the useful window; faster than 100 %/s brought the stall back; 30 %/s (3.3 s) is the slowest
// still drivable. 0 = limit off (exactly the old snap). Per-vehicle, NVS "throttle"/open_slew.
#define THROTTLE_OPEN_SLEW_DEFAULT_PCT_S 60.0f   // %/s - idle→full in 1.67 s, 0→30 % in 0.5 s
#define THROTTLE_OPEN_SLEW_MIN_PCT_S     30.0f   // %/s - lowest accepted non-zero rate (3.3 s to full)
#define THROTTLE_OPEN_SLEW_MAX_PCT_S     100.0f  // %/s - highest accepted rate (1 s to full)
#define THROTTLE_OPEN_SLEW_NVS_KEY       "open_slew"  // NVS "throttle" namespace, float %/s (owned by ThrottleController)
#define THROTTLE_OPEN_SLEW_MAX_DT_S      0.1f    // s - cap on the elapsed time per write, so a silent path never steps open

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

#define TRANS_OVERSHOOT_DWELL_MS            2000    // ms - overshoot dwell TIMEOUT: ends earlier on sensor confirmation, else gear assumed engaged
#define TRANS_SEQUENCE_STEP_DWELL_MS        1000    // ms - inter-step pause TIMEOUT: ends earlier when the sensor confirms the completed step
#define TRANS_DWELL_CONFIRM_MS              200     // ms - sensor must read the expected gear continuously this long to end a dwell / pause early (300–400 if the linkage bounces)

// Default gear positions (percent: 0.0 = 800µs, 100.0 = 2200µs)
#define TRANS_GEAR_DEFAULT_REVERSE_PCT      40.0f
#define TRANS_GEAR_DEFAULT_NEUTRAL_PCT      48.0f
#define TRANS_GEAR_DEFAULT_LOW_PCT          68.0f
#define TRANS_GEAR_DEFAULT_HIGH_PCT         58.0f

// Safety limits
#define TRANS_UNKNOWN_GEAR_THROTTLE_MAX     (float)8.5   // % - max throttle when physical gear UNKNOWN
#define TRANS_GEAR_CHECK_INTERVAL           500        // ms - physical gear verification period
#define TRANS_GEAR_READ_INTERVAL_MS         100        // ms - GPIO debounce cache for getPhysicalGear()
#define TRANS_GEAR_READ_RETRY_COUNT         3          // extra reads when all switches read inactive while servo is idle
#define TRANS_RESYNC_CONFIRM_CHECKS         2          // consecutive idle checks with the same sensor gear != target before adopting it
#define TRANS_INIT_SETTLE_MS                1500       // ms - init-step overshoot move from an unknown servo position (max 20 % + 1 % overshoot ≈ 0.75 s at 35 ms/%, doubled)

// ============================================================================
// BRAKE SYSTEM CONFIGURATION
// ============================================================================

// Brake Movement Parameters
// BRAKE_FULL_TRAVEL_TIME and BRAKE_HOLD_SPEED are the DEFAULTS of the per-vehicle, web-tunable
// values (change add-brake-tuning-card; NVS "brake"/travel_ms and hold_pwm, owned by
// VehicleController, law in include/BrakeLaw.h — keep its literals in sync).
#define BRAKE_FULL_TRAVEL_TIME 1500  // ms - default estimated time for 0-100% brake travel (measured on the quad)
#define BRAKE_TOLERANCE        1  // % - position tolerance for "at target"
#define BRAKE_SENSOR_OVERRUN_TIME 1000  // ms - continue moving after sensor triggers (full retraction)
#define BRAKE_HOLD_SPEED          30    // PWM (0-255) default push against the hydraulic return when at target position
// Travel 1000–3000 ms around the measured 1500 ms: covers a faster or slower actuator on the other
// vehicle; the cap also bounds how long a too-long setting stalls the actuator at full drive.
#define BRAKE_TRAVEL_MS_MIN    1000  // ms
#define BRAKE_TRAVEL_MS_MAX    3000  // ms
// Hold 0–100 on the 0–255 PWM scale: 0 = no push (coast); above ~100 the stalled actuator and the
// BTS7960 heat up for as long as the brake is held, so the cap is well below full drive.
#define BRAKE_HOLD_PWM_MIN     0
#define BRAKE_HOLD_PWM_MAX     100
#define BRAKE_NVS_NAMESPACE    "brake"
#define BRAKE_TRAVEL_NVS_KEY   "travel_ms"   // uint16, ms
#define BRAKE_HOLD_NVS_KEY     "hold_pwm"    // uint8, PWM 0-255 scale

// ============================================================================
// VEHICLE SPEED SENSOR (12V toothed-ring pickup, PCNT on PIN_SPEED_SENSOR)
// ============================================================================

// Sampling / signal conditioning
#define SPEED_SAMPLE_INTERVAL_MS      200    // ms between PCNT samples (5 Hz, matches telemetry)
// No edges for this long => the reported speed decays to 0 m/s. With the measured
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
#define SPEED_MAX_PULSES_PER_SAMPLE   20000  // pulses per sample above which the reading is a counter glitch, not motion

// Odometer / trip persistence interval (NVS namespace "speed", keys "odo_mm" / "trip_mm").
// The counters are flushed every kilometre of odometer growth, plus once on every ignition-OFF
// transition. That bounds the loss from a power cut to one kilometre while keeping the write
// budget negligible: NVS is wear-levelled and log-structured, so ~63 writes of the two uint64
// keys fill a 4096-byte page and cost one erase — ≈160 erases over 10 000 km, ≈0.16 % of the
// flash's ~100 000-cycle endurance.
#define ODO_NVS_WRITE_INTERVAL_MM     1000000ULL  // mm (1 km) of ODO growth between NVS writes

// Calibration defaults (NVS namespace "speed", keys "ppr" / "circ_mm"), both
// bench-measured on the vehicle and both settable at runtime from the web UI
// (speed_cal_ppr / speed_cal_circ) — no reflash needed to change them. The same namespace also
// holds the speed-limiter floor `thr_floor` (SPEED_LIMIT_FLOOR_NVS_KEY), owned and written by
// VehicleController, not SpeedSensor — never clear() the namespace.
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
#define SPEED_MAX_PLAUSIBLE_DECEL_MS2    2.2f   // m/s² (≈8 km/h per second)

// Maximum-speed throttle limiter. The ceiling has exactly ONE source: the autopilot's SPEED_MAX
// parameter (m/s), held in RAM only. There is no local ceiling, no ceiling NVS key and no master
// switch (the persisted floor below shapes HOW the limit is enforced, it never imposes one) —
// the limiter is always armed, and "no usable SPEED_MAX" (link down, stale, never received) or
// SPEED_MAX = 0 simply means NO LIMITING, exactly as ArduPilot itself reads a zero.
#define SPEED_LIMIT_WARN_MS           5000   // ms between "limiter armed but speed invalid" warnings

// Proportional taper + integral trim. Authority is withdrawn GRADUALLY as the limit is approached
// rather than in one step at the limit: a step lurches a heavy vehicle, invites hunting around the
// threshold, and does both exactly where it hurts most — mid-corner. The pure law lives in
// include/SpeedLimiterLaw.h (host-tested by test/test_speed_limiter).
//  - The taper band is proportional to the limit: band = clamp(0.20 × L, 0.5, 1.4) m/s, so a low
//    limit withdraws authority close to the limit, not from half of it.
//  - The ceiling is taper + an integral trim, clamped to [floor, 100]: above the limit the trim
//    keeps closing the throttle; below it, while the limiter binds, the trim gives back exactly the
//    authority that holds the limit — no proportional droop, whatever the engine, CVT or grade.
//  - The floor defaults to 0 % = the CALIBRATED IDLE position (ThrottleController::percentToUs(0)),
//    persisted per vehicle (0–20 %). That is not a hard cut: the ceiling only reaches the floor
//    through the taper, the trim and the ceiling slew, never as a step.
//  - The ceiling slew (both directions) is also persisted per vehicle (10–200 %/s, default 80 %/s:
//    100 → 0 takes ≥ 1.25 s); lower = gentler closing and return of the throttle. It caps the
//    trim's fall as well (inside the law), so a gentler slew also slows how fast it can close.
#define SPEED_LIMIT_TAPER_FRACTION     0.20f  // band = 0.20 × limit — the limiter takes the last fifth
#define SPEED_LIMIT_TAPER_MIN_MS       0.5f   // m/s - ≈3.5 speed quanta (0.142 m/s); narrower hunts
#define SPEED_LIMIT_TAPER_MAX_MS       1.4f   // m/s (≈5 km/h) - the old fixed band, proven at road limits
#define SPEED_LIMIT_FLOOR_DEFAULT_PCT  0.0f   // % - default floor = calibrated idle; below any holding throttle
#define SPEED_LIMIT_FLOOR_MIN_PCT      0.0f   // % - accepted floor range (web command / NVS validation)
#define SPEED_LIMIT_FLOOR_MAX_PCT      20.0f  // % - above this the floor itself becomes a hidden speed floor
#define SPEED_LIMIT_FLOOR_NVS_KEY      "thr_floor"  // NVS "speed" namespace, float % (owned by VehicleController)
#define SPEED_LIMIT_SLEW_DEFAULT_PCT_S 80.0f  // %/s - default max ceiling movement (both directions) — no servo snap.
                                              // Applied to the CEILING, not the demand: the driver's own
                                              // stick moves are never slowed by the limiter.
#define SPEED_LIMIT_SLEW_MIN_PCT_S     10.0f  // %/s - accepted slew range (web command / NVS validation)
#define SPEED_LIMIT_SLEW_MAX_PCT_S     200.0f // %/s - 100 → 0 in 0.5 s; faster is a step in all but name
#define SPEED_LIMIT_SLEW_NVS_KEY       "thr_slew"   // NVS "speed" namespace, float %/s (owned by VehicleController)
#define SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS 30.0f // %/s per m/s - 1.4 m/s over takes the trim 100 → 0 in ≈2.4 s
#define SPEED_LIMIT_TRIM_RISE_PCT_S    10.0f  // %/s - trim recovery cap; far below the slew, never a surge
#define SPEED_LIMIT_BIND_MARGIN_PCT    1.0f   // % - demand within this of the ceiling = limiter binding
                                              // (the trim only rises while binding: anti-windup)
// Saturation indicator (web console only — no serial log line, no MAVLink message): the ceiling is
// at the floor yet the vehicle stays above the limit, i.e. the floor is too high or it is downhill.
#define SPEED_LIMIT_SAT_MARGIN_MS      0.3f   // m/s (≈1 km/h, ~2 speed quanta) above the limit to count
#define SPEED_LIMIT_SAT_HOLD_MS        2000   // ms the condition must hold before speed_limit_sat (entry debounce)
#define SPEED_LIMIT_SAT_UI_HOLD_MS     10000  // ms speed_limit_sat_held stays true after it clears (display hold)
#define SPEED_LIMIT_LOG_MIN_MS         1000   // ms hold-off between "speed limit" logs
#define SPEED_LIMIT_LOG_EPSILON_MS     0.015f // m/s (≈0.05 km/h) - smaller ceiling moves are not worth a log

// Steering speed scaling. ArduPilot's own formula, scale = min(1, base / speed), moved to the
// ESP32 because the autopilot's speed silently falls back to raw GPS ground speed whenever EKF3
// has no velocity estimate — which on this vehicle is exactly when the ESP32's hall sensor has
// failed (observed 2026-09-24: parked, GPS 56 m/s, steering scaled to 0.018). Applies to the
// AUTOPILOT steering path only, in MANUAL only, and ANY fault resolves to scale = 1: no hold, no
// substitute speed, no minimum floor. A vehicle with full steering authority is the known-good
// state, so every one of those would be a way for a sensor fault to leave the driver unable to
// turn. Requires MANUAL_OPTIONS = 0 on the autopilot (an operator step — see MAVLINK_SETUP.md).
#define STEER_SCALE_SLEW_PER_S        2.0f   // scale units/s, both directions — a full 0 → 1 sweep in
                                             // 0.5 s: fast enough not to feel laggy, slow enough not
                                             // to snap the wheel. Applied to the SCALE, never to the
                                             // steering command (the driver's own moves pass at full rate)
#define STEER_SCALE_BASE_MAX_MS       10.0f  // m/s — matches MOT_SPD_SCA_BASE's range so a local value
                                             // and an autopilot value are interchangeable
#define STEER_SCALE_LOG_MIN_MS        1000   // ms hold-off between "steer scale" logs
#define STEER_SCALE_LOG_EPSILON       0.02f  // smaller scale moves are not worth a log
#define STEER_SCALE_WARN_MS           10000  // ms between "no base" / "speed invalid" warnings,
                                             // modelled on SPEED_LIMIT_WARN_MS
#define STEER_SCALE_TEST_SPEED_MS     60000  // ms before a bench set_test_speed override clears itself —
                                             // a fuse, so a bench setting can never become a driving one
#define STEER_SCALE_TEST_SPEED_MAX_MS 30.0f  // m/s — accepted range for the bench override
#define ROVER_CUSTOM_MODE_MANUAL      0      // ArduPilot Rover HEARTBEAT.custom_mode for MANUAL —
                                             // the ONLY mode in which the ESP32 scales steering

// NVS key for the locally stored speed-scaling base (m/s). It lives in the EXISTING "steering"
// namespace alongside the steering calibration (cal_ver / fc_raw / cc_raw / fl_pos / cl_raw /
// fr_pos / cr_raw — never clear() the namespace), because it is a property
// of the steering axis, not of the speed sensor. 0 or absent = "not set" → fall back to the
// autopilot's MOT_SPD_SCA_BASE, and to no scaling at all when that is unavailable too.
#define STEER_SCALE_BASE_NVS_KEY      "steer_sca_base"

// PRESENTATION ONLY. Every speed inside the firmware is m/s; this exists solely for the web JSON
// and human-readable debug strings, and must never appear in a control-path computation.
#define MS_TO_KMH                      3.6f

// ============================================================================
// SERIAL DEBUG
// ============================================================================

// The console is the ESP32-S3 native USB CDC device on the board's USB-C connector
// (build flag ARDUINO_USB_CDC_ON_BOOT=1). UART0 (GPIO43/44) is RESERVED for the
// CH9121T wired LAN on the quad bike (the steering VESC link on aztec) and must
// never be reclaimed for a console.
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

#define WIFI_AP_SSID          "QuadBike-Control"  // WiFi AP SSID base; "-XXXX" chip suffix (eFuse MAC) appended at boot
#define WIFI_AP_PASSWORD      ""                  // No password (open network)
#define WIFI_AP_CHANNEL       1                   // WiFi channel (1-13)
#define WIFI_AP_MAX_CLIENTS   5                   // Maximum simultaneous clients
#define WIFI_AP_IP            IPAddress(192, 168, 10, 1)    // ESP32 IP address
#define WIFI_AP_GATEWAY       IPAddress(192, 168, 10, 1)    // Gateway IP
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

#define OTA_HOSTNAME          "quadbike-control"  // OTA hostname base; "-xxxx" chip suffix appended at boot
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
#define CAN_REINIT_NO_DATA_MS        15000 // ms - full chip re-init when polling but no valid OBD response for this long (heals a wedged TX / latched ABAT once the ECU powers up); only while the ignition is ACC/IGNITION, window restarts at switch-on (CanWatchdogLaw.h)
#define CAN_REINIT_MIN_INTERVAL_MS   15000 // ms - minimum spacing between forced re-inits
// LWS silence watchdog (change add-event-journal-and-link-recovery D10). Runs only while the
// ignition is ACC or IGNITION (the LWS is fed from the ACC line) and arms on the first 0x305
// frame since the ignition was switched on. Fixed spacing, no back-off.
#define CAN_LWS_SILENT_MS            1000  // ms - no 0x305 frame for this long = LWS path silent
#define CAN_LWS_WD_OBD_ALIVE_MS      2000  // ms - an OBD reply within this window proves the bus works
#define CAN_LWS_REINIT_MIN_MS        5000  // ms - min spacing from the last re-init of any kind
#define CAN_REINIT_MAX_ATTEMPTS      3     // re-init attempts per trigger (retried on later update() calls)
#define CAN_REINIT_RETRY_MS          100   // ms - spacing between the attempts of one trigger
#define CAN_FIRST_FRAME_WAIT_MS      10000 // ms - give up waiting for the first LWS/OBD frame after a re-init
#define CAN_WD_REPEAT_LOG_MS         600000 // ms - journal a still-silent watchdog episode again after this long (with a count)

// MCP2515 hardware acceptance filtering (standard 11-bit IDs only), two paths on ONE bus.
// The ECU floods the bus with broadcast frames (Delphi MT05 sends 0x301 etc.) which
// overrun the two RX buffers faster than the loop can drain them, so only two things
// are accepted, each through its own buffer:
//   RXB0: MASK0 0x7F8, RXF0/RXF1 0x7E8      -> OBD-II responses 0x7E8-0x7EF
//   RXB1: MASK1 0x7FF, RXF2..RXF5 0x305     -> Bosch LWS steering angle (100 Hz)
// so the 100 Hz angle stream and the OBD replies never evict each other. BUKT (RXB0 ->
// RXB1 rollover, set by the library) stays on: an OBD reply arriving while RXB0 is full
// lands in RXB1; the only loss is one LWS frame on RX1OVR, which is harmless.
// The coryjfowler MCP_CAN library expects standard mask/filter IDs shifted left by 16
// (mcp2515_write_mf() takes bits 26:16 as the 11-bit ID when ext == 0).
#define CAN_RX_OBD_MASK           0x7F8   // 11-bit acceptance mask (care bits 10:3)
#define CAN_RX_OBD_FILTER         0x7E8   // 11-bit acceptance filter (OBD-II ECU responses)
#define CAN_RX_OBD_MASK_REG   (((uint32_t)CAN_RX_OBD_MASK) << 16)    // 0x07F80000
#define CAN_RX_OBD_FILTER_REG (((uint32_t)CAN_RX_OBD_FILTER) << 16)  // 0x07E80000
#define CAN_RX_LWS_MASK           0x7FF   // 11-bit mask: exact match
#define STEER_LWS_CAN_ID          0x305   // Bosch LWS steering-angle frame (standard ID)
#define STEER_LWS_CFG_ID          0x7C0   // LWS configuration frame — sent ONLY by the zero-set sequence
                                          // in CANController, with ONLY [05 00] / [53 30] (never [03 00])
#define CAN_RX_LWS_MASK_REG   (((uint32_t)CAN_RX_LWS_MASK) << 16)    // 0x07FF0000
#define CAN_RX_LWS_FILTER_REG (((uint32_t)STEER_LWS_CAN_ID) << 16)   // 0x03050000

// ECU module (12 V battery) voltage below this = "low" (telemetry ecu_voltage_low → web "Увага"
// warning only). Lead-acid starter battery at rest: 12.0 V ≈ 50 % state of charge.
#define ECU_VOLTAGE_LOW_V         12.0f // V

// ECU Capability Probe (on-demand diagnostic sweep)
#define CAN_PROBE_RETRY_ATTEMPTS  1     // Retries per probe request (keeps worst-case sweep duration bounded)
#define PROBE_RESULT_TTL          8000  // ms - window during which completed probe results are embedded in telemetry

// CAN bus sniffer (on-demand, standstill diagnostic: all IDs accepted for a fixed window)
#define CAN_SNIFF_WINDOW_MS        10000 // ms - window used by the web "CAN sniffer (10 s)" button
#define CAN_SNIFF_MAX_WINDOW_MS    60000 // ms - upper clamp for startSniff()
#define CAN_SNIFF_MAX_IDS          32    // distinct IDs kept in the results table
#define CAN_SNIFF_DRAIN_PER_UPDATE 32    // frames read per update() while sniffing (5 in normal operation)
#define SNIFF_RESULT_TTL           30000 // ms - completed sniffer results stay in telemetry this long

// Transmission Safety (CAN-based)
#define TRANS_SPEED_INTERLOCK_THRESHOLD_MS  1.4f  // m/s (≈5 km/h) - Block gear changes above this speed
#define TRANS_CAN_TIMEOUT                5000  // ms - Allow gear change if CAN fails this long

// PID-Controlled RPM Boost During Gear Changes
#define TRANS_GEAR_BOOST_TARGET_RPM      2100  // RPM target to hold during gear change
#define TRANS_GEAR_BOOST_MAX_PCT         27.0f // % - max throttle PID can command (safety cap)
#define TRANS_GEAR_BOOST_TIMEOUT         5000  // ms - max boost duration (safety escape)
#define TRANS_GEAR_BOOST_PID_KP          0.015f // proportional gain (tunable)
#define TRANS_GEAR_BOOST_PID_KI          0.01f // integral gain (tunable)
#define TRANS_GEAR_BOOST_PID_KD          0.0f  // derivative gain (0 = disabled; noisy on RPM signal)
#define TRANS_GEAR_BOOST_SLEW_RATE_US    16    // µs per CAN update
#define BOOST_ENABLED_NVS_KEY            "enabled" // NVS "boost" namespace, bool — gear-change RPM boost on/off
#define BOOST_ENABLED_DEFAULT            true  // absent key → boost on (quad behaviour); off → idle hold during the shift

// ============================================================================
// EVENT JOURNAL (LittleFS /events.log + /events.1, change add-event-journal-and-link-recovery)
// ============================================================================

#define JOURNAL_FILE_MAX_BYTES     131072  // bytes per journal file (active + one rotated)
#define JOURNAL_LINE_MAX           120     // bytes per line incl. '\n'; longer lines end with '~'
#define JOURNAL_RAM_BUF_BYTES      4096    // unflushed lines held in RAM
#define JOURNAL_FLUSH_MS           1000    // ms — at most one flash flush per interval
#define JOURNAL_FLUSH_DEFER_MAX_MS 10000   // ms — max flush deferral while the steering motor drives
#define JOURNAL_DEDUPE_MS          2000    // ms — same (source, event) written at most once per window
#define JOURNAL_SNAP_MS            60000   // ms — SNAP line period
#define JOURNAL_RTC_RING           16      // unflushed lines kept in RTC memory across a reset
#define JOURNAL_LOOP_OVERRUN_MS    50      // ms — loop iteration longer than this is journaled
#define JOURNAL_HEAP_WARN_BYTES    40960   // bytes — free heap below this = heap_low (40 KB)
#define JOURNAL_HEAP_HYST_BYTES    8192    // bytes — heap_ok once free heap is this far above the threshold

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

// Firmware version string (semantic versioning: MAJOR.MINOR.PATCH)
// Update this constant when releasing new firmware versions, and add a matching
// entry to include/Changelog.h — the web UI shows both.
#define FIRMWARE_VERSION "1.5.28"

// Hardware this build targets. One value per hardware branch ("Control_v0" on
// master, "DevKitC-1" on old-board). Reported next to the version in telemetry
// so the web UI can tell which board the running firmware was built for.
#define BOARD_NAME "Control_v0"

#endif // CONSTANTS_H
