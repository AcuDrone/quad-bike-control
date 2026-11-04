#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// ============================================================================
// GPIO PIN ASSIGNMENTS — hand-wired ESP32-S3-DevKitC-1
// ============================================================================
//
// WIRING AUTHORITY: GPIO_PINOUT_S3.md. Relays are driven straight from GPIO,
// gear switches and the brake limit sensor are read straight with digitalRead()
// — there are no I2C port expanders and no 24V boost rail on this board.
// The debug console is UART0 (GPIO43/44) via the DevKit's USB-UART bridge.
//
// Pins that deliberately have NO firmware function on this board:
//   GPIO18    — former PIN_STEER_LPWM (BTS7960 steering), RESERVED/FREE.
//   GPIO41/42 — former AS5600 steering-sensor I2C (SDA/SCL). The AS5600 was physically
//               removed: steering feedback is the Bosch LWS on the CAN bus. RESERVED/FREE,
//               no I2C bus is started.
//   GPIO43/44 — UART0 debug console (DevKit USB-UART bridge, "COM" port).
// GPIO19/20 are NOT the USB console: they carry the R/N gear switches (see below),
// which is why the native USB CDC console must never be enabled on this board.

// MAVLink telemetry link (UART to Pixhawk TELEM2)
#define PIN_MAVLINK_RX      GPIO_NUM_8   // UART1 RX (non-inverted)
#define PIN_MAVLINK_TX      GPIO_NUM_15  // UART1 TX (non-inverted) — confirmed free; do NOT use GPIO 9 (PIN_TRANS_SERVO)
#define MAVLINK_UART_NUM    UART_NUM_1   // UART1 for MAVLink
#define MAVLINK_BAUD_RATE   115200       // Pixhawk TELEM2 baud

// Transmission Servo (HappyModel Super400 Plus)
#define PIN_TRANS_SERVO     GPIO_NUM_9   // LEDC Channel 2

// Steering Actuator — driven by a Flipsky 75200 VESC over UART2 (brushed-DC mode).
// See "VESC STEERING DRIVER CONFIGURATION" below.
//
// The former BTS7960 steering PWM pins (GPIO17/18) and their LEDC channels (6/7)
// were freed when the BTS7960 driver was removed. GPIO17 is now the hall speed
// sensor input; GPIO18 and LEDC channels 6/7 stay reserved/free.
#define LEDC_CH_STEER_RPWM  6             // RESERVED/FREED
#define LEDC_CH_STEER_LPWM  7             // RESERVED/FREED

// VESC steering driver UART (UART_NUM_2; UART0 = debug console, UART1 = MAVLink)
#define PIN_VESC_TX         GPIO_NUM_4    // ESP TX -> VESC RX (3.3V logic, direct wiring)
#define PIN_VESC_RX         GPIO_NUM_5    // ESP RX <- VESC TX
#define VESC_UART_NUM       2             // UART_NUM_2
#define VESC_UART_BAUD      115200        // VESC app default baud

// Steering angle feedback: Bosch LWS on the CAN bus (see "CAN CONTROLLER CONFIGURATION"
// below). The former AS5600 I2C pins GPIO41 (SDA) / GPIO42 (SCL) are RESERVED/FREE —
// nothing is wired to them and the firmware never opens an I2C bus.

// Throttle Servo (PWM via LEDC)
#define PIN_THROTTLE_PWM    GPIO_NUM_3   // LEDC Channel 1
#define LEDC_CH_THROTTLE    1

// Brake Linear Actuator (BTS7960)
// Note: R_EN and L_EN hardwired to 5V (always enabled)
#define PIN_BRAKE_RPWM      GPIO_NUM_7   // LEDC Channel 4
#define PIN_BRAKE_LPWM      GPIO_NUM_6   // LEDC Channel 5
#define LEDC_CH_BRAKE_RPWM  4
#define LEDC_CH_BRAKE_LPWM  5

// SPI CAN Controller (MCP2515 + transceiver, 8 MHz crystal → MCP_8MHZ). One shared
// 500 kbit/s bus: the engine ECU (OBD-II 0x7DF/0x7E8) AND the Bosch LWS steering
// angle sensor (0x305, 100 Hz). See "CAN CONTROLLER CONFIGURATION" for the filter plan.
// The MCP2515 INT line is not wired on this board — the driver polls.
#define PIN_CAN_MOSI        GPIO_NUM_11  // SPI MOSI
#define PIN_CAN_MISO        GPIO_NUM_13  // SPI MISO
#define PIN_CAN_SCK         GPIO_NUM_12  // SPI SCK
#define PIN_CAN_CS          GPIO_NUM_10  // SPI Chip Select

// Relays — driven directly from GPIO (active HIGH), no port expander
#define PIN_RELAY1      GPIO_NUM_36   // Relay 1 — ignition/ECU line
#define PIN_RELAY2      GPIO_NUM_37   // Relay 2 — starter solenoid
#define PIN_RELAY3      GPIO_NUM_38   // Relay 3 — front light
#define PIN_WHEEL_LOCK  GPIO_NUM_39   // Relay 4 — front-wheel lock (also JTAG MTCK; JTAG unused here)

// Gear position switches and brake limit sensor — direct inputs, read with digitalRead().
// Gear switches: floating inputs with EXTERNAL pull-ups on the harness, active LOW.
// Brake limit sensor: plain INPUT, HIGH = brake fully released.
#define PIN_GEAR_REVERSE  GPIO_NUM_19
#define PIN_GEAR_NEUTRAL  GPIO_NUM_20
#define PIN_GEAR_LOW      GPIO_NUM_21
#define PIN_GEAR_HIGH     GPIO_NUM_47
#define PIN_BRAKE_SENSOR  GPIO_NUM_14

// Driveline speed sensor (hall) pulse input, PCNT counted.
// Freed when the BTS7960 steering driver was removed (was PIN_STEER_RPWM).
// Wiring CONFIRMED on the bench 2026-09-29: 12V toothed-ring hall pickup → 1 kΩ series
// resistor → PC817 opto LED; PC817 phototransistor collector → this pin, emitter → GND.
// The PC817 stage INVERTS and does the level shifting, so nothing 12V reaches the pin and the
// signal here is ACTIVE LOW (LED conducting ⇒ pin pulled low). The counted edge is the falling
// one, i.e. the start of the active phase.
// ⚠ An external 1 kΩ pull-up from this pin to 3.3V is MANDATORY. The first build had none and
// relied on the ESP32-S3 internal pull-up (~45 kΩ) alone; that leaves a high-impedance node
// between pulses, which picked up ignition-coil interference as phantom pulses and tripped the
// gear interlock. With the 1 kΩ fitted the noise counters read 0 with the engine idling.
#define PIN_SPEED_SENSOR    GPIO_NUM_17
// GPIO18 (former PIN_STEER_LPWM) stays RESERVED/FREE.

// ============================================================================
// RELAYS (direct GPIO)
// ============================================================================
//
// RelayController keeps one shadow byte for the four relays, one bit per relay
// (bit0 = Relay 1 … bit3 = Relay 4), and writes it out to the four GPIOs above.
// A GPIO write cannot fail silently, so there is no read-back verification and no
// starter escalation on this board (those belong to the I2C relay board only).
// Each function drives exactly ONE relay here (no paralleled pairs).
#define RELAY_COUNT             4           // relays wired on this board (Relay 1..4)
#define RELAY_MASK_IGNITION     0b0001      // Relay 1 (GPIO36) — ignition/ECU line
#define RELAY_MASK_STARTER      0b0010      // Relay 2 (GPIO37) — starter solenoid
#define RELAY_MASK_FRONT_LIGHT  0b0100      // Relay 3 (GPIO38) — front light
#define RELAY_MASK_WHEEL_LOCK   0b1000      // Relay 4 (GPIO39) — front-wheel lock
#define RELAY_MASK_RESERVED     0b0000      // no spare relays on this board

// Web relay test (bench wiring check). Any of the four relays can be toggled from the
// web UI, but ONLY while the ignition is OFF and no real function relay is energized
// (light off, wheel lock released). Every test bit auto-releases after
// RELAY_TEST_TIMEOUT_MS; the starter relay (Relay 2) is capped much shorter so a test
// cannot crank the engine for long.
#define RELAY_TEST_TIMEOUT_MS      30000UL  // ms a test relay stays on before auto-off
#define RELAY_TEST_STARTER_MAX_MS  2000UL   // ms cap for a starter-relay (Relay 2) test

// The front-wheel lock is fitted on the quad bike (Relay 4, RELAY_MASK_WHEEL_LOCK above);
// RelayController drives it from MAVLink channel 7 and the web `set_wheel_lock`.
#define RELAY_WHEEL_LOCK_FITTED 1           // 1 = relay behind the wheel-lock function

// Cranking Parameters
#define CRANKING_TIMEOUT           2000  // ms - maximum cranking duration
#define ACC_PRECRANK_DWELL_MS      2000  // ms - ignition/ECU line (R1) must be powered this long before the starter (R2) engages
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
#define STEER_POSITION_TOLERANCE  10    // 0.1° (1.0°) - park window: driving stops (coast, the worm holds) within this — bench-tune
#define STEER_RESTART_TOLERANCE   30    // 0.1° (3.0°) - a parked loop drives again only beyond this (stop hysteresis) — bench-tune 2–5°
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

// Proportional PWM control (include/SteerPositionLaw.h): duty = clamp(|error| * STEER_KP + boost, floor, MAX),
// floor = STEER_PWM_MIN_DUTY (break-away) from rest / after a reversal until 0.3° of progress
// (STEER_BOOST_MIN_PROGRESS), then STEER_PWM_TRACK_MIN_DUTY while tracking.
#define STEER_KP                  1.15f // PWM duty (0-255) per 0.1° of error (saturates at ≈22°) — bench-tune
#define STEER_PWM_MIN_DUTY        140   // break-away floor: overcomes the worm's static friction (bench: no movement at 70);
                                        // used from rest until 0.3° of progress; also the rate-mode and jog floor
#define STEER_PWM_TRACK_MIN_DUTY  100   // floor while driving after break-away; must exceed the worm's running friction — bench-tune 90–120
#define STEER_PWM_MAX_DUTY        255   // full speed for large errors

// Anti-stall boost: steering load rises toward the locks; when commanded motion
// makes no progress, duty escalates above the P term until movement resumes.
// (Bench: closed-loop stalled ~72 counts short of the left lock at MIN duty.)
#define STEER_BOOST_CHECK_MS      100   // ms - progress check period
#define STEER_BOOST_MIN_PROGRESS  3     // 0.1° (0.3°) per check period considered "moving" — bench-tune
#define STEER_BOOST_STEP          30    // duty added (stalled) or removed (moving) per check

// Calibration jog (momentary open-loop drive from the web UI)
#define STEER_JOG_DUTY            180   // PWM duty while jogging (default when the UI sends no speed; min working duty is ~140)
#define STEER_JOG_TIMEOUT_MS      500   // ms - auto-stop if the jog command is not refreshed
#define STEER_NUDGE_DUTY          150   // PWM duty for a precision nudge pulse (just above friction)
#define STEER_NUDGE_MS            70    // ms - nudge pulse length (a degree or two per tap)

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

// VESC steering driver telemetry monitor + failsafe
// (Over-current is left to the VESC itself: Motor Current Max clamp, Absolute Max
//  Current fault — surfaced here as a fault code — and MOSFET temperature limiting.
//  Mechanical jams are caught by the steering-angle stall detector, STEER_STALL_TIMEOUT,
//  while the sensor is healthy; sensorless jog relies on the VESC current clamp + fault stop.)
#define STEER_VESC_TELEM_MS         300   // ms — COMM_GET_VALUES poll period (~3 Hz)
#define STEER_VESC_COMM_TIMEOUT_MS  1000  // ms — no valid GET_VALUES reply for this long -> driver fault (stop, reject moves)
// Steering VESC input voltage below this = "low" (telemetry steer_vin_low → web "Увага" warning only,
// no control action). 21.5 V ≈ 3.58 V/cell on a 6s NMC pack, ≈ 3.07 V/cell on a 7s LiFePO4 pack —
// low but not yet harmful for either. Adjust per vehicle build.
#define STEER_VIN_LOW_V             21.5f // V

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
#define TRANS_UNKNOWN_GEAR_THROTTLE_MAX     (float)8.5   // % - max throttle when physical gear UNKNOWN
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
//    through the taper, the trim and the 80 %/s slew (100 → 0 takes ≥ 1.25 s), never as a step.
#define SPEED_LIMIT_TAPER_FRACTION     0.20f  // band = 0.20 × limit — the limiter takes the last fifth
#define SPEED_LIMIT_TAPER_MIN_MS       0.5f   // m/s - ≈3.5 speed quanta (0.142 m/s); narrower hunts
#define SPEED_LIMIT_TAPER_MAX_MS       1.4f   // m/s (≈5 km/h) - the old fixed band, proven at road limits
#define SPEED_LIMIT_FLOOR_DEFAULT_PCT  0.0f   // % - default floor = calibrated idle; below any holding throttle
#define SPEED_LIMIT_FLOOR_MIN_PCT      0.0f   // % - accepted floor range (web command / NVS validation)
#define SPEED_LIMIT_FLOOR_MAX_PCT      20.0f  // % - above this the floor itself becomes a hidden speed floor
#define SPEED_LIMIT_FLOOR_NVS_KEY      "thr_floor"  // NVS "speed" namespace, float % (owned by VehicleController)
#define SPEED_LIMIT_TRIM_KI_PCT_S_PER_MS 30.0f // %/s per m/s - 1.4 m/s over takes the trim 100 → 0 in ≈2.4 s
#define SPEED_LIMIT_TRIM_RISE_PCT_S    10.0f  // %/s - trim recovery cap; far below the slew, never a surge
#define SPEED_LIMIT_BIND_MARGIN_PCT    1.0f   // % - demand within this of the ceiling = limiter binding
                                              // (the trim only rises while binding: anti-windup)
#define SPEED_LIMIT_CEILING_SLEW_PCT_S 80.0f // %/s - max ceiling movement (both directions) — no servo snap.
                                              // Applied to the CEILING, not the demand: the driver's own
                                              // stick moves are never slowed by the limiter.
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

// The console is UART0 (GPIO43 TX / GPIO44 RX) through the DevKit's on-board
// USB-UART bridge — the "COM" port. The native USB CDC console must NOT be enabled
// on this board: its D-/D+ pins (GPIO19/20) carry the R/N gear switches.
// The firmware never waits for a host.
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

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

// Firmware version string (semantic versioning: MAJOR.MINOR.PATCH)
// Update this constant when releasing new firmware versions, and add a matching
// entry to include/Changelog.h — the web UI shows both.
#define FIRMWARE_VERSION "1.5.7"

// Hardware this build targets. One value per hardware branch ("Control_v0" on
// master, "DevKitC-1" on old-board). Reported next to the version in telemetry
// so the web UI can tell which board the running firmware was built for.
#define BOARD_NAME "DevKitC-1"

#endif // CONSTANTS_H
