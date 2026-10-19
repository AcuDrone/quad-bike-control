#include <Arduino.h>
#include "Constants.h"
#include "Debug.h"
#include "ServoController.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include "BTS7960Controller.h"
#include "VescMotorDriver.h"
#include "TransmissionController.h"
#include "WebPortal.h"
#include "VehicleController.h"
#include "TelemetryManager.h"
#include "MavlinkInterface.h"
#include "RelayController.h"
#include "BoardInputs.h"
#include "SpeedSensor.h"
#include "nvs_flash.h"
#include <Wire.h>

// ============================================================================
// ACTUATOR INSTANCES
// ============================================================================

// Steering Actuator (Flipsky 75200 VESC over UART2 + two AS5600 angle sensors:
// FINE on the steering shaft via I2C1 `Wire` / X14, COARSE on a geared-down shaft
// via I2C2 `Wire1` / X15 — together an absolute multi-turn position).
// The VESC driver MUST be constructed before the steering controller that
// references it (global init order is top-to-bottom within this file). The buses
// are only bound here; they are opened in setup() before steering begin().
VescMotorDriver steeringVesc(VESC_UART_NUM);
SteeringController steeringActuator(steeringVesc, Wire, Wire1);

// Throttle Servo
ThrottleController throttle;

// Transmission servo (PWM servo-based gear selector)
TransmissionController transmissionActuator;
// Brake actuator (BTS7960)
BTS7960Controller brakeActuator;

// MAVLink interface to Pixhawk (TELEM2)
MavlinkInterface mavlinkInterface;

// ── I2C TOPOLOGY (scan-verified; overrides the schematic's port names) ───────
//   I2C2 (Wire1, GPIO48/47) : on-board PCA9557s — opto inputs U10 @0x1A,
//                             CD4051 mux U2 @0x1C — + relay board @0x1F and the
//                             COARSE steering AS5600 @0x36, both on X15
//   I2C1 (Wire,  GPIO1/2)   : external headers only — FINE steering AS5600 @0x36 on X14
// Expected boot scan: I2C1 = 0x36; I2C2 = 0x1A 0x1C 0x1F 0x36.
// Only the fine AS5600 may restart its own bus (I2C1). I2C2 is shared, so it is
// restarted ONLY by recoverI2C2IfWedged() below, never by a device driver.
// Fallback if X15 ever goes silent: the relay board also works on X14 → I2C1
// (change the RelayController argument below from Wire1 to Wire).

// Opto-isolated board inputs (3-bit gear code on port bits 0/1/6, brake endstop bit 4) on I2C2
BoardInputs boardInputs(Wire1);

// Relay Controller for ignition, starter and front light (aztec; wheel lock not fitted) — I2C2 @0x1F via X15
RelayController relayController(Wire1);

// Driveline speed sensor (12V hall via the X2 opto input, PCNT on GPIO8)
SpeedSensor speedSensor;

// Vehicle Controller (coordinates all actuators and input sources)
VehicleController vehicleController(steeringActuator, throttle, transmissionActuator, brakeActuator,
                                     mavlinkInterface, relayController, boardInputs, speedSensor);

// Web Portal for telemetry and manual control
WebPortal webPortal;

// Telemetry Manager (collects and broadcasts telemetry data)
TelemetryManager telemetryManager(vehicleController, webPortal, mavlinkInterface);

// ============================================================================
// BRING-UP HELPERS
// ============================================================================

// Probe the 7-bit address range 0x08-0x77 on one bus and print a single summary
// line. Master-gated only (no feature flag) so a plain `debug on` shows it.
static void scanI2CBus(TwoWire& bus, const char* label) {
    String found;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        bus.beginTransmission(addr);
        if (bus.endTransmission() == 0) {
            char hex[8];
            snprintf(hex, sizeof(hex), " 0x%02X", addr);
            found += hex;
        }
    }
    Debug::println("[I2C] " + String(label) + " devices:" + (found.isEmpty() ? " none" : found));
}

// Coordinated I2C2 (Wire1) recovery. The ESP32 I2C driver can wedge into
// ESP_ERR_INVALID_STATE after a corrupted transaction and stays broken until the
// bus is torn down and restarted. Wire1 is shared (opto inputs, relay board,
// coarse AS5600), so no single driver may restart it. It is restarted here only
// when two INDEPENDENT devices fail together — the coarse AS5600 (at least
// AS5600_RECOVER_AFTER_FAILS failed transactions in a row) AND the opto-input
// expander (BoardInputs faulted) — which points to the bus, not to one device.
// Called at the end of loop(), after every Wire1 user has finished for the
// iteration (all I2C traffic runs on this loop task, so nothing is in flight).
// The restart does not touch any PCA9557 register: relay outputs and expander
// configuration stay as the devices hold them; BoardInputs re-configures itself
// on its own recovery cadence and RelayController keeps verifying its writes.
static void recoverI2C2IfWedged() {
    static uint32_t lastAttemptMs = 0;
    static bool attempted = false;

    if (steeringActuator.getCoarseConsecutiveFailures() < AS5600_RECOVER_AFTER_FAILS) return;
    if (!boardInputs.isFaulted()) return;   // expanders read fine: bus healthy, steering fault only

    uint32_t now = millis();
    if (attempted && (now - lastAttemptMs) < I2C2_BUS_RECOVER_MS) return;
    attempted = true;
    lastAttemptMs = now;

    // Master-gated (no feature flag), like the boot bus scan: one line per attempt.
    Debug::println("[I2C] I2C2 (Wire1) suspected wedged (coarse AS5600 failures=" +
                   String(steeringActuator.getCoarseConsecutiveFailures()) +
                   ", opto inputs faulted) — re-initializing");
    Wire1.end();
    if (!Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ)) {
        Debug::println("[I2C] ERROR: I2C2 (Wire1) re-init failed");
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // 24V boost rail FIRST: it feeds the steering VESC, the Jetson, Starlink and
    // the cameras, so it must be up before any subsystem that depends on it.
    // GPIO46 has a 100K pulldown (R7) so the rail is off through reset.
    pinMode(PIN_BOOST_EN, OUTPUT);
    digitalWrite(PIN_BOOST_EN, HIGH);

    // Console is USB-CDC (ARDUINO_USB_CDC_ON_BOOT=1). Never wait for a host —
    // the vehicle must boot with no laptop attached; early output may be lost.
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("[INIT] 24V boost rail enabled (GPIO46)");

    // Initialize NVS first (required by Debug utility)
    Serial.println("[INIT] Initializing NVS...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("[INIT] NVS needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err == ESP_OK) {
        Serial.println("[INIT] NVS initialized successfully");
    } else {
        Serial.printf("[INIT] ERROR: NVS init failed with error: %d\n", err);
    }
    ESP_ERROR_CHECK(err);

    // Initialize debug utility (load state from NVS)
    Serial.println("[INIT] Initializing Debug utility...");
    Debug::begin();
    Serial.println("[INIT] Debug utility initialized");

    // Feature debug flags — MUST be set after Debug::begin(), which overwrites
    // the in-memory flags with the NVS-stored state. Uncomment one to trace it.
    // Debug::setFeatureEnabled(DebugFeature::CAN, true);           // MCP2515 health, unmatched RX, overflow
    // Debug::setFeatureEnabled(DebugFeature::RELAY, true);         // relay/ignition switching
    // Debug::setFeatureEnabled(DebugFeature::BRAKE, true);         // brake moves + endstop
    // Debug::setFeatureEnabled(DebugFeature::VEHICLE, true);       // speed sensor + vehicle state
    // Debug::setFeatureEnabled(DebugFeature::TRANSMISSION, true);
    // Debug::setFeatureEnabled(DebugFeature::MAVLINK, true);       // diagnose command-stream / fail-safe flapping
    Debug::setFeatureEnabled(DebugFeature::SERVO, false);

    // Ungated state dump so a disabled master switch is visible on the monitor
    Serial.printf("[INIT] Debug: master=%s SERVO=%s\n",
                  Debug::isEnabled() ? "ON" : "OFF",
                  Debug::isFeatureEnabled(DebugFeature::SERVO) ? "ON" : "OFF");

    Debug::println("\n=== ESP32-S3 Quad Bike Control (Control_v0) ===");

    // Record the boost enable and start rail monitoring (grace window + ADC setup)
    vehicleController.initBoostRail();

    // ── I2C buses (map at the top of this file) ──────────────────────────────
    // main.cpp is the single owner of both buses: they are opened here, before any
    // consumer, so no driver depends on another driver's initialization succeeding.
    if (!Wire.begin(PIN_STEER_SDA, PIN_STEER_SCL, I2C_BUS_FREQ_HZ)) {
        Debug::println("[INIT] ERROR: I2C1 (Wire) init failed");
    }
    if (!Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ)) {
        Debug::println("[INIT] ERROR: I2C2 (Wire1) init failed");
    }

    // Enumerate both buses before any driver claims them, so a missing or
    // mis-strapped expander is obvious from the boot log alone.
    if (Debug::isEnabled()) {
        scanI2CBus(Wire,  "I2C1 (Wire)");
        scanI2CBus(Wire1, "I2C2 (Wire1)");
    }

    // Opto-isolated inputs (gear code lines + brake limit sensor)
    if (!boardInputs.begin()) {
        Debug::println("[INIT] ERROR: Board input expander failed — gear will report UNKNOWN");
    }
    {
        BoardInputs::Snapshot snap = boardInputs.getSnapshot();
        Debug::printfFeature(DebugFeature::BRAKE, "Brake endstop: %s\n",
            !snap.valid ? "UNKNOWN (input expander unavailable)"
                        : (snap.brakeReleased ? "Released (endstop reached)" : "Not released"));
    }

    // Initialize throttle servo (loads calibration from NVS, moves to calibrated idle)
    if (!throttle.begin(PIN_THROTTLE_PWM, LEDC_CH_THROTTLE)) {
        Debug::printlnFeature(DebugFeature::SERVO, "ERROR: Throttle servo failed");
    }

    // Initialize the steering VESC UART (structural success even if the VESC is
    // silent; the controller stays driver-down until the first valid reply).
    steeringVesc.begin(PIN_VESC_RX, PIN_VESC_TX, VESC_UART_BAUD);

    // Initialize steering actuator (VESC driver + fine/coarse AS5600 pair — no homing:
    // the absolute turn is recovered from the coarse sensor). begin() ALWAYS loads the
    // calibration; a sensor that did not answer is re-probed from update().
    if (!steeringActuator.begin(PIN_STEER_SDA, PIN_STEER_SCL, PIN_RELAY_SDA, PIN_RELAY_SCL)) {
        Debug::printlnFeature(DebugFeature::SERVO, "ERROR: Steering sensor(s) missing — steering held until they answer");
    }
    // Boot centering only with a calibration, both sensors healthy and the turn
    // recovered without a mismatch (isSensorOk() covers the last two when calibrated).
    // The move cannot be issued here: the VESC link is only known to be up after its
    // first reply, inside update(). requestBootCenter() defers it to update() and
    // drops it after STEER_BOOT_CENTER_TIMEOUT_MS ("[STEER] Moving to center" is
    // logged when the move is actually issued).
    if (steeringActuator.isCalibrated() && steeringActuator.isSensorOk() &&
        steeringActuator.isTurnRecovered()) {
        Debug::printlnFeature(DebugFeature::SERVO, "[STEER] Boot centering requested (waiting for VESC link)");
        steeringActuator.requestBootCenter();
    } else {
        Debug::printlnFeature(DebugFeature::SERVO,
            !steeringActuator.isSensorOk() ? "[STEER] Sensor fault / turn not recovered — holding position"
                                           : "[STEER] Not calibrated — holding position");
    }

    // Initialize transmission servo
    if (transmissionActuator.begin()) {
        transmissionActuator.initGearSensors(boardInputs);
        transmissionActuator.loadDefaultPositions();
        transmissionActuator.loadGearOvershoots();
    } else {
        Debug::printlnFeature(DebugFeature::TRANSMISSION, "ERROR: Transmission servo failed");
    }

    // Initialize brake
    if (!brakeActuator.begin(PIN_BRAKE_RPWM, PIN_BRAKE_LPWM,
                             LEDC_CH_BRAKE_RPWM, LEDC_CH_BRAKE_LPWM)) {
        Debug::printlnFeature(DebugFeature::BRAKE, "ERROR: Brake actuator failed");
    }

    brakeActuator.stop();

    // Initialize MAVLink interface (Pixhawk TELEM2)
    if (!mavlinkInterface.begin()) {
        Debug::printlnFeature(DebugFeature::MAVLINK, "ERROR: MAVLink interface failed");
    }

    if (!relayController.begin()) {
        Debug::printlnFeature(DebugFeature::RELAY, "ERROR: Relay controller failed");
    }

    // Initialize the driveline speed sensor (loads calibration from NVS, starts PCNT)
    if (!speedSensor.begin()) {
        Debug::println("[INIT] ERROR: Speed sensor PCNT init failed — speed stays invalid");
    }

    // Restore the engine hour meter from NVS (must run after NVS is up, so not in a ctor)
    vehicleController.initEngineHourMeter();

    // Initialize CAN controller
    if (!vehicleController.initCAN()) {
        Debug::printlnFeature(DebugFeature::CAN, "WARNING: CAN controller failed (will continue without vehicle data)");
    }

    // Initialize web portal
    if (!webPortal.begin()) {
        Debug::printlnFeature(DebugFeature::WEB, "ERROR: Web portal failed");
    }

    Debug::println("\n✓ Ready");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Poll the opto-input expander (rate-limited internally) before any consumer
    // reads the snapshot this iteration.
    boardInputs.update();

    // Sample the hall speed counter (rate-limited internally) before the control
    // logic and telemetry read the speed this iteration.
    speedSensor.update();

    // Update MAVLink interface (parse inbound, request command stream)
    mavlinkInterface.update();

    // Update web portal (handles OTA, WebSocket cleanup, etc.)
    webPortal.update();

    // Determine current input source (MAVLINK > WEB > FAILSAFE)
    InputSource currentSource = telemetryManager.determineInputSource();
    vehicleController.setInputSource(currentSource);

    // Process web commands if web control is active
    WebPortal::WebCommand cmd = webPortal.getCommand();
    if (cmd.hasCommand) {
        vehicleController.processWebCommand(cmd, webPortal);
        webPortal.clearCommand();
    }

    // Update vehicle controller (failsafe, actuators, etc.)
    vehicleController.update();

    // Report vehicle state back to the MAVLink network (rate-limited internally)
    CANController::VehicleData vd = vehicleController.getVehicleData();
    String gearToStr   = vehicleController.getTargetGearString();  // current step / assumed gear
    String gearFromStr = vehicleController.getFromGearString();    // gear the step is leaving
    // Named local, NOT a temporary: StateReport stores a const char* into this String, so it
    // must outlive the report() call below (same lifetime pattern as the two gear strings above).
    String gearPhysStr = vehicleController.getCurrentGearString();  // physically sensed ("?" = unknown)
    MavlinkInterface::StateReport report;
    report.canValid     = vd.dataValid;
    report.engineRpm    = vd.engineRPM;
    report.coolantTemp  = vd.coolantTemp;
    report.gearFrom     = gearFromStr.c_str();
    report.gearTo       = gearToStr.c_str();
    report.gearMoving   = vehicleController.getTransmission().isGearChangeActive();
    report.ignition     = getRelayIgnitionStateName(vehicleController.getIgnitionState());
    report.failsafe     = (vehicleController.getInputSource() == InputSource::FAILSAFE);
    // bit0 (wheel lock) is always 0 on aztec: not fitted, getWheelLock() is always false.
    report.digitalFlags = (vehicleController.getWheelLock()  ? EFI_DIGITAL_FLAG_WHEEL_LOCK  : 0)
                        | (vehicleController.getFrontLight() ? EFI_DIGITAL_FLAG_FRONT_LIGHT : 0);
    report.speedValid   = vehicleController.isVehicleSpeedValid();
    report.speedMs      = vehicleController.getVehicleSpeedMs();
    report.intakeTemp   = vd.intakeTemp;
    report.moduleVoltageMv = vd.moduleVoltageMv;
    report.throttlePosition = vd.throttlePosition;              // measured (ECU)
    report.throttleCmdPct   = vehicleController.getThrottlePercent();  // commanded (arbitrated)
    report.gearPhysical     = gearPhysStr.c_str();               // measured (opto switches)
    report.mapKpa           = vd.mapKpa;
    report.engineLoad       = vd.engineLoad;
    report.travelDirection  = vehicleController.getTravelDirection();  // PHYSICAL gear sign
    report.odoKm            = vehicleController.getOdoKm();       // total, never resettable
    report.tripKm           = vehicleController.getTripKm();      // resettable from the GCS
    report.engineHours      = vehicleController.getEngineHours(); // total, no reset on any interface
    report.engineTripHours  = vehicleController.getEngineTripHours(); // zeroed by the TRIP reset
    // Steering VESC telemetry (STEER_A / VESC_V / VESC_TEMP / VESC_OK named floats).
    // Gate: the DRIVER's own link health.
    const SteeringController& steering = vehicleController.getSteering();
    report.steerDriverOk      = steering.isDriverOk();
    report.steerMotorCurrentA = steering.getMotorCurrent();        // MOTOR current, not input
    report.steerFetTempC      = steering.getFetTemp();
    report.steerInputVoltageV = steering.getInputVoltage();        // 24 V boost rail at the VESC
    // Steering POSITION from the same controller but a DIFFERENT gate: the AS5600's, not the
    // VESC's. Filled unconditionally — the gating is applied where the message is packed, so
    // this stays a plain snapshot.
    report.steerPercent     = steering.getSteeringPercent();       // -100 left .. 0 .. +100 right
    report.steerSensorOk    = steering.isSensorOk();
    report.steerCalibrated  = steering.isCalibrated();
    // The applied steering speed-scale (STEER_SCA). Always finite, 1.0 = not scaling — there is
    // no "unknown" state, so this is never gated and never NaN.
    report.steerScale       = vehicleController.getSteerScale();
    mavlinkInterface.report(report);

    // Broadcast telemetry to web clients
    telemetryManager.update();

    // Last: every Wire1 user (boardInputs, steering, relays, web commands) has
    // finished for this iteration, so a coordinated I2C2 restart cannot cut a
    // transaction in half.
    recoverI2C2IfWedged();
}