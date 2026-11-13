#ifndef STEER_DIAG_H
#define STEER_DIAG_H

#include <Arduino.h>
#include "Constants.h"
#include "IMotorDriver.h"
#include "SpeedSensor.h"
#include "SteeringController.h"
#include "SteerDiagLaw.h"

/**
 * @brief Steering diagnostics console on the USB-CDC `Serial` (change
 * add-steering-serial-diagnostics). Host side: tools/steer_diag.py.
 *
 * Only the native USB `Serial` is used — never Serial0 / UART0 (the aztec VESC) or UART2
 * (the quad VESC). Polled from loop() after VehicleController::update(); non-blocking.
 *
 * DevKitC-1 (old-board): `Serial` IS UART0 here (the DevKit's USB-UART bridge, "COM" port,
 * 115200) because the native USB CDC console is never enabled (GPIO19/20 = R/N gear
 * switches). Everything works the same over the COM port, except the USB-host-lost abort:
 * `(bool)Serial` on a HardwareSerial is always true, so a closed host only ends diag through
 * STEER_DIAG_IDLE_TIMEOUT_MS. The `K` keepalive lines are harmless on a UART.
 *
 * COMMANDS (one per line, case-insensitive, CR / LF / CRLF):
 *   diag on                 take exclusive steering ownership (refused above STEER_DIAG_MAX_SPEED_KMH):
 *                           motor stopped, MAVLink / web / failsafe / boot-centre steering, jog, nudge,
 *                           captures and the zero-set refused, VESC GET_VALUES every STEER_DIAG_VESC_TELEM_MS
 *   diag off                release (motor stopped); automatic after STEER_DIAG_IDLE_TIMEOUT_MS without
 *                           any line, and when the USB host goes away (`(bool)Serial` false for
 *                           STEER_DIAG_HOST_LOST_MS after it read true once since diag on)
 *   status                  one I,status line (works with diag off too)
 *   stop | s                cut the motor at once, cancel the experiment (works with diag off too)
 *   help | ?                list the commands
 *   ramp L|R [max=220] [step=2] [stepms=150] [start=40]
 *                           duty from start, +step every stepms up to max; movement = 0.5° from the start
 *                           or STEER_DIAG_MOVE_RATE °/s toward the drive; then duty 0 and 600 ms of coast
 *                           log; ends no_move after 1 s at max, hard timeout 15 s
 *   pulse L|R <duty> <ms>   one open-loop pulse (ms ≤ 1000) + 600 ms log
 *   hold L|R <duty> <ms>    constant duty (ms ≤ 1500) + 600 ms log; rate/current averaged over the 2nd half
 *   goto <deg>              one move of the normal closed loop to <deg> (needs both limits); logged until
 *                           the loop stops + 600 ms, then the loop is dropped to idle (coast)
 * Every experiment needs diag on, both limits captured, a fresh valid LWS angle, the VESC link up
 * with no fault, standstill, and must not start toward a limit closer than STEER_DIAG_LIMIT_MARGIN.
 * It is aborted (motor cut) on: speed, angle invalid, VESC link down / fault, driving to within
 * STEER_DIAG_LIMIT_MARGIN of a limit, `stop`, a new command, idle timeout, USB host lost.
 *
 * OUTPUT LINES (fields comma-separated, no spaces; other firmware logs may interleave — the
 * host keeps only lines starting with one of these prefixes):
 *   OK <cmd>                        command accepted (<cmd> = diag_on, diag_off, ramp, pulse, hold, goto, stop)
 *   E,<cmd>,<reason>                command refused (reasons: unknown_cmd, bad_dir, bad_arg, range,
 *                                   missing_arg, line_too_long, diag_off, speed, angle, driver, fault,
 *                                   not_calibrated, near_limit, refused)
 *   E,abort,<reason>                running experiment aborted (speed, angle, driver, fault, limit,
 *                                   stop, host, idle_timeout, superseded)
 *   I,status,diag=0|1,exp=<kind|none>,angle=<deg|nan>,rate=<deg/s|nan>,limL=<deg|nan>,limR=<deg|nan>,
 *            cal=0|1,sensor=0|1,drv=0|1,fault=<code>,duty=<n>,cur=<A>,vin=<V>,speed=<km/h>,
 *            speedValid=0|1,loop=idle|drive|park,fw=<version>
 *   I,diag,on|off[,reason=<why>]    ownership change (reason: cmd, idle_timeout, host)
 *   D,<t_ms>,<duty>,<pos>,<rate>,<cur>,<vin>,<state>,<ref>
 *        50 Hz (STEER_DIAG_LOG_MS) plus one line per duty step / phase change while an experiment
 *        runs. t_ms since the experiment start; duty = signed effort last sent to the VESC
 *        (-255..255); pos = angle in 0.1° (firmware sign, + = right); rate = LWS °/s (empty when
 *        invalid); cur = VESC motor current A; vin = VESC input V; state = ramp|pulse|hold|coast
 *        (open loop) or drive|park|idle (goto, the loop state); ref = start position (open loop)
 *        or target (goto), 0.1°. Lines are dropped (counted) rather than block when the USB TX
 *        buffer is full.
 *   S,ramp,dir=L|R,result=moved|no_move|timeout,break_duty=<n>,break_cur=<A>,break_ms=<n>,
 *          moved_cut=<deg>,moved_end=<deg>,coast=<deg>,peak_cur=<A>,start=<deg>,drop=<n>
 *   S,pulse,dir=..,duty=<n>,ms=<req>,actual_ms=<n>,moved_cut=<deg>,moved_end=<deg>,coast=<deg>,
 *          peak_cur=<A>,start=<deg>,drop=<n>
 *   S,hold,dir=..,duty=<n>,ms=<n>,rate_pos=<deg/s>,rate_lws=<deg/s>,mean_cur=<A>,peak_cur=<A>,
 *          moved_cut=<deg>,coast=<deg>,start=<deg>,drop=<n>
 *   S,goto,target=<deg>,start=<deg>,end=<deg>,result=park|idle|timeout,overshoot=<deg>,
 *          residual=<deg>,t_stop=<ms>,t_end=<ms>,restarts=<n>,peak_cur=<A>,drop=<n>
 *   (moved_* signed, firmware sign; coast in the drive direction, ≥ 0 = kept turning.)
 *   K                               keepalive (HWCDC stuck-TX workaround): every STEER_DIAG_KEEPALIVE_MS
 *                                   while diag is on and for STEER_DIAG_KEEPALIVE_WINDOW_MS after the
 *                                   last command line, except while D-lines flow; skipped when the USB
 *                                   TX buffer is nearly full. Carries no data — the host MUST ignore
 *                                   lines equal to `K` (and blank lines).
 */
class SteerDiag {
public:
    SteerDiag(SteeringController& steering, IMotorDriver& motor, const SpeedSensor& speed);

    /** @brief Read command lines, run the experiment, apply the safety checks — every loop() */
    void update();

    bool isActive() const { return active_; }

private:
    SteeringController& steering_;
    IMotorDriver& motor_;
    const SpeedSensor& speed_;

    bool active_;
    uint32_t lastCmdMs_;
    bool lineSeen_;              // a command line was received since boot (keepalive window)
    uint32_t lastLineMs_;        // last command line (keepalive window; not touched by setActive)
    uint32_t lastKeepaliveMs_;
    char line_[STEER_DIAG_LINE_MAX + 1];
    uint16_t lineLen_;
    bool lineOverflow_;

    SteerDiagExp exp_;
    uint32_t lastLogMs_;
    uint32_t lastDriveMs_;
    int16_t lastDuty_;
    uint32_t dropped_;
    SteerDiagHostWatch hostWatch_;   // debounced USB host-lost check

    void readSerial();
    void keepalive(uint32_t now);
    void handleLine(const char* line);
    void setActive(bool on, const char* reason);
    void startExperiment(const SteerDiagCmd& c);
    void serviceExperiment(uint32_t now);
    void abortExperiment(const char* reason);
    void finishExperiment();
    /** @brief Refusal reason for starting a motion now (nullptr = OK) */
    const char* motionRefusal() const;
    /** @brief Reason to abort the running experiment now (nullptr = keep going) */
    const char* abortReason() const;
    void applyDuty(int16_t duty, uint32_t now, bool force);
    void logLine(uint32_t now);
    void printSummary();
    void printStatus();
    void printHelp();
    SteerDiagSample sample(uint32_t now) const;
    float speedKmh() const;
    bool moving() const;
};

#endif // STEER_DIAG_H
