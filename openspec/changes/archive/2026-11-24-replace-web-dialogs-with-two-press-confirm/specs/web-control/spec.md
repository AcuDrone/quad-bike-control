## ADDED Requirements
### Requirement: Two-Press Confirmation for Destructive Web Actions
The web interface SHALL NOT use native browser dialogs (`alert()`, `confirm()`, `prompt()`): they
are suppressed in the phone's captive-portal sign-in webview and in Chrome after "prevent this
page from creating additional dialogs", where `confirm()` returns false and the action silently
does nothing. The actions "Set sensor zero" (steering calibration card), "Reset to defaults" (gear
codes card), "Clear" (event journal card) and "Upload Firmware" (firmware update card) SHALL
instead be confirmed by a second press of the same button, through one shared helper.

| Action | Armed button label (uk / en) | Status-line warning (uk / en) |
|---|---|---|
| Set sensor zero | Підтвердити нуль / Confirm zero | Це положення стане центром. Межі буде стерто. / This position becomes the center. The limits will be erased. |
| Gear codes reset | Підтвердити скидання / Confirm reset | Відновити типові коди передач? / Restore the default gear codes? |
| Journal clear | Підтвердити видалення / Confirm delete | Журнал буде видалено, лічильник запусків лишиться. / The journal will be deleted; the boot counter is kept. |
| Firmware upload | Підтвердити завантаження / Confirm upload | Авто перейде в безпечний стан і перезавантажиться. / The vehicle will enter safe state and reboot. |

#### Scenario: First press arms the button
- **WHEN** the operator presses one of these buttons and its existing pre-checks pass (WebSocket
  open for "Set sensor zero", a file selected for "Upload Firmware")
- **THEN** nothing SHALL be sent to the firmware
- **AND** the button label SHALL change to the armed label and the button SHALL be highlighted with
  the theme's warning colour
- **AND** the card's status line SHALL show the warning in the warning colour

#### Scenario: Second press within 5 s runs the action
- **WHEN** the operator presses the same armed button again within 5 s
- **THEN** the original label SHALL be restored, the warning cleared, and the action SHALL run
  exactly as before (the same command or request is sent)

#### Scenario: Arming expires after 5 s
- **WHEN** 5 s pass without a second press
- **THEN** the original label SHALL be restored and the highlight removed
- **AND** the warning SHALL be cleared only if the status line still shows it; a newer message SHALL
  be left in place

#### Scenario: Buttons arm independently
- **WHEN** one button is armed and the operator presses another of these buttons
- **THEN** the second press SHALL arm that other button and SHALL NOT run either action

#### Scenario: Language switch while armed
- **WHEN** the UI language is switched while a button is armed
- **THEN** the armed label and a warning still on screen SHALL be shown in the new language
- **AND** on the second press or on expiry the original label SHALL be restored in the new language

#### Scenario: No native dialogs remain
- **WHEN** the served page is searched for `alert(`, `confirm(` and `prompt(` calls
- **THEN** none SHALL be found

## MODIFIED Requirements
### Requirement: Steering Calibration and Sensor Zero via Web Interface
The web portal SHALL provide steering calibration controls for the Bosch LWS sensor: jog and nudge that are always available, a "Set sensor zero" action, and Left/Right limit capture. These commands SHALL be accepted regardless of the active input source (same privilege level as the other calibration commands). The calibration card SHALL hold the actions and a compact readout only; the full sensor diagnostics SHALL be shown in the Diagnostics section ("Кермо: стан").

#### Scenario: Set sensor zero
- **WHEN** the operator presses "Set sensor zero" and confirms it with a second press of the same button, as defined by "Two-Press Confirmation for Destructive Web Actions", after the status line has warned that the current position becomes the steering center and that the stored limits will be erased
- **THEN** a `steer_lws_zero` WebSocket command SHALL be sent
- **AND** the firmware SHALL start the zero-set sequence with no ignition, engine-RPM or standstill precondition
- **AND** the firmware SHALL report success only after an accepted sensor frame shows sensor OK, no fault code, the calibrated bit set and an angle within `STEER_LWS_ZERO_TOLERANCE` of zero, within `STEER_LWS_ZERO_CONFIRM_MS` of the second frame
- **AND** otherwise it SHALL report failure with a reason: `busy`, `can_unavailable`, `tx_failed`, `range_latched` (restart the vehicle and retry), `no_frames` or `not_confirmed`
- **AND** the "Кермо: стан" card SHALL show the progress (`steer_zero_state`) and the reason (`steer_zero_error`), and the calibration card's status line SHALL show the command response

#### Scenario: No connection to the board
- **WHEN** the operator presses "Set Left", "Set Right" or "Set sensor zero" while the WebSocket to the board is not open
- **THEN** no command SHALL be sent and "Set sensor zero" SHALL NOT be armed
- **AND** the calibration card's status line SHALL show «Немає з'єднання з платою» / "No connection to the board" in the danger colour instead of silently doing nothing

#### Scenario: Capture limits only
- **WHEN** the operator presses "Set Left" or "Set Right"
- **THEN** a `steer_cal_left` or `steer_cal_right` command SHALL be sent
- **AND** the response SHALL report the captured angle in degrees on success, or name the reason on failure (sensor fault, too close to zero, or a reading that contradicts the stored opposite limit together with the "Set sensor zero" remedy)
- **AND** when the capture toggled the sensor-direction flag, the success response SHALL say so (e.g. "sensor direction detected: mirrored — capture the right limit again") and SHALL NOT claim the calibration is complete
- **AND** a capture beyond ±550° (`STEER_LWS_SAFE_RANGE`) SHALL succeed, and its response SHALL carry a warning that the sensor's latched-fault threshold is near
- **AND** no center capture button SHALL be shown
- **AND** a `steer_cal_center` command SHALL be answered with a failure that points to "Set sensor zero"

#### Scenario: Jog and nudge always enabled
- **WHEN** the calibration card is shown
- **THEN** the jog and nudge controls SHALL NOT be disabled because of the steering sensor state or the calibration state
- **AND** while `steer_sensor_ok` is false the card SHALL show a note that jog is running without a sensor and without stall protection

#### Scenario: Compact readout in the calibration card
- **WHEN** the calibration card is shown
- **THEN** it SHALL show the jog speed slider, the current angle, the Left/Right limits (with the ⚠ beyond-550° note when `steer_range_warning` is true), and one status line with the sensor state and the calibration state
- **AND** it SHALL NOT show the LWS status bits, fault code, frame rate, error counters, zero-set state, motor current, FET temperature or driver state, which are shown in "Кермо: стан"
- **AND** readouts shown both here and in "Кермо: стан" SHALL be rendered from the same telemetry frame so the two never disagree

#### Scenario: Calibration help text
- **WHEN** the engineer opens the calibration card's ⓘ help
- **THEN** its help text (en/uk) SHALL describe the flow: jog to straight-ahead, restart the vehicle, press "Set sensor zero", then jog to each lock and capture Left and Right
- **AND** it SHALL state that the sensor direction (normal or mirrored mount) is detected automatically from the first limit capture, and that after re-mounting the sensor the operator sets the sensor zero again and captures both limits
- **AND** it SHALL tell the operator to check, right after the first capture, that Jog ▶ makes the angle grow, and that a shrinking angle means Left and Right were captured at the wrong locks
- **AND** it SHALL state that a sensor reporting `range_latched` is cleared only by restarting the vehicle, and that jogging past about 550° risks that latch
