## ADDED Requirements
### Requirement: Steering Calibration and Sensor Zero via Web Interface
The web portal SHALL provide steering calibration controls for the Bosch LWS sensor: jog and nudge that are always available, a "Set sensor zero" action, and Left/Right limit capture. These commands SHALL be accepted regardless of the active input source (same privilege level as the other calibration commands).

#### Scenario: Set sensor zero
- **WHEN** the operator presses "Set sensor zero" and confirms the dialog explaining that the current position becomes the steering center and that the stored limits will be erased
- **THEN** a `steer_lws_zero` WebSocket command SHALL be sent
- **AND** the firmware SHALL start the zero-set sequence with no ignition, engine-RPM or standstill precondition
- **AND** the firmware SHALL report success only after an accepted sensor frame shows sensor OK, no fault code, the calibrated bit set and an angle within `STEER_LWS_ZERO_TOLERANCE` of zero, within `STEER_LWS_ZERO_CONFIRM_MS` of the second frame
- **AND** otherwise it SHALL report failure with a reason: `busy`, `can_unavailable`, `tx_failed`, `range_latched` (restart the vehicle and retry), `no_frames` or `not_confirmed`
- **AND** the card SHALL show the progress (`steer_zero_state`) and the reason (`steer_zero_error`)

#### Scenario: Capture limits only
- **WHEN** the operator presses "Set Left" or "Set Right"
- **THEN** a `steer_cal_left` or `steer_cal_right` command SHALL be sent
- **AND** the response SHALL report the captured angle in degrees on success, or name the reason on failure (sensor fault, wrong side of zero, too close to zero)
- **AND** a capture beyond ±550° (`STEER_LWS_SAFE_RANGE`) SHALL succeed, and its response SHALL carry a warning that the sensor's latched-fault threshold is near
- **AND** no center capture button SHALL be shown
- **AND** a `steer_cal_center` command SHALL be answered with a failure that points to "Set sensor zero"

#### Scenario: Jog and nudge always enabled
- **WHEN** the calibration card is shown
- **THEN** the jog and nudge controls SHALL NOT be disabled because of the steering sensor state or the calibration state
- **AND** while `steer_sensor_ok` is false the card SHALL show a note that jog is running without a sensor and without stall protection

#### Scenario: Calibration help text
- **WHEN** the calibration card is shown
- **THEN** its help text (en/uk) SHALL describe the flow: jog to straight-ahead, restart the vehicle, press "Set sensor zero", then jog to each lock and capture Left and Right
- **AND** it SHALL state that a sensor reporting `range_latched` is cleared only by restarting the vehicle, and that jogging past about 550° risks that latch
