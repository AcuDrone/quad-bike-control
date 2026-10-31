# Change: Aztec steering VESC moves from UART0 to the shared MCP2515 CAN bus

## Why
The steering VESC (Flipsky 75200) talks to the ESP32 over UART0 on the Prog header X1
(`update-aztec-vesc-uart0`). That link needs R63/R64 removed on the board, keeps X1 busy, and is a
second cable harness next to the CAN bus that already runs to the steering column for the Bosch LWS
angle sensor (`replace-aztec-steering-sensor-with-bosch-lws`). The VESC has a native CAN port and a
native CAN protocol: duty command, periodic status broadcasts, and command forwarding for anything
else. Moving the VESC onto the existing MCP2515 bus (500 kbit/s, shared with OBD-II and the LWS)
removes the UART link and frees X1. It also gives telemetry at a fixed rate without request/reply
traffic.

## What Changes
- **Transport.** The steering VESC is driven over CAN with the native VESC protocol:
  - 29-bit extended IDs `(cmd << 8) | STEER_VESC_CAN_ID`.
  - VESC controller ID **1** (`STEER_VESC_CAN_ID`). Factory ID 0 is forbidden.
  - The ESP32 uses ID **2** on this bus for replies (`CAN_CONTROLLER_VESC_SENDER_ID`).
  - The bus is the existing MCP2515 bus at 500 kbit/s. It is **not** DroneCAN. Merging it with a
    future DroneCAN bus is out of scope. `DRONECAN_RESEARCH.md` shows that a DroneCAN message from a
    node whose ID equals the VESC ID, with a type-ID low byte of 0–4, would drive the motor, and no
    ID choice removes that.
- **Command.** `CAN_PACKET_SET_DUTY` (cmd 0) carries an int32 big-endian value, duty × 100000.
  - The firmware's 0..255 drive scale maps linearly: 255 → 100000, so `STEER_PWM_MIN_DUTY` (140),
    `STEER_JOG_DUTY` and the rate-mode mapping keep their meaning.
  - Stop = duty 0 (coast), as today.
  - A changed duty is sent at once, at most every `STEER_VESC_CAN_MIN_TX_MS` (10 ms).
  - A non-zero duty is re-sent every `STEER_VESC_CAN_KEEPALIVE_MS` (200 ms), so the VESC's own
    command timeout (VESC Tool *Timeout*, 1000 ms) never trips while driving. Duty 0 is not
    re-sent. It is retried only until it has been sent once.
- **Telemetry from the VESC broadcasts** (VESC Tool: STATUS_1, STATUS_4 and STATUS_5 at 5 Hz):
  - Motor current comes from `CAN_PACKET_STATUS` (9).
  - FET temperature comes from `CAN_PACKET_STATUS_4` (16).
  - Input voltage comes from `CAN_PACKET_STATUS_5` (27). The 1.4.0 `steer_vin` low-voltage warning
    keeps working.
  - `driverOk` = a STATUS_1 frame from ID 1 seen within `STEER_VESC_COMM_TIMEOUT_MS` (1000 ms,
    i.e. 5 missed frames).
- **Fault code.** The fault code is polled every `STEER_VESC_FAULT_POLL_MS` (5000 ms) with
  `COMM_GET_VALUES_SELECTIVE`, sent through VESC CAN forwarding:
  - The request is `CAN_PACKET_PROCESS_SHORT_BUFFER`, cmd 8, with the mask bit 15 = fault code.
  - The reply is `[COMM id, mask(4), fault]`, 6 bytes, so it is expected as **one**
    `PROCESS_SHORT_BUFFER` (cmd 8) frame addressed to ID 2. bldc `comm_can_send_buffer()` sends a
    payload of ≤ 6 bytes that way; this is an assumption to verify on the bench.
  - The longer `FILL_RX_BUFFER` (5) + `PROCESS_RX_BUFFER` (7) form is also parsed, with
    reassembly and a CRC check.
  - There is no on-demand poll.
  - A missing reply does **not** clear `driverOk`. It keeps the last fault code and sets
    `steer_vesc_fault_stale`.
- **MCP2515 acceptance.** RXB1 is unchanged: LWS `0x305` only. RXB0 keeps mask `0x7F8`, and filter
  0 stays OBD-II `0x7E8`. Only **filter 1** changes, from standard `0x7E8` to **extended
  `0x00000000`**. Through the existing mask this accepts every extended ID below `0x200000`, which
  covers all VESC frames. Software then dispatches on the command and the ID byte. With RXB0→RXB1
  rollover, VESC frames get the 2-deep path (design D5).
- **Hard dependency: interrupt/task-driven CAN reception.** The VESC sends STATUS_1/4/5 (and
  multi-frame replies) back-to-back, about 260 µs apart. The MCP2515 has two buffers and is
  polled from the cooperative main loop, so in polled mode 2 of every 3 status frames would be lost
  systematically. This change therefore requires `move-can-rx-to-interrupt` (firmware 1.6.0:
  MCP2515 INT on GPIO38, the `can_rx` reader task, a receive queue drained by
  `CANController::update()`, one SPI mutex and the non-blocking `queueTx()` transmit) to be
  implemented first. It designs no polled workaround (design D6). The suggested single-mask RXB1 layout
  (`0x1FFF00FF`) was rejected: for standard frames the MCP2515 applies the EID mask bits to data
  bytes 0–1, so it would drop most LWS frames.
- **Transmit whitelist.** `CANController` gets one VESC transmit entry point. It can only send ID
  `(cmd << 8) | STEER_VESC_CAN_ID` with cmd ∈ {SET_DUTY, PROCESS_SHORT_BUFFER}. No other VESC
  command (current, RPM, position, broadcast ID 255) can be sent.
- **Code.**
  - New pure header `include/VescCan.h`: IDs, the duty mapping, status decoding, fault-poll
    request and reply parsing, reassembly, and the transmit schedule.
  - New pure header `include/Mcp2515Filter.h`: an acceptance-filter model.
  - New native test `test/test_vesc_can`.
  - New `VescCanDriver : IMotorDriver`, fed by `CANController` dispatch, which transmits through
    `CANController`.
  - `VescMotorDriver` (UART) is **deleted**.
  - `VescProtocol.h` stays as a header-only, Arduino-free home for the CRC16, the COMM IDs and the
    big-endian helpers. The UART framing and the `COMM_GET_VALUES` decoder are deleted, and
    `VescProtocol.cpp` is deleted.
- **UART0 / X1 freed.**
  - `PIN_VESC_TX/RX`, `VESC_UART_NUM`, `VESC_UART_BAUD` and `STEER_VESC_TELEM_MS` are removed.
  - The UART0 VESC init is removed from `main.cpp`.
  - R63/R64 stay removed. Restoring the CH9121T LAN chip is out of scope.
  - Logging stays off UART0 (`CORE_DEBUG_LEVEL=0`, the ROM putc is closed, `esp_log` goes to
    USB-CDC). It is no longer required, but it keeps X1 quiet.
- **Telemetry / web.**
  - The keys `steer_driver_ok`, `steer_motor_current`, `steer_fet_temp`, `steer_vin`,
    `steer_vin_low` and `steer_vesc_fault` keep their names and meaning.
  - New key `steer_vesc_fault_stale`. `steer_vesc_link` is not added: the link is always CAN.
  - The "Кермо: стан" driver tile is labelled "VESC (CAN)" and marks a stale fault code.
  - MAVLink is unchanged (the six named floats). Web "Увага" is unchanged.
- **Docs.** `openspec/project.md` (wiring paragraph), `GPIO_PINOUT_CUSTOM_BOARD_S3.md`, the
  `Constants.h` and `platformio.ini` comments, `MAVLINK_SETUP.md` (VESC data source and rate) and
  `WEB_PORTAL_SETUP.md` (driver tile).
- **Version.** `FIRMWARE_VERSION` 1.6.0 → **1.7.0** (minor: wiring change), with a `Changelog.h`
  entry in uk and en. 1.6.0 is `move-can-rx-to-interrupt`.
- **BREAKING (hardware / VESC configuration).**
  - The VESC must be wired to the CAN bus: CAN_H, CAN_L, GND.
  - The bus termination must be checked: ≈60 Ω between CAN_H and CAN_L with power off.
  - In VESC Tool: *Controller ID* = 1, *CAN Baud Rate* = 500K, *CAN Mode* = VESC, status
    STATUS_1/4/5 at 5 Hz, *Timeout* = 1000 ms with a brake current of 0.
  - A 1.7.0 firmware does not drive a UART-wired VESC.
  - No steering recalibration is needed.

## Out of scope
- DroneCAN, the TWAI peripheral, and merging the VESC with an autopilot CAN bus.
- The interrupt/task-driven MCP2515 reception itself. It is the prerequisite change
  `move-can-rx-to-interrupt`.
- On-demand fault polling, VESC parameter access over CAN, and firmware updates of the VESC over CAN.
- Re-fitting R63/R64 or any other use of X1.

## Relationship to other pending changes (archive order)
This change SHALL be **implemented** only after `move-can-rx-to-interrupt` (interrupt/task-driven
MCP2515 reception, firmware 1.6.0) is implemented. `move-can-rx-to-interrupt` SHALL be archived
**immediately before** this change. That is a functional prerequisite, not just a spec-ordering one
(design D6).

`move-can-rx-to-interrupt` modifies `CAN Communication Error Handling` (re-init under the SPI
lock, the receive-queue drain). This change's MODIFIED block for that requirement is re-based on
that text. `Shared-Bus Steering Angle Frame Reception` is not modified there, so this change still
quotes it from `replace-aztec-steering-sensor-with-bosch-lws`.

This change's MODIFIED blocks quote the pending deltas. It SHALL be archived **after**:
0. `move-can-rx-to-interrupt` (prerequisite, archived immediately before this change):
   `CAN Communication Error Handling` (can-controller) is quoted from that change.
1. `update-aztec-vesc-uart0`: `VESC Brushed-DC Steering Driver` (vehicle-actuators) and `USB-CDC
   Serial Console` (debug-logging) are quoted from that change.
2. `replace-aztec-steering-sensor-with-bosch-lws`: `Hardware Configuration` and `VESC Fault
   Monitoring and Communication Failsafe` (vehicle-actuators) and `Shared-Bus Steering Angle Frame
   Reception` (can-controller) are quoted from that change. `CAN Communication Error Handling` is
   quoted through `move-can-rx-to-interrupt`, which builds on it.
3. `redesign-web-ui-service-console`: `Steering VESC Input Voltage Telemetry` (web-telemetry) is
   quoted from that change.
4. `add-aztec-steering-rate-mode`: it touches none of the requirements above, but it is the 1.5.0
   baseline this change builds on (rate drive duty scale, `STEER_PWM_MIN_DUTY`).

`add-aztec-dual-as5600-steering` is archived before all of these, as their own proposals require.

## Impact
- Affected specs:
  - `vehicle-actuators`:
    - MODIFIED: Hardware Configuration, VESC Brushed-DC Steering Driver, VESC Fault Monitoring and
      Communication Failsafe
    - ADDED: VESC Fault Code Poll via CAN Forwarding
  - `can-controller`:
    - MODIFIED: CAN Communication Error Handling, Shared-Bus Steering Angle Frame Reception
    - ADDED: Shared-Bus Steering VESC Frames
  - `web-telemetry`: MODIFIED Steering Motor Telemetry, Steering VESC Input Voltage Telemetry
  - `debug-logging`: MODIFIED USB-CDC Serial Console (UART0 is no longer the VESC link)
  - `vehicle-systems`: MODIFIED 24V Boost Rail Control and Monitoring (boost enabled before the
    steering VESC link is brought up, instead of "before the VESC UART")
  - `mavlink-interface`: unchanged. "Valid VESC reply" now means a STATUS_1 frame; the
    requirement text is transport-neutral.
- Affected code:
  - New: `include/VescCan.h`, `include/Mcp2515Filter.h`, `include/VescCanDriver.h`,
    `src/VescCanDriver.cpp`, `test/test_vesc_can/test_main.cpp`.
  - Deleted: `include/VescMotorDriver.h`, `src/VescMotorDriver.cpp`, `src/VescProtocol.cpp`.
  - Changed:
    - `include/VescProtocol.h`: header-only.
    - `include/CANController.h`, `src/CANController.cpp`: filter 1, dispatch, VESC transmit.
    - `include/VehicleController.h`, `src/VehicleController.cpp`: wiring.
    - `src/main.cpp`.
    - `include/IMotorDriver.h`, `include/SteeringController.h`, `src/SteeringController.cpp`:
      comments only.
    - `include/Constants.h`.
    - `include/TelemetryManager.h`, `src/TelemetryManager.cpp`, `src/WebPortal.cpp`.
    - `data/index.html`.
    - `include/Changelog.h`.
    - `platformio.ini`: native `test_filter` and comments.
- Hardware: VESC CAN port to the MCP2515 bus. X1 is left unconnected.
- Safety: steering now depends on the one CAN bus for both the angle and the drive. A dead MCP2515
  or bus takes out sensorless jog and rate mode too. The motor still stops: the VESC times out, and
  the firmware sees `driverOk` false. See design "Risks".
