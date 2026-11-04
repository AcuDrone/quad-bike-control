# DroneCAN instead of MAVLink-over-UART — research (Rover 4.7+)

Research only. Nothing in the firmware has changed. Facts marked **[src]** were checked in the ArduPilot
source at tag `Rover-4.7.2` (sparse clone, Oct 2026). **[doc]** = ardupilot.org / dronecan.github.io.
**[unverified]** = my inference or a third-party claim.

## TL;DR

1. Rover 4.7 can talk to the ESP32 over DroneCAN in two ways: **(A) a MAVLink2 "virtual serial port"
   tunnelled in `uavcan.tunnel.Targetted`**, or **(B) native DroneCAN messages** (ArrayCommand, ICE Status…).
2. **A keeps 100 % of today's MAVLink logic** (SERVO_OUTPUT_RAW, PARAM, mode, VISO, EFI_STATUS, plugin). Only the byte transport changes. ArduPilot does not care if the node has a real UART **[src]**.
3. **B loses features**: there is no DroneCAN backend for VISO/wheel odometry, no flight mode, and no way to read autopilot params. EFI fields go through float16 and 3 repurposed fields are dropped.
4. Put DroneCAN on the **ESP32-S3 TWAI** with its own transceiver: a dedicated Pixhawk↔ESP32 bus at 1 Mbit/s or 500 kbit/s. Keep the MCP2515 (8 MHz → **max 500 kbit/s**, 2 RX buffers, polled) for OBD-II, LWS and VESC.
5. Plan: **A first (≈4–6 days)**. Then optionally add native ArrayCommand for actuators (hybrid C). Do not use B alone.

## Options

| | A: MAVLink via DroneCAN tunnel | B: native DroneCAN only | C: hybrid (A + native actuators/EFI) |
|---|---|---|---|
| ESP32 changes | CAN node core + byte-stream adapter under the existing `MavlinkInterface` | Rewrite the link layer and most of the telemetry | A + an ArrayCommand handler (+ optional ICE Status) |
| Keeps VISO odometry | yes | **no** (no DroneCAN VISO/WheelEncoder backend) | yes (via tunnel) |
| Keeps flight mode, SPEED_MAX, MOT_SPD_SCA_BASE | yes | **no** (Lua workaround only) | yes |
| MP plugin | unchanged (still compid 25) | must be rewritten | unchanged |
| Command latency | MAVLink stream (as today) | ArrayCommand up to `SRV_RT` (default 50 Hz) | ArrayCommand |
| Documented use | tunnel is documented for AP_Periph UARTs. A software endpoint works by source, but is not documented | fully documented | both |
| Effort (ESP32) | ≈4–6 d | ≈10–15 d + plugin | ≈7–9 d |

## Mapping: today's MAVLink → DroneCAN

| Today (MAVLink, compid 25) | Native DroneCAN equivalent in Rover 4.7 | Status |
|---|---|---|
| HEARTBEAT in (link valid / failsafe) | AP `uavcan.protocol.NodeStatus` 1 Hz **[src]** + ArrayCommand timeout. `ardupilot.indication.NotifyState` (armed and failsafe flags, `CAN_D1_UC_NTF_RT` default 20 Hz) **[src]** | OK, but **no flight mode number** |
| HEARTBEAT.custom_mode (MANUAL gate for steer scaling) | none | **tunnel only** (or a Lua hack) |
| SERVO_OUTPUT_RAW ch1–7 | `uavcan.equipment.actuator.ArrayCommand` for every output in `CAN_D1_UC_SRV_BM`, any `SERVOn_FUNCTION`, **including RCIN passthrough 51–66** (sent if PWM ≠ 0) **[src]** | OK |
| PARAM_REQUEST_READ SPEED_MAX / MOT_SPD_SCA_BASE | The AP is a param **client** only (GetSet to nodes). It does not serve its own params over DroneCAN **[src]** | **tunnel only** (or Lua `DroneCAN_Handle` broadcast **[unverified on F427 RAM]**) |
| COMMAND_LONG MAV_CMD_USER_1 (trip reset) + COMMAND_ACK | none. Could become a node param written from MP's DroneCAN screen | tunnel, or redesign |
| EFI_STATUS (19 fields) | `uavcan.equipment.ice.reciprocating.Status` (ID 1120), `EFI_TYPE=5` (DroneCAN). AP re-sends EFI_STATUS **as compid 1** **[src]** | partial — see note 1 |
| VFR_HUD (hall ground speed) | none for a peripheral. AP sends its own VFR_HUD from the EKF | tunnel only |
| VISION_POSITION_DELTA → EKF3 ExtNav | **none**: `VISO_TYPE` = 0/1 MAVLink/2 T265/3 VOXL. `WENC_TYPE` = 0/1 Quadrature/10 SITL **[src]** | **tunnel only, or dropped** |
| NAMED_VALUE_FLOAT ×6 | `uavcan.protocol.debug.KeyValue` is **not** handled by AP (no handler in AP_DroneCAN) **[src]** | tunnel only |
| STATUSTEXT | `uavcan.protocol.debug.LogMessage` → AP sends `"CAN[<node>] <text>"` STATUSTEXT if `CAN_LOGLEVEL` ≥ the level, and always logs it to the dataflash log **[src]** | OK (with a prefix) |

**Note 1 (EFI)**: the ICE Status fields are float16 / uint7 / uint17. Odometer (km) and hours in float16 are
exact only up to 2048, then step 2, 4… **Lost**: `throttle_out`, `pt_compensation` (digital-flag bitmask)
and `ignition_voltage` (AP fills these from its own state **[src]**). `coolant_temperature` is logged,
but it is not in AP's EFI_STATUS. Temperatures are in K. RPM/load/throttle cannot be NaN. Upside: MP's
native EFI widgets fill in, because the source is compid 1.

## ArduPilot parameters (Rover 4.7.2 names)

| Param | Value | Note |
|---|---|---|
| `CAN_P1_DRIVER` | 1 | CAN1 → driver 1 |
| `CAN_P1_BITRATE` | 1000000 (default) or 500000 | range 10 k–1 M **[src]**. DroneCAN is bit-rate agnostic. 500 k is a recommended rate **[doc spec §8]** |
| `CAN_D1_PROTOCOL` | 1 (DroneCAN) | default already 1 |
| `CAN_D1_UC_NODE` | 10 (default) | AP's own node ID |
| `CAN_D1_UC_OPTION` | 0. Bit 3 = ignore unhealthy node, bit 4 = SendServoAsPWM | bit 4 sends raw µs instead of −1..1 |
| **A:** `CAN_D1_UC_SER_EN` | 1 (reboot) | needs `HAL_PROGRAM_SIZE_LIMIT_KB>1024` → OK on **fmuv3** (2 MB). **Not on fmuv2** **[src]** |
| **A:** `CAN_D1_UC_S1_NOD` / `_IDX` / `_BD` / `_PRO` | ESP32 node ID (e.g. 25) / 0 / 115200–460800 / **2** (MAVLink2) | virtual port = internal serial index 41 **[src]**. `_BD` only sets the MAVLink bandwidth estimate |
| **A:** `SERIAL2_PROTOCOL` | −1/0 once TELEM2 is freed | fmuv3 has 8 MAVLink channels **[src]** |
| **B/C:** `CAN_D1_UC_SRV_BM` | bits of the outputs to send (e.g. 0x6F = 1,2,3,4,6,7) | |
| **B/C:** `CAN_D1_UC_SRV_RT` | 50 (1–200) | limited by the main loop rate as well |
| `CAN_D1_UC_ESC_BM/ESC_OF/ESC_RV` | 0 | RawCommand (−8191..8191 or 0..8191) **sends 0 when disarmed** **[src]**. Not suitable for steering, brake or gear |
| `BRD_SAFETY_MASK` / `BRD_SAFETY_DEFLT` | include the CAN outputs, or 0 | while the safety switch is ON, DroneCAN servo outputs are masked **[src]** |
| `EFI_TYPE` | 5 (DroneCAN) — B/C only | the value is **5, not 8** (8 = Hirth). 9 = MAVLink **[src]** |
| `CAN_LOGLEVEL` | 2–3 | forwards LogMessage → STATUSTEXT |

**ArrayCommand details [src]**: `actuator_id = servo output number (1-based)`. Value = `(pwm−1000)/500−1`,
clamped to −1..1 (UNITLESS), or raw PWM if OPTION bit 4 is set. There are at most 15 commands per message.
An output with PWM 0 (e.g. `MOT_SAFE_DISARM=1` while disarmed) is **not sent**, so the node must time out
to neutral. There is no extra 4.7 rename: `CAN_Dx_UC_*` has been the name since 4.3, and only
`GPS_TYPE→GPS1_TYPE` affects us.

**Node liveness for AP [src]**: the node must send NodeStatus at 1 Hz with health=OK and
mode=OPERATIONAL, or arming fails with "Node N unhealthy!". It must answer GetNodeInfo, because AP
verifies the node and checks for duplicates by unique_id. A static node ID is fine, and the tunnel needs
one anyway. Dynamic allocation (DNA) is optional.

**Tunnel semantics [src, `AP_DroneCAN_serial.cpp`, `AP_Periph/serial_tunnel.cpp`]**: both directions use
the broadcast `uavcan.tunnel.Targetted` (ID 3001, ≤120 B per transfer, multi-frame). AP→node carries
`target_node=S1_NOD`, `serial_id=S1_IDX`, `protocol=MAVLINK2` and a keepalive with len 0 every 500 ms.
AP accepts node→AP data when `source_node == S1_NOD && serial_id == S1_IDX`. It ignores `target_node`
and the protocol field. Nothing requires a physical UART: the ESP32 feeds the bytes into the existing
MAVLink parser. **Throughput (my estimate)**: 120 B ≈ 19 extended frames ≈ 34 % efficiency. That gives
≈340 kbit/s ceiling at 1 M and ≈170 kbit/s at 500 k, against 92 kbit/s for today's 115200 UART.

## ESP32 stack and effort

- **Stack**: [DroneCAN/libcanard](https://github.com/DroneCAN/libcanard) (v0, 3 C files) + headers
  from [dronecan_dsdlc](https://github.com/DroneCAN/dronecan_dsdlc) (the same generator AP uses) + ESP-IDF
  `driver/twai.h` (in Arduino-ESP32 3.x/pioarduino). Glue: TWAI RX queue → `canardHandleRxFrame`, TX
  queue ← `canardPeekTxQueue`, plus a 1 ms tick for `canardCleanupStaleTransfers`. Use TWAI acceptance
  filters for extended frames only.
- **Minimum node services**: NodeStatus 1 Hz, GetNodeInfo server, static node ID (NVS), RestartNode
  (optional), and the param GetSet/ExecuteOpcode server (optional, so MP's DroneCAN page can edit
  settings). Firmware update over DroneCAN is not needed (we have Wi-Fi OTA/USB).
- **Ready-made**: [DroneCAN_ESP32](https://github.com/dukang1996/DroneCAN_ESP32) (libcanard + ACAN_ESP32,
  Arduino, June 2026, single author, S3 not stated) **[unverified quality]**.
  [Arduino-DroneCAN](https://github.com/BeyondRobotix/Arduino-DroneCAN) is STM32-only, so use it only as a
  structural reference. **AP_Periph is not an option**: `AP_HAL_ESP32` in 4.7.2 has no CAN driver **[src]**,
  and AP_Periph would replace our firmware anyway.
- **Effort**: (a) tunnel-only node: transceiver wiring and bring-up 1 d, libcanard + TWAI glue 1–1.5 d,
  NodeStatus/GetNodeInfo/Targetted 1 d, a `Stream` adapter under `MavlinkInterface` 0.5 d, bench and
  failsafe tests 1–2 d → **≈4–6 days**. (b) native actuator + EFI + LogMessage + param server, with
  telemetry redesign and plugin rework → **≈10–15 days**, and it still loses VISO and mode.

## Hardware / bus plan

- **MCP2515 at 8 MHz**: TQ = 2/Fosc = 250 ns. The 8 TQ minimum gives **500 kbit/s max in spec**. The
  `mcp_can` library's 8 MHz "1000 kbps" preset uses 4 TQ (PS2 = 1 TQ) — **out of datasheet spec**
  **[unverified vs datasheet text]**. The 500 k preset samples at 75 % (CNF 0x00/0xD1/0x81, computed),
  while DroneCAN recommends 85–90 % at 500 k **[doc spec §8]**. The chip has 2 RX buffers and the
  driver polls them: a 19-frame tunnel burst would overflow it. **Do not put DroneCAN on the MCP2515.**
- **ID collisions on a shared bus**: DroneCAN message ID = prio(5) | type(16) | 0 | src(7) **[doc]**.
  - A VESC frame `cmd<<8 | id` looks to AP like message type `cmd` with priority 0 from node `id`. AP only
    accepts known types. `cmd=1` (SET_CURRENT) = **Allocation (ID 1)**, and AP processes it only when the
    source is 0. So **never use VESC ID 0**, and never use VESC IDs equal to DroneCAN node IDs.
  - In the other direction, the VESC decodes `cmd = (eid>>8)&0xFF`, `id = eid&0xFF`. A DroneCAN message
    from node N == VESC ID with type ID low byte 0–4 (e.g. 1025 → 1 SET_CURRENT, 1027 → 3 SET_RPM) would
    **drive the motor**. VESC frames also have priority 0 and win arbitration over all DroneCAN traffic.
  - 11-bit frames (Bosch LWS, OBD-II) are ignored by AP's DroneCAN driver unless `CAN_D1_PROTOCOL2` is set
    **[src]**. The spec explicitly allows sharing a bus with CAN 2.0A **[doc]**.
  - OBD-II 29-bit (0x18DAxxxx) is a theoretical overlap with low-priority DroneCAN. In any case, never
    bridge the engine-ECU bus to the autopilot bus.
- **Recommendation: two buses.** Bus 1 = **TWAI ↔ Pixhawk CAN1**, DroneCAN only, 1 Mbit/s if the cable is
  short and direct (500 k is fine too). Bus 2 = MCP2515 at 500 k: OBD-II, Bosch LWS (11-bit, fixed 500 k
  **[unverified]**) and any future VESC CAN.
- **TWAI pins**: GPIO4/5 (X6) are **no longer free**. Commit `d255b8a` reserves X6 for a 4th BTS7960
  (`include/Constants.h:22`). TWAI routes through the GPIO matrix, so pick two other free GPIOs. Use a
  3.3 V-logic transceiver directly on the pins (TJA1051T/3 with VIO = 3.3 V, as already used, or
  SN65HVD230). **Do not route CAN through the BSS138 shifters**: their slow rising edges are risky at
  500 k–1 M **[unverified, measure]**.
- **Pixhawk 2.4.8**: 2 × CAN in spec. There is a 4-pin CAN port (5V, CAN_H, CAN_L, GND) = CAN1. CAN2 TX/RX
  is on GPS-port pins 4/5 **[doc]**, probably logic level without a transceiver **[unverified]**. Clones use
  DF13 or Molex PicoBlade. On-board termination is **[unverified]**. Measure CAN_H–CAN_L with power off:
  you need ≈60 Ω with both ends terminated. ArduPilot needs fmuv3 (2 MB) firmware for the tunnel.

## Risks / unknowns

1. A software tunnel endpoint is not documented. It works by source reading (4.7.2), but could change upstream. Mitigation: pin the AP version and add a bench test.
2. Stream rates on the virtual MAVLink channel: which `SRn_*` maps to it is unverified. The existing SET_MESSAGE_INTERVAL re-request logic should cover it.
3. The fmuv3 build must actually run on this 2.4.8. A 1 MB-bug F427 board → fmuv2 → no tunnel and no VISO.
4. CAN wiring and noise on a buggy (ignition, 24 V motors). Use twisted pair, 2 × 120 Ω, and a common ground.
5. Failsafe: the ESP32 must keep its 500 ms command timeout. The tunnel adds DroneCAN-thread jitter (small).
6. Safety switch masking of DroneCAN servos in B/C. RawCommand zeros when disarmed.
7. DroneCAN_ESP32 library maturity is unknown. Owning libcanard glue (~300 lines) is safer.

## Open questions

- Which free GPIO pair is left for TWAI after the BTS7960 plan? Is there board space for a second transceiver?
- Is the Pixhawk running the fmuv3 or fmuv2 build? Is the CAN connector DF13 or PicoBlade, and is it terminated?
- Bus rate: 1 M (short cable) or 500 k (margin)?
- Should the VESC ever move to CAN? If yes, on the MCP2515 bus, with VESC IDs ≠ 0 and ≠ DroneCAN node IDs.
- For C: is 50 Hz ArrayCommand worth a second command source while SERVO_OUTPUT_RAW is still available?

## Ranked plan

1. **A**: TWAI + transceiver, libcanard node (static ID 25), NodeStatus/GetNodeInfo/Targetted, and a `Stream` adapter so `MavlinkInterface` is unchanged. Set `CAN_D1_UC_SER_EN=1`, `S1_NOD=25`, `S1_IDX=0`, `S1_PRO=2`. Keep the UART path as a compile-time fallback until bench-proven.
2. Bench-check that VISO, the PARAM poll, mode, the plugin, failsafe timing and CAN error counters all work.
3. Optional **C**: consume ArrayCommand (SRV_BM for outputs 1–7, OPTION bit 4 = PWM) as the primary command source, with SERVO_OUTPUT_RAW as cross-check. Optionally add ICE Status + `EFI_TYPE=5` for MP's native EFI view, while keeping the full EFI_STATUS in the tunnel.
4. Do **not** do B alone: it loses VISO, mode and params.

## Sources

- ArduPilot source, tag Rover-4.7.2: [AP_DroneCAN.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_DroneCAN/AP_DroneCAN.cpp) (params, SRV_send_actuator, SRV_push_servos, handle_debug), [AP_DroneCAN_serial.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_DroneCAN/AP_DroneCAN_serial.cpp), [AP_DroneCAN.h](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_DroneCAN/AP_DroneCAN.h), [AP_DroneCAN_DNA_Server.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp), [AP_Canard_iface.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_DroneCAN/AP_Canard_iface.cpp), [AP_CANIfaceParams.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_CANManager/AP_CANIfaceParams.cpp), [AP_CANManager_CANDriver_Params.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_CANManager/AP_CANManager_CANDriver_Params.cpp), [AP_EFI_DroneCAN.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_EFI/AP_EFI_DroneCAN.cpp), [AP_EFI.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_EFI/AP_EFI.cpp), [AP_VisualOdom.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_VisualOdom/AP_VisualOdom.cpp), [AP_WheelEncoder.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_WheelEncoder/AP_WheelEncoder.cpp), [AP_SerialManager_config.h](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_SerialManager/AP_SerialManager_config.h), [GCS_MAVLink.h](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/GCS_MAVLink/GCS_MAVLink.h), [fmuv3 hwdef](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat), [AP_Periph serial_tunnel.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/Tools/AP_Periph/serial_tunnel.cpp), [AP_HAL_ESP32](https://github.com/ArduPilot/ardupilot/tree/Rover-4.7.2/libraries/AP_HAL_ESP32), [Lua bindings.desc](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.2/libraries/AP_Scripting/generator/description/bindings.desc)
- ArduPilot docs: [DroneCAN Serial PassThrough](https://ardupilot.org/rover/docs/common-dronecan-serial.html), [Pixhawk overview](https://ardupilot.org/rover/docs/common-pixhawk-overview.html); forum: [Serial over DroneCAN support](https://discuss.ardupilot.org/t/serial-over-dronecan-support/109783), [Virtual serial ports over CAN nodes](https://discuss.ardupilot.org/t/virtual-serial-ports-over-can-nodes/141793), [DroneCAN_ESP32 announcement](https://discuss.ardupilot.org/t/dronecan-esp32-an-arduino-library-for-esp32-dronecan-communication/144201)
- DroneCAN: [spec 4.1 CAN transport](https://github.com/DroneCAN/dronecan.github.io/blob/master/Specification/4.1_CAN_bus_transport_layer.md), [spec 8 HW recommendations](https://github.com/DroneCAN/dronecan.github.io/blob/master/Specification/8._Hardware_design_recommendations.md), DSDL [tunnel/3001.Targetted](https://github.com/DroneCAN/DSDL/blob/master/uavcan/tunnel/3001.Targetted.uavcan), [ice/reciprocating/1120.Status](https://github.com/DroneCAN/DSDL/blob/master/uavcan/equipment/ice/reciprocating/1120.Status.uavcan), [actuator](https://github.com/DroneCAN/DSDL/tree/master/uavcan/equipment/actuator); [libcanard](https://github.com/DroneCAN/libcanard), [dronecan_dsdlc](https://github.com/DroneCAN/dronecan_dsdlc), [gui_tool](https://github.com/DroneCAN/gui_tool)
- Other: [PX4 VESC DroneCAN setup](https://docs.px4.io/main/en/peripherals/vesc) (VESC "CAN Mode: UAVCAN", 500 k), [MCP2515 product page](https://www.microchip.com/en-us/product/MCP2515), [MCP_CAN_lib](https://github.com/coryjfowler/MCP_CAN_lib) (`mcp_can_dfs.h` 8 MHz presets), [Arduino-DroneCAN](https://github.com/BeyondRobotix/Arduino-DroneCAN)
