# Design: Aztec steering VESC on the shared MCP2515 CAN bus

## Context
- **Today (1.6.0).**
  - `VescMotorDriver` sends `COMM_SET_DUTY` over UART0 (X1, 115200) on every
    `IMotorDriver::setSpeed()` call. The closed-loop path calls it on every loop iteration.
  - It polls `COMM_GET_VALUES` every `STEER_VESC_TELEM_MS` (300 ms) and decodes motor current, FET
    temperature, input voltage and fault code.
  - `driverOk()` is true while a valid reply was seen within `STEER_VESC_COMM_TIMEOUT_MS` (1000 ms).
- **The CAN side.**
  - `CANController` drives a polled MCP2515 (coryjfowler `mcp_can`, 8 MHz crystal, 500 kbit/s) on
    one bus shared with the engine ECU (OBD-II `0x7DF`/`0x7E8`, plus a flood of Delphi MT05
    broadcasts such as `0x301`) and the Bosch LWS (`0x305`, 100 Hz).
  - RXB0 = OBD-II window: mask `0x7F8`, filters 0 and 1 = `0x7E8`.
  - RXB1 = LWS only: mask `0x7FF`, filters 2–5 = `0x305`.
  - Since 1.6.0 (`move-can-rx-to-interrupt`), the `can_rx` reader task, woken by MCP2515 INT
    with a 2 ms polling fallback, empties RXB0/RXB1 into a 32-frame queue. `update()` drains the
    queue in every mode through `routeRxFrame()`, which hands `0x305` to `LwsSensor`. Every
    MCP2515 access holds one SPI mutex.

This change moves the VESC onto that bus with the VESC's native CAN protocol (bldc `comm_can.c`).

## Goals / Non-Goals
- Goals:
  - Same `IMotorDriver` behaviour for `SteeringController` (duty scale, coast on stop, driverOk,
    fault stop, telemetry accessors).
  - No new GPIO. X1 is freed.
  - LWS and OBD-II reception are not degraded beyond harmless, counted LWS frame losses.
  - A pure, host-tested protocol layer.
- Non-Goals:
  - DroneCAN or TWAI.
  - The interrupt/task-driven CAN reception itself. That is a prerequisite change (D6).
  - VESC configuration over CAN.
  - On-demand fault polls.
  - Any change to the steering control logic.

## Decisions

### D1 — Transport and IDs
- **ID layout.** VESC frames are CAN 2.0B extended data frames with `eid = (cmd << 8) | id`:
  - `cmd` = `CAN_PACKET_ID`;
  - `id` = the target's controller ID for commands, or the sender's ID for status broadcasts.
- **IDs on this bus:**
  - `STEER_VESC_CAN_ID = 1`: the VESC. VESC Tool → App Settings → General → *Controller ID* = 1.
    ID 0 (factory default) is forbidden. With ID 0, a VESC frame `cmd=1` looks like DroneCAN
    *Allocation* from node 0. Rejecting ID 0 also keeps a factory-fresh VESC from being driven by
    accident.
  - `CAN_CONTROLLER_VESC_SENDER_ID = 2`: the ESP32's own ID. It goes into the `sender_id` byte of
    forwarded requests, and the VESC addresses its replies to it. It is a constant, not a node on a
    managed network, and must differ from the VESC ID.
- **Bus.** The existing MCP2515 bus at 500 kbit/s. VESC Tool *CAN Baud Rate* = `CAN_BAUD_500K`,
  *CAN Mode* = `VESC`.
- **ID spaces do not collide.**
  - Standard frames (LWS `0x305`, OBD-II `0x7DF`/`0x7E8`–`0x7EF`, LWS config `0x7C0`, ECU
    broadcasts) cannot alias extended ones.
  - The VESC ignores standard frames in VESC mode.
  - VESC frames have SID bits = 0. They win arbitration over every standard frame. They are short
    and sparse (D7), so they cannot starve the bus.
- **Not DroneCAN.** `DRONECAN_RESEARCH.md` §"Hardware / bus plan": the VESC decodes any
  extended frame as `cmd = (eid >> 8) & 0xFF`, `id = eid & 0xFF`. A DroneCAN message from node N ==
  VESC ID with a type-ID low byte of 0–4 (e.g. type 1025 → `SET_CURRENT`) would drive the motor. No
  ID choice removes that. The two buses stay separate, and merging them is out of scope.

### D2 — Duty mapping, send cadence and keep-alive
- **Mapping.** `duty_e5 = clamp(speed, −255, 255) × 100000 / 255`, in integer arithmetic,
  truncated toward zero:
  - 255 → 100000, −255 → −100000, 140 → 54901, 180 → 70588, 0 → 0.
  - The 0..255 scale and every duty constant (`STEER_PWM_MIN_DUTY` 140, `STEER_PWM_MAX_DUTY`,
    `STEER_JOG_DUTY` 180, the rate-mode curve) keep their meaning. UART `COMM_SET_DUTY` used the
    same float `speed/255` → ×1e5 scale.
- **Frame.** `CAN_PACKET_SET_DUTY` (cmd 0), ID `0x00000001`, DLC 4, payload = int32 big-endian
  `duty_e5`.
  - Duty 0 is a stop (coast), exactly like `COMM_SET_DUTY 0` today. Both end in the VESC's
    `mc_interface_set_duty()`.
- **Cadence.** Today's UART driver wrote a frame on every `setSpeed()` call, i.e. every loop
  iteration during a closed-loop move. That is not acceptable on a shared bus. A pure scheduler
  (`VescCan::DutyTxScheduler`) decides when to send:
  - `setSpeed(s)` records the wanted `duty_e5`.
  - If it differs from the last **sent** value, it is sent immediately when at least
    `STEER_VESC_CAN_MIN_TX_MS` (10 ms) has passed since the last duty frame. Otherwise it is marked
    pending, and `poll()` sends the newest wanted value once the 10 ms have passed. The maximum is
    100 duty frames/s.
  - **Duty 0 bypasses the 10 ms limit**: a stop is sent on the call that requests it.
  - **Keep-alive:** while the last sent duty is non-zero and no duty frame went out for
    `STEER_VESC_CAN_KEEPALIVE_MS` (200 ms), `poll()` re-sends it.
  - **Duty 0 is not kept alive.** After it is sent once, nothing is re-sent. The VESC is coasting,
    and its own timeout would also release the motor.
  - **Transmit failure** (`sendMsgBuf` ≠ `CAN_OK`, or CAN not initialized): the value stays pending
    and is retried on the next `poll()` after the 10 ms gap. This also applies to duty 0, so a stop
    is never silently dropped.
  - At boot `begin()` queues one duty 0, so a VESC left running by a reset is stopped.
- **VESC command timeout.**
  - The VESC's app timeout (VESC Tool → App Settings → General → *Timeout*, default 1000 ms;
    *Timeout Brake Current*, default 0 A) is reset by every `CAN_PACKET_SET_DUTY`. When it expires,
    the VESC releases the motor.
  - The 200 ms keep-alive gives 5 frames of margin against 1000 ms.
  - The timeout is also a safety net: an ESP32 that hangs mid-move (until the task watchdog resets
    it) leaves the steering motor driven for at most 1 s. That bound already existed with UART.
  - Recommendation: keep *Timeout* = 1000 ms and the brake current = 0.

### D3 — Status broadcasts (VESC → ESP32)
- **VESC Tool → App Settings → General:**
  - *CAN Status Rate 1* = 5 Hz, with messages STATUS_1, STATUS_4 and STATUS_5 selected (all others
    off);
  - *CAN Status Rate 2* = 0 / none.
  - This assumes VESC FW ≥ 6.0, which has per-message selection. See the risk on older firmware.
- **Payloads** (all big-endian, DLC 8, ID low byte = VESC ID 1). Decoded in `VescCan.h`:

| Frame | cmd | ID | Bytes |
|-------|-----|----|-------|
| `CAN_PACKET_STATUS` | 9 | `0x0901` | 0–3 int32 ERPM · 4–5 int16 motor current ×10 (A) · 6–7 int16 duty ×1000 |
| `CAN_PACKET_STATUS_4` | 16 | `0x1001` | 0–1 int16 FET temp ×10 (°C) · 2–3 int16 motor temp ×10 · 4–5 int16 input current ×10 · 6–7 int16 PID pos ×50 |
| `CAN_PACKET_STATUS_5` | 27 | `0x1B01` | 0–3 int32 tachometer · 4–5 int16 input voltage ×10 (V) · 6–7 reserved |

- **Use.**
  - `motorCurrentA()` = STATUS_1 current. It is the *average motor current*, the same quantity as
    `COMM_GET_VALUES` `avg_motor_current`, so `STEER_A` keeps its meaning.
  - `fetTempC()` = STATUS_4 FET temp.
  - `inputVoltageV()` = STATUS_5 input voltage.
  - ERPM, duty, motor temp, input current, PID pos and tacho are decoded but only logged (the
    periodic debug line in D8).
- **Length check.** A frame shorter than the bytes it needs is dropped and counted. STATUS_1 and
  STATUS_4 need 8 bytes; STATUS_5 needs 6.
- **driverOk** = a valid STATUS_1 frame from ID 1 was received within `STEER_VESC_COMM_TIMEOUT_MS`
  (1000 ms, i.e. 5 missed frames at 5 Hz).
  - It is false from boot until the first STATUS_1, so a silent VESC never blocks boot.
  - STATUS_4 and STATUS_5 do not count, so the link flag depends only on the one frame that is
    reliably received (D6).
- **Stale values.** STATUS_4/5 values keep their last decoded value. They are only shown while
  `driverOk` is true: web `--` and MAVLink `NaN` as today.

### D4 — Fault code poll through CAN forwarding
The status frames carry no fault code, so the firmware asks for it every `STEER_VESC_FAULT_POLL_MS`
(5000 ms).
- **Gating.** The poll is periodic only and has no on-demand trigger (user decision). It is sent
  only while `driverOk` is true: there is nobody to answer otherwise, and on a bus with no other
  node an un-ACKed frame would only produce transmit errors.
- **Request.** `CAN_PACKET_PROCESS_SHORT_BUFFER` (cmd 8) to the VESC, ID `0x00000801`, DLC 7:

  | Byte | Value | Meaning |
  |------|-------|---------|
  | 0 | `0x02` | `sender_id` = `CAN_CONTROLLER_VESC_SENDER_ID` (the VESC replies to this ID) |
  | 1 | `0x00` | `send` = 0: process the command and send the reply back over CAN |
  | 2 | `0x32` | `COMM_GET_VALUES_SELECTIVE` (50) |
  | 3–6 | `00 00 80 00` | mask, uint32 big-endian, bit 15 = fault code |

  - Mask bit assignment, from bldc `commands.c` (`COMM_GET_VALUES_SELECTIVE`): bit 0 temp FET,
    1 temp motor, 2 avg motor current, 3 avg input current, 4 avg id, 5 avg iq, 6 duty, 7 rpm,
    8 v_in, 9–12 Ah/Wh, 13 tacho, 14 tacho abs, **15 fault code (uint8)**, 16 PID pos,
    17 controller ID, …
  - **Assumption:** bit 15 = fault code (stable since FW 3.x), checked from memory against bldc
  `commands.c`. Bench task 12.6 verifies it by
    forcing a fault. If the reply's echoed mask differs from the request, the reply is rejected and
    logged, so a wrong assumption shows up as `steer_vesc_fault_stale` and never as a wrong code.
- **Reply payload.** `[0x32][mask BE 4 bytes][fault uint8]`, 6 bytes.
  - **Assumption** (bldc `comm_can.c` `comm_can_send_buffer()`): a payload of at most 6 bytes is
    sent as **one** `CAN_PACKET_PROCESS_SHORT_BUFFER` frame to the requester. The request's
    `send = 0` makes `commands_process_packet()` reply through `send_packet_wrapper` →
    `comm_can_send_buffer(rx_buffer_last_id, …, 1)`.
  - So the **expected** reply is a single frame:
  - ID `0x00000802` (cmd 8, low byte = our ID 2), DLC 8;
  - data `[sender = 0x01][send flag][0x32][00 00 80 00][fault]`. The send flag is 1 in bldc;
    any value is accepted.
- **Long-reply path.** It is implemented for robustness: some firmware appends fields, or the mask
  may later grow. A reply longer than 6 bytes arrives as:
  - `CAN_PACKET_FILL_RX_BUFFER` (cmd 5, ID `0x00000502`), one per 7 bytes:
    `[index][up to 7 data bytes]`, stored at `buf[index]`;
  - then `CAN_PACKET_PROCESS_RX_BUFFER` (cmd 7, ID `0x00000702`), DLC 6:
    `[sender = 0x01][send flag][len_hi][len_lo][crc_hi][crc_lo]`.
  - The firmware reassembles into a 64-byte buffer. It accepts the result only if:
    - the sender is the VESC ID;
    - `len` ≤ 64 and ≤ the highest byte written;
    - `crc16(buf, len)` matches. This is CRC16-XMODEM, the same CRC as the UART packet layer,
      kept in `VescProtocol.h`.
  - Then it parses the same selective payload.
  - `FILL_RX_BUFFER_LONG` (cmd 6) is not needed: the payload is < 256 bytes. Such frames are
    ignored.
- **Reply handling.**
  - A valid reply with mask `0x00008000` sets `faultCode()` and clears the stale flag.
  - If no valid reply arrives within `STEER_VESC_FAULT_REPLY_MS` (500 ms) of a request, the stale
    flag is set and the last fault code is kept.
  - A missing reply never changes `driverOk`. Only STATUS_1 does that.
  - While `driverOk` is false the fault code is not polled. The last code is kept, and the stale
    flag is set.
- **Fault reaction.** It is unchanged. `SteeringController` stops the motor and takes the
  stall-latch path on `hasFault()`.
  - Detection latency grows from ≤ 300 ms to ≤ 5 s. That is acceptable: the VESC stops the motor
    itself when it faults, so the firmware stop is a backstop that latches the direction and
    reports. Over-current protection is the VESC's own *Motor Current Max* clamp, as before.

### D5 — MCP2515 acceptance masks and filters
**How the chip filters** (MCP2515 datasheet §4.5):
- Each buffer has one mask. RXB0 uses mask 0 with filters 0–1; RXB1 uses mask 1 with filters 2–5.
- A mask has SID10..0, EID17..16 and EID15..0 bits, and no IDE bit.
- Each filter has the same bits plus `EXIDE`. An `EXIDE` = 0 filter matches only standard frames,
  and an `EXIDE` = 1 filter matches only extended frames. RXM = 00 is set by `MCP_STDEXT`, which the
  firmware already uses.
- For an **extended** frame, the 29-bit ID maps as SID10..0 = ID28..18, EID17..0 = ID17..0.
- For a **standard** frame, the mask's EID15..0 bits are compared against **data bytes 0 and 1**
  (data-byte filtering, §4.5.1). This applies to every filter.

**How `mcp_can` encodes masks and filters** (`mcp2515_write_mf()`):
- `ext = 0`: argument bits 26..16 = the 11-bit SID, and bits 15..0 go to EID15..0. `EXIDE` is clear.
- `ext = 1`: the argument is the raw 29-bit ID. `EXIDE` is set; it is ignored in the mask registers.

**Rejected option.** One RXB1 mask `0x1FFF00FF` (ext), with extended filters matching low byte 1
and low byte 2, plus a standard filter for `0x305`. The mask's EID7..0 care bits would compare LWS
data byte 1 (the angle's low byte) against the filter. Only ~1/256 of LWS frames would pass. So any
mask that cares about the VESC low byte breaks the standard `0x305` path on the same buffer.

**Chosen layout (the simplest robust option):**

| Buffer | Register | `mcp_can` call | Value | Accepts |
|--------|----------|----------------|-------|---------|
| RXB0 | Mask 0 | `init_Mask(0, 0, 0x07F80000)` | SID `0x7F8`, EID 0 (unchanged) | — |
| RXB0 | Filter 0 | `init_Filt(0, 0, 0x07E80000)` | std `0x7E8` (unchanged) | std `0x7E8`–`0x7EF` (OBD-II replies) |
| RXB0 | Filter 1 | `init_Filt(1, 1, 0x00000000)` | **ext, ID 0** (was std `0x7E8`, a duplicate of filter 0) | ext IDs with ID28..21 = 0, i.e. `0x00000000`–`0x001FFFFF`: every VESC frame |
| RXB1 | Mask 1 | `init_Mask(1, 0, 0x07FF0000)` | SID `0x7FF`, EID 0 (unchanged) | — |
| RXB1 | Filters 2–5 | `init_Filt(n, 0, 0x03050000)` | std `0x305` (unchanged) | std `0x305` only (LWS) |

- Mask 0's EID bits are 0. So a standard OBD reply's data bytes are not filtered, and an extended
  frame is matched on ID28..21 only.
- RXB1 still rejects every extended frame: all its filters have `EXIDE` = 0.
- The ECU's MT05 broadcasts (`0x301` …) stay rejected.
- An ECU 29-bit diagnostic ID (`0x18DAxxxx`) would also be rejected (ID28..21 ≠ 0).
- **Software dispatch** narrows "ext ID < `0x200000`" to the frames this firmware uses:
  - bits 31/30 of the `readMsgBuf` ID flag extended/remote. A remote frame is ignored.
  - `eid` ≤ `0xFFFF`.
  - `(eid & 0xFF) == STEER_VESC_CAN_ID` and cmd ∈ {9, 16, 27} → status.
  - `(eid & 0xFF) == CAN_CONTROLLER_VESC_SENDER_ID` and cmd ∈ {8, 5, 7} → fault reply.
  - Any other extended frame is counted and dropped, and is not logged as an unmatched OBD frame.
- **Why VESC frames go to RXB0 and not RXB1:**
  - RXB0 has the `BUKT` rollover into RXB1. A VESC frame arriving while RXB0 is full still lands in
    RXB1, so VESC frames get a **2-deep** path.
  - RXB0's other traffic is at most one outstanding OBD reply. RXB0 is therefore nearly always empty
    when a VESC burst starts.
  - RXB1 has no rollover, and it is busy with the 100 Hz LWS stream.
- The constants become `CAN_RX_VESC_FILTER_REG` = `0x00000000` (ext) next to the existing
  `CAN_RX_OBD_*` and `CAN_RX_LWS_*`.
- `applyRxFilters()` still sets both masks and all six filters, and still runs after every chip
  reset.
- `Mcp2515Filter.h` models the acceptance rule, including data-byte filtering, so the native test
  can prove all of this:
  - the chosen values accept `0x305` with any data, `0x7E8`–`0x7EF`, `0x0901`, `0x1001`, `0x1B01`,
    `0x0802`, `0x0502` and `0x0702`;
  - they reject `0x301`, `0x7DF`, `0x7C0` and `0x18DAF110`;
  - the rejected `0x1FFF00FF` layout drops `0x305` frames whose data byte 1 ≠ 0.

### D6 — Receive timing: hard dependency on `move-can-rx-to-interrupt`
- **Frame times at 500 kbit/s** (2 µs/bit):
  - standard DLC 6 ≈ 95 + ≤ 20 stuff bits → 190–230 µs;
  - extended DLC 8 ≈ 131 + ≤ 29 stuff bits → 262–320 µs, about 260 µs apart back-to-back.
- **The VESC burst.** The bldc status thread (`comm_can.c`) sends every enabled STATUS message
  (1, 4, 5) **back-to-back in one tick**. A multi-frame forwarded reply (FILL… + PROCESS) is sent
  back-to-back the same way. The VESC cannot stagger them: all messages of one rate group go out in
  the same tick.
- **What the MCP2515 can hold.**
  - It has two receive buffers. `BUKT` only rolls RXB0 into RXB1, never the other way.
  - Today the firmware drains them by **polling from the cooperative main loop**. That loop also
    serves the web, telemetry and MAVLink, and its period is milliseconds.
  - A 3-frame burst needs a buffer emptied within ≈ 1–2 frame times (≈ 260–550 µs) of its first
    frame. In the polled mode, **2 of every 3 status frames would be lost systematically**:
    - Through RXB1 (no rollover), only the first frame of each burst survives.
    - Through RXB0 + rollover, the third frame is lost on every burst, and the second one whenever
      RXB1 holds an LWS frame.
- **Hard dependency.** This change **requires** the CAN receive path to be interrupt/task driven
  first. That is change `move-can-rx-to-interrupt` (firmware 1.6.0): MCP2515 `INT` on
  `PIN_CAN_INT` (GPIO38), a priority-12 reader task that empties RXB0/RXB1 into a software queue
  within tens of µs, and a single queue consumer (`routeRxFrame()`) in `update()`.
  - That change SHALL be implemented first and archived **immediately before** this one
    (proposal "archive order", tasks 0.x).
  - This change does not design any workaround for polled reception: no staggering, and no
    reliance on rollover luck.
- **What this change assumes from it:**
  - Every frame accepted by the hardware filters (D5) reaches `dispatchSharedFrame()`.
  - Back-to-back bursts of at least 3 extended frames are received.
  - Frames keep their arrival order within one buffer.
  - The RX overflow counter stays at 0 under LWS + VESC + OBD traffic.
  - MCP2515 transmits (`sendVescFrame`, OBD-II, LWS zero-set) stay safe against the reader task.
    That change defines one SPI mutex with bounded waits, and a non-blocking `queueTx()` that does
    not hold the mutex while the frame waits for arbitration. `sendVescFrame()` uses `queueTx()`
    so a keep-alive never blocks the reader during a status burst.
- **Buffer layout.** The D5 layout still matters with a fast reader. VESC frames use RXB0 with
  rollover (2-deep), so a burst is safe even when the reader is briefly delayed. LWS keeps RXB1 to
  itself, and OBD replies share RXB0 only with sparse VESC traffic.
- **driverOk** depends only on STATUS_1 (D3). A rare lost STATUS_4/5 never affects the link flag.
- **Measurement.**
  - `VescCanDriver` counts frames per type: STATUS_1/4/5, fault replies, rejects and ignored
    extended frames.
  - A `[VESC]` debug line every `STEER_VESC_RX_LOG_MS` (5 s) prints the per-type rates and the
    MCP2515 RX overflow count.
  - Bench acceptance (task 12.4):
    - STATUS_1, STATUS_4 and STATUS_5 each 5 Hz ± 0.5;
    - RX overflow = 0;
    - LWS checksum, counter and miss counters stay 0 (miss counter: no growth beyond what it
      shows with the VESC unplugged);
    - LWS `timeout` never.

### D7 — Bus-load estimate (500 kbit/s, worst-case bit stuffing)

| Traffic | Frame | Rate | Load |
|---------|-------|------|------|
| LWS `0x305` | std DLC 6, ≤ 115 bit | 100 /s | 11.5 kbit/s (2.3 %) |
| VESC STATUS_1/4/5 | ext DLC 8, ≤ 160 bit | 15 /s | 2.4 kbit/s (0.5 %) |
| VESC SET_DUTY, moving | ext DLC 4, ≤ 120 bit | ≤ 100 /s (10 ms limit) | ≤ 12.0 kbit/s (2.4 %) |
| VESC SET_DUTY, holding / keep-alive | ext DLC 4 | 5 /s | 0.6 kbit/s (0.1 %) |
| VESC fault poll + reply | ext DLC 7 + DLC 8 | 2 per 5 s | 0.06 kbit/s |
| OBD-II request + reply | std DLC 8, ≤ 135 bit each | ≤ ~23 pairs/s (RPM at 50 ms during a gear change, others ≥ 500 ms) | ≤ 6.2 kbit/s (1.2 %) |
| **Firmware-related total** | | | **≤ ~33 kbit/s (≈ 6.5 %)** |

- The ECU's own MT05 broadcasts come on top of this. Their rate is unmeasured. They are filtered in
  hardware and do not cost CPU.
- Even at several hundred frames/s they would leave the bus far below saturation. The new VESC
  traffic adds ≤ 3 % at worst.
- The VESC frames' arbitration priority (SID 0) cannot block LWS or OBD frames for more than one
  frame time each.

### D8 — Driver structure
- **`include/VescCan.h`** — pure, header-only, Arduino-free. It includes `VescProtocol.h`:
  - CAN packet IDs: `SET_DUTY = 0`, `FILL_RX_BUFFER = 5`, `FILL_RX_BUFFER_LONG = 6`,
    `PROCESS_RX_BUFFER = 7`, `PROCESS_SHORT_BUFFER = 8`, `STATUS = 9`, `STATUS_4 = 16`,
    `STATUS_5 = 27`.
  - `makeEid(cmd, id)` and `splitEid(eid, cmd, id)`.
  - `dutyE5FromSpeed(int16_t)` and `encodeSetDuty(int32_t, uint8_t out[4])`.
  - `decodeStatus1/4/5(data, len, out)`.
  - `buildFaultPoll(senderId, out[7])`.
  - `parseSelectiveFault(payload, len, &fault)`: checks the command byte, the echoed mask and the
    length.
  - `RxReassembler`: FILL / PROCESS handling, the 64-byte buffer and the CRC check.
  - `DutyTxScheduler`: D2's rules as a pure state machine taking `now`.
- **`include/VescProtocol.h`** — header-only, Arduino-free (`<stdint.h>`):
  - `crc16()` (CRC16-XMODEM);
  - `CommId` with `COMM_GET_VALUES_SELECTIVE = 50` added;
  - big-endian get/put helpers.
  - `frameShort()`, `encodeSetDuty()` (UART form) and `decodeGetValues()` are removed together
    with `src/VescProtocol.cpp`. They have no user left.
- **`include/Mcp2515Filter.h`** — pure: `accepts(maskRegs, filterRegs, frame)`, built from the
  `mcp_can` argument encoding, used by the native test.
- **`VescCanDriver : IMotorDriver`** (`include/VescCanDriver.h`, `src/VescCanDriver.cpp`):
  - Constructor `(uint8_t vescId, uint8_t ownId)`.
  - `attachCan(CANController*)`.
  - `onFrame(uint32_t eid, const uint8_t* data, uint8_t len, uint32_t nowMs)`, called by
    `CANController` dispatch.
  - `setSpeed()` / `stop()` → scheduler, plus an immediate send when due.
  - `poll()` sends pending, keep-alive and fault-poll frames, checks the fault reply timeout, and
    writes the 5 s debug line.
  - `driverOk()`, `hasFault()`, `faultCode()`, `faultStale()`, `motorCurrentA()`, `fetTempC()` and
    `inputVoltageV()`.
  - It owns no transport. Every transmit goes through `CANController::sendVescFrame()`.
- **`CANController`:**
  - `attachVescSink(VescCanDriver*)`.
  - `bool sendVescFrame(uint8_t cmd, const uint8_t* data, uint8_t len)`:
    - **whitelist**: cmd ∈ {`SET_DUTY`, `PROCESS_SHORT_BUFFER`};
    - ID always `(cmd << 8) | STEER_VESC_CAN_ID`, `ext = 1`, len ≤ 8;
    - refuses when not initialized;
    - callable in every CAN state (IDLE, WAITING_RESPONSE, PROBING, LWS zero-set), because steering
      never waits for OBD-II.
    - VESC transmit failures are counted separately. They do **not** feed
      `consecutiveSendFailures_`, so VESC traffic cannot trigger an MCP2515 re-init. OBD traffic
      still detects a wedged chip.
  - `dispatchSteeringFrame()` becomes `dispatchSharedFrame()`. It is called first on every receive
    path:
    - standard `0x305` → `LwsSensor`;
    - extended (D5 rules) → `VescCanDriver`;
    - it returns true for anything it consumed or deliberately dropped.
- **`IMotorDriver`.** The interface is unchanged. `faultStale()` is added as a new optional hook
  that returns false by default, and the comments say "VESC over CAN".
- **Wiring:**
  - `main.cpp` constructs
    `VescCanDriver steeringVesc(STEER_VESC_CAN_ID, CAN_CONTROLLER_VESC_SENDER_ID)` before
    `SteeringController steeringActuator(steeringVesc)`.
  - The `steeringVesc.begin(PIN_VESC_RX, …)` UART init is removed.
  - `vehicleController.attachSteeringVesc(&steeringVesc)` is called before `initCAN()`.
  - `VehicleController::initCAN()` does `canController_.attachVescSink(vesc)` and
    `vesc->attachCan(&canController_)`. Both are attached even if CAN init failed, so the driver
    simply reports down.
- **Order inside one loop iteration:** `canController_.update()` (receive + dispatch) runs before
  `steering_.update()` → `motor_.poll()` (transmit). That order is already in `VehicleController`.

### D9 — What stays unchanged
- `SteeringController`: position loop, jog, nudge, rate drive, stall latch, boot centring, the
  fault and comm stops. Only comments change. The `IMotorDriver` contract is identical.
- The LWS decoder, its health logic, the zero-set sequence (still the only `0x7C0` transmitter),
  the OBD-II scheduler and the ECU probe.
- MAVLink: `STEER_A`, `VESC_V`, `VESC_TEMP` and `VESC_OK` keep their names, units and gates.
  `VESC_OK` follows `driverOk` (STATUS_1). The data now refreshes at 5 Hz instead of 3.3 Hz.
- Web telemetry key names, the `steer_vin_low` rule and the "Увага" lines.
- NVS, calibration and every steering constant.
- Logging stays off UART0. `CORE_DEBUG_LEVEL=0`, `uartSetDebug(NULL)` and the `esp_log` redirect
  are kept: they cost nothing and keep the freed X1 quiet. Only their comments change.

## Risks / Trade-offs
- **One bus for angle and drive.**
  - An MCP2515 or bus failure (`can_down`, shorted CAN_H/L, a cable off) now takes out both the LWS
    and the VESC.
  - Sensorless jog and rate mode no longer survive a CAN failure. Steering stops: `driverOk` goes
    false within 1 s, and the VESC times out within 1 s.
  - Accepted (user decision). Bench task 12.11 covers it.
- **VESC CAN timeout.** If the keep-alive stalls, for example during a long blocking operation or a
  chip re-init, the VESC releases the motor after 1 s. That is the safe direction. The keep-alive
  margin is 5×.
- **Chip re-init.** `CAN_REINIT_NO_DATA_MS` (15 s without an OBD reply, e.g. ECU off on the bench)
  resets the MCP2515. A duty frame in flight can be lost: the keep-alive restores a non-zero duty
  within 200 ms, and a pending stop is retried. Masks and filters are re-applied as before.
- **RX overrun / dependency.** Without `move-can-rx-to-interrupt`, two of the three status frames
  of every burst are lost (D6). The change must not be implemented before that one is in place.
  Task 0.2 is a hard gate.
- **Older VESC firmware.** Before FW 6.0, VESC Tool offers only cumulative sets (STATUS_1,
  1-2, …, 1-2-3-4-5) at one rate.
  - "1-2-3-4-5" makes a 5-frame burst, carrying STATUS_2/3, which are not used. The interrupt-driven
    reader must absorb it, and the frames are dropped in software.
  - Bench task 12.1 records the FW version. Updating to FW ≥ 6.0 is recommended, so that only
    STATUS_1/4/5 are sent.
- **Fault latency.** It is ≤ 5 s instead of ≤ 0.3 s (D4). The VESC still stops itself on a fault.
- **Fault-mask assumption.** Bit 15 = fault code (D4). A mismatch is detected by the echoed mask and
  shows as stale.
- **Termination.** The VESC adds a node. Wrong termination (one or three 120 Ω resistors) causes
  errors on all three devices. The bench measures ≈ 60 Ω with power off.
- **Bus priority.** VESC frames (SID 0) always win arbitration. Rate limiting (D2) bounds their
  share to ≤ 3 %.

## Migration
1. Flash 1.7.0 and upload the matching LittleFS image.
2. Set the VESC in VESC Tool over USB: Controller ID 1, CAN baud 500K, CAN mode VESC, status
   STATUS_1/4/5 at 5 Hz, Timeout 1000 ms, brake current 0.
3. Disconnect the VESC UART from X1.
4. Wire the VESC CAN port to the bus: CAN_H, CAN_L, GND.
5. Check the termination (≈ 60 Ω).
6. No recalibration is needed.
7. Rollback = flash 1.6.0 (interrupt-driven RX, UART VESC) and re-wire the UART. The VESC
   settings may stay as they are.

## Open Questions
None blocking.
- `STEER_VESC_CAN_KEEPALIVE_MS` = 200 ms against the 1000 ms VESC timeout is bench-tunable.
  Recommendation: keep both values.
- The receive path itself is defined by `move-can-rx-to-interrupt`, not here.
