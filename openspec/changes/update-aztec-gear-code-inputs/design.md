## Context
The board, the U10 expander (PCA9557 @ 0x1A on I2C2), the X16 connector and the opto polarity are
the same as on the quad bike. A line reads HIGH ("1") when it is engaged. Only the meaning of the
lines changes. On aztec, three of them form a binary code and bit 7 is not wired.

## Decisions
- **Code bit order**: the code is written `[bit0 bit1 bit6]` and packed as
  `(bit0 << 2) | (bit1 << 1) | bit6`, so the literal `0b101` reads exactly like the table
  (L = `101`, H = `001`, N = `011`, R = `010`).
- **One decode point**: `BoardInputs` stays a raw reader. It publishes the undecoded 3-bit code and
  never looks at bit 7. `TransmissionController::decodeGearCode()` is the only code-to-gear mapping.
  It is a `const` instance method over the runtime table, so telemetry can show the instant decode
  next to the cached physical gear.
- **Runtime code table, web-calibrated**: `gearCodes_[4]` (indexed by `Gear`) starts from the
  `GEAR_CODE_*` defaults (their `static_assert`s stay). The web "Save as X" stores the CURRENT live
  code for gear X: the operator puts the lever in a known gear and captures what the ESP reads, so a
  selector whose table differs from the assumed one is fixed without a rebuild. Persisted in the
  existing `"transmission"` NVS namespace (keys `code_r/n/h/l`, lowercase like `pct_r`/`ovr_r`),
  loaded first thing in `begin()` (independent of the servo). A missing key keeps that default; an
  invalid table (assigned `000`, `111`, >7, duplicates) falls back wholesale to the defaults with an
  error log. "Reset to defaults" removes the keys.
- **Capture validation keeps the fail-safe**: `000` and `111` stay reserved for UNKNOWN and a stale
  snapshot is refused. A capture during a gear change is refused (the code is mid-transition).
- **Take-over instead of a duplicate reject**: capturing a code that another gear owns moves it to
  the captured gear and leaves the other gear UNSET (`GEAR_CODE_UNSET` = `0xFF`, in RAM and NVS). An
  UNSET gear never matches, so it reads UNKNOWN (throttle capped) until captured — fail-safe, and the
  invariant is "assigned codes are distinct". This lets a swapped pair be fixed in two captures. The
  previous owner's NVS key is written first, so a power loss between the two writes leaves the code
  unassigned rather than assigned twice. Telemetry shows an UNSET entry as `-1`. Capture and reset are refused while the engine runs — the same gate as
  throttle calibration. After a change the gear cache is invalidated and the code re-logged.
- **Live view needs no new field**: `gear_code` already reads the snapshot directly (it never goes
  through the 100 ms gear cache), so the calibration card reuses it. Telemetry adds the active table
  `gear_codes {R,N,H,L}` and `gear_codes_custom` (entries that differ from the default; bit0 R,
  bit1 N, bit2 H, bit3 L) — six small values, well inside the 4096-byte document.
- **Fail-safe is preserved**: with all lines open (e.g. mid-shift) the code reads `000`. With the
  opto feed lost, all lines float HIGH and the code reads `111`. Neither is assigned to a gear, so
  both decode to `GEAR_UNKNOWN`, just as none-active and all-active did in the one-hot decode.
  `static_assert`s keep it that way. `100` and `110` are also UNKNOWN.
- **Retry intent kept**: the one-hot "activeCount != 1 while idle → consume the next snapshot"
  becomes "code decodes to UNKNOWN while idle → consume the next snapshot". It uses the same
  `TRANS_GEAR_READ_RETRY_COUNT` and the same non-blocking rule (no back-to-back I2C reads).
- **Telemetry**: `gear` (cached physical gear) is unchanged. `gear_code` is the raw code, or -1 while
  the snapshot is stale. `gear_sensed` is that code decoded without the retry cache. Together with
  the existing `io_raw` they let the bench verify the wiring per lever position.

## Risks / Trade-offs
- A single-bit glitch between two valid codes (e.g. N `011` → R `010`) decodes to another valid gear
  rather than to UNKNOWN, so the UNKNOWN retry does not filter it. A one-hot decode would have
  caught it. Mitigation: a transient mis-read is held for at most one `TRANS_GEAR_READ_INTERVAL_MS`
  (100 ms) cache window, and the next read corrects it. The code table is fixed by the selector
  hardware, so it cannot be re-ordered to add Hamming distance.
- The code table is unverified on the vehicle until the bench sweep (tasks 4.x); the web capture
  lets the bench fix it in place.
- After a take-over the other gear reads UNKNOWN in its lever position until the operator captures
  it. The response names that gear, and the web table shows it as "not set" in a warning colour.
