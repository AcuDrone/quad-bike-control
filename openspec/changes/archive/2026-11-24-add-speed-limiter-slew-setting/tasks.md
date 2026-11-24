## 1. Firmware
- [x] 1.1 `Constants.h`: `SPEED_LIMIT_SLEW_DEFAULT_PCT_S` (80), `SPEED_LIMIT_SLEW_MIN_PCT_S` (10), `SPEED_LIMIT_SLEW_MAX_PCT_S` (200), `SPEED_LIMIT_SLEW_NVS_KEY` ("thr_slew"); remove `SPEED_LIMIT_CEILING_SLEW_PCT_S`; update the limiter comment block.
- [x] 1.2 `VehicleController`: load `thr_slew` at boot from NVS "speed" (NaN / out of range → logged, default, not rewritten); `getSpeedLimitSlewPctS()`; pass the live value as `SpeedLimiterParams.slewPctS`.
- [x] 1.3 `set_speed_limit_slew` → `processSetSpeedLimitSlewCommand()`: validate 10–200 %/s, persist first, then apply; ack/error replies like the floor command.

## 2. Telemetry and web
- [x] 2.1 `speed_limit_slew` (one decimal) in `WebPortal::Telemetry`, `TelemetryManager`, `createTelemetryJSON` (capacity note updated).
- [x] 2.2 `data/index.html`: rate row (number 10–200, step 5, Save), hint line, local validation, fill from telemetry, help sentence; uk + en keys; route `Speed-limiter …` replies to the card status line.

## 3. Tests, docs
- [x] 3.1 `test/test_speed_limiter`: `test_lower_slew_falls_slower` (20 vs 80 %/s over 0.5 s).
- [x] 3.2 `Changelog.h` 1.5.12 bullet (uk + en).
- [ ] 3.3 Bench: the row shows 80 on first boot; set 40, reboot, it reads back 40; limiter closes visibly slower on a test drive.
