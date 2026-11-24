## ADDED Requirements
### Requirement: Service Brake Episode Line
On aztec, the journal SHALL record one line per service-brake request episode under the source
`BRAKE`:
`BRAKE I episode ms=<n> rod_max=<mm|-> spd_min=<n> runout=<min>..<max> stop=<0|1>`
(`runout=-` when no non-zero sample was taken). This line is the only journal line for normal
braking; the `BRAKING` / `RELEASING` state transitions themselves SHALL stay unjournaled.

- An episode SHALL start in the slot whose brake state enters `BRAKING` from `READY` and SHALL end in
  the first slot whose state is neither `BRAKING` nor `RELEASING`; `RELEASING` back to `BRAKING`
  SHALL continue the same episode. The line SHALL be written once, when the episode ends, whatever
  state ends it.
- A sample SHALL be the byte 3 (pRunout, bar) of each checksum-valid, trusted `0x38F` frame newly
  decoded since the previous slot, taken only in a slot in which the request is sent and a fresh
  `0x39D` reports internal state 3. No other `0x38F` frame SHALL be sampled.
- `runout` SHALL be the lowest and highest non-zero sample; `stop` SHALL be 1 if any sample was 0
  (stroke stop), else 0; `spd_min` SHALL be the lowest `0x38B` byte 2 sent in the episode; `rod_max`
  SHALL be the highest fresh rod position in mm (`-` if none); `ms` SHALL be the episode duration.
- The line SHALL NOT be exempt from the 2 s (source, event) dedupe: at most one `episode` line per
  2 s, the held-back ones counted and appended as `sup=<n>` to the next written one.
- The tracker SHALL be pure logic in `include/IboosterLaw.h`, native-tested.

#### Scenario: Commanded brake at standstill
- **WHEN** the brake is commanded at standstill, the unit accepts the request and reports 103 bar
  throughout, and the brake is then released to `READY`
- **THEN** exactly one line `BRAKE I episode ms=<n> rod_max=<mm> spd_min=5 runout=103..103 stop=0`
  SHALL be journaled when the state returns to `READY`

#### Scenario: Before acceptance and after release
- **WHEN** the unit reports 50 or 51 before it has accepted the request (internal state not 3), or
  reports the decay from 103 to 50 after the request is cleared
- **THEN** those values SHALL NOT appear in `runout`

#### Scenario: Stroke stop
- **WHEN** the unit reports 0 in some samples and 103 in others within one episode
- **THEN** the line SHALL show `runout=103..103 stop=1`
- **AND** when it reports only 0, the line SHALL show `runout=- stop=1`

#### Scenario: Ceiling changes during a hold
- **WHEN** the unit reports 103 and later 51 within one episode
- **THEN** the line SHALL show `runout=51..103`

#### Scenario: Re-press during release
- **WHEN** the state goes `BRAKING` → `RELEASING` → `BRAKING` → `RELEASING` → `READY`
- **THEN** one line SHALL be journaled, covering the whole sequence

#### Scenario: Episode ended by trouble
- **WHEN** an episode ends because the state becomes `LOST`, `FAULT`, `STARTING` or `ASLEEP`
- **THEN** the `episode` line SHALL still be journaled, after the `state` line of that slot

#### Scenario: Pumping the brake
- **WHEN** episodes end less than 2 s apart
- **THEN** at most one `episode` line SHALL be written per 2 s, and the next written one SHALL carry
  `sup=<n>` with the number held back
