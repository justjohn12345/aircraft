# The A32NX SEC (generated `SecComputer`) and its readable rewrite

The Spoiler Elevator Computer model: ADR / IR selection (two of each per SEC), LGCIU monitoring and on ground, the
flight / ground chart, hydraulic supply, sidestick priority, engagement (SECs 1 and 2 take pitch only when both ELACs
lost it; SEC 3 has no elevator), the pitch law capability (alternate 1 / 2, direct) with the alternate and direct
laws, the THS (SEC 1), and the spoilers: speed brakes (with inhibition), ground spoilers (landing, rejected take-off),
partial lift dumping, and the roll order from an ELAC (or the SEC's own direct law) on its spoiler pairs. 1439
generated lines; the laws are the ELAC's (`docs/elac.md`): `LateralDirectLaw` (stepped on the roll stick), 
`PitchAlternateLaw`, `PitchDirectLaw`.

- Generated: `fbw-a32nx/src/wasm/fbw_a320/src/model/SecComputer.cpp` (+ `.h`, `_data.cpp`, `_types.h`). Never edited;
  identical in every lab variant.
- Readable: `tools/fmgc-lab/readable/sec/` (namespace `sec::readable`), laws in `readable/efcs/`; bit-exact (below).
- Wrapper: `fbw_a320/src/sec/Sec.cpp`; sim wiring `FlyByWireInterface::updateSec()` (~1492).
- Parameters: `docs/params-SecComputer.txt`. Findings: `docs/findings.md` section 10 (S1-S6).

## Where it runs

SEC 1, 2, 3 run after the ELACs (they see this frame's ELAC buses and discretes) and before the FACs and FCDCs (their
buses are the previous frame's for the ELACs). SEC 1 sees SEC 2's previous outputs, SEC 2 SEC 1's of this frame. Spoiler
pairs (wrapper indexes): SEC 1 spoilers 3 and 4, SEC 2 spoiler 5, SEC 3 spoilers 1 and 2. ADR / IR: SEC 1 1 + 3, SEC 2
1 + 2, SEC 3 2 + 3. Power: DC ESS (SEC 1, or the battery) / DC 2. Unhealthy: bus FailureWarning with 0 (the speed brake
command word left at {0, 0}), discretes false except `sec_failed` (the wrapper's self test / fault light), orders 0.
The model holds while `computer_running` is false and restarts on the first running frame (232-280): everything reset
but the THS integrator (reloaded).

## Structure (generated line ranges -> readable)

| Generated | What | Readable |
|---|---|---|
| 24-152 | library blocks (BitfromLabel, rate limiters, pulse / confirm nodes, SSM test, bit packing) | `efcs/efcs_blocks.h` |
| 232-280 | restart | `sec.cpp` `restart()` |
| 282-402 | ADR / IR validity and selection (S1) | `sec.cpp` `selectAirData` |
| 404-506 | LGCIU words: gear not uplocked, uplock disagreement / fault, on ground chart | `sec.cpp` `monitorLandingGear` |
| 508-557 | tracking, flight / ground chart (no RA), hydraulics | `sec.cpp` `updateFlightPhase` |
| 560-644 | sidestick priority, roll stick, elevators, THS, pitch priority, spoiler pairs | `sec.cpp` `updateEngagement` |
| 645-720 | abnormal attitude, pitch law capability, pitch stick, THS ground setting | `sec.cpp` `updateLawCapability` |
| 721-830 | speed brake inhibition, ground spoilers, partial lift dumping, ground spoiler order | `spoilers.cpp` |
| 831-857 | speed brake command | `spoilers.cpp` |
| 858-987 | roll order and the four spoiler orders | `spoilers.cpp` |
| 988-1138 | flap index, alternate and direct laws, elevator order, THS integrator, battery supply | `pitch.cpp` |
| 1139-1405 | discrete words, outputs, bus | `output_packing.cpp` |

## Inputs (`sec_inputs`)

| Field | Sim source | Used for |
|---|---|---|
| `pitch_not_avail_elac_1/2` | !ELAC pitch axis ok (SECs 1, 2) | pitch priority, THS, speed brake inhibition, ELAC elevator order |
| `left/right_elev_not_avail_sec_opp` | the other SEC's elevator ok (SECs 1, 2) | SEC 1 priority, inhibition |
| `digital_output_failed_elac_1/2` | !ELAC digital output validated | "engaged in roll" |
| `*_low_pressure`, servo failures, takeover buttons, THS override | as the ELAC | |
| `sec_engaged_from_switch`, `sec_in_emergency_powersupply`, `sfcc_*_slats_out`, `load_factor_acc_*` | | unused |
| sticks (pitch: SECs 1, 2), elevators, THS (SECs 1, 2) | as the ELAC | laws, bus |
| `spd_brk_lever_pos` | handle position, -0.05 when armed | speed brakes, ground spoilers |
| `thr_lever_1/2_pos` | TLAs | ground spoilers, inhibition, bus (ELAC pitch law) |
| spoiler positions | hydraulics deflections x 50 | orders while unavailable, bus |
| `wheel_speed_left/right` | wheel rpm x 0.118921 | rejected take-off (23 / 72 kt) |
| ADR / IR, ELAC 1 / 2, FCDC 1 / 2, SFCC 1 / 2, LGCIU 1 / 2 buses | | as above |

## Functions

- **ADR**: both valid: ADR 2's data (averaged with itself, S1); one valid: that one; none: 0. **IR**: average / the
  valid one / 0.
- **Landing gear**: a gear not uplocked (LGCIU word 3 bits 11-13) from a valid LGCIU when the other is invalid or
  agrees; fault when both LGCIUs are invalid or one reports bit 14 (handle up) with no gear while the other reports bit
  14 with a gear for 10 s. On ground: any main gear compressed (word 2 bits 13 / 14 of either LGCIU).
- **Pitch**: SEC 1 on blue, SECs 2 / 3 green (left) / yellow (right); SEC 3 never engages. SEC 2 has priority when both
  ELACs lost pitch, SEC 1 when SEC 2's elevators are lost too. Capability: direct with both IRs failed or in flight
  with a gear not uplocked (SFCC bit 20 with the LGCIUs in fault); alternate 2 with both ADRs failed, after an abnormal
  attitude, or an elevator lost while neither ELAC is engaged in roll; else alternate 1. Abnormal attitude as the ELAC
  (unfiltered alpha). THS (SEC 1 only, both ELACs' pitch lost): as the ELAC, including the ground setting.
- **Speed brakes**: the lever (+-1, half with an ELAC's AP bit) through 0 / -2 / -25 / -40 deg at 0.05 / 0.051 / 0.5 / 1,
  5 deg/s; inhibited (latched until the lever is retracted for 10 s) by an ELAC's alpha protection, flaps FULL, a thrust
  lever at 35 deg or more, or the elevators lost.
- **Ground spoilers** (-50 deg, 20 deg/s): with the lever armed or extended and both thrust levers below 20 deg, or a
  reverser selected (the other at 35 deg or less): at touchdown (both main gears: left on LGCIU 1, right on LGCIU 2,
  S4), or above 72 kt wheel speed after the wheels were below 23 kt on the ground (rejected take-off). **Partial lift
  dumping** (-10 deg): one main gear (the same two signals) under the same lever conditions, or at touchdown with the
  lever armed and both thrust levers at 25 deg or less.
- **Roll**: ELAC 1's roll spoiler order, else ELAC 2's, else the SEC's direct law x 35 / 25, +-35 deg; ELAC discrete
  word 1 bit 28 enables SEC 1's pair 1 and bit 29 SEC 3's pair 2. The roll part goes on the down-going wing's spoilers;
  speed brakes on SEC 1 (both pairs) and SEC 3 (half, pair 2). Each order: ground spoilers when active, else the sum
  with the excess of the other side beyond -50 moved over, -50..0 deg, 50 deg/s; a pair not available follows its
  measured positions.

## Outputs

Bus (bit table in `readable/sec/output_packing.cpp`): spoiler / elevator / THS positions, sidesticks, speed brake lever
(30 deg per unit) and command, TLAs, discrete word 1 (servo failures, availability, active law, engagement, ground
spoilers out / armed), word 2 (sidestick priority, LGCIU fault, gear not uplocked). Discretes: elevators ok, ground
spoiler out, damping modes, THS active, battery supply (`thr_reverse_selected`, `sec_failed` constant false). Analog
orders: elevators, THS, four spoilers.

## Hidden state

`Runtime_MODE`; 2 LGCIU confirm nodes, both charts, `in_flight` and the touchdown time, 3 hydraulic confirm nodes,
priority pulse nodes / memories / confirm nodes, slew confirm, abnormal latch, landing pulse, THS ground latch; lever
retract confirm, inhibition latch, touchdown / lift-off / gear pulses, wheels-slow latch, ground spoiler and partial lift
dumping memories, 6 rate limiters; THS order, integrator, reload; battery confirm; the three laws (2 + 28 + 2 values):
120 values per SEC, 360 per step.

## Replay (`src/efcs_drive.h`, `sec-replay`)

The same drive as the ELAC's (SECs after the ELACs). Exact: as the ELAC, plus TLAs (recorded angles), the speed brake
lever (recorded handle position and armed flag), ELAC buses / discretes of the frame, the other SEC. Rebuilt from the
FACs: LGCIU main gears and "gear not uplocked" (FAC word 5 bits 18-20; the handle bit from the recorded gear handle).
Estimated: wheel speeds (not recorded: the ground speed, and FMGC 1's wheel speed above 70 kt discretes), spoiler /
elevator positions (bus float copies).

Result (30 recordings, 4.34 M frames): SEC frames with known inputs 10,622,247, of which 10,620,525 (99.98 %)
bit-exact on every bus word, discrete and analog order. The rest: discrete word 2 bit 18 (gear not uplocked) on 573
frames at gear transitions (the LGCIU word 3 bits are rebuilt from the FAC's single "gear down" bit), the ground
spoiler out discrete on 12 frames and the spoiler orders on 766 (pair 2) and 1,149 (pair 1) frames around touchdown (wheel speeds estimated).
The speed brake lever is read from the next frame's record (the recorded handle position is the L:var written at the
end of the previous frame). Approximate frames (2,044,727): 2,041,185 bit-exact.

## Verification (readable vs generated)

`sec-check-<variant>` (src/check_sec.cpp): the three SECs in lockstep, every leaf of `sec_outputs` (525 per SEC) and the
120 hidden memories per SEC bit-exact every step. Injectors: `--script` (src/sec_scenarios.h: touchdown, rejected
take-off, partial lift dumping, speed brakes and inhibition, roll spoilers, LGCIU words, SEC pitch and THS, ADR / IR
patterns and abnormal attitude), `--failures`, `--discretes`, `--power`, `--sim`.

- `UNIT=sec SEEDS=1 scripts/check-efcs-corpus.sh`: 271 runs, 39.2 M steps x 3 SECs, 610,938 SEC restarts, **0
  mismatches** (gcc and the clang coverage build). Mean step time: generated 0.34 us, readable 0.23 us.
- Coverage: `SecComputer.cpp` 95.8 % of branches (799/834), 98.5 % of lines. Of the 35 sides not taken, 33 are
  unreachable: library blocks (39, 40, 45), constant inputs (287, 293, 306, 317, 537, 667, 674, the constant
  FailureWarning word 1119), implied conditions (the ADR / IR chains 331-382, the sidestick memories 581 / 585, the
  duplicated gear terms 782 / 809, the flap index 1004, the THS reload at a restart 1084) and the spoiler order
  saturations to -50..0 that the computation cannot leave (927-975); 2 are reachable but not reached (671: SFCC 2 bit
  20 on a failed word in flight with the LGCIUs in fault; 1249: SEC 1 commanding the THS with green and yellow lost).
  Readable `sec/` 98.9 % of branches.
- Mutation test (`UNIT=sec scripts/mutate-efcs.sh`): **533 of 582 mutants caught** of `readable/sec/` (two rounds;
  the threshold phase killed 19 of the first 68 survivors). The 49 survivors: 26 equivalent (the alternate law's dead
  spoiler inputs (2), THS limit and spoiler order saturations that return or cannot reach the limit (11), SEC 3's
  constant FailureWarning word (1), initial values (2), the on-ground chart's `> 0.1` on booleans (4), the duplicated
  gear terms (4), SEC 3's pair 1 speed brake (limited to 0) and a zero roll order taking either branch (2)), 11
  comparisons at a threshold never met exactly, and 12 reachable but not killed (flap index 1 / 2 and 4 / 6 in the
  alternate law, the LGCIU bit-14 fault patterns (5), a simultaneous takeover press, SFCC 1 bit 21 in the direct law
  condition, the abnormal bank / pitch thresholds 125 / 50 moved to 187.5 / 75 on the SEC, the speed brake lever clamp
  below -0.5).

## FCOM checks (DSC-27-10-20 spoilers, DSC-27-20-20)

Matches: ground spoilers armed / extended with both main gears and idle or reverse; rejected take-off above 72 kt;
partial (phased) lift dumping with one main gear, and with both main gears and the thrust levers at or below CLB when
armed (bounce); law reconfiguration with the LGCIUs lost (flap condition). Differences / doubtful: S4, S5.
