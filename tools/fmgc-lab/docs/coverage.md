# Coverage of the generated FMGC (Phase 4)

What the differential checks actually exercise of the generated stock model (`build/models/stock`, upstream master
daee2f1f2): clang source-based coverage of `FmgcComputer.cpp` and `FmgcOuterLoops.cpp` only (`-fprofile-instr-generate
-fcoverage-mapping`, `-O2 -ffp-contract=off`, no other file instrumented). Line numbers are those of the stock files
(the same as `docs/structure.md`). The unreached branches below are where the readable rewrite is **not** verified by
the bit-exact checks: every one of them is either dead in the generated code, reachable only with physically impossible
data, or needs an input situation no recording or injector produced yet.

Reproduce: `JOBS=4 scripts/build-fuzz.sh`, then `JOBS=4 SEEDS=2 scripts/coverage.sh replay` (HTML in
`build/coverage/replay/html/`, the full uncovered list in `build/coverage/replay/uncovered.txt`), and for a fuzz corpus
`scripts/coverage.sh fuzz NAME build/fuzz/corpus` and `scripts/coverage.sh merge NAME+replay replay NAME`.

## Inputs

- **Replay + scenarios** (`coverage.sh replay`): the 22 recordings of `~/fdr-corpus/raw` (3.08 M steps), each run
  plain and with 9 scenario configurations x 2 seeds: `--fuzz` (input fuzzer), `--approach`, `--athr --approach`,
  `--lateral --approach`, `--engagement --approach`, `--targets --approach`, `--failures --approach` (sensor and
  equipment failures), `--tcas --situation` (TCAS RA episodes, held Mach cruise / ALT soft, V/S knob, engines stopped
  on the ground) and all of them at once. 418 runs, 58.4 M steps, 2 minutes with 4 processes.
- **Fuzzer** (`coverage.sh fuzz`): the libFuzzer corpus after the first campaign (see the end of this file).

## Numbers

Replay + scenarios (llvm-cov counts each branch condition's true and false outcome separately):

| File | Lines | Regions | Branches (outcomes) | Functions |
|---|---|---|---|---|
| FmgcComputer.cpp | 5047 / 5068 (99.59 %) | 3619 / 3640 (99.42 %) | 4196 / 4274 (98.18 %) | 20 / 21 (`terminate()` never called) |
| FmgcOuterLoops.cpp | 2626 / 2743 (95.73 %) | 871 / 962 (90.54 %) | 705 / 810 (87.04 %) | 33 / 33 |

Lines never executed: FmgcComputer.cpp 74–86 (bit extraction of a word ≥ 2^32 or a negative bit number), 1579–1580
(TCAS RA word ≥ 2^32), 5217 (`terminate()`); FmgcOuterLoops.cpp 1812–1846 (left/right states of the second
turn-direction chart instance) and the twelve "else" assignments of the constant switches in the speed-protection laws
(2989, 2998, 3046, 3055, 3370, 3379, 3427, 3435, 3873, 3882, 3933, 3941). Everything else that is not covered is one
side of a condition.

Branch conditions per subsystem, replay + scenarios (conditions / both outcomes seen / one outcome only / never
evaluated; from `build/coverage/replay/uncovered.txt`). Subsystems with every condition covered both ways are omitted:
init, ADR/IR validity, vote and selection, engines/FCU, ILS, TCAS conditioning, tune inhibit, LOC*/LOC, ALT arm,
G/S armed/G/S*/G/S, TCAS modes, engagement stage 2, the targets except the HDG/TRK preset, output assembly and packing.

| FmgcComputer.cpp subsystem | cond. | both | one | never |
|---|---|---|---|---|
| block library helpers | 29 | 19 | 7 | 3 |
| conditioning: RA | 10 | 9 | 1 | 0 |
| conditioning: FAC selection, rudder control | 16 | 15 | 1 | 0 |
| engagement (1380–1559) | 138 | 134 | 4 | 0 |
| vertical prelude: TCAS RA decoding | 22 | 21 | 1 | 0 |
| altitude target | 22 | 20 | 2 | 0 |
| SRS takeoff | 37 | 36 | 1 | 0 |
| RWY, RWY TRK | 43 | 42 | 1 | 0 |
| LAND armed, LAND, ROLL OUT | 101 | 97 | 4 | 0 |
| FINAL armed, FINAL | 60 | 59 | 1 | 0 |
| lateral mode reversion, HDG/TRK | 76 | 71 | 5 | 0 |
| NAV armed, NAV, LOC armed | 72 | 71 | 1 | 0 |
| ALT* | 42 | 40 | 2 | 0 |
| ALT, ALT CRZ | 37 | 35 | 1 | 1 |
| V/S reversion, V/S-FPA | 90 | 88 | 2 | 0 |
| CLB armed, CLB, DES armed | 149 | 137 | 9 | 3 |
| DES, OP CLB/DES, EXP CLB/DES | 207 | 203 | 3 | 1 |
| targets: HDG/TRK preset | 24 | 23 | 1 | 0 |
| law selection | 47 | 46 | 1 | 0 |
| LAND 2/3 capability | 61 | 59 | 2 | 0 |
| autothrust modes, FMA | 193 | 189 | 4 | 0 |
| autothrust N1 command | 35 | 27 | 8 | 0 |

| FmgcOuterLoops.cpp subsystem | cond. | both | one | never |
|---|---|---|---|---|
| block library helpers (filters, limiters, charts) | 76 | 52 | 10 | 14 |
| bank limit (and the TAS > 600 kt guard in the locals) | 14 | 11 | 3 | 0 |
| LOC_CPT, Phi_loc_c | 9 | 8 | 1 | 0 |
| roll law switch | 19 | 17 | 2 | 0 |
| ROLL_OUT | 16 | 11 | 5 | 0 |
| sideslip, nosewheel | 21 | 17 | 4 | 0 |
| AP roll | 10 | 6 | 4 | 0 |
| ALT_ACQ | 27 | 18 | 9 | 0 |
| ALT_HOLD | 10 | 9 | 1 | 0 |
| FLARE V/S law | 14 | 11 | 3 | 0 |
| G/S and path laws | 13 | 8 | 5 | 0 |
| FPA | 29 | 20 | 9 | 0 |
| SPD_MACH | 22 | 19 | 3 | 0 |
| V/S | 32 | 23 | 9 | 0 |
| SRS | 24 | 20 | 4 | 0 |
| VPATH | 6 | 5 | 1 | 0 |
| flare law / AP side | 7 | 6 | 1 | 0 |
| AP output | 18 | 15 | 3 | 0 |

The plain replays alone (no injector) reach far less: see "Contribution of the injectors" below.

## Unreached branches, by subsystem and kind

Kinds: **dead** (cannot happen in the generated code: constant operands, a condition implied by an earlier one in the
same expression, an `else if` that is the complement of the previous branch, the saturation of a signal that cannot
leave its range); **masked** (reachable in principle, but another term of the same expression always decides first
with any input the interface can produce); **data range** (needs a physically impossible value: TAS or ground speed
above ~1900 kt, an ARINC word ≥ 2^32); **failure only** (needs an equipment failure in a specific situation);
**needs a scenario** (a legal situation nobody has produced yet: these are the real gaps).

### FmgcComputer.cpp

| Subsystem | Line:col (outcome never seen) | Kind | What it would take |
|---|---|---|---|
| helpers: `MATLABFunction_i` (bit from label) | 68 (word < 2^32 never false), 78/79/84 (bit index ≤ 0) | data range / dead | a word ≥ 2^32; every call passes a constant bit 1–32 |
| helpers: lag / lead-lag filters | 117, 146 (`!pU_not_empty`) | dead | pY and pU are primed together |
| helpers: MTrigNode | 181, 182 | dead | every instance is rising-edge (the falling-edge variant is never instantiated) |
| conditioning: RA | 1088 (`raOppInvalid` never false) | dead | implied by the previous `else if` |
| conditioning: FAC rudder control | 1226 (`!Constant_Value_a`) | dead | constant-true enable (`docs/conditioning.md` finding 6) |
| engagement: AP latch reset | 1473 (`ap_instinctive_disc` never true) | masked | the instinctive disconnect always trips an earlier reset term (`rtb_y_m2` with AP engaged, AP inop) first |
| engagement: A/THR latch | 1529 (`raOwnInvalid` never true), 1531 (constant struct) | masked / dead | dual RA failure sets the RA to 2500 ft, so `RA > 100 ft` decides first (`docs/conditioning.md` finding 4); the struct field is a constant `false` |
| engagement: opposite priority | 1547 (`!ap_opp_engaged` never true) | masked (probably dead) | evaluated only with the own AP engaged and `rtb_y_ab`, which already requires the opposite side engaged |
| vertical prelude: TCAS RA word | 1573 (< 2^32 never false) | data range | RA word data ≥ 2^32 |
| altitude target | 1646 (`vertical_flight_plan_valid` never false), 1652 (`levelAtCstr` never true in the descending branch) | needs a scenario | levelled at a constraint (ALT/ALT* + constraint applicable + NAV) when the vertical flight plan becomes invalid; levelled at a constraint more than 250 ft above the aircraft with the FCU below |
| SRS takeoff | 1683 (`rtb_y_o4` never false) | dead | `!x \|\| (x && ...)`: the second operand runs only when x is true |
| RWY | 1750 (`Constant_Value_j`) | dead | constant 1 |
| LAND armed | 1844, 1845 (`!Constant2_Value_p`) | dead | constant 0 |
| LAND armed | 1846 (`!both_ils_valid` never true) | failure only | LAND arm attempted with one ILS failed and the two receivers disagreeing; the sim never fails an ILS (FW SSM), `docs/conditioning.md` finding 5 |
| LAND reset | 1896 (`pitch_goaround_prev` never true) | needs a scenario | TOGA (SRS GA) while LAND is engaged with none of the earlier reset terms true: go-around from LAND below 400 ft with the AP on |
| FINAL armed, NAV armed | 1941, 2200 (`raOwnInvalid` never true) | masked | dual RA failure reads 2500 ft, so the RA threshold decides first |
| lateral reversion (NAV term) | 2098–2102 (`!rwy/!GA TRK/!LOC*/!LOC/!LAND` never false) | needs a scenario | the NAV-loss reversion condition on a frame where NAV and another lateral mode overlap (handover frame) |
| ALT* (TCAS) | 2526 (submode == ALT_ACQ never false, `!rtb_y_n` never false) | needs a scenario | TCAS ending in its ALT_ACQ submode with the ALT* capture condition: the TCAS ALT_ACQ / ALT_HOLD submodes were never true in Phase 2 either |
| ALT (TCAS) | 2566 (submode == ALT_HOLD never true), 2567 (never evaluated) | needs a scenario | same, ALT_HOLD submode |
| V/S reversion | 2666, 2671 (`Constant_Value_m`) | dead | speed-protection reversions compiled in but disabled (constant 0) |
| CLB armed | 2742, 2743, 2869 (the ALT-push sub-terms) | dead | subsumed by the first term (`docs/modes/climb-descent.md`, "a subset ... no effect") |
| CLB armed / CLB / DES | 2749, 2750, 2755, 2819 (`rtb_y_bn`) | dead | reads a constant default struct (`any_lateral_mode_engaged = false`): always true |
| CLB armed reset | 2776 (levelled not at a constraint), 2778 (`!vertical_flight_plan_valid`) never true | masked / needs a scenario | earlier reset terms always fire first; would need CLB armed while levelled at a constraint and the vertical flight plan lost with every earlier term false |
| CLB engage | 2820 (`rtb_y_bq` never false) | needs a scenario | CLB engagement by the constraint-change path with the ALT-push variant (6 evaluations only) |
| DES engage | 2938:47/71/83 (ALT-push variant near a constraint) | needs a scenario | DES armed, ALT pushed within 250 ft of the constraint |
| OP CLB / CLB reversion | 2989 (`rtb_y_d` never true) | needs a scenario | CLB with that term (vertical flight plan / speed condition, see the CLB doc) as the only reason |
| HDG/TRK preset | 3422 (`rtb_y_d2_tmp` never false) | masked | below 30 ft RA the own RA is always valid in the corpus (a failed own RA low down) |
| law selection (TCAS) | 3774 (submode == ALT_ACQ never false) | needs a scenario | the TCAS ALT_ACQ submode |
| LAND capability | 3803 (`rtb_y_e0` never false), 3813 (`fwc_opp_valid` never false) | failure only | opposite FMGC LAND bit without a normal SSM; own FWC invalid with the opposite valid, in LAND |
| A/THR FMA | 4395 (`!raOwnInvalid` never false) | masked | dual RA failure (2500 ft) |
| A/THR FMA | 4403, 4415, 4421 | dead | each is the complement of the previous `else if` (MACH after SPEED, THR IDLE after THR LVR, TOGA LK after A.FLOOR) |
| A/THR N1 | 4464 (ground speed > 1000 m/s) | data range | |
| A/THR N1 | 4492, 4531, 4556 (`!pU_not_empty`) | dead | primed together |
| A/THR N1 | 4580 (`tracking_mode_on_override`) | needs a scenario | the replay forces it false (not recorded); the fuzzer's SimFlags event sets it |
| A/THR N1 | 4606, 4608 (THR ramp rate saturation) | needs a scenario (or data range) | ramp rate × table beyond ±10 %/s: extreme N1 or dt |
| A/THR N1 | 4618 (`icLoad` never true) | dead in practice | the integrator re-initialises only on the first running frame, when the A/THR cannot be active yet (the N1 mutation "FMGC restart with A/THR active" stayed uncaught for this reason) |

### FmgcOuterLoops.cpp

| Subsystem | Lines | Kind | What it would take |
|---|---|---|---|
| helpers: filters | 1551, 1735, 1885, 1914, 1978 (`!pU_not_empty`) | dead | primed together |
| helpers: turn-direction chart, second instance | 1792–1839 (all states except "any"), 22 outcomes | dead | this instance (LOC_TRACK / ROLL_OUT) is called with `use_short_path = 1` (`docs/laws/lateral.md`) |
| locals / bank limit | 2228 (\|TAS\| > 600 kt), 2247 (exact breakpoint) | data range / corner | TAS > 600 kt; TAS exactly on a 150 kt breakpoint |
| bank limit, AP roll | 2274, 2754 (`ManualSwitch_CurrentSetting == 1`) | dead | constant 0 |
| LOC_CPT, ROLL_OUT | 2356, 2544 (DME > 100 nm) | needs a scenario | localizer distance above 100 nm while computing LOC_CPT / ROLL_OUT |
| roll law switch, ROLL_OUT, sideslip, nosewheel, AP roll, G/S, AP output | 2497/2499, 2603/2605, 2653/2655, 2695/2697, 2745/2747, 3493/3495, 4553/4555 (saturation to [0, 1]) | dead | saturations of rate-limited 0/1 signals (mode or engagement fade-ins), which cannot leave [0, 1] |
| ROLL_OUT, G/S | 2574, 2611, 3518 (`!storage_not_empty`) | masked | the sample-and-hold is primed by the mode-change term of the same `\|\|` |
| AP roll, AP output | 2719, 4540 (`icLoad`) | masked | `!ap_engaged \|\| icLoad`: icLoad is only set on the first frame, when the AP is never engaged |
| ALT_ACQ, ALT_HOLD, FLARE, FPA, G/S, SPD_MACH, V/S, SRS, VPATH | 2801, 2952, 2965, 3009, 3022, 3094, 3157, 3166, 3324, 3333, 3346, 3390, 3403, 3532, 3558, 3572, 3641, 3793, 3834, 3844, 3893, 3903, 3959, 3970, 4030, 4046, 4080 (TAS or ground speed in m/s above 1000 or 2000) | data range | TAS / GS above 1944 kt |
| ALT_ACQ, FPA, V/S | 2987, 2996, 3044, 3053, 3368, 3377, 3425, 3433, 3871, 3880, 3931, 3939 (`Constant - Constant > 0`) | dead | constant switches in every speed-on-pitch protection (`docs/structure.md`: the ±4° limit is always selected) |
| G/S | 3525 (G/S angle < −4°) | needs a scenario | `fms_unrealistic_gs_angle_deg` below −4° (the injectors never set it) |
| VS limiter | 4311 (\|x\| < −2) | dead | absolute value below a negative bound |

### Summary

Of the 72 FmgcComputer.cpp outcomes never seen (llvm-cov reports 78: a few conditions are counted once per macro
expansion), 43 are dead or masked, 3 need impossible data, 3 need a failure in a specific situation, and **23 need a
scenario** (2776/2778 may turn out masked): the TCAS ALT_ACQ / ALT_HOLD submodes (2526, 2566,
2567, 3774), go-around from LAND (1896), losing the vertical flight plan or an unusual constraint geometry while
levelled at a constraint (1646, 1652, 2776, 2778), the ALT-push variants of CLB/DES engagement (2820, 2938), a NAV-loss
reversion on a lateral handover frame (2098–2102), one OP CLB reversion term (2989), the A/THR tracking-mode override
(4580) and the THR ramp rate limits (4606/4608). In FmgcOuterLoops.cpp only three outcomes need a scenario (DME > 100
nm twice, G/S angle < −4°) plus one exact-breakpoint corner (2247); the rest is dead (constant switches, [0, 1] saturations, the second chart instance) or needs TAS
above ~1900 kt. So the replay + injector runs reach every legal path of the generated code except those ~27 outcomes,
and those are where the readable rewrite is unverified. (Coverage is necessary, not sufficient: a covered branch can
still hide a rewrite difference that no checked input distinguishes; the mutation tests of each stage measure that.) They are the targets for new injector episodes (or fuzzer events).

## Contribution of the injectors

The 22 plain replays alone (`SEEDS=0 COV_NAME=replay-plain scripts/coverage.sh replay`) reach FmgcComputer.cpp
98.62 % of lines but only **3616 / 4274 branch outcomes (84.6 %)** and FmgcOuterLoops.cpp 674 / 810 (83.2 %): the
recordings fly the same few situations. Conditions with both outcomes seen, per FmgcComputer subsystem with a gap in
any column (plain replay → + the injectors → + the fuzz corpus, i.e. `replay-plain`, `replay`, `final+replay`):

| subsystem | conditions | plain | + injectors | + fuzz |
|---|---|---|---|---|
| block library helpers | 29 | 18 | 19 | 19 |
| conditioning: ADR validity and vote | 21 | 10 | 21 | 21 |
| conditioning: IR validity and vote | 33 | 12 | 33 | 33 |
| conditioning: ADR selection, indicated altitude | 6 | 5 | 6 | 6 |
| conditioning: RA | 10 | 4 | 9 | 9 |
| conditioning: FAC selection, rudder control | 16 | 8 | 15 | 15 |
| conditioning: engines, FCU | 4 | 2 | 4 | 4 |
| conditioning: ILS | 10 | 5 | 10 | 10 |
| conditioning: TCAS | 5 | 3 | 5 | 5 |
| engagement (1380–1559) | 138 | 65 | 134 | 134 |
| TCAS RA decoding, TRK-FPA deselection | 22 | 8 | 21 | 21 |
| altitude target | 22 | 16 | 20 | 22 |
| SRS takeoff | 37 | 33 | 36 | 36 |
| RWY, RWY TRK | 43 | 34 | 42 | 42 |
| LAND armed, LAND, ROLL OUT, ALIGN | 101 | 63 | 97 | 98 |
| FLARE | 5 | 4 | 5 | 5 |
| FINAL armed, FINAL | 60 | 49 | 59 | 59 |
| SRS go-around, GA TRK | 51 | 46 | 51 | 51 |
| lateral mode reversion, HDG/TRK | 76 | 55 | 71 | 76 |
| NAV armed, NAV, LOC armed | 72 | 48 | 71 | 71 |
| LOC*, LOC | 72 | 66 | 72 | 72 |
| ALT arm possible, ALT armed | 31 | 28 | 31 | 31 |
| ALT* | 42 | 34 | 40 | 41 |
| ALT, ALT CRZ | 37 | 31 | 35 | 35 |
| V/S reversion, V/S-FPA | 90 | 67 | 88 | 88 |
| CLB armed, CLB, DES armed | 149 | 125 | 137 | 139 |
| DES, OP CLB/DES, EXP CLB/DES | 207 | 181 | 203 | 206 |
| G/S armed, G/S*, G/S | 77 | 67 | 77 | 77 |
| TCAS armed, TCAS | 46 | 36 | 46 | 46 |
| engagement stage 2 | 39 | 29 | 39 | 39 |
| targets: HDG/TRK preset | 24 | 19 | 23 | 23 |
| targets: selected/managed speed, SPD/MACH | 77 | 56 | 77 | 77 |
| targets: speed memos, SRS targets, DES margin | 18 | 13 | 18 | 18 |
| targets: ALT soft | 12 | 10 | 12 | 12 |
| law selection | 47 | 40 | 46 | 46 |
| LAND 2/3 capability | 61 | 34 | 59 | 60 |
| ap_fd_logic output assembly | 26 | 25 | 26 | 26 |
| autothrust: modes, alpha floor, FMA | 193 | 118 | 189 | 189 |
| autothrust: N1 command | 35 | 23 | 27 | 29 |
| output packing: value-word SSMs | 20 | 17 | 20 | 20 |
| output packing: discrete words | 26 | 23 | 26 | 26 |

The injectors written for the stage checks (failures, engagement, A/THR, approach, lateral, targets, TCAS) close
almost all of the conditioning, engagement and A/THR gaps of the plain replays; the fuzzer adds the combinations
(constraint geometry, mode handover frames, absurd air data) on top.

## After the first fuzz campaign

Campaign "first" (`WORKERS=4 TIME=3600 scripts/fuzz.sh`, libFuzzer fork mode, before the whole-computer oracle was
wired): 1.50 M inputs in 3600 s (~420 inputs/s for 4 workers, ~1000 model steps per input, so ~400 k generated steps/s
with the property and stage oracles), corpus 743 seeds → 6918 inputs, libFuzzer `cov` (model edges + state counters)
3665 → 4100, no differential mismatch (output packing, A/THR resync). Its corpus through the coverage build
(`coverage.sh fuzz first`, 6918 inputs, 10 s) and the union with the replay (`coverage.sh merge first+replay`):

| | FmgcComputer.cpp branches | FmgcOuterLoops.cpp branches | FmgcComputer lines | FmgcOuterLoops lines |
|---|---|---|---|---|
| replay + scenarios | 4196 / 4274 (98.18 %) | 705 / 810 (87.04 %) | 99.59 % | 95.73 % |
| fuzz corpus alone | 4213 / 4274 (98.57 %) | 737 / 810 (90.99 %) | 98.05 % | 95.52 % |
| union | 4213 / 4274 (98.57 %) | 739 / 810 (91.23 %) | 99.64 % | 96.86 % |

The fuzz corpus alone covers every FmgcComputer outcome the 418 replay runs cover (the replay adds only line coverage
of the long-recording paths) plus 17 more. Newly covered, by the table above:

- **needs a scenario → covered**: 1646 and 1652 (vertical flight plan lost / constraint above while levelled at a
  constraint), 1896 (TOGA while LAND engaged), 2098–2102 (NAV-loss reversion on a lateral handover frame), 2526:7
  (TCAS ending in its ALT_ACQ submode → ALT*: the first time the TCAS ALT_ACQ path to ALT* has run), 2778 (CLB armed
  reset on vertical flight plan loss), 2938:71 (DES ALT-push variant), 2989 (OP CLB reversion term), 4580 (A/THR
  tracking-mode override, from the SimFlags event), and in the outer loops 2356 / 2544 (DME > 100 nm) and 3525 (G/S
  angle < −4°);
- **reclassified**: 2742 (CLB-armed ALT-push sub-term) is reachable, only redundant (no effect on the result), and
  2574, 2611, 3518 (sample-and-hold priming) are reachable, not masked;
- **data range → covered**: 4464 and the TAS/GS saturations of every outer-loop law (2228, 2801, 2952, ... 4080),
  from the any-field mutation events (absurd airspeeds); 3813 (own FWC invalid in LAND) from the discrete flips.

Still never seen in the union (FmgcComputer: 54 outcomes; FmgcOuterLoops: 57, all dead except the 2247 breakpoint
corner): **needs a scenario** 2526:110, 2566, 2567, 3774 (the TCAS **ALT_HOLD** submode and the ALT_ACQ submode
with the RA still in a non-VS state: still never reached, also not by the second campaign's TCAS seeds so far),
2776, 2820, 2938:47 (CLB/DES constraint paths), 4606/4608 (THR ramp rate limits); **failure only** 1846, 3803;
**data range** 68, 1573; the rest is dead or masked as listed above. So after the replay, the injectors and one hour
of fuzzing, the generated code paths not exercised by any bit-exact comparison are those 10 scenario outcomes, 2
failure combinations and the dead code.

### After the whole-computer campaigns

Campaigns "whole" (~22 min effective, 3 workers) and "whole2" (40 min, 4 workers) ran with the whole-computer
differential (`docs/properties.md`, Campaigns). The final corpus (8060 inputs, `coverage.sh fuzz final`, union
`final+replay`): FmgcComputer.cpp 4214 / 4274 branch outcomes (98.60 %; one more than after "first": 2938:83, the
DES ALT-push comparison), FmgcOuterLoops.cpp 739 / 810 (91.23 %, unchanged), lines 99.64 % / 96.86 %. libFuzzer's
`cov` stayed at 4098 through both campaigns (features still grew, 37 976 → 39 653): the model's edges are saturated
for this input encoding, and what is left is the list above. The integration's open targets over all 2370 saved fuzz
inputs (`fmgc-fuzz-run run --summary`, `cov` lines): TCAS submode ALT_ACQ reached (112 steps; never in the corpus
runs), **ALT_HOLD still never**; A/THR FMA message 1 (LVR TOGA) never, FMA modes 2 and 4 (MAN GA SOFT, MAN DTO)
never (they are never produced, `docs/athr-modes.md`). Reaching TCAS ALT_HOLD needs an RA that ends in ALT_ACQ and
then holds within 40 ft of the target for 0.8 s with the RA word still in the right state: a dedicated injector
episode (RA, FCU altitude at the current altitude, the RA word held after the capture) is the next step.
