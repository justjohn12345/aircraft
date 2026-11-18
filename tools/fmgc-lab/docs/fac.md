# The A32NX FAC (generated `FacComputer`) and its readable rewrite

The Flight Augmentation Computer model has two halves. The yaw functions command the rudder actuators: yaw damper,
rudder trim and rudder travel limiter, each with its engagement logic (FAC 1 has priority). The flight envelope function
computes what the PFD speed scale and the FMGCs use: V alpha max / prot, VSW, VLS, VS1g, F, S, green dot, VMAX, VFE
next, the speed trend, alpha floor, the low energy ("SPEED SPEED SPEED") and pitch-pitch warnings, the estimated
sideslip and the engine-out sideslip target. It is 1,587 generated lines: 13 lag filters, 2 washouts, 10 rate limiters,
8 confirm nodes, 2 monostables, one Stateflow chart (alpha floor flight phase), two latches and look-up tables.

- Generated: `fbw-a32nx/src/wasm/fbw_a320/src/model/FacComputer.cpp` (+ `.h`, `_data.cpp`, `_types.h`, and the
  `look1_binlxpw` / `look2_binlxpw` / `look2_pbinlxpw` helpers). Never edited. Identical in every lab variant (last
  changed by #10509, May 2026).
- Readable: `tools/fmgc-lab/readable/fac/` (namespace `fac::readable`), bit-exact (below).
- Wrapper: `fbw_a320/src/fac/Fac.cpp`; sim wiring `FlyByWireInterface::updateFac()` (~2418).
- Parameters (block name = value): `docs/params-FacComputer.txt`.
- Findings: `docs/findings.md`, section 9 (A1-A17).

## Where it runs

Every frame, after the sensors (RA, LGCIU, SFCC, ILS, ADIRS), the FCU, FMGC 1, FMGC 2, the ELACs and the SECs,
`updateFac(i)` runs for FAC 1 then FAC 2; the FCDCs and FADECs follow, then the flight data recorder. So FAC 1 sees FAC
2's bus of the previous frame, FAC 2 sees FAC 1's of this frame, and both see this frame's FMGC B buses and ELAC buses.
The FACs (like the ELACs and SECs) do not run in active pause; the FMGCs do.

`Fac::update()` monitors the power supply (DC ESS SHED for FAC 1, DC 2 for FAC 2): an outage over 10 ms stops the
model (not stepped, outputs held), over 200 ms makes the FAC unhealthy; on the ground with the nose gear compressed
and the engines "stopped" (hard-wired true) a 20 s self test runs after an outage over 4 s. `facHealthy` is an S-R
flip-flop: set by software healthy and (a FAC pushbutton press while unhealthy, or nose gear compressed), reset by a
long power failure, a FAC failure or the self test. `computer_running` = `facHealthy`. When unhealthy, the bus is all
FailureWarning (SSM 0, data 0), the engaged discretes and analog orders are false / 0, and `fac_healthy` shows the self
test light pattern. In flight a FAC that lost power stays unhealthy until its pushbutton is cycled.

When `computer_running` is false the model does nothing (outputs and memories held). The first running frame
afterwards (or the very first step) **restarts** it: every filter, rate limiter, confirm node, monostable, latch and
the flight phase chart go back to their initial state (generated 380-424); only the rudder trim integrator keeps its
value (it is reloaded instead, see A4).

## Structure (generated line ranges -> readable files)

| Generated | What | Readable |
|---|---|---|
| 19-275 | library blocks: SSM tests, BitfromLabel, lag / washout filters, rate limiters, confirm node, MTrigNode, VS formula, bit packing | `fac_blocks.h` (+ `readable/blocks.h`) |
| 380-424 | restart on the first running frame | `fac.cpp` `restart()` |
| 426-466 | landing gear: own LGCIU, the opposite FAC's copy | `sensors.cpp` `updateGear` |
| 467-557 | slats / flaps: own SFCC, the opposite FAC's copy, gear-down default; FPPU -> surface angle | `sensors.cpp` `updateSlatFlap` |
| 558-638 | ADR and IR selection (own, 3, none), failure flags | `sensors.cpp` |
| 640-665 | RA (own / opposite FMGC), FMGC selection, on ground | `sensors.cpp` |
| 666-685 | yaw damper / rudder trim / travel limiter engagement; speed scale visible (10 s after lift-off) | `yaw_functions.cpp`, `characteristic_speeds.cpp` |
| 686-721 | estimated sideslip, sideslip target | `flight_path.cpp` `updateSideslip` |
| 722-831 | filtered alpha, alpha floor threshold, speed rate, flight phase chart, alpha floor latch | `alpha_floor.cpp` |
| 832-985 | V alpha max / prot, VSW, VS, VLS (buffet, take-off configuration, speed brakes), F, S, green dot, VMAX, VFE next, speed trend | `characteristic_speeds.cpp` (+ `vls_buffet_table.h`) |
| 986-1094 | pitch-pitch, gamma_a, gamma_t, inertial FPA, low energy | `flight_path.cpp` `updateWarnings` |
| 1095-1169 | rudder trim | `yaw_functions.cpp` `RudderTrim` |
| 1170-1219 | yaw damper, travel limiter | `yaw_functions.cpp` |
| 1220-1563 | FAC bus, discrete and analog outputs, logic / flight envelope / laws copies | `output_packing.cpp` |
| 1568-1574 | `initialize()` | `fac.cpp` |

`readable/fac/fac.h`: `fac::readable::FacComputer` with `initialize()` and `const fac_outputs& step(const fac_inputs&)`,
no heap, no static state, copyable (state accessors for the checker). Its `initialize()` is a complete reset (checked
with `fac-check --warm`); the generated one is not (`Runtime_MODE`, filter flags), so the checker re-constructs the
generated instance. The generated `m_bpIndex` (the buffet table's search start) is not reproduced: `look2_pbinlxpw`
returns the same value whatever the start index (the interval holding the input is unique; NaN takes the last one).

## Inputs (`fac_inputs`)

| Field | Sim source (`updateFac`) | Used for |
|---|---|---|
| `time.dt` | frame sample time | every filter, rate limiter, confirm node |
| `time.simulation_time`, `monotonic_time` | | unused |
| `sim_data.computer_running` | `facHealthy` (wrapper) | run / hold / restart |
| `sim_data.slew_on / pause_on / tracking_mode_on_override` | | `logic.tracking_mode_on` only |
| `ap_own_engaged`, `ap_opp_engaged` | FMGC own / opposite AP discrete | FMGC selection, rudder trim, pitch-pitch |
| `yaw_damper / rudder_trim / rudder_travel_lim_opp_engaged` | opposite FAC discretes | FAC 2 priority; discrete word 2 |
| `elac_1/2_healthy` | ELAC `digital_output_validated` | yaw damper, auto trim, low energy |
| `engine_1/2_stopped` | true (hard-wired) | unused by the model (wrapper self test) |
| `rudder_trim_switch_left/right`, `rudder_trim_reset_button` | RUDDER_TRIM_LEFT/RIGHT/RESET events (one frame each) | rudder trim |
| `fac_engaged_from_switch` | FAC pushbutton | engagement of the three functions |
| `fac_opp_healthy` | opposite FAC healthy | unused |
| `is_unit_1` | FAC index | priority, FMGC selection |
| `rudder_trim/travel_lim_actuator_healthy` | true (hard-wired) | engagement |
| `slats_extended` | !(own SFCC actual position word bit 12, when normal) | SFCC monitor (finding A15) |
| `nose_gear_pressed` | LGCIU nose gear compressed | LGCIU monitor; wrapper self test and reset |
| `ir_3_switch`, `adr_3_switch` | false (hard-wired) | unused |
| `yaw_damper_has_hyd_press` | green (FAC 1) / yellow (FAC 2) pressurised | yaw damper engagement |
| `yaw_damper_position_deg` | 0 (hard-wired) | yaw damper order while not engaged |
| `rudder_trim_position_deg` | -rudder trim feedback angle | trim order while not engaged / after reset; bus copy |
| `rudder_travel_lim_position_deg` | travel limiter position | limit order while not engaged; bus copy |
| `fac_opp_bus` | opposite FAC bus | LGCIU / SFCC fallback (discrete word 5, angles, word 1) |
| `adr_own/opp/3_bus`, `ir_own/opp/3_bus` | ADIRS (FAC 1 own = 1, FAC 2 own = 2) | air data / inertial data; opposite: failure flags only |
| `fmgc_own/opp_bus` | FMGC B buses | RA, FM weight / CG, FAC weight, N1 words, discrete word 1, ATS word |
| `sfcc_own_bus`, `lgciu_own_bus` | SFCC / LGCIU of the same side | slats / flaps, gear |
| `elac_1/2_bus` | ELAC buses | yaw damper orders, normal law / high AoA bits, speed brakes |

## Functions

### Input conditioning (`sensors.cpp`)

- **Landing gear**: own LGCIU valid while its discrete word 1 is not FailureWarning and its nose gear bit (word 2 bit
  12) has not disagreed with the nose gear discrete for 1 s. Valid: main gear compressed = word 2 bits 13 / 14, gear
  down = word 3 bit 11 or 12. Else the opposite FAC's discrete word 5 (bit 16 = its LGCIU valid; 18, 19, 20), else
  nothing pressed. **On ground** = either main gear compressed for 0.5 s (in flight as soon as neither is).
- **Slats / flaps**: own SFCC valid while its system status word is not FailureWarning and the slat-extended discrete
  has not agreed with the actual position word bit 12 for 1 s. Configuration index (0 CONF 0, 1 CONF 1, 2 CONF 1+F,
  3 CONF 2, 4 CONF 3, 5 FULL) from the status word bits 17 (0), 18 + 26 (1), 18 alone (1+F), 19, 20, 21; angles = the
  FPPU words. Fallbacks: the opposite FAC's copy (word 5 bits 11-15, angles, word 1), else with the gear down CONF FULL
  at full angles, else CONF 0. FPPU -> surface angle tables for flaps (0-40 deg) and slats (0-27 deg).
- **ADR**: own unless its CAS or AoA word is FailureWarning, else ADR 3, else zeros. NCD and FT data are used. The
  opposite ADR is never used (only in `double_self_detected_adr_failure`).
- **IR**: own unless its yaw rate or lateral acceleration word is not NormalOperation, else IR 3, else zeros.
- **Radio height**: the own FMGC's `fg_radio_height_ft` unless failed for 0.6 s, else the opposite FMGC's; all RA
  failed when both are.
- **FMGC selection** (discrete word 1, ATS word): own when its AP is engaged (FAC 2 yields if both APs are), or with no
  AP when its FM weight and CG words are normal; else the opposite FMGC.

### Yaw functions (`yaw_functions.cpp`)

Each function can engage with the FAC pushbutton on and its actuator available (yaw damper: hydraulic pressure;
the trim and limiter actuators are hard-wired healthy), and engages when FAC 1 is this unit or FAC 1's function is not
engaged. FAC 2's functions are synchronised followers (orders = measured positions) while FAC 1 holds them.

- **Yaw damper**: ELAC 1's (else ELAC 2's) yaw damper order when that ELAC is healthy with discrete status word 1 bits
  20 and 26; else the FAC's alternate law: yaw rate through a 20 rad/s washout, gain 1, +-5 deg. Then +-20 deg, 40
  deg/s; not engaged: the measured position (0).
- **Rudder trim**: an integrator (+-20 deg) of the trim switches (1 deg/s, left positive), or with an AP engaged of
  1/15 of the ELAC yaw damper order (at most 1.7 deg/s) above 200 ft (own FMGC radio height value). The RESET
  pushbutton (no AP) latches a reset until a switch or an AP: the integrator held at 0. The order follows the
  integrator at 2 deg/s; not engaged: the measured position (the integrator is reloaded with it).
- **Rudder travel limiter**: the limit for the CAS (25 deg up to 160 kt, 14.5 at 200, 8.8 at 240, 4.8 at 320, 3.4 from
  380 kt), 1 deg/s; not engaged: the measured position.

### Flight envelope (`alpha_floor.cpp`, `characteristic_speeds.cpp`, `flight_path.cpp`)

- **Alpha floor**: filtered alpha (lag 0.5 rad/s) above the threshold for Mach and configuration (rate limited 1
  deg/s; CONF 0: 7.6 deg at M 0.5, 6.03 at M 0.9; CONF 1-FULL 11.8-13.1 deg) lowered by half the deceleration (kt/s,
  lag 0.4 rad/s) at most 3 deg in CONF 3 and FULL, Mach < 0.6, an ELAC in normal law (word 1 bit 26), and the flight
  phase chart not inhibiting it. The chart: Landed (inhibit) -> Takeoff100ft when airborne -> Flying above 100 ft
  (own FMGC RA; 200 ft when failed) -> Landing100ft below 100 ft (inhibit) -> Landed on the ground. The latch is
  reset when inhibited, no ELAC high AoA protection (word 2 bit 23) or no normal law. Discrete word 5 bit 29.
- **Alpha speeds**: V alpha max / prot / VSW = CAS x sqrt(|alpha_f - alpha0| / (limit - alpha0)), alpha0 per
  configuration, limits for Mach and configuration (alpha max 8.7 / 13.6 / 14.2 / 13.1 / 13.0; alpha prot = alpha stall
  warning 6.5 / 11.7 / 11.9 / 11.0 / 10.6), each limit rate limited 1 deg/s.
- **VS1g** = a + b (W - 40 t) per configuration (CONF 0: 125.64 + 1.2 kt/t ... FULL: 84.47 + 0.81), W the FM weight
  (else the FAC weight word the FMGC relays), rate limited 5 kt/s from 0.
- **VLS** = max(buffet CAS from 20000 ft, VMCL 116 kt, k x VS) + 0..18 kt for the speed brakes (ELAC 1, else 2,
  extension -40..0 deg). k = 1.28 in CONF 0, 1.13 while the configuration is the one held on the ground (both main
  gears compressed, an LGCIU), 1.23 after any change, rate limited 0.1/s from 0. Buffet: Mach from a 45 x 11 table of
  a weight/CG parameter and the flight level (210-410), converted to CAS at the static pressure.
- **F** = max(1.26 VS(CONF 1+F), VMCL + 15) shown in CONF 2 and 3; **S** = max(1.23 VS(CONF 0), VMCL + 20) shown in
  CONF 1 and 1+F; **green dot** = 2 W(t) + 85 kt + 1 kt / 1000 ft above 20000 ft, shown in CONF 0.
- **VMAX** = min(VMO 350 kt or VLE 280 kt with the gear down, CAS of MMO 0.82, VFE 350 / 230 / 215 / 200 / 185 / 177);
  **VFE next** = 230 / 200 / 200 / 185 / 177 / 177, shown below 15000 ft and before FULL.
- **Speed trend** = 10 x washout(lag(CAS 30..660 kt)), both 1 rad/s.
- Alpha speeds and VLS are NormalOperation only from 10 s after lift-off (NCD on the ground); VS, VMAX, trend always.
- **Estimated sideslip** = ny g / (q S CYbeta / m) with q from CAS, S 122 m2, m 70 t, CYbeta -0.62 (0 below 30 kt),
  lag 4 rad/s. **Sideslip target**: (alpha x N1 diff x 0.0302 + 0.1101 x N1 diff) / CAS^2, from the own FMGC's N1
  words; sent NCD.
- **gamma_a** = lag(theta - alpha cos(phi)) (theta for alpha on the ground); **gamma_t** = lag(57.3 nx - nz alpha),
  both 1 rad/s; inertial FPA = lag(57.3 x V/S / GS) 0.5 rad/s (V/S word read as ft/min).
- **Low energy**: confirmed 0.5 s, held at least 3 s, cleared with all RA failed: CAS < VLS - 10, or CAS < VLS with
  alpha_f above 9.5-10.7 deg (flaps) + FPA term (-3..+3 deg) + deceleration term (gamma_t - gamma_a, -5..0 deg); in
  flight, CONF 2 or more, 100 < RA < 2000 ft, no alpha floor (ATS bit 23), not SRS TO (word 1 bit 15) or within 8 s of
  SRS GA (bit 16), an ELAC in normal law with bit 20. Discrete word 3 bit 11.
- **Pitch-pitch**: theta + theta rate > 10 deg, RA <= 20 ft (not NCD for 1 s, not all failed), no AP, not the TOGA
  go-around lever bit (word 1 bit 29). Discrete word 3 bit 15.

## Outputs

FAC bus (SSM: NO NormalOperation, NCD NoComputedData; bits 1-based); the full table is in
`readable/fac/output_packing.h`:

| Word | SSM | Data |
|---|---|---|
| discrete word 1, slat / flap angle | NO with the own SFCC valid, else NCD | the selected SFCC actual position word value, FPPU angles |
| gamma_a, gamma_t | NO | path angles |
| total weight, CG | NO | constant 0 (finding A13) |
| sideslip target | NCD always | beta target |
| discrete word 2 | NO | 11-16: yaw damper, rudder trim, travel limiter engaged (own, opposite) |
| rudder travel limit command | NO | the measured limiter position |
| yaw damper | NO | constant 0 |
| estimated sideslip | NO | |
| V alpha lim, VLS, V alpha prot, VSW | NO from 10 s after lift-off, else NCD | |
| VS, speed trend, VMAX | NO | |
| F, S, green dot, VFE next | NO when shown, else NCD | |
| discrete word 3 | NO | 11 low energy, 15 pitch-pitch |
| discrete word 4 | NO | 0 |
| discrete word 5 | NO | 11-15 CONF 1..FULL, 16 LGCIU own valid, 17 all LGCIU lost, 18/19 main gear, 20 gear down, 21 SFCC own valid, 22 all SFCC lost, 29 alpha floor |
| rudder trim order, position | NO | trim order (measured position while not engaged), measured position |

Discretes: `fac_healthy` true (the wrapper decides), the three engaged flags, `yaw_damper_avail_for_norm_law` = yaw
damper can engage, `rudder_travel_lim_emergency_reset` false. Analog orders: the law orders while engaged, else 0.
Consumers: PFD (speeds, trend, sideslip), FMGCs (VLS, VMAX, F, S, green dot, alpha floor, gear / SFCC bits), FWC
(discrete word 3, health, trim position), SD F/CTL page (travel limit word), RTPI (discrete word 2, trim position).

## Hidden state (what the checker compares besides the outputs)

Per FAC: `Runtime_MODE`; the 8 confirm nodes (time, output); the alpha filter, the alpha floor threshold rate limiter,
the speed rate filter and the previous half CAS (`Delay_DSTATE`), the chart (active, state), `sAlphaFloor`; the alpha0 /
max / prot / stall warning rate limiters, VS and VLS factor rate limiters, `takeoff_config`, the speed trend lag and
washout; the 9 flight path lag filters, the two monostables; the trim reset latch, the reset button edge detector,
the trim integrator, `icLoad`, the trim order rate limiter, the yaw damper washout and rate limiter, the travel limiter
rate limiter: 107 values per FAC (every filter's pY, pU and both "not empty" flags).

## Replay: inputs rebuilt from the record (`src/fac_drive.h`, `fac-replay`)

No recording format holds the FAC model inputs: the stock 3200006 record and the personal 3209001-3209004 ones write
the FAC bus, discretes and analog orders of each FAC (`FacRecord`, the wrapper's outputs) and nothing of its inputs. The
drive rebuilds them, frame by frame, from what is recorded:

| Input | Source | Exact? |
|---|---|---|
| dt | base record (`calculatedSampleTime`, the FAC's dt) | yes |
| computer_running | the recorded bus: FailureWarning on discrete word 2 = the wrapper's unhealthy output | yes |
| ADR / IR own, opp, 3 | FMGC 1's recorded inputs of the frame (the same `adrBusOutputs` / `irBusOutputs`) | yes while FMGC 1 runs |
| ELAC buses, healthy | ELAC records (bus, `digital_output_validated`) | yes |
| opposite FAC bus / discretes | FAC 1: FAC 2's record of the previous frame (zeros on frame 0); FAC 2: FAC 1's of this frame | yes |
| FMGC 1 B bus, AP | FMGC 1's record when it ran (the record changed), else zeros / false (the wrapper while not healthy) | yes |
| FMGC 2 B bus, AP | the next frame's FMGC 1 inputs: health, AP discrete, A bus (discrete words 1-4 and ATS: the same packing, checked on FMGC 1: identical on every frame), RA 2 / RA 1 through the FMGC's RA selection, weight / CG / N1 / FAC weight words from FMGC 1's B bus | yes, except across an active pause gap or at the end of the file (held) |
| SFCC own bus | from what this FAC reported: own SFCC valid (word 5 bit 21) -> status word with the bits of the reported configuration, the angle words and the actual position word it copied; else FailureWarning | yes (the FAC only reads those) |
| LGCIU own bus, nose gear | valid (word 5 bit 16) -> the reported main gear / gear down bits, nose bit = nose discrete; else FailureWarning | yes |
| FAC pushbutton, hydraulics | `rudder_trim_engaged` (FAC 1: = pushbutton), `yaw_damper_avail_for_norm_law` (= pushbutton and pressure); FAC 2 when FAC 1 does not hold the function; otherwise they cannot change an output | yes |
| trim / travel limiter positions | the copies on the FAC bus (float of the sim's double) | float precision (used only after a restart or while not engaged) |
| trim switches, reset button | searched per frame: the generated FAC is stepped with none, the previous frame's, left, right, reset, the rest until the trim order and bus word match | yes when found (never needed in the corpus: the trim was never used) |

FMGC 1 does not run during its 30 s start-up self test; in cold-start recordings the FACs run about 10-60 s (345 s in
one file) before it does, and its record holds nothing then. Those frames get stand-ins: the first running frame's
ADIRS / RA / FMS words, per FAC switched to the frozen (failed) record while the FAC reports VMAX 0 (no static pressure:
ADRs off) or flight path angles of exactly 0 (IRs off). They are compared separately ("approximate"), with the 60 s
after them while filters converge; a record frozen to the end of the file is treated the same way (FMGC 1 switched off
first, sensor buses stale).

`fac-replay` sets FTZ/DAZ (the sim flushes denormals) and compares every field bit-exact, reporting the largest ULP and
absolute difference per field (`--ieee` keeps denormals, `--no-trim-search` disables the search, `--trace T0 T1`).

Result (30 recordings, 4.34 M frames, both FACs: 8.27 M FAC frames with known inputs, 0.20 M approximate): on every
frame with known inputs every bus word, discrete and analog order is bit-exact except

- `speed_trend_kn` on 2.48 M FAC frames, at most 2.4e-7 kt, and
- `yaw_damper_order_deg` on 19,406 FAC frames (the alternate law, before the ELACs provide an order), at most 2.2e-16 deg,

the two outputs of washout filters, `y = A pY + (u ca - pU ca)`, whose difference of two nearly equal products shows a
different rounding of either product. The sim fuses some multiply-adds: frame 0 of 2026-11-14-13-57-13 (FAC 1's yaw
damper order, a single washout step from the input) is exactly `A u + fma(u, ca, -(pU ca))`, and no plain evaluation or
dt gives it; native clang `-ffp-contract=on -mfma` reproduces that frame but not every later one, so the sim's fusion
pattern (in fbw.wasm's build or MSFS's compilation of it) is not clang's (finding A1). The rudder trim inputs were
never needed (the pilot never trimmed the rudder in the corpus). Approximate frames: mismatches in gamma_a / gamma_t /
VMAX where the stand-in sensor values differ.

## Verification (readable vs generated)

- `fac-check-<variant>` (src/check_fac.cpp): both FACs, generated and readable in lockstep, free-running, every leaf of
  `fac_outputs` (633 per FAC, the inputs copy included) and the 107 hidden memories per FAC bit-exact every step, plus
  the power-up outputs and the latch table against the readable latch. Floats by bit pattern, except that two NaNs
  match whatever their sign and payload: x86 takes a NaN result from whichever operand the compiler put first in a
  commutative operation (the generated and readable lag filters differ there with NaN inputs), and no FAC output depends
  on a NaN's sign (bit decoding, comparisons, fmin/fmax, abs ignore it).
- Inputs: the replay drive above, on any recording; injectors (src/fac_scenarios.h): `--script` (a tour of every
  function, fallback and threshold), `--failures` (any ARINC word the FAC reads: random SSM, NaN / inf / 0 / huge /
  scaled data, random discrete bits), `--discretes` (any discrete input incl. unit 1, analog positions incl. NaN, bits of
  the decoded discrete words), `--power` (restarts: 1 frame, 2-10 frames, 0.2-5 s, 5-60 s, flicker, either or both
  FACs), `--sim` (dt 0, 1e-6, 0.2-2 s; time jumps; slew, pause, tracking).
- `scripts/check-fac-corpus.sh` (JOBS=2, SEEDS=2): 511 runs over the 30 recordings, 73.9 M steps x 2 FACs, 31,204
  restarts, **0 mismatches** (gcc build and the clang coverage build). Mean step time: generated 0.39 us, readable
  0.30 us (gcc -O2).
- Coverage (`COVERAGE=1`, clang source-based, the same 511 runs; also 0 mismatches): `FacComputer.cpp` 94.4 % of
  branches (553/586), 97.8 % of lines; every branch side not taken is unreachable:
  - library blocks with constant parameters (22): BitfromLabel's left-shift and range tests (constant bit numbers 11-29,
    4 sides); the lag / washout filters' second "not empty" flag (always set with the first, 3); the reset input of the
    rate limiters without an initial value (constant false, 4); MTrigNode and PulseNode falling-edge branches (all
    instances detect rising edges, 11);
  - conditions implied by an earlier test (5): CONF 1+F's "bit 18 and not bit 26" after "bit 18 and bit 26"; "AP own not
    engaged" in the FMGC selection's second factor (reached only with the own AP engaged); "not unit 1" in the three
    priorities (reached only when not unit 1);
  - saturations and switches with constant inputs (6): the travel limit 0..30 deg (the table spans 3.4..25), the speed
    scale lost constant (2), the sideslip / sideslip target SSM switch thresholds (2).
  Readable FAC: 100 % of branches except the one "AP own not engaged" side in `sensors.cpp` (kept to mirror the
  generated condition); the functions not covered are the checker's state accessors.
- Mutation test (`scripts/mutate-fac.sh`, `scripts/mutate-fcu.py` with `UNIT=fac`): 412 mutants of `readable/fac/`
  (&& <-> ||, ! dropped, == <-> !=, < <-> <=, > <-> >=, true <-> false, += -> -=, every decimal literal x1.5 (0 -> 1)
  except in table rows, every integer constant + 1, i.e. every ARINC bit number; `fac_blocks.h` included, the buffet
  table header not): **390 caught**. The first run left 41; a mutant that crashes (an out-of-bounds table row) or hangs
  (an endless search loop) now counts as caught, and a scripted phase holding the inputs exactly at the thresholds
  (radio height 20 / 100 / 200 ft, 20000 ft, alpha on the alpha floor threshold, pitch + pitch rate 10 deg, CAS on the
  travel limiter breakpoints, beyond the trend and sideslip target limits, the low energy deceleration term beyond
  its limits, ELAC 2 with one of its two bits) killed 19 more. The 22 survivors are equivalent:
  - reset and default values that are never read (6): the filtered alpha's reset value, `running_` in `initialize()`
    (everything is already reset), the two rate limiters' default "primed" flags, the speed brake default (limited to
    <= 0 anyway), the buffet CAS below 20000 ft (below the VMCL floor);
  - limits that are never reached (5): the buffet Mach limit 1 (the table tops at 0.77), the travel limit range 0..30
    (the table spans 3.4..25), VMO 350 kt (VFE in CONF 0 is 350 kt too), the 200 ft RA default of the flight phase chart
    (300 is also above 100 ft);
  - values that cannot occur (2): Mach exactly 0.6 (a float never equals the double 0.6), the anticipation limit outside
    CONF 3 / FULL (limited to 0 either way);
  - comparisons at a boundary that give the same value (5): the look-up's first / last breakpoint tests, the saturation
    returning the limit, the CONF bit loop running to an index 6 that the LGCIU bit overwrites;
  - exact equality of computed doubles (4): an inner look-up breakpoint taken from the left interval with fraction 1
    (the same value for these tables), and the low energy comparisons alpha == threshold, VLS == CAS, VLS - 10 == CAS.

### Rerun

```sh
TARGETS='fac-check-stock fac-replay-stock fac-dump-params-stock' JOBS=2 tools/fmgc-lab/scripts/build.sh stock=master
JOBS=2 tools/fmgc-lab/scripts/check-fac-corpus.sh                 # every recording x configs, then fac-replay on each
FAC_ONLY=1 tools/fmgc-lab/scripts/build-fuzz.sh && COVERAGE=1 JOBS=2 tools/fmgc-lab/scripts/check-fac-corpus.sh
tools/fmgc-lab/build/fac-replay-stock ~/fdr-corpus/raw/<file>.fdr          # harness vs sim
tools/fmgc-lab/build/fac-check-stock --script 1 --failures 1 ~/fdr-corpus/raw/<file>.fdr
JOBS=2 tools/fmgc-lab/scripts/mutate-fac.sh
```

## FCOM checks (A319/A320/A321 FCOM, 02 MAY 23)

Matches: FAC 1 controls the yaw functions with FAC 2 in standby, taking over per function (DSC-22_40-10); the flight
envelope function stays active with the FAC pushbutton off (DSC-22_40-10, -30); manual rudder trim 1 deg/s, the knob
and RESET inactive with the AP engaged (DSC-27-10-20); FAC 2's trim synchronised as a backup; yaw alternate law =
yaw damping only, +-5 deg (DSC-27); VLS 1.13 VS at take-off and after a touch and go, 1.23 VS after a flap step, 1.28
VS clean, Mach-corrected above 20000 ft, increased with speed brakes, >= VMCL (DSC-22_10-50-20); F = 1.26 VS(CONF 1+F)
>= VMCL + 15 shown in CONF 2-3, S = 1.23 VS(clean) >= VMCL + 20 shown in CONF 1 (DSC-22_10-50-20, DSC-31-40); VMAX =
lowest of VMO/MMO, VLE, VFE; VFE next below 15000 ft; VLS inhibited from touchdown until 10 s after lift-off; speed
trend = speed in 10 s (DSC-31-40, but A2); alpha floor available from lift-off to 100 ft RA in approach, inhibited
above M 0.6 (DSC-22_30-50-50, DSC-22_40-30); low energy in CONF 2, 3, FULL between 100 and 2000 ft RA, inhibited by
alpha floor and both RA failed (DSC-22_40-30).

Differences and doubtful points are findings A2-A17 (`docs/findings.md` section 9).
