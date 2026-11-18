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
- Findings: `docs/findings.md`, section 9 (A1-A18).

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
