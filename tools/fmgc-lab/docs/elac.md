# The A32NX ELAC (generated `ElacComputer` and its control laws) and its readable rewrite

The Elevator Aileron Computer model: sensor voting (ADR, IR, RA), on ground and the pitch law's flight / ground chart,
hydraulic supply, sidestick priority, engagement in pitch and roll (ELAC 2 has pitch priority, ELAC 1 roll priority),
law capability and the active laws (normal, alternate 1 / 2, direct, abnormal attitude), the protections (high angle
of attack, high speed, AP disconnect and authorisation), and the orders: ailerons (with droop / anti-droop and
cross-command), roll spoilers, yaw damper, elevators (double pressurisation), THS (auto trim, ground setting). The
laws are separate generated classes stepped every frame: `LateralNormalLaw`, `LateralDirectLaw`, `PitchNormalLaw`,
`PitchAlternateLaw`, `PitchDirectLaw` (the SEC steps the last three too, see `docs/sec.md`).

- Generated: `fbw-a32nx/src/wasm/fbw_a320/src/model/ElacComputer.cpp` (1945 lines), `PitchNormalLaw.cpp` (2465),
  `PitchAlternateLaw.cpp` (906), `LateralNormalLaw.cpp` (498), `PitchDirectLaw.cpp` (61), `LateralDirectLaw.cpp` (45),
  with their `.h`, `_data.cpp`, `_types.h` and `look1_binlxpw` / `look2_binlxpw`. Never edited. Identical in every lab
  variant (stock, cst, cst_athr, combined).
- Readable: `tools/fmgc-lab/readable/elac/` (namespace `elac::readable`) and the laws in `readable/efcs/` (namespace
  `efcs::readable`, shared with the SEC); bit-exact (below).
- Wrapper: `fbw_a320/src/elac/Elac.cpp`; sim wiring `FlyByWireInterface::updateElac()` (~1350).
- Parameters (block name = value): `docs/params-ElacComputer.txt`, `docs/params-EfcsLaws.txt` (`efcs-dump-params`).
- Findings: `docs/findings.md`, section 10 (L1-L16).

## Where it runs

Every frame after the FMGCs: ELAC 1, ELAC 2, then SEC 1-3, FAC 1-2, FCDC 1-2, FADECs and the recorder. ELAC 1 sees
ELAC 2's outputs of the previous frame, ELAC 2 ELAC 1's of this frame; both see the previous frame's SEC, FAC and FCDC
outputs. The EFCS computers do not run in active pause.

`Elac::update()` monitors the power supply (DC ESS for ELAC 1, DC 2 or the battery for ELAC 2): an outage over 10 ms
stops the model, a self test (short, or long after 3 s without hydraulics) follows; `monitoringHealthy` = no fault,
power, self test complete and the pushbutton on = `computer_running`. Unhealthy: every bus word FailureWarning with 0
(but `elevator_double_pressurization_command_deg` and `speedbrake_extension_deg`, which the wrapper leaves at {0, 0},
also FailureWarning), discretes false, orders 0.

When `computer_running` is false the model holds everything; the first running frame afterwards **restarts** it
(generated 371-426): every memory back to its initial state and the five laws `reset()`, except the THS integrator
(reloaded from the measured THS, `icLoad`) and the event times (reloaded on their first use).

## Structure (generated line ranges -> readable)

| Generated | What | Readable |
|---|---|---|
| ElacComputer 31-258 | library blocks (SSM tests, BitfromLabel, rate limiters, lag filter, confirm node, hysteresis, pulse node, bit packing, flap handle index) | `efcs/efcs_blocks.h` (+ `blocks.h`, `fac/fac_blocks.h`), `protections.cpp` `flapHandleIndex` |
| 371-426 | restart | `elac.cpp` `restart()` |
| 428-546 | ADR validity, voting (average of two valid, else the valid one), alpha lag 0.5 rad/s | `sensors.cpp` `voteAirData` |
| 549-663 | IR validity and voting (pairs 2+3, 1+2, 1+3; singles 1, 3, 2) | `sensors.cpp` `voteInertialData` |
| 664-730 | RA: disagreement, coherence monitor (L1), value | `sensors.cpp` `updateRadioHeight` |
| 730-777 | on ground, the pitch law flight / ground chart | `sensors.cpp` `updateOnGround`, `updateFlightPhase` |
| 779-801 | hydraulic supply (pressure 1750 / 1450 psi hysteresis and pressure switch, 0.5 s) | `sensors.cpp` `updateHydraulics` |
| 802-847 | sidestick priority, roll stick total | `engagement.cpp` `updateSidestickPriority` |
| 849-894 | elevator / THS / aileron availability, engagement, cross-command | `engagement.cpp` `updateEngagement` |
| 895-983 | lateral and pitch law capability, abnormal attitude | `engagement.cpp` `updateLawCapability` |
| 985-1071 | the other ELAC's capabilities, active laws, pitch stick total | `engagement.cpp` `updateActiveLaws` |
| 1073-1085 | THS ground setting, THS commanded | `engagement.cpp` `updateThsEngagement` |
| 1086-1238 | alpha prot / max, high speed protection, high AoA protection, AP disconnect phases and disconnect, AP authorisation | `protections.cpp` |
| 1239-1384 | AP orders, lateral laws, droop, aileron orders, roll spoiler, yaw damper | `orders.cpp` `updateLateralOrders` |
| 1385-1620 | pitch laws, elevator order and double pressurisation, THS integrator, damping, battery supply | `orders.cpp` `updatePitchOrders` |
| 1621-1913 | discrete words, bus, discrete / analog outputs, logic copies | `output_packing.cpp` |
| 1919-1931 | initialize | `elac.cpp` |
| LateralNormalLaw 196-491 | flight / ground mode chart, pedal feed-forward (unused, L9), roll rate -> bank integrator with bank protection, roll attitude law, turn coordination / yaw damping, ground yaw | `efcs/lateral_normal_law.cpp` |
| LateralDirectLaw | aileron = -25 deg x stick, 50 deg/s | `efcs/lateral_direct_law.h` |
| PitchNormalLaw 1388-2421 | flare mode, auto trim and freeze, flight phase, C* channels (theta max / min, nz max / min, AP, high AoA, flare, normal with high speed protection), votes, in-flight integrator, ground law and rotation | `efcs/pitch_normal_law.cpp`, `efcs/pitch_law_blocks.h` (map in the file comments) |
| PitchAlternateLaw 441-887 | the normal law's in-flight part with the speed stabilities instead of protections | `efcs/pitch_alternate_law.cpp` |
| PitchDirectLaw | elevator = -30 deg x stick (-30..+17), 45 deg/s; THS limits -11 / +3.5 | `efcs/pitch_direct_law.cpp` |

`readable/elac/elac.h`: `elac::readable::ElacComputer` with `initialize()` and `const elac_outputs& step(const
elac_inputs&)`, no heap, no static state, state accessors for the checker. Its `initialize()` is a complete reset
(checked with `elac-check --warm`); the generated one is not (`Runtime_MODE`, filters, laws), so the checker
re-constructs the generated instance.

## Inputs (`elac_inputs`, `FlyByWireInterface::updateElac`)

| Field | Sim source | Used for |
|---|---|---|
| `time.dt`, `simulation_time`, `monotonic_time` | frame time, sim time, time since start | filters, event times, alpha protection after 10 s |
| `sim_data.slew_on / pause_on / tracking_mode_on_override` | slew (the frame after), pause, external override | tracking mode; slew 0.2 s confirm inhibits the abnormal attitude law |
| `tailstrike_protection_on` | ModelConfiguration.ini (default off) | pitch normal law |
| `computer_running` | `monitoringHealthy` (wrapper) | run / hold / restart |
| `ground_spoilers_active_1/2` | SEC 1 and SEC 2 (ELAC 1) / SEC 3 (ELAC 2) ground spoiler out | on ground, anti-droop |
| `is_unit_1/2`, `opp_axis_pitch_failure`, `opp_left/right_aileron_lost` | unit; the other ELAC's pitch axis ok / aileron ok | priority, cross-command |
| `ap_1/2_disengaged` | !FMGC 1 / 2 AP engaged | AP control, protections, AP authorisation |
| `fac_1/2_yaw_control_lost` | !FAC yaw damper available for normal law | lateral law capability |
| `lgciu_1/2_left/right_main_gear_pressed` | LGCIU discretes (nose gear inputs unused) | on ground |
| `l/r_ail/elev_servo_failed`, `ths_motor_fault` (false), `ths_override_active` | failures, THS override | availability, bus SSMs |
| `yellow/blue/green_low_pressure` | !pressure switch | hydraulic supply |
| `capt/fo_priority_takeover_pressed` | sidestick pushbuttons (also the AP instinctive disconnect) | sidestick priority |
| `elac_engaged_from_switch`, `normal_powersupply_lost`, `sfcc_1/2_slats_out`, `load_factor_acc_1/2_g` | | unused by the model |
| `capt_pitch/roll_stick_pos`, `rudder_pedal_pos` (F/O sticks 0) | -sim inputs | laws, AP authorisation, bus |
| `left/right_elevator_pos_deg`, `ths_pos_deg`, `left/right_aileron_pos_deg` | hydraulics deflections x 30 / x 25, -eta trim | bus copies; THS integrator; aileron orders while not engaged; normal law `eta_deg` (unused by the law) |
| `blue/green/yellow_hyd_pressure_psi` | system pressures | hydraulic supply |
| `adr_1/2/3`, `ir_1/2/3`, `ra_1/2` | ADIRS, RAs | sensors |
| `fmgc_1/2_bus` | FMGC B buses: AP roll / pitch / yaw orders | AP orders |
| `sfcc_1/2_bus` | SFCC: actual position word bits 19, 20, 23; system status word (flap handle index) | droop, gear-down fallback, flaps FULL, alpha limits, laws |
| `fcdc_1/2_bus` | status word 1 bit 29 (SEC 3 lost), word 3 bits 23 / 24 (copies of SEC 1 word 1 bits 15 / 16) | lateral capability |
| `sec_1/2_bus` | discrete words (spoiler pairs, gear), TLAs, speed brake command | lateral capability, gear down, pitch law TLAs, damping, bus copy |
| `elac_opp_bus` | the other ELAC: capabilities, aileron order, double pressurisation order, elevator availability | active laws, cross-command, elevator, damping |

## Functions

### Sensors (`sensors.cpp`)
- **ADR**: valid unless Mach, CAS, TAS or AoA is FailureWarning. CAS, TAS, Mach, AoA = the float average of the first
  valid pair (2+3, 1+3, 1+2), else the only valid ADR, else 0. Double / triple failure flags. Alpha filtered (lag
  0.5 rad/s) for the protections.
- **IR**: valid unless pitch, roll, yaw rate, accelerations or attitude rates are not NormalOperation (the body pitch
  rate is used but not monitored). Pairs 2+3, 1+2, 1+3, singles 1, 3, 2.
- **RA**: both valid: the average; disagreeing by more than 50 ft for 1 s: the lower one once the flaps (SFCC bit 23)
  have been FULL for 10 s, else 250 ft. One valid: that one. None: 250 ft. The coherence monitor and the 250 ft
  default above 180 kt are unreachable (L1).
- **On ground**: both LGCIUs' main gears, or one LGCIU below 50 ft RA, or both ground spoiler discretes below 50 ft.
- **Flight / ground chart** (pitch laws' `in_flight`): to flight off the ground with pitch > 8 deg or above 400 ft RA;
  back on the ground 5 s after touchdown with pitch < 2.5 deg.
- **Hydraulics**: pressure above 1750 psi (down to 1450) and the pressure switch, confirmed 0.5 s.

### Engagement and laws (`engagement.cpp`)
- **Sidestick priority**: a takeover press disables the other stick while held, for good when held 30 s; the last
  press wins.
- **Pitch**: ELAC 1 drives both elevators on blue, ELAC 2 the left on green and the right on yellow; the THS needs
  green or yellow. ELAC 2 has priority, ELAC 1 takes over when ELAC 2 reports its pitch axis lost.
- **Roll**: ELAC 1 left aileron blue / right green, ELAC 2 the opposite. ELAC 1 has priority; ELAC 2 takes over when
  ELAC 1 lost both ailerons and cross-commands an aileron ELAC 1 lost with ELAC 1's aileron order.
- **Lateral capability**: direct with both FACs' yaw damping lost, all three SECs lost, or no spoiler pair available on
  SEC 1 / 2 (their word 1 bits 15 / 16 and the FCDC copies).
- **Pitch capability**: alternate 2 with all ADRs failed, after an abnormal attitude (latched until the ground), or
  both ailerons and an elevator lost; alternate 1 with two IRs or two ADRs failed or an elevator lost; direct with all
  IRs failed or, in flight with the gear down (SEC word 2 bit 18; SFCC bit 20 when no SEC knows the gear), when an
  alternate law condition, both RAs failed or the lateral direct law applies. Normal otherwise.
- **Abnormal attitude**: bank > 125, pitch > 50 / < -30, CAS > 440 / < 60 kt, Mach > 0.91, AoA > 40 / < -10 deg
  (L10).
- **Active laws**: normal only when the roll ELAC's lateral capability and the pitch ELAC's pitch and lateral
  capabilities are normal (ALT2 read as ALT1 across ELACs, L4); otherwise roll direct, pitch alternate 1 when only the
  roll side lost its normal law, else this ELAC's capability; direct in the remaining case (L5).
- **THS**: commanded in flight in normal / alternate law without abnormal attitude; after touchdown (falling edge of
  in flight) the ground setting drives it to 0 at up to 0.7 deg/s until within 0.02 deg.

### Protections (`protections.cpp`)
- alpha max / prot: tables of Mach and the flap handle index, rate limited 1 deg/s; alpha prot = alpha max for 5 s
  after lift-off.
- High speed protection: without AP in a normal law above min(356 kt, M 0.83 as CAS) (350 / 0.82 when diving more
  than 3 deg), until below 350 kt / M 0.82; thresholds 350 / 380 kt, M 0.82 / 0.88 to the pitch law.
- High AoA protection: above alpha prot in flight in a normal law without AP (10 s after power-up); held while the
  stick is not pushed forward of -0.5 or of -1/32 for 0.5 s, and alpha >= alpha prot - 2 deg, the stick at 0.5 or more,
  or above 200 ft RA.
- AP disconnect: alpha > alpha prot + 0.25 (alpha max below 100 ft RA in approach), 3 s above min(365 kt, M 0.86 / 0.83
  diving) unless both laws degraded, or a protection active. AP authorised with sticks within half deflection, pedals
  within 0.4, pitch -13..25 deg, bank within 45 deg, its axes available.

### Orders (`orders.cpp`)
- Ailerons: the active lateral law's order (direct law: -25 x stick at 50 deg/s), plus droop 5 deg (flaps extended,
  1 deg/s) and anti-droop -30 deg (ground spoilers, flaps, pitch < 2.5, no AP, normal law, 20 deg/s), +-25 deg, 50
  deg/s; following the measured positions while not engaged.
- Roll spoiler order: 1.4 x the roll order (5 deg dead band and gain 1.25 outside flaps FULL).
- Elevator: the active pitch law's order; double pressurisation above 30 deg (180 kt) .. 6 deg (400 kt); not engaged in
  pitch: the other ELAC's double pressurisation order.
- THS: the law's trim rate integrated, within the law's limits.

## Outputs

ELAC bus (the full bit table is in `readable/elac/output_packing.cpp`): surface positions (NO, NCD when the servo
failed), sidestick / pedal commands (16 / 20 / 30 deg per unit), aileron command (NO when engaged in roll with an
aileron lost), roll spoiler command (NO when engaged in roll), yaw damper (NO in roll normal law), double
pressurisation order, SEC 1's speed brake command copied, discrete words 1 (availability, engagement, active laws) and
2 (capabilities, sidestick priority, droop, AP, alpha protection). Discretes: pitch axis ok, aileron ok, AP authorised
(both), active modes, damping modes, THS active, battery supply. Analog orders: elevators, THS, ailerons. Consumers:
the other ELAC, the SECs, the FACs (yaw damper order, laws, speed brakes), the FCDCs (ECAM, PFD), the FMGCs (AP
authorisation), the hydraulics (orders, modes).

## Hidden state (compared by the checker)

`Runtime_MODE`; sensors (alpha filter, 3 RA confirm nodes and the 2 rejection latches, the FULL time, both charts and
`in_flight`, touchdown time, 3 hysteresis nodes and 3 confirm nodes); engagement (2 pulse nodes, stick disabled /
locked memories and their confirm nodes, slew confirm, abnormal latch, landing pulse, THS ground latch); protections
(2 rate limiters, 3 event times with their flags, both protection latches, the AP chart); orders (4 rate limiters, the
pitch acceleration filter and delay, THS order, integrator and reload, battery confirm); and every law memory (11
lateral normal, 2 lateral direct, 89 pitch normal + block IO, 28 alternate, 2 direct). 306 values per ELAC, 612 per
step.

## Replay: inputs rebuilt from the record (`src/efcs_drive.h`, `elac-replay`)

No recording format up to 3209004 holds the ELAC model inputs; the drive rebuilds them (the file comment lists every
field), with a hook (`recordedInputs()`) for a format that records them. Exact: time, sticks and pedals, THS, hydraulic
pressures, ADIRS / RA (FMGC 1's recorded bus inputs), FMGC buses and AP discretes (as `fac_drive.h`), the other ELAC,
SECs and FACs (records of the right frame), SFCC words from the FACs, LGCIU main gears from the FACs, FCDC bits from the
SECs, servo failures from the bus SSMs; the sidestick takeover buttons are searched per frame until discrete word 2
matches (they are the AP disconnect buttons: pressed in most flights). Estimated: the pressure switches (the Rust
switches are filtered and relative to ambient; the drive uses their 1750 / 1450 psi hysteresis on the recorded psi),
slew (a frame after a slew), the elevator and aileron positions (the float copies on the bus).

Result (30 recordings, 4.33 M frames): ELAC frames with known inputs 7,767,464, of which 4,820,654 (62 %) bit-exact on
every bus word, discrete and analog order; every bus word and discrete is bit-exact on all of them. The remaining
frames differ only in law-driven analog orders, from lift-off on: elevator orders (at most 2.1e-7 deg), THS order
(1.4e-12 deg), aileron orders (2.8e-14 deg), a few ulps that the integrators carry on (L2). Approximate frames
(756,042: before FMGC 1 runs, SFCC unknown, hydraulic transitions, long frames, + 60 s): 625,074 bit-exact.

## Verification (readable vs generated)

- `elac-check-<variant>` (src/check_elac.cpp): both ELACs, generated and readable in lockstep, free-running, every leaf
  of `elac_outputs` (717 per ELAC, the input copy included) and the 306 hidden memories per ELAC bit-exact every step,
  plus the power-up outputs and the latch table. Floats by bit pattern, two NaNs equal (harness rule).
- Law-level checks with synthetic trajectories: `efcs-lateral-law-check` (src/check_lateral_laws.cpp),
  `efcs-pitch-law-check` (src/check_pitch_laws.cpp, 20 seeds x 1 M steps per law, 0 mismatches).
- Injectors: `--script` (src/elac_scenarios.h: cross-command, RA disagreement / failures, high speed and high AoA
  protections, abnormal attitudes, sidestick priority lock, THS ground setting, double pressurisation, ADR / IR
  voting patterns, law capability words, droop), and the generic `--failures`, `--discretes`, `--power`, `--sim`
  (src/efcs_scenarios.h).
- `UNIT=elac scripts/check-efcs-corpus.sh`: RESULTS_ELAC
- Coverage: COVERAGE_ELAC
- Mutation test: MUTATION_ELAC

### Rerun

```sh
TARGETS='elac-check-stock elac-replay-stock efcs-dump-params-stock efcs-lateral-law-check-stock efcs-pitch-law-check-stock' \
  JOBS=2 tools/fmgc-lab/scripts/build.sh stock=master
UNIT=elac JOBS=2 tools/fmgc-lab/scripts/check-efcs-corpus.sh
EFCS_ONLY=1 JOBS=2 tools/fmgc-lab/scripts/build-fuzz.sh && UNIT=elac COVERAGE=1 JOBS=2 tools/fmgc-lab/scripts/check-efcs-corpus.sh
tools/fmgc-lab/build/elac-replay-stock ~/fdr-corpus/raw/<file>.fdr
UNIT=elac JOBS=2 tools/fmgc-lab/scripts/mutate-efcs.sh
```

## FCOM checks (A319/A320/A321 FCOM, 02 MAY 23, DSC-27)

Matches: the law reconfiguration table (two ADRs or IRs: alternate with reduced protection; three ADRs: alternate
without protection; three IRs: direct; both RAs: direct at gear extension, a flap condition when both LGCIUs are lost);
alternate law -> direct (flare mode) at gear extension; abnormal attitude law -> alternate without protection and roll
direct for the rest of the flight; alpha prot = alpha max for 5 s at take-off; AP disconnect at VMO + 15 kt / MMO +
0.04; high speed protection off below VMO / MMO; anti-droop conditions (ground spoilers, flaps, pitch < 2.5 deg, manual,
normal law). Differences: L10 (abnormal attitude thresholds).
