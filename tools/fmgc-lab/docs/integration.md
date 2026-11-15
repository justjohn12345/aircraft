# The readable FMGC computer (integration)

Readable: `readable/fmgc.{h,cpp}` (`fmgc::readable::FmgcComputer`), with the glue it needed: `readable/tcas.{h,cpp}`
(TCAS RA decoding and flags), `readable/law_selection.{h,cpp}` (longitudinal law), `readable/outer_loop_inputs.{h,cpp}`
(the outer loops' input bus), and the `ap_fd_logic` bus assembly in `fmgc.cpp`. Generated: `build/models/stock/
FmgcComputer.cpp` + `FmgcOuterLoops.cpp` (= upstream master). Check: `src/check_fmgc.cpp`,
`scripts/check-fmgc-corpus.sh`.

The readable computer runs on its own state and only ever sees the inputs. It reproduces `FmgcComputer_Y.out` (the
whole `fmgc_outputs`: input copy, logic bus, ap_fd_logic bus, outer-loop commands, A/THR, discrete outputs, A and B
buses) bit for bit, every frame, free-running against the generated model, over the corpus and every scenario
combination (results at the end).

## Interface

```cpp
fmgc::readable::FmgcComputer fmgc;   // no heap, no static mutable state; copyable (snapshot = copy)
fmgc.initialize();                   // power-up: generated FmgcComputer::initialize()
const fmgc_outputs& out = fmgc.step(in);   // in: the generated model's fmgc_inputs (time.dt, sim_data, ...)
```

`step()` takes exactly the generated model's input (`ExternalInputs_FmgcComputer_T::in`): `time.dt` is the frame time
used by every timer and filter, `sim_data.computer_running` gates the frame. It returns the outputs by reference (they
stay valid until the next step). `buses()` keeps this frame's intermediate buses (logic, ap_fd_logic, the outer loops'
`ap_laws_output` with inputs and outputs, athr) and the stage accessors expose each stage's state, for checkers and
probes.

- `computer_running == false`: nothing runs, nothing changes, the outputs stay those of the last running frame (zero
  before the first one, = `out_Y0`), including `out.data`. The generated step only clears `Runtime_MODE`.
- The first running frame after `initialize()` or after a non-running frame is a **restart** (generated 452–692): every
  latch, timer, filter and delay is reset, the previous-frame buses are zero (`Delay_InitialCondition`,
  `Delay1_InitialCondition`, `Delay2_InitialCondition` are all zero / NONE / VS; `check_fmgc` verifies this), and
  `FmgcOuterLoops::reset()` runs. Since the generated `Runtime_MODE` starts false, the first frame after power-up is a
  restart too, so `initialize()` and a restart differ only by the block-output memories below.
- The wasm wrapper (`fmgc/Fmgc.cpp`) sets `computer_running = monitoringHealthy` (power supply, self test, fault) and
  calls `initialize()` only in the constructor; its `clearMemory()` ("erase all data in RAM" after a long outage) is
  empty. So in the sim the memories kept over a restart survive any power cycle.

## Stage graph

One frame, in generated order. "prev" = the previous frame's value (an end-of-step delay or the stage's own memory);
everything else is this frame's.

| # | Stage (readable) | Generated | Reads | Writes |
|---|---|---|---|---|
| 1 | input conditioning (`conditioning::Conditioning`) | 695–1379 | inputs; prev ap_fd_logic (ILS tune inhibit: RWY LOC, LAND, LAND armed) | logic bus except the 8 engagement fields |
| 2 | engagement stage 1 (`Engagement::updateEngagement`) | 1380–1559 | inputs, logic (1); prev ap_fd_logic (modes, `fmgc_opp_mode_sync`, LAND/GA); own AP / A/THR latches (prev) | logic: AP/FD/A/THR common conditions, FD/AP/A/THR own engaged, AP/A/THR inop, opp priority; ap_fd_logic: opp mode sync, any AP/FD engaged, mode reset |
| 3 | TCAS decoding (`tcas::TcasLogic`, new) | 1560–1668, 1792 | inputs (RA word, FCU word 1), logic (ADR V/S, TCAS mode available); prev ap_fd_logic (TCAS mode, armed modes) | `tcas_ra_corrective`, `tcas_vs_target`, `tcas_ra_inhibited`, `trk_fpa_deselected`, `longi_large_box_tcas` |
| 4 | altitude target, vertical and lateral modes (`VerticalModeLogic`, signals `AltCstrApplicable` … `TcasAltHold`, then V/S, FPA, FLARE) | 1636–3330 | inputs, logic, 2 (opp sync, any AP/FD, mode reset), 3 (TCAS V/S target, corrective), conditioning's 5 s in-flight confirm; prev ap_fd_logic (almost every mode, `alt_cstr_applicable`, TCAS submode, `any_longitudinal_mode_engaged`, HDG/TRK preset, selected speed); prev outer loops (`flare_law.condition_Flare`, `Phi_loc_c`); signals updated earlier this frame | every mode / armed mode / reversion, `alt_sel_or_cstr`, `alt_cstr_applicable`; aux: `tla_to_ga_set`, `tcas_alt_acq_cond`, `tcas_alt_hold_cond` (read by the TCAS submode latches); TCAS submode; `any_longitudinal_mode_engaged` |
| 5 | engagement stage 2 (`Engagement::updateFdFlags`) | 3335–3403 | inputs, logic, 2, this frame's modes and reversions (4) | `fd_auto_disengage`, `vs_target_not_held`, `ap_fd_mode_reversion`, `pitch_fd_bars_flashing` |
| 5b | roll FD bars (signal `RollFdBarsFlashing`) | 3392–3404 | lateral reversion, TCAS mode (4), any AP/FD (2) | `roll_fd_bars_flashing` |
| 6 | targets (`targets::Targets`) | 3405–3735 | inputs, logic, this frame's modes (2, 4); prev ap_fd_logic (speed latches); first-frame flag | HDG/TRK preset, selected/managed speed, Mach, `spd_target_kts`, `pfd_spd_target_kts`, ALT soft; the HDG/TRK preset and ALT soft latches are also the mode logic's |
| 7 | law selection (signals `AnyLateralMode`, `ActiveLateralLaw`; `selectLongitudinalLaw`, new) | 3736–3792 | this frame's modes and TCAS submode (4), FMS DES submode | `any_lateral_mode_engaged`, `active_lateral_law`, `active_longitudinal_law` |
| 8 | LAND capability (`Engagement::updateLandCapability`) | 3796–3863 | inputs, logic, AP / A/THR engaged (2), LAND armed / LAND (4); own `pLand3*` memories | LAND 2 / 3 capability, inop, capacity |
| 9 | logic and ap_fd_logic bus assembly (`fmgc.cpp` `copyModes` and the stage outputs, new) | 3864–4054 (ap_fd_logic from 3972; a few fields earlier: 1635, 1666–1668, 1792, 1910, 2465, 2609, 3357, 3402–3404, 3685–3710, 3740–3792) | 2–8 | the whole ap_fd_logic bus |
| 10 | outer-loop inputs (`assembleOuterLoopInputs`, new) | 4055–4170 | inputs, logic, ap_fd_logic (laws, targets, modes, TCAS V/S); own DME and track memories | `ap_laws_output` time / data / input |
| 11 | outer loops (`laws::LateralLaws`, then `laws::VerticalLaws`) | FmgcOuterLoops 2151–4584 | 10; the flare law reads this frame's lateral AP sideslip command | `ap_raw_output` (FD / AP Θ Φ β, nosewheel, `Phi_loc_c`, flare law) |
| 12 | autothrust (`athr::Autothrust` modes, then N1) | 4180–4641 | inputs, logic, ap_fd_logic (laws, speed target, modes, TCAS V/S target, ALT soft); prev athr (`Delay1_DSTATE`) | athr bus |
| 13 | output packing (`output::assemble`) | 4642–5106 | inputs, logic, ap_fd_logic, outer-loop outputs, athr (all this frame; no state) | `fmgc_outputs` |
| 14 | end-of-step delays | 5107–5128 | 9, 11, 12 | prev ap_fd_logic, prev outer loops, prev athr (the other delays belong to their stages: AP / A/THR latch copies, `next_alt_cstr` DetectChange, G/S DetectDecrease, SPD/MACH latch copy, first-frame flag, A/THR TCAS latch, N1 integrator) |

Ordering notes:
- The generated code evaluates the HDG/TRK preset (3434) and the ALT soft mode (3734) once; the readable mode logic
  and the targets block each have a copy (shared functions in `targets.cpp`, so the stage checkers could run them
  separately). The computer runs the targets' copy and writes the result into the mode signals (whose latches are the
  memories the mode logic reads next frame); the mode logic's copies and their state are unused.
- V/S, FPA and FLARE are combinational (the V/S-FPA latch split by the FCU HDG-V/S selection; flare condition or FLARE
  memory). They sit at the end of the `Signal` order (output assembly, 3993–4001) but stages 5 and 6 read them, so the
  computer evaluates them right after the latches; no signal before reads this frame's values.
- The two TCAS submode latches (3311, 3318) take their set terms from the ALT* and ALT conditions computed at 2549 and
  2610 (`VerticalAux`); the vertical stage checker took them from the generated outputs.
- `any_longitudinal_mode_engaged` (4022) = V/S ∨ FPA ∨ every other longitudinal latch ∨ LAND (the generated temporary
  `rtb_fmgcOppPriority_tmp` holds V/S ∨ FPA from 3372).

## State inventory

Reset on restart = reset on the first running frame after a stop (generated init block 452–692). Every memory is reset
by `initialize()`. All memories live in the computer's members; the stages own theirs.

| Memory | Lives in | Generated | Reset on restart |
|---|---|---|---|
| runway heading memory (ILS course, frozen while the tune is inhibited) | `Conditioning` `State::rwyHdgMemo` | `FmgcComputer_B.u_lyjj` | **no** |
| 5 s in-flight confirm | `Conditioning` `State::inFlight5s` | `sf_MATLABFunction_fw` | yes |
| engagement: 6 confirm nodes, 12 edge detectors, 3 monostables, the AP request / AP / A/THR inhibit / A/THR latches, the 2 LAND 3 held capabilities | `engagement::State` (27 memories) | `sf_MATLABFunction_kv` … `ppo`, `Memory_PreviousInput(_g, _g1, _p)`, `Delay_DSTATE_p/_k`, `pLand3FailOp/Pass` | yes |
| TCAS flags: 3 monostables, HDG-V/S edge, HDG-V/S-during-TCAS latch and its delayed reset | `tcas::State` | `sf_MATLABFunction_an, bz, g0, pl4`, `Memory_PreviousInput_e`, `Delay_DSTATE_o` | yes |
| TCAS V/S sample-and-hold | (none: output only when just updated) | `FmgcComputer_B.u_lyjjlj` | (no) |
| mode latches (39) and combinational signals | `Signals` (`FmgcComputer::signals_`) | `Memory_PreviousInput_*` (`docs/latches.tsv`, `src/vertical_latches.h`) | yes |
| mode logic timers, edges, filters (ALT, V/S-TCAS, CLB/DES incl. the two `next_alt_cstr` DetectChange copies, approach incl. the G/S lag and its DetectDecrease, lateral) | `AltState`, `VsTcasState`, `ClimbDescentState`, `ApproachState`, `LateralModesState` in `VerticalModeLogic` | `sf_MATLABFunction_*`, `sf_LagFilter_k`, `DelayInput1_DSTATE(_n, _b)` | yes |
| ALT* target at ALT* engagement (V/S reversion when it moves 250 ft) | `VsTcasState::altCaptureTargetFt` | `FmgcComputer_B.u_ly` | **no** (rewritten on the restart frame) |
| TCAS ALT_ACQ target | `VsTcasState::tcasAltAcqTargetFt` | `FmgcComputer_B.u_lyj` | **no** (rewritten on the restart frame) |
| targets: speed latches, Mach latch, DES lower-margin latch, edges, confirms, ALT soft timers, GA / EO speed memos | `targets::State`, `targets::Values` | `Memory_PreviousInput_cu/_hk/_bo/_dh/_ak/_bw`, `Delay_DSTATE_c`, `vMemoGa/Eo` (+ `_not_empty`), `sf_*` | yes |
| VAPP memo | `targets::Values::vappMemoKts` | `FmgcComputer_B.u_l` | **no** |
| first running frame | `FmgcComputer::firstFrame_` | `Delay_DSTATE_fe` | set |
| ILS DME distance (frozen while the tune is inhibited) | `OuterLoopInputState::locDistanceMemoNmi` | `FmgcComputer_B.u` | **no** |
| track held by GA TRK / RWY TRK | `OuterLoopInputState::trackMemoDeg` | `FmgcComputer_B.u_lyjjl` | **no** |
| lateral laws: NAV rate limiter and lag, HDG / TRACK first-second detectors, turn charts, bank limits, LOC* limit, LOC filters and derivative, ROLL OUT integrator and memories, output stage | `laws::LateralLaws` | FmgcOuterLoops `DWork` | yes (`reset()`) |
| vertical laws: ALT* capture law persistents, speed-on-pitch filters (one set per law and protection), V/S / FPA level-off, flare law, output stage | `laws::VerticalLaws` | FmgcOuterLoops `DWork` | yes |
| ALT hold target | `AltLawState::holdTargetH` | `FmgcOuterLoops_B.u` | **no** (`init()` only) |
| A/THR: TCAS latch and timers, alpha floor, THR / SPEED / RETARD latches, LVR CLB / MCT latches | `athr::ModesState` | `Memory_PreviousInput_jb/_hu/_bh/_cm/_ol/_kr/_km`, `sf_*` | yes |
| A/THR N1: filters, derivative, ALT soft blend, integrator and its load flag | `athr::N1State` | `pY_*`, `sf_LeadLagFilter(_b)`, `sf_LagFilter_g`, `Delay_DSTATE_i/_l/_f`, `icLoad` | yes |
| previous ap_fd_logic / athr / outer-loop outputs | `FmgcComputer::lastApFd_`, `lastAthr_`, `lastOuterLoops_` | `Delay_DSTATE`, `Delay1_DSTATE`, `Delay2_DSTATE` | yes (zero) |
| running | `FmgcComputer::running_` | `Runtime_MODE` | — |
| outputs | `FmgcComputer::out_` | `FmgcComputer_Y.out` | held while stopped |

So 8 values survive a restart, all generated block outputs (`FmgcComputer_B`, `FmgcOuterLoops_B`) that only
`initialize()` / `init()` set: the runway heading memory, VAPP, the ALT* and TCAS ALT_ACQ targets, the ILS DME
distance, the GA/RWY TRK track, the TCAS V/S hold (no effect) and the ALT hold target. The checker compares 65 hidden
memories (every latch plus these) with the generated ones every frame, so a divergence shows before it reaches an
output.

## Glue written for the integration

- **TCAS decoding** (`tcas.cpp`, 1560–1668): `tcas_ra_corrective` = RA word bit 18 ∨ ¬19 ∨ ¬20 (true with no RA);
  `tcas_vs_target` = ±(rate + 200) for a corrective RA (bits 11–16 × 100, 200 added only to a non-zero rate, negative
  with bit 17), else the selected ADR's barometric V/S; `tcas_ra_inhibited`, `trk_fpa_deselected`,
  `longi_large_box_tcas` (10 s monostables). `active_tcas_submode` from the two latches.
- **Longitudinal law selection** (`law_selection.cpp`, 3756–3792): the priority chain of `docs/structure.md`. The
  lateral one was already a mode signal. `check_vertical` and `check_athr` still take `active_longitudinal_law` from the
  generated outputs; `check_fmgc` checks the readable one.
- **ap_fd_logic bus assembly** (`fmgc.cpp`, 3972–4054 and the earlier field writes) including `any_longitudinal_mode_engaged`, `tla_to_ga_set`
  (the SRS takeoff thrust condition, 1683), `tcas_alt_acq_cond` / `tcas_alt_hold_cond` (ALT* / ALT conditions),
  `cruise_active` and `loc_bc_selection` (constants false).
- **Outer-loop input assembly** (`outer_loop_inputs.cpp`, 4055–4170): the 55 arguments of `FmgcOuterLoops::step()`.
- **Outer loops**: every `ap_raw_output` field was already computed by the law stages (the laws check covers each);
  the computer only wires them: lateral laws and their output stage first, the AP sideslip command into the vertical
  laws (flare law), then the vertical laws and their output stage.
- Interface changes to the stages: `VerticalModeLogic::aux` (the three auxiliary conditions), const `state()`
  accessors; `src/vertical_latches.h` (the signal ↔ generated latch map, moved out of `check_vertical.cpp`). Every
  stage check is unchanged otherwise.

## Findings

- One stage fix was needed: the lateral output stage computed the FD roll command as `lag(limited − Φ)`, the generated
  code as `−1 · lag(−1 · (limited − Φ))` (Gain_Gain_lv, Gain_Gain_on). Equal except the sign of an exact zero (a sum of
  opposite values is +0 either way round), which `check_laws` cannot see: it compares `memcmp || ==`, so ±0 pass. The
  whole-computer check compares bit patterns and caught it after a restart in a fuzz + power run (generated −0, readable
  +0 for 2597 frames, `roll_fd_command` on the A bus). Fixed by keeping the two gains (`laws_lateral.cpp`). Other ±0
  differences in law commands that never reach an output are possible; `check_fmgc` sees every one that does.
- The FPA target the outer loops get in DES is `fms_inputs.vs_target_ft_min`, the same field as the V/S target
  (4119–4134): the FMS has to put an FPA in a field named in ft/min when it requests DES FPA_SPEED.
- `tcas_vs_target` passes through a sample-and-hold (`B.u_lyjjlj`) that is updated exactly when it is output, so it is
  stateless; the hold never shows.
- The ILS DME distance and the runway course reach the outer loops frozen together (both follow the ILS tune
  inhibit), and both survive restarts; the GA TRK / RWY TRK track memory too. A restart in GA TRK flies the track
  memorised before the stop.
- Because the generated `Runtime_MODE` is false at construction, the outer loops' `reset()` runs on the first frame
  anyway; `init()` adds nothing but the ALT hold target (and constants `reset()` sets again).
- `Delay_DSTATE_fe` ("first frame") equals "restarted" on every frame: it is set by both initialisation paths and
  cleared at every running step's end.
- The generated restart resets every `DWork` memory (including the filter "not empty" flags); only block outputs
  survive. The sim wrapper never calls `initialize()` again (`clearMemory()` is empty), so those 8 memories survive any
  power outage in the sim.
- Performance: the readable step costs the same as the generated one (about 2.1 µs per frame on this machine, the
  readable about 1 % slower; numbers below). The stage split (per-signal `switch`, per-field targets update) costs
  nothing measurable.

## For Phase 4 (differential fuzzing of the whole computer)

- One frame of each model:
  ```cpp
  // generated
  FmgcComputer::ExternalInputs_FmgcComputer_T ext{}; ext.in = in;
  gen.setExternalInputs(&ext); gen.step(); const fmgc_outputs& g = gen.getExternalOutputs().out;
  // readable
  const fmgc_outputs& r = fmgc.step(in);
  ```
  Compare with the leaf list of `fields.gen.h` (offsets collected once, memcmp per leaf: `check_fmgc.cpp`
  `LeafCollector`), not a whole-struct memcmp (padding).
- Reset between fuzz inputs: readable `initialize()` is a complete reset (power-up of a fresh computer; it allocates
  nothing and costs a few kB of stores). The generated `initialize()` is **not** a complete reset of a used instance:
  it does not clear `Runtime_MODE`, so the next running frame skips the restart block and the confirm nodes, edge
  detectors, monostables, filters and the outer loops' filters keep their values (it resets the latches, the delays,
  the block outputs and calls `FmgcOuterLoops::init()`). In the sim it is only called on a new object, where it is
  complete. Re-construct the generated model in place instead (`gen->~FmgcComputer(); new (gen) FmgcComputer();
  gen->initialize();`, copy and move are deleted), as `check_fmgc --warm FILE` does: both computers run over another
  recording first, then reset, then the main run is compared (0 mismatches over three file pairs with power and approach scenarios). Make the first fuzzed frame
  running or not explicitly; the first running frame is the restart.
- Deterministic: no clocks, no randomness, nothing static written during a step in either model (the generated
  parameters are non-const statics, never written). Same inputs, same outputs, bit for bit, at `-O2 -ffp-contract=off`
  (keep FMA contraction off for both: the readable laws reproduce the generated operation order, a fused multiply-add
  changes the last bit).
- Input domain: libFuzzer bytes can make `bool` fields other than 0/1 (undefined behaviour; the two models may branch
  differently) and enums out of range; sanitize those. NaN / Inf floats and dt ≤ 0 or huge dt are legal for both but
  untested here; expect the bit-exactness to hold on the mode logic and to be worth a look on the laws (NaN payloads
  and the order of comparisons with NaN).
- Coverage hints: the recordings have no ILS approach, TCAS RA, failure or restart; the scenario generators
  (`src/scenario_*.h`) show which input patterns reach those paths, and `check_fmgc` prints per-field change counts and
  enum values (`cov`, `enum` lines) to measure what a fuzz corpus reaches.
- Coverage seen over the corpus run below (generated values): every lateral and longitudinal law, every A/THR FMA
  message except 1 and FMA mode except 2 and 4, the TCAS submodes VS and ALT_ACQ but **never ALT_HOLD** (an RA with
  ALT armed that holds within 40 ft for 0.8 s is not generated by any scenario), `loc_bc_selection` and
  `cruise_active` never change (constants). A fuzzer that reaches TCAS ALT_HOLD would cover the one readable path
  (`activeSubmode` ALT_HOLD → ALT_HOLD law) no run has exercised.

## Verification (2026-11-14)

`JOBS=4 scripts/check-fmgc-corpus.sh 2`: 22 recordings, plain plus, per seed (2), every scenario alone (8), every pair
(28), all but one (8) and all (1): **2002 runs, 279.9 M steps (266.9 M running), 49 123 (re)starts, 0 mismatching
fields, 0 hidden-memory divergences** (17.6 min at 4 jobs). The first attempt found the FD roll ±0 difference above
(2 runs, both with `--power`).

Mutation check (temporary edits, reverted): the TCAS flag duration 10 → 9.9 s is caught by the `--athr` and `--targets`
runs (`tcas_ra_inhibited`, `trk_fpa_deselected`, FMGC A bus word 7); the DME memo updated while inhibited instead of
while not shows as a hidden-memory divergence at once and reaches `Phi_loc_c` later; DES VS_SPEED → FPA law is caught
on `active_longitudinal_law` in the plain replay.

Timing (single thread, `--time`, the 22 recordings, 3.08 M steps of which 2.97 M running, chrono around each call):
generated 2.07 µs/step, readable 2.10 µs/step (repeats: 2.07 / 2.09, 2.03 / 2.06), i.e. 6.4 s for the corpus each.

Stage corpus scripts re-run after the integration (default seeds, `JOBS=4`), all 0 mismatches: vertical 374 runs,
laws 132, athr 440, engagement 572, targets 440, conditioning 638, output 484.
