# fmgc-lab

Native (no MSFS) build of the generated A32NX FMGC (`FmgcComputer` + `FmgcOuterLoops`) with a flight data recorder
replay harness. Phase 0 of the FMGC reverse-engineering plan (`~/fmgc-lab/PLAN.md`). The FCU (`FcuComputer`) and the
FADEC (`FadecComputer`) have the same treatment: see "FCU" and "FADEC" at the end.

**Findings:** every bug and FCOM difference found so far, with status, is indexed in `docs/findings.md`.

## Build

No host compiler is needed: the build runs in the FBW dev container and produces static binaries that run on the host.

```sh
tools/fmgc-lab/scripts/build.sh                      # default variants: stock, cst, cst_athr
tools/fmgc-lab/scripts/build.sh mine=my-branch       # any name=git-ref pairs
```

Each variant is the generated `model/` folder at a git ref (`scripts/extract-model.sh`), built into
`build/fmgc-replay-<name>`. Default variants:

| Variant | Ref | What |
|---|---|---|
| `stock` | `master` | upstream generated code |
| `cst` | `personal/fg-alt-cst-reversion` | + ALT CST* → V/S on constraint sequencing patch |
| `cst_athr` | `efc76f451` | + A/THR SPEED in FINAL patch (test/combined as installed 2026-11-12) |
| `combined` | `test/combined` | the current personal build, including FMGC fixes from `personal/fmgc-fixes` (e.g. ALT* creep) |

## Replay

```sh
build/fmgc-replay-cst_athr [--from N] [--to N] [--filter name] [--abs-tol X] [--rel-tol X] file.fdr
scripts/replay-corpus.sh [~/fdr-corpus/raw] [replay options]   # every file, with the variant from corpus-variants.tsv
```

For each recorded frame, the harness rebuilds `fmgc_inputs` the way `FlyByWireInterface::updateFmgc()` does
(dt and simulation time from the base data, recorded bus/discrete/FMS inputs), calls `step()`, and compares every
output field (logic, ap_fd_logic, outer loops, athr, discretes, A/B buses) with the recording. The report lists each
differing field with its number of frames, first/last frame, values at the first difference and the largest difference.

Details:
- **Computer not running** (power-up self-test, unpowered): the model leaves its outputs untouched, so the recorded FMGC
  block is frozen; such frames are stepped with `computer_running = false` and not compared.
- **Unrecorded frames**: the recorder is skipped during active pause while the computers keep running. These show as a
  simulation time jump larger than the frame's dt; the harness steps through them with the last inputs held
  (`--no-fill-gaps` to disable). Control-law filter states still differ slightly for a while afterwards.
- **Not recoverable**: `sim_data.slew_on` (true on the first frame after a slew), `tracking_mode_on_override`.

## Results (2026-11-13, 17 recordings, ~2.7 M frames, ~6 s for the whole corpus)

- Every mode, logic and discrete field matches on every frame of every recording, except frame 0 of some files.
- Recordings without unrecorded gaps match on every field within 1e-6 (native glibc maths vs the sim's wasm maths).
- With gaps, small control-law command differences (a few hundredths of a degree, < 0.4 % N1) start exactly at the gap
  and die out afterwards.
- Speed: ~400,000 `step()` calls per second per core (~100,000 frames/s including decompression and comparison).
- Detection check: replaying 2026-11-12-17-08-01 (recorded on `cst`) through `cst_athr` shows the A/THR FMA mode
  replayed as SPEED (7) where the sim latched THR IDLE (12) at FINAL engagement (t = 4472.87 s).

## Phase 1 tools (mapping the generated code)

Built by `scripts/build.sh` alongside the replay binaries:

- `build/fmgc-blocks-test-<variant>`: differential test of the readable block library (`readable/blocks.h`: A429
  decoding, ConfirmNode, PulseNode, MTrigNode, SR flip-flop, lag/lead-lag/washout filters, switch blend, turn-direction
  chart, heading bank limit, NAV gains, median voter, V/S limiter, speed protection) against the generated helpers on
  random input sequences: bit-identical over ~71 M checks. Also checks that all 55 SR flip-flop truth tables are the
  Simulink default (reset wins).
- `build/fmgc-dump-params-<variant> [FmgcComputer|FmgcOuterLoops]`: every parameter as `block name = value`
  (`docs/params-*.txt`).
- `build/fmgc-probe-alt-capture-<variant> file.fdr`: ALT* capture law state at every capture (see
  `docs/laws/alt-capture.md`). `src/replay_core.h` is the shared replay loop for writing probes like it.

Backward slicer: `scripts/ast.sh <variant> <FmgcComputer|FmgcOuterLoops>` dumps the clang AST of `step()` (in the dev
container), then

```sh
python3 scripts/slice.py build/ast/stock-FmgcComputer.json build/models/stock/FmgcComputer.cpp \
  FmgcComputer_DWork.Memory_PreviousInput_ne --at 2549 --no-wrap --stop 'Memory_PreviousInput|Delay_DSTATE' \
  --params docs/params-FmgcComputer.txt --keys
```

prints only the lines that can affect a variable at a given line (data and control dependencies, with reaching
definitions so the generator's reused temporaries are resolved), annotated with parameter values, and lists the leaves
(inputs, parameters, previous-frame state). `docs/latches.tsv` lists every SR flip-flop latch, its update line and the
mode output it drives.

Documents: `docs/modes/*.md` (vertical mode logic: set/reset conditions per mode), `docs/laws/alt-capture.md`.

## Phase 2: readable vertical mode logic

`readable/vertical.h` declares every vertical mode, armed mode and derived flag as a `Signal`, updated in the order the
generated `step()` computes them. Each group has its own files: `readable/vertical_<group>.{h,cpp}` (logic, block
memories, inputs it needs from the parts not rewritten yet) and `src/external_<group>.h` (fills those inputs from the
generated model). Groups: `alt` (ALT arm possible, ALT armed, ALT*, ALT, ALT CRZ), `vs_tcas`, `climb_descent`,
`approach`.

Check against the generated stock model (the oracle, so any recording works, whatever build flew it):

```sh
build/fmgc-check-vertical-stock [--free] [--fuzz SEED] file.fdr
scripts/check-vertical-corpus.sh [fuzz runs per file]     # every recording: resync, free-running, fuzzed
```

- resync: each signal starts from the generated values and is compared and reset to the generated value right after
  its readable update, so a mismatch points at one signal's logic.
- `--free`: the readable logic keeps its own state, as it would in the sim.
- `--fuzz SEED`: random episodes where one input field (discrete, FMS or any ARINC bus field) is stuck at a mutated
  value for 0.5–30 s (`src/input_fuzzer.h`). The recordings alone miss most edges (a changed 0.72 s takeover delay or
  250 ft threshold passes them); with fuzzing those mutations are caught.

Signals not rewritten yet take the generated value.

## Phase 3: readable vertical control laws

`readable/laws.h`: every vertical law computes its FD and AP pitch command (delta theta) every frame; the output stage
(`readable/laws_output.cpp`, done) picks the active law's and turns them into the FD and AP theta commands. Groups:
`laws_alt` (ALT_ACQ, ALT_HOLD), `laws_vs_fpa`, `laws_speed` (SPD_MACH, SRS), `laws_path` (G/S, VPATH, FLARE + flare law
outputs). Laws not rewritten yet return NaN and are skipped.

Check: `build/fmgc-check-laws [--free] [--fuzz SEED] [--approach SEED] [--tol X] file.fdr`, whole corpus
`JOBS=4 scripts/check-laws-corpus.sh [fuzz runs]`. It runs an instrumented copy of the stock model
(`scripts/gen_hooked_model.py`, built into `build/cmake/hooked-stock/`; the repo's model/ is never touched): the
FmgcComputer's calls to its outer loops are captured with all inputs and outputs (`src/lab_hooks.h`), and probe
statements inserted before given lines (`src/probes/*.txt`, `BEFORE <line> <name> <expr>`) report intermediate values.
`law_selection.txt` taps every law's FD/AP command just before the two law switches, so each law is compared every
frame whichever law is active; add your own probe file to tap a law's internals while debugging. Exact comparison.

## Phase 5c: readable AP / FD / A/THR engagement

`readable/engagement.{h,cpp}`: the AP/FD/A/THR common conditions, FD engaged, AP engage request and AP engaged, A/THR
inhibit and A/THR engaged, AP/A/THR inop, opposite-FMGC priority and mode sync, `any_ap_fd_engaged`, the mode reset
(stage 1, before the mode logic) and the FD auto disengage, V/S target not held, mode reversion and pitch FD bars flags
(stage 2, after the vertical modes). Spec: `docs/modes/engagement.md`.

Check: `build/fmgc-check-engagement-stock [--free] [--fuzz SEED] [--approach SEED] [--athr SEED] [--lateral SEED]
[--engagement SEED] file.fdr`, whole corpus `JOBS=4 scripts/check-engagement-corpus.sh [runs]`. `--engagement` injects
synthetic AP/FD/A/THR episodes (`src/scenario_engagement.h`): pushbutton presses, disconnects, the engagement envelope,
FD pushbuttons, opposite-FMGC patterns and unit 2, ground windows with engine starts and levers, A/THR presses around
100 ft RA, common condition losses. The report lists, per output, mismatches, steps true and rises (generated), and
the steps each intermediate condition was reached (readable, = generated in resync mode).

## Phase 5d: readable targets block

`readable/targets.{h,cpp}`: HDG/TRK preset, selected/managed speed, the SPD/MACH latch, the speed memos, the speed target
for the laws and the A/THR (`spd_target_kts`), the PFD target and the ALT soft mode (spec: `docs/modes/targets.md`).
The preset and ALT soft functions are shared with the vertical/lateral mode logic.

```sh
build/fmgc-check-targets-stock [--free] [--fuzz SEED] [--approach SEED] [--targets SEED] [--events] file.fdr
JOBS=4 scripts/check-targets-corpus.sh [fuzz runs per file]
```

`--targets SEED` adds synthetic speed-mode episodes (`src/scenario_targets.h`); `--events` prints a timeline of the
selected/managed/Mach changes with the FCU and FMS speeds. Every field (latches, memos, both speed targets) is compared
exactly; the report lists per-field coverage and the named set/reset causes and target branches seen.

## Phase 5e: readable input conditioning

`readable/conditioning.{h,cpp}`: the logic bus except the engagement fields: ADR / IR 2-of-3 voting and selection
(own, then 3), indicated altitude, RA, FAC selection and its failure flags, on ground and the 5 s in-flight confirm,
flap lever, engines, FCU, ILS (with the back-beam sign), TCAS mode availability, ILS tune inhibit and the runway heading
memory (spec and findings: `docs/conditioning.md`).

```sh
build/fmgc-check-conditioning-stock [--free] [--fuzz SEED] [--approach SEED] [--engagement SEED] [--lateral SEED] \
  [--failures SEED] file.fdr
JOBS=4 scripts/check-conditioning-corpus.sh [runs per file]
```

Every leaf of the logic bus (172: each ARINC word's SSM and data, floats bit for bit) is compared. `--failures SEED`
injects sensor and equipment failures (`src/scenario_conditioning.h`): SSM FW/NCD/FT on one, two or three ADRs/IRs,
RA/ILS/FAC/FCU/TCAS failures, discrete flips, thresholds and glitches around the 5 s confirmation. The report lists
per-leaf mismatches, changes and steps true, and how often each selection branch was taken.

## Phase 5f: altitude target and output packing

The altitude target (`alt_cstr_applicable` / `alt_sel_or_cstr`) is computed by the readable vertical logic
(`readable/vertical_alt.cpp`, signals `AltCstrApplicable` and `AltSelOrCstr`) and compared by `check_vertical` like
the other signals.

`readable/output_packing.{h,cpp}`: everything from the stage results to `FmgcComputer_Y.out`: one function per FMGC
A/B bus discrete word (bit tables in the comments; ARINC bit numbers, 1-based), the value words with their SSM rules,
the B bus, the discrete outputs and the assembly. Spec, word/bit tables, consumers and findings: `docs/output-buses.md`.

```sh
build/fmgc-check-output-stock [--fuzz SEED] [--approach SEED] [--athr SEED] [--lateral SEED] [--engagement SEED] \
  [--targets SEED] file.fdr
JOBS=4 scripts/check-output-corpus.sh [runs per file]
```

The packing has no state, so there is no resync/free mode: every running frame, `assemble()` gets the generated
stage results and every field of `fmgc_outputs` is compared bit-exact (SSMs as integers, Data by bit pattern); also
checked: `out.data` = the step's inputs, and what the end-of-step delays hold. The report lists per-word SSM counts and
per-bit steps/rises; the corpus script lists the bits never set (only spares) over all runs.

## Phase 5g: the readable computer

`readable/fmgc.{h,cpp}`: `fmgc::readable::FmgcComputer` chains every readable stage into one `initialize()` /
`step(const fmgc_inputs&)` with the generated model's interface, on its own state (no heap, no static state; restart
semantics as generated: `computer_running` false holds the outputs, the next running frame resets everything but 8
block-output memories). Glue written for it: TCAS RA decoding and flags (`readable/tcas.{h,cpp}`), the longitudinal law
selection (`readable/law_selection.{h,cpp}`), the outer loops' input bus (`readable/outer_loop_inputs.{h,cpp}`) and
the ap_fd_logic bus assembly. Stage graph, state inventory, findings, notes for fuzzing: `docs/integration.md`.

```sh
build/fmgc-check-fmgc-stock [--fuzz SEED] [--approach SEED] [--athr SEED] [--lateral SEED] [--engagement SEED] \
  [--targets SEED] [--failures SEED] [--power SEED] [--warm FILE] [--time] file.fdr
JOBS=4 scripts/check-fmgc-corpus.sh [seeds per file]   # every scenario alone, in pairs, all but one, all
```

Both computers run in lockstep on the same inputs, free-running (the readable one never sees a generated value), and
the whole `fmgc_outputs` (1017 leaves) is compared bit-exact every frame, running or not. The first mismatch is
reported with the differing fields, the earliest pipeline stage they belong to and the first divergence of the 65
hidden memories compared alongside (every mode latch, the engagement / targets / A/THR latches, the memories that
survive a restart). `--power SEED` adds FMGC power episodes (`src/scenario_power.h`: off for one frame to a minute,
flicker, opposite FMGC lost), `--warm FILE` checks that `initialize()` is a complete reset, `--time` reports the step
time of both computers only.

## Phase 4: coverage, fuzzing, property checks

```sh
JOBS=4 scripts/build-fuzz.sh                  # clang tools into build/cmake-clang (+ gcc build/fmgc-drive-stock)
JOBS=4 SEEDS=2 scripts/coverage.sh replay     # corpus x every scenario injector -> build/coverage/replay
WORKERS=4 TIME=3600 scripts/fuzz.sh --coverage   # libFuzzer campaign -> build/fuzz, coverage of its corpus
```

- **Coverage** (`docs/coverage.md`): clang source-based coverage of `FmgcComputer.cpp` and `FmgcOuterLoops.cpp` only
  (`-fprofile-instr-generate -fcoverage-mapping`, `-O2 -ffp-contract=off`). `build/fmgc-cov-drive` is the corpus
  driver (`src/drive.cpp`: replay + any of `--fuzz --approach --athr --lateral --engagement --targets --failures
  --tcas --situation SEED`) linked against the instrumented model; `build/fmgc-cov-fuzz-run` runs fuzz inputs through
  it. Each result dir under `build/coverage/` has `report.txt` (lines/regions/branches per file), `html/`,
  `uncovered.txt` (`scripts/coverage_report.py`: branch sides never taken, grouped by the subsystems of
  `docs/structure.md`) and, for the replay, `props.txt` (the property table over every run).
- **Fuzzer** (`src/fuzz_fmgc.cpp`, `build/fmgc-fuzz`): an input (`src/fuzz_case.h`) picks a snapshot of the bank
  (`src/fuzz_bank.h`, built by `build/fmgc-fuzz-bank` from the corpus: model state every 120 s of each recording + the
  next 30 s of replay inputs), restores the model state (one 10 KB copy, no re-replay) and runs the recorded inputs with
  timed events on top: FCU knobs/buttons/values, AP/FD/A/THR buttons and disconnects, FMS constraint/phase/speed/flags,
  SSM failures, discrete flips, any-field mutations, noise, power cycles, levers, radio height, on-ground, opposite
  FMGC, engines, TCAS, ILS, sim flags, dt changes, and whole scenario injectors (`src/scenario_*.h`) for a duration.
  Coverage feedback from the model only (`-fsanitize=fuzzer-no-link` on the two generated files) plus model-state
  counters (mode transitions/pairs, armed sets, laws, A/THR FMA, property bad states). `build/fmgc-fuzz-run` is the same
  code without libFuzzer: `run [--trace] [--summary]`, `minimize CHECK IN OUT`, `seeds DIR`, `info`.
- **Oracles** (`src/fuzz_oracle.h`, every step): the behavioural properties (`src/properties.h`, `docs/properties.md`),
  the stateless differential check of the readable output packing, the resync differential check of the readable
  A/THR, and the whole-computer differential against the readable computer (below).
  Property violations are findings about stock behaviour (reproducers in `build/fuzz/findings`); differential
  mismatches abort (`FMGC_FUZZ_ABORT=diff`, default) so libFuzzer keeps the input.
- **Corpus property check**: `JOBS=4 scripts/properties-corpus.sh` runs the gcc driver with the oracles over the corpus
  and every scenario config (`build/properties/props.txt`: per property frames checked, violations, first reproducer
  as recording + flags + time; replay it with `build/fmgc-drive-stock FLAGS --trace T0 T1 file.fdr`).
- **Reproducers**: `build/fmgc-fuzz-run run --trace FILE` replays a fuzz input with a mode timeline;
  `build/fmgc-fuzz-run minimize CHECK IN OUT` shrinks it (drops events, cuts the run after the first hit, halves
  durations, time scale 1). Every byte string is a valid input and runs are deterministic, so a `.bin` plus the bank
  file is a complete reproducer (the bank is rebuilt identically from the same corpus and build).

Input encoding (`src/fuzz_case.h`): 8-byte header (snapshot index, start offset in the snapshot's window, length
32–2072 steps, time scale 1–4 window frames per step, seed) followed by 8-byte event records (gap to the previous
event, duration, kind, three parameters). Fixed-size records keep libFuzzer's byte mutations meaningful (a flipped byte
changes one parameter of one event; insertions shift whole events). Reset strategy: the bank file is mmap'ed shared by
all workers; a run restores the 10 KB model state with one `memcpy` (the generated class has no pointers) and reads the
window's inputs in place, so there is no per-input replay from frame 0 and no allocation beyond the injector objects.

Whole-computer differential (`WholeComputerOracle`, `FMGC_WHOLE_COMPUTER=1`): the readable computer
(`readable/fmgc.{h,cpp}`, linked as `fmgc_readable_<variant>`) runs free on the same inputs; every leaf of
`fmgc_outputs` and the hidden memories `check_fmgc` compares are checked by bit pattern every step, running or not. The
bank stores the readable computer's bytes with each snapshot (warmed up alongside the generated model from frame 0;
rebuild the bank with `scripts/fuzz.sh --rebuild-bank` after changing either computer), and `fmgc-fuzz-bank` checks
that a generated model restored from snapshot bytes steps exactly like the replayed one (output leaves, 2000 steps per
snapshot). The corpus driver runs it from frame 0 of every corpus run. Inputs are sanitised to the domain both models
are defined on (`sanitizeInputs` in `src/fuzz_case.h`: bools 0/1, enums in range, finite floats, 0 < dt ≤ 1 s).
Minimise a set of reproducers with `scripts/minimize-findings.sh [FINDINGS_DIR] [OUT_DIR]`. libFuzzer's time box
is wall-clock time: if the host suspends the WSL VM, the suspension counts as fuzzing time and the inputs running at
that moment are saved as `timeout-` / `slow-unit-` artifacts (check `uptime` against the log before chasing them).

Results (coverage numbers, unreached branches by subsystem, campaigns, property status and reproducers):
`docs/coverage.md`, `docs/properties.md`.

## FCU: replay, readable FCU, differential check

The generated A32NX FCU (`FcuComputer`) gets the same treatment: a native build per model variant
(`fcu_model_<variant>`; the FCU model is identical in every variant so far), a replay of the personal recording format
3209001 against the recorded FCU outputs, a readable rewrite in `readable/fcu/` (namespace `fcu::readable`) that is
bit-exact against the generated code, and a differential checker. Spec, inputs, outputs, memories, quirks, FCOM checks:
`docs/fcu.md`; findings U1–U11 in `docs/findings.md` section 7; parameters `docs/params-FcuComputer.txt`.

```sh
TARGETS='fcu-check-stock fcu-replay-stock fcu-synth-fdr-stock' JOBS=4 scripts/build.sh   # just the FCU tools
build/fcu-replay-stock [--from N] [--to N] [--sync] file.fdr      # 3209001: generated FCU vs recorded outputs
build/fcu-check-stock [--synthetic] [--fmgc2 opp|copy|invalid|ncd|swapped] [--reconstruct] [--script SEED] \
  [--panel SEED] [--priority SEED] [--bus SEED] [--sim SEED] [--power SEED] [--warm FILE] file.fdr
JOBS=4 [SEEDS=2] scripts/check-fcu-corpus.sh                     # every recording x configs, then fcu-replay on 3209001
FCU_ONLY=1 scripts/build-fuzz.sh && COVERAGE=1 JOBS=4 scripts/check-fcu-corpus.sh   # + clang coverage
JOBS=4 scripts/mutate-fcu.sh                                     # mutation test of the checker
build/fcu-synth-fdr-stock [--script SEED] [--corrupt FRAME] [--from FRAME] in.fdr out.fdr   # make a 3209001 file
```

- `fcu-replay-<variant>` (`src/fcu_replay.cpp`): steps the generated FCU with each 3209001 frame's recorded FCU inputs
  and compares logic, discrete and bus outputs field by field, bit-exact (counts, first frame and values per field,
  mismatch episodes, how many follow an active-pause gap). On a 3200006 file it says there is no FCU data and exits 0.
  `--sync`: for a file that starts with the FCU already running, the generated memories are first set from frame 0's
  recorded outputs (approximate for dash timers and pulses in progress; converges once they expire).
- `fcu-check-<variant>` (`src/check_fcu.cpp`): generated and readable FCU in lockstep, free-running, every leaf of
  `fcu_outputs` (357) and 30 hidden memories bit-exact each step; per-field mismatch counts and the first mismatch; exit
  1 on any mismatch. Inputs (`src/fcu_drive.h`): the recorded FCU inputs of a 3209001 file, or a synthetic drive from
  any recording: FMGC 1's previous-frame A bus and discretes, FMGC 2's bus and discretes as FMGC 1 received them
  (`--fmgc2` variants), the recorded time; `--reconstruct` infers the panel actions from the recorded FCU bus (knob
  pushes/pulls and pbs from the pulse bits, EFIS selectors from the EIS words, turns/presses to follow the recorded
  values: selected values then match the recorded bus on >99.8 % of frames, printed as `fidelity` lines). Seeded
  injectors (`src/fcu_scenarios.h`): scripted tour of every control, random panel actions, AP/FD/A/THR priority
  episodes, FMGC bus episodes (random SSM/bit/value on every word the FCU reads, end of TCAS into V/S, level-off push,
  SRS/TCAS on FMGC 1, Mach mode, managed/selected, presets), sim flags/dt/time steps, power drops.
- `fcu-synth-fdr-<variant>` (`src/fcu_synth_fdr.cpp`): writes a 3209001 file from any recording with the generated FCU
  (synthetic drive + reconstruction), to exercise the 3209001 paths before real recordings exist.

Results (2026-11-15, 25 recordings, all 3200006): `check-fcu-corpus.sh` 650 runs, 87.4 M steps (85.0 M running),
14,692 restarts, 0 mismatches; step time generated 0.42 us, readable 0.21 us. Coverage of `FcuComputer.cpp`: 98.2 % of
branches (542/552; the 10 missing sides are unreachable), 99.1 % of lines; readable FCU 96.7-100 % of branches. Mutation test: 371 of 383 mutants caught,
the 12 survivors equivalent or near-equivalent (`docs/fcu.md`). `fcu-replay` on synthesized 3209001 files: 0
mismatches over 59,510 frames; a corrupted output field is reported. No real 3209001 recording yet: when one exists,
`build/fcu-replay-stock <file>` proves the harness reproduces the sim, and `check-fcu-corpus.sh` adds recorded-input
runs for it automatically.

## FADEC: replay, readable FADEC, differential check

The generated A32NX FADEC (`FadecComputer`, two instances) the same way: a native build per model variant
(`fadec_model_<variant>`; identical in every variant so far), a replay of the personal recording format 3209004 (both
FADECs' model inputs and outputs, branch personal/fdr-fadec), a readable rewrite in `readable/fadec/` (namespace
`fadec::readable`) that is bit-exact against the generated code, and a differential checker. Spec, inputs, outputs,
memories, FCOM checks: `docs/fadec.md`; findings E1-E9 in `docs/findings.md` section 8; parameters
`docs/params-FadecComputer.txt`.

```sh
TARGETS='fadec-check-stock fadec-replay-stock fadec-synth-fdr-stock' JOBS=4 scripts/build.sh   # just the FADEC tools
build/fadec-replay-stock [--from N] [--to N] [--sync] file.fdr    # 3209004: generated FADECs vs recorded outputs
build/fadec-check-stock [--synthetic] [--script SEED] [--lever SEED] [--athr SEED] [--sim SEED] [--warm FILE] \
  [--trace T0 T1] file.fdr
JOBS=4 [SEEDS=2] scripts/check-fadec-corpus.sh                   # every recording x configs, then fadec-replay on 3209004
FADEC_ONLY=1 scripts/build-fuzz.sh && COVERAGE=1 JOBS=4 scripts/check-fadec-corpus.sh   # + clang coverage
JOBS=4 scripts/mutate-fadec.sh                                   # mutation test of the checker
build/fadec-synth-fdr-stock [--script SEED] [--athr SEED] [--corrupt FRAME] in.fdr out.fdr   # make a 3209004 file
```

- `fadec-replay-<variant>` (`src/fadec_replay.cpp`): steps both generated FADECs with each 3209004 frame's recorded
  inputs and compares `data_computed`, `output` and the ECU bus of both, and FADEC 1's stock record part, bit-exact. On an
  older file it says there is no FADEC model data and exits 0.
- `fadec-check-<variant>` (`src/check_fadec.cpp`): generated and readable FADECs (engine 1 and 2) in lockstep,
  free-running, every leaf of `athr_out` (112 per FADEC) and 17 hidden memories per FADEC bit-exact each step; exit 1 on
  any mismatch. Inputs (`src/fadec_drive.h`): the recorded inputs of a 3209004 file, or a synthetic drive from any
  recording (TLA, N1, on ground, engine running, instinctive disconnect and FCU bus from the record; thrust limits
  learned from FADEC 1's recorded outputs; commanded N1 from a static plant; `fidelity` lines compare generated FADEC 1
  with the recorded one). Seeded injectors (`src/fadec_scenarios.h`): scripted tour, random lever, A/THR and sim
  episodes. Also prints event counts (control lost by cause, thrust lock, A/THR disabled) and the recorded thrust lock
  episodes.
- `fadec-synth-fdr-<variant>` (`src/fadec_synth_fdr.cpp`): writes a 3209004 file from any recording with the generated
  FADECs, to exercise the 3209004 paths before real recordings exist.

Results (2026-11-17, 29 recordings: 25 x 3200006, one each 3209001/3209002, two 3209003): `check-fadec-corpus.sh` 436
runs, 62.9 M steps x 2 FADECs, 0 mismatches; step time generated 0.075 us, readable 0.050 us. Coverage of
`FadecComputer.cpp`: 94.8 % of branches (182/192; the 10 missing sides are unreachable), 97.4 % of lines; readable FADEC
100 % of branches but one unreachable side. MUTATION_SUMMARY `fadec-replay` on synthesized 3209004 files: 0 mismatches over
494,560 frames; a corrupted output field is reported. No real 3209004 recording yet: when one exists,
`build/fadec-replay-stock <file>` proves the harness reproduces the sim, and `check-fadec-corpus.sh` adds recorded-input
runs for it automatically.
