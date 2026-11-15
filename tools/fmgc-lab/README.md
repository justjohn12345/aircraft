# fmgc-lab

Native (no MSFS) build of the generated A32NX FMGC (`FmgcComputer` + `FmgcOuterLoops`) with a flight data recorder
replay harness. Phase 0 of the FMGC reverse-engineering plan (`~/fmgc-lab/PLAN.md`).

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
