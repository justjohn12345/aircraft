# Shadow mode: FMGC (Phase 6a), FCU and FADEC, readable FMGC primary (Phase 6b)

The readable FMGC (`readable/fmgc.{h,cpp}`, `fmgc::readable::FmgcComputer`) runs in the sim alongside the generated
`FmgcComputer`, on exactly the inputs the generated one got, every frame, for FMGC 1 and FMGC 2. Every output leaf is
compared by bit pattern. A frame where anything differs is counted and logged so it can be reproduced offline. By
default the aircraft only ever uses the generated outputs: the readable outputs are compared, never read by anything
else.

The readable FCU and the readable FADECs have the same shadow mode (own ini switches, logs and L:vars), see
[FCU and FADEC shadow mode](#fcu-and-fadec-shadow-mode). With `[FMGC] READABLE_PRIMARY` the roles of the two FMGCs are
swapped: the readable FMGC flies and the generated one is the shadow, see
[Readable FMGC primary (Phase 6b)](#readable-fmgc-primary-phase-6b). Everything up to those sections describes the FMGC
shadow; the FCU and FADEC shadows follow the same design.

## Files

| File | What |
|---|---|
| `fbw-a32nx/src/wasm/fbw_a320/src/fmgc/FmgcShadow.{h,cpp}` | `FmgcShadow` (one per FMGC: owns a readable computer, compares, counts, keeps episode details) and `FmgcShadowLog` (the log file and console lines). Plain C++, no MSFS SDK: the lab builds the same file natively. |
| `fbw-a32nx/src/wasm/fbw_a320/src/fmgc/FmgcOutputLeaves.gen.h` | Generated, committed: every scalar leaf of `fmgc_outputs` (1017: name, offset, size, kind) as a constexpr table, plus `static_assert(sizeof(fmgc_outputs) == 3808)`. |
| `fbw-a32nx/src/wasm/fbw_a320/src/fmgc/Fmgc.{h,cpp}` | `enableShadow(readablePrimary)`, and after the generated step: `if (shadow) shadow->step(modelInputs.in, fmgcComputer.getExternalOutputs().out);` (plus the readable-primary swap, Phase 6b) |
| `fbw-a32nx/src/wasm/fbw_a320/src/FlyByWireInterface.{h,cpp}` | ini switch, L:vars, the per-frame log update after the flight data recorder, the summary in `disconnect()`. |
| `fbw-a32nx/src/wasm/fbw_a320/src/recording/FlightDataRecorder.{h,cpp}` | Three getters (current file name, records in it, records in the session) so a log record can name its `.fdr` file and record index. What is recorded is unchanged. |
| `fbw-a32nx/src/wasm/fbw_a320/build.sh` | Compiles `FmgcShadow.cpp` and `tools/fmgc-lab/readable/*.cpp` (all but `*_test.cpp`) into the module. |
| `igniter.config.mjs` | `tools/fmgc-lab/readable` added to the `systems-fbw` task's inputs, so a change there rebuilds `fbw.wasm`. |
| `tools/fmgc-lab/scripts/gen_leaf_table.py` | Generator of the leaf table (`--check`: compare only). |
| `tools/fmgc-lab/src/shadow_test.cpp` | Native (and wasm) driver and test of `FmgcShadow` from recordings: `build/fmgc-shadow-test-<variant>`. |
| `tools/fmgc-lab/scripts/check-shadow-corpus.sh` | Every recording, both units, plain and with power episodes, plus the self-test. |
| `tools/fmgc-lab/scripts/build-shadow-wasm.sh`, `check-shadow-wasm-corpus.sh`, `wasi-run.mjs` | The same test as a wasm32-wasi module built with the MSFS sysroot and `fbw.wasm`'s flags, run with node's WASI. |

## Build integration

One copy of the code: `fbw_a320/build.sh` compiles the readable sources straight from `tools/fmgc-lab/readable`, in a
second `clang++` call with its own object folder (`obj/fmgc-shadow/`) and include paths (`src/model` and
`tools/fmgc-lab`, nothing else):

- `readable/fmgc.cpp` and `fmgc/Fmgc.cpp` would both become `fmgc.o` on a case-insensitive file system (Windows
  checkouts mounted into the dev container), and `#include "fmgc.h"` from `src/fmgc/` could resolve to `Fmgc.h`.
  `FmgcShadow.cpp` therefore includes `"readable/fmgc.h"`, and `FmgcShadow.h` only forward-declares the readable
  computer, so `Fmgc.cpp` and `FlyByWireInterface.cpp` need no new include path.
- Same flags as the main compile (`-flto -O2`, wasm32-wasi, `-fno-exceptions`, no fast-math), plus `-ffp-contract=off`
  as in the lab. wasm32 has no scalar fused multiply-add, so contraction cannot happen for either computer anyway; the
  flag documents the requirement and keeps it if the target ever gains FMA.
- The source list is `readable/*.cpp` minus `*_test.cpp`. The lab's CMake fails if `READABLE_COMPUTER_SOURCES` (the
  library every lab check links) differs from that glob, so both builds compile the same files.
- The wasm module's real build is `fbw_a320/build.sh` (run by `npm run build-a32nx:fbw`, igniter task
  `a32nx/wasm/systems-fbw`); `fbw_a320/CMakeLists.txt` is only for IDEs (it lists the new sources too).

**Leaf table: committed generated header, not build-time generation.** The table is derived from
`model/FmgcComputer_types.h`, which only changes when the Simulink model is regenerated (rare, and done outside this
build). Committing it keeps the wasm build a pure compile with no Python step, identical on Linux, in the container and
on Windows (`run.cmd`), and makes the table reviewable in diffs. Staleness cannot go unnoticed:

1. `static_assert(sizeof(fmgc_outputs) == 3808)` (the layout is computed by the generator) and the `offsetof()` of every
   leaf fail the wasm compile when a field is added, removed, renamed or retyped;
2. the lab build runs `gen_leaf_table.py --check` (target `fmgc-shadow-leaves-check`, part of `all`) and fails if the
   committed file is not exactly what the generator produces from the repo's types header;
3. the self-test compares the table with the lab's independent leaf list (`fields.gen.h`): names, offsets, sizes, kinds.

Regenerate (from the repository root):

```sh
python3 tools/fmgc-lab/scripts/gen_leaf_table.py fbw-a32nx/src/wasm/fbw_a320/src/model/FmgcComputer_types.h \
    fbw-a32nx/src/wasm/fbw_a320/src/fmgc/FmgcOutputLeaves.gen.h
```

## Switch

`\work\ModelConfiguration.ini` (the package's work folder, where the `.fdr` files are):

```ini
[FMGC]
SHADOW_ENABLED = true
```

Default `false`. The console prints `WASM: FMGC      : SHADOW_ENABLED = 0|1` at load.

- **Off**: no shadow object exists (`Fmgc::shadow` is null), no L:var is registered, nothing is logged. The generated
  path is the stock one; the only differences are one null-pointer test per FMGC step and per frame, and the module
  containing the (unused) readable code.
- **On**: `loadConfiguration()` (called from `connect()`, before the first FMGC step) creates one `FmgcShadow` per FMGC,
  so both computers start from power-up together. The generated outputs are copied to the aircraft exactly as before;
  the shadow only reads them. An FMGC replaced by client data (`[MODEL] FMGC_DISABLED`) is not stepped, so its shadow
  does not run either.

## What a frame does

`Fmgc::update()` sets `computer_running`, steps the generated model, copies its outputs, then
`shadow->step(modelInputs.in, generated outputs)`:

1. counts the frame (running frames, restarts, frames with the `sim_data` inputs the recorder cannot record:
   `slew_on`, `pause_on`, `tracking_mode_on_override`, `tailstrike_protection_on`);
2. steps the readable computer with the same `fmgc_inputs`;
3. compares the two `fmgc_outputs` (3808 bytes) as 476 64-bit words XOR-ed under a mask of the bytes that belong to a
   leaf (padding excluded), branch-free;
4. only if a bit differs: walks the 1017 leaves, counts each differing one, classifies it (`value`; `nan-payload`: both
   NaN, different bits; `zero-sign`: +0 vs -0) and, on the first frame of an episode, keeps up to 24 fields with both
   values and the frame's time and `sim_data`.

No allocation, no I/O, no clock in `step()`. Memory is fixed when the shadow is created: `FmgcShadow` 32 KB (episode
details, per-leaf counters) plus the readable computer 9.4 KB, per FMGC, on the heap.

Once per frame, after the flight data recorder, `FlyByWireInterface::updateFmgcShadow()` passes the recorder's current
file and this frame's record index to `FmgcShadowLog::update()`, which writes new episode records (if any, so almost
never) and updates `L:A32NX_FMGC_1_SHADOW_MISMATCHES` / `L:A32NX_FMGC_2_SHADOW_MISMATCHES` (mismatching frames, written
only when they change). `disconnect()` writes the summary.

## After a mismatch: free-running, counted in episodes

Both computers keep running on their own state; nothing is re-initialised or re-synced. An **episode** is a run of
consecutive mismatching frames; it ends at the first frame where all outputs match again. The first 16 episodes per
FMGC keep their details (start record with fields, values and inputs; end record with the length); later ones are
counted. Every leaf keeps its count of mismatching frames and its first frame.

Why not the alternatives:

- **Re-initialise the readable computer** after a mismatch: it would restart (every latch, filter and mode reset) while
  the generated one carries on in flight, so it would mismatch for much longer than the original difference and could
  never re-converge on its own. Useless.
- **Re-sync it from the generated state** (as the lab's `src/*_resync.h` do per stage): needs every hidden memory of
  the generated model, which is private (the lab reads it with `#define private public`, not something to ship), and the
  resync headers cover only some stages (A/THR, engagement, targets), not the whole computer. It would also make the
  sim's behaviour after the first mismatch differ from an offline replay, which is free-running.
- **Free-running** (chosen) is exactly what `check_fmgc`, the fuzzer's whole-computer oracle and `fmgc-shadow-test` do,
  so the first mismatch of a session is the first mismatch of an offline replay of that session. The cost: after a
  divergence that sticks (a latch set differently), later independent differences are hidden inside the same episode
  and only the per-leaf counters grow. In practice most differences heal (filters converge, modes change, a restart
  resets almost everything), which the episode records show, and the first mismatch is the one to chase anyway: fix
  it and the rest re-runs offline.

## Log

**Where**: the work folder, next to the recordings, one file per session named after the session's first recording:
`2026-11-15-10-00-00.fdr` → `2026-11-15-10-00-00.fmgc-shadow.log`. With the recorder off, the UTC time of the first
record (`<utc>.fmgc-shadow.log`). The recorder's clean-up only deletes `*.fdr`; shadow logs stay (a few kB each). The
file is opened in append mode only to write a record and closed again, so a crash loses at most the summary.

**Console**: at a unit's first mismatch

```
WASM: FMGC 1 SHADOW: first mismatch at frame 8595, sim time 127.721 s: 1 output fields differ (ap_fd_logic.active_lateral_law); fdr=2026-11-08-01-12-39.fdr fdr_record=8594 utc=2026-11-15-15-29-27; log \work\2026-11-08-01-12-39.fmgc-shadow.log
```

and at the end of the session `WASM: FMGC SHADOW: FMGC 1 14324 frames, 0 mismatching (0 episodes); FMGC 2 ...; log ...`.

**Format** (version 1): one record per line, `KEY name=value ...`; indented lines belong to the record above.

```
# FMGC shadow log: readable FMGC (tools/fmgc-lab/readable) vs generated FmgcComputer, ...
FMGC_SHADOW_LOG version=1 leaves=1017 output_bytes=3808 session_fdr=2026-11-08-01-12-39.fdr created_utc=2026-11-15-15-29-27 primary=generated
MISMATCH unit=1 episode=1 frame=8595 sim_time=127.721136 running=1 fields=1 listed=1 fdr=2026-11-08-01-12-39.fdr fdr_record=8594 utc=2026-11-15-15-29-27
  FIELD name=ap_fd_logic.active_lateral_law kind=enum class=value generated=1 readable=0 generated_bits=0x00000001 readable_bits=0x00000000
  INPUTS dt=0.011694099754095078 simulation_time=127.72113616939168 monotonic_time=110.24372700508684 slew_on=0 pause_on=0 tracking_mode_on_override=0 tailstrike_protection_on=0 computer_running=1
END unit=1 episode=1 frame=8596 length=1 fdr=2026-11-08-01-12-39.fdr fdr_record=8595 utc=2026-11-15-15-29-27
SESSION_END utc=2026-11-15-15-29-27
SUMMARY unit=1 frames=14324 running_frames=9974 restarts=2 mismatch_frames=1 episodes=1 episodes_logged=1 longest_episode=1 in_episode=0 first_mismatch_frame=8595 first_mismatch_sim_time=127.721136 nan_only_frames=0 zero_sign_leaf_frames=0
UNRECORDED unit=1 slew_on_frames=0 pause_on_frames=0 tracking_mode_on_override_frames=0 tailstrike_protection_on_frames=0
  COUNT unit=1 name=ap_fd_logic.active_lateral_law frames=1 first_frame=8595 first_sim_time=127.721136
```

| Record | Fields |
|---|---|
| `MISMATCH` | first frame of an episode: `unit` (FMGC 1/2), `episode` (1-based), `frame` (FMGC steps since the module loaded, 1-based), `sim_time` (`time.simulation_time` = the recording's `base.simulation_time_s` of that frame), `running` (`computer_running`), `fields` (differing leaves), `listed` (≤ 24 listed below), `fdr` / `fdr_record` (the recording and the 0-based record index of this frame in it; `none` if the frame was not recorded: recorder off, active pause), `utc` |
| `FIELD` | leaf name (as in `fields.gen.h` / `check_fmgc`), `kind` (bool, float, int, enum), `class` (`value`, `nan-payload`, `zero-sign`), both values (`%.17g` doubles, `%.9g` floats) and bit patterns |
| `INPUTS` | the frame's `time` and `sim_data`: the inputs a recording does not hold |
| `END` | first matching frame after the episode and its `length` in frames |
| `SUMMARY` | per unit at the end: frames, running frames, restarts, mismatching frames, episodes (all / logged), longest episode, still in an episode, first mismatch, frames where only NaN payloads differed, leaf-frames of ±0 differences |
| `UNRECORDED` | frames with `slew_on`, `pause_on`, `tracking_mode_on_override`, `tailstrike_protection_on` set: an offline replay assumes all four false |
| `COUNT` | per leaf that ever differed: mismatching frames, first frame and sim time |

## Reproducing a logged mismatch offline

1. Copy the recording named in the `MISMATCH` record (`fdr=`) from the work folder into the corpus (the sim keeps only
   15 recordings). If the episode started on an unrecorded frame (`fdr_record=none`), use the recording of the
   surrounding records; the replay fills unrecorded stretches (active pause) by holding the last inputs.
2. Build the lab (`tools/fmgc-lab/scripts/build.sh stock=HEAD` on the branch the sim build came from) and run the sim's
   own checker on the recording:

   ```sh
   build/fmgc-shadow-test-stock --unit 1 --log /tmp/replay.log ~/fdr-corpus/raw/2026-11-15-10-00-00.fdr
   ```

   It writes the same log format: compare `sim_time` and the `FIELD` lines with the sim's log. Frame numbers differ
   (the sim counts from module load, the replay from the file's first record, plus gap-fill steps); `sim_time` is exact.
3. Diagnose with the whole-computer checker, which adds the stage the first differing field belongs to and the first
   divergence of the 65 hidden memories (usually before the output):

   ```sh
   build/fmgc-check-fmgc-stock ~/fdr-corpus/raw/2026-11-15-10-00-00.fdr      # FIRST MISMATCH at step N, t = ...
   ```

4. If the native replay is clean, replay it in wasm (MSFS libc/libm, `fbw.wasm`'s flags):

   ```sh
   tools/fmgc-lab/scripts/build-shadow-wasm.sh
   CORPUS=/dir/with/that/file tools/fmgc-lab/scripts/check-shadow-wasm-corpus.sh
   ```

Telling a rewrite bug from the rest:

| Native replay | wasm replay | Meaning |
|---|---|---|
| same mismatch | — | **Rewrite bug** (or a stock behaviour the rewrite missed) on an input pattern the corpus and fuzzing had not reached: debug with `check_fmgc`, add the recording to the corpus. |
| clean | same mismatch | **Toolchain**: the two computers compute the same thing differently in wasm only (see below). |
| clean | clean | **Inputs not in the recording**: `UNRECORDED` counts or the `INPUTS` line show `slew_on` / `tracking_mode_on_override` / `tailstrike_protection_on`, the episode is on FMGC 2, the session started before the recording (recorder switched on later, file rotation), or the shadow was not enabled from power-up. Or the sim runtime: `class=nan-payload` only (`nan_only_frames`). |

FMGC 2 is not recorded (the recorder writes FMGC 1 only). `--unit 2` replays FMGC 1's inputs with `is_unit_1` false,
which runs the unit-2 paths in both computers but is not FMGC 2's real input history; an FMGC 2-only mismatch needs a
recording of FMGC 2's inputs (an FDR interface change, not done here).

Only meaningful against the generated model the readable one reproduces: **stock** (upstream master). On a build whose
generated model carries FMGC fixes (`test/combined`, from `personal/fmgc-fixes`), the shadow reports every frame where a
fix changes an output; those are expected, not rewrite bugs (replay with that variant's model to confirm).

## wasm versus native

What was checked offline is native x86-64 (gcc and clang, `-O2 -ffp-contract=off`, glibc libm). In the sim both
computers run in the same wasm module, with the same libm and compiler, so a native-versus-wasm difference matters
only if it treats the two computers differently. Known candidates and their status:

- **libm**: wasi-libc's libm (musl-based) is not correctly rounded everywhere, glibc's mostly is. Both computers call
  the same functions at runtime, so this alone gives the same result in both; it is why the sim's recorded outputs
  and a native replay differ by ~1e-6, which does not affect the lockstep comparison.
- **Compile-time evaluation**: clang folds libm calls with constant arguments using the host's correctly rounded
  arithmetic. If one computer computes `f(constant)` at runtime and the other has it folded, wasm can differ by an ulp
  where glibc (correctly rounded) cannot. Covered by the wasm corpus run below; any remaining case shows as
  "native clean, wasm mismatch".
- **FMA contraction**: none on wasm32 (no scalar FMA instruction; `llvm.fmuladd` lowers to multiply + add).
- **float/double**: wasm has native f32/f64 (FLT_EVAL_METHOD 0, as x86-64 SSE); no extended precision.
- **LTO, wasm-opt -O1**: neither changes IEEE results without fast-math; binaryen may fold NaN payloads differently.
- **NaN payloads**: the wasm spec leaves NaN sign/payload of arithmetic results non-deterministic, and two different
  operation orders producing NaN may give different bits. Flagged as `class=nan-payload` / `nan_only_frames`.
- **Stack**: `fbw.wasm` has wasm-ld's default 64 KiB stack (no guard: overflow silently overwrites data). Measured
  maximum depth of the test harness in wasm: 5856 bytes, the same with and without the shadow step (the readable step
  is shallower than the deepest path already there).
- **Non-0/1 booleans** (`boolean_T` is `unsigned char`): a sim input of 2 could take different branches in the two
  computers (the fuzzer sanitises bools to 0/1). Inputs are recorded byte for byte, so such a case replays exactly.

**wasm corpus check** (`check-shadow-wasm-corpus.sh`): the test module built with the MSFS sysroot and the module's
flags, every recording, both units, plain and with power episodes: 12.3 M frames, 0 mismatches.

## Cost

Per frame and per FMGC, single thread, the 22 recordings (3.08 M frames), mean of two runs:

| | generated step | readable step | comparison | shadow total |
|---|---|---|---|---|
| native (gcc -O2) | 1.89 µs | 1.78 µs | 0.15 µs | 1.93 µs |
| wasm (MSFS sysroot, `-flto -O2`, node 26 / V8) | 1.87 µs | 2.03 µs | 0.21 µs | 2.23 µs |

So the shadow roughly doubles the FMGC cost: about 4.5 µs per frame for both FMGCs, 0.27 ms per second at 60 fps
(0.03 % of a core). The sim's wasm runtime is not V8; allow a factor of 2 either way. The first-mismatch path (leaf walk,
up to 24 copies) costs a few µs once per episode; writing a record opens and closes the log file (once per episode).
`fbw.wasm` grows from 1 225 640 to 1 388 631 bytes (+163 kB, +13.3 %; the leaf names are 48 kB of it).

## Tests

```sh
JOBS=8 tools/fmgc-lab/scripts/build.sh stock=HEAD          # builds build/fmgc-shadow-test-stock, checks the leaf table
JOBS=8 POWER_SEEDS=2 tools/fmgc-lab/scripts/check-shadow-corpus.sh
tools/fmgc-lab/scripts/build-shadow-wasm.sh && JOBS=8 tools/fmgc-lab/scripts/check-shadow-wasm-corpus.sh
```

Results (2026-11-15, 22 recordings):

- native: 44 plain runs (both units), 6.15 M frames, 0 mismatching frames; 88 runs with power episodes (2 seeds,
  8664 restarts), 12.3 M frames, 0 mismatching frames;
- wasm: 44 plain runs and 44 power runs, 12.3 M frames, 0 mismatching frames;
- self-test (native and wasm): the leaf table matches `fields.gen.h`; clean runs of both units report nothing; one bit
  flipped on one frame in a float (`pitch_fd_command.Data`), a bool (`ap_own_engaged`), an enum (`active_lateral_law`)
  and an SSM (`discrete_word_1.SSM`) is reported as exactly one mismatching frame and one episode, with the `MISMATCH`,
  `FIELD`, `INPUTS`, `END`, `SUMMARY` and `COUNT` lines; the readable `dt` × 1.5 on one frame is reported from that
  frame on and reaches computed outputs (flight director, flare law, FMGC A bus pitch command).

Rerun 2026-11-18 (30 recordings, `POWER_SEEDS=1`, with the FCU/FADEC and readable-primary changes): FMGC native 60 plain
runs and 60 power runs, 8.72 M frames each, 0 mismatching; wasm the same (8.72 M plain + 8.72 M power frames), stack
depth unchanged at 5856 bytes; both self-tests pass. Cost per FMGC and frame on this corpus: native generated 2.28 µs,
shadow 2.28 µs (readable 2.04 + comparison 0.24); wasm (V8) generated 2.13 µs, shadow 2.48 µs.

## FCU and FADEC shadow mode

The readable FCU (`readable/fcu/`, `fcu::readable::FcuComputer`, one unit) and the readable FADECs
(`readable/fadec/`, `fadec::readable::FadecComputer`, engine 1 and 2) shadow the generated `FcuComputer` and the two
`FadecComputer`s exactly as the readable FMGC shadows the generated one: stepped after the generated step on the same
inputs, every output leaf compared by bit pattern, free-running after a mismatch, mismatches grouped in episodes and
logged, memory fixed at construction, no allocation or I/O in `step()`. The aircraft only uses the generated outputs.

### Files

| File | What |
|---|---|
| `fbw_a320/src/shadow/ShadowChecker.{h,cpp}` | `shadow::ShadowChecker`: the comparison core of FmgcShadow made independent of the output type (leaf table, output size and whether the inputs have `base_sim_data` are constructor arguments; per-leaf arrays allocated once). `shadow::ShadowLog`: the log file and console lines for one computer type. FmgcShadow is unchanged and keeps its own copy of this logic (porting it onto the core would change nothing it does, but was left out to keep the FMGC shadow exactly as tested). |
| `fbw_a320/src/shadow/FcuShadow.{h,cpp}`, `FadecShadow.{h,cpp}` | Own the readable computer, build the frame's `FrameInputs`, call the checker; `makeLog()`. |
| `fbw_a320/src/shadow/FcuOutputLeaves.gen.h`, `FadecOutputLeaves.gen.h` | Generated, committed: the 357 leaves of `fcu_outputs` (1104 bytes) and the 112 leaves of `athr_out` (576 bytes), `gen_leaf_table.py --root fcu_outputs --namespace fcu_shadow` / `--root athr_out --namespace fadec_shadow` (shared `shadow::Leaf` type). The lab build checks both (target `unit-shadow-leaves-check`). |
| `fbw_a320/src/fcu/Fcu.{h,cpp}` | `enableShadow()`, and after the generated step `if (shadow) shadow->step(modelInputs.in, fcuComputer.getExternalOutputs().out);` (before the wrapper sets the FCU 1/2 healthy bits on its copy of the bus). |
| `fbw_a320/src/FlyByWireInterface.{h,cpp}` | `[FCU]` / `[FADEC] SHADOW_ENABLED`, the two `FadecShadow`s, after each generated FADEC step `fadecShadows[i]->step(fadecInputs[i].in, fadecs[i].getExternalOutputs().out)`, L:vars, `updateShadows()` (was `updateFmgcShadow()`: one FDR record index per frame for all three logs), summaries in `disconnect()`. |
| `fbw_a320/build.sh` | Two more compiles into their own object folders (`obj/fcu-shadow`, `obj/fadec-shadow`): `readable/`, `readable/fcu/` and `readable/fadec/` each have an `output_packing.cpp`. `tools/fmgc-lab/readable` is on their include path (the FCU/FADEC blocks include the FMGC's `blocks.h`). The three shadow compiles share one flag array; the flags are the ones the FMGC shadow compile had. The lab's CMake fails if `READABLE_FCU_SOURCES` / `READABLE_FADEC_SOURCES` differ from the `readable/fcu/*.cpp` / `readable/fadec/*.cpp` globs build.sh compiles. |
| `tools/fmgc-lab/src/fcu_shadow_test.cpp`, `fadec_shadow_test.cpp`, `shadow_unit_test.h` | Native (and wasm) drivers and self-tests: `build/fcu-shadow-test-<variant>`, `build/fadec-shadow-test-<variant>`. |

### Switches, outputs

```ini
[FCU]
SHADOW_ENABLED = true
[FADEC]
SHADOW_ENABLED = true
```

Both default `false` (console `WASM: FCU       : SHADOW_ENABLED = 0|1`, same for FADEC); off, no shadow object, L:var or
log exists. On, `loadConfiguration()` creates them before the first step, so both computers start from power-up:

- FCU: the readable FCU is `initialize()`d like the generated one (`Fcu::Fcu()`). `computer_running` comes from the
  wrapper's FCU 1/2 self-monitoring as for the generated FCU. With `[MODEL] FCU_DISABLED` the FCU is not stepped, nor its
  shadow.
- FADEC: the wrapper never calls the generated `initialize()` (it only constructs `FadecComputer`); every initial
  condition is 0, so a constructed generated FADEC equals the readable one after its `initialize()` (`docs/fadec.md`).
  A FADEC replaced by client data (`[MODEL] FADEC_DISABLED`) is not stepped, nor its shadow.
- L:vars `A32NX_FCU_SHADOW_MISMATCHES`, `A32NX_FADEC_1_SHADOW_MISMATCHES`, `A32NX_FADEC_2_SHADOW_MISMATCHES`
  (mismatching frames).
- Logs `<session fdr name>.fcu-shadow.log`, `<session fdr name>.fadec-shadow.log` in the work folder, the FMGC log's
  format with header `FCU_SHADOW_LOG` / `FADEC_SHADOW_LOG` (version 1, `leaves`, `output_bytes`), `unit=1` for the FCU,
  `unit=1|2` = engine for the FADECs. The FADEC inputs have no `monotonic_time` and no `base_sim_data`, so FADEC
  `MISMATCH` records have no `running=`, `INPUTS` has `dt` and `simulation_time` only, `SUMMARY` has no
  `running_frames`/`restarts` and there is no `UNRECORDED` line. Console: `WASM: FCU SHADOW: first mismatch ...`,
  `WASM: FADEC 1 SHADOW: ...`.

### Reproducing offline

Recordings with the FCU record (interface 3209001 and later) hold the FCU model inputs of each frame; 3209004 holds both
FADECs' model inputs. The shadow tests replay them (`src/fcu_drive.h`, `src/fadec_drive.h`); on older recordings they
use the lab's synthetic drive, which tests the checkers but is not the sim's input history.

```sh
build/fcu-shadow-test-stock [--log FILE] [--time] [--synthetic] [--power SEED] file.fdr...
build/fadec-shadow-test-stock [--log FILE] [--time] [--synthetic] file.fdr...
build/fcu-shadow-test-stock --self-test [--log-dir DIR] file.fdr        # same for fadec-shadow-test
build/fcu-check-stock file.fdr      # then the differential checker: first diverging hidden memory
```

The self-tests: the committed leaf table equals the lab's leaf list (`fcu_fields.gen.h` / `fadec_fields.gen.h`); a clean
run reports nothing; one bit flipped on one frame in a float, a bool, an enum and an SSM (FCU
`bus_outputs.selected_alt_ft.Data`, `discrete_outputs.afs_outputs.ap_1_light_on`, `logic.capt_efis.efis_filter`,
`bus_outputs.fcu_discrete_word_1.SSM`; FADEC 1 `output.N1_c_percent`, `output.athr_control_active`,
`output.thrust_limit_type`, `fadec_bus_output.ecu_status_word_3.SSM`, FADEC 2 untouched) is exactly one mismatching
frame and one episode with all log lines; a perturbed readable input on one frame (FCU: ALT knob +3 turns, FADEC: dt x
1.5) is reported from that frame on and reaches computed outputs (FCU: `logic.afs.alt_display_value`, ...; FADEC:
`output.sim_throttle_lever_pos`). The ALT knob replaces the dt perturbation for the FCU because dt only reaches FCU
outputs while a timer runs.

### Results and cost (2026-11-18, 30 recordings)

- FCU: 30 plain runs, 4.36 M frames, 0 mismatching frames (5 recordings with recorded FCU inputs: 3209001, 3209002,
  2 x 3209003, 3209004; 25 synthetic), and 30 runs with power episodes (`--power 1`), 4.36 M frames, 0 mismatching.
- FADEC: 30 runs x 2 engines, 8.72 M FADEC-frames, 0 mismatching (recorded inputs on the 3209004 recording, synthetic on
  the others).
- Both self-tests pass, native and wasm.
- wasm (`build-shadow-wasm.sh`, MSFS sysroot, the module's flags; `check-shadow-wasm-corpus.sh`, node 26 / V8): the same
  FCU (plain and power) and FADEC runs, 4.36 M + 4.36 M FCU frames and 8.72 M FADEC-frames, 0 mismatching.

Per unit and frame, native (gcc -O2), mean over the corpus:

| | generated step | readable step | comparison | shadow total |
|---|---|---|---|---|
| FCU | 0.45 µs | 0.16 µs | 0.13 µs | 0.30 µs |
| FADEC (per engine) | 0.073 µs | 0.053 µs | 0.021 µs | 0.074 µs |
| FCU, wasm (V8) | 0.39 µs | 0.31 µs | 0.15 µs | 0.45 µs |
| FADEC, wasm (V8) | 0.11 µs | 0.088 µs | 0.043 µs | 0.13 µs |

So about 0.45 µs per frame for the FCU and both FADECs together, a quarter of one FMGC shadow. The comparison is a
loop over a runtime-sized mask (the FMGC's is compile-time sized), which is why the FCU comparison costs nearly as much
as its readable step; still well under a microsecond. Memory, allocated once: FCU shadow about 16 KB (16 episode
records of 504 bytes, 357 x 20 bytes of per-leaf counters, the 1104-byte mask) plus the readable FCU (1.8 KB); FADEC
shadow about 11 KB per engine plus the readable FADEC (0.7 KB). `fbw.wasm` (after `wasm-opt -O1`, as the build
produces it): 1 441 628 bytes, +53 kB on the FMGC-shadow build's 1 388 631 (the leaf names are about 19 kB of it).

## Readable FMGC primary (Phase 6b)

```ini
[FMGC]
READABLE_PRIMARY = true
```

Default `false`. When `true`, the readable FMGC's outputs are the ones the aircraft uses and the generated
`FmgcComputer` is the shadow:

- `READABLE_PRIMARY` implies the FMGC shadow (`SHADOW_ENABLED` is forced on: the readable FMGC lives in `FmgcShadow`).
  Console at load: `SHADOW_ENABLED = 1`, `READABLE_PRIMARY = 1`.
- `Fmgc::update()` is unchanged up to the shadow step: power supply monitoring, self-test, `computer_running`, the
  generated step, `modelOutputs` from the generated outputs, `shadow->step()` (readable step + comparison, same
  episodes and log). Then, only with `READABLE_PRIMARY`, `modelOutputs = shadow->readableOutputs();`.
  `getBusOutputs()` / `getDiscreteOutputs()` (what the other computers and the FCU get) read `modelOutputs` as before,
  and `getDebugOutputs()` (the flight data recorder, the autoland debug L:vars) returns the readable outputs. So
  restart, `computer_running`, self-test and initialisation semantics are exactly today's: both computers are created
  and initialised at the same points and step on the same inputs every frame; only which of the two output structs is
  read changes.
- `READABLE_PRIMARY = false`: the swap is a not-taken branch; outputs, recording and behaviour are as before. The FMGC
  log header gains `primary=generated|readable` (an added key; format version still 1) and the end-of-session console
  line says `FMGC SHADOW (readable primary)` when the readable FMGC flies.
- Why both still run: the generated FMGC as shadow keeps proving the swap invisible in flight; a mismatch is logged the
  same way (the `FIELD` lines still say `generated=` / `readable=`). After a mismatch the aircraft keeps flying on the
  readable FMGC (free-running, no fallback to the generated one): a shadow mismatch with the readable FMGC primary means
  the aircraft is now flying behaviour the stock build would not, which the log pinpoints.

Test (`fmgc-shadow-test --primary-test [--unit 1|2] [--power SEED] [--log FILE] file.fdr`, built when the lab's model
variant is the repository's own `FmgcComputer`, since it compiles the sim wrapper `fmgc/Fmgc.cpp`): three copies of the
wrapper on a recording's inputs (generated primary without and with the shadow, readable primary), the wrapper's power
input following the replayed `computer_running` (with `--power`, the power episodes of `src/scenario_power.h`, so
outages, self-tests and restarts run); every frame their `getBusOutputs()`, `getDiscreteOutputs()` and
`getDebugOutputs()` must be bit-identical, both shadows report 0 mismatches, the debug outputs of the readable-primary
wrapper must be the readable computer's (address check, so the test cannot pass with the swap unwired) and the log
header says `primary=readable`. Result (2026-11-18): 120 runs (30 recordings x FMGC 1/2 x plain/`--power 1`), 17.4 M
frames (16.1 M healthy, 6774 health changes), 0 differing frames.
