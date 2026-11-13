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
| `cst_athr` | `test/combined` | + A/THR SPEED in FINAL patch |

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
