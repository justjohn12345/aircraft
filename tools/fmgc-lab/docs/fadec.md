# The A32NX FADEC (generated `FadecComputer`) and its readable rewrite

The FADEC model turns a thrust lever angle (TLA) into an N1 command and moves the sim's throttle until the sim engine
follows it. It also keeps the FADEC side of the A/THR: whether the FADEC follows the FMGC's N1 command, the 15 s
instinctive disconnect lock-out and the thrust lock (THR LK). It is small (476 generated lines): three SR flip-flops,
one FLX latch, two timers, two edge detectors, a frozen N1 and three integrators. There is no engine model.

- Generated: `fbw-a32nx/src/wasm/fbw_a320/src/model/FadecComputer.cpp` (+ `.h`, `_data.cpp`, `_types.h`). Never edited.
  Identical in every lab variant so far.
- Readable: `tools/fmgc-lab/readable/fadec/` (namespace `fadec::readable`), bit-exact (below).
- Sim wiring: `FlyByWireInterface::updateFadec()` (~2626), two instances (`fadecs[2]`, engine 1 and 2).
- Parameters (block name = value): `docs/params-FadecComputer.txt`.
- Findings: `docs/findings.md`, section 8 (E1-E9).

## Where it runs

Every frame, after the FCU, the two FMGCs, the ELACs, SECs, FACs and FCDCs, `updateFadec(i)` runs for FADEC 1 then 2.
So the FADEC sees **this** frame's FCU bus, whose A/THR words are the priority FMGC's of the previous frame (the FCU
passes them through), and the FMGCs see the FADEC buses of the previous frame. The model has no power or health logic
and is stepped every frame; its `initialize()` is never called (it would only set what the constructor already zeroes).

After FADEC 2, the wrapper sends both `sim_throttle_lever_pos` (limited to 99.9999999999999) and `sim_thrust_mode` to the
sim, sets `L:A32NX_AUTOTHRUST_DISABLED` = either FADEC's `ATHR_disabled`, and the L:vars `N1_TLA`, `REVERSE`, `N1_c` per
engine and `THRUST_LIMIT_TYPE` / `THRUST_LIMIT` from **FADEC 1 only** (EWD). ECU status word 3 and maintenance word 6
of each FADEC go to `L:A32NX_ECU_x_STATUS_WORD_3` / `_MAINTENANCE_WORD_6` (the PFD FMA shows THR LK from either word 6).

## Structure (generated line ranges -> readable files)

| Generated | What | Readable |
|---|---|---|
| 6-91 | library blocks: SSM == NO, A429ValueOrDefault, TimeSinceCondition, BitfromLabel, bit packing, PulseNode | `fadec_blocks.h` (+ `readable/blocks.h`) |
| 114-118 | A/THR disabled after a 15 s instinctive disconnect | `athr_interface.cpp` |
| 119-124 | A/THR engaged (ATS bit 13 and N1 command valid), engage edge | `athr_interface.cpp` |
| 125-134 | FLX valid, FLX latch, FLX active | `thrust_lever.cpp` `FlexSelection` |
| 135-137 | time on ground, TLA in active range | `fadec.cpp` |
| 138-145 | A/THR control active, loss edge | `athr_interface.cpp` |
| 146-150 | thrust lock | `athr_interface.cpp` |
| 151-192 | N1 for the lever (N1_TLA), reverse | `thrust_lever.cpp` `n1ForLever` |
| 193-246 | thrust limit type and value | `thrust_lever.cpp` `thrustLimit` |
| 247-281 | N1 command (A/THR, thrust lock, manual) | `n1_control.cpp` `command` |
| 282-323 | trim and throttle integrators | `n1_control.cpp` `throttle` |
| 325-414 | ECU bus | `output_packing.cpp` |
| 415-452 | `out.data_computed`, `out.output`, sim thrust mode | `fadec.cpp`, `thrust_lever.cpp` `simThrustMode` |
| 455-463 | `initialize()` | `fadec.cpp` |

`readable/fadec/fadec.h`: `fadec::readable::FadecComputer` with `initialize()` and `const athr_out& step(const athr_in&)`,
no heap, no static state, copyable (state accessors for the checker). Its `initialize()` is a complete reset (checked
with `fadec-check --warm`); the generated one is not (the "not yet initialised" flags of the timers, edge detectors and
pU stay set), so the checker re-constructs the generated instance.

## Inputs (`athr_in`)

| Field | Sim source (`updateFadec`) | Used for |
|---|---|---|
| `time.dt` | frame sample time | the integrators |
| `time.simulation_time` | sim time | the 15 s disconnect timer, time on ground (they stand still in pause) |
| `data.on_ground` | LGCIU i left and right main gear compressed | reverse, thrust limit, FLX, alpha floor command |
| `data.is_engine_operative` | `ENG COMBUSTION` i | thrust limit on the ground |
| `data.engine_N1_percent` | `TURB ENG N1` i | frozen N1 (thrust lock), trim integrator, bus |
| `data.commanded_engine_N1_percent` | `TURB ENG COMMANDED N1` + N1 - corrected N1 | throttle integrators |
| `data.engine_N2_percent` | constant 0 | bus only |
| `data.TAT_degC` | TAT | FLX valid (flex temperature > TAT) |
| `data.V_*`, `alpha_deg`, `H_*`, `flap_handle_index`, `OAT_degC` (`ISA_degC`, density: not set) | sim | unused (copied to `out.data`) |
| `input.TLA_deg` | `L:A32NX_AUTOTHRUST_TLA:i` (ThrottleAxisMapping: detents give exactly -20, -6, 0, 25, 35, 45) | everything |
| `input.thrust_limit_IDLE/CLB/MCT/FLEX/TOGA_percent` | `L:A32NX_AUTOTHRUST_THRUST_LIMIT_*` (TS) | rating curve, thrust limit |
| `input.thrust_limit_REV_percent` | TOGA x `THRUST_LIMIT_REVERSE_PERCENTAGE_TOGA` (ini, 0.813) | reverse curve |
| `input.ATHR_disconnect` | `A32NX.FCU_ATHR_DISCONNECT_PUSH` / `AUTO_THROTTLE_DISCONNECT` (one frame) or `L:A32NX_AUTOTHRUST_DISCONNECT` (held by the cockpit button) | 15 s timer, control active reset |
| `input.ATHR_reset_disable` | `A32NX.ATHR_RESET_DISABLE` (sent by the MCDU FMS reset when disabled) | resets A/THR disabled |
| `input.is_anti_ice_active`, `is_air_conditioning_active` | engine 1 anti-ice and pack 1 for both FADECs | unused |
| `input.tracking_mode_on_override` | not set | unused |
| `fcu_input` | the FCU bus of this frame | ATS discrete word bits 13, 14, 23; N1 command; flex temperature |

### What the FADEC reads from the FCU bus

| Word | Bits / SSM | Meaning (FMGC side, `readable/output_packing.cpp`) | FADEC use |
|---|---|---|---|
| ATS discrete word | 13 (SSM ignored) | A/THR engaged | engaged; control active reset when 0; alpha floor command |
| ATS discrete word | 14 | A/THR active | A/THR command only when set |
| ATS discrete word | 23 | alpha floor (incl. TOGA LK) | thrust limit TOGA in flight; command range; status word 3 bit 21 |
| N1 command | SSM NO, data | FMGC `athr.n1_c_percent` | part of "engaged"; the command (0 when not NO) |
| FLEX TO temperature | SSM NO, data | MCDU FLX TO temperature | FLX valid when NO and > TAT; echoed on the ECU bus |

## Outputs

### `out.output` and `out.data_computed`

| Field | Value |
|---|---|
| `sim_throttle_lever_pos` | forward integrator (0..100) outside reverse, reverse integrator (-100..0) in reverse |
| `sim_thrust_mode` | from the raw TLA: 1 < 0, 2 = 0, 3 (0, 25), 4 [25, 35), 5 [35, 45), 6 = 45, 0 above 45 or NaN |
| `N1_TLA_percent` | N1 for the lever (below) |
| `is_in_reverse` | TLA < 0 on the ground (never in flight) |
| `thrust_limit_type`, `thrust_limit_percent` | below |
| `N1_c_percent` | N1 command (below) |
| `athr_control_active` | the FADEC follows the FMGC (SR flip-flop) |
| `memo_thrust_active` | thrust lock (SR flip-flop) |
| `TLA_in_active_range` | TLA <= 35 |
| `is_FLX_active` | below |
| `ATHR_disabled` | 15 s lock-out latch |
| `time_since_touchdown` | seconds continuously on the ground (0 in flight); nobody reads it |

`out.time`, `out.data`, `out.input`, `out.fcu_input` are copies of the inputs.

### ECU bus (`fadec_bus_output`)

| Word | SSM | Data |
|---|---|---|
| selected TLA | NO | TLA |
| N1 ref | NO | N1 for the lever (the FMGC's A/THR upper limit: max of both FADECs) |
| selected flex temp | NO when FLX active, else NCD | FCU flex temperature (0 when not NO) |
| ECU status words 1, 2 | NO | 0 |
| ECU status word 3 | NO | 15 limit REVERSE, 19 A/THR control active (FMGC: "A/THR engaged in the ECU"), 21 alpha floor, 22 TOGA, 23 FLEX, 24 MCT, 25 CLB |
| N1 limit | NO | thrust limit |
| N1 maximum | NO | TOGA limit (the FMGC's alpha floor command) |
| N1 command | NO | N1_c |
| selected N2 / N1 actual | NO | N2 (0) / N1 |
| ECU maintenance word 6 | NO | 12 thrust lock |

## Thrust lever to thrust

**N1 for the lever** (151-192). Piecewise linear between the limits at the detents: TLA 0-25 IDLE -> CLB, 25-35 CLB ->
FLX (FLX active) or MCT, above 35 FLX/MCT -> TOGA (not limited at 45). Reverse, on the ground only: |TLA| 6 -> 20 maps
|IDLE + 1| -> |REV|, |TLA| below 6 counts as 6. In flight the TLA is limited to >= 0, so there is no reverse.

**Thrust limit** (193-246), the EWD limit (FADEC 1's):
- in flight, or on the ground with the engine not running: alpha floor -> TOGA; TLA > 35 TOGA; > 25 MCT (or FLEX);
  >= 0 CLB; < 0 REVERSE;
- on the ground, engine running: TOGA, or FLEX when FLX is active and TLA <= 35; REVERSE below 0.
- NaN TLA: NONE, 0 %.

**FLX** (125-134). Valid = FCU flex temperature word NO and above TAT. On the ground FLX is active whenever valid. A latch
is set on the ground when valid with the lever exactly at 35, and cleared with the lever exactly at 25 or 45; in flight
FLX is active while the latch is set.

## A/THR interface

- **Engaged** = ATS bit 13 and the N1 command word NO. Its rising edge (not on the first frame) sets **control active**.
- **Control active** is reset (reset wins) by the instinctive disconnect input, by A/THR disabled, or ATS bit 13 false.
  ATS bit 14 (A/THR active) does not reset it; it only gates the command.
- **A/THR disabled**: the instinctive disconnect input continuously true for >= 15 s of simulation time sets it; only
  `ATHR_reset_disable` resets it. While set, control active cannot be set. The FMGC has its own 15 s latch (engagement,
  `athrInhibited`, never reset) and drops A/THR 0.9 s after an ECU stops reporting status word 3 bit 19.
- **Thrust lock**: set when control active is lost (rising edge of the flip-flop's second output) with the lever in the
  CL band (24-26 deg) or the FLX/MCT band (34-36 deg); reset (reset wins) when the lever leaves both bands or control
  becomes active again. Any loss counts: an instinctive disconnect too (finding E1).

**N1 command** (247-281):
- A/THR in control: lever <= 35 deg or alpha floor, ATS bit 14, engaged, control active. The FMGC's command is limited
  to [IDLE, N1 for the lever]; in alpha floor in flight to [N1 for the lever, TOGA]. The limit that applies is rounded
  to float.
- else thrust lock: the actual N1 of the frame before the lock, held.
- else: N1 for the lever.

**Throttle** (282-323). A trim integrator (0.3 /s, +-2 %) on N1_c - N1, reset to 0 when the error exceeds 0.8 % (then
its input is 0 too); then the forward integrator (5 /s, 0..100) or the reverse one (-5 /s, -100..0) on
N1_c + trim - commanded N1. Each is held at 0 while the other one is used.

## Power-up

Everything at 0 / false. The PulseNodes start with "previous input = true", so an A/THR already engaged on the first
frame does not count as an engage edge. pU (frozen N1) takes the first frame's N1. The timers start at the first
frame's time; an event time of exactly 0 counts as unset (a condition true since time 0 starts counting at the first
non-zero time).

## Hidden state (what the checker compares besides the outputs)

Per FADEC: `Memory_PreviousInput` (disabled), `_p` (control active), `_j` (thrust lock), `latch` (FLX), `pU` and
`pU_not_empty`, `Delay_DSTATE` / `_n` / `_l` (trim, forward, reverse), both TimeSinceCondition event times and flags,
both PulseNode previous inputs and flags: 17 memories, 34 for the pair.

## Verification

- `fadec-check-<variant>` (src/check_fadec.cpp): both generated and readable FADECs in lockstep, free-running, every leaf
  of `athr_out` (112 per FADEC) and the 17 hidden memories bit-exact every step, plus the power-up outputs and the three
  flip-flop tables against the readable formula. Inputs (src/fadec_drive.h): 3209004 recordings (recorded inputs), or
  the synthetic drive from any recording (TLA, N1, on ground, engine running, disconnect input and FCU bus from the
  record; thrust limits learned from FADEC 1's recorded outputs; commanded N1 from a static plant on the throttle
  output). Seeded injectors `--script --lever --athr --sim` (src/fadec_scenarios.h).
- Synthetic drive realism (`fidelity` lines: generated FADEC 1 vs the recorded FADEC 1, 29 plain runs, mean / lowest
  file): A/THR control active, thrust lock, reverse, sim thrust mode, N1 maximum 100 %; thrust limit type and value
  94 % / 17 % (lowest in short cold-start files with the engines off); N1 ref 99.2 % / 95.4 %;
  N1_TLA 90 % / 66 % (the IDLE limit is only learned at TLA 0, and flight idle changes with altitude); N1_c 87.5 % /
  70 %; throttle 20 % (the plant is not the sim engine).
- `scripts/check-fadec-corpus.sh` (JOBS=4, SEEDS=2): 436 runs over the 29 recordings, 62.9 M steps x 2 FADECs,
  **0 mismatches**. Mean step time: generated 0.07-0.08 us, readable 0.05 us (gcc -O2).
- Coverage (`COVERAGE=1`, clang source-based, same runs): `FadecComputer.cpp` 94.8 % of branches (182/192), 97.4 % of
  lines. The 10 branch sides never taken are unreachable: BitfromLabel's left-shift and range branches (constant bit
  numbers 13, 14, 23: 4 sides), the falling-edge branch of the PulseNode (both instances are rising: 5 sides), and
  "ATS bit 13 false" inside the A/THR command branch (implied by "engaged": 1 side). Readable FADEC: 100 % of branches
  except the same bit 13 side in `n1_control.cpp`. The functions not covered are the checker's state accessors.
- Mutation test (`scripts/mutate-fadec.sh`, 171 mutants of `readable/fadec/`): 148 caught, the 23 survivors
  equivalent (below).
- `fadec-replay-<variant>` against recorded outputs: no 3209004 recording exists yet; checked on synthesized files
  (`fadec-synth-fdr`): 0 mismatches over 494,560 frames, and a single corrupted output field is reported. `--sync`
  is for a file that starts with the FADECs running (approximate for the trim integrator and a held disconnect).

### Mutation test

`scripts/mutate-fadec.sh` (`scripts/mutate-fcu.py` with `UNIT=fadec`): 171 mutants of `readable/fadec/` (&& <-> ||,
! dropped, == <-> !=, < <-> <=, > <-> >=, true <-> false, += -> -=, every numeric literal x1.5 (0 -> 1), every ARINC bit
number +1, including the ECU bus packing; `fadec_blocks.h` included). 148 caught. The first run left 27; 4 were
killed by scenarios added for them: N1 actual exactly 0.8 % from N1_c (trim reset `>` vs `>=`), TAT equal to the flex
temperature, limits far apart at exactly 35 deg (segment end rounding), and the 15 s hold reached exactly (simulation
time in 1/8 s steps). The 23 survivors are equivalent:

- reset and default values that are never read (11): `engaged_` in `reset()`; TimeSinceCondition's default event time /
  flag and `reset()` values (a reset "primed" flag is equivalent because an event time of 0 re-primes it); the
  flip-flop's default `q` / `qbar` and `reset()` `qbar` (recomputed by every update); the integrator's default state; the
  frozen N1's reset value and flag (the lock cannot be set on the first frame, so the first frame always loads N1);
- a limit hit exactly gives the same value (4): the integrator limits `>=` / `<=`, the A/THR command limits `>=` / `<=`
  (the float command equals the float-rounded limit);
- defaults never used (2): the N1 command's A429 default (read only when the word is NO) and the flex temperature
  default in "FLX valid" (behind the SSM test);
- values that cannot reach the mutated test (6): `fmax(1, TLA)` in flight (0-25 deg is CLB either way), the four
  `TLA < 0 ? REVERSE : NONE` variants (only negative or NaN TLA get there), and `TLA >= 0` for sim thrust mode 3
  (0 returns 2 before).

### Rerun

```sh
TARGETS='fadec-check-stock fadec-replay-stock fadec-synth-fdr-stock fadec-dump-params-stock' JOBS=4 tools/fmgc-lab/scripts/build.sh
JOBS=4 tools/fmgc-lab/scripts/check-fadec-corpus.sh               # every recording, 3209004 ones also replayed
FADEC_ONLY=1 tools/fmgc-lab/scripts/build-fuzz.sh && COVERAGE=1 JOBS=4 tools/fmgc-lab/scripts/check-fadec-corpus.sh
tools/fmgc-lab/build/fadec-replay-stock ~/fdr-corpus/raw/<3209004 file>.fdr      # harness vs sim
tools/fmgc-lab/build/fadec-check-stock --script 1 --athr 1 ~/fdr-corpus/raw/<file>.fdr
JOBS=4 tools/fmgc-lab/scripts/mutate-fadec.sh
```

## FCOM checks (A319/A320/A321 FCOM, 02 MAY 23)

Matches: rating limit of the higher detent between detents (DSC-70-30-30); FLX on the ground only when the FLX TO
temperature is above TAT, MCT otherwise (DSC-70-30-40: the N1 at the FLX/MCT detent is MCT; the displayed limit is TOGA);
after takeoff FLX changes to MCT only after the lever went to TOGA or CL, "setting the thrust lever out of FLX/MCT detent
without reaching TOGA or CL detent has no effect" (but see E4); in A/THR the thrust is "limited to the value
corresponding to the thrust lever position (unless the alpha-floor mode is activated)" (DSC-70-30-40); thrust lock holds
"the thrust at the value set before the A/THR disconnection" and is released by moving the levers
(DSC-22_30-50-40, with a +-1 deg band); 15 s instinctive disconnect disables A/THR including A.FLOOR (DSC-22_30-50-40).

Differences and doubtful points are findings E1-E9 (`docs/findings.md` section 8).
