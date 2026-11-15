# FMGC output buses (output packing)

Generated: `build/models/stock/FmgcComputer.cpp` (= upstream master) 4642–4925 (SSM decisions and discrete words),
4926–5106 (assembly of `FmgcComputer_Y.out`), 5107–5128 (end-of-step delays). Readable: `readable/output_packing.{h,cpp}`
(one function per word, bit tables in the comments). Check: `src/check_output.cpp`, `scripts/check-output-corpus.sh`.

## Conventions

- An ARINC word is `{SSM, Data}`: SSM a `SignStatusMatrix` value as uint32 (0 FW, 1 NCD, 2 FT, 3 NO), Data a float32.
- **Bit numbers are ARINC bits, 1-based**, as every consumer reads them: `Arinc429Utils::bitFromValue` is
  `(uint32)Data >> (bit − 1)`, the TS `Arinc429Word.bitValue` the same, and the FMGC's own `BitfromLabel`
  (`MATLABFunction_i`) shifts by `bit − 1`. The packer `MATLABFunction_gy` ORs element i of a 19-bool vector at shift
  i + 10, so **element i = ARINC bit i + 11**, and every discrete word uses exactly the ARINC data field, bits 11–29
  (1–8 label, 9–10 SDI, 30–31 SSM, 32 parity are never set). PLAN.md's "bits 10–28" are shifts.
- Data of a discrete word = `(float)bits`; exact (19-bit span).
- Every discrete word has SSM NO, always. Value words: SSM NO unless a rule is given below.
- Label numbers: not in the code (the bus structs name fields, not labels) and not in any consumer; the names in
  `fbw-a32nx/docs/a320-simvars.md` and `A32NXFGBusPublisher.ts` (shown as "doc:" below where they differ) look like
  Airbus label definitions but carry no label numbers.
- The packing runs only when the computer runs; otherwise `FmgcComputer_Y.out` is left as it was (last running frame,
  or `out_Y0` before the first). The wasm wrapper (`fmgc/Fmgc.cpp` 132–143) replaces the whole bus by zeros (SSM FW)
  when its monitoring is not healthy, except `discrete_word_5.SSM` = FT/FW during self test.
- The packing has **no state**: every field is a function of this frame's inputs, logic bus, ap_fd_logic bus,
  outer-loop commands and A/THR outputs.

## FMGC A bus (`bus_outputs.fmgc_a_bus`)

To the opposite FMGC (its `fmgc_opp_bus`), the FCU (`fmgc_1/2_bus`), and through L:vars `A32NX_FMGC_n_*`
(FlyByWireInterface 1864–1882) to the PFD/FMA, MCDU, LNAV and FWS.

### Discrete word 1: vertical modes

| bit | meaning | readers |
|---|---|---|
| 11 | climb: CLB ∨ OP CLB ∨ EXP CLB | shim, FCU, MCDU, FMA, opp FMGC |
| 12 | descent: DES ∨ OP DES ∨ EXP DES | same |
| 13 | spare, 0 (doc: "immediate mode") | FCU reads it (no effect) |
| 14 | open: OP CLB ∨ OP DES | shim, FCU, MCDU, opp |
| 15 | SRS takeoff | shim, FCU (FMGC1 only), FAC (low-energy inhibit), opp |
| 16 | SRS go-around | shim, FCU, FAC, opp |
| 17 | V/S | shim, FCU, MCDU, opp |
| 18 | FPA | same |
| 19 | ALT*  ∨ ALT ("ALT mode") | shim, FCU (ALT CST), opp |
| 20 | ALT ∨ G/S ("track mode") | shim, opp |
| 21 | ALT* ∨ G/S* ("capture mode") | shim, opp |
| 22 | G/S* ∨ G/S | shim, FCU, PFD, FWS, opp |
| 23 | FINAL DES | shim, FCU, FMA, FWS, opp |
| 24 | EXPED: EXP CLB ∨ EXP DES | shim, FCU, opp |
| 25 | FLARE | shim, FMA |
| 26 | FMA dash display (ALT CRZ) | shim, FMA |
| 27 | MCDU preset Mach > 0 ∨ preset speed > 0 ("SPEED/MACH preset display") | nobody |
| 28 | spare, 0 | |
| 29 | TOGA lever set for go-around (`tla_to_ga_set`) | FAC (pitch-pitch warning inhibit); not in a320-simvars.md |

### Discrete word 2: lateral modes

| bit | meaning | bit | meaning |
|---|---|---|---|
| 11 | RWY ∨ RWY TRK (`rwy_active`) | 20 | RWY LOC submode |
| 12 | NAV | 21 | spare (doc: "H/PATH submode") |
| 13 | LOC* | 22 | spare (doc: "HDG submode") |
| 14 | LOC | 23 | RWY TRK submode |
| 15 | GA TRK | 24 | spare (doc: "VOR submode") |
| 16 | HDG | 25 | ALIGN submode |
| 17 | TRK | 26 | ROLL OUT submode |
| 18, 19 | spare | 27 | spare |
| | | 28 | roll FD bars flashing |
| | | 29 | spare |

The shim decodes RWY = 11 ∧ 20, RWY TRK = 11 ∧ 23 (FlyByWireInterface 1923–1928).

### Discrete word 3: armed modes

| bit | meaning | bit | meaning |
|---|---|---|---|
| 11 | HDG/TRK preset available | 20 | LAND armed |
| 12 | ALT armed | 21 | spare (doc: "G/S capt before LOC capt") |
| 13 | ALT arm possible (read by nobody) | 22 | G/S armed |
| 14 | NAV armed | 23 | FINAL DES armed |
| 15 | spare | 24 | CLB armed |
| 16 | LOC armed | 25 | DES armed |
| 17 | spare (doc: "FG approach phase") | 26 | spare (doc: "long mode reset") |
| 18 | spare (doc: "FMA longi mode"; FMA.tsx reads it as mode reversion) | 27 | spare (doc: "lat mode reset") |
| 19 | LOC backbeam selected (FMS input) | 28 | spare (doc: "QFU equal COM") |
| | | 29 | spare (doc: "FMA longi box") |

### Discrete word 4: AP/FD engagement and landing capability

| bit | meaning | bit | meaning |
|---|---|---|---|
| 11 | AP instinctive disconnect pushbutton (input) | 21 | LAND 3 FAIL PASSIVE inop |
| 12 | AP own engaged | 22 | LAND 3 FAIL OP inop |
| 13 | FD own engaged | 23 | LAND 2 capacity |
| 14 | LAND | 24 | LAND 3 FAIL PASSIVE capacity |
| 15 | spare | 25 | LAND 3 FAIL OP capacity |
| 16 | LAND 2 capability | 26 | ILS tune inhibit (doc: "RWY heading memorized") |
| 17 | LAND 3 FAIL PASSIVE capability | 27 | FD auto disengage |
| 18 | LAND 3 FAIL OP capability | 28 | AP/FD mode reversion (triple click) |
| 19 | AP inop | 29 | V/S target not held (speed protection) |
| 20 | LAND 2 inop | | |

### Discrete word 5: speed control (A bus only)

| bit | meaning |
|---|---|
| 19 | managed speed (`auto_spd_control_active`) |
| 20 | selected speed (`manual_spd_control_active`) |
| 24 | pitch FD bars flashing |
| 28 | MCDU preset speed/Mach activated (FMS input `preset_spd_mach_activate`; doc: "top of SPD synchro") |
| others | spare (doc lists 11 Mach selected, 12–16 DES/V/S/FPA/ALT submodes, 27 SPD window display, 29 FMS Mach selection) |

### Discrete word 6 (A bus only)

All 19 bits spare (constant 0), SSM NO. Nobody reads it.

### Discrete word 7: TCAS (A bus only)

| bit | meaning |
|---|---|
| 11 | AP/FD TCAS mode installed: constant 1 |
| 12 | TCAS mode armed |
| 13 | TCAS mode active (read by the opposite FMGC for TCAS sync, the FCU, the shim) |
| 18 | TCAS large box on the FMA (`longi_large_box_tcas`) |
| 24 | TCAS RA inhibited message |
| 25 | TRK FPA deselected message |
| 27 | TCAS RA corrective |
| 28 | `tcas_vs_target > 0` (doc: "RA nose up") |
| 29 | `tcas_vs_target < 0` (doc: "RA nose down") |
| others | spare (doc: 14 TCAS mode fault, 20 SOFT GA inop) |

### ATS discrete word (A and B bus)

| bit | meaning | bit | meaning |
|---|---|---|---|
| 13 | A/THR own engaged | 20 | RETARD |
| 14 | A/THR active | 21 | THR (N1) mode |
| 17 | A/THR instinctive disconnect pushbutton (input) | 22 | spare (doc: "THR EPR mode") |
| 18 | SPEED/MACH mode | 23 | alpha floor |
| 19 | Mach control active (FMGC SPD/MACH latch) | 24 | A/THR inop |
| | | 25 | A/THR limited |
| others | spare | | |

### ATS FMA discrete word (A bus only): A/THR FMA column

| bit | text | when (`athr.fma_mode` / `fma_message`) | bit | text | when |
|---|---|---|---|---|---|
| 11 | TOGA | MAN_TOGA | 18 | TOGA LK | TOGA_LK |
| 12 | MCT | MAN_MCT ∨ THR_MCT | 19 | SPEED | SPEED |
| 13 | FLX | MAN_FLEX | 20 | MACH | MACH |
| 14 | CLB | THR_CLB | 21 | ASYM | message LVR_ASYM |
| 15 | THR | MAN_THR ∨ THR_LVR | 22 | LVR CLB | message LVR_CLB |
| 16 | IDLE | THR_IDLE | 23 | LVR MCT | message LVR_MCT |
| 17 | A.FLOOR | A_FLOOR | | | |

Consumers read the FCU's copy of the ATS words (`A32NX_FCU_ATS_*`), not the FMGC L:var.

### Value words

| word | Data | SSM |
|---|---|---|
| `pfd_sel_spd_kts` | `pfd_spd_target_kts` | NCD when the FCU speed and Mach words are both not NO and the speed is not managed, else NO |
| `runway_hdg_memorized_deg` | `logic.rwy_hdg_memo` | NO when the ILS runway-heading word is NO **or** the ILS tuning is inhibited, else NCD |
| `preset_mach_from_mcdu` | `fms.preset_mach` | NO when > 0 |
| `preset_speed_from_mcdu_kts` | `fms.preset_spd_kts` | NO when > 0 |
| `roll_fd_command` | FD Φ_c | NO with any lateral mode, except in RWY (LOC submode) and ROLL OUT |
| `pitch_fd_command` | FD Θ_c | NO with any longitudinal mode, except in ROLL OUT |
| `yaw_fd_command` | FD β_c | NO only in RWY (LOC submode), ALIGN or ROLL OUT |
| `fm_alt_constraint_ft` | `fms.next_alt_cstr_ft` (always sent) | NO only while `alt_cstr_applicable` (the constraint is the ALT*/ALT target), else NCD |
| `altitude_ft`, `mach`, `cas_kts` | whole words: `logic.altitude_indicated_ft`, selected ADR Mach / CAS | the source's |
| `flx_to_temp_deg_c` | `fms.flex_temp_deg_c` | NO when ≠ 0 |
| `synchro_spd_mach_value` | constant 0 | NO |
| `low/high_target_speed_margin_kts` | `fms.v_lower/upper_margin_kts` | NO when `fms.show_speed_margins` |
| `delta_p_ail_voted_cmd_deg`, `delta_p_splr_voted_cmd_deg` | AP Φ_c (both) | NO (constant) |
| `delta_r_voted_cmd_deg` | AP β_c | NO |
| `delta_nosewheel_voted_cmd_deg` | `Nosewheel_c` | NO |
| `delta_q_voted_cmd_deg` | AP Θ_c | NO |
| `track_deg`, `heading_deg`, `fpa_deg`, `vertical_speed_ft_min` | whole words from the selected IR (magnetic track/heading, FPA, inertial V/S) | the source's |
| `n1_command_percent` | `athr.n1_c_percent` | NO (constant, also with A/THR off) |

## FMGC B bus (`bus_outputs.fmgc_b_bus`)

To the ELACs (`fmgc_1/2_bus`) and FACs (`fmgc_own/opp_bus`). Discrete words 1–4 and the ATS word are the same words as
on the A bus.

| word | Data | SSM |
|---|---|---|
| `fac_weight_lbs`, `fac_cg_percent` | whole words from the chosen FAC bus | the source's |
| `fm_weight_lbs`, `fm_cg_percent` | `fms.fms_weight_lbs`, `fms.fms_cg_percent` | NO (constant) |
| `fg_radio_height_ft` | whole word, `logic.ra_computation_data.radio_height_ft` | the source's |
| `approach_spd_target_kn` | constant 0 | NO |
| `delta_p_ail_cmd_deg`, `delta_p_splr_cmd_deg` | AP Φ_c | NO |
| `delta_r_cmd_deg` | AP β_c | NO |
| `delta_nose_wheel_cmd_deg` | `Nosewheel_c` | NO |
| `delta_q_cmd_deg` | AP Θ_c | NO |
| `n1_left_percent`, `n1_right_percent` | constant 0 | NO |

## Discrete outputs (`discrete_outputs`)

| output | value |
|---|---|
| `athr_own_engaged`, `fd_own_engaged`, `ap_own_engaged` | logic bus |
| `fcu_own_fail` | constant false (read by nobody) |
| `fmgc_healthy` | constant true (the wrapper handles health) |
| `ils_test_inhibit` | `logic.ils_tune_inhibit` (→ `A32NX_FMGC_n_ILS_TUNE_INHIBIT`) |
| `stick_rudder_lock` | `logic.ap_own_engaged` |

## Assembly and end-of-step delays

`FmgcComputer_Y.out` = {`data` = the step's inputs unchanged, `logic`, `ap_fd_logic`, `ap_fd_outer_loops` (the outer
loops' outputs as returned), `athr`, `discrete_outputs`, `bus_outputs`}. The end-of-step delays (5107–5128) are not part
of the packing; each belongs to a stage, and `check_output` verifies what they hold (group `delay.*`):

| delay | holds | used by |
|---|---|---|
| `Delay_DSTATE` | this frame's `ap_fd_logic` | every mode/engagement condition next frame (`last`) |
| `Delay2_DSTATE` | this frame's outer-loop outputs | previous `Phi_loc_c` (LOC*), lateral/vertical laws |
| `Delay1_DSTATE` | this frame's `athr` outputs | A/THR modes (`last`) |
| `Memory_PreviousInput_g` ← `Delay_DSTATE_p` | AP engaged | AP latch Q (engagement) |
| `Memory_PreviousInput_p` ← `Delay_DSTATE_k` | A/THR engaged | A/THR latch Q (engagement) |
| `DelayInput1_DSTATE`, `_n` ← `next_alt_cstr_ft` | this frame's constraint | CLB / DES "constraint changed" |
| `DelayInput1_DSTATE_b` | G/S deviation (DetectDecrease, reused temporary `rtb_DataTypeConversion2_i`) | G/S capture (approach) |
| `Delay_DSTATE_fe` ← 0 | "first running frame" flag | targets |
| `Memory_PreviousInput_bo` ← `Delay_DSTATE_c` | SPD/MACH latch | targets |
| `Memory_PreviousInput_jb` | A/THR TCAS latch (`Logic_table_lwo`) | A/THR modes |
| `icLoad` ← false, `Delay_DSTATE_f` ← `Delay_DSTATE_l` | N1 command integrator | A/THR N1 |

## Results

`scripts/check-output-corpus.sh 3` (JOBS=4): 484 runs (22 recordings × plain + 3 seeds of fuzz, approach, lateral,
A/THR, engagement, targets and all scenarios combined), ~65 M checked steps, 1752 fields per step (bus outputs,
discrete outputs, the `fmgc_outputs` copies, `data` = inputs, the end-of-step delays): **0 mismatches**.

Coverage (generated values over all runs):
- Every non-spare discrete-word bit is set at some point, and rises at least 3 times. The bits never set are exactly
  the spares listed above (dw1 13, 28; dw2 18, 19, 21, 22, 24, 27, 29; dw3 15, 17, 18, 21, 26–29; dw4 15; dw5 all but
  19, 20, 24, 28; dw6 all; dw7 14–17, 19–23, 26; ATS 11, 12, 15, 16, 22, 26–29; ATS FMA 24–29). No bit outside 11–29.
- Every conditional SSM was seen both NO and NCD (`pfd_sel_spd`, `runway_hdg_memorized`, presets, FD commands,
  `fm_alt_constraint`, FLX temperature, speed margins). Discrete words and the constant/AP/N1 words were NO on every
  step. The relayed words (ADR, IR, RA, FAC) also carried FW/NCD/FT from the fuzzer.
- Never non-zero, as expected: dw6, `synchro_spd_mach_value`, `approach_spd_target_kn`, `n1_left/right_percent`.

Mutation testing (`readable/output_packing.cpp`, throwaway generator: bit position ±1, bit forced false / negated,
`||`/`&&`, comparison operators, dropped `!`, SSM rule negated or NO↔NCD, source swaps (FD↔AP, Φ↔Θ↔β, margins,
presets, weight/CG), deleted assignments): 570 mutants, 559 compiled, **558 caught, 1 equivalent** (deleting
`d.fcu_own_fail = false`, already the default). Each mutant ran against the plain replay of one flight, then the
all-scenarios run of every recording, plain replays, and the per-scenario runs, stopping at the first mismatch.

## Findings

Stock-faithful: nothing is fixed here. "Shim" = the FMA L:var decoding in `FlyByWireInterface.cpp`.

Packing (what the FMGC sends):
1. **SSM constant NO on the AP command words** (`delta_*_voted_cmd_deg` on the A bus, `delta_*_cmd_deg` on the B bus)
   and on `n1_command_percent`, whether or not the AP / A/THR is engaged. The ELAC's SSM gate on
   `delta_p_ail_cmd_deg`/`delta_q_cmd_deg` (ElacComputer.cpp 1239–1246) therefore never rejects anything; AP validity
   rests on the AP disengaged discretes alone. Roll goes to ailerons and spoilers as the same AP Φ_c.
2. **Constant words with SSM NO**: `fm_weight_lbs`/`fm_cg_percent` are NO even when the FMS has no weight (0), so the
   FAC never falls back to its own weight (FacComputer.cpp 860–865); `n1_left/right_percent` are 0 with NO, so the
   FAC engine-out sideslip target (696–712) is always 0; `approach_spd_target_kn` and `synchro_spd_mach_value` are 0
   with NO.
3. `fmgc_healthy` is a constant true and `fcu_own_fail` a constant false (health is the wasm wrapper's job;
   `fcu_own_fail` is read by nobody).
4. `runway_hdg_memorized_deg` is NO when the ILS runway-heading word is NO **or** the ILS tuning is inhibited, so it
   stays NO with a failed ILS once tuning is inhibited (the memo is then frozen).
5. The MCDU preset words carry a non-zero Data (the FMS sentinel) while NCD; dw1 bit 27 uses the same `> 0` tests.
6. Bit meanings differing from `a320-simvars.md` / `A32NXFGBusPublisher.ts`: dw4 bit 26 is ILS tune inhibit (doc: RWY
   heading memorized); dw5 bit 28 is the FMS `preset_spd_mach_activate` (doc: top of SPD synchro; the FCU reads it as
   "preset shown", consistent with the packing); dw1 bit 29 (`tla_to_ga_set`) is not documented (the FAC uses it).
   Documented but never packed: dw2 21/22/24, dw3 17/18/21/26–29, dw5 11–16/27/29, dw7 14/20, ATS 22.
7. Packed but read by nobody: dw1 27, dw3 13, dw4 11/19–22/26, dw7 11/29, ATS 17, all of dw6;
   `runway_hdg_memorized_deg` (L:var only), `synchro_spd_mach_value`, the voted roll/rudder/pitch words; on the B bus
   `approach_spd_target_kn`, `fac_cg_percent`, dw2–4, `delta_p_splr`, `delta_nose_wheel`.

Consumers (what they read; from a survey of FlyByWireInterface.cpp, the FCU/ELAC/FAC models and the TypeScript):
8. **ALT CRZ shows no vertical mode in the shim**: ALT/ALT* require `!dw1 b26` (FlyByWireInterface 1959–1973) and there
   is no ALT CRZ case, so with ALT CRZ (b19 + b20 + b26) `A32NX_FMA_VERTICAL_MODE` = 0. The PFD FMA handles it
   (FMA.tsx 929), but Phase.ts and VerticalProfileManager.ts read the shim value.
9. **FMA borders take FLARE from dw2 bit 25 (ALIGN)** (FMA.tsx 146); elsewhere FLARE is dw1 bit 25.
10. **FMA mode-reversion box reads dw3 bit 18**, which is a spare (FMA.tsx 973); the FMGC packs the reversion at dw4 bit
    28 (read there by the shim and the FWS).
11. FMA reads ATS FMA bit 29 ("MAN DTO", FMA.tsx 523), never packed. The shim's armed bitmask never sets ALT CST or TCAS
    armed (FlyByWireInterface 2007).
12. The FCU reads dw1 bit 13 (spare) in its managed CLB/DES test (no effect), and reads some words from FMGC 1 only
    (TRK/FPA reset dw1 15/16 + dw7 13, Mach from ATS 19, FD auto activate/deactivate), not from the priority FMGC.
    Its TCAS-exit V/S logic uses dw7 24/27/28 but not 29.
13. The PFD FG source selection treats ATS bit 24 (A/THR inop) as "FG inop" (FgBusProvider.ts 298–305).
14. LnavDriver.ts 80: the FMGC 2 discrete word 2 variable reads `A32NX_FMGC_1_DISCRETE_WORD_2` (no FMGC 2 fallback).
15. FlyByWireInterface 2243 sends `pfd_sel_spd_kts.Data` to the sim even when NCD.

The opposite FMGC reads (FmgcComputer.cpp, `fmgc_opp_bus`) all agree with the packing: the mode sync terms read
dw1/dw2/dw3/dw4/dw5/dw7 and the ATS word at the bits above, without SSM checks except for the LAND capability.
