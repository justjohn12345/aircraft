# Autothrust modes and FMA (alpha floor, THR / SPEED / RETARD, A/THR active, LVR messages)

Source: `tools/fmgc-lab/build/models/stock/FmgcComputer.cpp` (= upstream master), lines 4181–4423; outputs 4947–4955,
`Delay1_DSTATE` 5116–5125, initialisation 511–518 and 674–681. Parameter values from `docs/params-FmgcComputer.txt`.
Readable rewrite: `readable/athr_modes.cpp` (bit-exact, checked by `src/check_athr.cpp`). This stage produces the nine
mode fields of `base_fmgc_athr_outputs`; the N1 command (4424–4640) reads them.

Conventions:

- Thrust lever angle (TLA, deg): IDLE 0, CL 25, FLX/MCT 35, TOGA 45, negative = reverse.
- `TLA` = A429ValueOrDefault of the FADEC `selected_tla_deg` word (0 unless SSM = normal operation); `TLAraw` = the raw
  word value, any SSM. Both are used (see finding 1). `maxTLAraw = fmax(own, opp)` (float).
- `last.X` = the autothrust output X of the previous frame (`Delay1_DSTATE`).
- Latches are SR flip-flops `Logic_table[((S << 1) + R) << 1) + Q]`; all seven tables used here (`_lwo`, `_lm`, `_ac`,
  `_ma`, `_acc`, `_b3`, `_mj`) are `0 1 0 0 1 1 0 0 | 1 0 1 1 0 0 0 0` (hold / reset / set / reset wins; the second
  half is the !Q output). All initial conditions are 0. On the first running frame the latches, the four confirm nodes
  (output false, timer 0) and the four pulse nodes (first-call priming) are reset.
- Confirm node "set recently" = `ConfirmNode(x, falling edge, 0.72 s)`: true while x is true and for 0.72 s after.
- Bits are 1-based (`bit N = (word >> (N-1)) & 1`).
- The generated code reads all its inputs from this frame's `rtb_BusAssignment_jmp` (= `out.data`, `out.logic`,
  `out.ap_fd_logic`): the autothrust runs after this frame's AP/FD mode logic and sees this frame's modes and law.

| Output field | Generated | Meaning |
|---|---|---|
| `athr_active` | `rtb_y_er` (4231) | A/THR active (controls thrust), as opposed to armed (engaged, levers above the active range) |
| `athr_limited` | `rtb_OR1_fl` (4243) | active with a lever below the CL detent (one engine out: below MCT) |
| `alpha_floor_mode_active` | latch `Memory_PreviousInput_hu` | alpha floor / TOGA LK |
| `thrust_mode_active` | latch `Memory_PreviousInput_bh` | THR mode (pitch controls speed, A/THR a thrust level) |
| `thrust_target_idle` | `rtb_adr3Invalid` (4316) | THR mode target is IDLE (else CLB/MCT, the lever) |
| `speed_mach_mode_active` | latch `Memory_PreviousInput_cm` | SPEED/MACH mode |
| `retard_mode_active` | latch `Memory_PreviousInput_ol` | RETARD (autoland flare idle) |
| `fma_mode`, `fma_message` | `rtb_y_ag`, `rtb_y_js` | FMA column 1 and the LVR message |

## TCAS latch (`Memory_PreviousInput_jb`, 4183–4196, stored at 5116)

Alpha floor is inhibited while TCAS mode holds this latch.

- S = Pulse↑(`tcas_active`) (TCAS mode engagement)
- R = `¬tcas_active ∨ (Confirm↑(tcas_active, 5 s) ∧ (TLA_opp < 24 ∨ TLA_own < 24))`
- Only the !Q output is used (`Logic_table_lwo[i + 8]`): `notInTcas = ¬S ∧ (R ∨ ¬Q)`. It differs from ¬Q_next only when
  S and R are both true, which cannot happen (R's confirm needs 5 s of TCAS mode, S is its first frame).

FCOM (DSC-22_30-50, TCAS mode, "Alpha floor is temporarily inhibited"): all levers in CL at the RA → inhibited while
TCAS mode is engaged; a lever below CL → inhibited 5 s, then re-enabled if a lever remains below CL. The code matches,
with one generalisation: after 5 s the reset is evaluated continuously, so retarding a lever below 24° at any later
time in TCAS mode also re-enables alpha floor. A failed TLA word reads 0° and counts as "below CL".

## Alpha floor (`Memory_PreviousInput_hu`, 4197–4203)

- S = Pulse↑(`notInTcas ∧ FAC word 5 bit 29`) (FAC alpha floor condition, chosen FAC bus)
- R = `¬athr_own_engaged`

Only the rising edge of the FAC condition engages it; it then stays until the A/THR is disengaged (TOGA LK once the
FAC condition is gone). The same FAC bit rising edge also engages the A/THR itself (logic section, 1507–1530, not part
of this stage), so on that frame `athr_own_engaged` is already true. This frame's latch value is used by everything
below (A/THR active, mode inhibition), while the other thrust mode latches use the *previous* frame's alpha floor.

## A/THR active (`rtb_y_er`, 4204–4231) and limited (`rtb_OR1_fl`, 4232–4244)

```
active  = athr_own_engaged ∧ ( alphaFloor
                             ∨ (0 ≤ TLA_opp ≤ 25 ∧ 0 ≤ TLA_own ≤ 25)
                             ∨ (¬flexOpp ∧ ¬flexOwn ∧ one_engine_out ∧ 0 < TLA_opp ≤ 35 ∧ 0 < TLA_own ≤ 35) )
limited = active ∧ (TLA_opp < 24 ∨ TLA_own < 24 ∨ (one_engine_out ∧ (TLA_opp < 34 ∨ TLA_own < 34)))
```

`flexX` = FADEC `selected_flex_temp_deg` SSM is normal operation (a FLEX temperature is entered).
`one_engine_out = eng_opp_stop xor eng_own_stop` (1308).

## THR, SPEED/MACH and RETARD latches (4245–4311)

All three have the same structure:

```
set    = (oppSync ∧ opp ATS word bit N) ∨ (¬oppSync ∧ <engage condition> ∧ ¬alphaFloor)
recent = Confirm↓(set, 0.72 s)
reset  = ¬athr_own_engaged ∨ ((last.<any other of alpha floor / THR / SPEED / RETARD>) ∧ ¬recent)
```

| Latch | Opp ATS bit | Engage condition | Reset by last frame's |
|---|---|---|---|
| THR (`bh`) | 21 | Pulse↑(A/THR engaged ∧ law ∈ {SPD_MACH, SRS} or (VPATH ∧ FMS submode VPATH_THRUST)) | alpha floor, RETARD, SPEED |
| SPEED (`cm`) | 18 | Pulse↑(A/THR engaged ∧ law ∈ {ALT_HOLD, ALT_ACQ, VS, FPA, GS, FLARE} or (VPATH ∧ (submode VPATH_SPEED ∨ FINAL DES)) or (NONE ∧ (¬last.RETARD ∨ ¬on_ground))) | alpha floor, RETARD, THR |
| RETARD (`ol`) | 20 | LAND active ∧ own AP engaged ∧ raw RA ≤ 40 ft (level, not an edge) | alpha floor, THR, SPEED |

The opposite FMGC's ATS discrete word is the one this FMGC packs at 4769–4788 (bits: 14 active, 18 SPEED, 19 MACH,
20 RETARD, 21 THR, 23 alpha floor, 25 limited).

`thrust_target_idle` = DES ∨ OP DES ∨ EXP DES ∨ FINAL DES active (4312–4316): a level of this frame's modes, independent
of the THR latch.

Behaviour that follows from the structure:

- The engage conditions of THR and SPEED are rising edges of "a law of that family is active with the A/THR engaged",
  so engaging the A/THR, or switching between families, starts the matching mode. A law change within the same family
  (e.g. V/S → ALT*) does not re-trigger; the latch simply stays.
- The handover resets read the previous frame's outputs: on the frame a new mode sets, the old one is still set (two
  thrust modes for one frame; the FMA and the N1 stage use their own priorities). An old mode is not reset while its own
  set condition was true within the last 0.72 s, so rapid law flip-flops can keep two latches set for up to 0.72 s.
- Alpha floor blocks every set term of this frame (current-frame `hu`) and resets the others one frame later
  (`last.alpha_floor_mode_active`). As alpha floor is reset only with the A/THR, no thrust mode can engage again until
  the A/THR is disengaged and re-engaged.
- SPEED is engaged by "no vertical law" (no AP/FD), except while RETARD is latched on the ground, so RETARD remains
  after touchdown when the AP and FDs are disconnected.

## Lever message latches (4317–4342)

```
clbPhase   = H_ind ≥ thrust_reduction_alt ∨ ALT* ∨ ALT ∨ V/S ∨ FPA     (H_ind = logic.altitude_indicated_ft.Data, any SSM)
disarm     = on_ground ∨ (FMS phase = GOAROUND ∧ H_ind < thrust_reduction_alt)
LVR CLB armed (kr): S = clbPhase ∧ ¬one_engine_out, R = one_engine_out ∨ disarm
LVR MCT armed (km): S = clbPhase ∧ one_engine_out,  R = ¬one_engine_out ∨ disarm
```

The same thrust reduction altitude is used with one engine out (no engine-out acceleration altitude).

## FMA message (4343–4377), first match wins

Let `eng = athr_own_engaged ∧ ¬alphaFloor`, levers raw.

| Message | Condition |
|---|---|
| LVR CLB | `eng ∧ ((LVR CLB armed ∧ ¬climbRaKeep) ∨ (TCAS ∧ ¬active ∧ tcas_vs_target ≤ 500)) ∧ ((both < 24) ∨ either > 26)` |
| LVR MCT | `eng ∧ LVR MCT armed ∧ both < 34` |
| LVR ASYM | `eng ∧ ¬one_engine_out ∧ one lever in (24, 26) and the other < 24 or > 26` |
| NONE | otherwise (LVR_TOGA is never produced) |

`climbRaKeep = IAS ≤ VMAX − 20 kt ∧ tcas_vs_target > 500 ∧ TCAS ∧ ¬active` (IAS = selected ADR CAS, VMAX = chosen FAC
`v_max_kn`, both 0 when not normal; float `VMAX + (−20)`). `tcas_vs_target` is the RA V/S (± rate + 200 fpm for a
corrective RA, the current ADR V/S for a preventive one).

## FMA mode (4378–4423), first match wins

`manual = athr_own_engaged ∧ ¬active`, `auto = athr_own_engaged ∧ active`, `flex = flexOpp ∨ flexOwn`.

| # | Mode | Condition |
|---|---|---|
| 1 | MAN TOGA | manual ∧ maxTLAraw > 44 |
| 2 | MAN FLX | manual ∧ 34 < maxTLAraw < 36 ∧ flex |
| 3 | MAN MCT | manual ∧ 34 < maxTLAraw < 36 ∧ ¬flex |
| 4 | MAN THR | manual ∧ maxTLAraw > 24 |
| 5 | SPEED | auto ∧ SPEED latch ∧ ¬mach_control_active |
| 6 | MACH | auto ∧ SPEED latch ∧ mach_control_active |
| 7 | THR MCT | auto ∧ THR latch ∧ maxTLAraw > 34 ∧ ¬idle target |
| 8 | THR CLB | auto ∧ THR latch ∧ maxTLAraw > 24 ∧ ¬idle target |
| 9 | THR LVR | auto ∧ THR latch ∧ maxTLAraw < 25 ∧ ¬idle target (always true after 8 unless both raw TLAs are NaN) |
| 10 | THR IDLE | auto ∧ ((THR latch ∧ idle target) ∨ RETARD latch) |
| 11 | A.FLOOR | auto ∧ alpha floor ∧ FAC word 5 bit 29 |
| 12 | TOGA LK | auto ∧ alpha floor ∧ ¬FAC word 5 bit 29 |
| – | NONE | otherwise (also: engaged, not active, max lever ≤ 24, e.g. a lever in reverse or a failed TLA word) |

MAN GA SOFT and MAN DTO are never produced. RETARD shows as THR IDLE.

## Findings and differences from the FCOM (A320 FCOM DSC-22_30-90 / DSC-22_30-100 / DSC-22_30-50)

1. **Two readings of the lever angle.** The A/THR active range, limited flag and TCAS latch use the SSM-validated TLA
   (0° when the FADEC word is not normal), the FMA mode and LVR messages the raw value. A failed TLA word therefore
   counts as IDLE: the A/THR can be active (and "limited") whatever that lever's real position, while the FMA uses
   the raw angle.
2. **One engine out active range.** FCOM: A/THR active when the live engine's lever is between IDLE and MCT. Code:
   both levers in (0°, 35°] and no FLEX temperature on either FADEC. With the dead engine's lever at IDLE (as the
   engine failure procedure leaves it) and the live one in MCT, the A/THR is not active (MAN MCT) — IDLE is excluded
   from the engine-out range. Both-levers-in-[0, 25] still makes it active.
3. **LVR MCT only below the MCT detent.** LVR MCT needs both levers < 34°. With one engine out above the thrust
   reduction altitude and the levers in TOGA or FLX, no message flashes (FCOM: LVR MCT flashes at EO ACC ALT to bring
   the lever to MCT). LVR MCT also arms at the (two-engine) thrust reduction altitude, not the engine-out acceleration
   altitude.
4. **TCAS and LVR CLB.** FCOM: LVR CLB inhibited at THR RED ALT for a climbing RA > 500 fpm, shown even below THR RED
   ALT if speed reaches VFE − 20 kt or the RA is ≤ 500 fpm. Code: the inhibition applies only below VMAX − 20 kt and
   with the A/THR not active; above VMAX − 20 kt the inhibition is lifted but LVR CLB still needs the THR RED latch, so
   below THR RED ALT nothing is shown. The ≤ 500 fpm path matches but needs the A/THR not active.
5. **LVR CLB over LVR ASYM.** Above THR RED ALT, one lever in CL and the other above 26° shows LVR CLB, not LVR ASYM.
   LVR CLB/MCT/ASYM only need the A/THR engaged (armed or active), not active.
6. **A/THR limited with either lever.** FCOM: both levers below CL (one engine: below MCT). Code: either lever below
   24° (one engine out: either below 34°).
7. **RETARD and AP disconnection.** FCOM: if the AP is disengaged in the flare before touchdown, SPEED replaces RETARD.
   Code: RETARD is a latch reset only by another thrust mode or A/THR disengagement; with an FD still on, the law stays
   FLARE (already in the SPEED family, no new rising edge), so RETARD stays. Only with AP and both FDs off in the air
   (law NONE) does SPEED take over. RETARD engages at ≤ 40 ft RA (FCOM: approximately 40 ft); the RA is read
   without an SSM check.
8. **Mode transitions with alpha floor.** A thrust mode set within the last 0.72 s survives alpha floor for up to 0.72 s
   and, being higher in the FMA chain, shows instead of A.FLOOR during that time. A.FLOOR / TOGA LK are the last
   entries of the chain.
9. FCOM lists FLARE among the THRUST-mode pitch modes; the code puts the FLARE law in the SPEED family (in an autoland
   RETARD takes over at 40 ft anyway).

## Coverage (check_athr corpus, `JOBS=6 scripts/check-athr-corpus.sh 3`)

Without `--athr`, the recordings and the fuzzer never produce THR MCT and rarely alpha floor, LVR MCT, MAN MCT or engine
out. With the `--athr` scenarios (src/scenario_athr.h) every FMA mode the code can produce and every message are seen
(THR MCT is the rarest: it needs one engine out, both levers in (34, 35], THR mode and no FLEX). Never produced by the
code: MAN GA SOFT, MAN DTO, LVR TOGA. Input conditions are counted per run (`conditions:` line of the report).
