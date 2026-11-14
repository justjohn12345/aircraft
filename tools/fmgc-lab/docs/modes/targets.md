# Targets: speed modes, SPD/MACH, speed targets, HDG/TRK preset, ALT soft

Readable specification of the targets block of the generated `FmgcComputer::step()` (3405–3735): the HDG/TRK preset,
selected (manual) and managed (auto) speed, the SPD/MACH latch, the speed memos, the speed target fed to the laws and
the A/THR (`V_c` = `spd_target_kts`), the PFD speed target, and the ALT soft mode. Code: `readable/targets.cpp`
(bit-exact against the generated model, checked by `build/fmgc-check-targets-stock`). Line numbers refer to
`build/models/stock/FmgcComputer.cpp` (= upstream master).

Not in this block: `alt_sel_or_cstr` / `alt_cstr_applicable` (the altitude target) are computed at 1636–1664, inside the
AP/FD engagement block, and `rwy_hdg_memo` (the memorised ILS course used by the ground speed mini) at 1366–1375.

## Conventions

As in `lateral.md`: every latch is a reset-priority SR flip-flop (default truth table, initial value false); `prev.X`
is last frame's `ap_fd_logic` output (`Delay_DSTATE`); `Confirm(u, rising|falling, T)`, `Rise(u)`, `Mono(u, T)` are the
`readable/blocks.h` blocks; every edge and timer block runs every frame. The block runs after all vertical and lateral
modes, so mode names without `prev.` are **this frame's** values. `valueOr(w)` = the ARINC value when its SSM is
normal operation, else 0; `raw(w)` = the value whatever the SSM.

- `own` = `!fmgc_opp_mode_sync`; opposite FMGC discrete word 5: bit 19 managed speed, bit 20 selected speed.
- FCU discrete word 1 (bit = index + 11, packed at `FcuComputer.cpp` 1340–1359): 11 SPD knob pushed, 12 SPD knob
  pulled, 15 SPD/MACH value changed (a pulse from the FCU when its SPD/MACH value changes), 21 SPD/MACH pushbutton,
  24 HDG-V/S selected. Word 2: 11 HDG/TRK knob pushed, 12 pulled.
- `onGround`, `gnd_eng_stop_flt_5s` (on the ground with both engines stopped, or airborne for 5 s), `engine_running`
  (at least one engine), `one_engine_out` come from input conditioning (`logic.*`).
- `noSpeed`: the FMS sends 0 for "no managed speed" (`v_managed_kts`) and "no V2" (`v_2_kts`).
- `firstFrame`: the first running frame after power-up or an FMGC restart (`Delay_DSTATE_fe`: 1 from the initialisation
  until the end of that step).

## Selected speed (`manual_spd_control_active`, latch `Memory_PreviousInput_cu`, 3436–3482)

**Set** = `(opp bit 20 && oppSync) || (own && any of)`:

| cause | condition |
|---|---|
| power-up in flight | `firstFrame && !onGround` |
| no FMS speed at lift-off | `v_managed == 0 && Rise(Confirm(!onGround, rising, 5 s)) && noSrsGsLand` |
| no AP/FD | `!any_ap_fd_engaged && phase != APPROACH` (a level, not an edge) |
| SPD knob pulled | `Rise(word 1 bit 12) && gnd_eng_stop_flt_5s` |
| MCDU preset | `preset_spd_mach_activate && (!SRS takeoff \|\| altitude <= FCU altitude)` |
| FMS speed lost | `Rise(v_managed == 0) && !onGround && noSrsGsLand` |
| TCAS | `Rise(TCAS mode)` |

`noSrsGsLand` = no SRS takeoff, SRS go-around, G/S*, G/S or LAND. Altitude = `valueOr(logic.altitude_indicated_ft)`,
FCU altitude = `valueOr(fcu selected_alt_ft)`.

**Reset** = `(prev.auto_spd_control_active && !Confirm(set, falling, 0.72 s)) || (onGround && (Rise(engine_running) ||
Rise(any_ap_fd_engaged)))`: managed speed replaces selected speed 0.72 s after the last selected-speed set condition;
on the ground the first engine start and the first AP/FD engagement clear it.

## Managed speed (`auto_spd_control_active`, latch `Memory_PreviousInput_hk`, 3483–3522)

**Set** = `(opp bit 19 && oppSync) || (own && cause && (v_managed != 0 || v_2 != 0))`, cause any of:

| cause | condition |
|---|---|
| on the ground, AP/FD and V2 | `onGround && ((Rise(any AP/FD) && V2) \|\| (Rise(engine_running) && V2 && any AP/FD) \|\| (Rise(V2 != 0) && any AP/FD))` |
| SRS | `Rise(SRS takeoff \|\| SRS go-around)` |
| SPD knob pushed | `Rise(word 1 bit 11)` |
| EXPED | `Rise(EXP CLB \|\| EXP DES)` |

**Reset** = `(prev.manual_spd_control_active && !Confirm(set, falling, 0.72 s)) || (onGround && !SRS GA && !SRS TO &&
!finalApproachOrRunway && (Rise(v_managed == 0) || Rise(v_2 == 0)))`, with `finalApproachOrRunway` =
`prev.rwy_loc_submode_active || prev.land_active || (prev.land_armed && raw RA <= 700 ft)`.

The two latches are independent: both are false at power-up on the ground (until AP/FD with V2 or engine start), and
both can be true for up to 0.72 s while one takes over from the other. Only the managed latch selects the speed
target (below); the selected latch drives the FMA/FCU (and the opposite FMGC through word 5).

## SPD/MACH (`mach_control_active`, latch memory `Memory_PreviousInput_bo`, output `Delay_DSTATE_c`, 3523–3536)

`mach` = prev.mach_control_active (the latch memory `_bo` is a copy of `Delay_DSTATE_c`, written at 5115).

- **Set** = `(!mach && Rise(word 1 bit 21)) || fms_mach_mode_activate || (preset_spd_mach_activate && preset_mach > 0)`
- **Reset** = `(Rise(word 1 bit 21) && mach) || fms_spd_mode_activate || (preset_spd_mach_activate && preset_spd_kts > 0)`

The pushbutton toggles; the FMS requests (`AP_MANAGED_SPEED_IN_MACH_ON/OFF` events, one-frame pulses, at the managed
crossover) and the MCDU presets set it explicitly; reset wins (a preset with both a speed and a Mach selects speed, as
do both FMS requests in one frame). The latch does not look at selected/managed speed, at AP/FD, or at the opposite
FMGC (no `oppSync` term), and starts false (speed) at every power-up/restart.

## Speed memos (3567–3620)

- **VAPP memo** (`FmgcComputer_B.u_l`): `v_app_kts`, frozen while `finalApproachOrRunway` (same condition as the runway
  heading memo). Not reset on an FMGC restart (only in `initialize()`).
- **Go-around speed** (`vMemoGa`): CAS (`valueOr`), frozen while SRS go-around is engaged.
- **Engine-out speed** (`vMemoEo`): CAS, frozen while `one_engine_out`.
- Both CAS memos take the CAS on the first running frame (`*_not_empty` reset in the initialisation block).

## DES lower margin (latch `Memory_PreviousInput_dh`, 3650–3659)

`vpathSpeed` = `requested_des_submode == VPATH_SPEED`. **Set** = `CAS < v_lower_margin && vpathSpeed`; **reset** =
`CAS > v_managed || !vpathSpeed` (CAS = `valueOr`). On the descent path with the speed on thrust, once below the lower
margin the target stays at the margin until the speed is back above the managed speed.

## Speed target (`spd_target_kts` = V_c, and `pfd_spd_target_kts`, 3562–3712)

`floor` = `max(valueOr(v_man), valueOr(v_4), F)` (green dot, S, F from the chosen FAC; a speed whose SSM is not normal
counts as 0) with `F = valueOr(v_3)`, except `F = 0` when `flap_slat_lever_position == 4` (CONF 3) and
`flap_3_approach_selected` and phase APPROACH.

| managed latch | branch (first match) | V_c | PFD |
|---|---|---|---|
| set | LAND, LAND armed, or phase APPROACH | `max(floor, approach)` | `approach` |
| set | SRS go-around | `srsGa` | same |
| set | SRS takeoff, one engine out | `max(V2, min(V2 + 15, vMemoEo))` | same |
| set | SRS takeoff | `V2 + 10` | `V2` |
| set | DES && show_speed_margins && submode SPEED_THRUST | `max(floor, v_upper_margin)` | `v_managed` |
| set | DES && show_speed_margins && lower-margin latch | `max(floor, v_lower_margin)` | `v_managed` |
| set | otherwise | `max(floor, v_managed)` | `v_managed` |
| clear | Mach latch set | `CAS(raw FCU Mach, corrected average static pressure)` | same |
| clear | otherwise | `raw(FCU selected speed)` | same |

- `approach` (ground speed mini) = VAPP memo when the tower headwind word is not normal; else
  `min(limit, VAPP memo + max(cos(windDir − rwyCourse) × windSpeed − towerHeadwind, 0) / 3)` with
  `windDir`/`windSpeed` the IR wind (`valueOr`, true direction), `rwyCourse` = `logic.rwy_hdg_memo` (memorised ILS
  course), `towerHeadwind` = `raw(tower_headwind_kts)`, `limit` = VFE next when its word is normal, else VMAX − 5.
- `srsGa` = `max(VAPP memo, min(VMAX − 5, VLS + 25 (15 with one engine out), vMemoGa))`.
- Mach to CAS: `1479.1 × sqrt(((((1 + 0.2 M²)^3.5 − 1) × p / 1013.25 + 1)^(2/7)) − 1)` in float, p in hPa
  (`valueOr`, so 0 when the pressure word is not normal, giving V_c = 0).
- The SRS speeds are computed every frame but used only with the managed latch set.

The managed target in climb/cruise/descent is the FMS's `v_managed_kts` (`A32NX_SPEEDS_MANAGED_PFD`) as a CAS: the FMGC
does not convert a managed Mach. In managed speed the Mach latch only changes the FMA/FCU label (and the ALT soft
condition); the FMS converts its Mach target to CAS itself.

## ALT soft mode (`alt_soft_mode_active`, latch `Memory_PreviousInput_ak`, 3713–3735)

`cond = ALT && phase == CRUISE && Mach latch && athr_own_engaged`, `dV = |V_c − raw CAS|`.
**Set** = `Confirm(cond && dV <= 4 && !reset, rising, 120 s)`; **reset** = `!cond || Confirm(dV > 4, rising, 10 s) ||
Confirm(dV > 10, rising, 4 s) || CAS > VMAX − 5 || word 1 bit 15`. Consumed by the ALT_HOLD law and the A/THR gain
fade (see `alt-vs-fpa-tcas.md`, `docs/athr-n1.md`).

## HDG/TRK preset (`hdg_trk_preset_available`, latch `Memory_PreviousInput_bw`, 3405–3434)

See `lateral.md`. Set: `Mono(FCU selected heading or track valid, or knob pulled, 0.72 s)` && (`Confirm(NAV armed,
falling, 0.72 s)` || guided) && `low`; reset: `!low`, HDG, TRK, NAV, `Mono(knob pushed, 0.72 s)`, `Confirm(no AP/FD,
falling, 0.72 s)`; `guided` = LOC*, LOC, LAND, FINAL, GA TRK, RWY TRK; `low` = guided || onGround || (raw RA < 30 ft &&
RA not failed).

## Thresholds and times

| what | value | parameter |
|---|---|---|
| selected/managed takeover | 0.72 s | `ConfirmNode_timeDelay_ht`, `_gz` |
| "in flight" for the no-FMS-speed cause | 5 s | `ConfirmNode1_timeDelay_h` |
| VAPP memo frozen | LAND armed and RA ≤ 700 ft, LAND, RWY | `CompareToConstant_const_o2` |
| SRS takeoff | V2 + 10, engine out ≤ V2 + 15 | constants at 3643–3647 |
| SRS go-around | ≤ VLS + 25 (15 engine out), ≤ VMAX − 5 | `high_i`, 3624–3637 |
| ground speed mini | 1/3 of the gust, limit VFE next or VMAX − 5 | `Gain_Gain_on`, `Bias_Bias_j` |
| F speed dropped from the floor | flap lever index 4 (CONF 3) | `CompareToConstant_const_d4` |
| ALT soft | 4 kt / 120 s on; 4 kt 10 s, 10 kt 4 s, VMAX − 5 off | `ConfirmNode_timeDelay_mf` etc. |
| HDG/TRK preset | 30 ft RA, 0.72 s pulses | |

## Findings and differences from the FCOM (A320 FCOM DSC-22_30-40, DSC-22_30-80-40, DSC-22_30-90, DSC-22_30-70-70)

Stock behaviour, not fixed here.

1. **Managed speed without AP/FD.** FCOM: managed speed engages "whenever AP or FD is engaged" and the SPD knob is
   pushed, EXPED engages, V2 is inserted or SRS engages. The code needs AP/FD only for the V2 causes on the ground;
   a SPD knob push, EXPED or SRS sets managed speed with no AP/FD. Outside the approach phase "no AP/FD" keeps setting
   selected speed, so the result is a 0.72 s period with both latches true (managed target), then selected again.
2. **SPD/MACH pushbutton in managed speed.** FCOM describes the pushbutton for selected speed ("when the speed is
   selected, the flight crew has to perform the switching manually"; in managed speed the FMGC switches automatically).
   The latch toggles on every pushbutton edge whatever the speed mode. In managed speed this changes the FMA/FCU label
   (and the ALT soft condition) while the target stays `v_managed`, and nothing brings the latch back until the next
   FMS request (the next crossover) or preset: the FMGC's Mach state can disagree with the FMS's own speed/Mach state
   for the rest of the climb, cruise or descent. Seen in the corpus (`mach_push_in_managed`).
3. **No synchronisation of the Mach latch.** Unlike the speed latches, it has no opposite-FMGC term: each FMGC toggles
   on the same FCU edge independently, and a missed edge (FMGC not running, restart) leaves the two FMGCs in different
   states for good. A restart also sets it to speed whatever the flight level. The FCU takes its Mach display state from
   FMGC 1's ATS discrete word only (`FcuComputer.cpp` 572–577, bit 19), whichever FMGC has priority.
4. **TCAS selects the speed.** TCAS mode engagement sets selected speed (consistent with DSC-22_30-70-85: the target
   becomes the speed at the RA). The newer DSC-22_30-40 engagement list (MSN 06904 and later) says the TCAS mode
   engagement makes the speed *managed*.
5. **FCU words used raw.** The selected target uses the FCU speed/Mach `Data` without the SSM (the FCU sends them with
   the window dashed); with the managed latch clear and no valid FCU value (power-up on the ground, power-up in flight)
   V_c is whatever the FCU sends.
6. **Mach to CAS without pressure.** With the corrected average static pressure word not normal, the selected Mach
   target converts to 0 kt.
7. **Ground speed mini wind projection.** FCOM: the current headwind is "the projection of the current wind measured by
   the ADIRS on the aircraft longitudinal axis". The code projects the IR wind on the memorised ILS runway course
   (`rwy_hdg_memo`), and the wind direction is true while the ILS course is magnetic, so the headwind is off by the
   magnetic variation. Without an ILS the memorised course is 0 (or the last ILS course). The increment is 1/3 of the
   gust (the later FCOM version), not below VAPP, at most VFE next (VMAX − 5 when VFE next is not normal: VFE − 5 in
   CONF FULL if the FAC sends no VFE next there). The ground speed mini applies in the APPROACH phase or with LAND armed
   whatever the lateral/vertical modes and the altitude; without a valid tower wind the target is VAPP.
8. **No VMAX limit on the managed target.** FCOM note 4: the FMGS limits a managed target above VMAX to VMAX. Here the
   managed target (outside SRS GA and the ground speed mini) is only raised to the manoeuvring floor, never capped at
   VMAX (left to the FMS value and the laws' speed protection).
9. **Manoeuvring speed floor.** FCOM note 3: managed targets below manoeuvring speed are limited to the current
   configuration's VAPP, F, S or green dot. The code floors at max(green dot, S, F) of whatever the FAC sends normal
   (F dropped in CONF 3 with a CONF 3 landing in the approach phase); the PFD target is the unfloored value. SRS and
   selected targets are not floored.
10. **SRS go-around speed** matches DSC-22_30-80-40 (current speed or VAPP, whichever higher, limited to VLS + 25 /
    VLS + 15), plus a VMAX − 5 cap, with VAPP the memorised value. **SRS takeoff**: V2 + 10; engine out: the higher of
    V2 and the speed at the failure, limited to V2 + 15 (FCOM: "highest of V2 or current speed"; the code freezes the
    speed at the failure).
11. **Preset in SRS takeoff.** A preselected speed activates selected speed except during SRS takeoff above the FCU
    altitude.
12. **FMS speed loss.** FCOM: selected speed when the FM speed target is lost, except in SRS, G/S, LAND and GO AROUND:
    matches (on the edge, in flight); in addition, with no FMS speed 5 s after lift-off. On the ground, losing the FMS
    speed or V2 clears managed speed (not in SRS or final approach).
13. **ALT soft mode** uses ALT (not ALT CRZ) and the CRUISE phase: FCOM requires the cruise flight level (ALT CRZ). Any
    SPD/MACH value change on the FCU (bit 15) drops it. The ±50 ft band is in the ALT_HOLD law.
14. **Selected speed on the ground**: cleared at the first AP/FD engagement as well as at engine start (FCOM: engine
    start only).

## Verification (2026-11-14)

`JOBS=4 scripts/check-targets-corpus.sh 3`: 440 runs (22 recordings × resync, free-running, 3 × fuzzed resync/free,
approach + fuzz, targets scenarios alone / free with the fuzzer / free with the approaches), 0 mismatches, every field
non-zero and changing, every named condition seen (`build/check-targets/conditions.summary`). The vertical check
(which now calls the shared preset and ALT soft functions) still passes: 374 runs, 0 mismatches.

Without the synthetic episodes the recordings never have a HDG/TRK preset, the DES lower margin only in 2 files, and
few SPD/MACH pushes; `src/scenario_targets.h` supplies those (see its header).
