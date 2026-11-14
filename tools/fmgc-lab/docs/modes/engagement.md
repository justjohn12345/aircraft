# AP / FD / A/THR engagement and opposite-FMGC synchronisation

Readable specification of the engagement logic in the generated `FmgcComputer::step()`: the AP/FD/A/THR common
conditions, FD engaged, the AP engage request and AP engaged latches, the A/THR inhibit and A/THR engaged latches, AP
and A/THR inop, opposite-FMGC priority and mode synchronisation, `any_ap_fd_engaged`, the mode reset, and the FD/AP
flags computed after the vertical modes (FD auto disengage, V/S target not held, the mode reversion "triple click",
pitch FD bars flashing), and the landing capability computed after the law selection (LAND 2 / LAND 3 FAIL PASSIVE /
LAND 3 FAIL OP capability, inop and capacity). Code: `readable/engagement.cpp`, `readable/engagement_land.cpp`
(bit-exact against the generated model, checked by `build/fmgc-check-engagement-stock`). Line numbers refer to
`build/models/stock/FmgcComputer.cpp` (= upstream master).

Not here: the A/THR modes (`docs/athr-modes.md`), the roll FD bars flashing (lateral stage, `docs/modes/lateral.md`).

## Outputs

| Output | Generated | Line |
|---|---|---|
| `logic.ap_fd_athr_common_condition` | `fdOppOff` (confirm `sf_MATLABFunction_kv`) | 1382, 1422 |
| `logic.ap_fd_common_condition` | `rtb_y_ab` (confirm `sf_MATLABFunction_bv`) | 1395, 1423 |
| `logic.fd_own_engaged` | `fdOwnOff` (sic) | 1420 |
| `logic.ap_inop` | `rtb_ap_inop` | 1467 |
| `logic.ap_own_engaged` | latch `Delay_DSTATE_p` (Q from `Memory_PreviousInput_g`) | 1482 |
| `logic.athr_inop` | `rtb_athr_inop` | 1466, 1527 |
| `logic.athr_own_engaged` | latch `Delay_DSTATE_k` (Q from `Memory_PreviousInput_p`) | 1528 |
| `logic.fmgc_opp_priority` | `rtb_y_ab` | 1537 |
| `ap_fd_logic.fmgc_opp_mode_sync` | `rtb_OR2_l` | 1547 |
| `ap_fd_logic.any_ap_fd_engaged` | `apCondition` (reused name) = `rtb_y_bq` | 1548, 1558 |
| `ap_fd_logic.lateral_mode_reset`, `longitudinal_mode_reset` | `rtb_AND10_b` (one signal) | 1553 |
| `ap_fd_logic.fd_auto_disengage` | `rtb_Compare_ji` | 3348 |
| `ap_fd_logic.vs_target_not_held` | `rtb_y_n` | 3382 |
| `ap_fd_logic.ap_fd_mode_reversion` | monostable `sf_MATLABFunction_hdx` | 3388 |
| `ap_fd_logic.pitch_fd_bars_flashing` | monostable `sf_MATLABFunction_ppo` | 3396 |
| `ap_fd_logic.land_2_capability`, `_3_fail_passive_`, `_3_fail_op_` | `rtb_OR_b`, `rtb_y_oy`, `rtb_Equal_o` | 3824–3830, 4049–4051 |
| `ap_fd_logic.land_2_inop`, `_3_fail_passive_`, `_3_fail_op_` | `!rtb_y_e0`, `!rtb_OR1_j`, `!rtb_y_d2_tmp` | 3812–3823, 4052–4054 |
| `ap_fd_logic.land_2_capacity`, `_3_fail_passive_`, `_3_fail_op_` | opposite bus bits or own capability | 3838–3863 |

Internal latches: `Memory_PreviousInput` (AP engage request, 1468), `Memory_PreviousInput_g1` (A/THR inhibited, 1490).
The two "Q" memories `Memory_PreviousInput_g` / `_p` are copies of `Delay_DSTATE_p` / `_k` made at the end of the step
(5108–5109), so each AP/A/THR latch reads its own previous value. All four tables are the default SR table (reset
wins); every initial condition is false; on the first running frame the latches, the confirm nodes (output false,
timer 0), the monostables and the pulse nodes (first-call priming) are reset (454–459, 523–540, 640–642).

## Conventions

As in `lateral.md`: `prev.X` is last frame's `ap_fd_logic` output (`Delay_DSTATE`); `Confirm(u, rising|falling, T)`,
`Rise(u)`, `Mono(u, T)` (monostable, rising edge, not retriggerable) are the `readable/blocks.h` blocks; all of them run
every frame. `v(word)` = the word's value when its SSM is normal operation, else 0. Bits are 1-based.

Conditioned inputs (input conditioning, not rewritten) are read from the logic bus: `onGround`, `gnd_eng_stop_flt_5s`
(on ground with both engines stopped, or not on ground for 5 s), `double_adr_failure`, `double_ir_failure`,
`fac_lg_data_failure` (FAC discrete word 5 failed or its bit 17 set), `fac_weights_failure`, `fac_speeds_failure`,
`dual_ra_failure`, `fcu_failure` (both FCU channels unhealthy), `fac_rudder_control_failure`, `engine_running`, the
selected ADR/IR/FAC words, the raw selected radio height `ra` (2500 ft when both RAs failed), `rwy_hdg_memo`.

Discrete inputs from the opposite FMGC (`ap_opp_engaged`, `fd_opp_engaged`, `athr_opp_engaged`, `fmgc_opp_healthy`) are
its discrete outputs. FMGC 1 is stepped first, so it sees FMGC 2's previous-frame outputs and FMGC 2 sees FMGC 1's
current ones (`FlyByWireInterface.cpp` 1737–1751).

`landOrGa` = prev LAND || prev SRS GA.

## Common conditions (1380–1407)

**AP/FD/A/THR common condition** = `Confirm(c, rising, 0.72 s)`, `c` = no double ADR failure, no double IR failure,
(FAC landing gear data valid or `landOrGa`), and (FM valid, or `landOrGa`, or prev G/S with `ra < 700 ft`).

**AP/FD common condition** = `Confirm(c, rising, 0.72 s)`, `c` = (FAC weights valid or `landOrGa`), (FAC speeds valid or
prev LAND), (a radio altimeter valid, or none of LAND armed / G/S* / G/S / LAND last frame), (FCU healthy or
`landOrGa`), and not `Mono(prev ROLL OUT && |rwy_hdg_memo − v(IR true heading)| > 20°, 3 s)`: in ROLL OUT, a heading
more than 20° off the memorised runway course drops the AP/FD for 3 s (and the 0.72 s confirmation after), which
disengages the AP and the FD.

## FD (1408–1420)

FCU discrete word 2 bit 26 = FD 1 (captain) pushbutton off, bit 27 = FD 2 off (`FcuComputer.cpp` 1335–1336). Own = bit
26 on FMGC 1, bit 27 on FMGC 2.

`fd_own_engaged` = both common conditions && (own FD pb on || (opposite FD pb on && !fd_opp_engaged)). The second term
lets an FMGC drive the other side's FD when the other FMGC's FD is not engaged (failed, or its common conditions lost).
The FD engagement does not depend on the AP.

## AP (1434–1484)

`apCondition` = no AP instinctive disconnect, at least one ELAC authorising the AP (`!elac_own_ap_disc ||
!elac_opp_ap_disc`), FAC rudder control available. `ap_inop` = !commonAthr || !commonApFd || !apCondition.

**Engage request** (latch `Memory_PreviousInput`):

- Set: `Rise(fcu_ap_button)` with the AP off last frame, `gnd_eng_stop_flt_5s`, both common conditions, `apCondition`, and
  the engagement envelope: `v(VLS) ≤ v(IAS) ≤ v(VMAX)`, `−10° ≤ v(pitch) ≤ 22°`, `|v(roll)| ≤ 40°` (float compares).
- Reset (any of): `ap_inop`; the AP pushbutton pressed with the AP engaged; AP instinctive disconnect; the opposite AP
  engaging (`Rise(ap_opp_engaged)`) while none of LAND armed / LAND / SRS GA was active last frame; on FMGC 2 only, with
  the opposite AP engaged, the moment LAND armed, LAND and SRS GA have all ended (`Rise(none of them)`); on the ground,
  SRS GA last frame or a thrust lever `v(TLA) ≥ 36.7°`; on the ground, the first engine starting
  (`Rise(engine_running)`).

**AP engaged** (`Delay_DSTATE_p`, `logic.ap_own_engaged`): set by `Rise(fcu_ap_button) && request` (the same frame as the
request), reset by `!Confirm(request, falling, 0.2 s)` or the instinctive disconnect. So the AP disengages 0.2 s after
the request is reset (pushbutton, inop, opposite AP, ground), and at once on the instinctive disconnect.

The envelope is checked only at engagement; leaving it does not disengage the AP here (the ELACs do that through
`elac_*_ap_disc`).

## A/THR (1485–1533)

**Inhibit** (latch `Memory_PreviousInput_g1`): set by `Confirm(athr_instinctive_disc, rising, 15 s)`, reset by a constant
false (`Constant_Value_e`): the A/THR is lost until the FMGC restarts.

`athrPossible` = (prev managed or selected speed control active) && !inhibited && FCU healthy && FAC speeds valid.
`athr_inop` = !commonAthr || !athrPossible.

**A/THR engaged** (`Delay_DSTATE_k`, `logic.athr_own_engaged`):

- Set: commonAthr && athrPossible && one of: `Rise(fcu_athr_button)` with `ra > 100 ft` (or both RAs failed);
  `Rise(Confirm(prev SRS takeoff || prev SRS GA, rising, 0.9 s))`; `Rise(FAC discrete word 5 bit 29)` (alpha floor, of
  the selected FAC); `Rise(prev TCAS mode)`.
- Reset (any of): `athr_inop`; `Rise(prev fmgc_opp_mode_sync && !athr_opp_engaged)` (synchronised to an opposite FMGC
  without A/THR); the A/THR pushbutton pressed while engaged; the A/THR instinctive disconnect; `Rise(both v(TLA) <
  2.6°)` (both levers to IDLE; double compare); `Confirm(prev engaged && (either ECU status word 3 bit 19 clear), rising,
  0.9 s)`: an ECU not reporting A/THR engaged for 0.9 s (bit 19 is the FADEC's own A/THR engaged latch,
  `FadecComputer.cpp` 141–143, 356).

The pushbutton disconnect term is `prev engaged && Rise(button) && !rtP_fmgc_ap_fd_logic_output_MATLABStruct.
lateral_modes.land_active`: a constant all-false bus where the previous frame's LAND was presumably meant, so a push
disconnects in any mode (finding 6).

## Opposite FMGC priority and mode synchronisation (1534–1547)

`fmgc_opp_priority` (the opposite FMGC is master), with this frame's own AP/FD/A/THR and the opposite FMGC's discretes:

- FMGC 1: own AP off && (opp AP || own FD off) && (opp AP || opp FD || own A/THR off) && (opp AP || opp FD || opp
  A/THR).
- FMGC 2: (own AP off || opp AP) && (opp AP || opp FD || own FD off) && (opp AP || opp FD || opp A/THR || own A/THR
  off) && (own AP || own FD || own A/THR || !fmgc_opp_healthy).

So FMGC 1 is master unless only FMGC 2 has an AP, or FMGC 2 has an FD (or A/THR) that FMGC 1 lacks; FMGC 2 follows
FMGC 1 whenever it has something engaged that FMGC 1 also has (or has nothing engaged and FMGC 1 is unhealthy).

`fmgc_opp_mode_sync` = priority && (own AP off || opp AP off): with both APs engaged, neither FMGC copies the other's
modes (FMGC 2 keeps priority but runs its own mode logic). The mode logic copies the opposite FMGC's mode bits when
`fmgc_opp_mode_sync` (see `lateral.md`, `takeoff-ga-approach.md`); the A/THR modes copy its ATS bits.

## AP/FD engaged and mode reset (1548–1559)

`any_ap_fd_engaged` = own AP || opp AP || own FD || opp FD. `lateral_mode_reset` = `longitudinal_mode_reset` =
`!any_ap_fd_engaged || (Rise(engine_running) && onGround)` (a separate pulse node from the AP's, same input).

## FD/AP flags (3335–3403, after the vertical and lateral modes, this frame's modes)

- **FD auto disengage** (`fd_auto_disengage`, FMGC A bus discrete word 4 bit 27, read by the FCU as
  `fd_auto_deactivate`): own AP off, A/THR engaged, own FD engaged, and either (CLB, OP CLB or EXP CLB) with `v(VMAX) + 4
  < v(IAS)` or (DES, OP DES or EXP DES) with `v(IAS) < v(VLS) − 2` (float arithmetic).
- **V/S target not held** (`vs_target_not_held`, word 4 bit 29, read by `FlyByWireInterface.cpp` 1044 as "AP speed
  protection active"): own AP engaged in V/S or FPA, and either `v(IAS) < v(VLS) + 3` with the error below −threshold
  or `v(IAS) > v(VMAX) − 3` with the error above +threshold. V/S: error = `v(IR inertial V/S) − v(FCU V/S)`, threshold
  50 fpm; FPA: error = `v(IR FPA) − v(FCU FPA)`, threshold 0.1° (float error, double threshold).
- **Mode reversion** (`ap_fd_mode_reversion`, word 4 bit 28, the FMA triple click): `Mono(any_ap_fd_engaged && (V/S not
  held || FD auto disengage || OP CLB reversion || V/S reversion || lateral reversion), 1 s)`.
- **Pitch FD bars flashing**: `Mono(V/S reversion || (Rise(any_ap_fd_engaged) && !TCAS), 10 s)`; the same pulse node
  feeds the roll FD bars (rewritten separately in the lateral stage with its own, equivalent, edge detector).

## Landing capability (3796–3863, after the law selection, this frame's modes)

`ownApLand` = own AP && (LAND armed || LAND) this frame; `oppApLand` = `ap_opp_engaged` && (opposite A bus discrete
word 3 bit 20 (LAND armed) or word 4 bit 14 (LAND), each with its word's SSM normal).

Equipment ("inop" = its complement; FWC/PFD/BSCU validity are constants true in `FlyByWireInterface.cpp`):

- LAND 2: !ap_inop, an FWC valid, both PFDs valid, both ILS valid, the selected RA word normal.
- LAND 3 FAIL PASSIVE: LAND 2 equipment, !athr_inop, both RAs valid.
- LAND 3 FAIL OP: both FWCs, split power supply (no bus tie), both PFDs, both ELACs authorising the AP, both RAs, both
  ILS, all three ADRs, all three IRs, both BSCUs, both FACs with rudder control.

Capability:

- LAND 3 FAIL OP = held || (FAIL OP equipment && `ownApLand` && `oppApLand` && A/THR engaged);
- LAND 3 FAIL PASSIVE = held || (FAIL PASSIVE equipment && `ownApLand` && A/THR engaged && !LAND 3 FAIL OP);
- LAND 2 = LAND 2 equipment && `ownApLand` && neither LAND 3;
- below 100 ft RA with `ownApLand || oppApLand` both LAND 3 values are stored (`pLand3FailOp/Pass`) and held, else the
  memories are cleared. LAND 2 is not held, so LAND 2 can be lost (or a downgrade from LAND 3 appear as no capability)
  below 100 ft, while LAND 3 cannot be lost or gained.

Capacity (what the FMA shows): the opposite FMGC's word 4 bits 16/17/18 (SSM normal), unless this FMGC computes it:
FMGC 1 with both APs engaged, or, without both APs, when the opposite FMGC is not master (`!fmgc_opp_priority`); then
the own capability.

## Constants

| What | Value |
|---|---|
| common conditions confirmation | 0.72 s |
| FM loss tolerated in G/S below | 700 ft RA |
| ROLL OUT heading deviation, hold | > 20°, 3 s |
| AP engagement envelope | VLS ≤ IAS ≤ VMAX, pitch −10…22°, bank ≤ 40° |
| AP disengagement after the request drops | 0.2 s |
| AP disengaged on the ground with a lever at or above | 36.7° |
| A/THR instinctive disconnect lock-out | 15 s |
| A/THR pushbutton engagement above | 100 ft RA |
| A/THR armed after SRS takeoff/GA | 0.9 s |
| both levers at IDLE below | 2.6° |
| ECU A/THR engaged mismatch | 0.9 s |
| FD auto disengage | VMAX + 4 kt, VLS − 2 kt |
| V/S target not held | 3 kt from VLS/VMAX, 50 fpm, 0.1° |
| mode reversion pulse, pitch bars flashing | 1 s, 10 s |
| LAND 3 capabilities held below | 100 ft RA |

## Findings and differences from the FCOM (A320 FCOM DSC-22_30-20 "AP/FD/A/THR engagement", public text)

Stock behaviour is kept in the rewrite; nothing below is fixed.

1. **AP disconnection latency**: pressing the AP pushbutton, losing an AP condition (ELAC, FAC rudder, common
   conditions), the opposite AP engaging or a ground reset disengages the AP 0.2 s later (the AP latch resets on
   `!Confirm(request, falling, 0.2 s)`); only the instinctive disconnect is immediate. FCOM: immediate.
2. **Second AP outside approach**: engaging the second AP when neither LAND armed nor LAND nor SRS GA is active resets
   the first AP's request, so both APs are engaged together for 0.2 s (plus one frame for FMGC 1, which sees FMGC 2's
   outputs a frame late). FCOM: the first AP disengages. Dual AP is allowed with LAND armed (ILS APPR), LAND, or SRS
   GA; FCOM also lists ROLL OUT, which here is covered only because LAND stays active in ROLL OUT.
3. **Dual AP and mode synchronisation**: with both APs engaged, `fmgc_opp_mode_sync` is false on both FMGCs (each runs
   its own mode logic, FMGC 2 keeps `fmgc_opp_priority`). FCOM: FMGC 1 is master with both APs engaged.
4. **AP engagement envelope** (VLS ≤ IAS ≤ VMAX, pitch −10…22°, bank ≤ 40°) is an FBW choice, checked only at the
   engagement push; the FCOM states the AP cannot be engaged outside the normal flight envelope and leaves the limits to
   the protections.
5. **AP on the ground**: the request needs `gnd_eng_stop_flt_5s` (both engines stopped on the ground, or 5 s airborne),
   matching the FCOM (AP engagement on ground with engines stopped; in flight 5 s after liftoff); on the ground it is
   disengaged by the first engine start, SRS GA, or a lever at ≥ 36.7° (above FLX/MCT, below TOGA).
6. **A/THR pushbutton disconnect gated by a constant**: the `!LAND` term of the pushbutton disconnect reads
   `rtP_fmgc_ap_fd_logic_output_MATLABStruct` (the all-false default bus) instead of the previous frame's modes, so the
   term is always true: a push disconnects the A/THR in any mode. Same pattern as `cruise_active` and
   `loc_bc_selection` elsewhere.
7. **A/THR engagement** by the pushbutton only above 100 ft RA (or both RAs failed); at takeoff the A/THR is engaged
   (armed) 0.9 s after SRS takeoff engages, not by the lever position itself, so without SRS (no V2, no FD) the takeoff
   levers do not arm it. FCOM: armed when the levers are set to FLX/TOGA at takeoff (to verify against the FCOM text).
   Also engaged on alpha floor (FAC bit 29 edge) and on TCAS mode engagement.
8. **A/THR disconnection** matches the FCOM list: pushbutton, instinctive disconnect, both levers to IDLE (< 2.6°),
   and the FADEC disconnecting (ECU word bit 19 missing for 0.9 s; the FADEC latch also drops on reverse). The 15 s
   instinctive disconnect lock-out lasts until the FMGC restarts, as in the FCOM ("remainder of the flight"); alpha
   floor is then also lost because the A/THR cannot engage.
9. **A/THR needs a speed target**: `athrPossible` requires managed or selected speed control active last frame (targets
   block), so the A/THR cannot be engaged in the frame(s) where neither is active (power-up).
10. **FD transfer**: an FMGC engages its FD with its own FD pushbutton off when the opposite FD pushbutton is on and the
    opposite FMGC's FD is not engaged (the remaining FMGC drives both FDs), as on the aircraft.
11. **Opposite priority when the opposite FMGC is unhealthy**: FMGC 2 with nothing engaged gives priority to (and
    synchronises with) an unhealthy FMGC 1 if FMGC 1's AP/FD/A/THR discretes are false, which they are when it is
    failed; harmless because nothing is engaged, and lost as soon as FMGC 2 engages anything (the FD or A/THR clause
    then fails).
12. **FD auto disengage** checks A/THR *engaged*, not THR mode active (FCOM: A/THR in THR mode, AP off, FD on, speed at
    VLS/VMAX); it is a bus flag that the FCU turns into an FD pushbutton toggle (`fd_auto_deactivate`).
13. **V/S target not held** is computed with the AP only (not FD only) and compares the IR inertial vertical speed
    (labelled `_ft_s`, fpm in practice) with the FCU V/S; with FD only no triple click comes from this term.
14. **Landing capability needs an AP in LAND armed/LAND**: there is no capability with the FD only (FCOM: CAT 1 shown
    with FD only), LAND 3 needs the A/THR engaged (not active), and LAND 3 FAIL OP needs the opposite AP in LAND as
    reported on the opposite bus.
15. **Freeze below 100 ft** applies to LAND 3 FAIL OP / FAIL PASSIVE only; LAND 2 is recomputed every frame, so an AP
    inop or an RA/ILS loss below 100 ft drops a LAND 2 capability while a LAND 3 one is kept (FCOM: the capability is
    frozen below 100 ft RA; to verify which failures still downgrade it).
16. **Equipment inputs that are constants**: FWC, PFD and BSCU validity are wired `true` by `FlyByWireInterface`, so
    LAND 3 FAIL OP in the sim depends on the split power supply, ELACs, RAs, ILS, ADRs, IRs and FAC rudder only.

## Verification

See the end of this file (results of `scripts/check-engagement-corpus.sh` and the mutation test).
