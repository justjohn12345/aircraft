# Managed and open climb/descent modes (CLB, DES, OP CLB, OP DES, EXP CLB, EXP DES)

Source: `tools/fmgc-lab/build/models/stock/FmgcComputer.cpp` (identical to upstream
`fbw-a32nx/src/wasm/fbw_a320/src/model/FmgcComputer.cpp`). All line numbers refer to that file unless prefixed with
another file name. Parameter values come from `docs/params-FmgcComputer.txt`.

Conventions:

- `X_prev` = `FmgcComputer_DWork.Delay_DSTATE.<...>.X`, i.e. the value of the whole `ap_fd_logic` output on the
  previous frame. **Every** reset condition below reads previous-frame modes, never the latches computed earlier in the
  same frame.
- Every latch is `Memory_PreviousInput_X = Logic_table_Y[((S << 1) + R) << 1) + Q]`. All tables used here are
  `0 1 0 0 1 1 0 0 | ...` (hold / reset / set / reset wins), checked for `Logic_table_ns`, `_kw`, `_he`, `_cv`, `_jq`
  (params lines 829–833). The generator writes some indices as `R + (S << 1)`: the term shifted inside is
  always S. All the initial conditions are 0 (`SRFlipFlop_initial_condition_be/jv/p4/lz/oz/pr/eb/jw`, init lines
  485–494).
- Bit numbers are the 1-based numbers passed to `FmgcComputer_MATLABFunction_i` (bit N = `(word >> (N-1)) & 1`).
  The packers (`MATLABFunction_gy` in the FMGC, `_e` in the FCU) put element `i` of the bool vector at 1-based bit
  `i + 11`.

---


> Corrections found while rewriting this group (readable/vertical_climb_descent.cpp is exact against the generated
> code): the engaged-mode reset lists hold 16 entries (15 longitudinal modes plus LAND), not "the other 15"; DES armed
> differs from CLB armed in being also reset by FINAL DES and SRS GA (and CLB armed by CLB) — neither list contains
> SRS TO; all 8 SR tables used here (incl. `_lw`, `_ao`, `_om`) are the default reset-wins table.

## Shared conditions

These appear in several modes and are defined once here. Names in **bold** are used in the formulas.

### Opposite-FMGC mode sync (S1)

Every engage latch in this group has the same first SET term, so that the slave FMGC copies the master's mode from the
opposite FMGC's bus:

```
set = (oppModeBits && oppSync) || (ownPriority && <own engage condition>)
```

- **oppSync** = `rtb_OR2_l` (1547), output `ap_fd_logic.fmgc_opp_mode_sync` (4016). True when this FMGC must follow the
  opposite one. It depends on which unit this is and which AP/FD/A/THR are engaged on each side (1537–1547); see the
  AP/FD engagement document for the details.
- **ownPriority** = `rtb_y_oy = !rtb_OR2_l` (1687).
- The opposite modes are read from `fmgc_opp_bus.discrete_word_1` (active modes) and `discrete_word_3` (armed modes),
  packed at 4807–4858 (see the bit table at the end).

The own-engage branch is ignored completely when `oppSync` is true and vice versa. While `oppSync` is true, this FMGC's
mode is reset only by the general reset paths (S3 and S4), never by its own engage conditions.

### Reset pattern of the engaged modes (S2)

All six engage latches (not the two arm latches) use the same reset:

```
reset = longitudinalModeReset
     || (anyOtherLongModeActive_prev && !setRecently)
setRecently = ConfirmNode(set, isRisingEdge = 0, 0.72 s)
```

- `anyOtherLongModeActive_prev` = OR of every other longitudinal mode on the previous frame: V/S, FPA, ALT, ALT*, the
  other five modes of this group, FINAL DES, G/S*, G/S, SRS (TO), SRS (GA), TCAS, and `lateral_modes.land_active`.
  The exact lists are at 2822–2837 (CLB), 2941–2956 (DES), 3001–3016 (OP CLB), 3047–3062 (OP DES), 3087–3102 (EXP CLB),
  3127–3142 (EXP DES). Each list contains the other 15 modes and leaves out its own mode. FLARE is not in the lists.
- `setRecently`: a ConfirmNode with `isRisingEdge = 0` and 0.72 s
  (`ConfirmNode_timeDelay_nn/dy/ib/ob/io/p = 0.72`, `isRisingEdge_fc/j/ca/ep/b/ef = 0`). Its output goes true at once
  when the set condition is true and goes false 0.72 s after the set condition last dropped. It blocks the reset by
  another mode, so a newly engaged mode is not cancelled by the previous mode that is still active on the
  previous-frame bus.
- **Mode handover** (derived from the above): on frame N the new mode X sets. The old mode Y is still active in the
  output of frame N, because Y's reset reads frame N−1, where X was not active yet. On frame N+1 Y is reset by
  `X_prev`, unless Y's own set condition was true within the last 0.72 s. The output therefore has a one-frame overlap
  in which two longitudinal modes are true. Also, for 0.72 s after its set condition was last true, X cannot be
  cancelled by another mode (only by S3).

### General reset (S3)

**longitudinalModeReset** = `rtb_BusAssignment_m_ap_fd_logic_lateral_mode_reset = rtb_AND10_b` (1553/1559). It is
output both as `lateral_mode_reset` and as `longitudinal_mode_reset` (4031–4032).

```
longitudinalModeReset = !anyApFdEngaged || (risingEdge(engineRunning) && onGround)
```

- **anyApFdEngaged** = `rtb_y_bq` (1548), equal to `apCondition` from 1558 on (output `any_ap_fd_engaged`, 4017):
  own AP (`Delay_DSTATE_p`) || opp AP || own FD (`fdOwnOff`, which despite its name is "own FD engaged", 1420/3871) ||
  opp FD.
- `rtb_Equal_o = !anyApFdEngaged` (1552). It is still that value at 2780 and 2896, where it is a reset term of the
  arm latches.
- engineRunning = `!eng_opp_stop || !eng_own_stop` (1310); rising-edge PulseNode (`PulseNode3_isRisingEdge_f = 1`,
  1550).

### Flight-state and target signals (S4)

| name | definition | lines |
|---|---|---|
| **onGround** | `rtb_y_hy` = FAC discrete word 5: bit 18 && bit 19 && !bit 17 && SSM not FW. (inferred) "on ground": it is used for AP disconnect with TOGA lever on ground (1475) and for the 5 s airborne confirm. The FAC relays LGCIU gear data in these bits (FacComputer.cpp 426–462, 1347–1365); the exact gear meaning of each bit was not resolved. | 1263–1273 |
| **inFlight** | `rtb_OR1_j = !onGround` (not reassigned before 3468) | 1275 |
| **airborne5s** | `rtb_y_ew = ConfirmNode(inFlight, rising, 5 s) || (eng_opp_stop && eng_own_stop && onGround)`. `ConfirmNode1_timeDelay = 5`. The second term lets the modes engage on the ground with both engines stopped (inferred: ground testing). | 1276–1279 |
| **fcuAlt** | `fcu_bus.selected_alt_ft`, A429 value or 0 | e.g. 2757 |
| **indAlt** | `logic.altitude_indicated_ft` (FMGC-selected ADR altitude, assembled at 2406–2407), value or 0 | e.g. 2759 |
| **altCstrApplicable** | `rtb_NOT_oj` (1655), output `alt_cstr_applicable` (4014). See below. | 1640–1659 |
| **altTarget** | `rtb_DataTypeConversion_cm` = `next_alt_cstr_ft` if altCstrApplicable, else fcuAlt. Output `alt_sel_or_cstr` (4015). | 1660–1664 |
| **altAcqArmPossible_prev** | `armed_modes.alt_acq_arm_possible` of the previous frame (latch `Memory_PreviousInput_n`, 2418). SET = FCU bit 13 (ALT value changed) || \|indAlt − fcuAlt\| > 250 ft; RESET = ALT_prev && \|indAlt − fcuAlt\| ≤ 40 ft; initial value 1. In practice it is false while ALT holds the FCU altitude and the pilot has not turned the ALT knob since. | 2408–2421 |
| **altPushEdge** | rising edge of FCU discrete word 1 bit 17 (ALT knob pushed) | 2730–2733, 2803–2806, 2853–2856, 2913–2916 |
| **altPullEdge** | rising edge of FCU word 1 bit 18 (ALT knob pulled) | 2975–2978, 3035–3038 |
| **expedPushEdge** | rising edge of FCU word 1 bit 22 (EXPED pushed) | 3080–3083, 3120–3123 |
| **cstrChanged** | `next_alt_cstr_ft != value on the previous frame` (Simulink DetectChange; two separate copies: `DelayInput1_DSTATE` for CLB, `DelayInput1_DSTATE_n` for DES, both updated at the end of the step, 5111–5112, initial value 0) | 2802, 2937 |
| **navOrLoc_prev** | `nav_active || loc_cpt_active || loc_trk_active` (previous frame) | e.g. 2862–2864 |
| **altOrAltStar_prev** | `alt_acq_active || alt_hold_active` (previous frame) | e.g. 2807 |
| **vertFpValid** | `fms_inputs.vertical_flight_plan_valid` (FlyByWireInterface.cpp 1792, `L:A32NX_FM_VERTICAL_PROFILE_AVAIL`) | |
| **phase** | `fms_inputs.fms_flight_phase` (`L:A32NX_FMGC_FLIGHT_PHASE`). Enum: 0 Preflight, 1 Takeoff, 2 Climb, 3 Cruise, 4 Descent, 5 Approach, 6 Goaround, 7 Done (FmgcComputer_types.h 7–17) | |
| **accAlt** | `fms_inputs.acceleration_alt_ft` = ARINC word from `L:A32NX_FM1_ACC_ALT`, value or 0 (FlyByWireInterface.cpp 949, 1810) | |
| **nextCstr** | `fms_inputs.next_alt_cstr_ft` (`L:A32NX_FG_ALTITUDE_CONSTRAINT`); 0 = no constraint | |
| **manualSpd_prev** | `manual_spd_control_active` of the previous frame (latch `Memory_PreviousInput_cu`, 3480; selected speed) | |

**altCstrApplicable** (1640–1659), using fcuAlt and indAlt read at 1636–1639:

```
levelAtCstr  = altOrAltStar_prev && alt_cstr_applicable_prev && navOrLoc_prev && vertFpValid          (1640)
inRange      = fcuAlt > indAlt ? nextCstr < fcuAlt && (nextCstr + 250 > indAlt || levelAtCstr)        (1647–1649)
                               : nextCstr > fcuAlt && (nextCstr - 250 < indAlt || levelAtCstr)        (1650–1652)
altCstrApplicable = (clb_armed_prev || des_armed_prev || clb_prev || des_prev || levelAtCstr)
                    && nextCstr != 0 && inRange                                                        (1655–1659)
```

So the constraint is the altitude target only in managed modes (CLB/DES armed or active, or levelled at a
constraint), and only when it lies between the aircraft and the FCU altitude, or at most 250 ft on the wrong side of
the aircraft.

---

## CLB armed  (`armed_modes.clb_armed`, latch `Memory_PreviousInput_ma`, FmgcComputer.cpp:2761)

**Set (arm)**: any of:

- **sync**: opposite FMGC has CLB armed: opp `discrete_word_3` bit 24 && oppSync (2721–2724).
- ownPriority (S1) && one of:
  - **armedLevelAtCstr** (`rtb_NOT_b`, 2736–2744): all of
    - inFlight (`rtb_OR1_j`),
    - phase ≠ Descent (4) and ≠ Approach (5) (`EnumeratedConstant1_Value_f = 4`, `EnumeratedConstant2_Value_p = 5`),
    - nav_active_prev (NAV only, not LOC),
    - fcuAlt > indAlt (`rtb_y_o4`, 2725–2729),
    - vertFpValid,
    - altOrAltStar_prev && altCstrApplicable.
    The code ORs in a second term, `altPushEdge && altOrAltStar_prev && altCstrApplicable && |altTarget − indAlt| ≤ 250`
    (`CompareToConstant2_const_d = 250`). It is a subset of the first term and so has no effect (see Notes).
  - **preArmForTakeoff** (`rtb_GreaterThan3 && rtb_y_o4`, 2745–2756):
    `(onGround || pitch_goaround_prev || pitch_takeoff_prev)` && `accAltBelowTargets`, where
    `accAltBelowTargets = accAlt != 0 && accAlt < fcuAlt && (accAlt < nextCstr || nextCstr == 0) && true`
    (2751–2755). The last factor, `rtb_y_bn = !rtP_fmgc_ap_fd_logic_output_MATLABStruct.any_lateral_mode_engaged ||
    rwy_loc_submode_active_new`, reads a **constant default struct** whose field is `false`
    (FmgcComputer_data.cpp 3ff), so it is always true.

```
clbArmSet = oppClbArmed && oppSync
         || ownPriority && ( inFlight && phase∉{DES,APPR} && nav_prev && fcuAlt>indAlt && vertFpValid
                             && altOrAltStar_prev && altCstrApplicable
                           || (onGround || SRS_GA_prev || SRS_TO_prev) && accAltBelowTargets )
```

**Reset (disarm)**: any of (2761–2780):

- `(onGround || pitch_goaround_prev || pitch_takeoff_prev) && !accAltBelowTargets`, the complement of the pre-arm term;
- any of V/S, FPA, EXP CLB, EXP DES, CLB, DES, G/S*, G/S, OP CLB, OP DES, TCAS on the previous frame. **The arm is
  cleared on the frame after CLB engages**, and while any of these modes is active;
- altOrAltStar_prev && !altCstrApplicable (`rtb_OR_b = !rtb_NOT_oj`, 2598): levelling at the FCU altitude, not at a
  constraint;
- fcuAlt < indAlt (2757–2760, strict);
- altOrAltStar_prev && (!altCstrApplicable || !nav_active_prev || !vertFpValid);
- phase == Descent (4) || phase == Approach (5) (`EnumeratedConstant3_Value = 4`, `EnumeratedConstant4_Value = 5`);
- !anyApFdEngaged (`rtb_Equal_o`).

Unlike the engaged modes, the arm latch has no 0.72 s guard and uses `!anyApFdEngaged` rather than the full
`longitudinalModeReset` (no engine-start term).

**Notes**

- In flight, CLB is armed only while levelled (ALT or ALT*) at an applicable constraint with NAV engaged: the blue
  "CLB" under "ALT CST". It is not armed during OP CLB or V/S. On the ground and in SRS it is pre-armed when the
  acceleration altitude is below the FCU altitude and below the next constraint. The pre-arm drops as soon as that
  stops being true, because the same expression is the reset.
- The ALT-push sub-term (2741–2744) is redundant: `A && C || (P && A && C && D) == A && C`. The designer probably
  meant "push ALT within 250 ft of a constraint arms CLB" as a separate path, but it is subsumed (inferred).
- `accAlt` is always the all-engines ACC ALT (`A32NX_FM1_ACC_ALT`), also in go-around (see Open questions).
- Old Stateflow (`sCLB`, AutopilotStateMachine.cpp 3523–3527 at `afff8d290^`): similar idea, but its altitude test
  used a 50 ft margin (`distance_m > 50`). The new code uses a strict 0 ft comparison.

---

## DES armed  (`armed_modes.des_armed`, latch `Memory_PreviousInput_nt`, FmgcComputer.cpp:2875)

This latch is not in `docs/latches.tsv`'s Logic_table output column because it is assigned to the output only at 4012.

**Set (arm)**: any of:

- **sync**: opp `discrete_word_3` bit 25 (DES armed) && oppSync (2844–2847).
- ownPriority && all of (2859–2870):
  - phase ≠ Takeoff (1) and ≠ Goaround (6) (`EnumeratedConstant1_Value_c = 1`, `EnumeratedConstant2_Value_pi = 6`),
  - fcuAlt < indAlt (2848–2852),
  - navOrLoc_prev (NAV, LOC* or LOC),
  - vertFpValid,
  - altOrAltStar_prev && altCstrApplicable. As for CLB armed, an OR'ed ALT-push sub-term with `|altTarget − indAlt|
    ≤ 250` (`CompareToConstant2_const_e`) is a subset of this term and has no effect.

```
desArmSet = oppDesArmed && oppSync
         || ownPriority && phase∉{TO,GA} && fcuAlt<indAlt && navOrLoc_prev && vertFpValid
                        && altOrAltStar_prev && altCstrApplicable
```

**Reset (disarm)**: any of (2876–2896):

- any of V/S, FPA, EXP CLB, EXP DES, DES, G/S*, G/S, FINAL DES, SRS GA, OP CLB, OP DES, TCAS on the previous frame
  (note: **not** CLB and not SRS TO, unlike CLB armed);
- fcuAlt > indAlt (strict; read at 2871–2874);
- !navOrLoc_prev;
- phase == Takeoff (1) || Climb (2) || Goaround (6) (`EnumeratedConstant_Value_b = 1`, `EnumeratedConstant5_Value = 2`,
  `EnumeratedConstant6_Value = 6`);
- !vertFpValid;
- altOrAltStar_prev && !altCstrApplicable;
- !anyApFdEngaged (`rtb_Equal_o`).

**Notes**

- DES armed means: levelled at a descent constraint, with the FCU altitude below. There is no inFlight requirement
  (CLB armed has one) and no pre-arm on the ground.
- DES may be armed in phase Cruise or Preflight/Done (only TO, CLB and GA reset it). CLB armed is reset only in
  DES/APPR.

---

## CLB  (`longitudinal_modes.clb_active`, latch `Memory_PreviousInput_ec`, FmgcComputer.cpp:2842)

**Set (engage)**: `rtb_NOT_b` (2821), any of:

- **sync**: opp word 1 bit 11 (CLB/OP CLB/EXP CLB) && !bit 14 (OPEN) && !bit 24 (EXPED) && oppSync (2782–2791).
- **ownClb** (`rtb_y_nu`, 2812–2820): all of
  - ownPriority,
  - altAcqArmPossible_prev,
  - airborne5s,
  - !land_active_prev && !gs_trk_active_prev,
  - nav_active_prev (NAV only),
  - vertFpValid,
  - fcuAlt > indAlt (`rtb_GreaterThan3`, 2792–2796),
  - phase ≠ Descent (4) and ≠ Approach (5) (`EnumeratedConstant_Value_pq = 4`, `EnumeratedConstant1_Value_iv = 5`),
  - and a **trigger**:
    - **clbArmTrigger** (`rtb_y_fi`, 2801): `clb_armed_prev && (accAltCrossedUp || cstrChanged)`, where
      `accAltCrossedUp` = rising edge of `indAlt > accAlt` (2797–2800, `PulseNode_isRisingEdge_bg = 1`); **or**
    - **altPushEdge** (2803–2806) && (`!altOrAltStar_prev` || (`!clb_armed_prev` && (`!altCstrApplicable` ||
      `|altTarget − indAlt| > 250`))) (2807–2811, 2819–2820; `CompareToConstant2_const_a = 250`).

```
clbSet = oppClb && oppSync
      || ownPriority && altAcqArmPossible_prev && airborne5s && !LAND_prev && !GS_prev && nav_prev
         && vertFpValid && fcuAlt > indAlt && phase∉{DES,APPR}
         && ( clb_armed_prev && (risingEdge(indAlt > accAlt) || cstrChanged)
            || altPushEdge && (!altOrAltStar_prev
                               || !clb_armed_prev && (!altCstrApplicable || |altTarget − indAlt| > 250)) )
```

**Reset (disengage)**: pattern S2 (2822–2841): `longitudinalModeReset || (anyOther_prev && !ConfirmNode(clbSet, falling,
0.72 s))` with the list V/S, FPA, ALT, ALT*, DES, EXP CLB, EXP DES, FINAL DES, G/S*, G/S, OP CLB, OP DES, SRS TO,
SRS GA, TCAS, LAND.

In practice CLB is left through the engagement of another mode. The relevant ones and where they come from:

- **CLB → OP CLB** when the phase becomes Descent/Approach, NAV is lost, or the vertical flight plan becomes invalid
  (OP CLB set term `opClbReversion`, see OP CLB);
- **CLB → V/S** when the FCU altitude is set below the aircraft (`longitudinal_mode_reversion_vs`, 2659–2661, see
  "Reversion to V/S");
- CLB → ALT* on capture; CLB → OP CLB/EXP CLB/V/S/FPA on the FCU.

**Notes**

- CLB engages from the arm only at the instant of crossing ACC ALT upward (one-frame pulse), or on any change of
  `next_alt_cstr_ft` while armed. **If NAV is not engaged at that instant, CLB does not engage and OP CLB does instead**
  (OP CLB `accAltOpClbTrigger`). If CLB gets armed after ACC ALT has been crossed, only a constraint change or an ALT
  push engages it.
- `cstrChanged` has no altitude or phase guard beyond the list above. With CLB pre-armed in SRS (NAV engaged, more
  than 5 s airborne), a change of `next_alt_cstr_ft` below ACC ALT (e.g. sequencing a constrained waypoint, or the FMS
  rewriting the L:var) would engage CLB before ACC ALT (inferred from the code; not checked against a recording).
  See Open questions.
- Engagement is strict `fcuAlt > indAlt` (old Stateflow `condition.CLB`: > 50 ft and phase ≥ CLIMB,
  AutopilotStateMachine.cpp 4046 at `afff8d290^`). The new code does not require phase ≥ Climb, only ≠ Descent and
  ≠ Approach. Note also that phase Goaround (6) is not excluded here (it was in the old chart).
- `altAcqArmPossible_prev` blocks every engagement in this group while ALT holds the FCU altitude (within 40 ft) and
  the knob has not been turned since. This matches the FCOM behaviour "pushing/pulling ALT without changing the
  altitude has no effect" (inferred match).
- The set has no `!clb_active_prev` term (DES has `!des_active_prev`), so the set may stay true while CLB is active;
  harmless for an SR latch.

---

## DES  (`longitudinal_modes.des_active`, latch `Memory_PreviousInput_b3`, FmgcComputer.cpp:2961)

**Set (engage)**: `rtb_OR_b` (2940), any of:

- **sync**: opp word 1 bit 12 (DES/OP DES/EXP DES) && !bit 14 && !bit 24 && oppSync (2898–2907).
- **ownDes** (`rtb_y_nu`, 2922–2939): all of
  - ownPriority, altAcqArmPossible_prev, airborne5s,
  - none of G/S, LAND, **DES**, FINAL DES, SRS GA, SRS TO on the previous frame,
  - navOrLoc_prev (NAV, LOC* or LOC),
  - vertFpValid,
  - fcuAlt < indAlt (`rtb_y_o4`, 2908–2912),
  - phase ∉ {Takeoff (1), Climb (2), Goaround (6)} (`EnumeratedConstant_Value_m = 1`, `EnumeratedConstant1_Value_k = 2`,
    `EnumeratedConstant2_Value_pw = 6`),
  - and a **trigger**:
    - `des_armed_prev && cstrChanged` (2936–2937, separate DetectChange state `DelayInput1_DSTATE_n`); **or**
    - altPushEdge (FCU bit 17, 2913–2916) && (`!altOrAltStar_prev` || (`!des_armed_prev` && (`!altCstrApplicable` ||
      `|altTarget − indAlt| > 250`))) (2917–2919, 2938–2939; `CompareToConstant2_const_l = 250`;
      `rtb_OR_b` here is still `!altCstrApplicable` from 2598).

```
desSet = oppDes && oppSync
      || ownPriority && altAcqArmPossible_prev && airborne5s
         && !(GS || LAND || DES || FINAL || SRS_GA || SRS_TO)_prev && navOrLoc_prev && vertFpValid
         && fcuAlt < indAlt && phase∉{TO,CLB,GA}
         && ( des_armed_prev && cstrChanged
            || altPushEdge && (!altOrAltStar_prev
                               || !des_armed_prev && (!altCstrApplicable || |altTarget − indAlt| > 250)) )
```

**Reset (disengage)**: pattern S2 (2941–2960), list V/S, FPA, ALT, ALT*, CLB, EXP CLB, EXP DES, FINAL DES, G/S*, G/S,
OP CLB, OP DES, SRS TO, SRS GA, TCAS, LAND.

Main exits: **DES → V/S** (2646–2653 and 2656–2658, see "Reversion to V/S") when the phase becomes TO/CLB/GA, when
NAV/LOC is lost, when the vertical flight plan becomes invalid, or when the FCU altitude is set above the aircraft;
DES → ALT*, G/S*, FINAL DES; DES → OP DES/EXP DES/V/S/FPA on the FCU.

**Notes**

- There is no automatic engagement from "DES armed" other than a change of the next constraint. There is no
  equivalent of CLB's ACC ALT trigger, and no top-of-descent trigger: DES starts only on ALT push or on sequencing a
  constraint while DES is armed. DES can engage in phase Cruise (early descent on ALT push).
- DES accepts LOC*/LOC as the lateral mode, CLB requires NAV.
- Asymmetric loss-of-NAV handling: CLB → OP CLB, but DES → V/S. This matches FCOM and the FBW docs ("V/S on loss of
  … NAV in DES") (inferred match).
- Old Stateflow `condition.DES` (AutopilotStateMachine.cpp 4052): FCU 50 ft below and not in SRS/G/S/LAND. The DES
  state reverted to V/S when the FCU was more than 40 ft above. The new code uses 0 ft for both.

---

## OP CLB  (`longitudinal_modes.op_clb_active`, latch `Memory_PreviousInput_ae`, FmgcComputer.cpp:3021)

**Set (engage)**: `rtb_OR_b` (3000), any of:

- **sync**: opp word 1 bit 11 (CLB family) && bit 14 (OPEN) && oppSync (2963–2969).
- **ownOpClb** (`rtb_y_nu`, 2998–2999): ownPriority && altAcqArmPossible_prev && airborne5s && !land_active_prev &&
  fcuAlt > indAlt (`rtb_GreaterThan3`, 2970–2974) && **opClbTrigger**, where `opClbTrigger` (`rtb_y_fi`, 2996–2997) is
  any of:
  - **altPullEdge** (FCU bit 18, 2975–2978);
  - **opClbReversion** (`rtb_y_o4`, 2986–2989), any of:
    - `clb_active_prev && (phase == Descent (4) || phase == Approach (5) || !nav_active_prev || !vertFpValid)`
      (`EnumeratedConstant_Value_c = 4`, `EnumeratedConstant1_Value_c5 = 5`);
    - `ConfirmNode(manualSpd_prev && (exp_clb_prev || pitch_goaround_prev || pitch_takeoff_prev), rising, 0.72 s)`
      (2980–2985; `ConfirmNode_isRisingEdge_jo = 1`, `timeDelay_jo = 0.72`): a selected speed for 0.72 s in EXP CLB,
      SRS or SRS GA;
  - **accAltOpClbTrigger** (2990–2997): rising edge of `indAlt > accAlt` (separate PulseNode state `sf_MATLABFunction_lva`)
    && accAlt != 0 && clb_armed_prev && (!nav_active_prev || !vertFpValid).

```
opClbSet = oppOpClb && oppSync
        || ownPriority && altAcqArmPossible_prev && airborne5s && !LAND_prev && fcuAlt > indAlt
           && ( altPullEdge
              || CLB_prev && (phase∈{DES,APPR} || !nav_prev || !vertFpValid)
              || Confirm↑0.72s(manualSpd_prev && (EXP_CLB || SRS_GA || SRS_TO)_prev)
              || risingEdge(indAlt > accAlt) && accAlt != 0 && clb_armed_prev && (!nav_prev || !vertFpValid) )
```

**Reset (disengage)**: pattern S2 (3001–3020), list V/S, FPA, ALT, ALT*, DES, EXP CLB, EXP DES, FINAL DES, G/S*, G/S,
CLB, OP DES, SRS TO, SRS GA, TCAS, LAND.

Main exits: OP CLB → V/S when the FCU altitude is set below the aircraft (2659–2661); OP CLB → ALT*; OP CLB → CLB on
ALT push (CLB set conditions); OP CLB → EXP CLB on EXPED push.

**`longitudinal_mode_reversion_op_clb`** (output, 4038) = `rtb_y_bq = op_clb_active (new) && opClbReversion` (3023).
It is true while OP CLB is active and the reversion condition holds (the manual-speed term usually lasts only one
frame: once OP CLB is active, EXP CLB/SRS are reset on the next frame; the CLB term also drops on the next frame). It is
the OP CLB counterpart of `longitudinal_mode_reversion_vs`. **No consumer**: it is not read elsewhere in
FmgcComputer.cpp, and nothing in `fbw-a32nx/src` outside `model/` references it (it only reaches the output struct and
the recorder).

**Notes**

- OP CLB needs no NAV, flight plan or phase, only FCU above the aircraft, more than 5 s airborne, and "ALT arm possible".
- The 0.72 s manual-speed confirm in SRS/SRS GA means a selected speed in SRS changes the mode to OP CLB. Whether
  `manual_spd_control_active` can actually become true during SRS depends on latch `cu` (3480), which is not analysed
  here (open question).
- Old Stateflow: OP CLB was a state with an `EXPED_mode_active` sub-flag toggled by EXPED push (AutopilotStateMachine.cpp
  1980–1997). The new model has separate EXP CLB/EXP DES latches.

---

## OP DES  (`longitudinal_modes.op_des_active`, latch `Memory_PreviousInput_ev`, FmgcComputer.cpp:3067)

**Set (engage)**: `rtb_OR_b` (3046), any of:

- **sync**: opp word 1 bit 12 (DES family) && bit 14 (OPEN) && oppSync (3024–3030).
- **ownOpDes** (3044–3045): ownPriority && altAcqArmPossible_prev && airborne5s && !land_active_prev && fcuAlt < indAlt
  (3031–3034) && (altPullEdge (FCU bit 18, 3035–3038) ||
  `ConfirmNode(manualSpd_prev && exp_des_prev, rising, 0.72 s)` (3039–3043; `isRisingEdge_fq = 1`, `timeDelay_lp = 0.72`)).

```
opDesSet = oppOpDes && oppSync
        || ownPriority && altAcqArmPossible_prev && airborne5s && !LAND_prev && fcuAlt < indAlt
           && (altPullEdge || Confirm↑0.72s(manualSpd_prev && EXP_DES_prev))
```

**Reset (disengage)**: pattern S2 (3047–3066), list V/S, FPA, ALT, ALT*, CLB, EXP CLB, EXP DES, FINAL DES, G/S*, G/S,
OP CLB, DES, SRS TO, SRS GA, TCAS, LAND.

Main exits: OP DES → V/S when the FCU altitude is set above the aircraft (2656–2658); → ALT*, G/S*, FINAL DES; → DES on
ALT push; → EXP DES on EXPED push.

**Notes**

- No reversion from DES into OP DES (DES reverts to V/S). There is no OP DES counterpart of
  `longitudinal_mode_reversion_op_clb`.

---

## EXP CLB  (`longitudinal_modes.exp_clb_active`, latch `Memory_PreviousInput_mx`, FmgcComputer.cpp:3107)

**Set (engage)**: `rtb_OR_b` (3086), any of:

- **sync**: opp word 1 bit 11 (CLB family) && bit 24 (EXPED) && oppSync (3069–3075).
- **own** (3084–3085): ownPriority && altAcqArmPossible_prev && airborne5s && !land_active_prev && fcuAlt > indAlt
  (3076–3079) && expedPushEdge (FCU bit 22, 3080–3083).

**Reset (disengage)**: pattern S2 (3087–3106), list V/S, FPA, ALT, ALT*, DES, OP CLB, EXP DES, FINAL DES, G/S*, G/S,
CLB, OP DES, SRS TO, SRS GA, TCAS, LAND.

Main exits: EXP CLB → OP CLB after a selected speed for 0.72 s (OP CLB `opClbReversion`) or on ALT pull; → CLB on ALT
push; → V/S on FCU below the aircraft (2659–2661); → ALT*.

**Notes**

- A second EXPED push while in EXP CLB only sets the latch again; there is no toggle (the old chart toggled the EXPED
  flag, AutopilotStateMachine.cpp 1989–1991).
- The rising edge of `exp_clb || exp_des` (new values, 3501–3503) is one of the set terms of the managed-speed latch
  `auto_spd_control_active` (3509–3521), so EXPED forces managed speed (inferred from the latch; speed logic not
  analysed in detail).

---

## EXP DES  (`longitudinal_modes.exp_des_active`, latch `Memory_PreviousInput_o`, FmgcComputer.cpp:3147)

**Set (engage)**: `rtb_OR_b` (3126), any of:

- **sync**: opp word 1 bit 12 (DES family) && bit 24 (EXPED) && oppSync (3109–3115).
- **own** (3124–3125): ownPriority && altAcqArmPossible_prev && airborne5s && !land_active_prev && fcuAlt < indAlt
  (3116–3119) && expedPushEdge (FCU bit 22, 3120–3123).

**Reset (disengage)**: pattern S2 (3127–3146), list V/S, FPA, ALT, ALT*, CLB, EXP CLB, OP DES, FINAL DES, G/S*, G/S,
OP CLB, DES, SRS TO, SRS GA, TCAS, LAND.

Main exits: EXP DES → OP DES after a selected speed for 0.72 s, or on ALT pull; → DES on ALT push; → V/S on FCU above
(2656–2658); → ALT*, G/S*.

---

## Reversion to V/S (exit path for this group)

`longitudinal_mode_reversion_vs` = `rtb_Compare_ji` at 2646–2672 (output 4036–4037). It is a set term of the V/S/FPA
latch (`rtb_y_oy && ... || rtb_Compare_ji`, 2692–2694). V/S then resets the mode of this group on the next frame via
S2. The terms that involve this group's modes are:

| term | lines |
|---|---|
| `des_prev && (phase ∈ {TO (1), CLB (2), GA (6)} || !navOrLoc_prev || !vertFpValid)` | 2646–2653 (`EnumeratedConstant_Value_p = 1`, `_m = 2`, `_h = 6`) |
| `(des || exp_des || op_des)_prev && fcuAlt − indAlt > 0` | 2656–2659 (`CompareToConstant1_const_o = 0`; fcuAlt − indAlt computed at 2630–2634) |
| `(clb || exp_clb || op_clb)_prev && fcuAlt − indAlt < 0` | 2659–2662 (`CompareToConstant2_const_g = 0`) |
| `(des || exp_des || op_des || final_des)_prev && VLS − 2 > CAS && !apOwn && athrOwn && Constant_Value_m` | 2662–2666 (`Bias_Bias_e = −2`) |
| `(clb || exp_clb || op_clb)_prev && CAS > VMAX + 4 && !apOwn && athrOwn && Constant_Value_m` | 2667–2671 (`Bias1_Bias = 4`) |

`Constant_Value_m = 0`, so **the two speed-protection reversions (FD-only, A/THR on, speed below VLS−2 in descent or
above VMAX+4 in climb) are disabled in this build**. Both thresholds are strict 0 ft, compared with 40 ft in the old
chart.

---

## Nearby unnamed latches

| latch | line | what it is |
|---|---|---|
| `Memory_PreviousInput_no` | 2596 | **"at cruise altitude" latch.** SET = \|`fms_inputs.cruise_alt_ft` − indAlt\| < 60 ft (`CompareToConstant1_const_a = 60`, 2591–2594); RESET = !alt_hold_active_prev (2595). It feeds `longitudinal_modes.fma_dash_display = alt_hold_active (new) && !altCstrApplicable && no` (2598–2601, 2609), which FMA.tsx shows as **"ALT CRZ"** (FMA.tsx 929–930; packed as FMGC word 1 bit 26). It does not feed any mode in this group. |
| `Memory_PreviousInput_h` | 3311 | **TCAS ALT_ACQ submode.** SET = `rtb_AND1_c0` = the ALT* capture-distance condition computed at 2513–2517 (also output `tcas_alt_acq_cond`, 4044); RESET = \|`u_lyj` − altTarget\| ≥ 250 ft (`CompareToConstant_const_g = 250`; `u_lyj` = altTarget frozen while `active_tcas_submode_prev == ALT_ACQ`, 3301–3303) \|\| !TCAS vertical RA bit 20 \|\| !alt_acq_armed_prev. |
| `Memory_PreviousInput_cp` | 3318 | **TCAS ALT_HOLD submode.** SET = `active_tcas_submode_prev == ALT_ACQ (1)` && `rtb_AND8` (= ConfirmNode(\|altTarget − indAlt\| < 40 ft, 0.8 s), 2559–2561, 2610; output `tcas_alt_hold_cond`); RESET = !alt_acq_armed_prev \|\| !TCAS RA bit 20. `h`/`cp` select `active_tcas_submode` = ALT_ACQ / ALT_HOLD / VS (3320–3326). They belong to the TCAS mode, not to this group. |

---

## Bit tables used here

**FCU `fcu_discrete_word_1`** (FcuComputer.cpp 1340–1359, packed by `_e` with element i at 1-based bit i + 11):
11 SPD/MACH pushed, 12 SPD/MACH pulled, 13 ALT value changed, 14 (MTrig on a value change, `rtb_Equal6`), 15 (MTrig on
the Mach value change), 16 V/S-FPA pushed, **17 ALT pushed**, **18 ALT pulled**, 19 V/S-FPA pulled, 20 metric alt,
21 SPD/MACH switch pushed, **22 EXPED pushed**, 23 APPR pushed, 24 HDG-V/S (not TRK-FPA), 25 TRK-FPA.
Consistency check: FMGC line 2717 uses bit 24 to split the V/S-FPA latch into V/S (bit set) and FPA (bit clear).

**FMGC `discrete_word_1`** (4836–4858): **11 CLB/OP CLB/EXP CLB**, **12 DES/OP DES/EXP DES**, 13 spare, **14 OP CLB/OP DES**,
15 SRS TO, 16 SRS GA, 17 V/S, 18 FPA, 19 ALT*/ALT, 20 ALT/G/S, 21 ALT*/G/S*, 22 G/S*/G/S, 23 FINAL, **24 EXP CLB/EXP DES**,
25 FLARE, 26 ALT CRZ dash, 27 preset speed, 28 spare, 29 TLA to GA.

**FMGC `discrete_word_3`** (4807–4826): 11 HDG/TRK preset, 12 ALT armed, 13 ALT arm possible, 14 NAV armed, 16 LOC armed,
19 backbeam, 20 LAND armed, 22 G/S armed, 23 FINAL armed, **24 CLB armed**, **25 DES armed**.

---

## Signal dictionary

| name | generated temp + line | meaning | units |
|---|---|---|---|
| oppSync | `rtb_OR2_l` 1547 | follow the opposite FMGC's modes (`fmgc_opp_mode_sync`) | bool |
| ownPriority | `rtb_y_oy` 1687 | `!oppSync` | bool |
| anyApFdEngaged | `rtb_y_bq` 1548 / `apCondition` 1558 | any AP or FD (own or opp) engaged | bool |
| longitudinalModeReset | `rtb_BusAssignment_m_ap_fd_logic_lateral_mode_reset` 1559 (= `rtb_AND10_b` 1553) | `!anyApFdEngaged \|\| (engine start edge && onGround)` | bool |
| onGround | `rtb_y_hy` 1273 | FAC word 5 bits 18 && 19 && !17 && SSM OK (inferred: on ground) | bool |
| inFlight | `rtb_OR1_j` 1275 | `!onGround` | bool |
| airborne5s | `rtb_y_ew` 1278 | in flight for ≥ 5 s, or both engines stopped on ground | bool |
| altCstrApplicable | `rtb_NOT_oj` 1655 | next constraint is the altitude target | bool |
| !altCstrApplicable | `rtb_OR_b` 2598 (valid until 2940) | complement | bool |
| altTarget | `rtb_DataTypeConversion_cm` 1661/1663 | constraint or FCU altitude (`alt_sel_or_cstr`) | ft |
| levelAtCstr | `rtb_AND_j` 1640 | ALT/ALT* at an applicable constraint with NAV/LOC | bool |
| fcuAboveForClbArm | `rtb_y_o4` 2729 | fcuAlt > indAlt | bool |
| armedLevelAtCstr | `rtb_NOT_b` 2736 | in-flight CLB arm condition | bool |
| groundOrSrs | `rtb_GreaterThan3` 2745 | onGround \|\| SRS GA_prev \|\| SRS TO_prev | bool |
| alwaysTrue | `rtb_y_bn` 2749 | `!const_false \|\| rwy_loc` = true | bool |
| accAltBelowTargets | `rtb_y_o4` 2751 | accAlt ≠ 0, < fcuAlt, < next constraint (or none) | bool |
| clbArmSet | `rtb_Compare_ji` 2756 | CLB arm set | bool |
| oppClb | `rtb_Compare_ji` 2790–2791 | opp CLB (not OPEN, not EXPED) && oppSync | bool |
| fcuAboveForClb | `rtb_GreaterThan3` 2796 | fcuAlt > indAlt | bool |
| accAltCrossedUp | `rtb_y_bn` 2799 | rising edge of indAlt > accAlt | pulse |
| clbArmTrigger | `rtb_y_fi` 2801 | clb_armed_prev && (accAltCrossedUp \|\| cstrChanged) | bool |
| altPushEdge (CLB) | `rtb_AND10_b` 2805 | rising edge of FCU bit 17 | pulse |
| altOrAltStar_prev | `rtb_y_bn` 2807 / `rtb_y_d` 2917 | ALT* \|\| ALT on the previous frame | bool |
| awayFromCstr | `rtb_y_bq` 2811 | !altCstrApplicable \|\| \|altTarget − indAlt\| > 250 | bool |
| ownClb | `rtb_y_nu` 2812 | own CLB engage condition | bool |
| clbSet | `rtb_NOT_b` 2821 | CLB set | bool |
| setRecently (CLB) | `rtb_y_nu` 2839 | ConfirmNode(clbSet, falling, 0.72 s) | bool |
| clbReset | `rtb_Compare_ji` 2841 | CLB reset | bool |
| oppDesArmed | `rtb_Compare_ji` 2847 | opp DES armed && oppSync | bool |
| fcuBelowForDesArm | `rtb_GreaterThan3` 2852 | fcuAlt < indAlt | bool |
| altPushEdge (DES arm) | `rtb_y_bq` 2855 | rising edge of FCU bit 17 | pulse |
| desArmSet | `rtb_Compare_ji` 2859 | DES arm set | bool |
| oppDes | `rtb_Compare_ji` 2906–2907 | opp DES (not OPEN, not EXPED) && oppSync | bool |
| fcuBelowForDes | `rtb_y_o4` 2912 | fcuAlt < indAlt | bool |
| altPushEdge (DES) | `rtb_y_nu` 2915 | rising edge of FCU bit 17 | pulse |
| notDesArmed | `rtb_y_bq` 2919 | !des_armed_prev | bool |
| ownDes | `rtb_y_nu` 2922 | own DES engage condition | bool |
| desSet | `rtb_OR_b` 2940 | DES set | bool |
| oppOpClb | `rtb_Compare_ji` 2968–2969 | opp CLB family && OPEN && oppSync | bool |
| fcuAboveForOpClb | `rtb_GreaterThan3` 2974 | fcuAlt > indAlt | bool |
| altPullEdge (OP CLB) | `rtb_y_bq` 2977 | rising edge of FCU bit 18 | pulse |
| manualSpdInExpClbOrSrs | `rtb_y_fi` 2983 | Confirm↑0.72 s(manualSpd_prev && (EXP CLB \|\| SRS GA \|\| SRS TO)_prev) | bool |
| opClbReversion | `rtb_y_o4` 2986 | CLB lost NAV/FP/phase, or selected speed in EXP CLB/SRS | bool |
| accAltCrossedUp (OP CLB) | `rtb_y_d` 2992 | rising edge of indAlt > accAlt (own PulseNode) | pulse |
| noNavOrFp | `rtb_y_bn` 2994 | !nav_prev \|\| !vertFpValid | bool |
| opClbTrigger | `rtb_y_fi` 2996 | pull \|\| reversion \|\| ACC ALT without NAV | bool |
| opClbSet | `rtb_OR_b` 3000 | OP CLB set | bool |
| reversionOpClb | `rtb_y_bq` 3023 | output `longitudinal_mode_reversion_op_clb` | bool |
| oppOpDes | `rtb_Compare_ji` 3029–3030 | opp DES family && OPEN && oppSync | bool |
| opDesTrigger | `rtb_y_o4` 3037/3043 | ALT pull edge \|\| Confirm↑0.72 s(manualSpd_prev && EXP DES_prev) | bool |
| opDesSet | `rtb_OR_b` 3046 | OP DES set | bool |
| oppExpClb | `rtb_Compare_ji` 3074–3075 | opp CLB family && EXPED && oppSync | bool |
| expedPushEdge (EXP CLB) | `rtb_y_o4` 3082 | rising edge of FCU bit 22 | pulse |
| expClbSet | `rtb_OR_b` 3086 | EXP CLB set | bool |
| oppExpDes | `rtb_Compare_ji` 3114–3115 | opp DES family && EXPED && oppSync | bool |
| expedPushEdge (EXP DES) | `rtb_y_o4` 3122 | rising edge of FCU bit 22 | pulse |
| expDesSet | `rtb_OR_b` 3126 | EXP DES set | bool |
| reversionVs | `rtb_Compare_ji` 2646 | output `longitudinal_mode_reversion_vs` | bool |
| fcuMinusInd | `rtb_DataTypeConversion2_m` 2634 | fcuAlt − indAlt (used by the V/S reversion) | ft |
| atCruiseAlt | `Memory_PreviousInput_no` 2596 | within 60 ft of FMS cruise alt, held while in ALT | bool |
| altAcqArmPossible | `Memory_PreviousInput_n` 2418 | FCU alt changed or > 250 ft away; reset when ALT holds within 40 ft | bool |

---

## Open questions

1. **CLB engaged by a constraint change in SRS.** `clbArmTrigger` accepts `cstrChanged` with no altitude or mode guard.
   With CLB pre-armed in SRS, NAV engaged and more than 5 s airborne, any change of `A32NX_FG_ALTITUDE_CONSTRAINT`
   below ACC ALT should engage CLB early. Check in the FDR corpus whether `clb_active` ever rises while
   `pitch_takeoff_active` is true and indicated altitude is below `acceleration_alt_ft`. The same applies to DES armed
   and a constraint change while levelled at a constraint that does not apply any more.
2. **Go-around ACC ALT.** CLB pre-arm and the ACC ALT crossing use `fms_inputs.acceleration_alt_ft`, which always comes
   from `L:A32NX_FM1_ACC_ALT` (FlyByWireInterface.cpp 949). Unresolved whether the FMS writes the GA ACC ALT into that
   variable in the GA phase. Also, CLB engagement excludes only phases 4 and 5, so CLB can engage in phase Goaround (6),
   which the old chart did not allow. Unresolved whether this is intended.
3. **onGround bits.** FAC word 5 bits 18/19 (true) and 17 (false) were not mapped to specific LGCIU gear signals.
   FacComputer packs `rtb_AND1` (opp-FAC derived), `rtb_y_kh` and `rtb_Switch_io_idx_1` at those positions
   (FacComputer.cpp 1353–1355), but the LGCIU bit parameters were not resolved. Requiring bit 17 *clear* is odd if bit 17
   is a validity flag.
4. **Manual speed in SRS → OP CLB.** OP CLB `opClbReversion` includes a selected speed for 0.72 s in SRS TO/GA. Whether
   `manual_spd_control_active` (latch `cu`, 3480) can be true during SRS was not analysed, nor whether this matches the
   FCOM.
5. **`any_lateral_mode_engaged` constant.** The CLB pre-arm reads `rtP_fmgc_ap_fd_logic_output_MATLABStruct.any_lateral_mode_engaged`
   (a constant default, always false) instead of a live signal. The intent was probably "no lateral mode or RWY
   engaged" for the ground pre-arm; the term is dead in this build.
6. **One-frame overlap of modes on handover.** It follows from S2 (resets read previous-frame modes), so two
   longitudinal flags are true in the same output frame. Check how the law selection and the FMA (TS) prioritise during
   that frame, and confirm the overlap in a recording.
7. **Speed-protection reversion disabled** (`Constant_Value_m = 0`). Was it deliberately turned off upstream or left
   unfinished? Commit history of the Simulink model was not checked.
