# Input conditioning (the `logic` bus)

Readable specification of the FMGC input conditioning in the generated `FmgcComputer::step()`: ADR, IR, RA, FAC, ILS
and TCAS source selection and voting, failure flags, on ground, engines, the indicated altitude, the ILS tune inhibit
and the runway heading memory. Code: `readable/conditioning.{h,cpp}` (bit-exact against the generated model, checked
by `build/fmgc-check-conditioning-stock`). Line numbers refer to `build/models/stock/FmgcComputer.cpp` (= upstream
master).

The block runs first in every running step (695–1379, after the first-running-frame initialisation), before the AP/FD
engagement (1380). Its results are kept in temporaries and copied to `rtb_BusAssignment_jmp.logic` at 3864–3971;
some sub-bus words are written to the logic bus directly where they are selected (781–1364). None of the temporaries
is reassigned between 1379 and the final copy, so every logic field is the value computed here.

Not here: the eight logic fields of the AP/FD engagement (`ap_fd_athr_common_condition`, `ap_fd_common_condition`,
`fd_own_engaged`, `ap_own_engaged`, `athr_own_engaged`, `ap_inop`, `athr_inop`, `fmgc_opp_priority`,
`docs/modes/engagement.md`).

## Conventions

ARINC 429 SSM values: 0 failure warning (FW), 1 no computed data (NCD), 2 functional test (FT), 3 normal operation (NO).
Two validity tests are used, and the choice differs by family:

- "not failed" (`MATLABFunction_m`, `a429::isNotFailed`): SSM ≠ FW, so NCD and FT count as valid;
- "normal" (`MATLABFunction_ie`, `a429::isNormal`): SSM = NO.

Discrete-word bits are read with `BitfromLabel` (`a429::bit`, 1-based), which ignores the SSM.

## Outputs

| Output | Generated temporary | Lines |
|---|---|---|
| `double_adr_failure` | `rtb_y_er` | 716 |
| `all_adr_valid` | `rtb_OR1_fl` | 720 |
| `adr_computation_data` | `rtb_adrComputationBus_*`, `rtb_y_dd` (std alt), `rtb_DataTypeConversion2_g` / `rtb_y_a` (corrected 1/2), `rtb_y_j` (CAS); Mach, TAS, AoA written directly | 772–845 |
| `altitude_indicated_ft` | `rtb_fm_weight_lbs_SSM` (sic), `rtb_y_gd` | 847–864 |
| `double_ir_failure` | `rtb_adr3Invalid` (reused name) | 767 |
| `all_ir_valid` | `rtb_adrOwnInvalid` (reused name) | 771 |
| `ir_computation_data` | `rtb_irComputationBus_*` (9 words), the other 22 written directly | 866–1074 |
| `ra_computation_data` | `rtb_raComputationData_radio_height_ft_*` | 1076–1094 |
| `dual_ra_failure` | `raOwnInvalid` (reused) | 1096 |
| `both_ra_valid` | `raOppInvalid` (reused) | 1097 |
| `chosen_fac_bus` | `rtb_Switch_*` (10 words), the other 18 written directly | 1101–1213 |
| `fac_rudder_control_failure` | `rtb_irOwnInvalid` (reused) | 1226 |
| `both_fac_rudder_valid` | `rtb_ir3Invalid` (reused) | 1240 |
| `fac_speeds_failure` | `rtb_BusAssignment_n_logic_fac_speeds_failure` | 1256 |
| `fac_weights_failure` | `rtb_BusAssignment_n_logic_fac_weights_failure` | 1262 |
| `on_ground` | `rtb_y_hy` | 1273 |
| `fac_lg_data_failure` | `rtb_AND10_k` | 1274 |
| `gnd_eng_stop_flt_5s` | `rtb_y_ew` (confirm `sf_MATLABFunction_fw`) | 1276–1279 |
| `flap_slat_lever_position` | `rtb_handleIndex` | 1290–1302 |
| `fac_flap_slat_data_failure` | written directly | 1307 |
| `one_engine_out`, `engine_running`, `fcu_failure` | `rtb_BusAssignment_h_logic_*` | 1308–1313 |
| `ils_failure`, `both_ils_valid` | `rtb_BusAssignment_o_logic_*` | 1320–1321 |
| `ils_computation_data` | `rtb_Switch9`, `rtb_Switch_i_*`, `rtb_y` + `rtb_DataTypeConversion2_m` (LOC, via the copy `rtb_BusAssignment_f4_…localizer_deviation_deg` made at 1720); frequency written directly | 1322–1346 |
| `tcas_mode_available` | `rtb_NOT1_i` | 1350–1361 |
| `tcas_failure` | written directly | 1364 |
| `ils_tune_inhibit` | `rtb_y_d` → `rtb_BusAssignment_b_logic_ils_tune_inhibit` | 1366–1379 |
| `rwy_hdg_memo` | `FmgcComputer_B.u_lyjj` | 1373–1375, 3970 |

Memories: the 5 s confirm node `sf_MATLABFunction_fw` (reset on the first running frame, 522) and the runway heading
memory `FmgcComputer_B.u_lyjj`, a block output that is set to 0 by `initialize()` only: it survives a computer restart
(as the VAPP memo does, `docs/modes/targets.md`). The ILS tune inhibit reads the previous frame's modes
(`Delay_DSTATE`, which is the reset value `Delay_InitialCondition` on the first running frame).

## ADR (695–720, 772–845)

- An ADR is **invalid** when any of CAS, corrected AoA, Mach or standard altitude reports FW. NCD and FT count as
  valid; the other five labels (TAS, V/S, corrected altitudes, static pressure) do not count.
- `double_adr_failure`: at least two of own / opposite / 3 invalid. `all_adr_valid`: none invalid.
- **Selection**: the own ADR if valid, else ADR 3 if valid, else a bus with every word FW and 0
  (`FmgcComputer_P.Constant1_Value.adr_computation_data`). The opposite ADR only votes: it is never selected, even when
  it is the only valid one. The whole bus (all nine words) comes from one source.

## Indicated altitude (847–864)

This side's FCU EIS discrete word 2 (left for FMGC 1, right for FMGC 2): if bit 28 (STD) is set or the word reports FW,
`altitude_indicated_ft` is the selected ADR's standard altitude; otherwise its baro-corrected altitude 1 (FMGC 1,
captain's setting) or 2 (FMGC 2). An NCD word counts as "QNH/QFE" (corrected).

## IR (721–771, 866–1074)

- An IR is **invalid** unless body yaw rate, body lateral acceleration, pitch, roll, true heading, true track,
  inertial V/S and latitude are all NO (so NCD, e.g. during alignment, and FT count as invalid, unlike the ADR).
- `double_ir_failure` (2 of 3), `all_ir_valid`, and the selection (own → IR 3 → zero bus) as for the ADR; the opposite
  IR only votes. All 31 words come from one source.

## RA (1076–1100)

| own RA FW | opposite RA FW | `ra_computation_data.radio_height_ft` |
|---|---|---|
| no | no | own |
| no | yes | own |
| yes | no | opposite |
| yes | yes | SSM of the own word (FW), data 2500 ft |

`dual_ra_failure`: both FW. `both_ra_valid`: both NO (stricter). The consumers of the selected word (the TCAS
availability and the ILS tune inhibit here, the engagement's G/S-below-700-ft and A/THR-100-ft checks, the modes) read
its raw data whatever the SSM, so an NCD or FT radio height is used as is, and a dual failure reads as 2500 ft.

## FAC (1101–1307)

- **Selection**: the own FAC's bus when the discrete `fac_own_healthy` is set, else the opposite FAC's bus, whether the
  opposite FAC is healthy or not. All 28 words come from one source. The word SSMs play no part in the choice.
- **Rudder control**, from both FACs' discrete words 2, bit 11 (yaw damper, own channel) and bit 13 (rudder trim, own
  channel), read whatever the SSM or the FAC health:
  - `fac_rudder_control_failure` = no yaw damper engaged in either FAC, or no rudder trim engaged in either FAC, or
    both FACs unhealthy (plus a constant-true enable, `Constant_Value_a`, so that term is dead);
  - `both_fac_rudder_valid` = both FACs healthy and a yaw damper and a rudder trim engaged — the bits are ORed across
    the FACs, so one FAC with both engaged is enough (the name suggests both).
- `fac_speeds_failure`: VMAX or VLS word FW. `fac_weights_failure`: VMAN, CG or gross weight word FW (NCD valid).
- **FAC discrete word 5** of the chosen FAC (bits: 11–15 flap lever 1–5, 16 LGCIU own valid, 17 all LGCIUs lost,
  18/19 left/right main gear compressed, 21 SFCC own valid, 22 all SFCCs lost, 29 alpha floor; FacComputer.cpp
  1347–1366):
  - `on_ground` = bit 18 and bit 19 and not bit 17 and the word not FW;
  - `fac_lg_data_failure` = word FW or bit 17;
  - `gnd_eng_stop_flt_5s` = (both engines stopped and on ground) or **not on ground for 5 s** (confirm node, rising
    edge: goes true after 5.0 s of not-on-ground, false on the first on-ground frame; reset on a computer restart, so
    after an in-flight restart it is false for 5 s);
  - `flap_slat_lever_position` = the lowest of bits 11–15 that is set, as 1–5; 0 when none is set (also when the word
    has failed: bits are read whatever the SSM);
  - `fac_flap_slat_data_failure` = bit 22 or word FW.

## Engines, FCU (1308–1313)

`one_engine_out` = exactly one of `eng_own_stop` / `eng_opp_stop` (the interface sets them from the engine combustion
flags); `engine_running` = either engine running; `fcu_failure` = both FCU channels unhealthy.

## ILS (1314–1346)

- A receiver is **valid** when neither its G/S nor its LOC deviation word reports FW (NCD valid).
  `ils_failure`: both invalid. `both_ils_valid`: both valid.
- **Selection**: the own ILS bus (runway heading, frequency, LOC, G/S) when valid, else the opposite one, valid or not.
- When the FMS has a **back beam** selected (`fms_inputs.backbeam_selected`), the selected LOC deviation is multiplied by
  −1 (`Gain_Gain_f`, float).

## TCAS (1350–1364)

`tcas_mode_available` = the TCAS mode word (`tcas_bus.sensitivity_level`) not FW, bit 25 (STBY) clear, bits 26 and 27
set (sensitivity > 1 and > 2: TA/RA, `TcasComputer.ts` 1344–1346), and the selected radio height (raw) **> 900 ft**.
`tcas_failure`: the word FW.

## ILS tune inhibit and runway heading memory (1365–1379)

`ils_tune_inhibit` = previous frame RWY LOC submode, or LAND, or LAND armed with the radio height (raw) **≤ 700 ft**.
While it is false, the memory takes the selected ILS runway heading if that word is NO, else 0; while it is true the
memory holds. `rwy_hdg_memo` is the memory (double, from the float word). It is read by the ROLL OUT heading monitor
(engagement), the RWY mode and the LOC capture geometry (lateral modes).

## Constants

| Value | Meaning | Parameter |
|---|---|---|
| 5 s | not on ground this long: in flight | `ConfirmNode1_timeDelay` (rising edge) |
| 2500 ft | radio height when both RAs fail | literal |
| 900 ft | TCAS mode needs RA above | `CompareToConstant_const_n` |
| 700 ft | LAND armed at or below: ILS tune inhibit | `CompareToConstant_const_o2` |
| 0 | runway heading memorised when the ILS course word is not NO | `A429ValueOrDefault3_defaultValue` |
| −1 | LOC deviation gain for a back beam | `Gain_Gain_f` |
| true | rudder control enable (dead term) | `Constant_Value_a` |
| 28 | FCU EIS word 2 STD bit | `BitfromLabel1_bit` |
| 11, 13 | FAC word 2 yaw damper / rudder trim (own channel) | `BitfromLabel_bit`, `_1_bit_p`, `2`, `3`, `4`–`7` |
| 11–15, 17, 18, 19, 22 | FAC word 5 flap lever, LGCIUs lost, main gears, SFCCs lost | `BitfromLabel8_bit`…`12_bit`, `2_bit_p`, `_bit_p`, `1_bit_a`, `2_bit_l` |
| 25, 26, 27 | TCAS mode word STBY / TA / TA-RA | `BitfromLabel8_bit_c`, `1_bit_aq`, `2_bit_f` |

## Findings and differences from the real A320

Stock behaviour is kept in the rewrite; nothing below is fixed. Items from memory of the FCOM/AMM, not checked against a
document, are marked "to verify".

1. **No ADR/IR switching**: the selection falls back to ADR 3 / IR 3 automatically when the own unit fails; the AIR DATA
   and ATT HDG switching discretes (`adc_3_switch`, `att_3_switch`) are inputs but never read. On the aircraft the
   switching selectors decide when ADIRU 3 replaces ADIRU 1 or 2 for the side (to verify how the FG reacts to an own
   ADIRU failure without switching).
2. **The opposite ADR/IR only votes**: it is never selected, even as the last valid unit; with own and 3 failed the
   FMGC uses an all-FW zero bus while the opposite unit is still valid (the double failure then removes the AP/FD/A/THR
   common condition anyway). `double_adr_failure` is also set with own and opposite failed and ADR 3 selected and
   valid.
3. **ADR and IR validity are asymmetric**: an ADR is invalid only on a FW SSM in one of four labels (NCD/FT valid), an
   IR on anything but NO in one of eight labels (so an aligning IR, NCD, counts invalid). Within a unit, the other
   labels (ADR TAS, V/S, corrected altitudes, static pressure; most IR labels) are copied whatever their SSM.
4. **RA data used regardless of SSM**: the TCAS 900 ft and the tune-inhibit 700 ft checks (and the engagement's 700 ft
   G/S and 100 ft A/THR checks) read the selected word's raw data, so an NCD/FT radio height is taken at face value, and
   a dual RA failure reads 2500 ft (high: no LAND-armed tune inhibit, TCAS mode available). `both_ra_valid` needs NO on
   both while the selection only rejects FW.
5. **ILS validity never fails in the sim**: the FMGC rejects an ILS only on a FW deviation word, but the interface
   (FlyByWireInterface.cpp 1277–1285) sends NO or NCD (no signal), never FW. So `ils_failure` is never set by a lost
   signal and the opposite ILS is never selected in the sim; with no LOC signal the runway heading word is NCD and the
   memory takes 0. The ILS frequency word is always NO with data 0 (unused).
6. **`both_fac_rudder_valid` ORs the bits**: it needs both FACs healthy but only one yaw damper and one rudder trim
   engaged (bit 11 / 13, the own channel of either FAC). `fac_rudder_control_failure` contains a constant-true enable
   (`Constant_Value_a`, dead term). Bits 12/14 (opposite channel) are not read.
7. **FAC selection on the health discrete only**: with the own FAC unhealthy the opposite FAC's bus is used even when it
   is unhealthy too; the word SSMs then decide the speed/weight/gear flags.
8. **On ground from the FAC only**: `on_ground` comes from the chosen FAC's discrete word 5 (LGCIU data relayed by the
   FAC). The FMGC's own nose gear discretes (`nose_gear_pressed_own/opp`) are never read, and FAC word 5 has no nose
   gear bit. With the word failed or all LGCIUs lost, the FMGC is "in flight" (and `gnd_eng_stop_flt_5s` true 5 s
   later).
9. **5 s in-flight confirmation reset on a restart**: an FMGC restarting in flight has `gnd_eng_stop_flt_5s` false for
   5 s (AP engagement inhibited, the 5-s-airborne terms of the vertical modes delayed).
10. **Flap lever position** comes from FAC word 5 bits 11–15 whatever the word's SSM; 0 when none is set (also with a
    failed FAC word: no separate "unknown").
11. **TCAS mode availability** needs the TA/RA setting (word bits 26 and 27, not STBY) and RA > 900 ft. On the aircraft
    RAs are inhibited below 1000 ft AGL with a hysteresis (to verify the 900 ft value against the TCAS RA inhibit
    altitudes). TCAS faults (fault summary word) are not read; only the mode word's SSM.
12. **ILS tune inhibit**: RWY LOC submode, LAND, or LAND armed at or below 700 ft RA (previous frame's modes). FCOM
    (to verify): ILS tuning is inhibited with LOC or G/S engaged below 700 ft RA and in the takeoff RWY mode; LOC*/LOC/
    G/S engaged without LAND armed (LOC pushbutton only) does not inhibit here.
13. **Back beam inverts the LOC deviation** on the logic bus (`fms_inputs.backbeam_selected`), although there is no
    back-course mode logic (`loc_bc_selection` is a constant false, `docs/modes/lateral.md`).
14. **Unused inputs**: `mcdu_own_fail`, `mcdu_opp_fail`, `nav_control_own`, `nav_control_opp`, `adc_3_switch`,
    `att_3_switch`, `nose_gear_pressed_own`, `nose_gear_pressed_opp` are never read by `step()`. The opposite ADR bus is
    read for 4 labels and the opposite IR bus for 8 (voting only).
15. **Constant input words** (every recording; the replay feeds the recorded inputs): the IR labels discrete word 1,
    inertial altitude, N/S and E/W velocity, along/cross-track and vertical acceleration and flight path acceleration
    are not filled by the interface (FW, 0); the FAC's weight, CG, sideslip target, yaw damper order and discrete word
    4 are constants of the FAC model; the ILS frequency is 0.
16. **Indicated altitude**: STD or a failed FCU EIS word gives the standard altitude, otherwise the side's baro-corrected
    altitude (captain's for FMGC 1, first officer's for FMGC 2), as on the aircraft (each side's baro setting); an NCD
    EIS word counts as QNH.

## Verification (2026-11-14)

`JOBS=4 scripts/check-conditioning-corpus.sh 3`: 22 recordings, 638 runs (resync, free-running, fuzzed, failure
scenarios alone and with the fuzzer, approach, engagement and lateral scenarios), 86 M checked steps, 172 leaves
compared bit for bit: **0 mismatches**. Every leaf changes and every boolean is true in some run, and every selection
branch is taken (ADR/IR own/3/none, RA both/own/opposite/none, FAC own/opposite, ILS opposite, STD / FCU word failed /
corrected 1 / corrected 2, flap lever 0–5, each tune-inhibit cause). The recordings alone (44 runs) leave 22 leaves
constant (finding 15) and almost every failure flag false except at power-up.

## Mutation test (2026-11-14)

Throwaway generator over `conditioning.cpp` (`&&`/`||` swaps, dropped `!`, comparison strictness and direction,
constants ×1.25 / ×0.8 / ±1 for bit numbers, `?:` swaps, own/opposite/3 bus swaps, `isNormal`/`isNotFailed`/failed
swaps, stuck-at true/false on named booleans and outputs, the confirm-node edge flag, deleted statements), plus 20
hand-written ones (nested selection order, fallbacks to the opposite unit, vote terms dropped, flap lever bit order,
memory update conditions, raw vs SSM-checked radio height, on-ground with either gear). Each mutant runs over the corpus
with the failure scenarios first, then free-running, fuzz, approach, engagement and lateral, until the first mismatch.

- 324 mutants, 302 compile (the rest are `bool` stuck-at on struct fields and a malformed nested `?:`): **298 killed**.
- 4 survivors, all **equivalent**: the dual-RA-failure SSM taken from the opposite instead of the own word (twice: both
  are FW in that branch), `flap_slat_lever_position = false` (= 0), and a control rewrite of the corrected-altitude
  branch.
- Killed after adding a scenario pattern: the rudder trim bit of the opposite FAC replaced by the own one (the recordings
  never have the trim engaged in one FAC only; `FacWord2` scopes 4/5).

## Integration notes

- The conditioning runs first in each running step and needs only the inputs, the previous frame's `ap_fd_logic`
  (for the tune inhibit; the reset bus on the first running frame) and its two memories. Everything downstream reads its
  outputs; the generated code reads several of them through the temporaries rather than the logic bus (`rtb_y_hy` =
  on ground, `rtb_y_ew`, `rtb_handleIndex`, `rtb_raComputationData_radio_height_ft_Data`, `raOwnInvalid`, the selected
  CAS/VLS/VMAX/attitude data...), always with the same value.
- `rwy_hdg_memo` survives restarts (reset only by `initialize()`); the 5 s confirm node is reset on each restart.
- The localizer deviation goes through the copy `rtb_BusAssignment_f4_…localizer_deviation_deg` (1720) because its
  temporary `rtb_DataTypeConversion2_m` is reused from 2416 on; same value.
