# Vertical modes: ALT*, ALT, V/S, FPA, TCAS

Readable specification of the altitude capture/hold, V/S-FPA and TCAS mode logic of the generated A32NX
`FmgcComputer::step()`. Line numbers refer to `tools/fmgc-lab/build/models/stock/FmgcComputer.cpp` (identical to
upstream master). Parameter values come from `docs/params-FmgcComputer.txt`; every SR latch used below has the default
truth table `0 1 0 0 1 1 0 0 1 0 1 1 0 0 0 0` (hold / set / reset, **reset wins**), checked for `Logic_table_pk, nz, ob,
a, ny, id, fn, oq, dj, am`.

Conventions:
- `X_last` = `Delay_DSTATE.<...>.X`, the value of output X on the previous frame.
- `ConfirmNode(u, rising, T)`: rising = true after u has held T s, false at once; falling (`isRisingEdge = 0`) = true at
  once, false only after u has been false for T s ("u was true within the last T s").
- `MTrig(u, T)`: monostable, true for T s after a rising edge. `Pulse↑/Pulse↓`: one-frame edge detector.
- `bit(word, N)`: 1-based bit N (`MATLABFunction_i`). The packer `MATLABFunction_gy` puts element i at shift i+10, so
  **bit N = packed element N−11** (lines 190–199, 63–85).
- Every SET has the form `oppSync ∨ (ownCompute ∧ …)`, see "Common signals".

---

## Common signals

| Name | Definition | Lines |
|---|---|---|
| `oppModeSync` (`rtb_OR2_l`, output `fmgc_opp_mode_sync`) | this FMGC copies the opposite FMGC's modes (opp has priority; depends on AP/FD/A/THR engagement of both sides and `is_unit_1`) | 1537–1547, 4020 |
| `ownCompute` (`rtb_y_oy`) | `¬oppModeSync`: this FMGC computes modes itself | 1687 |
| `anyApFd` (`apCondition` after 1558, output `any_ap_fd_engaged`) | own AP ∨ opp AP ∨ own FD ∨ opp FD engaged (inferred names: `Delay_DSTATE_p`, `ap_opp_engaged`, `fdOwnOff` (reused as "FD own on"), `fd_opp_engaged`) | 1548, 1558, 4021 |
| `modeReset` (`rtb_BusAssignment_m_ap_fd_logic_lateral_mode_reset`, = `lateral_mode_reset` = `longitudinal_mode_reset`) | `¬anyApFd ∨ (Pulse↑(engine_running) ∧ onGround)`: all modes reset when no AP/FD is engaged or at the first engine start on the ground | 1550–1559, 4031–4032 |
| `onGround` (`rtb_y_hy`) | FAC discrete word 5: bit 18 ∧ bit 19 ∧ ¬bit 17 ∧ SSM not FW (LGCIU-derived, inferred) | 1263–1273 |
| `airborne5s` (`rtb_y_ew`) | `ConfirmNode(¬onGround, rising, 5 s) ∨ (both engines stopped ∧ onGround)` | 1275–1279 |
| `H` | indicated altitude (`logic.altitude_indicated_ft`, STD or baro-corrected, `rtb_y_gd`, 835–866), A429 value-or-0 | e.g. 2408, 2498 |
| `H_fcu` | `fcu_bus.selected_alt_ft` (value-or-0) | 2410, 1636 |
| `target` = `alt_sel_or_cstr` (`rtb_DataTypeConversion_cm`) | `alt_cstr_applicable ? fms.next_alt_cstr_ft : H_fcu` — see its own section | 1660–1664, 4019 |
| `VS_inertial` | voted IR `inertial_vertical_speed_ft_s` **in ft/min** despite the name (spline breakpoints in fpm, ×0.00508 → m/s) | 2480, 2497 |
| `fcuAltChanged` | FCU discrete word 1 bit 13 = FCU `alt_value_changed`: 0.5 s non-retriggerable pulse whenever the FCU altitude value changes (FcuComputer.cpp 1065–1068, 1092, 1342; `MTrigNode1_triggerDuration_h = 0.5`) | — |
| `tcasRaAvail` (`rtb_NOT1_i`; `rtb_y_n = ¬tcasRaAvail` until 3382) | TCAS mode word (`tcas_bus.sensitivity_level`) not FW ∧ RA > 900 ft ∧ ¬bit 25 (STBY) ∧ bit 26 (sens > 1) ∧ bit 27 (sens > 2), i.e. TA/RA mode above 900 ft (bits from TcasComputer.ts 1344–1346) | 1350–1361, 1609 |
| `raActive` | TCAS RA word bit 20 = top bit of the "combined control" field (values 4, 5, 6 = RA in progress; TcasComputer.ts 1321–1336) | 2519, 2563, 2642 |
| `raCorrective` (`rtb_y_d`, output `tcas_ra_corrective`) | `bit18 ∨ ¬bit19 ∨ ¬bit20` = combined control ∉ {2, 6} (6 = preventive). Note: also **true when no RA** (control 0) | 1560–1568, 1650 |

Discrete-word bits used here (bit N, 1-based):

| Word | Bit | Meaning | Source |
|---|---|---|---|
| FCU word 1 | 13 | ALT value changed (0.5 s pulse) | FcuComputer.cpp 1342 |
| FCU word 1 | 15 | SPD/MACH value changed | 1344, 1403 |
| FCU word 1 | 16 / 19 | V/S-FPA knob pushed / pulled | 1345, 1348 |
| FCU word 1 | 23 | APPR pushed | 1352 |
| FCU word 1 | 24 / 25 | HDG-V/S selected (¬trk_fpa) / TRK-FPA selected | 1353–1354 |
| FMGC dw1 | 17, 18 | V/S active, FPA active | 4843–4844 |
| FMGC dw1 | 19 | ALT* ∨ ALT | 4845 |
| FMGC dw1 | 20 | ALT ∨ G/S | 4847 |
| FMGC dw1 | 21 | ALT* ∨ G/S* | 4849 |
| FMGC dw1 | 26 | `fma_dash_display` (ALT CRZ) | 4852 |
| FMGC dw3 | 12, 13 | ALT armed, ALT arm possible | 4808–4809 |
| FMGC dw7 | 12, 13 | TCAS armed, TCAS active | 4905–4906, 5052 |

---

## ALT arm possible  (`armed_modes.alt_acq_arm_possible`, latch `Memory_PreviousInput_n`, FmgcComputer.cpp:2418)

**Initial value: TRUE** (`SRFlipFlop1_initial_condition_b = 1`, line 480/5163).

**Set** — any of (`rtb_Compare_ji` 2412–2417):
- `fcuAltChanged` (FCU word 1 bit 13; param `BitfromLabel1_bit_m = 13`) — 2410–2411
- `|H − H_fcu| > 250 ft` (`CompareToConstant1_const_hi = 250`) — 2416–2417

**Reset** — all of:
- `ALT_last` (ALT hold active last frame) ∧ `|H − H_fcu| ≤ 40 ft` (`CompareToConstant_const_mh = 40`) — 2419–2420

**Notes**
- Uses the **FCU** altitude, never the constraint. After levelling (ALT) at the FCU altitude the FMGC refuses to re-arm
  ALT until the pilot changes the FCU altitude or the aircraft departs by more than 250 ft.
- Holding ALT CST at a constraint far from the FCU altitude does not reset it.

## ALT armed  (`armed_modes.alt_acq_armed`, combinational, `rtb_Compare_ji`, FmgcComputer.cpp:2465)

Not a latch: recomputed every frame (2427–2440).

**True** — any of:
- `oppModeSync ∧ opp.dw3.bit12` (opp ALT armed; `BitfromLabel_bit_et = 12`) — 2421–2424
- `ownCompute` ∧ all of — 2427–2440:
  - `¬ALT*_last ∧ ¬ALT_last ∧ ¬G/S*_last ∧ ¬G/S_last ∧ ¬FINAL_DES_last ∧ ¬LAND_last`
  - `¬awayFromFcuAlt ∨ (¬VS_last ∧ ¬FPA_last ∧ ¬TCAS_last)` — in V/S, FPA or TCAS, ALT is armed only if the
    selected vertical target points towards the **FCU** altitude
  - `¬(SRS_GA_last ∨ SRS_TO_last) ∨ (H − H_fcu ≤ 0)` (`CompareToConstant2_const_jy = 0`) — in SRS, only with the FCU
    altitude at or above the aircraft
  - `anyApFd` (any AP or FD engaged)
  - `alt_acq_arm_possible` (this frame's latch)

`awayFromFcuAlt` (`rtb_OR_b`, 2425–2447), with `d = H − H_fcu` (2416) and `tgtVs` (2427–2440):
- `tgtVs` = FCU `selected_vz_ft_min` if `VS_last`; FCU `selected_fpa_deg` if `FPA_last`; if `TCAS_last ∧ ¬raCorrective`:
  `tcas_vs_target` (`rtb_Phi_c_deg` from 1599–1607); otherwise 0.
- `d < 0` (below FCU alt): `away = tgtVs ≤ 0`; otherwise `away = d > 0 ∧ tgtVs ≥ 0`. (`d = 0` → not away.)

**Notes**
- No on-ground, flight-phase or airborne inhibit: on the ground before takeoff with an FD on it is TRUE (see "ALT* on
  the ground").
- In V/S with a selected V/S of 0 below the FCU altitude, `away` is true → ALT is not armed (inferred consequence).
- In TCAS mode with a corrective RA, `tgtVs = 0` → never "towards", so ALT is not armed.

---

## ALT*  (`longitudinal_modes.alt_acq_active`, latch `Memory_PreviousInput_ne`, FmgcComputer.cpp:2547)

**Set (engage)** — any of (`rtb_OR_b` 2522–2526):
- **opp sync**: `oppModeSync ∧ opp.dw1.bit19 ∧ opp.dw1.bit21` (= opp ALT*, or ALT ∧ G/S*), params
  `BitfromLabel1_bit_ng = 19`, `BitfromLabel2_bit_n = 21` — 2466–2472
- **own**: `ownCompute ∧ captureCond ∧ (RA ≥ 400 ft ∨ (¬SRS_GA_last ∧ ¬SRS_TO_last)) ∧ tcasOk` — 2522–2526
  - `RA` = raw radio height Data (`MATLABFunction_c`, no SSM check; 2500 ft when both RAs failed, 1093),
    `CompareToConstant_const_l5 = 400`
  - `tcasOk = ¬TCAS_last ∨ (¬raActive ∧ active_tcas_submode_last == ALT_ACQ (1) ∧ tcasRaAvail)`
    (`BitfromLabel7_bit_j = 20`, `EnumeratedConstant3_Value_l = 1`)

`captureCond` (`rtb_NOT_b` 2513–2517, also output as `tcas_alt_acq_cond` via `rtb_AND1_c0` 2549, 4042) = all of:
- `alt_acq_armed_last`
- `¬MTrig(fcuAltChanged, 3 s, rising, retriggerable)` — no FCU altitude change in the last ~3 s
  (`BitfromLabel3_bit_f = 13`, `MTrigNode_triggerDuration_bh = 3`, `_retriggerable_pd = 1`) — 2474–2479
- `|target − H| < Dcapt(VS_inertial)` — 2496–2517
- `sign(target − H) == sign(VS_inertial)` (exact sign function; 0 only equals 0) — 2500–2511

Capture distance (ft): `Dcapt = min(3000, max(80, (VS·0.00508)² / (9.81 · f(|VS|)) · 3.28084))`, VS in ft/min, so
`(VS m/s)² / (f·g)` in m converted to ft. `f` is a piecewise-cubic (MATLAB `ppval`) "load factor" in g,
breakpoints `b = {0, 1000, 3333, 4000, 6000, 8000, 10000}` fpm, coefficients `c[24]` (lines 441–447; interval k uses
`c[k]`·dx³ + `c[k+6]`·dx² + `c[k+12]`·dx + `c[k+18]`, dx = |VS| − b[k]):

```
k :   cubic                    quadratic                linear                   constant
0 : -3.7631613045100394E-12   2.2735910872868498E-8   -1.897274956835846E-5    0.05
1 : -3.7631613045100418E-12   1.1446426959338374E-8    1.520958826384842E-5    0.05
2 :  6.2076488130688133E-12  -1.4891939010927404E-8    7.1712086474912069E-6   0.1
3 :  2.3375903616618146E-12  -2.4704337359767112E-9   -4.4094939746938354E-6   0.1
4 : -2.9675180723323623E-12   1.1555108433994175E-8    1.3759855421341094E-5   0.1
5 : -2.9675180723323619E-12  -6.25E-9                  2.4370072289329445E-5   0.15
```

Evaluated (same arithmetic as the code; computed for this document):

| |V/S| fpm | 0 | 250 | 500 | 750 | 1000 | 1500 | 2000 | 2500 | 3000 | 4000 | 5000 | 6000 | 8000 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f (g) | 0.050 | 0.047 | 0.046 | 0.047 | 0.050 | 0.060 | 0.073 | 0.086 | 0.096 | 0.100 | 0.096 | 0.100 | 0.150 |
| raw (ft) | 0 | 12 | 47 | 103 | 173 | 324 | 474 | 628 | 808 | 1381 | 2260 | 3107 | 3682 |
| **Dcapt (ft)** | **80** | **80** | **80** | 103 | **173** | 324 | **474** | 628 | 808 | **1381** | 2260 | 3000 | 3000 |

- Below ~650 fpm the 80 ft floor applies; the 3000 ft ceiling from ~5840 fpm.
- Above 10000 fpm the last cubic is extrapolated: f falls to 0 at ~11730 fpm and goes negative, so `raw < 0` and
  Dcapt collapses to 80 ft for |V/S| ≳ 11750 fpm (unreachable in practice).
- Same table and formula as the old `AutopilotStateMachine` (see Notes).

**Reset (disengage)** — any of (2527–2546):
- `modeReset` (immediate)
- any of `VS, FPA, ALT, CLB, DES, EXP CLB, EXP DES, FINAL DES, G/S*, G/S, OP CLB, OP DES, SRS GA, SRS TO, TCAS, LAND`
  active last frame **∧ ¬ConfirmNode(set, falling, 0.72 s)** (`ConfirmNode_isRisingEdge_hux = 0`,
  `_timeDelay_lk = 0.72`): another mode only takes over once the ALT* set condition has been false for 0.72 s.

**Notes**
- Because `alt_acq_armed` requires `¬ALT*_last`, the set condition drops the frame after engagement; the 0.72 s falling
  confirm is what protects a freshly engaged ALT* from a mode that latched on the same frame.
- Leaving ALT* to V/S is not in this latch: the "target changed by ≥ 250 ft" rule lives in
  `longitudinal_mode_reversion_vs`, which **sets V/S**, and V/S being active then resets ALT* (after the 0.72 s).
- Old Stateflow chart (`git show afff8d290^:…/AutopilotStateMachine.cpp`, lines 3604–3660): same spline and
  `min(3000, max(80, …))`, but additionally required **|H_dot| ≥ 100 fpm**, **H_radio > 400 ft always** (not only in
  SRS), and a 3 s persistence of the capture condition. The current code dropped the 100 fpm check and limited the
  400 ft RA check to SRS → ALT* on the ground (next section).
- `active_longitudinal_law = ALT_ACQ (2)` whenever this latch is set (or TCAS with submode ALT_ACQ) and no higher-
  priority law applies; priority (3755–3792): FLARE > GS > SRS > FPA > VS > SPD_MACH (CLB/OP CLB/OP DES/EXP) > ALT_ACQ >
  ALT_HOLD > VPATH (FINAL) > DES submode. So on the engagement frame from CLB/DES the law can still read SPD_MACH.

### ALT* on the ground

Answer: **there is no on-ground, flight-phase or airborne inhibit anywhere in the ALT* set path or in
`alt_acq_armed`**. On the ground before takeoff, with an FD on, every term is satisfied as soon as the field elevation
is within 80 ft of the FCU altitude:

| Term | Value on the ground before takeoff | Why |
|---|---|---|
| `alt_acq_arm_possible` | 1 | latch **initialises to 1** (480) and is only reset by ALT within 40 ft |
| `alt_acq_armed` (2427–2440) | 1 | no ALT*/ALT/G/S/FINAL/LAND/V/S/FPA/TCAS, not in SRS yet, `anyApFd` = FD on |
| no FCU alt change for 3 s | 1 | |
| `|target − H| < Dcapt` | 1 when `|H_fcu − H| < 80` | V/S ≈ 0 → Dcapt = max(80, ≈0) = **80 ft** |
| `sign(target − H) == sign(VS_inertial)` | ~50 % per frame | `VS_inertial` on the ground is numerical noise of ±1e-7…1e-4 fpm, **never exactly 0**, so its sign matches the target direction within a frame or two |
| `RA ≥ 400 ∨ (¬SRS_GA ∧ ¬SRS_TO)` | 1 | SRS TO is not active before takeoff thrust; the RA check applies only in SRS |
| `tcasOk` | 1 | no TCAS mode |

The FCU altitude is 100 ft after power-up (FcuComputer clamps to [100, 49000]), so any airport 20–180 ft elevation gets
ALT* at FD-on. The law follows directly: `Memory_PreviousInput_ne` → `active_longitudinal_law = ALT_ACQ (2)` at
3771–3773 (nothing of higher priority is active pre-takeoff).

Replay evidence (stock model, scratch probe, `on_ground = 1`, all with `alt_acq_armed = arm_possible = any_ap_fd = 1`):

| Recording | t (s) | H | FCU | dH | VS_inertial (fpm) | Ends |
|---|---|---|---|---|---|---|
| 2026-11-11-16-39-50 | 584.2 | 104 | 100 | −4 | −2e-07 | ALT after 0.8 s (585.0) |
| 2026-11-11-21-14-07 | 459.9 | 28 | 100 | 72 | 3.1e-06 | 549.2 (engine-start `modeReset`), re-engages next frame; at 925.1 FCU → 1000 gives **V/S on the ground** |
| 2026-11-12-17-08-01 | 66.2 | 96 | 100 | 4 | 5.8e-05 | ALT after 0.7 s |
| 2026-11-12-19-54-17 | 81.2 | 94 | 100 | 6 | 1.9e-06 | ALT after 0.7 s |
| 2026-11-12-21-10-58 | 123.8 | 22 | 100 | 78 | 2.4e-05 | 129.8: FCU → 1000, `reversion_vs` → **V/S on the ground** until takeoff |

Follow-on effects: within 40 ft, ALT engages after 0.8 s and then resets `alt_acq_arm_possible`; 40–80 ft away ALT*
stays engaged on the ground until a mode reset, SRS, or an FCU change (which reverts to V/S, since the reversion term
has no ground gating either). Possible fixes (inferred, matching the old chart): require `¬onGround` (or RA > 400 ft
outside SRS too) and/or `|VS| ≥ 100 fpm` in `captureCond`.

---

## ALT  (`longitudinal_modes.alt_hold_active`, latch `Memory_PreviousInput_cb`, FmgcComputer.cpp:2589)

**Set (engage)** — any of (`rtb_OR_b` 2568):
- **opp sync**: `oppModeSync ∧ opp.dw1.bit19 ∧ opp.dw1.bit20` (= opp ALT; `BitfromLabel1_bit_p2 = 19`,
  `BitfromLabel2_bit_fr = 20`) — 2550–2556
- **own** (`rtb_y_nu` 2564–2567): `ownCompute ∧ ALT*_last ∧ ConfirmNode(|target − H| < 40 ft, rising, 0.8 s) ∧
  (¬TCAS_last ∨ (¬raActive ∧ active_tcas_submode_last == ALT_HOLD (2) ∧ tcasRaAvail))`
  (`CompareToConstant_const_f = 40`, `ConfirmNode_timeDelay_at = 0.8`, `BitfromLabel7_bit_k = 20`,
  `EnumeratedConstant3_Value_p = 2`) — 2557–2567
  - the confirmed `|target − H| < 40` (`rtb_y_fi`) is also output as `tcas_alt_hold_cond` (`rtb_AND8` 2610, 4043)

**Reset (disengage)** — any of (2569–2588):
- `modeReset`
- any of `VS, FPA, ALT*, CLB, DES, EXP CLB, EXP DES, FINAL DES, G/S*, G/S, OP CLB, OP DES, SRS GA, SRS TO, TCAS, LAND`
  last frame ∧ `¬ConfirmNode(set, falling, 0.72 s)` (`ConfirmNode_isRisingEdge_lj = 0`, `_timeDelay_bd = 0.72`)

**Notes**
- Threshold is **40 ft for 0.8 s**, not the "20 ft" of the FBW docs list in PLAN.md; the old chart also used 40 ft /
  0.8 s (`c_R >= 40` restarts its timer, old lines 3594–3599, 4044).
- ALT is entered only from ALT* (own) — there is no direct ALT engagement (e.g. FCU ALT push is not handled here).
- ALT does not react to target changes; leaving ALT is by another mode (e.g. CLB/DES engaging on a new constraint).

## ALT CRZ (`longitudinal_modes.fma_dash_display`, FmgcComputer.cpp:2609)

`fma_dash_display = ALT (this frame, Memory_PreviousInput_cb) ∧ ¬alt_cstr_applicable ∧ crzLatch` — 2598–2609.
- `crzLatch` (`Memory_PreviousInput_no`, 2596): set `|fms.cruise_alt_ft − H| < 60 ft` (`CompareToConstant1_const_a =
  60`), reset `¬ALT_last`.
- Packed in FMGC dw1 bit 26; FMA.tsx 882/929–931 shows it as **ALT CRZ** ("dash mode" suppresses ALT/ALT CST text).
- Note: uses `rtb_OR_b = ¬rtb_NOT_oj`, and `rtb_NOT_oj` is still `alt_cstr_applicable` (1655) at 2598.

---

## V/S and FPA  (`vs_active`, `fpa_active`; one shared latch `Memory_PreviousInput_fg`, FmgcComputer.cpp:2697)

V/S and FPA are **one latch** split by the FCU HDG-V/S / TRK-FPA selection (2715–2719):
- `vs_active = rtb_AND12 = latch_fg ∧ fcu.w1.bit24` (HDG-V/S), output at 3993
- `fpa_active = latch_fg ∧ ¬fcu.w1.bit24` (TRK-FPA) (`BitfromLabel3_bit_e = 24`)
- Switching HDG-V/S ↔ TRK-FPA swaps V/S ↔ FPA on the same frame without any latch transition.
- The generator wrote this latch's index with the S term second: `Logic_table_ny[((R + (S << 1)) << 1) + Q]`
  (2697–2714) — S is `rtb_y_nu`, R is the first term.

**Set (engage)** — any of (`rtb_y_nu` 2692–2694):
- **opp sync**: `oppModeSync ∧ (opp.dw1.bit17 ∨ opp.dw1.bit18)` (opp V/S or FPA; `BitfromLabel1_bit_ax = 17`,
  `BitfromLabel2_bit_k = 18`) — 2670–2675
- **own** ∧ any of:
  - **knob**: `airborne5s ∧ Pulse↑(fcu.w1.bit19 ∨ fcu.w1.bit16) ∧ ¬LAND_last` — V/S-FPA knob pulled or pushed
    (`BitfromLabel4_bit_g = 19`, `BitfromLabel5_bit_n = 16`) — 2676–2681
  - **AP/FD engagement**: `¬onGround ∧ Pulse↑(anyApFd) ∧ ¬any_longitudinal_mode_engaged_last` — 2685–2689
  - **5 s after liftoff**: `ConfirmNode(¬onGround, rising, 5 s) ∧ ¬any_longitudinal_mode_engaged_last`
    (`ConfirmNode_timeDelay_op = 5`) — 2690–2691
  - **reversion**: `longitudinal_mode_reversion_vs` (below)

**Reset (disengage)** — any of (2697–2714):
- `modeReset`
- any of `ALT*, ALT, CLB, DES, EXP CLB, EXP DES, FINAL DES, G/S*, G/S, OP CLB, OP DES, SRS GA, SRS TO, TCAS, LAND`
  last frame ∧ `¬ConfirmNode(set, falling, 0.72 s)` (`ConfirmNode_isRisingEdge_lu = 0`, `_timeDelay_ll = 0.72`)

**Notes**
- Only the knob path is gated by `airborne5s`; the reversion path has **no ground gating** (V/S on the ground in two
  recordings, see "ALT* on the ground").
- `any_longitudinal_mode_engaged` (4022–4029) is the OR of all vertical latches, including V/S-FPA via the reused temp
  `rtb_fmgcOppPriority_tmp = rtb_AND12 ∨ fpa_active` (3372).

### V/S reversion  (`longitudinal_mode_reversion_vs`, `rtb_Compare_ji` 2646–2668, output 2720/4036)

True when any of (all `_last` values are previous-frame modes):
1. **G/S lost LAND**: `(G/S_last ∨ G/S*_last) ∧ Pulse↓(LAND_armed_last ∨ LAND_last)` (`rtb_AND12` at 2612–2616,
   `PulseNode3_isRisingEdge_n = 0`)
2. **FINAL lost**: `FINAL_DES_last ∧ (Pulse↓(NAV_last) ∨ Pulse↑(fcu.w1.bit23 APPR pushed) ∨ ¬fms.vertical_flight_plan_valid)`
   (2617–2623; `PulseNode4_isRisingEdge_f = 0`, `BitfromLabel6_bit_d = 23`, `PulseNode5_isRisingEdge_p = 1`)
3. **DES invalid**: `DES_last ∧ (fms_flight_phase ∈ {Takeoff 1, Climb 2, Goaround 6} ∨ ¬(NAV ∨ LOC* ∨ LOC)_last ∨
   ¬vertical_flight_plan_valid)` (`EnumeratedConstant_Value_p/1_Value_m/2_Value_h`)
4. **ALT\* target changed**: `ALT*_last ∧ |u_ly − target| ≥ 250 ft` (`CompareToConstant_const_om = 250`), where
   `u_ly = FmgcComputer_B.u_ly` is a sample-and-hold updated while `¬ALT*_last` (2626–2628), i.e. **the target value on
   the frame ALT* latched**.
5. **DES-family with FCU above**: `(DES ∨ EXP DES ∨ OP DES)_last ∧ H_fcu − H > 0` (2630, `CompareToConstant1_const_o = 0`)
6. **CLB-family with FCU below**: `(CLB ∨ EXP CLB ∨ OP CLB)_last ∧ H_fcu − H < 0` (`CompareToConstant2_const_g = 0`)
7. (dead) speed-protection reversions `… ∧ Constant_Value_m` with `Constant_Value_m = 0` (VLS−2 / VMAX+4 terms, 2660–2667)
8. **TCAS ended**: `(¬raActive ∨ ¬tcasRaAvail) ∧ TCAS_last ∧ (¬tcasRaAvail ∨ active_tcas_submode_last == VS (0))`
   (`rtb_y_bn` 2642–2645, `BitfromLabel7_bit_o = 20`, `EnumeratedConstant3_Value_ls = 0`)

It also feeds the triple-click / FD-flash pulses at 3384–3397 (not covered here).

### Relevance: ALT CST* → V/S on constraint sequencing

Term 4 compares the **current** `alt_sel_or_cstr` against its value when ALT* engaged. In ALT CST* the target is
`fms.next_alt_cstr_ft`; when the FMS sequences the constrained waypoint, `next_alt_cstr_ft` jumps to the next
constraint (or 0 → target falls back to the FCU altitude, `alt_cstr_applicable` → 0). Either way `|u_ly − target| ≥
250` → `reversion_vs` → V/S latches. No term distinguishes "pilot changed the FCU altitude" from "the FMS sequenced a
constraint"; CLB/DES-armed is not consulted.

Observed in the stock replay of 2026-11-11-21-14-07 (climb, NAV, CLB armed):

| t (s) | next cst | target | result |
|---|---|---|---|
| 2015.7 | 6000 → 8000 | 6000 → 8000 | V/S **and** CLB latch on the same frame; V/S survives, CLB drops after ~0.75 s; law VS |
| 2148.7 | 8000 → 13000 | 8000 → 13000 | same |
| 2355.7 | 13000 → 0 | 13000 → 37000 (FCU) | same (`alt_cstr_applicable` 1 → 0) |

Both latches have the 0.72 s "other mode" protection, so on the sequencing frame both are set; which one survives is
decided by which set condition stays true longer (inferred; see the CLB doc). The `cst` variant
(`personal/fg-alt-cst-reversion`, commit 0ead3c962) patches exactly term 4 with
`¬(CLB/DES armed ∧ next_alt_cstr changed) ∧ ¬CLB_last ∧ ¬DES_last`.

### V/S target not held  (`vs_target_not_held`, `rtb_y_n` at 3382, output 4040)

`apOwn ∧ (VS ∨ FPA) ∧ ((CAS < VLS + 3 ∧ err < −thr) ∨ (CAS > VMAX − 3 ∧ err > thr))` — 3372–3383
- `apOwn = Delay_DSTATE_p` (own AP engaged, inferred); V/S/FPA from this frame (`rtb_AND12`, `fpa_active`)
- V/S: `err = VS_inertial − selected_vz_ft_min`, `thr = 50 fpm`; FPA: `err = IR FPA − selected_fpa_deg`, `thr = 0.1°`
- i.e. speed protection is preventing the selected V/S/FPA (FBW "V/S reverts to speed" indication). Packed in FMGC
  dw4 element 18 (4755); triggers the 1 s MTrig at 3384–3388 (`MTrigNode2_triggerDuration_f = 1`).

---

## alt_cstr_applicable / alt_sel_or_cstr  (FmgcComputer.cpp:1640–1664, outputs 4018–4019)

Readable: `readable/vertical_alt.cpp`, signals `AltCstrApplicable` and `AltSelOrCstr` (the first two of the vertical
update order, value in `Signals::altSelOrCstr`); compared by `check_vertical` like every other signal (the target
bit-exact as a double), 0 mismatches over the 374-run corpus check.
Mutation test (throwaway generator, 41 compiled mutants of these two signals): 1 equivalent (the coverage bool of
`AltSelOrCstr`), 29 caught by the corpus runs with seed 1, 6 more with seeds 2–5 (LOC*/LOC in `levelAtCstr`,
`vertical_flight_plan_valid`, the climbing-side margin sign, `&&`→`||` on the descending side); 5 survive, all
exact-equality boundaries of float comparisons (`fcuAlt > h`, `cstr + 250 > h`, `cstr < fcuAlt`, `cstr > fcuAlt`,
`cstr − 250 < h` with `>=`/`<=`).

```
capturingCst = (ALT*_last ∨ ALT_last) ∧ alt_cstr_applicable_last
             ∧ (NAV ∨ LOC* ∨ LOC)_last ∧ fms.vertical_flight_plan_valid                       (1640–1646)
cst = fms.next_alt_cstr_ft
if H_fcu > H:   between = cst < H_fcu ∧ (cst + 250 > H ∨ capturingCst)                         (1647–1649)
else:           between = cst > H_fcu ∧ (cst − 250 < H ∨ capturingCst)                         (1650–1652)
alt_cstr_applicable = (CLB_armed ∨ DES_armed ∨ CLB ∨ DES ∨ capturingCst)_last ∧ cst ≠ 0 ∧ between   (1655–1659)
alt_sel_or_cstr     = alt_cstr_applicable ? cst : H_fcu                                         (1660–1664)
```
- The constraint is used only if it lies between the aircraft (±250 ft margin) and the FCU altitude, and a managed
  climb/descent is armed or active — or ALT*/ALT is already on a constraint (`capturingCst` latches applicability
  through the capture, ignoring the ±250 ft margin).
- The value is always the **current** `next_alt_cstr_ft`: `capturingCst` keeps applicability but not the captured
  value, which is why sequencing changes the ALT*/ALT target (see the reversion above).
- ALT arm logic (`arm_possible`, `awayFromFcuAlt`) uses the FCU altitude; ALT*/ALT capture and the reversion use
  `alt_sel_or_cstr`.
- `alt_cstr_applicable` also drives the FMA "CST" text (`altConstraintValid`, FMA.tsx 921–929, via the A bus at 4980).

---

## TCAS  (`longitudinal_modes.tcas_active`, latch `Memory_PreviousInput_i5`, FmgcComputer.cpp:3300)

**Set** — any of (`rtb_OR_b` 3279):
- **opp sync**: `oppModeSync ∧ opp.dw7.bit13` (`BitfromLabel2_bit_g = 13`) — 3269–3272
- **own**: `ownCompute ∧ tcasRaAvail ∧ Pulse↑(raActive)` (RA onset; `BitfromLabel1_bit_n4 = 20`) — 3264, 3273–3278

**Reset** — `(modeReset ∨ any other vertical mode last frame (VS, FPA, ALT, ALT*, CLB, EXP CLB, EXP DES, FINAL DES,
OP DES, G/S*, G/S, OP CLB, DES, SRS GA, SRS TO, LAND)) ∧ ¬ConfirmNode(set, falling, 0.72 s)` — 3280–3299
(`ConfirmNode_isRisingEdge_jj = 0`, `_timeDelay_ae = 0.72`).
Quirk: here `modeReset` is also delayed by the 0.72 s confirm (it is inside the AND), unlike ALT*/ALT/V/S.

TCAS mode does not end when the RA ends: it is replaced by ALT* / ALT (their `tcasOk` terms) or by V/S (reversion
term 8), depending on the TCAS submode.

### TCAS armed  (`armed_modes.tcas_armed`, latch `Memory_PreviousInput_n0`, 3266)
- Set: `(oppModeSync ∧ opp.dw7.bit12) ∨ (ownCompute ∧ tcasRaAvail ∧ discrete tcas_ta_display)` — 3260–3265
- Reset: `¬tcas_ta_display ∨ ¬tcasRaAvail ∨ TCAS_last` (S term written second in the index expression)

### TCAS outputs
- `tcas_vs_target` (`rtb_Phi_c_deg` 1572–1607, output 1666): if `raCorrective`: `±(rate + 200)` fpm, rate = RA word
  bits 11–16 × 100 (`(Data & 0xFC00) >> 10`), +200 fpm margin only when rate ≠ 0, sign − if bit 17; else (preventive)
  the current ADR baro V/S (`B.u_lyjjlj` tracks ADR V/S whenever `¬raCorrective ∨ ¬raActive`, so it is a pass-through).
- `active_tcas_submode` (`rtb_mode` 3320–3326): `ALT_ACQ` if latch `_h`, else `ALT_HOLD` if latch `_cp`, else `VS`.
  - `_h` (3311): S = `captureCond` (ALT* geometric condition, `rtb_AND1_c0`); R = `|B.u_lyj − target| ≥ 250 ∨
    ¬raActive ∨ ¬alt_acq_armed_last` (`B.u_lyj` = target held while submode_last == ALT_ACQ; `CompareToConstant_const_g =
    250`; `BitfromLabel3_bit_h = 20`).
  - `_cp` (3318): S = `submode_last == ALT_ACQ ∧ tcas_alt_hold_cond`; R = `¬alt_acq_armed_last ∨ ¬raActive`.
  - So the submode follows the ALT*/ALT geometry while an RA is active and ALT is armed; when the RA clears both
    submode latches reset → VS on the next frame (inferred timing: ALT*/ALT `tcasOk` sees the previous-frame submode, so
    an ALT_ACQ/ALT_HOLD submode can hand over to ALT*/ALT on the RA-clear frame).
- `tcas_alt_acq_cond` = `captureCond` (2549, 4042); `tcas_alt_hold_cond` = confirmed `|target − H| < 40` (2610, 4043).
- `tcas_ra_corrective` (1650), `longi_large_box_tcas` = 10 s MTrig on `TCAS_last ∧ (CLB ∨ DES ∨ FINAL ∨ G/S ∨ ALT
  armed)_last` (1626–1632, 1668), `trk_fpa_deselected` = 10 s MTrig after TCAS ends if the FCU switched to HDG-V/S
  during TCAS (latch `_e` 1618 / `Delay_DSTATE_o`, 1612–1625, 1667).
- In the law priority, `TCAS ∧ submode VS` → VS law, `ALT_ACQ` → ALT_ACQ law, `ALT_HOLD` → ALT_HOLD law (3765–3776).

---

## ALT soft mode  (`alt_soft_mode_active`, latch `Memory_PreviousInput_ak`, FmgcComputer.cpp:3734)

`cond = ALT (this frame) ∧ fms_flight_phase == Cruise (3) ∧ machMode (Delay_DSTATE_c) ∧ athrEngaged (Delay_DSTATE_k)`
(3713–3715), `dV = |speedTarget − CAS|` (`rtb_Nosewheel_c` = speed target, `rtb_y_j` = CAS; 3716).

- **Set**: `ConfirmNode(cond ∧ dV ≤ 4 kt ∧ ¬resetCond, rising, 120 s)` (`CompareToConstant_const_h = 4`,
  `ConfirmNode_timeDelay_mf = 120`) — 3728–3730
- **Reset** (`rtb_y_nu` 3726): `¬cond ∨ ConfirmNode(dV > 4, 10 s) ∨ ConfirmNode(dV > 10, 4 s) ∨ CAS > VMAX − 5 ∨
  fcu.w1.bit15 (SPD/MACH value changed)` (`ConfirmNode1_timeDelay_df = 10`, `CompareToConstant1_const_c = 10`,
  `ConfirmNode2_timeDelay_i = 4`, `Bias_Bias = −5`, `BitfromLabel3_bit_mz = 15`) — 3717–3727
- i.e. after 2 minutes of stable Mach cruise in ALT with A/THR, the soft altitude mode engages; it drops on speed
  deviation, VMAX proximity or a speed/Mach target change. Consumed by the outer loops (4159) and a rate limiter (4577).

---

## Signal dictionary

| Name | Generated temp + line | Meaning | Units |
|---|---|---|---|
| `oppModeSync` | `rtb_OR2_l` 1547 | follow opp FMGC modes | bool |
| `ownCompute` | `rtb_y_oy` 1687 | compute own modes | bool |
| `anyApFd` | `apCondition` 1558 | any AP/FD engaged | bool |
| `modeReset` | `rtb_BusAssignment_m_ap_fd_logic_lateral_mode_reset` 1559 | all-mode reset | bool |
| `onGround` | `rtb_y_hy` 1273 | on ground (FAC dw5) | bool |
| `airborne5s` | `rtb_y_ew` 1278 | airborne > 5 s | bool |
| `H` | `rtb_BusAssignment_ic/j/gt_logic_altitude_indicated_ft` (`rtb_y_gd`) | indicated altitude | ft |
| `H_fcu` | `rtb_y_lo` 2410, `rtb_y_i3` 1637 | FCU selected altitude | ft |
| `target` / `alt_sel_or_cstr` | `rtb_DataTypeConversion_cm` 1660 | ALT*/ALT target | ft |
| `alt_cstr_applicable` | `rtb_NOT_oj` 1655 | target is the constraint | bool |
| `capturingCst` | `rtb_AND_j` 1640 | ALT*/ALT on a constraint | bool |
| `VS_inertial` | `rtb_y_i3` 2480 | IR inertial V/S | ft/min |
| `Dcapt` | expression 2513–2516 | ALT* capture distance | ft |
| `captureCond` | `rtb_NOT_b` 2513 → `rtb_AND1_c0` 2549 | ALT* geometric condition / `tcas_alt_acq_cond` | bool |
| `altHoldCond` | `rtb_y_fi` 2559 → `rtb_AND8` 2610 | |dH| < 40 for 0.8 s / `tcas_alt_hold_cond` | bool |
| `awayFromFcuAlt` | `rtb_OR_b` 2444–2446 | selected V/S/FPA points away from FCU alt | bool |
| `fcuAltInhibit` | `rtb_NOT_b` 2476–2479 | 3 s after FCU alt change | bool |
| `u_ly` | `FmgcComputer_B.u_ly` 2626 | target at ALT* engagement | ft |
| `reversionVs` | `rtb_Compare_ji` 2646 → `rtb_BusAssignment_jh_..._reversion_vs` 2720 | V/S reversion | bool |
| `vsFpaLatch` | `Memory_PreviousInput_fg` 2697 | V/S-or-FPA | bool |
| `vs_active` | `rtb_AND12` 2717 | V/S | bool |
| `fpa_active` | `rtb_BusAssignment_jh_ap_fd_logic_longitudinal_modes_fpa_active` 2718 | FPA | bool |
| `tcasRaAvail` | `rtb_NOT1_i` 1360 (`rtb_y_n` = ¬ until 3382) | TCAS TA/RA mode > 900 ft RA | bool |
| `raActive` | `bit(RA word, 20)` 2519/2563/2642 | RA in progress | bool |
| `raCorrective` | `rtb_y_d` 1568 | RA not preventive | bool |
| `tcas_vs_target` | `rtb_Phi_c_deg` 1599–1607 | TCAS V/S target | ft/min |
| `active_tcas_submode` | `rtb_mode` 3320 | VS / ALT_ACQ / ALT_HOLD | enum |
| `vs_target_not_held` | `rtb_y_n` 3382 | speed protection overriding V/S/FPA | bool |
| `crzLatch` | `Memory_PreviousInput_no` 2596 | within 60 ft of cruise alt in ALT | bool |
| `machMode` | `Delay_DSTATE_c` 3535 | speed target is Mach | bool |
| `athrEngaged` | `Delay_DSTATE_k` 1528 | A/THR engaged (inferred) | bool |
| `apOwn` | `Delay_DSTATE_p` 1482 | own AP engaged (inferred) | bool |

## Open questions

1. **V/S vs CLB race on constraint sequencing**: both latch on the same frame; why V/S wins in the stock replays needs
   the CLB set condition's duration (CLB doc) — not resolved here.
2. `onGround` decoding of FAC dw5 bits 17/18/19 is inferred from usage (AP disconnect on ground, 5 s liftoff confirm).
3. Exact names of `Delay_DSTATE_p` (own AP) / `Delay_DSTATE_k` (A/THR) / `fdOwnOff` (own FD after 1420) are inferred
   from the engagement logic at 1420–1547.
4. TCAS hand-over timing between submode latches (`_h`, `_cp`, reset by `¬raActive`) and the ALT*/ALT `tcasOk`
   terms (which read the previous-frame submode) should be checked against a recording with an RA; none in the corpus
   was examined.
5. The speed-protection reversion terms (7) are compiled in but disabled by `Constant_Value_m = 0`; intent unknown.
6. Inventory gap: `docs/latches.tsv` misses latches whose index expression wraps onto the next line or has S second:
   `Memory_PreviousInput_n` (2418, ALT arm possible), `_fg` (2697, V/S-FPA), `_n0` (3266, TCAS armed), plus `_d` (1872),
   `_nt` (2875, des_armed), `_kr` (4324), `_km` (4337).
