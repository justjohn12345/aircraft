# Takeoff, go-around and approach vertical modes

Readable specification of the SRS (takeoff / go-around), G/S, FINAL, LAND and FLARE logic in the generated
`FmgcComputer::step()`. Line numbers refer to `tools/fmgc-lab/build/models/stock/FmgcComputer.cpp` (identical to
upstream master `fbw-a32nx/src/wasm/fbw_a320/src/model/FmgcComputer.cpp`); `OL:` line numbers refer to
`FmgcOuterLoops.cpp` in the same folder. Parameter values come from `docs/params-FmgcComputer.txt` and
`docs/params-FmgcOuterLoops.txt`.

## Conventions used below

- Every mode is an SR latch, `Q' = R ? false : (S ? true : Q)` (reset wins). In the generated index
  `Logic_table[((S << 1) + R) << 1) + Q]` the term shifted twice is the set. Watch out: some latches are written as
  `(R + (S << 1)) << 1` (glide arm, LAND arm), so read the shifts, not the order.
- `prev.X` = `FmgcComputer_DWork.Delay_DSTATE.X`, the whole `ap_fd_logic` output of the previous frame.
- `Confirm(u, rising, T)`: with `rising` the output goes true after `u` has been true for T s and false at once;
  with `falling` the output goes true at once and false only after `u` has been false for T s (`readable/blocks.h`).
  `Rise(u)` / `Fall(u)`: one-frame edge pulses. `Mono(u, T)`: monostable, true for T s after a rising edge.
- **Opposite-FMGC sync.** Every mode below has the same set structure:
  `S = (oppBusBit && oppSync) || (ownLogic && ownConditions)`, where
  `oppSync = rtb_OR2_l` (1547, output as `ap_fd_logic.fmgc_opp_mode_sync`, 4020) and `ownLogic = rtb_y_oy = !oppSync`
  (1687; not reassigned before 3828). When this FMGC is the slave it copies the mode bit from the other FMGC's bus; the
  reset terms are always computed locally. Below, "opp bit" names the bit the sync term reads and
  `own` stands for `ownLogic`.
- **otherVerticalModes(except X)**: the OR of `prev.longitudinal_modes.{vs, fpa, alt_acq, alt_hold, clb, des, exp_clb,
  exp_des, final_des, gs_capt, gs_trk, op_clb, op_des, pitch_goaround, pitch_takeoff, tcas}` and
  `prev.lateral_modes.land_active`, minus the mode itself. Each mode's reset is normally
  `lateralModeReset || (otherVerticalModes && !Confirm(S, falling, 0.72 s))`: another mode takes over, but not while
  this mode's own set condition is (or was within the last 0.72 s) true. Exceptions are noted.
- **lateralModeReset** = `rtb_BusAssignment_m_ap_fd_logic_lateral_mode_reset` = `rtb_AND10_b` (1553):
  `!(AP own || AP opp || FD own || FD opp engaged) || (Rise(engine running) && onGround)` (1548–1553). It resets every
  mode when no AP or FD is engaged on either side.

## Common signals

| name | definition | lines |
|---|---|---|
| `onGround` (`rtb_y_hy`) | FAC discrete word 5 (own FAC if healthy, else opp) bits 18 and 19 set, bit 17 clear, SSM not FW. FAC packs bits 18/19 from the LGCIU main-gear-compressed bits, bit 17 = no valid LGCIU (inferred from `FacComputer.cpp` 426–466, 1347–1366) | 1263–1273 |
| `flapHandle` (`rtb_handleIndex`) | 1..5 from FAC word 5 bits 11..15 (first bit set), 0 if none (FAC packs `handle == 1..5`) | 1280–1302 |
| `tlaOwn`, `tlaOpp` | `fadec_{own,opp}_bus.selected_tla_deg`, A429 value-or-0 | e.g. 1673–1680 |
| `toga(tla)` | `tla >= 36.7°` (above the MCT/FLX detent) | 1675, 1681, 1999, 2002 |
| `flx(tla, flexWord)` | `33.3° < tla < 36.7°` and that FADEC's `selected_flex_temp_deg` SSM is NormalOperation | 1677–1684 |
| `appPush` | `Rise(fcu_discrete_word_1 bit 23)` = APPR pushed | 1800, 1868, 1933 |
| `locPush` | `Rise(fcu_discrete_word_2 bit 13)` = LOC pushed | 1837, 1950, 2094 |
| `ra` | raw value of the selected radio altimeter: own if valid, else opp, **2500 ft if both report FailureWarning** | 1076–1094 |
| `raAvail` (`rtb_y_d2_tmp`) | not both RAs FailureWarning (`raOwnInvalid` at 1096 actually means "both RAs failed") | 1096, 1395 |
| `engineRunning` | `!eng_opp_stop || !eng_own_stop` | 1310 |
| `apOwn` (`Delay_DSTATE_p`) | AP own engaged (updated this frame at 1482) | 1482, 1534 |

FMGC bus bits used by the sync terms (packing at 4807–4878; `index + 10`):
word 1: 15 pitch T/O, 16 pitch G/A, 20 "track" (ALT or G/S trk), 21 "capture" (ALT* or G/S*), 22 G/S (capt or trk),
23 FINAL, 25 FLARE, 28 `tla_to_ga_set`; word 2: 15 roll G/A, 16 HDG, 17 TRK; word 3: 20 LAND arm, 22 G/S arm,
23 FINAL arm; word 4: 14 LAND active.

---

## SRS takeoff (`longitudinal_modes.pitch_takeoff_active`, latch `Memory_PreviousInput_k`, FmgcComputer.cpp:1696)

Output at 3989–3990 (`= Memory_PreviousInput_k`).

**Set (engage)** — `rtb_AND4_o` (1688–1693), any of:
- opp sync: `V2 > 0 && oppBit(word1 bit 15) && oppSync` — 1669, 1688–1689
- own, all of:
  - `!prev.pitch_takeoff_active && !prev.pitch_goaround_active` — 1690–1691
  - `Confirm(onGround, rising, 30 s)`: on ground for at least 30 s (`ConfirmNode_timeDelay_i = 30`) — 1671
  - `fms_inputs.v_2_kts > 0` (V2 entered) — 1692
  - `flapHandle >= 1` (inferred: flaps not 0) — 1692
  - `Rise(takeoffThrust)` with `takeoffThrust = toga(tlaOpp) || flx(tlaOpp) || toga(tlaOwn) || flx(tlaOwn)`:
    the moment either thrust lever reaches FLX/MCT (with a FLEX temperature on that FADEC) or TOGA — 1673–1685

**Reset (disengage)** — any of (1696–1712):
- `lateralModeReset` (not gated: SRS cannot exist without AP/FD) — 1696
- `otherVerticalModes(except pitch_takeoff) && !Confirm(S, falling, 0.72 s)` — 1697–1712 (the list includes
  `pitch_goaround` and `land_active`)

**Notes**
- SRS ends only when another vertical mode engages (CLB/OP CLB at acceleration altitude, V/S, ALT*, ... decided by
  those modes' own logic) or when all AP/FD are off. There is no altitude or time limit in this latch.
- The 30 s on-ground confirmation is the takeoff/go-around discriminator: the go-around latch uses
  `!Confirm(onGround, rising, 30 s)` (1994). Thrust levers set within 30 s of touchdown give GA, not T/O.
- Engagement needs the thrust-lever *edge*: levers already in FLX/TOGA when the other conditions become true do not
  engage SRS.
- SRS active for 0.9 s produces a rising edge that arms/engages A/THR (`Confirm(pitchGA || pitchTO, rising, 0.9 s)` →
  `Rise` → A/THR latch set term, 1494–1499, 1528–1529).
- Speed reference (for the SRS law; selection not fully traced here, 3605–3649): all engines `V2 + 10` (PFD target V2);
  one engine out `max(V2, min(V2 + 15, IAS memorised at engine failure))`.

## SRS go-around (`longitudinal_modes.pitch_goaround_active`, latch `Memory_PreviousInput_i`, FmgcComputer.cpp:2026)

Output at 3991–3992 (`= Memory_PreviousInput_i`). The latch is unnamed in `latches.tsv`; this is it.

**Set (engage)** — `rtb_y_gg` (2004–2005), any of:
- opp sync: `oppBit(word1 bit 16) && oppSync` — 1988–1991
- own, all of:
  - `!prev.pitch_takeoff_active && !prev.pitch_goaround_active` — 1993, 2004
  - `!Confirm(onGround, rising, 30 s)`: airborne, or on ground for less than 30 s — 1994, 2005
  - `flapHandle >= 1` (`CompareToConstant_const_ja = 1`) — 1996
  - `Rise(toga(tlaOpp) || toga(tlaOwn))`: a thrust lever reaches TOGA (FLX does not count) — 1997–2003

**Reset (disengage)** — `(lateralModeReset || otherVerticalModes(except pitch_goaround)) && !Confirm(S, falling, 0.72 s)`
(2006–2025). The list includes `pitch_takeoff`, `final_des`, `gs_*` and `land_active`.

**Notes**
- No flight-phase, altitude or approach condition: any TOGA edge in flight with flaps extended engages GA.
- Unlike SRS T/O, `lateralModeReset` is inside the 0.72 s gate, so GA survives for at least 0.72 s even when no AP/FD
  is engaged at that instant (inferred purpose: the FDs come back on at go-around; `fdOwn` conditions include
  `pitch_goaround_active`, 1380–1386).
- From LAND/FLARE: GA sets on the TOGA edge (`land_active` in its reset is gated by the 0.72 s confirm), and the next
  frame LAND resets because `prev.pitch_goaround_active` is in LAND's reset (1896). FLARE then drops with LAND.
- GA also drives: GA TRK (`Memory_PreviousInput_el`, 2052; set by `Mono(prev.pitch_goaround, 0.72 s)` when no
  rollGA/NAV/HDG/TRK, 2033–2040); NAV arming within 0.72 s of GA engagement when no HDG/TRK preset (2162–2170); LAND/FINAL disarm (1847, 1953).
- AP disconnects on ground when `onGround && (prev.pitch_goaround || toga(tlaOpp) || toga(tlaOwn))` (AP latch reset,
  1475–1477).
- Speed reference (3623–3641): `max(Vapp_memo, min(VLS + 25 (all engines) / + 15 (one engine out), VMAX − 5,
  IAS at GA engagement))`; `vMemoGa` tracks IAS while GA is not active (3615–3617).

## `tla_to_ga_set` (`ap_fd_logic.tla_to_ga_set`, FmgcComputer.cpp:4044)

`tla_to_ga_set = rtb_AND_j` (1683) `= toga(tlaOpp) || flx(tlaOpp) || toga(tlaOwn) || flx(tlaOwn)` — the same
"takeoff thrust" condition whose rising edge engages SRS T/O. It is a level, not an edge, and is only exported (FMGC
word 1 bit 28, 4857); no mode logic in the FMGC reads it. The GA engage uses its own TOGA-only comparison (1997–2003,
same 36.7° threshold).

---

## G/S arm (`armed_modes.glide_armed`, latch `Memory_PreviousInput_fm`, FmgcComputer.cpp:3162)

Index written as `(R + (S << 1)) << 1`.

**Set** — `rtb_Compare_ji` (3157), any of:
- opp sync: `oppBit(word3 bit 22) && oppSync` — 3149–3152
- own: `Rise(prev.land_armed || prev.land_active) && !prev.gs_capt_active && !prev.gs_trk_active` — 3154–3159

**Reset** — any of:
- `Fall(prev.land_armed)` (`PulseNode_isRisingEdge_lz = 0`) — 3160–3161
- `prev.gs_capt_active || prev.gs_trk_active` — 3163–3164

**Notes** — G/S arm is a slave of LAND arm: it arms on the frame after LAND arms and disarms when LAND disarms. It
also disarms at G/S* capture, and when LAND arm falls because LAND engages (by then G/S is already captured).

## G/S* (`longitudinal_modes.gs_capt_active`, latch `Memory_PreviousInput_nu`, FmgcComputer.cpp:3215)

**Set (engage)** — `rtb_OR_b` (3194), any of:
- opp sync: `oppBit(word1 bit 21) && oppBit(word1 bit 22) && oppSync` (capture && G/S) — 3170–3176
- own, all of (3190–3193):
  - `prev.glide_armed`
  - `prev.loc_cpt_active || prev.loc_trk_active` (LOC must already be captured)
  - G/S deviation SSM = NormalOperation (`_ie`, 3189)
  - `|dev| < 0.133°`, or `(gsLag < gsLagPrev && |dev| < 0.8°)`, where `dev` = selected ILS
    `glideslope_deviation_deg` (value-or-0), `gsLag = Lag(dev, C1 = 1 rad/s)` and `gsLagPrev` its previous-frame value
    (Simulink DetectDecrease, `DelayInput1_DSTATE_b`, 3181, 5113)

**Reset (disengage)** — `lateralModeReset || (otherVerticalModes(except gs_capt) && !Confirm(S, falling, 0.72 s))`
(3197–3214; the list includes `gs_trk_active` and `land_active`).

**Notes**
- The "deviation decreasing" test uses the **signed** filtered deviation, so the early (0.8°) capture works from one
  side of the beam only; from the other side capture waits for 0.133°. Which side depends on the sim's sign of
  `NAV GLIDE SLOPE ERROR` (open question).
- G/S* is left for G/S (via `gs_trk_active` in the list) or LAND (`land_active`).

## G/S (`longitudinal_modes.gs_trk_active`, latch `Memory_PreviousInput_as`, FmgcComputer.cpp:3258)

**Set (engage)** — `rtb_OR_b` (3236), any of:
- opp sync: `oppBit(word1 bit 20) && oppBit(word1 bit 22) && oppSync` (track && G/S) — 3217–3223
- own: `Confirm(G/S* latch, rising, 15 s) && |dev| < 0.133°` — 3225–3237. The confirm reads the G/S* latch value
  just written this frame (`Memory_PreviousInput_nu`), not the delayed output.

**Reset** — `lateralModeReset || (otherVerticalModes(except gs_trk) && !Confirm(S, falling, 0.72 s))` (3240–3257;
includes `gs_capt_active` and `land_active`).

**Notes**
- G/S* lasts at least 15 s (`ConfirmNode_timeDelay_m = 15`) and then needs the deviation within 0.133°.
  If G/S* lasts until LAND engages (below 400 ft), G/S never shows.
- Reversion: `(prev.gs_trk || prev.gs_capt) && Fall(prev.land_armed || prev.land_active)` sets the V/S–FPA latch
  (`rtb_AND12`, 2611–2616, 2646), which then resets G/S through the list. Selecting V/S, FPA, OP CLB/DES or EXPED with
  G/S engaged disarms LAND (1847–1848), so that is the path by which those selections drop G/S.

---

## LAND arm (`armed_modes.land_armed`, latch `Memory_PreviousInput_l`, FmgcComputer.cpp:1839)

**Set** — `!prev.land_active && (oppSync term || own term)` (1840–1846):
- opp sync: `oppBit(word3 bit 20) && oppSync` — 1794–1796
- own, all of:
  - `ilsAppOk` (`rtb_y_o`, 1807–1810):
    `selected_approach_type == ILS` && `!ils_failure` (at least one receiver with LOC and G/S not FW, 1314–1320)
    && `(!engineRunning || ra >= 400 ft)` && `raAvail` && `appPush` && `!Confirm(prev.land_armed, rising, 0.45 s)`
    && `!prev.final_des_active && !prev.final_des_armed`
  - `!prev.pitch_takeoff && !prev.pitch_goaround && !prev.rwy_active && !prev.roll_goaround_active` — 1841–1844
  - the two ILS receivers agree (same `ils_frequency_mhz` and `runway_heading_deg`), or not both valid — 1845–1846
    (`!Constant2_Value_p` terms, value 0, are always true)

**Reset** — any of (1846–1850):
- `prev.land_active` (LAND engaged) or `prev.pitch_goaround_active`
- `prev.nav_armed` while neither G/S nor LOC is captured
- `(prev.gs_capt || prev.gs_trk) && Rise(V/S || FPA || OP CLB || OP DES || EXP CLB || EXP DES)` — 1823–1829
- `(prev.loc_cpt || prev.loc_trk) && Rise(HDG || TRK || NAV)` — 1830–1834
- `appPush && prev.land_armed` (APPR toggles it off) — 1849
- `locPush` — 1835–1838
- `lateralModeReset`, `prev.tcas_active`

## LAND active (`lateral_modes.land_active`, latch `Memory_PreviousInput_d`, FmgcComputer.cpp:1872) — brief

**Set** — `rtb_y_gg` (1860–1862): `Confirm(!prev.land_active, rising, 0.72 s)` && (
`oppBit(word4 bit 14) && oppSync` || `own && prev.gs_trk_active && prev.loc_trk_active && Confirm(ra < 400 ft, rising,
1.2 s)`).

**Reset** (1872–1898): `(otherModes && !Confirm(S, falling, 0.72 s))` — every vertical mode and every lateral mode
except LAND itself (the list includes `gs_trk` and `loc_trk`, which are reset by LAND); or `prev.pitch_goaround`; or
`!apOwn && !ap_opp_engaged && appPush && Confirm(onGround, rising, 10 s)`; or `lateralModeReset`.

**How LAND gates the vertical modes**
- LAND does not test `land_armed`; it needs G/S **track** (not G/S*), LOC track and 1.2 s below 400 ft RA.
- `land_active` is in the reset list of SRS T/O, GA, G/S*, G/S and FINAL, so engaging LAND clears G/S. The vertical
  law stays `GS` while LAND is active (`active_longitudinal_law = GS` when `Memory_PreviousInput_d`, 3758–3760) until
  FLARE takes over (3756–3757).
- `land_active` blocks re-arming LAND and FINAL (1840, 1943), sets the G/S arm edge (3154) and selects the LOC_TRACK
  lateral law (3744).
- Submodes while LAND is active (other agents' scope, listed for context): ALIGN `ra <= 30 ft` (1910–1912); ROLL OUT
  `Confirm(ra <= 50 ft && (wheel speed > 70 kt with BSCU valid, or onGround without BSCU), rising, 0.36 s)`, reset when
  LAND drops (1899–1908).

## FLARE (`longitudinal_modes.flare_active`, = `rtb_AND2_gh` at FmgcComputer.cpp:4001; memory latch `Memory_PreviousInput_bc`, 1917)

`rtb_AND2_gh` is assigned at 1913 and 1920 and not reassigned before 3756 or 4001.

```
flareCond   = raAvail && prev_outer_loops.flare_law.condition_Flare && prev.land_active      // 1913–1914
flareMemory : S = Confirm(flareCond, rising, 0.36 s), R = !prev.land_active                  // 1915–1919
flare_active = flareCond || flareMemory                                                      // 1920
```

`prev_outer_loops` is `Delay2_DSTATE`, the previous frame's `ap_fd_outer_loops` output (assigned at 5110).
`condition_Flare` is computed in the outer loops (OL:4563):

```
condition_Flare = (vertical_law == FLARE)                                   // OL:3114, rtb_Compare_ck (law enum 7)
               || (H_radio < 80 ft && (H_radio * 14 <= |vsEst| || H_radio <= 42 ft))
vsEst  = clamp(raRate, Hdot_f - slope3, Hdot_f + slope3)                     // ft/min
raRate = 60 * Lag(LeadLag(H_radio; 15s/(s+15)), C1 = 4 rad/s)  (≈ dRA/dt, ft/s → ft/min)  // OL:3118–3130, OL:4560
Hdot_f = Lag(H_dot_ft_min, C1 = 1/30 rad/s, i.e. τ = 30 s)                  // OL:3115
slope3 = V_gnd_kn * 101.27 * 0.03  (3 % of ground speed, in ft/min)          // OL:4562
```

**Set (engage)**: LAND active, at least one RA not failed, and either below 42 ft RA or below 80 ft with the estimated
time to touchdown `ra / |vsEst|` at most 60/14 ≈ 4.3 s (e.g. 50 ft at 700 ft/min).
**Reset**: only by LAND dropping (`flareMemory` reset and `flareCond` both need `land_active`).

**Notes**
- `flare_active` is true from the first frame the condition holds, but is only latched after 0.36 s; a single-frame
  blip shows FLARE for that frame only. Once the vertical law is FLARE (3756–3757 selects FLARE when `rtb_AND2_gh`),
  `condition_Flare` holds itself through `vertical_law == FLARE`.
- RETARD at ~30 ft (FBW docs) is A/THR logic, not in this latch (open question for the A/THR group).

---

## FINAL arm (`armed_modes.final_des_armed`, latch `Memory_PreviousInput_dv`, FmgcComputer.cpp:1957)

**Set** — `rtb_GreaterThan3` (1943) = `!prev.land_active && (opp || own)`:
- opp sync: `oppBit(word3 bit 23) && oppSync` — 1921–1923
- own (`rtb_y_fi`, 1939–1942), all of:
  - `fms_inputs.lateral_flight_plan_valid` and `fms_inputs.vertical_flight_plan_valid`
  - `fms_inputs.selected_approach_type == RNAV` (`EnumeratedConstant_Value_f = 2`) — 1924
  - `!engineRunning || ra >= 400 ft || both RAs failed` — 1940–1941
  - `!prev.land_armed`
  - `appPush` (1931–1934) with (`rtb_AND10_b`, 1935–1938) `!prev.pitch_takeoff && !prev.rwy_active &&
    !prev.pitch_goaround && !prev.roll_goaround_active && !prev.final_des_active`
    and `!Confirm(prev.final_des_armed, falling, 0.72 s)` (FINAL not armed during the last 0.72 s, 1928–1930)

**Reset** — `rtb_Compare_ji` (1952–1956), any of:
- `prev.final_des_active` (armed → active hand-over)
- `prev.pitch_goaround_active`, `prev.tcas_active`, `lateralModeReset`
- `!fms_inputs.vertical_flight_plan_valid` — 1944
- `Fall(prev.nav_armed || prev.nav_active)` (`PulseNode3_isRisingEdge_k = 0`) — 1945–1947
- `appPush && prev.final_des_armed` (APPR toggles it off) — 1954
- `locPush` — 1948–1951

**Notes**
- Arming does not need NAV; instead the rising edge of `final_des_armed` arms NAV (APP NAV) when the lateral plan is
  valid (NAV arm set term `Rise(prev.final_des_armed)`, 2160–2170), and the falling edge of the current-frame FINAL arm
  disarms NAV (2181–2186). Losing NAV (armed or active) disarms FINAL.

## FINAL (`longitudinal_modes.final_des_active`, latch `Memory_PreviousInput_f`, FmgcComputer.cpp:1986)

**Set (engage)** — `rtb_y_gg` (1963–1965), any of:
- opp sync: `oppBit(word1 bit 23) && oppSync` — 1959–1962
- own, all of:
  - `prev.nav_active`
  - `prev.final_des_armed`
  - `fms_inputs.final_app_can_engage`
  - `fms_inputs.fms_flight_phase == Approach` (`EnumeratedConstant_Value_i = 5`)

**Reset (disengage)** — `lateralModeReset || (otherVerticalModes(except final_des) && !Confirm(S, falling, 0.72 s))`
(1966–1985). Nothing else.

**Notes**
- **What the FMGC adds on top of the FMS flag**: only APPR push (through the arm), lateral and vertical flight-plan
  validity and RNAV approach type (arm), RA ≥ 400 ft or engines off (arm only), NAV active (engage only), approach
  phase (engage only), and the inhibitions by T/O, GA, RWY, LAND, TCAS. There is no LOC condition, no RA condition at
  engagement, and no confirmation time on engagement (the 0.72 s confirms only delay *other* modes from resetting it
  and block re-arming within 0.72 s of a disarm).
- **`final_app_can_engage` and the approach phase are engagement-only.** Once the latch is set, the FMGC never reads
  them again; FINAL survives the flag going false. The FMS-side latch in `DescentGuidance.updateFinalAppCanEngage`
  ("keep the condition latched so the FMGC does not drop the mode", branch `feat/a32nx-final-app`) is therefore not
  needed for the FMGC, though harmless.
- **How FINAL actually disengages**: through another vertical mode. The V/S–FPA reversion latch
  (`Memory_PreviousInput_fg`, 2697) has the set term `prev.final_des_active && (Fall(prev.nav_active) || appPush ||
  !vertical_flight_plan_valid)` (2617–2625, `PulseNode4_isRisingEdge_f = 0`, FCU word 1 bit 23), so NAV loss, a second
  APPR push or `A32NX_FM_VERTICAL_PROFILE_AVAIL` = 0 give V/S (or FPA); the new mode then resets FINAL through the
  list. Pilot selections (ALT, OP DES, V/S, EXPED ...) and ALT* capture act the same way through their own latches.
  The speed-protection reversion term for DES/FINAL (`VLS − 2 > IAS`, 2662–2666) is disabled by
  `Constant_Value_m = 0`.
- **NAV is protected while FINAL is active**: the NAV → HDG/TRK reversion on `!lateral_flight_plan_valid ||
  !nav_capture_condition` is inhibited when `prev.final_des_active` (2056–2059). LOC push with NAV and FINAL active
  reverts to HDG/TRK (2103–2104), which drops NAV and then FINAL. When FINAL arm/active falls while NAV is active and
  no selected/managed vertical mode is engaged, NAV also reverts to HDG/TRK (2083–2085, 2096–2102).
- FINAL uses the `VPATH` law (3779–3780), the same as DES on path; the law follows the FMS `vs_target_ft_min` /
  altitude target.
- LAND and FINAL exclude each other at arming: LAND arm needs `!final_armed && !final_active` and ILS; FINAL arm needs
  `!land_armed` and RNAV.

**FMS inputs and their L:vars** (`FlyByWireInterface::updateFmgc`, fbw-a32nx/src/wasm/fbw_a320/src/FlyByWireInterface.cpp):

| fms_inputs field | L:var | line | used for |
|---|---|---|---|
| `final_app_can_engage` | `A32NX_FG_FINAL_CAN_ENGAGE` | 402, 1793 | FINAL engage |
| `selected_approach_type` | `A32NX_FG_RNAV_APP_SELECTED` ? RNAV : ILS | 401, 1778–1779 | FINAL arm (RNAV), LAND arm (ILS) |
| `vertical_flight_plan_valid` | `A32NX_FM_VERTICAL_PROFILE_AVAIL` | 397, 1792 | FINAL arm, FINAL disarm, V/S reversion from FINAL |
| `lateral_flight_plan_valid` | `A32NX_FM_LATERAL_FLIGHTPLAN_AVAIL` | 392, 1785 | FINAL arm; NAV |
| `fms_flight_phase` | `A32NX_FMGC_FLIGHT_PHASE` | 368, 1777 | FINAL engage (== Approach) |
| `nav_capture_condition` | `A32NX_FM1_NAV_CAPTURE_CONDITION` | 403, 1786 | NAV engage; NAV loss (not while FINAL) |
| `v_2_kts` | `AIRLINER_V2_SPEED` | 369, 1798 | SRS T/O engage (> 0) and target |

On master no TypeScript writes `A32NX_FG_FINAL_CAN_ENGAGE` or `A32NX_FG_RNAV_APP_SELECTED`, so stock always sends ILS
and `final_app_can_engage = false`: FINAL can neither arm nor engage. On `feat/a32nx-final-app` they are written by
`VnavDriver.publishFinalAppApproachSelected` and `DescentGuidance.updateFinalAppCanEngage` (approach phase, XTK,
distance to the final descent point, linear deviation, path angle).

---

## Unnamed latches near these lines

| latch | line | what it is |
|---|---|---|
| `Memory_PreviousInput_bc` | 1917 | FLARE memory: set `Confirm(flareCond, rising, 0.36 s)`, reset `!prev.land_active`; `flare_active = flareCond || bc` (1920) |
| `Memory_PreviousInput_i` | 2026 | SRS go-around = `longitudinal_modes.pitch_goaround_active` (assigned directly from the latch at 3991–3992) |
| `Memory_PreviousInput_dy` | 2129 | HDG-or-TRK latch: `hdg_active = dy && FCU word 1 bit 24 (HDG/VS)`, `trk_active = dy && !bit 24` (2137–2140, output 3977–3978). Set: opp HDG/TRK bit, or own `(airborne ≥ 5 s or engines stopped on ground) && Rise(HDG/TRK pull) && !LAND && (!rollGA || ra ≥ 100 ft)`, or `airborne && Rise(any AP/FD engaged) && no lateral mode && !pitchGA`, or the HDG/TRK reversion conditions of 2096–2104. Reset: `lateralModeReset || (RWY/rollGA/LOC*/LOC/NAV/LAND && !Confirm(S, falling, 0.72 s))` |
| `Memory_PreviousInput_k` | 1696 | SRS takeoff = `pitch_takeoff_active` (3989–3990) |

(`docs/latches.tsv` should map `_k` → `pitch_takeoff_active`, `_i` → `pitch_goaround_active`; the outputs are
assigned from the latch through a line break, which is probably why the inventory missed them.)

---

## Signal dictionary

| name | generated temp + line | meaning | units |
|---|---|---|---|
| oppSync | `rtb_OR2_l` 1547 | this FMGC copies modes from the opposite FMGC | bool |
| ownLogic | `rtb_y_oy` 1687 | `!oppSync` | bool |
| lateralModeReset | `rtb_AND10_b` 1553 / `rtb_BusAssignment_m_ap_fd_logic_lateral_mode_reset` 1559 | no AP/FD engaged on either side, or engine-start edge on ground | bool |
| onGround | `rtb_y_hy` 1273 | both main gears compressed per FAC word 5 (inferred) | bool |
| airborne5s | `rtb_y_ew` 1278 | `(both engines stopped && onGround) || Confirm(!onGround, rising, 5 s)` | bool |
| flapHandle | `rtb_handleIndex` 1290–1302 | flap handle index 0..5 from FAC word 5 | - |
| ra | `rtb_raComputationData_radio_height_ft_Data` 1082–1094, read raw via `MATLABFunction_c` | selected RA, 2500 if both failed | ft |
| raAvail | `rtb_y_d2_tmp` 1395 | not both RAs FailureWarning | bool |
| bothRaFailed | `raOwnInvalid` 1096 | both RAs FailureWarning (misleading generated name) | bool |
| engineRunning | `rtb_BusAssignment_h_logic_engine_running` 1310 | at least one engine not stopped | bool |
| ilsFailure | `rtb_BusAssignment_o_logic_ils_failure` 1320 | both ILS receivers have LOC or G/S FailureWarning | bool |
| bothIlsValid | `rtb_BusAssignment_o_logic_both_ils_valid` 1321 | both receivers LOC and G/S not FW | bool |
| takeoffThrust / tla_to_ga_set | `rtb_AND_j` 1683 | any lever in FLX (with flex temp) or TOGA | bool |
| srsToSet | `rtb_AND4_o` 1688 | SRS T/O set | bool |
| srsToSetHeld | `rtb_Compare_ji` 1694 | `Confirm(srsToSet, falling, 0.72 s)` | bool |
| onGround30s (T/O) | `rtb_y_nu` 1671 | `Confirm(onGround, rising, 30 s)` | bool |
| onGround30s (GA) | `rtb_y_bn` 1994 | same, second instance | bool |
| togaEdge | `rtb_y_fi` 2002–2003 | `Rise(toga(tlaOpp) || toga(tlaOwn))` | bool |
| gaSet | `rtb_y_gg` 2004 | SRS GA set | bool |
| appPush | `rtb_NOT_b` 1800 / `rtb_Compare_ji` 1868 / `rtb_y_m2` 1933 | `Rise(FCU word 1 bit 23)` | bool |
| locPush | `rtb_Compare_ji` 1837 / `rtb_y_o4` 1950 | `Rise(FCU word 2 bit 13)` | bool |
| ilsAppOk | `rtb_y_o` 1807 | LAND arm own preconditions incl. APPR push | bool |
| landActiveSet | `rtb_y_gg` 1860 | LAND engage | bool |
| flareCond | `rtb_AND2_gh` 1913 | `raAvail && condition_Flare && LAND` | bool |
| flare_active | `rtb_AND2_gh` 1920 | `flareCond || Memory_bc` | bool |
| condition_Flare | `Delay2_DSTATE.flare_law.condition_Flare`, OL:4563 | outer-loop flare trigger | bool |
| finalArmBase | `rtb_AND10_b` 1935 | APPR push and no T/O/RWY/GA/FINAL, not armed within 0.72 s | bool |
| finalArmOwn | `rtb_y_fi` 1939 | FINAL arm own set | bool |
| finalArmSet | `rtb_GreaterThan3` 1943 | FINAL arm set | bool |
| finalArmReset | `rtb_Compare_ji` 1952 | FINAL arm reset | bool |
| finalSet | `rtb_y_gg` 1963 | FINAL engage | bool |
| finalReset | `rtb_Compare_ji` 1985 | FINAL reset | bool |
| glideArmSet | `rtb_Compare_ji` 3157 | G/S arm set | bool |
| dev | `rtb_y_bb` 3180, `rtb_Nosewheel_c` 3184 (abs) | selected ILS G/S deviation | deg |
| gsLag | `rtb_DataTypeConversion2_i` 3181 | `Lag(dev, τ = 1 s)` | deg |
| gsCptSet | `rtb_OR_b` 3194 | G/S* set | bool |
| gsCpt15s | `rtb_NOT_b` 3225 | `Confirm(G/S* latch, rising, 15 s)` | bool |
| gsTrkSet | `rtb_OR_b` 3236 | G/S set | bool |
| vsReversion | `rtb_Compare_ji` 2646 (`..._mode_reversion_vs` 2720) | V/S–FPA reversion set, incl. FINAL term at 2625 | bool |

## Open questions

1. `docs/latches.tsv` lists `Memory_PreviousInput_k` and `_i` without output fields; they are `pitch_takeoff_active`
   and `pitch_goaround_active` (3989–3992). Worth fixing in the inventory script.
2. G/S deviation sign: the early G/S* capture (`gsLag` decreasing, |dev| < 0.8°) works from one side only. Which side,
   given MSFS `NAV GLIDE SLOPE ERROR` (FlyByWireInterface.cpp 1286)? Check against a corpus approach from above.
3. The 15 s G/S* → G/S confirmation (`ConfirmNode_timeDelay_m = 15`) is longer than the FCOM implies; verify with an
   FDR probe that G/S never appears sooner than 15 s after G/S*.
4. ILS validity in the sim: `FlyByWireInterface::updateIls` sends NCD (never FW) when invalid and always frequency 0
   with the same course on both receivers, so `ils_failure` is never true and the LAND-arm receiver-agreement check
   always passes (inferred). LAND arm with no ILS signal is therefore only stopped by the approach type.
5. `flapHandle` mapping: does index 1 mean handle position 1 (so `>= 1` means "not flaps 0")? Depends on how the FAC's
   `Switch_i_idx_0` is numbered.
6. SRS pitch limits (18° / 22.5° windshear) and the minimum V/S live in the SRS law of `FmgcOuterLoops` — not
   checked here.
7. Consumer of `tla_to_ga_set` (FMGC word 1 bit 28) outside the FMGC is not found in the TypeScript; maybe unused.
8. The RA passed to the outer loops as `H_radio_ft` (for `condition_Flare`) was assumed to be the same selected RA;
   not traced through the `LawMDLOBJ1.step` argument list (4138).
