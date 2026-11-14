# Lateral modes

Readable specification of the lateral mode logic in the generated `FmgcComputer::step()`: RWY, RWY TRK, ROLL OUT,
ALIGN, GA TRK, HDG/TRK and the lateral mode reversion, NAV armed, NAV, LOC armed, LOC*, LOC, the roll FD bars, the
HDG/TRK preset, `any_lateral_mode_engaged` and the lateral law selection. LAND armed / LAND are in
`takeoff-ga-approach.md`. Code: `readable/lateral_modes.cpp` (bit-exact against the generated model, checked by
`build/fmgc-check-vertical-stock`). Line numbers refer to `build/models/stock/FmgcComputer.cpp` (= upstream master).

## Conventions

As in `takeoff-ga-approach.md`: every mode is a reset-priority SR latch (all 11 lateral latches use the default
truth table `0 1 0 0 1 1 0 0 1 0 1 1 0 0 0 0` and start false); `prev.X` is last frame's `ap_fd_logic` output;
`Confirm(u, rising|falling, T)`, `Rise(u)`, `Fall(u)`, `Mono(u, T)` (monostable, rising edge, not retriggerable) are
the `readable/blocks.h` blocks. All edge and timer blocks run every frame, whatever the rest of the condition.

- **Opposite-FMGC sync**: each set is `S = (opp bus bit && oppSync) || (own && conditions)`, `own = !oppSync`. Resets are
  always local. Opp bits (FMGC discrete word 2 = lateral modes, packing at 4859–4877): 11 RWY (either submode),
  12 NAV, 13 LOC*, 14 LOC, 15 GA TRK, 16 HDG, 17 TRK, 20 RWY LOC submode, 23 RWY TRK submode, 25 ALIGN, 26 ROLL OUT,
  28 roll FD bars flashing; word 3: 14 NAV armed, 16 LOC armed. RWY copies `bit 11 && bit 20`, RWY TRK `bit 11 && bit 23`,
  HDG/TRK `bit 16 || bit 17`.
- **Takeover rule**: most modes reset with `modeReset || (otherLateralModes(prev) && !Confirm(S, falling, 0.72 s))`:
  another lateral mode active last frame replaces this one, unless this mode's own set condition was true within the
  last 0.72 s. `otherLateralModes` = RWY (`rwy_active`), GA TRK, NAV, LOC*, LOC, HDG, TRK, LAND minus the mode itself;
  for RWY and RWY TRK the *other submode* counts instead of `rwy_active`. Note that ROLL OUT and FINAL are not lateral
  modes for this purpose.
- **modeReset** (`lateral_mode_reset` = `longitudinal_mode_reset`, 1553): no AP and no FD engaged on either side, or
  the first engine start on the ground.
- **FCU** (`fcu_discrete_word_2`, packed at `FcuComputer.cpp` 1320–1339): bit 11 HDG/TRK knob pushed, 12 knob pulled,
  13 LOC pushed, 14 lateral value changed (unused here). `fcu_discrete_word_1` bit 24: HDG-V/S selected (else TRK-FPA).
- `ra`: raw selected radio height (2500 ft when both radio altimeters failed); `raFailed` = both failed.
- `onGround`, `gnd_eng_stop_flt_5s` (on ground with both engines stopped, or airborne for 5 s) come from input
  conditioning (`logic.*`).

## RWY (`lateral_modes.rwy_loc_submode_active`, latch `Memory_PreviousInput_c`, 1754)

FMA "RWY". **Set** (1745–1749): own, not RWY last frame, flap handle ≥ 1, SRS takeoff active for 0.9 s
(`Confirm(prev.pitch_takeoff_active, rising, 0.9)`), on ground for 30 s, |ILS runway course − magnetic heading| ≤ 20°,
|localizer deviation| < 0.4° and the localizer word valid (NormalOperation). Inputs: the selected ILS
(`logic.ils_computation_data`), the selected IR heading.

**Reset**: takeover rule, `modeReset`, or the localizer word not NormalOperation (**at any height**).

## RWY TRK (`lateral_modes.rwy_trk_submode_active`, latch `Memory_PreviousInput_b`, 1782)

**Set**: own, `Mono(Rise(ra >= 30 ft), 0.9 s)` (the radio height crosses 30 ft upwards, within 0.9 s), with neither
NAV engaged nor armed last frame. **Reset**: takeover rule, `modeReset`.

`rwy_active` = RWY || RWY TRK (this frame, 1793).

## ROLL OUT (`lateral_modes.rollout_submode_active`, latch `Memory_PreviousInput_m`, 1906)

**Set**: `Confirm(touchdown, rising, 0.36 s)`, `touchdown` = LAND active last frame, `ra <= 50 ft`, and either a wheel
above 70 kt with a valid BSCU (own or opp), or no valid BSCU and on ground. **Reset**: LAND not active last frame.

## ALIGN (`lateral_modes.align_submode_active`, combinational, 1910)

LAND active last frame, `ra <= 30 ft`, and FAC rudder control available (`!logic.fac_rudder_control_failure`).

## GA TRK (`lateral_modes.roll_goaround_active`, latch `Memory_PreviousInput_el`, 2052)

**Set**: own, `Mono(prev.pitch_goaround_active, 0.72 s)` (SRS go-around has just engaged), and last frame not GA TRK,
NAV, HDG or TRK. So a go-around from NAV keeps NAV, from HDG/TRK keeps HDG/TRK, and only from LOC*/LOC/LAND (or no
lateral mode) gives GA TRK. **Reset**: `modeReset`, takeover rule.

## Lateral mode reversion (`lateral_mode_reversion`, combinational, 2054–2101, output 4035)

Computed by both FMGCs (not gated by `own`); feeds the HDG/TRK set and the roll FD bars. True when any of:

1. **NAV lost**: NAV last frame, (no lateral flight plan or the FMS NAV capture condition lost), not in FINAL.
2. **LAND lost on the localizer**: LOC*/LOC last frame, `Fall(prev.land_armed || prev.land_active)`, none of RWY,
   GA TRK, NAV, LAND last frame, none of V/S, FPA, EXP CLB/DES, OP CLB/DES, CLB, DES last frame, and LOC not pushed in
   the last 0.72 s (`Mono(LOC push bit, 0.72)`).
3. **FINAL lost in NAV**: `Fall(prev.final_des_armed || prev.final_des_active)`, NAV last frame and no RWY, GA TRK,
   LOC*, LOC, LAND, and none of the vertical modes of 2.
4. **LOC pushed on the localizer without LAND**: LOC*/LOC, no LAND armed or active, `Rise(LOC push)`.
5. **LOC pushed in FINAL APP**: NAV and FINAL last frame, `Rise(LOC push)`.

## HDG/TRK (latch `Memory_PreviousInput_dy`, 2129; `hdg_active` / `trk_active` 2139–2140)

**Set**: own and any of
- HDG/TRK knob pulled (`Rise(bit 12)`) with `gnd_eng_stop_flt_5s`, not in LAND, and (not in GA TRK or `ra >= 100 ft`);
- `Rise(any AP/FD engaged)` in flight, with no lateral mode and no SRS go-around last frame (basic mode);
- the lateral mode reversion.

**Reset**: `modeReset`, or `(RWY, GA TRK, LOC*, LOC, NAV or LAND last frame) && !Confirm(S, falling, 0.72)`.

`hdg_active` = latch && FCU word 1 bit 24; `trk_active` = latch && !bit 24: the HDG-V/S / TRK-FPA pushbutton switches
between them while engaged.

## NAV armed (`armed_modes.nav_armed`, latch `Memory_PreviousInput_j`, 2183)

**Set**: own, lateral flight plan valid, and any of
- HDG/TRK knob pushed (`Rise(bit 11)`), or `Rise(direct_to_nav_engage)` while the FMS NAV capture condition is false;
  both only when not in LOC*, LOC or LAND;
- on ground with (no HDG/TRK preset last frame, or `Rise(lateral flight plan valid)`) and (no lateral mode last frame,
  or RWY);
- `Rise(prev.final_des_armed)` (APPR pushed for an RNAV approach arms NAV with FINAL);
- `Mono(prev.pitch_goaround_active, 0.72)` with no HDG/TRK preset.

**Reset**: `Rise(prev.hdg_trk_preset_available)`, knob pulled, `Rise(LOC*, LOC, LOC armed or LAND)`, no lateral flight
plan, `Fall(FINAL armed)` (**this frame's** FINAL armed latch), NAV engaged last frame, no AP/FD engaged on either side.

## NAV (`lateral_modes.nav_active`, latch `Memory_PreviousInput_ly`, 2214)

**Set**: own, FMS NAV capture condition, (NAV armed last frame, or `Mono(Rise(DIR TO), 0.72)` when not in LOC*, LOC or
LAND), (`ra >= 30 ft` or both RA failed), and (not in GA TRK or `ra >= 100 ft`).

**Reset**: no lateral flight plan, `modeReset`, takeover rule. The capture condition going false does *not* reset NAV
directly: it triggers the reversion (HDG/TRK), which then takes over.

## LOC armed (`armed_modes.loc_armed`, latch `Memory_PreviousInput_a`, 2241)

**Set**: own and either `Rise(prev.land_armed || prev.land_active)` (APPR arms LOC with LAND), or LOC pushed
(`Rise(bit 13)`) with an ILS available (`!logic.ils_failure`), `ra >= 400 ft`, FMS flight phase neither TAKEOFF nor
GOAROUND.

**Reset**: `Fall(prev.land_armed)` unless LOC was armed by a push in the last 0.72 s, LOC* or LOC last frame, SRS
go-around, LOC pushed again while LOC armed without LAND, `Rise(prev.nav_armed)`.

## LOC* (`lateral_modes.loc_cpt_active`, latch `Memory_PreviousInput_cv`, 2374)

Geometry (2261–2357, single precision):
- `courseErr` = memorised runway course (`logic.rwy_hdg_memo`) − magnetic heading, wrapped to [−180°, 180°];
- `bankReached`: the LOC capture law's bank command of the previous step (`Phi_loc_c`, outer loops) points towards the
  course (`sign(courseErr) == sign(Phi_loc_c)`), and either |Phi_loc_c| > 5° while the aircraft is within 5° of wings
  level or banked the other way, or the command is at least as large as the bank in the same direction;
- `capture` (only if |courseErr| < 115°): (|courseErr| > 25°, |dev| < 10°, deviation on the other side of the course
  error, `bankReached`) or |dev| < 1.92°; then `bankReached || (|courseErr| < 15° && |dev| < 1.1°)`.

**Set**: own, LOC armed last frame, `capture`, localizer word NormalOperation. **Reset**: takeover rule, `modeReset`.

## LOC (`lateral_modes.loc_trk_active`, latch `Memory_PreviousInput_lq`, 2404)

**Set**: own, LOC* last frame, |dev| < 0.16° for 10 s (`Confirm(rising, 10)`). **Reset**: takeover rule, `modeReset`.

## Roll FD bars flashing (`roll_fd_bars_flashing`, 3392–3404)

`Mono(reversion || (Rise(any AP/FD engaged) && !TCAS this frame), 10 s)`.

## HDG/TRK preset (`hdg_trk_preset_available`, latch `Memory_PreviousInput_bw`, 3434)

- `guided` = this frame's LOC*, LOC, LAND, FINAL, GA TRK or RWY TRK;
- `low` = `guided || onGround || (ra < 30 ft && !raFailed)`.

**Set**: `Mono(selected HDG valid || selected TRK valid || knob pulled, 0.72)` (an FCU heading/track value appears),
`(Confirm(NAV armed this frame, falling, 0.72) || guided)`, and `low`.
**Reset**: `!low`, HDG or TRK engaged, NAV engaged (this frame), `Mono(knob pushed, 0.72)`,
`Confirm(no AP/FD engaged, falling, 0.72)`.

## any_lateral_mode_engaged (3736, output 3798)

RWY || RWY TRK || GA TRK || LOC* || LOC || HDG || TRK || NAV || LAND (this frame). ROLL OUT alone does not count.

## Lateral law selection (3740–3753)

First match wins (this frame's modes):

| law | modes |
|---|---|
| ROLL_OUT | ROLL OUT, **or RWY** (the takeoff runway centreline uses the roll-out law) |
| LOC_TRACK | LOC, or LAND |
| LOC_CPT | LOC* |
| HPATH | NAV |
| TRACK | TRK, RWY TRK, GA TRK |
| HDG | HDG |
| NONE | otherwise |

## `loc_bc_selection` (4039)

The constant `rtP_fmgc_ap_fd_logic_output_MATLABStruct.loc_bc_selection` = false: there is no back-course logic (the
FMS `backbeam_selected` only inverts the localizer deviation in input conditioning).

## Thresholds and times

| what | value |
|---|---|
| RWY: SRS T/O for / on ground for | 0.9 s / 30 s |
| RWY: heading vs course, localizer deviation | ≤ 20° (not wrapped), < 0.4° |
| RWY TRK: radio height crossing, window | 30 ft, 0.9 s |
| ROLL OUT: radio height, wheel speed, confirm | ≤ 50 ft, > 70 kt, 0.36 s |
| ALIGN: radio height | ≤ 30 ft |
| GA TRK, NAV arm after SRS G/A, presets, pushes | 0.72 s pulses |
| HDG/TRK pull and NAV in GA TRK | ≥ 100 ft RA |
| NAV engagement | ≥ 30 ft RA |
| LOC arming by LOC push | ≥ 400 ft RA |
| LOC*: course error, wide / normal / close capture | < 115°; > 25° & < 10°; < 1.92°; < 15° & < 1.1° |
| LOC*: bank command vs roll | 5° |
| LOC: deviation, confirm | < 0.16°, 10 s |
| roll FD bars flashing | 10 s |
| takeover delay | 0.72 s |

## Differences from the FCOM (A320 FCOM DSC-22_30-60, DSC-22_30-80-20/30, public text)

1. **RWY heading check is not wrapped at 360°**: `|course − heading| ≤ 20` with raw values, so a runway with a course
   near north (e.g. 002°) is rejected when the heading reads 359°. FCOM: heading within 20° of the ILS course. The
   FCOM's "ILS course identical to the runway heading of the origin airport" check does not exist.
2. **RWY disengagement**: only a non-normal localizer word (at any height, not only below 30 ft) or another mode; a
   heading difference > 20° after engagement does not disengage RWY.
3. **Go-around lateral mode**: SRS go-around gives GA TRK only when the aircraft was not in NAV, HDG or TRK; from NAV
   it stays NAV (the "SRS/NAV" behaviour of newer standards), and from HDG/TRK it stays in HDG/TRK at any height (FCOM:
   HDG only below 100 ft). GA TRK can be left for HDG/TRK or NAV only above 100 ft RA, as in the FCOM.
4. **DIR TO** engages NAV unless LOC*, LOC or LAND is engaged, at any height (FCOM: except below 700 ft RA in LOC).
   DIR TO with the capture condition false arms NAV instead.
5. **LOC push** is refused in the TAKEOFF and GOAROUND *flight phases* (FCOM: when TO or GA *mode* is engaged), and
   needs only `!ils_failure` (FCOM: frequency and course tuned). LOC pushed in LOC*/LOC without LAND reverts to
   HDG/TRK (FCOM: LOC remains engaged and G/S reverts to V/S): here the reversion sets HDG/TRK, which takes over LOC.
6. **NAV disarm by LOC**: NAV armed is reset on the rising edge of LOC armed, LOC*, LOC or LAND (FCOM: arming LOC,
   LAND engaging) and also when FINAL armed drops.
7. **Preset**: available on the ground / below 30 ft RA while NAV is (recently) armed, and in LOC*, LOC, LAND, FINAL,
   GA TRK, RWY TRK; it triggers on an FCU selected heading/track becoming valid *or the knob pulled* (the knob-pulled
   bit is used where the lateral-value-changed bit 14 would be expected). Cancelled by pushing the knob, NAV engaging,
   HDG/TRK engaging, no AP/FD for 0.72 s, or leaving those conditions (climbing above 30 ft without a guided mode).
8. **HDG/TRK basic mode on AP/FD engagement** only in flight; on the ground the knob pull needs both engines stopped.
9. **ROLL OUT** uses wheel spin-up (> 70 kt) from a valid BSCU, or on ground when no BSCU is valid; ALIGN needs FAC
   rudder control. ALIGN is not a latch: it is LAND below 30 ft.
10. **No LOC back course**: `loc_bc_selection` is constant false.
11. **RWY uses the ROLL_OUT law**, LAND the LOC_TRACK law; ROLL OUT is not counted in `any_lateral_mode_engaged`.

## Verification (2026-11-14)

`JOBS=6 scripts/check-vertical-corpus.sh 3`: 22 recordings × (resync, free, 3 × fuzz, free+fuzz, approach, lateral,
approach+lateral) = 374 runs, 0 mismatches on every vertical and lateral signal and on `active_lateral_law`.

The recordings alone fly mostly NAV (RWY TRK, the preset and LOC arming by the LOC button never occur); the lateral
scenario (`--lateral`, `src/scenario_lateral.h`) brings every lateral signal and every law value to hundreds or
thousands of engagements. Still thin: ROLL OUT (≈15 engagements per corpus run, only with the approach scenario).
Never true: `loc_bc_selection` (constant).

Mutation test (throwaway generator over `lateral_modes.cpp`: `&&`/`||` swaps, dropped `!`, comparison strictness and
direction, constants ×1.25 / ×0.8 / ±1 bit, edge-detector and monostable flags, stuck-at true/false on every input and
named condition): 1049 compiling, non-identical mutants, 1015 killed (96.8 %; 98.4 % excluding the provably
equivalent ones). The 34 survivors:

- **Equivalent (18)**: float compared with a non-representable double (0.4, 1.92, 1.1, 0.16) so `<` = `<=`;
  `|x|` of ±0; `d < 0` after the `d == 0` branch; course error exactly ±180° (outside 115°); `|roll| <= 5` vs `< 5`
  (both branches give true); the flap handle is an integer (≥ 0.8 = ≥ 1); a set term that also resets (localizer
  invalid in RWY, `!guidedOrLow` in the preset, no flight plan in NAV armed: reset wins); `dual_ra_failure` terms
  (both RAs failed means the raw radio height is 2500 ft); `bankReached` stuck true inside the wide-capture term
  (the close capture cannot hold above 25°); `any_lateral_mode_engaged` at an AP/FD engagement edge (modes are held
  reset while no AP/FD is engaged); the bool of `ActiveLateralLaw` (the enum is compared).
- **Measure-zero equalities (2)**: `|Phi_loc_c| > 5` and `|Phi_loc_c| >= |roll|` need a double outer-loop command
  exactly equal to 5 or to the float roll.
- **Transition frames only (8)**: terms that need NAV together with RWY / GA TRK / LOC* / LOC / LAND, LOC* together
  with LAND, or LOC armed together with LAND in the same previous frame; these overlap only for the one frame of a
  takeover.
- **Uncovered (6)**: RWY's own 30 s on-ground timer (redundant with SRS takeoff's unless on-ground drops after SRS
  engages); the opposite BSCU alone valid at touchdown; the monostables on SRS go-around being retriggerable (needs
  SRS G/A to re-engage within 0.72 s, only via opposite-FMGC sync); LAND engaging without LAND armed (only via
  opposite-FMGC sync) for LOC arming; TCAS active at an AP/FD engagement (roll FD bars).
