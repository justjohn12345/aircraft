# Findings index

Every issue found while reverse-engineering the A32NX FMGC, in one place. Details stay in the stage docs (linked);
this file says what each one is, whether it shows in the sim, and what has been done about it. The readable rewrite
reproduces all of them (stock-faithful); fixes live on personal/fmgc-fixes or in upstream PRs.

Status: **fixed (personal)** = hand edit in personal/fmgc-fixes, in the personal build; **PR/issue** = upstream;
**open** = not acted on; **declined** = left for FBW. "Checked" = confirmed against the source by hand, not only by the
agent that found it.

## 1. Bugs outside the FMGC (code that reads the FMGC outputs)

These are in FBW's TypeScript/C++ consumers, not in the generated model, so they're the easiest to fix and report.

| # | Finding | Effect in the sim | Where | Status |
|---|---|---|---|---|
| C1 | FMA borders read FLARE from discrete word 2 bit 25, which is ALIGN; FLARE is word 1 bit 25 | none (latent): the border test reduces to "LAND or ROLL OUT"; LAND stays set through FLARE and ROLL OUT and both FLARE and ALIGN need LAND, so either bit gives the same result. FLARE text reads the right bit (FMA.tsx:1391) | `PFD/FMA.tsx:147` | open, checked, latent |
| C2 | FMA vertical-column reversion read uses discrete word 3 bit 18, never packed (the FMGC packs `ap_fd_mode_reversion` at word 4 bit 28 as a 1 s triple-click pulse). Introduced by #7587 (afff8d290), which replaced the old held `fmaModeReversion` simvar | none: the flashing white reversion box is dead code. Stock shows what the FCOM describes (DSC-22_30-40, 2023 FCOM: triple click, new mode boxed 10 s, FD bars flash 10 s); the flashing box would not match the FCOM if it worked | `PFD/FMA.tsx:973` (box 975–981, flash 1081) | dead code; fix branch `fix/a32nx-fma-mode-reversion-box` (local, not to be PR'd) |
| C3 | Vertical-mode shim has no ALT CRZ case (ALT/ALT* need `!dw1 b26`) | `A32NX_FMA_VERTICAL_MODE` = 0 in ALT CRZ; the PFD is fine, but Phase.ts and VerticalProfileManager.ts read the shim | `FlyByWireInterface.cpp` ~1959–1973 | open |
| C4 | FMA reads ATS FMA word bit 29 ("MAN DTO"), never packed | that indication can never show | `PFD/FMA.tsx:523` | open |
| C5 | Shim armed bitmask never sets ALT CST or TCAS armed | consumers of `A32NX_FMA_VERTICAL_ARMED` never see them | `FlyByWireInterface.cpp` ~2007 | open |
| C6 | FCU reads TRK/FPA reset, Mach (ATS bit 19) and FD auto activate/deactivate from FMGC 1 only, not the priority FMGC | wrong with FMGC 2 in control | FCU wrapper/model inputs | open |
| C7 | PFD FG source selection treats ATS bit 24 (A/THR inop) as "FG inop" | PFD switches FG source on an A/THR fault | `FgBusProvider.ts` 298–305 | open |
| C8 | FMGC 2 discrete word 2 variable reads `A32NX_FMGC_1_DISCRETE_WORD_2` | no FMGC 2 fallback for LNAV | `LnavDriver.ts:80` | open |
| C9 | `pfd_sel_spd_kts.Data` sent to the sim even when NCD | stale selected speed exposed | `FlyByWireInterface.cpp` ~2243 | open |
| C10 | ILS words are sent NO or NCD, never FW | `ils_failure` never set by a lost signal; opposite ILS never selected; LAND-arm receiver agreement always passes | `FlyByWireInterface.cpp` 1277–1285 | open |
| C11 | `a320-simvars.md` bit meanings wrong: dw4 b26 is ILS tune inhibit (doc: RWY hdg memorized), dw5 b28 is preset activate (doc: top of SPD synchro); dw1 b29 `tla_to_ga_set` undocumented (the FAC uses it) | documentation only | `fbw-a32nx/docs/a320-simvars.md` | open |
| C12 | FCU never re-derives the SPD/MACH state; nothing resyncs it on entry to managed speed, and the pb toggles it in managed | FCU stuck in MACH (or SPD) after a selected-speed toggle, below crossover | FCU wrapper + FMS | PR #11015 / issue #11012 |
| C13 | FCU word 1 bit 15 ("SPD/MACH value changed", a monostable on the FCU's selected SPD/MACH memory `pValue_p`) also pulses when that memory changes by itself: in managed speed (window dashed) it tracks the actual Mach rounded to 0.01, so Mach crossing x.xx5 pulses the bit. The FMGC treats bit 15 as a pilot speed change and drops the ALT soft mode | ALT soft mode (FCOM DSC-22_30-70-70: ±50 ft to absorb Mach deviations and "minimize the changes in thrust") drops whenever cruise Mach wanders across a rounding boundary, then needs 120 s of stable flight to re-engage; the A/THR goes back to full gain and the thrust hunts. Seen in 2026-11-11-21-14-07 (FL370, M0.78): engaged at t=3388, dropped at 3401 when Mach reached 0.785 (FCU Mach 0.78→0.79), re-engaged 3531; soft mode active 0.5 of 4.6 min in cruise | `FcuComputer.cpp` 1073–1078 (bit 15 source), `readable/targets.cpp` `updateAltSoft` | fixed on fix/a32nx-fcu-spd-value-change (FCU wrapper `Fcu.cpp` clears bit 15 in managed speed unless the pilot used the SPD/MACH controls within 1 s; merged into test/shadow-fixes and test/combined). Replay with the bit masked (`fmgc-probe-soft-mode`): 36.1 min soft mode, no drop until descent, vs 29.6 min / 3 drops as recorded |

Details: `docs/output-buses.md` (Findings 8–15), `docs/conditioning.md` (5), `docs/modes/targets.md`.

## 2. Bugs in the generated FMGC (modelling errors)

| # | Finding | Effect | Details | Status |
|---|---|---|---|---|
| F1 | ALT* capture: the previous capture's offset leaks into the next entry gain; an ALT* entry with ~0 V/S (on the ground) makes the offset huge | "400 fpm creep" into every later level-off of the flight | `docs/laws/alt-capture.md` Findings 1–2 | fixed (personal), confirmed in flight |
| F2 | ALT CST*: constraint sequencing changes the captured target ≥ 250 ft → reverts to V/S | V/S reversion when the FMS sequences a constrained waypoint during ALT CST* | `docs/modes/alt-vs-fpa-tcas.md` (term 4) | issue #11011, personal patch |
| F3 | A/THR N1 integrator not re-initialised on a mode change; winds down to the 18 % floor in SPEED/idle; THR ramp 1 %/s above 30 % N1 | THR CLB from idle takes ~73 s; OP CLB + EXP at FL250 descended | `docs/athr-n1.md` | declined (EXP side: upstream PR #10957) |
| F4 | A/THR pb disconnect `!LAND` term reads a constant all-false bus | A/THR pb disconnects in every mode, LAND included | `docs/modes/engagement.md` (5) | open |
| F5 | CLB ground pre-arm reads constant `any_lateral_mode_engaged` (false) | term dead | `docs/modes/climb-descent.md` (5) | open |
| F6 | Speed-protection mode reversions disabled (`Constant_Value_m = 0`) | no VLS−2 / VMAX+4 reversion | `docs/modes/alt-vs-fpa-tcas.md` (7) | open |
| F7 | `cruise_active`, `loc_bc_selection` constants | no LOC back course; cruise-dependent terms dead | `docs/modes/lateral.md` (10) | open |
| F8 | V/S and FPA level-off rate limit only counts a positive previous target | level-off from a descent keeps the 0.05 g limit | `docs/laws/alt-capture.md` (Related) | open, probable bug |
| F9 | SPD/MACH latch: no opposite-FMGC sync, reset on restart, no self-resync; the pb toggles it in managed speed | FMA/FCU show MACH with a 290 kt target until an FMS request or pb press | `docs/modes/targets.md` (2, 3) | worked around in FCU/FMS (PR #11015) |
| F10 | Selected Mach converts to V_c = 0 kt when the static pressure word isn't normal | zero speed target with a failed ADR static pressure | `docs/modes/targets.md` (6) | open |
| F11 | RWY heading check not wrapped at 360° | RWY mode rejected on runways near north (course 002°, heading 359°) | `docs/modes/lateral.md` (1) | open |
| F12 | AP command words and N1 command always SSM NO; FM weight/CG NO with no weight; N1 left/right constant 0 NO | ELAC SSM gate never rejects; FAC never falls back to its own weight; FAC engine-out sideslip target always 0 | `docs/output-buses.md` (1–2) | open |
| F13 | `fac_rudder_control_failure` has a constant-true term; `both_fac_rudder_valid` ORs bits across FACs | one FAC with both yaw damper and trim engaged counts as both valid | `docs/conditioning.md` (6) | open |
| F14 | Opposite ADR/IR only votes, never selected; ADR/IR switching discretes unread | with own and 3 failed, an all-failed bus is used while the opposite unit is valid | `docs/conditioning.md` (1–2) | open |
| F15 | RA data used regardless of SSM (TCAS 900 ft, tune inhibit 700 ft, G/S 700 ft, A/THR 100 ft); dual RA failure reads 2500 ft | NCD/FT radio height taken at face value | `docs/conditioning.md` (4) | open |
| F16 | A failed TLA word counts as IDLE for the A/THR active range but the raw angle is used for the FMA | A/THR active/limited regardless of that lever's real position | `docs/athr-modes.md` (1) | open |
| F17 | ILS without DME treated as 2 nm | LOC* cross-beam term under-scaled | `docs/laws/lateral.md` | open |
| F18 | Managed speed can engage (0.72 s) by SPD push/EXP/SRS with no AP/FD | brief selected+managed overlap | `docs/modes/targets.md` (1) | open |
| F19 | One-frame overlap of longitudinal modes on handover (resets read previous-frame modes) | two vertical mode flags true for one frame | `docs/modes/climb-descent.md` (6) | open, harmless? |
| F20 | In DES, the outer loops' FPA target is `fms_inputs.vs_target_ft_min` (the V/S target field) | DES FPA_SPEED submode would fly a V/S number as an angle | `docs/integration.md` | open |
| F21 | `tcas_ra_corrective` is true when there is no RA | harmless while TCAS mode is off; misleading flag | `docs/integration.md` | open |
| F22 | Eight memories survive a restart (rwy heading, VAPP, ALT* and TCAS ALT_ACQ targets, ILS DME, GA/RWY TRK track, ALT hold target); the sim wrapper never calls `initialize()` after construction and `clearMemory()` is empty | they survive any power outage in the sim; a restart in GA TRK flies the track memorised before the stop | `docs/integration.md` (state inventory), `fmgc/Fmgc.cpp` | open |

## 3. Differences from the real aircraft (FCOM), by area

Design-level differences rather than coding errors; some may be deliberate simplifications. Items from memory of the
public FCOM text are marked "to verify" in the stage docs.

- **AP/FD/A/THR engagement** (`docs/modes/engagement.md`): AP disengagement 0.2 s late except instinctive disconnect;
  a second AP outside LAND overlaps 0.2 s; no mode sync in dual AP; engage envelope checked only at the press; A/THR
  engages 0.9 s after SRS at takeoff (not from lever position); landing capability needs the AP in LAND (no CAT 1 on
  FD only); FD auto-disengage tests A/THR engaged, not THR mode.
- **Targets** (`docs/modes/targets.md`): TCAS selects speed (newer FCOM: managed); no SSM check on FCU speed words;
  ground speed mini projects true wind on the magnetic ILS course and applies at any altitude with LAND armed; no VMAX
  cap on the managed target; ALT soft uses ALT + CRUISE phase, dropped by any FCU SPD/MACH value change; managed
  speed cleared on the ground when V2/FMS speed is lost.
- **Autothrust** (`docs/athr-modes.md`, `docs/athr-n1.md`): engine-out active range excludes IDLE; LVR MCT only below
  the MCT detent and armed at THR RED ALT; LVR CLB/TCAS interplay; A/THR limited triggers on either lever; RETARD
  stays if the AP is lost in the flare with an FD on; no approach (CONF ≥ 1, < 3200 ft) gain change; THR modes have no
  thrust target (ramp to the lever limit).
- **Lateral** (`docs/modes/lateral.md`, `docs/laws/lateral.md`): RWY not disengaged by heading; GA TRK only from
  LOC/LAND/none; DIR TO blocked in LOC at any height; LOC push refused by flight phase, and LOC push in LOC* reverts to
  HDG/TRK; bank limit is a TAS schedule (no 15° low-speed/engine-out logic); HDG always takes the shortest turn.
- **Vertical** (`docs/modes/*.md`): CLB can engage from a constraint change in SRS; CLB can engage in GA phase; 15 s
  G/S* → G/S confirmation (to verify); DES → V/S on loss of NAV (matches FCOM).
- **Input conditioning** (`docs/conditioning.md`): validity asymmetry ADR vs IR (aligning IR invalid); FAC chosen by
  health discrete only; on-ground from the FAC only (nose gear discretes unused); 5 s in-flight confirm reset on
  restart; TCAS mode needs RA > 900 ft (aircraft ~1000 ft, to verify); ILS tune inhibit not set by LOC/G/S without LAND
  armed (to verify); back beam inverts LOC deviation without a back-course mode.

## 4. Unused inputs and dead outputs

Inputs never read by `step()`: `mcdu_own/opp_fail`, `nav_control_own/opp`, `adc_3_switch`, `att_3_switch`,
`nose_gear_pressed_own/opp` (`docs/conditioning.md` 14). Outputs packed but read by nobody, and documented bits never
packed: `docs/output-buses.md` 6–7.

## 5. Property violations found by fuzzing (Phase 4)

Checked against the generated stock model by `src/properties.h` (40 properties); reproducers (hex fuzz inputs or
plain replays) in `docs/properties.md`. Overlaps with sections 2–3 are noted.

| # | Finding | Effect | Reproducer | Status |
|---|---|---|---|---|
| P1 | **V/S engages on the ground**: the V/S reversion paths (ALT* target change, DES with FCU above, ...) have no ground/airborne gating | V/S active after landing | plain replay 2026-11-08-01-19-49 t 3559.6 | open |
| P2 | **NaN outputs**: `Phi_loc_c` = … / ground speed is NaN at standstill (fed back to LOC* through the outer-loop delay); `flare_law.H_dot_c_fpm` NaN/inf when FLARE engages at RA 0 (tau = 0) | NaN on the outer-loop bus; FD/AP commands stayed finite | plain replay 2026-11-08-01-12-39 step 0; fuzz | open |
| P3 | NAV engages on the first frame after a short power cut (FD kept engaged, NAV capture true) | mode on the restart frame | fuzz (properties.md) | open |
| P4 | DES engages from G/S* (the DES set condition doesn't exclude G/S*/G/S) | DES taking over a glideslope capture | `--approach 1` 2026-11-09-18-55-35 t 2251.7 | open |
| P5 | CLB engages in the GOAROUND phase (only DES/APPR phases excluded) | CLB with SRS GA | `--approach 2 --athr 2` 2026-11-09-18-55-35 t 1040 | open (cf. climb-descent.md 2) |
| P6 | Managed speed target above VMAX (e.g. 250 kt with flaps VMAX 215) | target overspeed with flaps | plain replay 2026-11-08-01-19-49 t 896.5 | open (= targets.md 8) |
| P7 | SRS takeoff target outside [V2, V2+15]: the approach-speed branch outranks SRS TO; V2 changes move the target | wrong SRS target in edge cases | `--approach 1` replay; fuzz | open |
| P8 | RETARD latched outside LAND; RETARD at 40 ft (doc 30 ft) | RETARD in a non-autoland flare | properties.md | open (= athr-modes.md 7) |
| P9 | ROLL OUT without FLARE; FLARE above 80 ft RA (fuzz only) | mode order not enforced | properties.md | open |
| P10 | A/THR active with no mode and a blank FMA after opposite-FMGC sync ends | blank A/THR FMA | fuzz | open |
| P11 | Mode overlaps: one-frame handovers (expected, F19); V/S+TCAS 1.4 s at RA end; LOC*+LOC+HDG > 1 s; SRS GA with no AP/FD for 0.74 s | brief double FMA modes | properties.md | open |
| P12 | Selected speed target 0 kt (dashed window, Mach with no static pressure) or 416 kt from raw FCU words | out-of-range V_c | plain replay 2026-11-08-01-19-49 t 252.8 | open (= F10) |
| P13 | ALT engages within 40 ft (FBW docs say 20 ft) | doc difference | plain replay | doc |
| P14 | SRS FD pitch 19.7° with degraded air data (the 120 ft/min minimum-climb term beats the 18° limit) | by design of the median | fuzz | open |

Coverage of the generated code after replay + injectors + fuzz: FmgcComputer 98.6 % of branch outcomes, FmgcOuterLoops
91.2 % (rest mostly dead). Still unreached: TCAS ALT_HOLD submode, CLB/DES constraint paths 2776/2820/2938, THR ramp
rate limits 4606/4608, two failure combinations (`docs/coverage.md`).

## 6. Build and wrapper issues found while adding shadow mode (Phase 6a)

| # | Finding | Effect | Where | Status |
|---|---|---|---|---|
| B1 | `rm -f "${DIR}/obj/*.o"`: the quotes stop the glob, so stale objects are never deleted and still linked by `obj/*.o` | a removed/renamed source can stay linked into fbw.wasm | `fbw-a32nx/src/wasm/fbw_a320/build.sh` | open |
| B2 | fbw.wasm runs on wasm-ld's default 64 KiB stack with no guard; an overflow silently overwrites data | latent (measured depth 5.9 KB) | fbw.wasm link | open |
| B3 | `Fmgc` constructors leave `monitoringHealthy`, `powerSupplyOutageTime`, `selfTestTimer`, ... uninitialised; zero only because the global `FlyByWireInterface` is zero-initialised; the copy constructor copies only `isUnit1` (never called) | latent | `fbw_a320/src/fmgc/Fmgc.{h,cpp}` | open |
| B4 | The flight data recorder writes FMGC 1 only (`writeFmgc(fmgc2)` commented out) | FMGC 2 behaviour can't be replayed exactly | `fbw_a320/src/recording/FlightDataRecorder.cpp` | open |
| B5 | First in-sim shadow flight (2026-11-15-18-40-33, LTFM, 98 min, both units, 229,512 running frames each): the readable and generated FMGC differ only by 1–2 ULP in six outer-loop float outputs (FD/AP `Theta_c_deg`, FD `Phi_c_deg`, FD `Beta_c_deg`, flare law `H_dot_radio_fpm`/`H_dot_c_fpm`); no mode, logic, discrete or bus difference. The offline native and wasm checks are bit-exact, so the cause is in how fbw.wasm is built (the sim build uses `-flto -O2` with `--lto-O2`; the shadow objects add `-ffp-contract=off`) | none on the flying; the shadow log is flooded (≈96% of frames in an episode) so a real divergence would be hidden | `fbw_a320/build.sh`, `FmgcShadow.cpp` | open: reproduce offline with the sim's flags, or compare floats with a 4-ULP tolerance |
