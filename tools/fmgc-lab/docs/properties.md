# Behavioural properties of the stock FMGC (Phase 4)

`src/properties.h` checks these properties every step on the **generated stock model's** inputs and outputs (no access
to its internals). They come from the Phase 4 plan, the docs-derived oracle list in PLAN.md (FBW pilot docs, FCOM) and
the mode docs (`docs/modes/*.md`, `docs/athr-modes.md`, `docs/conditioning.md`). A violation is a finding about stock
behaviour, not a test failure; nothing here is fixed.

Each property is "this bad condition must not hold", with a tolerance: violated when the condition has held for longer
than the tolerance (0 = the first frame; "1 frame" = more than one frame; time tolerances get two frames of slack for
the fuzzer's large time steps). Strict and "_1s" forms exist where the generated logic hands over between latches with
a one-frame overlap and a 0.72 s "set recently" guard (`docs/modes/climb-descent.md` S2): the strict form records that
the overlap exists, the tolerant form whether an overlap persists. Edge properties (something engages) are checked on
the engagement frame and skip engagements copied from the opposite FMGC (`fmgc_opp_mode_sync`), the restart frame
after a power cut, and the first frames of a run (their "previous frame" is not the generated model's).

## Where they ran (2026-11-15)

- **corpus**: `scripts/properties-corpus.sh`: 22 recordings x 19 configurations (plain, the input fuzzer and every
  scenario injector alone and all together, 2 seeds), 418 runs, 58.4 M steps (56.5 M with the computer running). The
  same runs compare the readable whole computer: **0 mismatching steps, 0 hidden-memory divergences**. Reproducer =
  recording, flags, step, time: `build/fmgc-drive-stock FLAGS --trace T0 T1 ~/fdr-corpus/raw/FILE`.
- **fuzz**: the campaigns of `scripts/fuzz.sh` (see the end), re-triaged with the final property code (2370 saved
  inputs, 3.38 M steps, also 0 whole-computer mismatches). Minimised reproducers: `scripts/minimize-findings.sh`
  → `build/fuzz/min-final/<id>.bin` (+ `.trace`, the mode timeline). The hex below recreates one without the build
  directory: `echo HEX | xxd -r -p > case.bin; FMGC_FUZZ_BANK=build/fuzz/bank.bin build/fmgc-fuzz-run run --trace
  case.bin` (needs the bank built from the same 22 recordings with the default bank options).

Columns: checked = frames (level properties) or engagements (edge properties) where the property applied, corpus
total; violations = episodes beyond the tolerance, corpus total (runs with one); fuzz = violated in the fuzz
re-triage (inputs).

## Holds everywhere

| Property | Source | Checked (corpus) | Notes |
|---|---|---|---|
| `law.vertical_consistent`: the vertical law is the law of an active vertical mode | structure.md law selection | 56.5 M frames | also in every fuzz input |
| `law.lateral_consistent` | same | 56.5 M | |
| `engaged.ap_implies_any`: own AP ⇒ any_ap_fd_engaged | engagement.md | 56.5 M | |
| `athr.active_implies_engaged`: A/THR active ⇒ A/THR engaged | task list, athr-modes.md | 56.5 M | |
| `athr.modes_imply_engaged`: THR/SPEED/RETARD/alpha floor ⇒ engaged | athr-modes.md | 56.5 M | |
| `athr.single_mode_1s`: two A/THR modes never > 1 s together (own logic) | athr-modes.md | 56.5 M (11 373 overlap frames, none persisting) | |
| `armed_active.exclusive_1s` | task list | 56.5 M (32 962 overlap frames, none persisting) | the first campaign's 1 s violations were all large-dt artefacts |
| `land.flare_implies_land`, `land.rollout_implies_land` (one frame of lag allowed) | takeoff-ga-approach.md, lateral.md | 56.5 M (246 / 135 one-frame frames) | FLARE / ROLL OUT read the previous frame's LAND: always exactly one frame |
| `land.flare_after_land`: FLARE engages only from LAND | task list (LAND/FLARE/ROLL OUT order) | 262 engagements | |
| `land.engage_conditions`: LAND below 400 ft RA from LOC + G/S | PLAN docs oracle | 333 | |
| `vs.engage_reason`: V/S-FPA engages only for a listed reason (knob, AP/FD engagement with no mode, 5 s airborne with no mode, G/S / FINAL / DES loss, ALT* target change ≥ 250 ft, CLB/DES family with the FCU on the wrong side, TCAS end, opposite sync) | PLAN Phase 4 + FBW docs/FCOM | 12 677 engagements | every engagement in the corpus and the fuzz inputs had a listed reason. Note: the "5 s airborne with no mode" path also fires 5 s after every in-flight FMGC restart (`sinceStart`), as if after lift-off |
| `open_modes.airborne_5s`: OP CLB/OP DES/EXPED after 5 s airborne | PLAN docs oracle | 912 | |
| `alt_star.target_change_reverts_vs`: ALT* target moved ≥ 250 ft ⇒ V/S within 2 frames | PLAN Phase 4 | 168 changes | holds for the own logic (not checked while synchronised: the slave copies the master) |
| `alt.capture_40ft`: ALT engages within 40 ft | alt-vs-fpa-tcas.md | 611 | |
| `srs.pitch_limit_18` (corpus) | PLAN docs oracle | 4.67 M frames | violated in the fuzz inputs, see below |
| `cond.double_failure_disengages`: dual ADR/IR failure ⇒ AP and A/THR off within 1 s | conditioning.md 2 + engagement.md | 56.5 M (555 frames in the first second) | |
| `running.frozen_when_off`: outputs unchanged while not running | task list ("no mode engages while the computer isn't running") | 1.94 M frames | also every power event of the fuzzer |

## Violated (stock behaviour; reproducers)

| Property | Source | Corpus: checked / violations (runs) | Fuzz | First / minimal reproducer | What it shows |
|---|---|---|---|---|---|
| `vert.single_active`: at most one vertical mode | PLAN Phase 4 | 56.5 M / 27 120 (331) | yes | plain replay 2026-11-08-01-19-49 t 896.50 (CLB+SRS TO) | the designed one-frame handover overlap (every mode change) |
| `vert.single_active_1s` | PLAN + S2 | 56.5 M / 13 (13) | yes | 2026-11-09-18-55-35 `--situation 1 --tcas 1` t 3548.48: V/S + TCAS for 1.4 s | at the end of a TCAS RA, V/S (reversion) and TCAS stay active together until the TCAS set condition has been false for 0.72 s; with an RA flickering both stay for seconds. Fuzz: V/S + ALT after an opposite-FMGC episode |
| `lat.single_active` | task list | 56.5 M / 43 184 (310) | yes | plain replay 2026-11-11-21-14-07 t 1826.59 (RWY+NAV) | one-frame lateral handover overlap |
| `lat.single_active_1s` | task list | 56.5 M / 606 (179) | yes | 2026-11-08-01-19-49 `--approach 2` t 398.37: LOC*+LOC+HDG | HDG engaged by the LOC-loss reversion stays together with LOC*/LOC re-captured from the same G/S / LOC signals for > 1 s (three lateral modes on the FMA) |
| `armed_active.exclusive` | task list | 56.5 M / 27 250 (339) | yes | plain replay 2026-11-08-01-19-49 t 864.12 (NAV armed + NAV) | the armed latch is reset by the previous frame's active mode: one frame of armed + active at every capture |
| `modes.need_ap_fd`: no mode without AP/FD for more than one frame | climb-descent.md S3 | 56.5 M / 138 (81) | yes | 2026-11-08-01-19-49 `--approach 1` t 3180.03: SRS GA for 0.74 s with no AP/FD | TOGA with both FDs off engages SRS GA (and the go-around speed); the mode reset clears it only once the 0.72 s set guard expires. FCOM: TOGA engages the FDs, so "SRS GA without FD" should not exist |
| `athr.single_mode` | athr-modes.md | 56.5 M / 3593 (255) | yes | plain replay 2026-11-08-01-19-49 t 1610.32 (THR+SPEED) | designed A/THR handover overlap (same 0.72 s guard as the modes) |
| `athr.active_has_mode`: A/THR active ⇒ a mode (after 1 s) | FBW docs/FCOM FMA | 56.5 M / 41 (28) | yes | 2026-11-08-01-19-49 `--approach 1 --athr 1` t 1480.99: FMA NONE with A/THR active | the THR/SPEED latches only set on a rising edge of "engaged ∧ law"; when the opposite-FMGC sync that supplied the mode ends (or the A/THR engages under sync), nothing re-establishes a mode: A/THR active, controlling thrust, with no mode and a blank FMA until the next law change |
| `athr.complementary`: pitch on speed ⇒ THR, pitch on path ⇒ SPEED | PLAN docs oracle | 13.5 M / 63 (29) | yes | same run t 1483.35 | consequence of the above (and of alpha floor / TOGA LK ending without a new edge) |
| `athr.retard_in_land`: RETARD only with LAND/FLARE | PLAN docs (RETARD in FLARE) | 56.5 M / 461 (60) | yes | 2026-11-13-19-30-04 `--approach 2` t 1418.20: RETARD in RWY / V/S | RETARD is a latch reset only by another thrust mode or A/THR disengagement (athr-modes.md finding 7): after LAND ends (go-around without TOGA, AP/FD off) it stays, i.e. idle thrust in V/S |
| `athr.retard_height_30ft` | FBW docs (RETARD at 30 ft) | 471 / 402 (62) | yes | plain replay 2026-11-13-22-45-26 t 4418.91 (38.9 ft) | doc difference: RETARD engages at ≤ 40 ft RA (FCOM "about 40 ft") |
| `land.rollout_after_flare`: ROLL OUT only after FLARE | task list | 29 / 1 (1) | yes | 2026-11-12-17-08-01 `--approach 1 --lateral 1` t 2341.15, RA 50 | ROLL OUT needs only LAND + RA ≤ 50 ft + wheel spin-up (or on ground) for 0.36 s: a touchdown without FLARE (FLARE inhibited, RA jump) goes straight to ROLL OUT |
| `land.flare_height`: FLARE engages below 80 ft RA | PLAN docs (FLARE ~40 ft) | 262 / 0 | yes | fuzz (minimal reproducer below): FLARE at RA 100 ft | the FLARE condition comes from the previous frame's outer loops (Delay2): with a radio height that jumps (injectors) FLARE engages on a frame where the RA is already above 80 ft; the minimal fuzz reproducer has this FMGC as unit 2 following the opposite FMGC. Holds in the corpus (continuous RA) |
| `vs.airborne_5s`: V/S engages only after 5 s airborne | PLAN docs oracle | 12 677 / 1960 (246) | yes | plain replay 2026-11-08-01-19-49 t 3559.56 (on ground) | **V/S on the ground**: the reversion path (ALT* target change, DES with FCU above, ...) has no ground or airborne gating (alt-vs-fpa-tcas.md); the first reproducer is a real flight after landing |
| `alt.capture_20ft` | FBW docs (ALT within 20 ft) | 611 / 281 (154) | yes | plain replay 2026-11-08-01-19-49 t 1623.76 (35 ft) | doc difference: ALT engages within 40 ft for 0.8 s (as the old Stateflow chart) |
| `clb.engage_conditions`: CLB needs NAV, FCU above, not DES/APPR/GA | PLAN docs oracle | 184 / 4 (3) | yes | 2026-11-09-18-55-35 `--approach 2 --athr 2` t 1040.00: CLB in phase GOAROUND | the code excludes only the DES and APPR phases: a constraint change with CLB armed engages CLB in the go-around phase (fuzz: CLB + SRS TO + ALT* together) |
| `des.engage_conditions`: DES needs FCU below, NAV/LOC, valid path, not from G/S / LAND / FINAL | PLAN docs oracle | 278 / 40 (30) | yes | 2026-11-09-18-55-35 `--approach 1` t 2251.70: DES from G/S* with LOC | DES engages from G/S* (DES armed at a constraint, or an ALT push; minimal fuzz reproducer below: DES from G/S* with LOC*+LOC at 208 ft, phase PREFLIGHT); the DES set condition does not exclude G/S*/G/S, only their reset lists do |
| `spd_target.range`: 90 ≤ V_c ≤ 400 kt with AP/FD | FCU range, targets.md 5/6 | 54.8 M / 3392 (315) | yes | plain replay 2026-11-08-01-19-49 t 252.79: V_c 0 | the selected target uses the raw FCU words (dashed window, Mach with no static pressure → 0 kt; fuzz: 416 kt) |
| `spd_target.le_vmax`: managed target ≤ VMAX | FCOM note 4 (targets.md 8) | 30.3 M / 6711 (270) | yes | plain replay 2026-11-08-01-19-49 t 896.52: 250 kt with VMAX 215 (flaps) | no VMAX cap on the managed target |
| `spd_target.srs_to`: SRS TO target in [V2, V2+15] | PLAN docs oracle | 2.23 M / 96 (66) | yes | 2026-11-09-18-55-35 `--approach 1` t 968.48: 133 kt, V2 141 | the approach-speed branch (LAND armed or APPROACH phase) has priority over SRS takeoff; and V2 changed during SRS moves the target (fuzz) |
| `srs.pitch_limit_18`: SRS FD pitch ≤ 18° | PLAN docs oracle | 4.67 M / 0 | yes | fuzz (failure scenario, below): 19.7° | the SRS law takes the median of the speed command, the 18° limit and the 120 ft/min minimum climb: with degraded air data the 120 ft/min term wins over 18° (by design, laws_speed.cpp) |
| `running.clean_restart`: no mode on the first frame after a power cut | task list | 499 / 0 | yes | fuzz (below): NAV engaged on the restart frame | with an FD still engaged across a short power cut and the NAV capture condition true, NAV engages on the first running frame (own logic, not sync) |
| `outputs.finite` | robustness | 56.5 M / 289 (176) | yes | plain replay 2026-11-08-01-12-39 step 0: Phi_loc_c = NaN | `Phi_loc_c = sin(loc error)·DME·k / V_gnd`: NaN (0/0) whenever the ground speed is 0 (standstill), fed back to the LOC* capture through Delay2_DSTATE (comparisons with NaN are false: no LOC* at standstill). Fuzz: `flare_law.H_dot_c_fpm` NaN/inf when FLARE engages at RA 0 (`tau = H / (sink - touchdown V/S)` = 0); the FD/AP commands stayed finite |

## Minimal fuzz reproducers

`scripts/minimize-findings.sh` over the 4390 reproducer files of the three campaigns, with the final build: one minimal
input per violated property (`build/fuzz/min-final/<id>.bin`, with `.trace`). Snapshot = bank index (recording, time).

| Property | Bytes (hex) | Snapshot | First hit |
|---|---|---|---|
| alt.capture_20ft | `68015b1002fd732e` | 360 (2026-11-12-21-10-58 t=1590.5) | step 159: \|H − target\| 34 ft |
| armed_active.exclusive | `b200000500ffa14b230115b7b7b7b72e` | 178 (2026-11-10-21-18-10 t=1940.5) | step 71: ALT armed + ALT* |
| athr.active_has_mode | `7900be8e03ffffff23bc1401d3779477` | 121 (2026-11-10-13-37-00 t=2537.1) | step 1166: FMA NONE, A/THR active |
| athr.complementary | `ba00ffcc03ffffbdffff14ff01000000` | 186 (2026-11-10-21-18-10 t=2900.6) | step 1659: V/S law with THR |
| athr.retard_height_30ft | `6d0032b303ff0079c8bc140000000000` | 109 (2026-11-10-13-37-00 t=1097.0) | step 1456: RA 39.2 |
| athr.retard_in_land | `7900bea503ffffff23ff1401d3779477` | 121 | step 1346: RETARD in NAV / DES |
| athr.single_mode | `9e01ff0000735f68` | 414 (2026-11-13-19-30-04 t=3263.8) | step 0: THR + SPEED |
| clb.engage_conditions | `6701d13100d1d12200a5142e2e62696ed10007000000d101` | 359 (2026-11-12-21-10-58 t=1470.4) | step 418: CLB in phase GOAROUND (with SRS TO + ALT*) |
| des.engage_conditions | `8501ff7003fbc0ffff9a1400f2c167d2cf0000d64bce77e3` | 389 (2026-11-13-19-30-04 t=263.3) | step 924: DES from G/S* |
| land.flare_height | `9f01d126010012d3309215730072645f740014d100006b67` | 415 (2026-11-13-19-30-04 t=3383.8) | step 328: FLARE at RA 100 |
| land.rollout_after_flare | `9f01217201d1807661bc1abc1e612e402fa514d1d0d02fd1d08e1430d7d92d2c` | 415 | step 940: ROLL OUT without FLARE |
| lat.single_active | `b200002703ffa14b009a1432af642a4b239815b7b7b7b72e627f1569de0aa92d` | 178 | step 338: NAV + GA TRK |
| lat.single_active_1s | `6201203e0320683dc581140087ff9ecd` | 354 (2026-11-12-21-10-58 t=870.4) | step 520: LOC + HDG > 1 s |
| modes.need_ap_fd | `b200002703ffa14b009a1432af642a4b239815b7b7b7b72e627f1569de0aa92d` | 178 | step 336: SRS GA with no AP/FD |
| outputs.finite | `b200000603ffa14b00811432af642a4b00811432af642aff` | 178 | step 78: Phi_loc_c NaN |
| running.clean_restart | `7d01d17d03bd63d1ff00190000000000a38f14ec2d000000600010002e2e0000` | 381 (2026-11-12-21-10-58 t=4110.6) | step 1029: NAV on the restart frame |
| spd_target.le_vmax | `aa01000f032f4629` | 426 (2026-11-13-22-45-26 t=1219.4) | step 150: 250 kt, VMAX 215 |
| spd_target.range | `a601011b01746872` | 422 (2026-11-13-22-45-26 t=739.3) | step 244: V_c 0 |
| spd_target.srs_to | `0a002e00006e8dd800001428588ed800` | 10 (2026-11-08-01-19-49 t=860.2) | step 0: 161.9 kt, V2 146 |
| srs.pitch_limit_18 | `0a00001602a58dd800891405583e872d` | 10 | step 206: FD θ 19.7° |
| vert.single_active | `b200000500ffa14b230115b7b7b7b72e` | 178 | step 71: seven vertical modes at once (opposite-FMGC bits) |
| vert.single_active_1s | `3400ef7e00ffffffff00190000000000ef0c15ffffffffff` | 52 (2026-11-09-18-55-35 t=2319.8) | step 1035: V/S + ALT > 1 s |
| vs.airborne_5s | `9f0127220189ff349500151500afa963` | 415 | step 300: V/S on the ground |

## Not yet checkable

| Property | Source | Why |
|---|---|---|
| SRS 22.5° pitch limit in windshear | PLAN docs oracle | no windshear input reaches the FMGC; the outer loop has no 22.5° limit (structure.md) |
| EXP DES 340 kt / M0.80 at idle; EXP CLB green dot | PLAN docs oracle | the targets come from the FMS (`v_managed_kts`); the FMGC only passes them on |
| "V/S on loss of G/S, FINAL, LOC, NAV in DES, vertical path, TCAS end" (the converse of `vs.engage_reason`: the loss must give V/S) | FBW docs | needs a definition of "loss" independent of the code (which mode replaces it is a choice of the reset lists); the ALT* case is checked |
| ALT* arming conditions, FINAL APP engagement (stock never writes A32NX_FG_FINAL_CAN_ENGAGE) | PLAN | FINAL needs FMS inputs no recording has; the fuzzer sets `final_app_can_engage` but there is no oracle beyond the code |
| A/THR SPEED whenever pitch on a VPATH / idle path | PLAN docs oracle | depends on the FMS DES submode; `athr.complementary` excludes VPATH |

## Campaigns

- **first** (2026-11-14, before the whole-computer oracle): 4 workers, fork mode, 3600 s, 1.50 M inputs (~420/s,
  ~1000 steps each), no differential mismatch (output packing, A/THR resync). 30 property ids saved reproducers;
  after re-triage with the final property code, 7 of them (`vs.engage_reason`, `open_modes.airborne_5s`,
  `alt_star.target_change_reverts_vs`, `athr.single_mode_1s`, `armed_active.exclusive_1s`, `land.flare_implies_land`,
  `land.rollout_implies_land`) were artefacts of the property formulation (airborne time and the ALT* sample after an
  in-flight restart, the V/S loss terms on the first frames of a run, one-frame lags and 0.72 s guards at the fuzzer's
  large time steps) and are fixed in `properties.h`; none of them reproduces with the final code.
- **whole** (2026-11-14/15, whole-computer oracle): 3 workers; the host suspended the WSL VM after 22 min (uptime
  advanced 2 h while the wall clock advanced 11.5 h), so it ran ~1320 s: 220 k inputs (~170/s for 3 workers), **0
  whole-computer mismatches**. libFuzzer then saw the suspension as a 9.8 h run and wrote one `timeout-` and two
  `slow-unit-` artifacts, which run in milliseconds (kept in `build/fuzz/artifacts-whole-suspended/`).
- **whole2** (2026-11-15, whole-computer oracle): 4 workers, fork mode, 2418 s wall, **541 801 inputs (~224/s,
  ~56/s per worker; both computers, the stage differentials and the properties every step: ~220 k steps/s of each
  computer), 0 whole-computer mismatches, 0 crashes/timeouts**; corpus 7291 → 8060. Property reproducers for 24
  ids, no new kind of violation; one `vs.engage_reason` hit was a property bug (the property read the opposite FMGC's
  discrete word without the generated rounding of noisy word data) and is fixed.

Whole-computer differential in total: corpus runs 58.4 M steps + fuzz ~760 k inputs (~0.75 G steps of the campaigns
"whole" and "whole2", plus 3.38 M steps re-running the 2370 saved inputs): **no mismatch of any of the 1017 output
leaves or of the hidden memories**, so no minimised differential reproducer exists.
