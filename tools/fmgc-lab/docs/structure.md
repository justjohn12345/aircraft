# Structure of the generated FmgcComputer::step()

Line numbers: `build/models/stock/FmgcComputer.cpp` (= upstream master). Hypothesis: PLAN.md Phase 1b Step 2. Status: the
outline below is read from the code; per-mode detail is in `docs/modes/`, law detail in `docs/laws/`.

| Lines | Subsystem | Notes |
|---|---|---|
| 201–450 | locals | ~250 temporaries, many reused for unrelated signals |
| 451 | `if (computer_running)` | everything below runs only when powered and healthy; else outputs are left as they were and `Runtime_MODE = false` |
| 452–692 | (re)initialisation | on the first running frame: every latch to its initial condition, `Delay_DSTATE` (previous ap_fd_logic), `Delay1_DSTATE` (previous athr), `Delay2_DSTATE` (previous outer-loop output), block states reset, `LawMDLOBJ1.reset()` (673) |
| ~700–1630 | input conditioning (`logic`) | own/opp/3rd ADR, IR, RA, ILS, FAC, FCU, TCAS source selection by SSM, voting, failure flags, on-ground, engines; assembled into `rtb_BusAssignment_*.logic.*` (final assignment block 3865–3971) |
| 1490–1700 | AP/FD engagement and opp-side sync | latches g1 (1490), e (1618), k (1696) — see the mode docs |
| 1754–2420 | lateral and approach logic | RWY/RWY TRK (c, b), LAND armed (l), ROLL OUT (m), FINAL armed/active (dv, f), GA roll (el), NAV armed/active (j, ly), LOC armed/CPT/TRK (a, cv, lq) |
| 2465–3330 | vertical mode logic | ALT* (ne), ALT (cb), CLB armed/active (ma, ec), DES (b3), OP CLB/DES (ae, ev), EXP CLB/DES (mx, o), G/S armed/CPT/TRK (fm, nu, as), TCAS (i5), V/S, FPA, SRS |
| 3434–3734 | targets | HDG/TRK preset (bw), manual/managed speed (cu, hk), PFD speed target, ALT soft mode (ak) |
| 3736–3800 | law selection | `active_lateral_law`, `active_longitudinal_law`: priority chains, below |
| 3983–4030 | ap_fd_logic output bus | mode flags copied from the latches (see `docs/latches.tsv`) |
| 4138–4170 | `LawMDLOBJ1.step(...)` | FmgcOuterLoops: all control laws, with H_c = `alt_sel_or_cstr`, V_c = `spd_target_kts` |
| 4180–4330 | autothrust | A/THR engage, THR / SPEED / RETARD mode latches (hu, bh, cm, ol), alpha floor, N1 command |
| 4640–4925 | output buses | FMGC A/B bus words; discrete words packed with `MATLABFunction_gy` (19 bools → bits 10–28) |
| 4926–4960 | outputs | `FmgcComputer_Y.out = ...` |
| 5100–5120 | delays | `Delay_DSTATE = ap_fd_logic` (so next frame's conditions see this frame's modes), `Delay2_DSTATE`, `Delay1_DSTATE` |

## Law selection (3740–3792)

Vertical law, first match wins:

| Law | When |
|---|---|
| FLARE | FLARE mode (rtb_AND2_gh) |
| GS | G/S* or G/S or latch d (LAND/align, see approach doc) |
| SRS | pitch takeoff (k) or pitch go-around (i) |
| FPA | FPA mode |
| VS | V/S mode, or TCAS in its V/S submode |
| SPD_MACH | CLB, OP DES, OP CLB, EXP CLB, EXP DES |
| ALT_ACQ | ALT*, or TCAS in its ALT_ACQ submode |
| ALT_HOLD | ALT, or TCAS in its ALT_HOLD submode |
| VPATH | FINAL |
| (DES) | DES by FMS-requested submode: SPEED_THRUST → SPD_MACH; VPATH_THRUST or VPATH_SPEED → VPATH; FPA_SPEED → FPA; VS_SPEED → VS; NONE → NONE |
| NONE | otherwise |

Lateral law: ROLL_OUT (rollout m or RWY c) > LOC_TRACK (lq or d) > LOC_CPT (cv) > HPATH (NAV ly) > TRACK (TRK, RWY TRK b, GA roll el) > HDG > NONE.

The law is selected whether or not an AP or FD is engaged; the outer loops run every frame and their states (filters,
the ALT* capture law) evolve regardless.

## FmgcOuterLoops::step() (FmgcOuterLoops.cpp 2151–4584)

The interface (`ap_laws_output`: `data`, `input.vertical_law`, `H_c_ft`, `H_dot_c_fpm`, `FPA_c_deg`, `V_c_kn`, ...) is
the old `AutopilotLaws` model's (removed in afff8d290), so that file is its direct ancestor.

| Lines | Block |
|---|---|
| 2280–2750 | lateral: NAV gains (tau, zeta), HDG/TRK (turn direction chart, bank limit), LOC, roll out, AP roll fade-in |
| 2755–3067 | ALT_ACQ: capture law (`docs/laws/alt-capture.md`) + VLS/VMAX speed protection |
| 3067–3110 | ALT_HOLD: lag-filtered altitude error, ALT soft mode (speed term) |
| ~3110–3470 | FPA (level-off flags 3446–3467) + speed protection |
| 3470–3560 | G/S |
| ~3560–3951 | V/S (level-off flags 3809–3832, TCAS) + speed protection |
| ~3951–4095 | VPATH (FMS profile: lag of H_c_profile − H, plus H_dot profile) |
| 4095–4170 | FD output: switch on law |
| 4170–4480 | SPD/MACH (speed on pitch), FLARE law, VS limiters |
| 4482–4559 | AP output: switch on law → θ_c = θ + Δθ (±20°), rate-limited integrator (0.6 g), lag 2 rad/s, AP fade-in |
| 4560–4580 | flare condition (RA < 80 ft and RA·14 ≤ |predicted V/S| or RA ≤ 42 ft), delays |

Every path law has the same shape: path error → Δγ = asin(ΔV/S / V_tas); two energy-style speed-on-pitch commands (to
VLS and to VMAX); `SpeedProtectionSignalSelection` with a 5 kt margin; AP command a fixed fraction of the FD command
(0.35 / 0.36 in ALT_ACQ; other laws not yet checked). That repetition explains the block inventory (32 low-pass + 32
high-pass filters: one pair per speed law instance). Line ranges marked ~ are approximate boundaries.
