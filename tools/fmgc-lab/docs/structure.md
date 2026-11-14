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

Corrections from the Phase 3 G/S-VPATH-FLARE rewrite: VPATH is only 4070–4094; the FLARE V/S law (exponential flare,
Tau/H_bias, RA-rate lead-lag) is at 3108–3186; flare outputs and the 0.3 g / 0.6 g AP limits at 4310–4319; the flare
condition (4560–4565) uses a second 4 rad/s lag of the RA rate; the 100-sample delays Delay_DSTATE_l4/_n belong to the
lateral laws; the G/S law has no speed protection. The flare law reads the lateral AP sideslip command
(output.autopilot.Beta_c_deg).
Corrections from the V/S-FPA rewrite: FPA is 3323–3473 (3110–3320 is the flare V/S law and G/S), V/S 3792–3955,
SPD_MACH 3560–3790. AP gains: 0.35 × path command, 0.36 × speed laws. In every speed-on-pitch law the switch on the
constant (1 − 0 > 0) always selects the ±4° limit on speed error / 3; the −0.93 / 0.6·(0.7·accel + γ) limits are dead.
The "target set to 0 from > 500 fpm" level-off term applies whatever the active law is (docs/laws/alt-capture.md).
Corrections from the speed/SRS rewrite: SPD_MACH law 3557–3655; SRS 3956–4069 (VPATH proper 4070–4095); 4170–4480
holds the flare law and the AP side of SPD/SRS (4456–4476). SPD_MACH and SRS share one speed-on-pitch subsystem
(acceleration in g from a complementary filter of ground-speed and IAS rates, + γ, speed error at 3 kt/deg limited by
energy rate, ≥ 120 ft/min towards the altitude target). SRS: 18° pitch limit and 120 ft/min minimum confirmed; no
22.5° windshear limit in the outer loop; the speed target comes in as V_c.
Corrections from the ALT*/ALT rewrite: the ALT_HOLD AP command is computed at 4304 (vsLimit(0.7 × FD)); the ALT*
speed laws start at 2951 (2815–2950 fills a bus structure); the ALT hold target is frozen while ALT_HOLD is active and
set to 0 only at power-up (init), not on reset.
Corrections from the lateral rewrite (`docs/laws/lateral.md`): the lateral block is 2232–2750: bank limit 2232–2271
(computed first), NAV feedforward 2276–2289, HDG 2290–2330, LOC_CPT and Phi_loc_c 2331–2370, LOC_TRACK filters and
runway alignment 2371–2412, TRACK 2412–2442, roll law switch 2443–2531 (HPATH and LOC_TRACK are computed inside their
cases), FD roll 2532–2540, ROLL_OUT 2541–2626, sideslip and nosewheel switches 2627–2716, AP roll 2718–2750. The
100-sample delays Delay_DSTATE_l4/_n (shifted at 4566–4572) detect the first second of HDG / TRACK (the turn is then
always the shorter way); Delay_DSTATE (no suffix) is LOC_TRACK's deviation derivative, Delay_DSTATE_p the ROLL_OUT
deviation integral, Delay_DSTATE_f/icLoad the AP roll integrator. FD and AP share one roll command per law; FD β is the
lag of AP β. The RWY LOC submode selects the ROLL_OUT law.
Corrections from the lateral mode rewrite (`docs/modes/lateral.md`): 1722–1793 RWY (c), RWY TRK (b) and `rwy_active`
(c || b, 1793); 1894–1912 ROLL OUT (m) and ALIGN (combinational, LAND below 30 ft RA with FAC rudder control);
2029–2052 GA TRK (el); 2054–2101 the lateral mode reversion (output 4035, `rtb_y_o` from 2141), feeding the HDG/TRK
latch (dy, 2129; hdg/trk split at 2139–2140 by FCU word 1 bit 24); 2143–2183 NAV armed (j, reads this frame's FINAL
armed dv); 2184–2214 NAV (ly); 2215–2241 LOC armed (a); 2257–2374 LOC* (cv: capture geometry from the memorised
runway course `rwy_hdg_memo` = `FmgcComputer_B.u_lyjj`, IR roll and the outer loops' previous `Phi_loc_c`); 2375–2404
LOC (lq). The roll FD bars flashing monostable is at 3392–3404, the HDG/TRK preset (bw) at 3405–3434,
`any_lateral_mode_engaged` at 3736. `lateral_mode_reset` and `longitudinal_mode_reset` are the same signal (1553).
`loc_bc_selection` is a constant (false). FCU word 2 bits: 11 HDG/TRK knob pushed, 12 pulled, 13 LOC pushed.
Corrections from the engagement rewrite (`docs/modes/engagement.md`): the AP/FD/A/THR engagement block is 1380–1559,
not 1490–1700 (1560–1700 is the TCAS RA decoding, the TRK-FPA deselection latch e, the altitude target and SRS takeoff
k, which belong to the vertical stage). Order: AP/FD/A/THR common condition (confirm kv, 1382), ROLL OUT heading
monostable (1389), AP/FD common condition (confirm bv, 1395), FD engaged (1420), AP engage request latch
`Memory_PreviousInput` (1468), AP engaged `Delay_DSTATE_p` (1482), A/THR inhibit latch g1 (instinctive disconnect held
15 s, 1490), A/THR engaged `Delay_DSTATE_k` (1528), opposite priority and mode sync (1534–1547), `any_ap_fd_engaged`
and the mode reset (1548–1559). The AP and A/THR latches read their Q from `Memory_PreviousInput_g` / `_p`, copies made
at 5108–5109. After the vertical modes, 3335–3403 compute the FD auto disengage, V/S target not held, the AP/FD mode
reversion pulse and the pitch/roll FD bars flashing. The landing capability (LAND 2/3 capability, inop, capacity;
memories `pLand3FailOp/Pass`) is 3796–3863, outputs 4049–4054 (rewritten with the engagement, `engagement_land.cpp`). `rtb_fmgcOppPriority_tmp` is `!ap_own_engaged` up to 3372 (read at 1896, 2666–2670,
3347–3348), then V/S || FPA.
