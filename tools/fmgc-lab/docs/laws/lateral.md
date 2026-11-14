# Lateral control laws (FmgcOuterLoops, lateral_law 0–6)

Source: `build/models/stock/FmgcOuterLoops.cpp` (= upstream master), lines refer to it. Readable rewrite:
`readable/laws_lateral.{h,cpp}`, bit-exact; checked by `build/fmgc-check-laws` (probes `src/probes/lateral.txt`).
Signs are the sim's: roll and heading errors positive to the right, `nav_loc_error_deg` and `beta_deg` as the
FmgcComputer passes them.

## Structure

Every frame **every** lateral law runs, whatever the active law (`input.lateral_law`, chosen by the FmgcComputer:
ROLL_OUT for ROLL OUT and RWY in its LOC submode, LOC_TRACK, LOC_CPT, HPATH for NAV, TRACK for TRK, RWY TRK and
GA TRK, HDG, else NONE), so all filters, memories and turn-direction charts keep moving. Each law produces

- a roll command φ_c (deg), the same for FD and AP (unlike the vertical laws there is no separate AP gain);
- an AP sideslip/yaw command β_c (deg): 0 except LOC_TRACK (runway alignment below 30 ft) and ROLL_OUT;
- a nosewheel command: 0 except ROLL_OUT (the same value as its β_c).

The output stage (2232–2275, 2532–2540, 2627–2750) picks the active law's commands. Law values outside 0–5 take the
ROLL_OUT commands (the switches' default case).

| law | φ_c | β_c |
|---|---|---|
| NONE (0) | current roll (zero FD roll error) | 0 |
| HDG (1), TRACK (2) | heading/track select law | 0 |
| HPATH (3) | NAV law (FMS roll + track/cross-track errors) | 0 |
| LOC_CPT (4) | localizer capture law (also output `Phi_loc_c`) | 0 |
| LOC_TRACK (5) | localizer track law, faded in from LOC_CPT | runway alignment |
| ROLL_OUT (6) | 0 (wings level) | roll-out rudder law |

Inputs worth knowing (FmgcComputer.cpp 4138, the outer-loop call):

- "localizer course" `nav_loc_deg` is `logic.rwy_hdg_memo`: the ILS runway heading (the interface sends the FMS LS
  course minus the station's magnetic variation, i.e. true), memorised by the FmgcComputer while not in RWY LOC, LAND,
  or LAND armed at low height; `nav_loc_magvar_deg` is the constant 0;
- `nav_dme_nmi` is the DME distance when valid, else 0 (then clamped to 2 nm by the laws);
- `zeta_deg` (rudder position) is the constant 0, so ROLL_OUT's "rudder at engagement" memory is always 0 (dead).

## Output stage

**Bank limit** (2232–2271), common to all laws, rate limited at 5°/s (starts at 25°):

- from |TAS|: 15° at 0 kt, 30° from 150 to 300 kt, 19° from 450 kt (linear in between);
- capped at 25° except in LOC_CPT, LOC_TRACK and ROLL_OUT, which keep up to 30° — but get **10° below 700 ft RA**;
- in HPATH, the FMS bank limit (`fms_phi_limit_deg`) replaces it when non-zero.

With the 25° cap the selected-mode limit is 15° at 0 kt rising to 25° at 100 kt TAS, 25° up to 368 kt TAS, then down to
19° at 450 kt. φ_c is clipped to ±limit (a NaN command gives −limit).

**FD roll** (2532–2540): `FD φ = lag(φ_c,limited − φ)`, lag 2 rad/s. The FD bar shows the lagged roll error.

**AP roll** (2718–2750): an integrator starting at the current roll (reloaded while the AP is off) moves towards
φ_c,limited at the roll-rate limit, 5°/s (7.5°/s in LOC_CPT, LOC_TRACK and ROLL_OUT; the limit itself changes at
2.5°/s²); then a 1 rad/s lag; then faded in over 1 s when the AP engages (out in 0.1 s):
`AP φ = (1 − e)·φ + e·lag(integrator)`.

**Sideslip and nosewheel** (2627–2716): `AP β = β_c` of the active law; `FD β = lag(AP β)` at 2 rad/s (the FD yaw bar,
in every mode, though only LOC_TRACK below 30 ft and ROLL_OUT/RWY give non-zero values); `Nosewheel = clamp(−0.04 ×
nosewheel command, ±1)` when the AP is engaged, else 0.

## HDG and TRACK (2290–2330, 2412–2442)

Two identical instances: heading (Ψ_c vs magnetic heading) and track (χ_c vs magnetic track).

- Error: the shorter way to the target, through a **turn-direction chart**: when the shorter turn is between 10° and
  20° the chart latches that direction and keeps it (even if the target moves past the reciprocal) until either way is
  below 10°. During the **first second of the mode** (a 100-sample delay line of "law active", 1 s / dt samples) it
  always takes the shorter way. Dead code: the same chart with `use_short_path = 1` (LOC_TRACK, ROLL_OUT) is a plain
  shortest-angle.
- Law: `φ_c = 1.16 · K(TAS) · error − 2 · r`, K = 1.6 (≤150 kt TAS), 2 (200), 2.8 (250), 3.2 (300), 4.2 (400), 4.5
  (≥500 kt) deg of roll per deg of error, r = yaw rate (deg/s): proportional heading control with yaw-rate damping, no
  integrator (≈1.9°/° at 150 kt, 3.2°/° at 250 kt).
- **Heading bank limit** (`blocks::HeadingBankLimit`): when the target changes (or the error grows past the value
  memorised) the roll command is limited to f(|error|) = 5° up to 5° of heading change, 10° at 10°, 30° from 20°
  (linear), until the command is back within that limit with a steady target; then 30°. Small heading changes are
  flown at small bank.

## HPATH = NAV (2276–2289, case 3 at 2466–2477)

`φ_c = φ_FMS + k2·V_gnd · (TAE + clamp(k1 · XTK / V_gnd, ±45°))`

- φ_FMS: FMS roll order (feedforward of the planned turn), rate limited 10°/s, lagged 4 rad/s.
- TAE (`fms_tae_deg`) and XTK (`fms_xtk_nmi`): the cross-track error turned into a track-angle correction, limited to a
  45° intercept.
- Gains (`blocks::navGains(τ = 3, ζ = 0.8)`): k2 = ζ / (π·g·τ) and k1 = 180 / (4π²·ζ·τ) (τ in hours, g in kt/h,
  215666.57 = π·9.81·3600²/1852). With φ ≈ V·ω/g this is a second-order path capture: commanded turn rate
  ω = (ζ/πτ)·(track error + cross-track/(4πζτ·V)), i.e. damping ζ = 0.8 and natural frequency 1/(2πτ) = 0.053 rad/s
  (118 s period) whatever the speed. In aircraft terms: 0.62°, 1.1°, 1.8° of roll per degree of TAE at 140, 250, 400 kt
  ground speed; 1 nm of XTK is worth 49°, 27°, 17° of track (hence the 45° cap).

## LOC_CPT (2331–2370; output Phi_loc_c)

The NAV law on the beam with τ = 2 s (period 79 s, 0.93° of roll per degree at 140 kt):

`φ_c = k2·V_gnd · (shortest(course + δ − χ) + clamp(k1 · sin(δ)·DME / V_gnd, ±L))`

- course = `nav_loc_deg − nav_loc_magvar_deg` (true), δ = `nav_loc_error_deg`, χ = true track; the track error is
  taken against **course + δ** (the beam bearing, i.e. relative to the line through the antenna) rather than the course.
- Cross-beam distance: sin(δ) · DME with DME clamped to 2–100 nm (`nav_dme_nmi`; at 2 nm minimum when no DME).
- L: the intercept angle |course − track|, limited to 15–115°, tracked continuously while not in LOC_CPT and **frozen at
  LOC* engagement** (lowered to 15° if the intercept angle falls below 15°): the deviation term never asks for more
  convergence than the intercept angle at capture.

## LOC_TRACK (2371–2412, case 5 at 2482–2526)

`φ_c = blend · clamp(A + B + C + D, ±15°) + (1 − blend) · φ_LOC_CPT`, blend 0 → 1 over 2 s after LOC engages.

- A, deviation: δ lagged (1 rad/s) plus 10 s of its rate, lagged (1 rad/s), × 1.1 · K(TAS) (the HDG table) · G(RA):
  G = 3.7 (≤50 ft), 4.4 (100 ft), 7.8 (1000 ft), 15 (≥2500 ft). E.g. 1.6 · 1.1 · 7.8 = 13.7° of roll per degree of
  deviation at 140 kt and 1000 ft, 6.5°/° at 50 ft.
- B, course error: 0.2 · V_gnd · sin(course − track) = 0.2° per kt of cross-course speed. Track here is magnetic track +
  (true − magnetic heading), not `Chi_true_deg` as in LOC_CPT.
- C, sideslip: 0.8 · β above 35 ft RA, 0 below 30 ft.
- D, alignment: 0.4 · β_align (from 0 ft RA; the gain ramps to 0 at −10 ft and is extrapolated below).

**Runway alignment** (decrab, 2392–2403): below 30 ft RA, `β_align = clamp(lag(1.5 · (β − shortest(course − Ψ_true)),
0.7 rad/s), ±15°)`, else the lag decays to 0. It is LOC_TRACK's AP β_c: the AP yaws the nose onto the runway course
while the roll term D holds the track. Matches the FCOM's "at 30 ft RA the AP/FD aligns the yaw axis with the runway
centerline" (ALIGN sub-mode).

## ROLL_OUT (2541–2626; also RWY LOC submode on the take-off roll)

Roll command 0. Rudder/nosewheel law:

- Deviation: `y = clamp(sin(δ) · DME · 1200, ±70)` (DME clamped to 2–100 nm), integral `∫ 0.1 · y dt` limited to ±3,
  reset while not in ROLL_OUT.
- Heading target: `Ψ + [y · G(V_gnd) + ∫ + 1.8 · shortest(course₀ + δ − Ψ)]`, G = 0.8 at 0 kt, 0.2 from 150 kt;
  course₀ = the localizer course memorised at engagement. Heading error e = shortest(target − Ψ).
- Yaw order: `u = clamp((0.3 · r · w + (1 − w) · ζ₀) − 1.2 · e, ±20)`, ζ₀ = rudder position memorised at engagement,
  w = 0 → 1 over 8 s: the rudder held at touchdown is meant to be faded into yaw-rate damping, but the FmgcComputer
  passes ζ = 0, so in practice the yaw-rate damping alone fades in over 8 s and the output over 3 s.
- Output: `β_c = 2.4 · u · f + (1 − f) · ζ₀`, f = 0 → 1 over 3 s (the generated code has two identical copies, Gain7
  and Gain3, for β_c and the nosewheel). The FD yaw bar is its 2 rad/s lag; the nosewheel gets −0.04 × it with the AP
  engaged.

## Compared with the A320 FCOM (DSC-22)

- Bank limits: the FCOM gives 25° in selected and 30° in managed lateral modes, reduced to 15° below the manoeuvring
  speed −10 kt (in take-off/approach and engine out). The model uses a TAS schedule (15° at 0 kt), 25° for all non-LOC
  laws (NAV gets 30° only through the FMS limit), 30° in LOC above 700 ft and 10° below 700 ft RA; no engine-out or
  manoeuvring-speed logic.
- Turn direction in HDG/TRK: the FCOM says the knob's rotation direction usually sets the turn, the shortest turn only
  for a preset heading at engagement. The model always takes the shorter way (with the 10–20° latch); the "first second"
  logic matches the "shortest at the moment of engagement" rule.
- LOC capture: the deviation term limit (intercept angle frozen at capture, 15–115°) echoes the FCOM's enhanced LOC
  capture numbers (15° convergence, track/QFU difference up to 115°), but there is no FMS-position pre-capture.
- The yaw bar (FD β) is the lagged AP yaw command in every mode; only RWY/ROLL OUT and LOC TRACK below 30 ft give it a
  value (FCOM: yaw bar in RWY, FLARE and ROLL OUT).
- RWY (LOC submode, latch `Memory_PreviousInput_c`) uses the ROLL_OUT law: on the take-off roll the yaw bar steers
  to the ILS runway heading memorised at RWY engagement, with the localizer deviation terms.
- Distances from angular deviation use the DME (sin(δ) · DME): an ILS without a DME is treated as 2 nm from the
  antenna, which under-scales the LOC* cross-beam term (by the true distance / 2 nm) and makes the capture a mostly
  track-to-beam-bearing law; on the runway the 2 nm floor applies anyway.

## Checks and coverage (2026-11-14)

`JOBS=6 scripts/check-laws-corpus.sh 3`: 176 runs (22 recordings; generated commands into the output stage,
free-running, 3 fuzzed and 3 fuzzed-approach runs each), 0 mismatches on every lateral and vertical channel. The
checker prints the steps per active lateral law. The recordings fly NAV and HDG with the AP, TRK with the AP (one
file), two ILS approaches (LOC*, LOC, roll-out; one with the AP to the end) and RWY take-off rolls; the approach
scenarios add LOC, roll-out and HDG/TRK with and without AP.

Mutation test (throwaway script, 119 single mutations of `laws_lateral.cpp`: gains, limits, filter constants, table
values and breakpoints, comparisons, dropped terms, memories): 107 caught on a 15-run subset, 2 more on the full
corpus (HDG's first-second detector fed by the TRACK flag; LOC* deviation limit not lowered to 15°). 10 survive:

- equivalent: ROLL_OUT rudder memory not taken / dropped from the output (ζ input is the constant 0); `r == 1`
  interpolation branch removed (the general formula gives the same value); `<` → `<=` in the shortest-angle choice
  (differs only at exactly 180°);
- unreachable: initial bank limit 25 → 24° (matters only in the first 2 s after a power-up, on the ground with
  φ_c = φ); law values outside 0–6 sent to NONE instead of ROLL_OUT; NaN roll command clipped to −limit;
- uncovered: DME upper clamp 100 → 50 nm (no LOC deviation beyond 50 nm), and the length of the HDG/TRK "first
  second" window (1 → 2 s, 100 → 50 samples): it only matters when a 10–20° turn is engaged and the target is then
  moved through the reciprocal within the window.

Situations the corpus lacks: engine-out flight (large steady sideslip, the LOC_TRACK sideslip term at its full 0.8
gain), HDG/TRK reversals through the reciprocal shortly after engagement, back-course or offset localizers, LOC
capture far from the antenna (> 50 nm DME). The approach-scenario runs are synthetic (deviation profiles injected on
top of unrelated flight data), so the physical response of the LOC laws is only seen in the two real approaches.
