# Autothrust N1 command

Readable code: `readable/athr_n1.cpp` (state: `N1State` in `readable/athr.h`, resync: `resyncAthrN1` in
`src/athr_resync.h`). Generated: `FmgcComputer.cpp` lines 4425-4640 on master, output `out.athr.n1_c_percent` (4956),
end-of-step state 5126-5128, initialisation 519-521 and 682-691. Checked bit for bit by `fmgc-check-athr`
(`scripts/check-athr-corpus.sh`).

## Structure

The FMGC never computes an N1 target. Every mode commands an N1 **rate** (%/s), and one integrator, shared by all
modes, turns it into the command sent to the FADECs:

```
n1_c = limit[18 %, N1 ref](n1_c(previous frame) + rate · dt)
```

The mode stage (`athr_modes.cpp`) provides the inputs: A/THR active, alpha floor, the SPEED/MACH, THR and RETARD
latches and `thrust_target_idle`. In priority order:

| Condition | N1 rate |
|---|---|
| `tracking_mode_on_override` (sim) | 0 (hold) |
| RETARD | -10 %/s |
| SPEED/MACH | speed controller below, limited to ±5 %/s |
| THR | +ramp(N1 actual), or -ramp(N1 actual) when `thrust_target_idle`, limited to ±10 %/s |
| no mode | 0 (hold) |

Alpha floor (and TOGA LK, the same latch) replaces the output with the higher engine's `n1_maximum_percent` (TOGA).
It does not drive the integrator, which tracks the actual N1 while alpha floor is on (see below).

All filters run every frame, whatever the mode, so their state is current when SPEED/MACH engages.

## SPEED/MACH law

Speed error: `e = median(VLS, speed target, VMAX) - IAS` (kt, float arithmetic; positive = too slow). The median is
the only speed protection in this law: a target below VLS or above VMAX is replaced by the limit.

**Speed trend (prediction).** The controller works on a predicted speed error:

```
e_pred = e - trend · k(e)
trend  = 1.944 · [ HP_0.2(a_inertial) + 5s/(5s+1) · IAS ]          (kt; IAS and ground speed in m/s, limited to 60-480 kt)
k(e)   = 1.8 for e ≤ -20 kt (well above target), 1 at e = 0, 1.2 for e ≥ 10 kt (well below), linear between
```

- `5s/(5s+1) · IAS` is 5 s times the IAS rate lagged 5 s: `e - 5·dV/dt` is the speed error expected in about 5 s if the
  acceleration holds.
- `a_inertial` is the ground speed rate (washout `2s/(s+2)` on 2·V_gnd, i.e. dV/dt lagged 0.5 s) minus
  `g · [(θ - γ)(1 - cos φ) + sin φ · sin(track - heading)]`, the gravity/attitude terms of the inertial along-track
  acceleration in a bank and with drift (γ from inertial V/S and ground speed). It is high-passed with `0.2s/(0.2s+1)`:
  only its fast part is kept. It is an acceleration (m/s²) added to a speed (m/s), so it weighs the fast part of the
  acceleration as if it lasted 1 s: a quick response to gusts before the IAS trend (lagged 5 s) builds up.
- The larger trend gain far above target makes thrust come back earlier during a large deceleration, so the speed does
  not undershoot the target.

**Rate command.**

```
rate = [ 0.35 · e_pred  +  LAG_0.1(d e_pred / dt)  +  10 · Δn_z  +  3 · WO_0.1(10 · Δn_z) ] · G_N1(n1_c) · G_soft
rate limited to ±5 %/s
```

- `0.35 %/s per kt` of predicted error, plus the error rate (a discrete derivative, lagged 0.1 s): a PD law on the
  predicted error that integrates into N1, i.e. a PI law from speed error to N1.
- `Δn_z` (float arithmetic) = normal acceleration (lagged 1/3 s) - cos θ / cos φ: the load factor increment over the
  1 g of steady flight at the current attitude. 10 %/s per g, plus 30 %/s per g on changes (washout 0.1 s): when the
  aircraft is pulled up thrust is added before the speed starts to decay.
- `G_N1`: scheduled on the *previous frame's* N1 command: 5 below 20 % N1, decreasing linearly to 1 at 45 % and
  above. Low N1 commands (near or below idle) move five times faster.
- `G_soft`: ALT soft mode (cruise). A blend rises from 0 to 1 in 10 s while `alt_soft_mode_active` (back to 0 in
  0.5 s) and the gain goes from 1 to 0.01: in soft mode the thrust is nearly frozen.

## THR law

```
rate = ±ramp(min(N1 actual own, N1 actual opp))     + towards the N1 ref limit, - when thrust_target_idle
ramp = 10 %/s at 0-20 % N1, linear to 1 %/s at 30 %, 1 %/s above 30 % N1 (table on 0/20/30/45/60/80/100 %)
```

The ramp depends on the **actual** engine N1, not on the command, and is 1 %/s in every flight case (flight idle at
altitude is above 30 % N1). The target is never computed: in THR CLB/MCT the command ramps up until it reaches the
upper limit (the N1 ref from the thrust lever detent, see below), in THR IDLE it ramps down until the 18 % floor. The
±10 %/s saturation after the table is never active (the table never exceeds 10).

## RETARD

-10 %/s down to the 18 % floor.

## Integrator and limits

- **Load (initial condition).** The integrator is loaded with the **higher** actual N1 of the two engines on the first
  running frame of the computer, and on every frame where A/THR is not engaged (`athr_own_engaged` false), not active
  (`athr_active` false, e.g. thrust levers out of the active range), or in alpha floor/TOGA LK. Generated:
  `icLoad = !athr_own_engaged || !athr_active || alpha_floor || icLoad`, `icLoad` cleared at the end of every step.
  While loaded, `n1_c = (N1 actual + rate·dt - rate·dt)`, i.e. the actual N1 up to rounding. The `!athr_own_engaged`
  term is redundant (A/THR active implies engaged).
- **Upper limit**: the higher engine's `n1_ref_percent` (the N1 for the thrust lever position: CLB in the CL detent,
  MCT in the MCT detent). This is the "thrust not greater than the thrust commanded by the thrust lever" of the FCOM.
- **Lower limit**: a fixed **18 %** N1. It is checked after the upper limit, so if the N1 ref were below 18 % the
  command would be the N1 ref.
- The limited value is the integrator state for the next frame (clamping anti-windup at 18 % and N1 ref only).
- Alpha floor overrides the output after the integrator (`max(N1 max own, N1 max opp)`).

There is no knowledge of the engine's idle N1: nothing stops the integrator between flight idle (about 30-35 % at
cruise altitudes) and 18 %.

## Windup at a SPEED → THR CLB change (2026-11-14-16-38-03)

Replayed through the stock model (probe, values from the generated outputs):

| t (s) | mode | n1_c (%) | N1 actual (%) | IAS / target (kt) |
|---|---|---|---|---|
| 923 | ALT, SPEED | 86.0 | 86.0 | 314 / 315 |
| 924 | target 290 | 82.3 | 84.5 | 314 / 290 |
| 924-937 | SPEED, rate saturated at -5 %/s | 82 → 18 | 84 → 36 | 314 → 307 |
| 937-941 | SPEED, command at the 18 % floor | 18.0 | 36 → 34 | 307 → 303 |
| 941.1 | OP CLB, THR CLB | 18.1 | 34.0 | 303 / 290 |
| 955 | THR CLB, command crosses actual | 32 | 31.6 | 289 |
| 960-1005 | THR CLB, speed held by pitch: descent | 38 → 83 | 32 → 83 | 286 |
| ≈1014 | command reaches CLB (N1 ref 91.4) | 91.4 | 91.8 | 292 |

Altitude: 25 034 ft at 954 s, 24 110 ft at 1006 s (-920 ft) while in OP CLB.

Why the command was 16 % below the actual N1 at the mode change:

1. In SPEED mode, decelerating 25 kt in level flight, the rate was at its -5 %/s limit for 13 s. The engines stopped
   at flight idle (~34 % at FL250), but the integrator only has the 18 % floor, so the command kept going down to
   18 % at 937 s (and stayed there: the speed was still 13 kt above target).
2. At 941.1 s the mode changed from SPEED to THR CLB. The integrator is re-loaded from the actual N1 only when A/THR
   is not engaged, not active, or in alpha floor. A/THR stayed engaged and active throughout (thrust levers in CL), so
   `icLoad` stayed false: a change of A/THR mode does not re-initialise the integrator. THR CLB started from the
   wound-down 18 %.
3. The THR ramp rate comes from the actual N1 (34 %, above the 30 % breakpoint): 1 %/s. The command needed 14 s to get
   back to the actual N1 (engines at idle meanwhile) and 73 s in total to reach climb N1. The OP CLB pitch law held
   the target speed, so with idle and then slowly rising thrust it traded altitude for speed.

So the windup cost the first ~14 s (engines at idle after OP CLB), and the 1 %/s ramp the rest: starting from the
actual 34 %, the command would still have needed ~57 s to reach CLB (≈998 s instead of ≈1014 s).

### Smallest change to start the THR ramp from the actual N1 (implemented on personal/fg-athr-thr-reload; this branch stays stock)

Add one term to the integrator load condition: entering THR mode while the command is below the actual N1.

```
icLoad = !athr_own_engaged || !athr_active || alpha_floor || icLoad
         || (thrust_mode_active && !previous.thrust_mode_active && n1_c(previous) < max(N1 actual))
```

In Simulink: an OR input on the `DiscreteTimeIntegratorVariableTsLimit` external reset/IC-load built from a rising
edge on `thrust_mode_active` (the previous outputs are already available as `Delay1_DSTATE`) ANDed with a comparison
of the integrator state to `max(N1 actual)`. The existing load path then sets the command to the actual N1, so THR CLB
ramps from 34 % instead of 18 %. The `n1_c < actual` condition avoids a step down when THR follows a SPEED spool-up
(command ahead of the engines). It does not change the 1 %/s ramp, which is the larger part of the delay.

Verified (2026-11-18): replaying 2026-11-14-16-38-03 through personal/fmgc-allfixes (bdf73cb7a) open loop, the
command at the SPEED → THR CLB change (941.05 s) is 33.99 % (the actual N1) instead of 18 %; only `n1_c` differs, for
the 5362 frames until the command would reach CLB. Readable vs generated lockstep over the corpus: 2730 runs, 0
mismatches.

A more general alternative is an anti-windup floor instead of the fixed 18 %: limit the integrator below to
`min(N1 actual) - margin` (the command cannot run away from the engines at idle in any mode, which also helps a
SPEED-mode re-acceleration after a deceleration at idle). That changes SPEED mode behaviour and needs a margin chosen
against the engine response, so it is not the smallest change.

## Compared with the FCOM (A320 FCOM DSC-22_30-90, A/THR)

In the public FCOM text (archive.org "A320 Neo FCOM 1") the A/THR modes are in DSC-22_30-90; DSC-22_30-50 there is
the AP/FD modes section.

- *THRUST mode "commands a specific thrust level"* (THR CLB = climb thrust, THR IDLE = idle): the FMGC has no thrust
  target; it ramps the command at a fixed 1 %/s (above 30 % actual N1) until the thrust lever limit (N1 ref) or the
  18 % floor. The resulting climb thrust is set by the FADEC limit, and idle by the engine, not by the FMGC.
- *A/THR "uses a thrust not greater than the thrust commanded by the thrust lever"*: matches (upper limit = N1 ref).
- *SPEED/MACH: no excursions beyond VLS and VMAX (selected) or maneuvering speed / maximum speed (managed)*: the law
  only uses VLS and VMAX (median with the target); any managed limits must come with the target from the FMS.
- *Approach autothrust: below 3200 ft RA with at least CONF 1 the A/THR is more responsive*: there is no such gain
  change in this law (no radio height or configuration input); the only gain schedules are on the N1 command, the
  speed error and ALT soft mode.
- *RETARD commands idle during the flare*: implemented as a -10 %/s ramp to the 18 % floor.
- *ALPHA FLOOR commands TOGA, frozen as TOGA LK*: matches (output = N1 maximum while the alpha floor latch, which
  includes TOGA LK, is set). The integrator tracks the actual N1 meanwhile, so leaving alpha floor (A/THR
  disconnected) does not produce a jump in the integrator.
- Not in the FCOM: the THR ramp depends on actual N1 (10 %/s below 20 % N1, i.e. on the ground or windmilling), the
  SPEED law gain is 5 times larger at low N1 commands, and a mode change never re-initialises the command (windup
  above).

## Mutation test

99 single mutations of `readable/athr_n1.cpp` (gains, limits, table values and breakpoints, filter constants, flipped
comparisons, swapped branches, dropped terms, A429 defaults), each rebuilt and run over the corpus (resync and free on
all 22 recordings, then fuzz seeds 1-3, then approach scenarios 1-3 for survivors). Throwaway scripts, not committed.

**84 of 99 caught** (78 by the plain corpus, 5 only by fuzzing, 1 only by the approach scenarios), i.e. 84 of the 94
non-equivalent mutants.

Equivalent (no observable difference):

- THR saturation 10 → 12 %/s: the ramp table never exceeds 10 %/s (10 → 9.5 is caught).
- 18 % floor `<` → `<=`: at equality both give 18.
- Integrator load without `!athr_own_engaged`: A/THR active implies engaged.
- Integrator load without alpha floor: the alpha floor/TOGA LK latch keeps A/THR active and is only reset by A/THR
  disengagement, which loads the integrator on the same frame; meanwhile the output is overridden by N1 max.
- Speed error computed in double and rounded to float: one rounding of an exact float difference, same result.

Not covered by the corpus, fuzzer or approach scenarios:

- `tracking_mode_on_override` (2 mutants: branch removed, RETARD given priority over it): the replayer sets it false
  and the fuzzer skips `sim_data.*`. Suggested checker change: let the fuzzer toggle
  `sim_data.tracking_mode_on_override` in episodes.
- Hold rate (0 → 0.1 %/s): reachable only through tracking override, or with no mode latched, which in the corpus only
  happens with A/THR inactive, where the loaded integrator cancels the rate.
- THR ramp table value at 0 % N1 (10 → 8): THR mode with an engine below 20 % N1 (engine start, shutdown or flameout
  with A/THR active). Rates between 20 and 30 % N1 are covered.
- THR ramp on the higher instead of the lower engine N1: both engines' N1 are equal in the recordings (and a
  difference only matters below 30 % N1). Needs an engine-out or asymmetric-idle case in THR mode.
- Upper and lower N1 limits checked in the other order: only differs when the N1 ref is below 18 % while the command is
  above it (FADEC N1 ref invalid or very low with the engines running).
- ALT soft blend initial value (0 → 1) and no integrator load on the computer's first frame: both only matter if the
  FMGC (re)starts with A/THR engaged and active in SPEED mode (power interruption in flight).
- Pitch angle default 0 → 1° (attitude correction) and VLS default 0 → 100 kt: the default is only used when the
  selected IR / FAC data is invalid (fuzzing one bus makes the logic pick another source), and the VLS default only
  matters with a speed target or VMAX below 100 kt.
