# ALT* capture law (FmgcOuterLoops, vertical law ALT_ACQ = 2)

Source: `build/models/stock/FmgcOuterLoops.cpp` (= upstream master `model/FmgcOuterLoops.cpp`), lines refer to it.
Probe: `build/fmgc-probe-alt-capture-<variant> <file.fdr>` (src/probe_alt_capture.cpp) reads the law's state on every
capture during a replay.

## The law

An embedded MATLAB function, inlined at lines 2766–2797, runs **every frame regardless of the active law**:

```
dH = H_c − H_ind                                    ft; H_c = ap_fd_logic.alt_sel_or_cstr, H_ind = indicated altitude
r  = dH + sign(dH) · dH_offset                      dH_offset is persistent
if (law == ALT_ACQ) and not last frame:             law entry (wasActive_a)
    k         = Hdot / r                            1/min
    dH_offset = | 500 / |k| − 100 |                 ft
    r         = r + sign(r) · dH_offset
    k         = Hdot / r
    maxH_dot  = |Hdot|                              fpm, V/S at entry
Hdot_c = clamp(k · r, ±maxH_dot)                    fpm
```

Then (2800–2814): `Δγ = asin(clamp((Hdot_c − Hdot) · 0.00508 / clamp(V_tas·0.5144, 1, 2000), ±1))` in degrees, the
flight path angle change needed to reach the commanded V/S.

Speed protection (2935–3067), the same pattern as every path law: two speed-on-pitch commands, VLS (rtb_k1) and VMAX
(r), each from an energy-style speed law (ground-speed-derived acceleration through a washout, gravity term
`g·(θ − γ)` through a lead-lag high pass, IAS through a lead-lag low pass, then a gain on the speed error to VLS or VMAX).
`SpeedProtectionSignalSelection` (readable `blocks::speedProtection`) picks, with a 5 kt margin:
- FD: Δγ, or min(Δγ, VLS cmd) below VLS(−5 kt when the target is at or below VLS) + 5, or max(Δγ, VMAX cmd) above VMAX − 5;
- AP: the same with the AP gains 0.35·Δγ, 0.36·VLS cmd, 0.36·VMAX cmd.

Output (4482–4559, shared by all laws): `switch (vertical_law)` picks the law's AP Δθ (case 2 = ALT_ACQ), θ_c = θ + Δθ
limited to ±20°, then a rate-limited integrator (±0.6 g equivalent, `g/V_tas · 0.6` rad/s in deg) and a lag filter
(2 rad/s), faded in by the AP engage ramp (1/s in, 10/s out).

### Design intent

`Hdot_c = k·(dH + offset)`: a first-order (exponential) approach to the target altitude, with gain `k` chosen at
entry so the command starts at the current V/S (no step), plus a constant offset so the command does not decay to zero
but arrives at the target with `k · offset ≈ |500 − 100·k|` fpm. With typical entry values (k ≈ 3.5–5.3 /min) the
offset is 5–35 ft and the arrival V/S 20–120 fpm; ALT then engages within 20 ft (see the ALT mode doc).

Capture distance (FmgcComputer, ALT* engagement, lines 2480–2517): `|dH| < clamp((Hdot·0.00508)² / (g·f(|Hdot|))·3.28, 80, 3000)`
ft, a stopping distance with a V/S-dependent normal-acceleration factor f. At entry dH ≈ Hdot/4.4 for the corpus
captures (445 ft at 1900 fpm).

## Finding 1: the previous capture's offset leaks into the next capture (bug)

At entry, `r` is computed with the **old** `dH_offset` (from the previous capture) before the new one is added, so the
entry gain is `k = Hdot / (dH + sign·(old + new))` instead of `Hdot / (dH + sign·new)`. The state is only reset at
power-up (`LawMDLOBJ1.reset()` when the computer starts running, FmgcComputer.cpp:673; `dH_offset = 0` at
FmgcOuterLoops.cpp:2090). Consequences:
- First capture of a flight: correct (old = 0), the command starts at the current V/S.
- Later captures, normal case (old offset 5–70 ft): the command steps down at entry by 2–20 % (e.g. 1000 → 843 fpm,
  1339 → 1092 fpm in the corpus), then the capture is slightly slower. Minor.

## Finding 2: an ALT* entry with zero V/S poisons every later capture of the flight (the "400 fpm creep")

If ALT_ACQ is entered with Hdot ≈ 0, then `k ≈ 0` and `dH_offset = 500/|k| − 100` becomes huge (1e7–1e13 ft in the
corpus). If Hdot were exactly 0 the offset would be infinite and `k·r = 0·inf` NaN for every later capture (inferred,
not seen in the corpus: the recorded on-ground V/S is tiny but not exactly 0); `std::fmin/fmax` in the downstream
limiters return the non-NaN argument, so that would drive the pitch command to a rate limit. Through finding 1 that offset
enters every following capture: with `O` huge, `k ≈ Hdot/(O_old + O_new)` and

```
Hdot_c ≈ Hdot · O_new / (O_old + O_new) ≈ 500 · |Hdot| / (500 + |Hdot|)      (≈ 400 fpm from 2000 fpm, 445 from 4000)
```

constant for the whole capture, since dH ≪ O. Instead of an exponential capture from the current V/S, the aircraft
drops to ~350–450 fpm the moment ALT* engages and creeps to the target at that rate.

Where it happens: ALT* engages **on the ground** before takeoff (on_ground, RA 0, V/S 0) when the FCU altitude is within
the 80 ft minimum capture distance of the aircraft altitude. Every recording with a capture since 2026-11-11 16:39
has it (2026-11-11-16-39-50, -21-14-07, 2026-11-12-17-08-01, -19-54-17, -21-10-58); the earlier ones don't.

Probe output (recorded IR V/S 5, 10 and 20 s after ALT* engagement, and how long ALT* lasted):

| Recording | Capture | Entry V/S | Law cmd after entry (intended) | V/S +5 s / +10 s / +20 s | ALT* duration |
|---|---|---|---|---|---|
| 2026-11-10-21-18-10 (healthy) | climb, 413 ft | 1804 | 1804 (1804) | 1617 / 1043 / 380 | 21.7 s |
| 2026-11-10-15-32-54 (healthy) | climb, 455 ft | 1947 | 1947 (1947) | 1749 / 1149 / 483 | 24.7 s |
| 2026-11-12-21-10-58 (poisoned at t=124 s on the ground) | climb, 535 ft | 2208 | 408 (2208) | 931 / 371 / 422 | 56.5 s |
| 2026-11-11-21-14-07 (poisoned at t=460 s on the ground) | climb, 1382 ft | 4006 | 445 (4006) | 1262 / 298 / 399 | 89.7 s |

The replays match the recorded outputs exactly, so this is what the sim's FMGC computed on those flights.

Fix options (personal build only, hand edit of the generated code, or in the Phase 2/3 readable rewrite):
1. Compute the entry gain from `dH` alone: `k = Hdot / dH` before adding the new offset (removes finding 1 and 2's
   propagation).
2. Guard the offset: if |Hdot| is tiny (e.g. < 100 fpm), or `k` is 0, skip the update or clamp `dH_offset` to a sane
   range (e.g. ≤ 500 ft); also avoid `0 · inf` when dH = 0.
3. Inhibit ALT* (the law) on the ground: see the ALT mode doc for why it engages there.

## Related: level-off rate limits in V/S and FPA (possible bug)

V/S law (3809–3832): the pitch-change limit is 0.05 g, raised to 0.1 g during a level-off and 0.3 g in TCAS. Level-off
is `law == VS` entered with target 0, or target set to 0 **from a previous target > 500 fpm**, held while the target
stays 0. The FPA law (3446–3467) is the same with `prevTarget > 1.0°`. Only positive previous targets count: a level-off
from a V/S descent target (−1000 → 0) keeps the 0.05 g limit. Probably meant to be |prevTarget| (inferred).
