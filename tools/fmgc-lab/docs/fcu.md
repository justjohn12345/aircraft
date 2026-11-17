# The A32NX FCU computer (generated `FcuComputer`) and its readable rewrite

The FCU model turns the flight control unit panel (AFS part: SPD/MACH, HDG/TRK, ALT, V/S-FPA knobs and the LOC, APPR,
EXPED, SPD/MACH, TRK/FPA, METRIC ALT pushbuttons; two EFIS control panels: FD, LS, ND filters, baro) into the FCU bus
the FMGCs, DMCs and FADECs read, and into the panel lights and window values. It is small (1,570 generated lines, no
filters or look-up tables): a few latches, four window memories, dash timers, and bit packing.

- Generated: `fbw-a32nx/src/wasm/fbw_a320/src/model/FcuComputer.cpp` (+ `.h`, `_data.cpp`, `_types.h`). Never edited.
- Readable: `tools/fmgc-lab/readable/fcu/` (namespace `fcu::readable`), bit-exact (below).
- Wrapper: `fbw_a320/src/fcu/Fcu.cpp`; sim wiring `FlyByWireInterface::updateFcu()` (~2113).
- Parameters (block name = value): `docs/params-FcuComputer.txt`.
- Findings: `docs/findings.md`, section 7 (U1–U11), with C6 and C13 in section 1.

## Where it runs

Every frame, `FlyByWireInterface::update()` runs `updateFcu()` **before** the two FMGCs, so the FCU sees the FMGCs'
outputs of the previous frame, and the FMGCs see this frame's FCU bus. `Fcu::update()` computes the FCU health
(power supply of each of the two channels, 4 s self test after power returns, FCU1/FCU2 failures), sets
`sim_data.computer_running` = either channel healthy, and steps the model every frame. `Fcu::getBusOutputs()` returns
an all-zero bus when unhealthy, else the model bus with FCU 1/2 healthy on FCU discrete word 2 bits 24/25;
`getDiscreteOutputs()` all zero except `fcu_healthy` when unhealthy. (The personal builds with the C13 fix also clear
word 1 bit 15 in managed speed unless the SPD/MACH controls were used within 1 s.)

When `computer_running` is false the model does nothing: outputs (including `out.data`) and every memory are held.
The first running frame afterwards (or the very first step) **restarts** it: every memory is reset (generated 364-441)
and the windows re-read the aircraft. See "Restart" below.

## Structure (generated line ranges -> readable files)

| Generated | What | Readable |
|---|---|---|
| 6-310 | library blocks: MTrigNode, A429ValueOrDefault, PulseNode, BitfromLabel, dash timer (`MATLABFunction1`), EFIS filter, baro reference/value, FD latch, T flip-flop, bit packing, baro display | `fcu_blocks.h` (+ `readable/blocks.h`), `efis_panel.cpp` |
| 364-441 | restart on the first running frame | `fcu.cpp` `restart()` |
| 444-480 | FMGC priority, chosen aircraft data | `fmgc_interface.cpp` |
| 481-548 | knob and pushbutton pulses (0.1 s) | `afs_controls.cpp` |
| 549-572 | any AP/FD engaged; TRK-FPA and METRIC ALT latches | `fcu.cpp`, `afs_controls.cpp` |
| 574-737 | FMGC mode decoding: Mach, managed/selected speed, lateral/vertical managed, LOC/APPR/EXPED lights | `fmgc_interface.cpp` |
| 738-829 | SPD/MACH window | `speed_window.cpp` |
| 830-860 | `logic` bus assembly (interleaved) | `fcu.cpp` `afsLogic()` |
| 861-933 | HDG/TRK window | `heading_window.cpp` |
| 934-954 | ALT window | `altitude_window.cpp` |
| 955-1060 | V/S-FPA window | `vs_fpa_window.cpp` |
| 1061-1078 | value change pulses (0.5 s) | `fcu.h` `ValueChangeDetector` |
| 1079-1183 | FD auto on/off (FMGC 1), captain and F/O EFIS panels | `efis_panel.cpp`, `fcu.cpp` |
| 1184-1533 | discrete outputs and FCU bus | `output_packing.cpp` |
| 1534-1537 | value change detectors' previous values | `fcu.cpp` |

`readable/fcu/fcu.h`: `fcu::readable::FcuComputer` with `initialize()` and `const fcu_outputs& step(const fcu_inputs&)`,
no heap, no static state, copyable (state accessors for the checker). It is a complete reset: `initialize()` on a used
instance equals a fresh one (checked with `fcu-check --warm`); the generated `initialize()` is not (its `Runtime_MODE`
stays true), so the checker re-constructs the generated instance.

## Inputs (`fcu_inputs`)

| Field | Sim source (`updateFcu`) | Used for |
|---|---|---|
| `time.dt` | frame sample time | the 0.1 s / 0.5 s / 1 s monostables |
| `time.simulation_time` | sim time | the 10 s / 45 s dash timers (they stand still in pause) |
| `time.monotonic_time` | | unused |
| `sim_data.computer_running` | FCU health (wrapper) | run / hold / restart |
| `sim_data.slew_on`, `pause_on`, `tracking_mode_on_override`, `tailstrike_protection_on` | | unused |
| `sim_input.spd_mach`, `hdg_trk`, `alt`, `vs_fpa` | `A32NX.FCU_*_SET` events (integer; -1 = none) | sets the window (SPD/MACH: Mach x100; V/S-FPA: FPA x10) |
| `sim_input.left/right_baro_setting_hpa` | `A32NX.FCU_EFIS_L/R_BARO_SET`, KOHLSMAN events, init (-1 = none) | sets the baro value (hPa, inHg = hPa x 0.02953 unrounded) |
| `discrete_inputs.ap/fd/athr_1/2_engaged` | FMGC 1/2 discrete outputs (previous frame) | FMGC priority, AP/FD engaged, AP/A/THR lights, V/S sync on AP engagement |
| `discrete_inputs.lights_test` | `L:A32NX_OVHD_INTLT_ANN` | unused (the cockpit XML does the light test itself) |
| `discrete_inputs.pin_prog_qfe_avail` | constant false | QFE never available: baro push toggles QNH/QFE and is forced back to QNH |
| `capt/fo_efis_inputs.efis_range/mode/navaid_1/2/baro_is_inhg` | `L:A32NX_FCU_EFIS_L/R_*` | packed raw into the EIS words; baro unit |
| `capt/fo_efis_inputs.baro_knob`, `fd/ls/cstr/wpt/vord/ndb/arpt_button_pushed` | events, true for one frame | EFIS panel logic |
| `afs_inputs.*_knob.pushed/pulled/turns`, `*_button_pressed` | events, true / +-1 for one frame | AFS pulses and windows |
| `afs_inputs.alt_increment_1000` | `L:A32NX_FCU_ALT_INCREMENT_1000` | ALT knob step 100 / 1000 ft |
| `bus_inputs.fmgc_1_bus`, `fmgc_2_bus` | FMGC 1/2 A bus (previous frame) | everything FMGC-related (below) |

Knob turns are assigned, not accumulated, by `SimConnectInterface` (`turns = 1` / `-1` per event): several clicks in one
frame count once (finding U5).

### What the FCU reads from the FMGCs

FMGC priority (444-452): AP1 -> 1; else AP2 -> 2; else FD1 -> 1; else FD2 -> 2; else A/THR1 -> 1; else A/THR2 -> 2;
else 1. All words below are from the **priority** FMGC except where marked **FMGC 1** (C6). Bits are ARINC bits
(1-based), the FMGC side is `readable/output_packing.cpp`.

| Word | Bits | Meaning | FCU use |
|---|---|---|---|
| CAS, Mach, heading, track, altitude, V/S, FPA | data (0 unless SSM NO) | aircraft values (ADR/IR) | `chosen_fmgc_data`; window sync and dashed values |
| ATS discrete word (**FMGC 1**) | 19 | Mach control active | SPD/MACH window in Mach (`mach_active`) |
| dw1 (**FMGC 1**) | 15, 16 | SRS TO, SRS GA | TRK-FPA reset; 16 also FD auto on |
| dw7 (**FMGC 1**) | 13 | TCAS | TRK-FPA reset, FD auto on |
| dw4 (**FMGC 1**) | 27 | FD auto disengage | FD auto off |
| dw5 | 19, 20, 28, SSM | managed speed, selected speed, preset activate | SPD/MACH managed dot, dashes; preset |
| preset Mach / speed | data | MCDU preset (by FMGC 1's Mach bit) | SPD/MACH memory while bit 28 and > 0 |
| dw2 | 11-15, 16-17 | RWY, NAV, LOC*, LOC, GA TRK; HDG, TRK | HDG/TRK managed / selected |
| dw3 | 11, 14, 16, 20, 22-25 | HDG preset, NAV/LOC/LAND/G/S/FINAL/CLB/DES armed | managed dots, LOC/APPR lights, HDG dashes |
| dw4 | 14 | LAND | lateral/vertical managed, APPR light |
| dw1 | 11, 12, 13, 14, 15, 16, 17, 18, 19, 22, 23, 24 | CLB, DES, (spare), OP, SRS TO, SRS GA, V/S, FPA, ALT*/ALT, G/S, FINAL, EXP | LVL/CH managed, V/S-FPA, EXPED light, APPR light |
| FM altitude constraint | SSM | constraint applicable | with dw1 bit 19: ALT CST (managed) |
| dw7 | 13, 24, 27, 28 | TCAS, RA inhibited, RA corrective, TCAS V/S > 0 | V/S-FPA target after TCAS |
| ATS FMA, ATS, flex temp, N1 command | whole word | | passed through to the FCU bus |

## Outputs

### `logic.afs` (FCU-internal, also packed)

`fmgc_1_has_priority`, `chosen_fmgc_data` (7 aircraft values), `any_ap_fd_engaged` (AP1/2 or FD1/2), `mach_active`,
`trk_fpa_active`, `metric_alt_active`, `auto_speed_control` (dw5 b19 NO), `selected_speed_control` (dw5 b20 NO, or dw5
FW), `spd_mach_display_value` / `_dashes`, `hdg_trk_managed` (NAV/LOC armed, or RWY/NAV/LOC*/LOC/GA TRK/LAND),
`hdg_trk_selected` (HDG/TRK), `hdg_trk_display_value` / `_dashes`, `hdg_trk_preset_available`, `alt_display_value`,
`lvl_ch_managed` (CLB/DES/G/S/FINAL armed, or ALT CST, managed CLB/DES, SRS, G/S, FINAL, LAND), `lvl_ch_vs_fpa`,
`vs_fpa_display_value` / `_dashes`, `exped_active`, `loc_only_active`, `appr_active`, the four knobs' 0.1 s pulses
(`*_buttons.pushed/pulled/turns`, turns is 0/1), `loc_pushed`, `exped_pushed`, `appr_pushed`,
`spd_mach_switching_pushed` (0.1 s), and the 0.5 s value change pulses `lat_value_changed`, `alt_value_changed`,
`vpath_value_changed`, `spd_mach_value_changed`.

### `logic.capt_efis` / `fo_efis`

`fd_on`, `fd_auto_activate` / `fd_auto_deactivate` (the FMGC 1 edges, same on both sides), `ls_on`, `efis_filter`,
`baro_std`, `baro_qnh` (QNH and not STD), `baro_qfe`, `baro_value_hpa`, `baro_value_inhg`.

### Discrete outputs (lights and windows)

Per EFIS panel: FD, LS and filter lights, `baro_value_mode` (0 STD, 1 hPa, 2 inHg), `baro_value` (0 in STD),
`baro_mode` (0 STD, 1 QNH, 2 QFE). AFS: LOC light = LOC only (LOC*/LOC/LOC armed without LAND armed/active), EXPED
light = EXP, APPR light = LAND armed/active or FINAL armed/active, AP1/AP2 lights = the engaged discretes, A/THR light =
either A/THR engaged, `trk_fpa_mode`, `mach_mode`, the four window values (as double) and dashes, `spd_mach_managed`,
`hdg_trk_managed`, `lvl_ch_managed` (the managed dots), `fcu_healthy` = true (the wrapper sets the real one).

### FCU bus

| Word | SSM | Data |
|---|---|---|
| selected HDG / TRK | NCD when dashed or in the other reference | HDG/TRK memory (both words) |
| selected ALT | always NO | ALT memory |
| selected SPD | NCD when dashed or in Mach | SPD memory, or the current CAS in Mach |
| selected Mach | NCD when dashed or not in Mach | Mach memory, or the current Mach in SPD |
| selected V/S / FPA | NCD when dashed or in the other reference | V/S-FPA memory (both words) |
| ATS FMA, ATS discrete, flex temp, N1 command | priority FMGC's | priority FMGC's |
| EIS discrete words 1/2 left/right, baro hPa/inHg left/right | NO | below |
| FCU discrete words 1, 2 | NO | below |

```
FCU discrete word 1                                     FCU discrete word 2
  11 SPD knob pushed        19 V/S knob pulled            11 HDG knob pushed         21 FMGC 2 has priority
  12 SPD knob pulled        20 METRIC ALT                 12 HDG knob pulled         24 FCU 1 healthy (wrapper)
  13 ALT value changed      21 SPD/MACH pb                13 LOC pb                  25 FCU 2 healthy (wrapper)
  14 V/S-FPA value changed  22 EXPED pb                   14 HDG/TRK value changed   26 capt FD off
  15 SPD/MACH value changed 23 APPR pb                    20 FMGC 1 has priority     27 F/O FD off
  16 V/S knob pushed        24 HDG-V/S
  17 ALT knob pushed        25 TRK-FPA
  18 ALT knob pulled
EIS discrete word 1: 11 baro inHg; 25-29 ND range 10, 20, 40, 80, 160 (320: no bit; the shim decodes "none" as 320)
EIS discrete word 2: 11-15 ND mode PLAN, ARC, ROSE NAV, ROSE VOR, ROSE ILS; 17-21 filter CSTR, WPT, VOR.D, NDB, ARPT;
  22 LS on; 23 FD off; 24/25 navaid 1/2 ADF; 26/27 navaid 1/2 VOR; 28 STD; 29 QNH
```

The FMGC reads word 1 bits 11, 12, 13 (ALT changed: ALT* arm-possible and the 3 s ALT* block), 15 (SPD/MACH changed:
ALT soft mode reset, C13), 16-19, 21-24 and word 2 bits 11-13, 26, 27. Nobody reads word 1 bit 14 or word 2 bit 14.

## The windows

All four share one pattern (`windows.h`), every running frame, in this order:

1. first frame after a (re)start: the memory takes the aircraft's current value (CAS or Mach; heading or track; altitude
   rounded to the knob step; V/S or FPA);
2. a value set from the sim replaces it;
3. a reference switch re-derives it (SPD<->MACH: the current CAS/Mach; HDG<->TRK: see below; V/S<->FPA: the current
   V/S/FPA);
4. **while dashed, it follows the aircraft's current value** (SPD/MACH and V/S-FPA unrounded until step 6; HDG/TRK
   rounded);
5. window-specific overrides;
6. the knob's turns of this frame are added, then the value is limited and rounded to the window resolution.

The memories are float; the readable code does the same float operations in the same order (a Mach is
`round(m / 0.01F) * 0.01F`, which is not the float nearest to 0.78), so the values are bit-identical.

### SPD/MACH (738-829; memory `pValue_p`)

- Unit: Mach when **FMGC 1**'s ATS word bit 19 (Mach control) is set (C6), knots otherwise. The SPD/MACH pb does not
  switch it in the FCU: the pb is sent to the FMGCs (word 1 bit 21) and the FCU follows FMGC 1's Mach bit.
- Dashes: at once when the FMGC leaves selected speed (falling edge of dw5 bit 20 / FW); 10 s after the last
  "selection" (selected speed, a knob turn, a sim value). With no AP/FD and the FMGC in neither managed nor selected
  speed, the window dashes 10 s after power-up.
- The dashed memory is the **current CAS/Mach** (the FMGC's ADR words), not the managed target: turning the knob in
  managed speed starts from the current speed. Every 1 kt / 0.01 Mach the speed crosses moves the memory and pulses
  word 1 bit 15 (C13).
- Overrides: the MCDU preset speed/Mach replaces the memory while dw5 bit 28 is set and the preset is above 0.
- Turns: +-1 kt or +-0.01 per click; 100-399 kt, M0.10-0.99.

### HDG/TRK (861-933; memory `pValue_m`)

- Dashes: at once when the FMGC has no HDG/TRK mode while an AP/FD is engaged (two rising edges: with and without a
  preset); 45 s after the last "selection" (HDG/TRK mode, no AP/FD, a turn, a sim value, or a preset available). Never
  dashed without AP/FD.
- Dashed memory: the current heading/track, rounded to 1 deg.
- HDG<->TRK switch: if the target is within 5 deg of the old reference (the current heading when switching to TRK, the
  current track when switching to HDG), or no AP/FD is engaged, it becomes the new reference (current track/heading);
  otherwise the number is kept (a selected heading becomes a selected track of the same value: U3).
- Turns: 1 deg per click, rounded, 0-359 wrapped.

### ALT (934-954; memory `pValue_e`)

- Never dashed. Power-up value: the current altitude rounded to the knob step (100 or 1000 ft), limited to 100 ft
  (so 100 ft on the ground: the ALT* creep trigger of F1).
- Turns: `round((value + turns * (step/2 + 1)) / step) * step`: one click moves to the next multiple of the step, so
  with 1000 ft steps 5300 goes to 6000 or 5000. More than one click in a frame under-counts (2 clicks -> 1 step), but
  the sim never sends more than one (U5). A sim value is rounded the same way. 100-49000 ft.

### V/S-FPA (955-1060; memory `pValue`)

- Dashes: at once when the FMGC leaves V/S-FPA with an AP/FD engaged; 45 s after the last "selection" (V/S-FPA mode, no
  AP/FD, a turn, a sim value).
- Dashed memory: the current V/S (ft/min, the IR word) or FPA, unrounded until the end of the frame.
- Overrides, in priority: level-off (V/S-FPA engages within 1 s of a V/S knob push) -> 0; after TCAS (V/S-FPA engages
  within 0.5 s of the end of TCAS mode): "RA inhibited" (dw7 b24) -> the current V/S if the TCAS target was up (b28),
  else 0; "RA corrective" (b27) -> 1000 ft/min towards the FCU altitude, 500 ft/min if climbing above 30000 ft (U7).
  Also: on every AP engagement (rising edge of AP1 or AP2) the memory takes the current V/S/FPA (U8).
- Turns: 100 ft/min or 0.1 deg per click; +-6000 ft/min, +-9.9 deg.

### Value change pulses (1061-1078)

A 0.5 s monostable (rising edge, not retriggerable) on "memory differs from the last frame" for each window. Because
the dashed SPD/MACH, HDG/TRK and V/S-FPA memories follow the aircraft, these pulses fire with no pilot input (U1): in
the 25 recordings driven without any panel input (`fcu-check --synthetic`, 3.36 M steps), about 9,100 SPD/MACH,
9,800 HDG/TRK and 12,200 V/S-FPA pulses, against 25 ALT pulses (one per file: the restart pulse below).

## Panel latches and EFIS panels

- TRK-FPA: toggles on the TRK/FPA pb; held at HDG-V/S while FMGC 1 reports SRS TO, SRS GA or TCAS (U4). METRIC ALT:
  toggles on its pb, nothing resets it except a restart.
- FD: auto on at a rising edge of (FMGC 1 SRS GA or TCAS), auto off at a rising edge of FMGC 1's FD auto disengage;
  else the FD button toggles it, **every frame the input is true** (no edge detection, U10).
- LS: T flip-flop on the LS pb (rising edge). ND filters: one at a time, pushing the lit one clears it, several pushes in
  one frame: CSTR > WPT > VOR.D > NDB > ARPT.
- Baro: pull -> STD; push in STD -> back to QNH/QFE; push otherwise -> swap QNH/QFE, forced back to QNH (no QFE pin).
  Knob: hPa +1 per click (745-1100) with inHg recomputed rounded to 0.01; inHg +0.01 per click (22.00-32.48) with hPa
  recomputed rounded to 1. **In STD the knob does nothing** (no QNH preselection; the 2023 FCOM text does not describe
  one). A sim value sets hPa and an unrounded inHg.

## Restart (power-up or after `computer_running` false)

Everything restarts (U9): TRK-FPA -> HDG-V/S, METRIC off, both EFIS panels at power-up (FD on, LS off, no filter, QNH,
1013 hPa / 29.92 inHg: FCOM DSC-31-50 "At the EFIS CP power-up, the barometric reference is QNH and the QNH value is set
to 1 013 hPa or 29.92 inHg"), SPD/MACH, HDG/TRK and V/S-FPA re-synchronised to the current values, ALT to the current
altitude rounded, and the value change detectors start from 0, so the first frame pulses every non-zero window,
including "ALT value changed" (the FMGC then blocks ALT* for 3 s). The memories are complete: nothing survives a
restart (unlike the FMGC, F22).

## Hidden state (what the checker compares besides the outputs)

`Runtime_MODE`, `p_trk_fpa_active`, `p_metric_alt_active`, `pValue_p` / `pValue_m` / `pValue_e` / `pValue`, the three
dash timers' event times (`eventTime`, `sf_MATLABFunction1`, `sf_MATLABFunction1_o`), `DelayInput1_DSTATE[0..3]`,
and per EFIS panel the filter, FD, LS, STD/QNH/QFE and both baro values: 30 memories.

## Verification

- `fcu-check-<variant>` (src/check_fcu.cpp): generated and readable FCU in lockstep, free-running, every leaf of
  `fcu_outputs` (357, including `data`) and the 30 hidden memories bit-exact every step, plus the power-up outputs.
  Inputs: 3209001 recordings (recorded FCU inputs); any recording through the synthetic drive (src/fcu_drive.h: FMGC 1
  bus = its previous-frame output, FMGC 2 bus = the opposite bus FMGC 1 received, discretes likewise; `--fmgc2
  copy|invalid|ncd|swapped`; `--reconstruct` infers the pilot's panel actions from the recorded FCU bus and follows it
  closely: selected values match the recorded bus on >99.8 % of frames); seeded injectors `--script --panel --priority
  --bus --sim --power` (src/fcu_scenarios.h).
- `scripts/check-fcu-corpus.sh` (JOBS=4, SEEDS=2): 650 runs over the 25 recordings, 87.4 M steps (85.0 M running),
  14,692 restarts, **0 mismatches**. Mean step time: generated 0.42 us, readable 0.21 us (gcc -O2).
- Coverage (`COVERAGE=1`, clang source-based, same 650 runs): `FcuComputer.cpp` 98.6 % of regions, 99.1 % of lines,
  98.2 % of branches (542/552). The 10 branch sides never taken are all unreachable: the retriggerable MTrig and
  BitfromLabel range branches (constant parameters), `else if` second conditions implied by the first (baro push, baro
  unit, HDG/TRK switch), and the TRK-FPA initialisation of the HDG and V/S memories (the TRK/FPA pb edge cannot fire on
  the restart frame, which resets TRK-FPA). Readable FCU: 96.7-100 % of branches per file (the misses are the same
  unreachable cases).
- Mutation test (`scripts/mutate-fcu.sh`, 383 mutants of `readable/fcu/`: && <-> ||, ! dropped, == <-> !=, < <-> <=,
  > <-> >=, true <-> false, += -> -=, every numeric literal x1.5, every ARINC bit number +1): 371 caught. The first
  run left 24; 12 of them were killed by bus episodes added for them (a single lateral mode bit, the managed CLB/DES
  terms alone with the spare word 1 bit 13, HDG<->TRK switching at exactly 5 deg, a TCAS flicker inside the 0.5 s
  window, post-TCAS altitude boundaries). The 12 survivors: 11 equivalent (the "previous unit/reference primed" flags of
  the three windows and `running_` in `initialize()`: a restart re-initialises the memory to the same value either way;
  the +-180 deg wrap limits of the HDG/TRK switch, which cannot change the `< 5 deg` test) and 1 near-equivalent (the
  post-TCAS "RA inhibited, not up" value 0 -> 1 rounds back to 0 in V/S; it differs only in TRK-FPA with the priority
  FMGC 2 in TCAS and FMGC 1 not).
- `fcu-replay-<variant>` against recorded outputs: no 3209001 recording exists yet; checked on synthesized 3209001
  files (`fcu-synth-fdr`): 0 mismatches over 59,510 frames, and a single corrupted output field is reported.
  `--sync` (a file that starts with the FCU running) converges once the dash timers in progress have expired.

### Rerun

```sh
tools/fmgc-lab/scripts/build.sh                                  # or TARGETS='fcu-check-stock fcu-replay-stock'
JOBS=4 tools/fmgc-lab/scripts/check-fcu-corpus.sh                # every recording, 3209001 ones also replayed
FCU_ONLY=1 tools/fmgc-lab/scripts/build-fuzz.sh && COVERAGE=1 JOBS=4 tools/fmgc-lab/scripts/check-fcu-corpus.sh
tools/fmgc-lab/build/fcu-replay-stock ~/fdr-corpus/raw/<3209001 file>.fdr       # harness vs sim
tools/fmgc-lab/build/fcu-check-stock --panel 1 --bus 1 ~/fdr-corpus/raw/<file>.fdr
JOBS=4 tools/fmgc-lab/scripts/mutate-fcu.sh
```

## FCOM checks (A319/A320/A321 FCOM, 02 MAY 23)

Matches: SPD/MACH 100-399 kt / M0.10-0.99; HDG 1 deg per click; ALT 100-49000 ft in 100/1000 ft steps; V/S +-6000
ft/min, FPA +-9.9 deg by 0.1 deg; dashes return 10 s (SPD/MACH) / 45 s (HDG/TRK, V/S-FPA) after a turn in managed
(DSC-22_10-40-20); "the value in the window first synchronizes with the current HDG/TRK (SPD/MACH)" (DSC-22_30-40-30-10,
DSC-22_30-60-40); "AP/FD is lost: the value becomes that of the aircraft current heading or track"; push V/S to level
off with a zero target; V/S preselection cancelled after 45 s; HDG-V/S at power-up; FDs on at go-around and TCAS RA;
FPV reverts to FD bars at go-around; TRK FPA deselected by TCAS; EFIS CP power-up at QNH 1013/29.92; pull = STD,
push = QNH.

Differences and doubtful points are findings U1-U11 (`docs/findings.md` section 7).
