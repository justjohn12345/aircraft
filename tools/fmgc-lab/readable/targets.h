// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC targets block (Phase 5c): HDG/TRK preset, selected (manual) and managed (auto) speed,
// the SPD/MACH (Mach) latch, the speed target fed to the laws and the A/THR (V_c = spd_target_kts), the PFD speed
// target, and the ALT soft mode. Stock behaviour: must reproduce the generated FmgcComputer exactly (checked by
// src/check_targets.cpp); fixes belong on personal/fmgc-fixes.
//
// Generated: FmgcComputer.cpp 3405-3734 on master (the block runs after the vertical and lateral mode logic and before
// the law selection). Spec and findings: docs/modes/targets.md.
//
// Inputs: this frame's raw inputs, input conditioning (`logic`) and modes (`modes`: the mode flags, opposite-FMGC sync
// and any-AP/FD-engaged of out.ap_fd_logic, all computed before this block), and the previous frame's ap_fd_logic
// (`last`, the generated Delay_DSTATE). The block's own outputs are read back only through `last`, except where a
// field updated earlier in this step is named (the speed target reads this frame's managed-speed and Mach latches, the
// ALT soft mode this frame's Mach latch and speed target).
//
// Not here: alt_sel_or_cstr / alt_cstr_applicable (the altitude target) are computed at FmgcComputer.cpp 1636-1664,
// inside the AP/FD engagement block; readable in vertical_alt.cpp (Signal::AltCstrApplicable / AltSelOrCstr).

#pragma once

#include <cstddef>

#include "FmgcComputer_types.h"
#include "blocks.h"

namespace fmgc::readable::targets {

/** The block's outputs and memories, in the order the generated step() computes them (line, generated name). */
enum Field : int {
  HdgTrkPreset = 0,   // [3434] ap_fd_logic.hdg_trk_preset_available        (Memory_PreviousInput_bw)
  ManualSpeed,        // [3481] ap_fd_logic.manual_spd_control_active       (Memory_PreviousInput_cu)
  AutoSpeed,          // [3521] ap_fd_logic.auto_spd_control_active         (Memory_PreviousInput_hk)
  MachMode,           // [3535] ap_fd_logic.mach_control_active             (Delay_DSTATE_c, latch memory _bo)
  VappMemo,           // [3568] VAPP, frozen in final approach              (FmgcComputer_B.u_l)
  GaSpeedMemo,        // [3616] CAS, frozen while SRS go-around is engaged  (vMemoGa)
  EoSpeedMemo,        // [3620] CAS, frozen while one engine is out         (vMemoEo)
  LowerMarginActive,  // [3656] DES VPATH_SPEED: speed fell below the lower margin (Memory_PreviousInput_dh)
  SpdTarget,          // [3660-3710] ap_fd_logic.spd_target_kts: V_c for the outer loops and the A/THR (kt)
  PfdSpdTarget,       // [3660-3710] ap_fd_logic.pfd_spd_target_kts (kt)
  AltSoft,            // [3734] ap_fd_logic.alt_soft_mode_active            (Memory_PreviousInput_ak)
  kFieldCount
};
const char* fieldName(int field);
inline bool isFlag(int field) {
  return field != VappMemo && field != GaSpeedMemo && field != EoSpeedMemo && field != SpdTarget &&
         field != PfdSpdTarget;
}

/** Field values. The latches and memos are also the block's memory from one frame to the next. */
struct Values {
  bool hdgTrkPreset = false;
  bool manualSpeed = false;
  bool autoSpeed = false;
  bool machMode = false;
  double vappMemoKts = 0;
  double gaSpeedMemoKts = 0;
  double eoSpeedMemoKts = 0;
  bool lowerMarginActive = false;
  double spdTargetKts = 0;
  double pfdSpdTargetKts = 0;
  bool altSoft = false;

  double get(int field) const;
  void set(int field, double v);
};

/**
 * Named conditions of this frame (coverage for the checker: which set/reset causes and target branches were seen).
 * Filled by the update of the field they belong to.
 */
enum Cond : int {
  // selected (manual) speed: set causes, reset causes
  ManualSetOppSync,           // the opposite FMGC (in control) has selected speed
  ManualSetPowerUpInFlight,   // first running frame, in flight
  ManualSetNoManagedAtLiftoff,// 5 s after lift-off with no FMS speed
  ManualSetNoApFd,            // no AP/FD, not in the approach phase
  ManualSetKnobPulled,        // SPD knob pulled (in flight for 5 s or engines stopped on the ground)
  ManualSetPreset,            // MCDU preselected speed/Mach activated
  ManualSetManagedLost,       // FMS speed target lost in flight (not SRS/G/S/LAND)
  ManualSetTcas,              // TCAS mode engaged
  ManualResetByAuto,          // managed speed took over (0.72 s after the last set condition)
  ManualResetGround,          // on the ground: engine start or first AP/FD engagement
  // managed (auto) speed
  AutoSetOppSync,
  AutoSetGroundApFd,          // on the ground: AP/FD engaged with V2, engines started with AP/FD and V2, V2 inserted
  AutoSetSrs,                 // SRS (takeoff or go-around) engaged
  AutoSetKnobPushed,          // SPD knob pushed
  AutoSetExped,               // EXPED engaged
  AutoSetNoSpeeds,            // a set cause, but neither FMS speed nor V2: blocked
  AutoResetByManual,
  AutoResetGround,            // on the ground, not SRS/approach: FMS speed or V2 lost
  // Mach latch
  MachSetPush,                // SPD/MACH pb in speed
  MachSetFms,                 // FMS Mach mode request (managed crossover)
  MachSetPreset,              // preset Mach activated
  MachResetPush,              // SPD/MACH pb in Mach
  MachResetFms,               // FMS speed mode request
  MachResetPreset,            // preset speed activated
  MachSetAndReset,            // set and reset in the same frame (reset wins)
  MachPushInManaged,          // SPD/MACH pb while the speed is managed
  // speed target branches
  TargetSelectedSpeed,
  TargetSelectedMach,
  TargetManaged,
  TargetManagedUpperMargin,   // DES SPEED_THRUST with speed margins
  TargetManagedLowerMargin,   // DES VPATH_SPEED below the lower margin
  TargetApproachGsMini,       // approach: VAPP + 1/3 headwind gust (tower headwind valid)
  TargetApproachVapp,         // approach without tower headwind
  TargetGsMiniLimited,        // ground speed mini limited by VFE next / VMAX - 5
  TargetGsMiniVmaxLimit,      // VFE next not normal: VMAX - 5 used as the limit
  TargetSrsTakeoff,           // V2 + 10
  TargetSrsTakeoffEngineOut,  // max(V2, min(V2 + 15, CAS at failure))
  TargetSrsGoAround,
  TargetFloorActive,          // V_c raised to the manoeuvring speed floor (managed, approach)
  TargetF3NoFFloor,           // CONF 3 landing in the approach phase: F speed not in the floor
  // ALT soft mode
  AltSoftCondition,           // ALT, cruise phase, Mach, A/THR engaged
  AltSoftEngage,              // condition held for 120 s with the speed on target
  AltSoftResetSpeedOff,       // speed off target (4 kt for 10 s or 10 kt for 4 s)
  AltSoftResetVmax,           // CAS above VMAX - 5
  AltSoftResetFcu,            // FCU word 1 bit 15
  kCondCount
};
const char* condName(int cond);

struct Trace {
  bool c[kCondCount]{};
};

struct Inputs {
  double dt;
  const fmgc_inputs& in;                          // raw inputs of this frame
  const base_fmgc_logic_outputs& logic;           // input conditioning (this frame)
  const base_fmgc_ap_fd_logic_outputs& modes;     // this frame's modes, opposite-FMGC sync, any AP/FD engaged
  const base_fmgc_ap_fd_logic_outputs& last;      // the FMGC's ap_fd_logic outputs of the previous frame
  bool firstFrame;                                // first running frame after power-up/restart (Delay_DSTATE_fe)
};

// ---------------------------------------------------------------------------------------------------------------------
// HDG/TRK preset and ALT soft mode: also called from the mode logic (lateral_modes.cpp, vertical_vs_tcas.cpp), which
// evaluates them at the same point of the step.
// ---------------------------------------------------------------------------------------------------------------------

struct HdgTrkPresetState {
  blocks::Monostable selectPulse;       // 0.72 s after an FCU heading/track preset appears or the knob is pulled (dln)
  blocks::ConfirmNode navArmedRecent;   // NAV armed within the last 0.72 s                                 (k4v)
  blocks::Monostable knobPushPulse;     // 0.72 s after the HDG/TRK knob is pushed                            (ppu)
  blocks::ConfirmNode noApFdRecent;     // no AP/FD within the last 0.72 s                                   (k0)
};

/** This frame's lateral modes the preset reads. */
struct PresetModes {
  bool navArmed, nav, locCapture, locTrack, land, final, gaTrk, rwyTrk, hdg, trk;
};

bool updateHdgTrkPreset(HdgTrkPresetState& st, double dt, const fmgc_inputs& in, const base_fmgc_logic_outputs& logic,
                        const PresetModes& m, bool anyApFdEngaged, bool q, Trace* trace = nullptr);

struct AltSoftState {
  blocks::ConfirmNode speedOff4kt10s;   // |speed target - CAS| > 4 kt for 10 s   (hi)
  blocks::ConfirmNode speedOff10kt4s;   // |speed target - CAS| > 10 kt for 4 s   (oep)
  blocks::ConfirmNode softCondFor120s;  // engagement condition held for 2 min    (nj)
};

/** altHold: ALT engaged this frame; machMode, spdTargetKts: this frame's Mach latch and speed target. */
bool updateAltSoft(AltSoftState& st, double dt, const fmgc_inputs& in, const base_fmgc_logic_outputs& logic,
                   bool altHold, bool machMode, double spdTargetKts, bool q, Trace* trace = nullptr);

// ---------------------------------------------------------------------------------------------------------------------
// The whole block
// ---------------------------------------------------------------------------------------------------------------------

/** Block memories other than the Values (generated counterparts in brackets). */
struct State {
  HdgTrkPresetState preset;

  // selected (manual) speed
  blocks::ConfirmNode airborne5s;           // in the air for 5 s                       (hjm)
  blocks::EdgeDetector liftoff5sPulse;      // ... rising edge                          (od)
  blocks::EdgeDetector knobPulled;          // SPD knob pulled, FCU word 1 bit 12       (d5y)
  blocks::EdgeDetector managedLostManual;   // FMS speed target became 0                (aag)
  blocks::EdgeDetector tcasEngaged;         // TCAS mode engaged                        (itu)
  blocks::ConfirmNode manualSetRecent;      // manual set condition within 0.72 s       (jcu)
  blocks::EdgeDetector engineStartManual;   // first engine started                     (abn)
  blocks::EdgeDetector apFdEngagedManual;   // first AP/FD engaged                      (kzm)

  // managed (auto) speed
  blocks::EdgeDetector apFdEngagedAuto;     // first AP/FD engaged                      (ab)
  blocks::EdgeDetector engineStartAuto;     // first engine started                     (hkc)
  blocks::EdgeDetector v2Inserted;          // V2 became non-zero                       (ft)
  blocks::EdgeDetector srsEngaged;          // SRS takeoff or go-around engaged         (mrn)
  blocks::EdgeDetector knobPushed;          // SPD knob pushed, FCU word 1 bit 11       (ih)
  blocks::EdgeDetector expedEngaged;        // EXP CLB or EXP DES engaged               (lr)
  blocks::ConfirmNode autoSetRecent;        // auto set condition within 0.72 s         (j3h)
  blocks::EdgeDetector managedLostAuto;     // FMS speed target became 0                (dq)
  blocks::EdgeDetector v2Deleted;           // V2 became 0                              (hpe)

  // Mach latch
  blocks::EdgeDetector spdMachPushed;       // SPD/MACH pb, FCU word 1 bit 21           (mw)

  // SRS speed memos: primed on the first running frame [vMemoGa_not_empty, vMemoEo_not_empty]
  bool gaSpeedMemoPrimed = false;
  bool eoSpeedMemoPrimed = false;

  AltSoftState altSoft;
};

class Targets {
 public:
  /** Power-up (model initialize()): everything, including the VAPP memo. */
  void initialize(Values& v);
  /** First running frame after a restart: the generated initialisation block (the VAPP memo is kept). */
  void reset(Values& v);

  /**
   * Updates one field for this frame. Fields are updated in enum order; a field may read fields updated earlier this
   * frame from `v`.
   */
  void update(Field id, const Inputs& in, Values& v, Trace& trace);

  State& state() { return st_; }
  const State& state() const { return st_; }

 private:
  State st_;
};

}  // namespace fmgc::readable::targets
