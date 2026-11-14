// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Generated <-> readable correspondence for the targets block (check_targets): the readable state and values take the
// generated memories from before the step (resync mode), and generatedValue() reads each field after the step.

#pragma once

#include "FmgcComputer.h"
#include "targets.h"

namespace targets_resync_detail {
inline void confirm(fmgc::blocks::ConfirmNode& c, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_k_T& g) {
  c.output = g.output;
  c.timeSinceCondition = g.timeSinceCondition;
}
inline void edge(fmgc::blocks::EdgeDetector& e, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_c_T& g) {
  e.previousInput = g.previousInput;
  e.primed = g.previousInput_not_empty;
}
inline void mono(fmgc::blocks::Monostable& m, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_f_T& g) {
  m.previousInput = g.previousInput;
  m.remainingTriggerTime = g.remainingTriggerTime;
}
}  // namespace targets_resync_detail

/** Every readable memory has a single generated counterpart. */
inline void resyncTargets(fmgc::readable::targets::State& s, fmgc::readable::targets::Values& v,
                          const FmgcComputer::D_Work_FmgcComputer_T& pre,
                          const FmgcComputer::BlockIO_FmgcComputer_T& preB) {
  using namespace targets_resync_detail;
  mono(s.preset.selectPulse, pre.sf_MATLABFunction_dln);
  confirm(s.preset.navArmedRecent, pre.sf_MATLABFunction_k4v);
  mono(s.preset.knobPushPulse, pre.sf_MATLABFunction_ppu);
  confirm(s.preset.noApFdRecent, pre.sf_MATLABFunction_k0);

  confirm(s.airborne5s, pre.sf_MATLABFunction_hjm);
  edge(s.liftoff5sPulse, pre.sf_MATLABFunction_od);
  edge(s.knobPulled, pre.sf_MATLABFunction_d5y);
  edge(s.managedLostManual, pre.sf_MATLABFunction_aag);
  edge(s.tcasEngaged, pre.sf_MATLABFunction_itu);
  confirm(s.manualSetRecent, pre.sf_MATLABFunction_jcu);
  edge(s.engineStartManual, pre.sf_MATLABFunction_abn);
  edge(s.apFdEngagedManual, pre.sf_MATLABFunction_kzm);

  edge(s.apFdEngagedAuto, pre.sf_MATLABFunction_ab);
  edge(s.engineStartAuto, pre.sf_MATLABFunction_hkc);
  edge(s.v2Inserted, pre.sf_MATLABFunction_ft);
  edge(s.srsEngaged, pre.sf_MATLABFunction_mrn);
  edge(s.knobPushed, pre.sf_MATLABFunction_ih);
  edge(s.expedEngaged, pre.sf_MATLABFunction_lr);
  confirm(s.autoSetRecent, pre.sf_MATLABFunction_j3h);
  edge(s.managedLostAuto, pre.sf_MATLABFunction_dq);
  edge(s.v2Deleted, pre.sf_MATLABFunction_hpe);

  edge(s.spdMachPushed, pre.sf_MATLABFunction_mw);

  s.gaSpeedMemoPrimed = pre.vMemoGa_not_empty;
  s.eoSpeedMemoPrimed = pre.vMemoEo_not_empty;

  confirm(s.altSoft.speedOff4kt10s, pre.sf_MATLABFunction_hi);
  confirm(s.altSoft.speedOff10kt4s, pre.sf_MATLABFunction_oep);
  confirm(s.altSoft.softCondFor120s, pre.sf_MATLABFunction_nj);

  v.hdgTrkPreset = pre.Memory_PreviousInput_bw;
  v.manualSpeed = pre.Memory_PreviousInput_cu;
  v.autoSpeed = pre.Memory_PreviousInput_hk;
  v.machMode = pre.Memory_PreviousInput_bo;  // the latch memory (= Delay_DSTATE_c at the start of every step)
  v.vappMemoKts = preB.u_l;
  v.gaSpeedMemoKts = pre.vMemoGa;
  v.eoSpeedMemoKts = pre.vMemoEo;
  v.lowerMarginActive = pre.Memory_PreviousInput_dh;
  v.spdTargetKts = pre.Delay_DSTATE.spd_target_kts;
  v.pfdSpdTargetKts = pre.Delay_DSTATE.pfd_spd_target_kts;
  v.altSoft = pre.Memory_PreviousInput_ak;
}

/** Generated value of a field after this step. */
inline double generatedTargetValue(int field, const FmgcComputer::D_Work_FmgcComputer_T& post,
                                   const FmgcComputer::BlockIO_FmgcComputer_T& postB, const fmgc_outputs& out) {
  namespace T = fmgc::readable::targets;
  switch (field) {
    case T::HdgTrkPreset: return post.Memory_PreviousInput_bw;
    case T::ManualSpeed: return post.Memory_PreviousInput_cu;
    case T::AutoSpeed: return post.Memory_PreviousInput_hk;
    case T::MachMode: return post.Delay_DSTATE_c;
    case T::VappMemo: return postB.u_l;
    case T::GaSpeedMemo: return post.vMemoGa;
    case T::EoSpeedMemo: return post.vMemoEo;
    case T::LowerMarginActive: return post.Memory_PreviousInput_dh;
    case T::SpdTarget: return out.ap_fd_logic.spd_target_kts;
    case T::PfdSpdTarget: return out.ap_fd_logic.pfd_spd_target_kts;
    case T::AltSoft: return post.Memory_PreviousInput_ak;
    default: return 0;
  }
}

/** Puts the readable value of an output field into an ap_fd_logic bus (free mode: the readable previous outputs). */
inline void putTargetOutput(base_fmgc_ap_fd_logic_outputs& o, int field, const fmgc::readable::targets::Values& v) {
  namespace T = fmgc::readable::targets;
  switch (field) {
    case T::HdgTrkPreset: o.hdg_trk_preset_available = v.hdgTrkPreset; break;
    case T::ManualSpeed: o.manual_spd_control_active = v.manualSpeed; break;
    case T::AutoSpeed: o.auto_spd_control_active = v.autoSpeed; break;
    case T::MachMode: o.mach_control_active = v.machMode; break;
    case T::SpdTarget: o.spd_target_kts = v.spdTargetKts; break;
    case T::PfdSpdTarget: o.pfd_spd_target_kts = v.pfdSpdTargetKts; break;
    case T::AltSoft: o.alt_soft_mode_active = v.altSoft; break;
    default: break;
  }
}
