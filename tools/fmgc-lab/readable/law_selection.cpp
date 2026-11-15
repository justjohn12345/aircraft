// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "law_selection.h"

namespace fmgc::readable {

vertical_law selectLongitudinalLaw(const base_fmgc_ap_fd_logic_outputs& modes, fmgc_des_submode requestedDesSubmode) {
  const auto& m = modes.longitudinal_modes;
  const tcas_submode submode = modes.active_tcas_submode;
  if (m.flare_active) {
    return vertical_law::FLARE;
  }
  // LAND keeps the G/S law until FLARE (LAND engages from G/S and stays while G/S* / G/S drop).
  if (m.gs_capt_active || m.gs_trk_active || modes.lateral_modes.land_active) {
    return vertical_law::GS;
  }
  if (m.pitch_takeoff_active || m.pitch_goaround_active) {
    return vertical_law::SRS;
  }
  if (m.fpa_active) {
    return vertical_law::FPA;
  }
  // TCAS flies its V/S target in submode VS, the ALT* / ALT laws in the other two.
  if (m.vs_active || (m.tcas_active && submode == tcas_submode::VS)) {
    return vertical_law::VS;
  }
  if (m.clb_active || m.op_des_active || m.op_clb_active || m.exp_clb_active || m.exp_des_active) {
    return vertical_law::SPD_MACH;
  }
  if (m.alt_acq_active || (m.tcas_active && submode == tcas_submode::ALT_ACQ)) {
    return vertical_law::ALT_ACQ;
  }
  if (m.alt_hold_active || (m.tcas_active && submode == tcas_submode::ALT_HOLD)) {
    return vertical_law::ALT_HOLD;
  }
  if (m.final_des_active) {
    return vertical_law::VPATH;
  }
  if (m.des_active) {
    // Managed descent: the law the FMS asks for. With no submode requested the pitch has no law.
    switch (requestedDesSubmode) {
      case fmgc_des_submode::SPEED_THRUST: return vertical_law::SPD_MACH;
      case fmgc_des_submode::VPATH_THRUST:
      case fmgc_des_submode::VPATH_SPEED: return vertical_law::VPATH;
      case fmgc_des_submode::FPA_SPEED: return vertical_law::FPA;
      case fmgc_des_submode::VS_SPEED: return vertical_law::VS;
      default: return vertical_law::NONE;
    }
  }
  return vertical_law::NONE;
}

}  // namespace fmgc::readable
