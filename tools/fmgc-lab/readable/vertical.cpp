// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "vertical.h"

namespace fmgc::readable {

const char* signalName(Signal s) {
  static const char* names[kSignalCount] = {
      "SrsTakeoff", "LandArmed", "Land",      "FlareMemory", "FinalArmed",  "Final",          "SrsGoAround",
      "AltArmPossible", "AltArmed", "AltCapture", "AltHold", "AltCruiseMemory", "AltCruise", "ReversionVs",
      "VsFpa", "ClbArmed", "Clb", "DesArmed", "Des", "OpClb", "ReversionOpClb", "OpDes", "ExpClb", "ExpDes",
      "GlideArmed", "GsCapture", "GsTrack", "TcasArmed", "Tcas", "TcasAltAcq", "TcasAltHold", "AltSoft", "VsActive",
      "FpaActive", "Flare"};
  return names[static_cast<std::size_t>(s)];
}

void VerticalModeLogic::reset(Signals& s) {
  s = Signals{};
  resetApproach(s);
  resetAlt(s);
  resetVsTcas(s);
  resetClimbDescent(s);
}

bool VerticalModeLogic::update(Signal id, const Frame& f, Signals& s) {
  return updateApproach(id, f, s) || updateAlt(id, f, s) || updateVsTcas(id, f, s) || updateClimbDescent(id, f, s);
}

bool anyOtherVerticalModeLast(const base_fmgc_ap_fd_logic_outputs& last, Signal except) {
  const auto& m = last.longitudinal_modes;
  auto active = [&](Signal s, bool v) { return s != except && v; };
  return active(Signal::VsActive, m.vs_active) || active(Signal::FpaActive, m.fpa_active) ||
         active(Signal::AltCapture, m.alt_acq_active) || active(Signal::AltHold, m.alt_hold_active) ||
         active(Signal::Clb, m.clb_active) || active(Signal::Des, m.des_active) ||
         active(Signal::ExpClb, m.exp_clb_active) || active(Signal::ExpDes, m.exp_des_active) ||
         active(Signal::Final, m.final_des_active) || active(Signal::GsCapture, m.gs_capt_active) ||
         active(Signal::GsTrack, m.gs_trk_active) || active(Signal::OpClb, m.op_clb_active) ||
         active(Signal::OpDes, m.op_des_active) || active(Signal::SrsGoAround, m.pitch_goaround_active) ||
         active(Signal::SrsTakeoff, m.pitch_takeoff_active) || active(Signal::Tcas, m.tcas_active) ||
         active(Signal::Land, last.lateral_modes.land_active);
}

}  // namespace fmgc::readable
