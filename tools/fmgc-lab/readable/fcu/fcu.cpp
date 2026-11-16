// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable FCU computer (generated FcuComputer::step / initialize, FcuComputer.cpp 312-1556). See fcu.h.

#include "fcu.h"

#include "output_packing.h"

namespace fcu::readable {

void FcuComputer::initialize() {
  running_ = false;
  restart();
  out_ = fcu_outputs{};  // out_Y0: all zero
}

void FcuComputer::restart() {
  // 364-441: the generated restart block resets every memory (DetectChange to 0, every MTrig/PulseNode, the TRK/FPA and
  // METRIC latches, the window memories' "not yet initialised" flags, both EFIS panels)
  afs_.reset();
  speed_.reset();
  heading_.reset();
  altitude_.reset();
  vsFpa_.reset();
  for (auto& c : changes_) {
    c.reset();
  }
  fdAutoActivate_.reset();
  fdAutoDeactivate_.reset();
  captEfis_.reset();
  foEfis_.reset();
}

const fcu_outputs& FcuComputer::step(const fcu_inputs& in) {
  if (!in.sim_data.computer_running) {
    running_ = false;  // outputs held
    return out_;
  }
  if (!running_) {
    restart();
    running_ = true;
  }

  base_fcu_logic_outputs l{};
  l.afs = afsLogic(in);

  // 1079-1089: FD auto on (SRS GA or TCAS) / off (FD auto disengage), FMGC 1 only, rising edges
  const bool fdOn = fdAutoActivate_.update(fmgc_.fdAutoActivateCondition, /*risingEdge*/ true);
  const bool fdOff = fdAutoDeactivate_.update(fmgc_.fdAutoDeactivateCondition, /*risingEdge*/ true);
  const auto& d = in.discrete_inputs;
  l.capt_efis = captEfis_.update(d.capt_efis_inputs, fdOn, fdOff, d.pin_prog_qfe_avail, in.sim_input.left_baro_setting_hpa);
  l.fo_efis = foEfis_.update(d.fo_efis_inputs, fdOn, fdOff, d.pin_prog_qfe_avail, in.sim_input.right_baro_setting_hpa);

  out_.data = in;
  out_.logic = l;
  out_.discrete_outputs = packDiscreteOutputs(in, l, EfisControlPanel::displayOutputs(l.capt_efis, d.capt_efis_inputs),
                                              EfisControlPanel::displayOutputs(l.fo_efis, d.fo_efis_inputs));
  out_.bus_outputs = packBusOutputs(in, fmgc_, l);

  // 1534-1537: the value change detectors' previous values
  changes_[kHdgTrk].previous = heading_.memory();
  changes_[kAlt].previous = altitude_.memory();
  changes_[kVsFpa].previous = vsFpa_.memory();
  changes_[kSpdMach].previous = speed_.memory();
  return out_;
}

base_afs_logic_outputs FcuComputer::afsLogic(const fcu_inputs& in) {
  const auto& d = in.discrete_inputs;
  fmgc_ = readFmgcBuses(in);
  const bool anyApFd = d.ap_1_engaged || d.ap_2_engaged || d.fd_1_engaged || d.fd_2_engaged;

  const AfsControls::Pulses& pulses = afs_.update(d.afs_inputs, in.time.dt, fmgc_);
  const bool trkFpa = afs_.trkFpaActive();
  const SpeedMachWindow::Output spd = speed_.update(in, fmgc_);
  const HdgTrkWindow::Output hdg = heading_.update(in, fmgc_, anyApFd, trkFpa);
  const float alt = altitude_.update(in, fmgc_);
  const VsFpaWindow::Output vs = vsFpa_.update(in, fmgc_, anyApFd, trkFpa, pulses.vsFpa.pushed, alt);

  base_afs_logic_outputs a{};
  a.fmgc_1_has_priority = fmgc_.fmgc1HasPriority;
  a.chosen_fmgc_data = fmgc_.data;
  a.any_ap_fd_engaged = anyApFd;
  a.mach_active = fmgc_.machActive;
  a.trk_fpa_active = trkFpa;
  a.metric_alt_active = afs_.metricAltActive();
  a.auto_speed_control = fmgc_.autoSpeedControl;
  a.selected_speed_control = fmgc_.selectedSpeedControl;
  a.spd_mach_display_value = spd.value;
  a.spd_mach_dashes = spd.dashes;
  a.hdg_trk_managed = fmgc_.lateralManagedArmed || fmgc_.lateralManagedActive;
  a.hdg_trk_selected = fmgc_.hdgTrkSelected;
  a.hdg_trk_display_value = hdg.value;
  a.hdg_trk_dashes = hdg.dashes;
  a.hdg_trk_preset_available = fmgc_.hdgTrkPresetAvailable;
  a.alt_display_value = alt;
  a.lvl_ch_managed = fmgc_.lvlChManaged;
  a.lvl_ch_vs_fpa = fmgc_.lvlChVsFpa;
  a.vs_fpa_display_value = vs.value;
  a.vs_fpa_dashes = vs.dashes;
  a.exped_active = fmgc_.expedActive;
  a.loc_only_active = fmgc_.locOnlyActive;
  a.appr_active = fmgc_.apprActive;
  a.hdg_trk_buttons = pulses.hdgTrk;
  a.spd_mach_buttons = pulses.spdMach;
  a.alt_buttons = pulses.alt;
  a.vs_fpa_buttons = pulses.vsFpa;
  a.loc_pushed = pulses.loc;
  a.exped_pushed = pulses.exped;
  a.appr_pushed = pulses.appr;
  a.spd_mach_switching_pushed = pulses.spdMachSwitch;

  // 1061-1078: value change pulses, in the generated order
  const double dt = in.time.dt;
  a.lat_value_changed = changes_[kHdgTrk].update(hdg.value, dt);
  a.alt_value_changed = changes_[kAlt].update(alt, dt);
  a.vpath_value_changed = changes_[kVsFpa].update(vs.value, dt);
  a.spd_mach_value_changed = changes_[kSpdMach].update(spd.value, dt);
  return a;
}

}  // namespace fcu::readable
