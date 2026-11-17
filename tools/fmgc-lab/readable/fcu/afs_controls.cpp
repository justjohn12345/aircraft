// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "afs_controls.h"

namespace fcu::readable {

namespace {
bool pulse(blocks::Monostable& m, bool u, double dt) {
  return m.update(u, dt, /*risingEdge*/ true, /*retriggerable*/ false, kButtonPulseSeconds);
}
}  // namespace

void AfsControls::reset() {
  hdgTrk_.reset();
  spdMach_.reset();
  alt_.reset();
  vsFpa_.reset();
  loc_.reset();
  exped_.reset();
  appr_.reset();
  spdMachPb_.reset();
  trkFpaPb_.reset();
  metricPb_.reset();
  trkFpaActive_ = false;
  metricAlt_ = false;
}

base_fcu_knob_inputs AfsControls::KnobPulses::update(const base_fcu_knob_inputs& knob, double dt) {
  base_fcu_knob_inputs out{};
  out.pushed = pulse(pushed, knob.pushed, dt);
  out.pulled = pulse(pulled, knob.pulled, dt);
  out.turns = static_cast<int8_T>(pulse(turned, knob.turns != 0, dt));
  return out;
}

const AfsControls::Pulses& AfsControls::update(const base_fcu_afs_panel_inputs& afs, double dt, const FmgcView& fmgc) {
  // 481-548, in the generated order (the order does not matter: every block has its own memory)
  pulses_.hdgTrk = hdgTrk_.update(afs.hdg_trk_knob, dt);
  pulses_.spdMach = spdMach_.update(afs.spd_knob, dt);
  pulses_.alt = alt_.update(afs.alt_knob, dt);
  pulses_.vsFpa = vsFpa_.update(afs.vs_fpa_knob, dt);
  pulses_.loc = pulse(loc_, afs.loc_button_pressed, dt);
  pulses_.exped = pulse(exped_, afs.exped_button_pressed, dt);
  pulses_.appr = pulse(appr_, afs.appr_button_pressed, dt);
  pulses_.spdMachSwitch = pulse(spdMachPb_, afs.spd_mach_button_pressed, dt);

  // 552-566: TRK/FPA latch
  const bool trkFpaPushed = trkFpaPb_.update(afs.trk_fpa_button_pressed, /*risingEdge*/ true);
  if (fmgc.trkFpaReset) {
    trkFpaActive_ = false;
  } else if (trkFpaPushed) {
    trkFpaActive_ = !trkFpaActive_;
  }

  // 568-572: METRIC ALT latch
  if (metricPb_.update(afs.metric_alt_button_pressed, /*risingEdge*/ true)) {
    metricAlt_ = !metricAlt_;
  }
  return pulses_;
}

}  // namespace fcu::readable
