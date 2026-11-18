// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// AFS control panel inputs (generated FcuComputer.cpp 481-572): the knob and pushbutton pulses sent to the FMGCs, and
// the two panel latches the FCU owns, TRK/FPA (HDG-V/S vs TRK-FPA) and the metric altitude display.
//
// Every knob action (push, pull, turn) and the LOC, EXPED, APPR and SPD/MACH pushbuttons become a 0.1 s pulse
// (MTrigNode, rising edge, not retriggerable): the FMGC sees "pushed", "pulled", "turned" for ~2-3 frames per action.
// A knob turn only marks "turned" (logic.afs.*_buttons.turns is 0/1, not a count); the windows take the count from the
// raw input. The pulses go out on FCU discrete words 1 and 2 (output_packing.cpp).

#pragma once

#include "FcuComputer_types.h"
#include "fcu_blocks.h"
#include "fmgc_interface.h"

namespace fcu::readable {

/** Knob and pushbutton pulse width (KnobMtrigProcessing*_triggerDuration, MTrigNode*_triggerDuration). */
constexpr double kButtonPulseSeconds = 0.1;

class AfsControls {
 public:
  struct Pulses {
    base_fcu_knob_inputs hdgTrk{};  // logic.afs.hdg_trk_buttons
    base_fcu_knob_inputs spdMach{};  // logic.afs.spd_mach_buttons
    base_fcu_knob_inputs alt{};      // logic.afs.alt_buttons
    base_fcu_knob_inputs vsFpa{};    // logic.afs.vs_fpa_buttons
    bool loc = false;                // logic.afs.loc_pushed
    bool exped = false;              // logic.afs.exped_pushed
    bool appr = false;               // logic.afs.appr_pushed
    bool spdMachSwitch = false;      // logic.afs.spd_mach_switching_pushed (SPD/MACH pb)
  };

  void reset();

  /**
   * One frame: the pulses, then the latches. TRK/FPA toggles on the TRK/FPA pb (rising edge) and is forced back to
   * HDG-V/S while FMGC 1 reports SRS TO/GA or TCAS (FmgcView::trkFpaReset; the pb is ignored then). METRIC ALT toggles
   * on its pb (rising edge), with no other reset.
   */
  const Pulses& update(const base_fcu_afs_panel_inputs& afs, double dt, const FmgcView& fmgc);

  const Pulses& pulses() const { return pulses_; }
  bool trkFpaActive() const { return trkFpaActive_; }    // p_trk_fpa_active
  bool metricAltActive() const { return metricAlt_; }    // p_metric_alt_active

 private:
  struct KnobPulses {
    blocks::Monostable pushed, pulled, turned;
    void reset() {
      pushed.reset();
      pulled.reset();
      turned.reset();
    }
    base_fcu_knob_inputs update(const base_fcu_knob_inputs& knob, double dt);
  };
  KnobPulses hdgTrk_, spdMach_, alt_, vsFpa_;
  blocks::Monostable loc_, exped_, appr_, spdMachPb_;
  blocks::EdgeDetector trkFpaPb_, metricPb_;
  bool trkFpaActive_ = false;
  bool metricAlt_ = false;
  Pulses pulses_;
};

}  // namespace fcu::readable
