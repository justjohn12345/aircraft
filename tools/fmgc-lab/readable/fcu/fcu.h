// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable FCU computer: a rewrite of the generated A32NX FcuComputer (fbw-a32nx/src/wasm/fbw_a320/src/model/
// FcuComputer.cpp, 1,570 lines) that reproduces it bit for bit, frame by frame (checked free-running by
// src/check_fcu.cpp). Stock behaviour, quirks included; fixes never go here. Spec, inputs/outputs and findings:
// docs/fcu.md.
//
// Interface as the generated model's: initialize() once at power-up, then step() every frame with the fcu_inputs the
// sim wrapper builds (FlyByWireInterface::updateFcu: time, sim_data.computer_running from the FCU self-monitoring,
// sim_input values set by events (-1 = none), the AP/FD/A/THR engaged discretes of both FMGCs, both EFIS panels, the
// AFS panel, and both FMGC A buses, all of the previous frame since the FCU runs before the FMGCs).
// When computer_running is false the outputs are held and nothing else changes; the first running frame after that
// (or after initialize()) restarts the FCU: every memory is reset and the windows re-read the aircraft's values.
//
// Pipeline of one running frame (generated order):
//   fmgc_interface   FMGC priority, chosen data, decoded FMGC modes            444-480, 549-737
//   afs_controls     knob/pb pulses, TRK/FPA and METRIC ALT latches            481-572
//   windows          SPD/MACH 738-829, HDG/TRK 861-933, ALT 934-954, V/S-FPA 955-1060
//   value changes    0.5 s pulse when a window memory changes                  1061-1078
//   efis_panel       FD auto on/off (FMGC 1), captain then F/O panel           1079-1183
//   output_packing   lights, window values, FCU bus                            1184-1537
//
// No heap allocation, no static mutable state: copyable, several instances can run side by side.

#pragma once

#include "FcuComputer_types.h"
#include "afs_controls.h"
#include "efis_panel.h"
#include "fcu_blocks.h"
#include "fmgc_interface.h"
#include "windows.h"

namespace fcu::readable {

/** Pulse width of the "value changed" bits on FCU discrete words 1 and 2 (MTrigNode*_triggerDuration_*, s). */
constexpr double kValueChangePulseSeconds = 0.5;

/**
 * DetectChange + MTrigNode (1061-1078, 1534-1537): a 0.5 s pulse (rising edge, not retriggerable) when a window memory
 * differs from its value in the previous running frame. Any change counts, including the dashed SPD/MACH and HDG/TRK
 * memories following the aircraft (C13). The previous values are 0 after a (re)start (DetectChange_vinit), so the first
 * frame pulses for every non-zero window.
 */
struct ValueChangeDetector {
  float previous = 0.0F;  // DelayInput1_DSTATE[i]
  blocks::Monostable pulse;

  void reset() {
    previous = 0.0F;
    pulse.reset();
  }
  bool update(float value, double dt) {
    return pulse.update(value != previous, dt, /*risingEdge*/ true, /*retriggerable*/ false, kValueChangePulseSeconds);
  }
};

class FcuComputer {
 public:
  /** Power-up: every memory to its initial value, outputs to zero (generated initialize()). */
  void initialize();

  /**
   * One frame. Returns the outputs (FcuComputer_Y.out): this frame's when computer_running, else the last running
   * frame's (zero before the first one).
   */
  const fcu_outputs& step(const fcu_inputs& in);

  const fcu_outputs& outputs() const { return out_; }
  /** True after a running frame (generated Runtime_MODE): the next running frame continues without a restart. */
  bool running() const { return running_; }

  // state access for the checker
  const FmgcView& fmgcView() const { return fmgc_; }
  const AfsControls& afsControls() const { return afs_; }
  const SpeedMachWindow& speedWindow() const { return speed_; }
  const HdgTrkWindow& headingWindow() const { return heading_; }
  const AltWindow& altitudeWindow() const { return altitude_; }
  const VsFpaWindow& vsFpaWindow() const { return vsFpa_; }
  const EfisControlPanel& captEfis() const { return captEfis_; }
  const EfisControlPanel& foEfis() const { return foEfis_; }
  /** Previous-frame window memories of the value change detectors: HDG/TRK, ALT, V/S-FPA, SPD/MACH. */
  float valueChangePrevious(int i) const { return changes_[i].previous; }

 private:
  void restart();
  base_afs_logic_outputs afsLogic(const fcu_inputs& in);

  bool running_ = false;
  FmgcView fmgc_;
  AfsControls afs_;
  SpeedMachWindow speed_;
  HdgTrkWindow heading_;
  AltWindow altitude_;
  VsFpaWindow vsFpa_;
  enum { kHdgTrk = 0, kAlt, kVsFpa, kSpdMach };
  ValueChangeDetector changes_[4];
  blocks::EdgeDetector fdAutoActivate_, fdAutoDeactivate_;  // sf_MATLABFunction_kj, _kq
  EfisControlPanel captEfis_, foEfis_;
  fcu_outputs out_{};
};

}  // namespace fcu::readable
