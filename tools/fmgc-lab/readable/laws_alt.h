// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ALT_ACQ (ALT*) and ALT_HOLD laws (laws_alt.cpp): block memories.

#pragma once

#include "FmgcOuterLoops_types.h"
#include "blocks.h"

namespace fmgc::readable::laws {

/**
 * Speed-on-pitch law used for the VLS and VMAX speed protection of the path laws (FmgcOuterLoops: one copy per law and
 * protection, all with the same gains): pitch change (deg) that holds the speed rate at zero plus a limited
 * correction of the speed error to a reference speed. See laws_alt.cpp.
 */
struct SpeedOnPitchLaw {
  blocks::WashoutFilter groundAccel;       // ground speed derivative
  blocks::LeadLagFilter inertialHighPass;  // ground acceleration minus turn correction, high passed (complementary)
  blocks::LeadLagFilter iasRate;           // IAS derivative, low passed (complementary)

  void reset() {
    groundAccel.reset();
    inertialHighPass.reset();
    iasRate.reset();
  }

  /** Pitch command (delta theta, deg) for flying at speedRefKn. */
  double update(const ap_laws_output& in, double speedRefKn);
};

struct AltLawState {
  // ALT* capture law (embedded MATLAB function, persistent variables)
  bool captureWasActive = false;   // law was ALT_ACQ last frame
  bool captureWasActivePrimed = false;
  double captureGain = 5.0;        // k, 1/min
  double captureOffsetFt = 0.0;    // dH_offset
  double captureMaxHdot = 1500.0;  // maxH_dot, fpm: |V/S| at entry
  SpeedOnPitchLaw altAcqVls;
  SpeedOnPitchLaw altAcqVmax;

  // ALT hold law
  double holdTargetH = 0.0;  // target in the H_ft frame, frozen while ALT_HOLD; set to 0 at power-up only
  blocks::LagFilter<double> holdErrorLag;
};

}  // namespace fmgc::readable::laws
