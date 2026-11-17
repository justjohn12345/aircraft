// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable FADEC: a rewrite of the generated A32NX FadecComputer (fbw-a32nx/src/wasm/fbw_a320/src/model/
// FadecComputer.cpp, 476 lines) that reproduces it bit for bit, frame by frame (checked free-running by
// src/check_fadec.cpp). Stock behaviour, quirks included; fixes never go here. Spec and findings: docs/fadec.md.
//
// One instance per engine (FlyByWireInterface::updateFadec, after the FMGCs, the FCU bus of this frame). Interface as
// the generated model's: initialize() once, then step() every frame. There is no power or restart logic.
//
// Pipeline of one frame (generated order):
//   athr_interface   A/THR disabled (15 s disconnect), engaged, control active        114-124, 138-145
//   thrust_lever     FLX selection, time on ground, active range                      125-137
//   athr_interface   thrust lock                                                      146-150
//   thrust_lever     N1 for the lever, thrust limit                                   151-246
//   n1_control       N1 command, trim and throttle integrators                        247-323
//   output_packing   ECU bus                                                          325-414
//   fadec.cpp        out.data_computed, out.output                                    415-452
//
// No heap allocation, no static mutable state: copyable, several instances can run side by side.

#pragma once

#include "FadecComputer_types.h"
#include "athr_interface.h"
#include "fadec_blocks.h"
#include "n1_control.h"
#include "thrust_lever.h"

namespace fadec::readable {

class FadecComputer {
 public:
  /**
   * Power-up: every memory to its initial value, outputs to zero. A complete reset, unlike the generated initialize(),
   * which leaves the "not yet initialised" flags of the timers, edge detectors and pU of a used instance set.
   */
  void initialize();

  /** One frame. Returns the outputs (FadecComputer_Y.out). */
  const athr_out& step(const athr_in& in);

  const athr_out& outputs() const { return out_; }

  // state access for the checker
  const AthrInterface& athr() const { return athr_; }
  const FlexSelection& flex() const { return flex_; }
  const N1Control& n1() const { return n1_; }
  const blocks::TimeSinceCondition& onGroundTimer() const { return onGround_; }

 private:
  AthrInterface athr_;
  FlexSelection flex_;
  blocks::TimeSinceCondition onGround_;  // sf_TimeSinceCondition1
  N1Control n1_;
  athr_out out_{};
};

}  // namespace fadec::readable
