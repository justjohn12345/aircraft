// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// N1 command and sim throttle (generated FadecComputer.cpp 247-323, 423-428).
//
// The FADEC does not run an engine: it chooses an N1 command (N1_c) and moves the sim's throttle lever
// (sim_throttle_lever_pos, -100..100 %) until the sim's commanded N1 matches it, through two integrators.

#pragma once

#include "FadecComputer_types.h"
#include "athr_interface.h"
#include "fadec_blocks.h"

namespace fadec::readable {

/** N1 error beyond which the trim integrator is reset (%). */
constexpr double kTrimResetError = 0.8;
/** Trim integrator: 0.3 /s on (N1_c - N1 actual), limited to +-2 % N1. */
constexpr double kTrimGain = 0.3, kTrimLimit = 2.0;
/** Throttle integrators: 5 /s (forward) and -5 /s (reverse) on (N1_c + trim - commanded N1). */
constexpr double kThrottleGain = 5.0, kReverseThrottleGain = -5.0, kThrottleLimit = 100.0;

class N1Control {
 public:
  void reset();

  /**
   * N1 command (247-281):
   * - A/THR in control (lever <= 35 deg or alpha floor, ATS bit 14 active, engaged, control active): the FMGC's N1
   *   command limited to [IDLE limit, N1 for the lever], or to [N1 for the lever, TOGA limit] in alpha floor in flight
   *   (needs ATS bits 13 and 23). The limit that applies is rounded to float, like the command word.
   * - else thrust lock: the actual N1 frozen at the lock (pU is the actual N1 of every frame without lock).
   * - else the N1 for the lever.
   */
  double command(const athr_in& in, const AtsBits& bits, const AthrInterface& athr, double n1ForLever);

  /**
   * Throttle (282-323, 423-428): a trim integrator on the N1 error (reset to 0 when the error exceeds 0.8 %, which also
   * zeroes this frame's input), then the forward integrator (held at 0 in reverse) or the reverse one (held at 0
   * outside reverse) on N1_c + trim - the sim's commanded N1. Returns sim_throttle_lever_pos.
   */
  double throttle(const athr_in& in, double n1Command, bool reverse);

  // state access for the checker
  double frozenN1() const { return frozenN1_; }
  bool frozenN1Primed() const { return frozenN1Primed_; }
  double trim() const { return trim_.state; }
  double forward() const { return forward_.state; }
  double reverse() const { return reverse_.state; }

 private:
  double frozenN1_ = 0.0;        // pU
  bool frozenN1Primed_ = false;  // pU_not_empty
  blocks::VariableTsIntegrator trim_;     // Delay_DSTATE
  blocks::VariableTsIntegrator forward_;  // Delay_DSTATE_n
  blocks::VariableTsIntegrator reverse_;  // Delay_DSTATE_l
};

}  // namespace fadec::readable
