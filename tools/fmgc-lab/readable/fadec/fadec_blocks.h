// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Library blocks of the generated FADEC (FadecComputer.cpp 6-91). The A429 helpers, the PulseNode and the bit packing
// are the FMGC's (readable/blocks.h):
//
//   FadecComputer_MATLABFunction        SSM == NO            -> fmgc::blocks::a429::isNormal
//   FadecComputer_MATLABFunction_p      A429ValueOrDefault   -> fmgc::blocks::a429::valueOr
//   FadecComputer_MATLABFunction_g      BitfromLabel         -> fmgc::blocks::a429::bit (1-based ARINC bit)
//   FadecComputer_MATLABFunction_l      19 bits -> 11..29    -> fmgc::blocks::packBits
//   FadecComputer_MATLABFunction_f      PulseNode            -> fmgc::blocks::EdgeDetector
//   FadecComputer_TimeSinceCondition    TimeSinceCondition   -> TimeSinceCondition (below)
//   Logic_table / _n / _h lookups       SRFlipFlop           -> SrFlipFlop (below)
//   DiscreteTimeIntegratorVariableTs    integrator           -> VariableTsIntegrator (below)

#pragma once

#include "FadecComputer_types.h"
#include "blocks.h"

namespace fadec::readable::blocks {

using fmgc::blocks::EdgeDetector;
using fmgc::blocks::packBits;
namespace a429 = fmgc::blocks::a429;

/**
 * TimeSinceCondition (20-33): seconds of simulation time the condition has been continuously true, 0 while false.
 * Quirk kept: an event time of exactly 0 counts as "not set", so a condition true since simulation time 0 starts
 * counting at the first frame with a non-zero time.
 */
struct TimeSinceCondition {
  double eventTime = 0.0;
  bool primed = false;

  void reset() {
    eventTime = 0.0;
    primed = false;
  }

  double update(double time, bool condition) {
    if (!primed) {
      eventTime = time;
      primed = true;
    }
    if (!condition || eventTime == 0.0) {
      eventTime = time;
    }
    return time - eventTime;
  }
};

/**
 * Simulink S-R flip-flop with the default truth table of the three Logic_table lookups: reset wins. Both outputs are
 * kept because the second one (`qbar`, Logic_table[index + 8]) is not !q when S and R are both true: both are false.
 */
struct SrFlipFlop {
  bool q = false;
  bool qbar = true;

  void reset() {
    q = false;
    qbar = true;
  }

  bool update(bool set, bool reset) {
    const bool next = !reset && (set || q);
    qbar = !set && (reset || !q);
    q = next;
    return q;
  }

  /** The generated lookup Logic_table[(((S << 1) + R) << 1) + Q] (+ 8 for the second output), to check a table. */
  static bool fromTable(const boolean_T table[16], bool set, bool reset, bool q, bool second) {
    return table[((((static_cast<unsigned>(set) << 1) + reset) << 1) + q) + (second ? 8U : 0U)];
  }
};

/**
 * DiscreteTimeIntegratorVariableTs, reset to its initial condition 0 by the caller: state += gain * u * dt, then
 * limited (NaN passes the limits). The generated product is Gain * u * Ts_Gain (1) * dt, in that order.
 */
struct VariableTsIntegrator {
  double state = 0.0;

  void reset() { state = 0.0; }

  double update(double u, double gain, double dt, double lower, double upper) {
    state += gain * u * 1.0 * dt;
    if (state > upper) {
      state = upper;
    } else if (state < lower) {
      state = lower;
    }
    return state;
  }
};

}  // namespace fadec::readable::blocks
