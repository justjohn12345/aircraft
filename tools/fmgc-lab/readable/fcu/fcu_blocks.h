// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Library blocks of the generated FCU (FcuComputer.cpp 6-310) that the FMGC block library (readable/blocks.h) does not
// already have. The FCU's A429ValueOrDefault, BitfromLabel, PulseNode, MTrigNode and bit packing are the same code as
// the FMGC's and come from fmgc::blocks:
//
//   FcuComputer_MATLABFunction     MTrigNode           -> fmgc::blocks::Monostable
//   FcuComputer_MATLABFunction_i   A429ValueOrDefault  -> fmgc::blocks::a429::valueOr
//   FcuComputer_MATLABFunction_d   PulseNode           -> fmgc::blocks::EdgeDetector
//   FcuComputer_MATLABFunction_m   SSM == NO           -> fmgc::blocks::a429::isNormal
//   FcuComputer_MATLABFunction_a   BitfromLabel        -> fmgc::blocks::a429::bit (1-based ARINC bit)
//   FcuComputer_MATLABFunction_e   19 bits -> 11..29   -> fmgc::blocks::packBits
//   FcuComputer_MATLABFunction1    display dash timer  -> DashTimer (below)
//   FcuComputer_MATLABFunction_b   T flip-flop         -> ToggleFlipFlop (below)
//
// The FCU-specific panel blocks (EFIS filter logic, baro reference and value, FD latch, baro display) live in
// efis_panel.cpp.

#pragma once

#include "blocks.h"

namespace fcu::readable::blocks {

using fmgc::blocks::EdgeDetector;
using fmgc::blocks::Monostable;
using fmgc::blocks::packBits;
namespace a429 = fmgc::blocks::a429;

/**
 * Window dash timer (generated FcuComputer_MATLABFunction1, 44-59, with 45 s; the SPD/MACH window has the same logic
 * inline at 740-753 with 10 s). The window shows dashes once `timeout` seconds have passed since the last "selection"
 * event; a "dashes" event dashes it at once (by moving the event time back by timeout + 1 s). Dashes wins when both
 * happen in the same frame. On the first call the event time is the current time (no dashes).
 * Time is the simulation time (s), not dt: the timer stands still while the sim is paused.
 */
struct DashTimer {
  double eventTime = 0.0;
  bool primed = false;

  void reset() { primed = false; }

  bool update(double simulationTime, bool setDashes, bool setSelection, double timeout) {
    if (!primed) {
      eventTime = simulationTime;
      primed = true;
    }
    if (setDashes) {
      eventTime = (simulationTime - timeout) - 1.0;
    } else if (setSelection) {
      eventTime = simulationTime;
    }
    return simulationTime - eventTime > timeout;
  }
};

/** T flip-flop (generated FcuComputer_MATLABFunction_b, 249-262): toggles on every true input; starts at `init`. */
struct ToggleFlipFlop {
  bool q = false;
  bool primed = false;

  void reset() { primed = false; }

  bool update(bool toggle, bool init) {
    if (!primed) {
      q = init;
      primed = true;
    }
    if (toggle) {
      q = !q;
    }
    return q;
  }
};

}  // namespace fcu::readable::blocks
