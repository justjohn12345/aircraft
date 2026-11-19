// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Library blocks of the generated EFCS computers (ElacComputer, SecComputer) and of their control laws
// (PitchNormalLaw, PitchAlternateLaw, PitchDirectLaw, LateralNormalLaw, LateralDirectLaw). They are the FMGC's and the
// FAC's (readable/blocks.h, readable/fac/fac_blocks.h): the same Simulink library compiled the same way. Generated
// names differ per model (ElacComputer_LagFilter, PitchNormalLaw_LagFilter_n, ...); each readable file says which
// generated function an instance replaces.
//
//   SSM != FailureWarning                 a429::isNotFailed       SSM == NormalOperation     a429::isNormal
//   BitfromLabel (1-based ARINC bit)      a429::bit               ConfirmNode                ConfirmNode
//   PulseNode                             EdgeDetector            HysteresisNode             Hysteresis (below)
//   SRFlipFlop (Logic_table)              SrLatch                 19 bits -> 11..29          packBits
//   LagFilter / WashoutFilter             LagFilter, WashoutFilter
//   RateLimiterVariableTs (initial value) RateLimiterInit (below)
//   RateLimiterGenericVariableTs          RateLimiter (starts at its input), RateLimiterFrom (starts at / resets to init)
//   look1_binlxpw, look2_binlxpw          lookup1, lookup2        Saturation                 limit

#pragma once

#include <cmath>
#include <cstdint>

#include "ElacComputer_types.h"
#include "blocks.h"
#include "fac_blocks.h"

namespace efcs::blocks {

using fac::readable::blocks::limit;
using fac::readable::blocks::lookup1;
using fac::readable::blocks::lookup2;
using fac::readable::blocks::RateLimiter;
using fac::readable::blocks::RateLimiterFrom;
using fmgc::blocks::ConfirmNode;
using fmgc::blocks::EdgeDetector;
using fmgc::blocks::LagFilter;
using fmgc::blocks::packBits;
using fmgc::blocks::SrLatch;
using fmgc::blocks::WashoutFilter;
namespace a429 = fmgc::blocks::a429;

/**
 * RateLimiterVariableTs: starts at a constant initial value on its first frame, then follows the input at no more than
 * |up| per second upwards and |lo| per second downwards (`pY += fmax(fmin(u - pY, |up| dt), -|lo| dt)`).
 */
struct RateLimiterInit {
  double pY = 0.0;
  bool primed = false;

  void reset() { primed = false; }

  double update(double u, double up, double lo, double dt, double init) {
    if (!primed) {
      pY = init;
      primed = true;
    }
    pY += std::fmax(std::fmin(u - pY, std::abs(up) * dt), -std::abs(lo) * dt);
    return pY;
  }
};

/**
 * HysteresisNode: goes true when the input reaches `high` (>=), back to false when it drops to `low` or below.
 * Generated: `out = (!out && u >= high) || ((!out || u > low) && out)`.
 */
struct Hysteresis {
  bool output = false;

  void reset() { output = false; }

  bool update(double u, double high, double low) {
    const bool wasOff = !output;
    output = (wasOff && u >= high) || ((wasOff || u > low) && output);
    return output;
  }
};

}  // namespace efcs::blocks
