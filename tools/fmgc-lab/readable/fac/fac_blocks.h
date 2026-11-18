// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Library blocks of the generated FAC (FacComputer.cpp 19-275). Most are the FMGC's (readable/blocks.h), the same
// Simulink library compiled the same way:
//
//   FacComputer_MATLABFunction          SSM != FW            -> a429::isNotFailed
//   FacComputer_MATLABFunction_l        SSM == NO            -> a429::isNormal
//   FacComputer_MATLABFunction_g        BitfromLabel         -> a429::bit (1-based ARINC bit)
//   FacComputer_MATLABFunction_i        A429 raw value       -> a429::rawValue
//   FacComputer_MATLABFunction_n        ConfirmNode          -> ConfirmNode
//   FacComputer_MATLABFunction_a        MTrigNode            -> Monostable
//   inline previousInput logic          PulseNode            -> EdgeDetector
//   FacComputer_MATLABFunction_g3       19 bits -> 11..29    -> packBits
//   Logic_table lookup                  SRFlipFlop           -> SrLatch (reset wins)
//   FacComputer_LagFilter / _k          LagFilter<double> / LagFilter<float>
//   FacComputer_WashoutFilter           WashoutFilter
//   FacComputer_RateLimiter             RateLimiter (below: starts at its first input)
//   FacComputer_RateLimiter_g / _p      RateLimiterFrom (below: starts at a given value, and resets to it)
//   look1_binlxpw, look2_(p)binlxpw     lookup1, lookup2 (below)

#pragma once

#include <cmath>
#include <cstdint>

#include "FacComputer_types.h"
#include "blocks.h"

namespace fac::readable::blocks {

using fmgc::blocks::ConfirmNode;
using fmgc::blocks::EdgeDetector;
using fmgc::blocks::LagFilter;
using fmgc::blocks::Monostable;
using fmgc::blocks::packBits;
using fmgc::blocks::SrLatch;
using fmgc::blocks::WashoutFilter;
namespace a429 = fmgc::blocks::a429;

/**
 * RateLimiterGenericVariableTs (79-100): the output follows the input at no more than |up| per second upwards and |lo|
 * per second downwards. Starts at its first input; `reset` makes it the input (no FAC instance resets it).
 */
struct RateLimiter {
  double pY = 0.0;
  bool primed = false;

  void reset() { primed = false; }

  double update(double u, double up, double lo, double dt, bool resetToInput = false) {
    if (!primed || resetToInput) {
      pY = u;
      primed = true;
    }
    const double y = resetToInput ? u : std::fmax(std::fmin(u - pY, std::abs(up) * dt), -std::abs(lo) * dt) + pY;
    pY = y;
    return y;
  }
};

/**
 * The same rate limiter with an initial value (220-264): starts at `init` on its first frame and while `holdAtInit`
 * is true (then the output is `init`).
 */
struct RateLimiterFrom {
  double pY = 0.0;
  bool primed = false;

  void reset() { primed = false; }

  double update(double u, double up, double lo, double dt, double init, bool holdAtInit) {
    if (!primed || holdAtInit) {
      pY = init;
      primed = true;
    }
    const double y = holdAtInit ? init : std::fmax(std::fmin(u - pY, std::abs(up) * dt), -std::abs(lo) * dt) + pY;
    pY = y;
    return y;
  }
};

/** Breakpoint search of look1_binlxpw: the interval [bp[i], bp[i+1]) holding u, extrapolating the end intervals. */
inline void findInterval(double u, const double* bp, uint32_t maxIndex, uint32_t& left, double& frac) {
  if (u <= bp[0]) {
    left = 0;
    frac = (u - bp[0]) / (bp[1] - bp[0]);
  } else if (u < bp[maxIndex]) {
    uint32_t i = maxIndex >> 1U, lo = 0, hi = maxIndex;
    while (hi - lo > 1U) {
      if (u < bp[i]) {
        hi = i;
      } else {
        lo = i;
      }
      i = (hi + lo) >> 1U;
    }
    left = lo;
    frac = (u - bp[lo]) / (bp[lo + 1U] - bp[lo]);
  } else {
    left = maxIndex - 1U;
    frac = (u - bp[maxIndex - 1U]) / (bp[maxIndex] - bp[maxIndex - 1U]);
  }
}

/** 1-D table, linear interpolation and extrapolation (look1_binlxpw); `n` breakpoints. */
template <uint32_t n>
double lookup1(double u, const double (&bp)[n], const double (&table)[n]) {
  uint32_t i;
  double frac;
  findInterval(u, bp, n - 1U, i, frac);
  const double y0 = table[i];
  return (table[i + 1U] - y0) * frac + y0;
}

/**
 * 2-D table (look2_binlxpw; look2_pbinlxpw gives the same results: it only starts its search at the previous
 * interval). `table` is stored with u0 varying fastest (n0 values per u1 breakpoint).
 */
template <uint32_t n0, uint32_t n1>
double lookup2(double u0, double u1, const double (&bp0)[n0], const double (&bp1)[n1], const double (&table)[n0 * n1]) {
  uint32_t i0, i1;
  double f0, f1;
  findInterval(u0, bp0, n0 - 1U, i0, f0);
  findInterval(u1, bp1, n1 - 1U, i1, f1);
  uint32_t k = i1 * n0 + i0;
  double y0 = table[k];
  y0 += (table[k + 1U] - y0) * f0;
  k += n0;
  const double y1 = table[k];
  return (((table[k + 1U] - y1) * f0 + y1) - y0) * f1 + y0;
}

/** Saturation block: NaN passes through (both comparisons false). */
inline double limit(double u, double lower, double upper) {
  if (u > upper) return upper;
  if (u < lower) return lower;
  return u;
}

}  // namespace fac::readable::blocks
