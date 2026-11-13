// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable versions of the FBW Simulink library blocks used by the generated FMGC (FmgcComputer, FmgcOuterLoops).
// Each one reproduces the generated arithmetic in the same order, so results are bit-identical; blocks_test.cpp checks
// that against the generated functions with random inputs.
//
// Generated name -> block:
//   FmgcComputer_MATLABFunction        a429::valueOr          FmgcComputer_MATLABFunction_a   ConfirmNode
//   FmgcComputer_MATLABFunction_i      a429::bit              FmgcComputer_MATLABFunction_g   EdgeDetector (PulseNode)
//   FmgcComputer_MATLABFunction_ie     a429::isNormal         FmgcComputer_MATLABFunction_m3  Monostable (MTrigNode)
//   FmgcComputer_MATLABFunction_m      a429::isNotFailed      FmgcComputer_MATLABFunction_gy  packBits
//   FmgcComputer_MATLABFunction_c      a429::rawValue         Logic_table[...] lookups        SrLatch (SRFlipFlop)
//   *_LagFilter / _LeadLagFilter / _WashoutFilter             LagFilter, LeadLagFilter, WashoutFilter
//   FmgcOuterLoops_RateLimiter(_g)     SwitchBlend            FmgcOuterLoops_Chart(_k)        TurnDirectionChart
//   FmgcOuterLoops_MATLABFunction      navGains               FmgcOuterLoops_MATLABFunction_g HeadingBankLimit
//   FmgcOuterLoops_Voter1              median3                FmgcOuterLoops_VSLimiter        vsLimit
//   FmgcOuterLoops_SpeedProtectionSignalSelection / _V_LSSpeedSelection1   speedProtection, vlsWithMargin

#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>

#include "FmgcComputer_types.h"

namespace fmgc::blocks {

// ----------------------------------------------------------------------------------------------------------------------
// ARINC 429 words
// ----------------------------------------------------------------------------------------------------------------------
namespace a429 {

inline bool isNormal(const base_arinc_429& w) {
  return w.SSM == static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
}

/** True unless the word reports a failure warning (NCD and functional test count as not failed). */
inline bool isNotFailed(const base_arinc_429& w) {
  return w.SSM != static_cast<uint32_t>(SignStatusMatrix::FailureWarning);
}

/** The word's value when its SSM is normal operation, else the default (block A429ValueOrDefault). */
inline float valueOr(const base_arinc_429& w, float defaultValue) {
  return isNormal(w) ? w.Data : defaultValue;
}

/** The value regardless of SSM. */
inline float rawValue(const base_arinc_429& w) {
  return w.Data;
}

/** Bit n (1-based) of a discrete word, ignoring SSM (block BitfromLabel). The value is rounded and saturated to uint32. */
inline uint32_t bit(const base_arinc_429& w, double n) {
  float r = std::round(w.Data);
  uint32_t word = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  double shiftLeft = -(n - 1.0);
  if (shiftLeft >= 0.0) {
    word = shiftLeft <= 31.0 ? word << static_cast<uint8_t>(shiftLeft) : 0U;
  } else {
    word = shiftLeft >= -31.0 ? word >> static_cast<uint8_t>(n - 1.0) : 0U;
  }
  return word & 1U;
}

}  // namespace a429

// ----------------------------------------------------------------------------------------------------------------------
// Discrete logic
// ----------------------------------------------------------------------------------------------------------------------

/**
 * ConfirmNode: when risingEdge, the output goes true only after the input has been true for `delay` seconds and goes
 * false immediately; when !risingEdge, the opposite (goes false only after `delay` seconds of false input).
 */
struct ConfirmNode {
  bool output = false;
  double timeSinceCondition = 0.0;

  void reset() {
    output = false;
    timeSinceCondition = 0.0;
  }

  bool update(bool u, double dt, bool risingEdge, double delay) {
    if (u == risingEdge) {
      timeSinceCondition += dt;
      if (timeSinceCondition >= delay) {
        output = u;
      }
    } else {
      timeSinceCondition = 0.0;
      output = u;
    }
    return output;
  }
};

/**
 * PulseNode: one-frame pulse on a rising (or falling) edge. On the first call the previous input is taken to be
 * `risingEdge`, so a rising-edge detector does not fire when the input is already true at start-up, and a falling-edge
 * detector does not fire either.
 */
struct EdgeDetector {
  bool previousInput = false;
  bool primed = false;

  void reset() { primed = false; }

  bool update(bool u, bool risingEdge) {
    if (!primed) {
      previousInput = risingEdge;
      primed = true;
    }
    bool y = risingEdge ? (u && !previousInput) : (!u && previousInput);
    previousInput = u;
    return y;
  }
};

/**
 * MTrigNode: true for `duration` seconds after an edge. When not retriggerable, edges during the pulse are ignored.
 * The remaining time is decremented before the edge check, so the pulse lasts ceil(duration/dt) frames.
 */
struct Monostable {
  bool previousInput = false;
  double remainingTriggerTime = 0.0;

  void reset() {
    previousInput = false;
    remainingTriggerTime = 0.0;
  }

  bool update(bool u, double dt, bool risingEdge, bool retriggerable, double duration) {
    if (remainingTriggerTime > 0.0) {
      remainingTriggerTime -= dt;
    } else if (remainingTriggerTime < 0.0) {
      remainingTriggerTime = 0.0;
    }
    bool edge = risingEdge ? (u && !previousInput) : (!u && previousInput);
    if ((retriggerable || remainingTriggerTime == 0.0) && edge) {
      remainingTriggerTime = duration;
    }
    previousInput = u;
    return remainingTriggerTime > 0.0;
  }
};

/**
 * SRFlipFlop. The generated code looks the next state up in the block's truth table,
 * `Logic_table[((S << 1) + R) << 1) + Q]`; every table in the FMGC is the Simulink default: hold, set, reset, and
 * reset wins when both are true. `fromTable` reproduces the lookup for checking a table.
 */
struct SrLatch {
  bool q = false;

  bool update(bool set, bool reset) {
    q = reset ? false : (set ? true : q);
    return q;
  }

  static bool fromTable(const bool table[16], bool set, bool reset, bool q) {
    return table[(((static_cast<uint32_t>(set) << 1) + reset) << 1) + q];
  }
};

/** Packs 19 discretes into bits 10..28 of an ARINC discrete word value. */
inline float packBits(const bool u[19]) {
  uint32_t out = 0U;
  for (int i = 0; i < 19; i++) {
    out |= static_cast<uint32_t>(u[i]) << (i + 10);
  }
  return static_cast<float>(out);
}

// ----------------------------------------------------------------------------------------------------------------------
// Filters (Tustin discretisation, variable time step). All initialise to the first input (output = input on frame 1).
// ----------------------------------------------------------------------------------------------------------------------

/** First-order lag 1/(1 + s/C1) (C1 in rad/s). */
template <typename T = double>
struct LagFilter {
  T pY{}, pU{};
  bool primed = false;

  void reset() { primed = false; }

  T update(T u, double c1, double dt) {
    if (!primed) {
      pU = u;
      pY = u;
      primed = true;
    }
    double d = dt * c1;
    double ca = d / (d + 2.0);
    T y;
    if constexpr (std::is_same_v<T, float>) {
      // FmgcComputer's float variant: coefficients rounded to float, arithmetic in float
      y = static_cast<float>((2.0 - d) / (d + 2.0)) * pY + (u * static_cast<float>(ca) + pU * static_cast<float>(ca));
    } else {
      y = (2.0 - d) / (d + 2.0) * pY + (u * ca + pU * ca);
    }
    pY = y;
    pU = u;
    return y;
  }
};

/** Washout (high pass) (s/C1)/(1 + s/C1). Note that the first output is not 0 but (2-d)/(d+2)·u. */
struct WashoutFilter {
  double pY = 0, pU = 0;
  bool primed = false;

  void reset() { primed = false; }

  double update(double u, double c1, double dt) {
    if (!primed) {
      pU = u;
      pY = u;
      primed = true;
    }
    double d = dt * c1;
    double ca = 2.0 / (d + 2.0);
    double y = (2.0 - d) / (d + 2.0) * pY + (u * ca - pU * ca);
    pY = y;
    pU = u;
    return y;
  }
};

/** Lead-lag (C1·s + C2)/(C3·s + C4). */
struct LeadLagFilter {
  double pY = 0, pU = 0;
  bool primed = false;

  void reset() { primed = false; }

  double update(double u, double c1, double c2, double c3, double c4, double dt) {
    if (!primed) {
      pU = u;
      pY = u;
      primed = true;
    }
    double denomA = 2.0 * c3;
    double denomB = dt * c4;
    double denom = denomA + denomB;
    double numB = dt * c2;
    double numA = 2.0 * c1;
    double y = ((numA + numB) / denom * u + (numB - numA) / denom * pU) + (denomA - denomB) / denom * pY;
    pY = y;
    pU = u;
    return y;
  }
};

/**
 * Rate limiter on a boolean (RateLimiterVariableTs fed by a switch condition): ramps from 0 to 1 at |up|/s and back at
 * |lo|/s. Used to blend between two commands when a law or mode switches.
 */
struct SwitchBlend {
  double pY = 0;
  bool primed = false;

  void reset() { primed = false; }

  double update(bool u, double up, double lo, double dt, double init) {
    if (!primed) {
      pY = init;
      primed = true;
    }
    pY += std::fmax(std::fmin(static_cast<double>(u) - pY, std::abs(up) * dt), -std::abs(lo) * dt);
    return pY;
  }
};

// ----------------------------------------------------------------------------------------------------------------------
// Lateral helpers (FmgcOuterLoops)
// ----------------------------------------------------------------------------------------------------------------------

/**
 * Turn direction for heading/track select (Stateflow chart): normally the shorter way. Unless use_short_path, when the
 * shorter turn is between 10 and 20 degrees it latches that direction, and only goes back to "shortest" once either
 * turn is below 10 degrees (or use_short_path). Prevents flip-flopping when the target is near the reciprocal.
 */
struct TurnDirectionChart {
  enum class State { Inactive, Any, Left, Right };
  State state = State::Inactive;

  void reset() { state = State::Inactive; }

  double update(double right, double left, bool useShortPath) {
    if (state == State::Inactive) {
      state = State::Any;
      return std::abs(left) < std::abs(right) ? left : right;
    }
    if (state == State::Any) {
      double r = std::abs(right);
      double l = std::abs(left);
      if (!useShortPath && r < l && r >= 10.0 && r <= 20.0) {
        state = State::Right;
        return right;
      }
      if (!useShortPath && l < r && l >= 10.0 && l <= 20.0) {
        state = State::Left;
        return left;
      }
      return l < r ? left : right;
    }
    double l = std::abs(left);
    double r = std::abs(right);
    if (useShortPath || r < 10.0 || l < 10.0) {
      state = State::Any;
      return l < r ? left : right;
    }
    return state == State::Left ? left : right;
  }
};

/** NAV law gains from a second-order response time constant tau (s) and damping zeta. Returns {k1 (xtk), k2 (tke)}. */
struct NavGains {
  double k1, k2;
};
inline NavGains navGains(double tau, double zeta) {
  double t = tau / 3600.0;
  return {180.0 / (39.478417604357432 * zeta * t), zeta / (215666.565757755 * t)};
}

/**
 * Bank limit for heading changes: when the commanded heading changes (or the remaining heading change grows), the
 * limit is set from the heading change (0-5°: 5°, 10°: 10°, 20°+: 30°, interpolated). It goes back to 30° once the
 * roll command is within the limit and the heading command is steady.
 */
struct HeadingBankLimit {
  double lastPsi = 0;
  bool primed = false;
  double limit = 30.0;
  double limitDeltaPsi = 0.0;

  void reset() {
    primed = false;
    limit = 30.0;
    limitDeltaPsi = 0.0;
  }

  /** Returns the symmetric limit (up = +limit, lo = -limit). */
  double update(double psiC, double dPsi, double phiC) {
    static const int8_t bp[5]{0, 5, 10, 20, 30};
    static const int8_t val[5]{5, 5, 10, 30, 30};
    if (!primed) {
      lastPsi = psiC;
      primed = true;
    }
    bool psiCmdChanged = psiC != lastPsi;
    if (psiCmdChanged || std::abs(dPsi) > limitDeltaPsi) {
      limitDeltaPsi = std::abs(dPsi);
      if (limitDeltaPsi > 30.0) {
        limit = 30.0;
      } else {
        // binary search for the segment, as generated
        int high = 5, low = 0, lowp1 = 2;
        while (high > lowp1) {
          int mid = ((low + high) + 1) >> 1;
          if (limitDeltaPsi >= bp[mid - 1]) {
            low = mid - 1;
            lowp1 = mid + 1;
          } else {
            high = mid;
          }
        }
        double r = (limitDeltaPsi - static_cast<double>(bp[low])) / static_cast<double>(bp[low + 1] - bp[low]);
        if (r == 0.0) {
          limit = val[low];
        } else if (r == 1.0) {
          limit = val[low + 1];
        } else if (val[low + 1] == val[low]) {
          limit = val[low];
        } else {
          limit = (1.0 - r) * static_cast<double>(val[low]) + static_cast<double>(val[low + 1]) * r;
        }
      }
    }
    if (!psiCmdChanged && std::abs(phiC) < limit) {
      limit = 30.0;
    }
    lastPsi = psiC;
    return limit;
  }
};

// ----------------------------------------------------------------------------------------------------------------------
// Vertical helpers (FmgcOuterLoops)
// ----------------------------------------------------------------------------------------------------------------------

/** Median of three. */
inline double median3(double u1, double u2, double u3) {
  double v[3]{u1, u2, u3};
  int i;
  if (u1 < u2) {
    i = u2 < u3 ? 1 : (u1 < u3 ? 2 : 0);
  } else {
    i = u1 < u3 ? 0 : (u2 < u3 ? 2 : 1);
  }
  return v[i];
}

/** Limits a flight path angle change (deg) to 0.15 g at the current TAS. */
inline double vsLimit(double u, double vTasKn) {
  double limit = 9.81 / (vTasKn * 0.51444444444444448) * 0.15 * 57.295779513082323;
  return std::fmax(-limit, std::fmin(limit, u));
}

/** VLS used by speed protection: VLS - 5 kt when the speed target is at or below VLS. */
inline double vlsWithMargin(double vC, double vls) {
  return vC <= vls ? vls - 5.0 : vls;
}

/**
 * Speed protection in path modes: below VLS(+margin) the pitch command is the lower of the path command and the
 * VLS (speed-on-pitch) command; above VMAX(-margin) the higher of the path command and the VMAX command.
 */
struct ProtectedCommand {
  double fd, ap;
};
inline ProtectedCommand speedProtection(double vC, double vls, double vIas, double vmax, double vsFd, double vsAp,
                                        double vlsFd, double vlsAp, double vmaxFd, double vmaxAp, double margin) {
  double vlsSel = vlsWithMargin(vC, vls);
  if (vIas < vlsSel + margin) {
    return {std::fmin(vsFd, vlsFd), std::fmin(vsAp, vlsAp)};
  }
  if (vIas > vmax - margin) {
    return {std::fmax(vsFd, vmaxFd), std::fmax(vsAp, vmaxAp)};
  }
  return {vsFd, vsAp};
}

}  // namespace fmgc::blocks
