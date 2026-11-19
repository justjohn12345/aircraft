// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Blocks shared by the readable pitch normal and alternate laws (generated PitchNormalLaw.cpp, PitchAlternateLaw.cpp):
//
//   Delay-based blocks (no first-frame flag; init() and reset() set their Delay states to the initial condition, 0):
//     DiscreteDerivativeVariableTs        DiscreteDerivative     (k u - u[-1]) / dt, every gain k is 1
//     Tustin lag written with Delay/Delay1 TustinLag              (LagFilter3, and the lag inside SubsystemN)
//     SubsystemN (derivative + lag)       LaggedDerivative       derivative of u through a Tustin lag
//     RateLimiterVariableTs (inlined)     DelayRateLimiter       y += fmax(fmin(u - y, up dt), dt lo)
//     DiscreteTimeIntegratorVariableTs    Integrator             with its icLoad reset and limits
//   Charts and helpers:
//     eta_trim_limit_lofreeze / upfreeze  HoldWhile              sample the input, hold it while the trigger is true
//     chart c7                            FlightPhaseChart       ground / flight clean / flight flaps: trim rate and
//                                                                load factor limits
//     chart c8                            AutoTrimModeChart      manual / automatic / reset / tracking: computed, read
//                                                                by nothing
//     chart c9                            TrimFreezeChart        auto trim frozen outside 0.5..1.25 g, bank > 30 deg,
//                                                                or (normal law) in flare
//     the "C* law" subsystems             CStarTerms, CStarChannel   (below)
//   From efcs_blocks.h: LagFilter, WashoutFilter, RateLimiterInit (RateLimiter / RateLimiter_c with an initial value),
//   lookup1 / lookup2, limit, median3 (VoterAttitudeProtection / the alternate law's inline vote).

#pragma once

#include <cmath>
#include <cstdint>

#include "efcs_blocks.h"

namespace efcs::readable {

namespace pitch_blocks {

using blocks::LagFilter;
using blocks::limit;
using blocks::lookup1;
using blocks::lookup2;
using blocks::RateLimiterInit;
using blocks::WashoutFilter;
using fmgc::blocks::median3;

constexpr double kDegToRad = 0.017453292519943295;
constexpr double kKtToMs = 0.51444444444444448;

/** DiscreteDerivativeVariableTs: (u - previous u) / dt; the previous input starts at 0 (init / reset). */
struct DiscreteDerivative {
  double pU = 0.0;  // Delay_DSTATE_*

  void reset() { pU = 0.0; }

  double update(double u, double dt) {
    const double d = (u - pU) / dt;
    pU = u;
    return d;
  }
};

/**
 * First-order Tustin lag 1/(1 + s/C1) written with two Delay blocks (state starts at 0, not at the first input):
 * y = 1/(a + 2) (2 - a) y[-1] + (u + u[-1]) a/(a + 2), a = dt C1.
 */
struct TustinLag {
  double y = 0.0;   // Delay1_DSTATE_*
  double pU = 0.0;  // Delay_DSTATE_*

  void reset() {
    y = 0.0;
    pU = 0.0;
  }

  double update(double u, double c1, double dt) {
    const double a = dt * c1;
    const double den = a + 2.0;
    y = 1.0 / den * (2.0 - a) * y + (u + pU) * (a / den);
    pU = u;
    return y;
  }
};

/** SubsystemN: the derivative of u (DiscreteDerivative) through a TustinLag. */
struct LaggedDerivative {
  DiscreteDerivative derivative;
  TustinLag lag;

  void reset() {
    derivative.reset();
    lag.reset();
  }

  double update(double u, double c1, double dt) { return lag.update(derivative.update(u, dt), c1, dt); }
};

/**
 * RateLimiterVariableTs written inline with a Delay block (no first-frame flag; starts at 0 after init / reset):
 * y += fmax(fmin(u - y, up dt), dt lo). `lo` is negative.
 */
struct DelayRateLimiter {
  double y = 0.0;  // Delay_DSTATE_*

  void reset() { y = 0.0; }

  double update(double u, double up, double lo, double dt) {
    y += std::fmax(std::fmin(u - y, up * dt), dt * lo);
    return y;
  }
};

/**
 * DiscreteTimeIntegratorVariableTs with external reset: while `reset` is true (and on the first step after init /
 * reset) the state is loaded with `initial`, written `x = initial - increment; x += increment` as generated (so not
 * exactly `initial`). The state is limited to [lo, hi].
 */
struct Integrator {
  double x = 0.0;       // Delay_DSTATE_*
  bool icLoad = false;  // icLoad*

  void requestLoad() { icLoad = true; }

  double update(double increment, bool reset, double initial, double lo, double hi) {
    icLoad = reset || icLoad;
    if (icLoad) {
      x = initial - increment;
    }
    x += increment;
    x = limit(x, lo, hi);
    icLoad = false;
    return x;
  }
};

/** eta_trim_limit_lofreeze / _upfreeze: follows the input, holds the value it had when `hold` went true. */
struct HoldWhile {
  double value = 0.0;  // frozen_eta_trim
  bool valid = false;  // frozen_eta_trim_not_empty

  void reset() { valid = false; }

  double update(double u, bool hold) {
    if (!hold || !valid) {
      value = u;
      valid = true;
    }
    return value;
  }
};

/**
 * Chart c7 (PitchNormalLaw.cpp 1545-1618, PitchAlternateLaw.cpp 515-587): flight phase for the auto trim rate and the
 * load factor limits. Ground on the first step; ground -> flight clean / flaps when in flight (by the flap lever);
 * clean <-> flaps by the flap lever; back to ground when not in flight. The limits depend only on the new state:
 *   flight clean   trim rate +-0.3 deg/s, nz -1 .. +2.5 g
 *   flaps, ground  trim rate +-0.7 deg/s, nz  0 .. +2 g
 */
struct FlightPhaseChart {
  enum class Phase : uint8_t { None = 0, FlightClean = 1, FlightFlaps = 2, Ground = 3 };  // generated values
  Phase phase = Phase::None;

  struct Limits {
    double trimRateUp, trimRateLo, nzUp, nzLo;
  };

  void reset() { phase = Phase::None; }

  Limits update(double inFlight, double flapsHandleIndex) {
    switch (phase) {
      case Phase::None:
        phase = Phase::Ground;
        break;
      case Phase::FlightClean:
        if (flapsHandleIndex != 0.0) {
          phase = Phase::FlightFlaps;
        } else if (inFlight == 0.0) {
          phase = Phase::Ground;
        }
        break;
      case Phase::FlightFlaps:
        if (flapsHandleIndex == 0.0) {
          phase = Phase::FlightClean;
        } else if (inFlight == 0.0) {
          phase = Phase::Ground;
        }
        break;
      default:  // Ground
        if (inFlight != 0.0 && flapsHandleIndex == 0.0) {
          phase = Phase::FlightClean;
        } else if (inFlight != 0.0 && flapsHandleIndex != 0.0) {
          phase = Phase::FlightFlaps;
        }
        break;
    }
    if (phase == Phase::FlightClean) {
      return {0.3, -0.3, 2.5, -1.0};
    }
    return {0.7, -0.7, 2.0, 0.0};
  }
};

/**
 * Chart c8 (PitchNormalLaw.cpp 1531-1562, PitchAlternateLaw.cpp 856-887): an auto trim mode (manual on the ground,
 * automatic in flight, tracking, reset until the THS is back at 0 after landing). Nothing reads it: it only has state.
 */
struct AutoTrimModeChart {
  enum class Mode : uint8_t { None = 0, Automatic = 1, Manual = 2, Reset = 3, Tracking = 4 };  // generated values
  Mode mode = Mode::None;

  void reset() { mode = Mode::None; }

  void update(double inFlight, bool trackingMode, double etaTrimDeg) {
    switch (mode) {
      case Mode::None:
        mode = Mode::Manual;
        break;
      case Mode::Automatic:
        if (inFlight == 0.0) {
          mode = Mode::Reset;
        } else if (trackingMode) {
          mode = Mode::Tracking;
        }
        break;
      case Mode::Manual:
        if (inFlight != 0.0) {
          mode = Mode::Automatic;
        }
        break;
      case Mode::Reset:
        if (inFlight == 0.0 && etaTrimDeg == 0.0) {
          mode = Mode::Manual;
        }
        break;
      default:  // Tracking
        if (!trackingMode) {
          mode = Mode::Automatic;
        }
        break;
    }
  }
};

/**
 * Chart c9 (PitchNormalLaw.cpp 1492-1510, PitchAlternateLaw.cpp 477-495): auto trim freeze. Running on the first step;
 * frozen when `inhibit` (normal law: in flare; alternate law: constant false), nz >= 1.25 g, nz <= 0.5 g or bank > 30
 * deg; running again only when none of those holds (a NaN nz or bank keeps the current state).
 */
struct TrimFreezeChart {
  enum class State : uint8_t { None = 0, Frozen = 1, Running = 2 };  // generated values
  State state = State::None;

  void reset() { state = State::None; }

  bool update(bool inhibit, double nzG, double phiDeg) {
    if (state == State::None) {
      state = State::Running;
      return false;
    }
    if (state == State::Frozen) {
      if (!inhibit && nzG < 1.25 && nzG > 0.5 && std::abs(phiDeg) <= 30.0) {
        state = State::Running;
        return false;
      }
      return true;
    }
    if (inhibit || nzG >= 1.25 || nzG <= 0.5 || std::abs(phiDeg) > 30.0) {
      state = State::Frozen;
      return true;
    }
    return false;
  }
};

// ---------------------------------------------------------------------------------------------------------------------
// C* law
// ---------------------------------------------------------------------------------------------------------------------
//
// Every pitch channel of both laws ends in the same "C* law" subsystem (7 copies in the normal law, 3 in the alternate
// law): it turns a load factor demand into an elevator deflection rate (deg/s, limited to +-30):
//
//   y   = (V/g q + nz - cos(theta)/cos(phi)) - (1 + 100 / V_tas) (nz_demand - cos(theta)/cos(phi))
//         (q in rad/s, V = 100 m/s fixed, V_tas in m/s limited to 100..2000, the "100" is a speed table with 100 at
//         every speed)
//   out = q_dot + 13.5 y + d/dt(0.5 y) + 0.06 V_tas_dot + spoiler term
//         (q_dot: derivative of qk; V_tas_dot: derivative of V_tas (kt/s) lagged 0.3 rad/s, limited to +-1; spoiler
//         term: min(left, right spoiler) washed out at 5 rad/s, limited to +-1, times a gain: normal law 0 / -30 by
//         radio height (0..50 ft: 0, 100 ft and above: -30), alternate law -30)
//
// Everything except the d/dt(0.5 y) term is the same in every copy (same blocks, same inputs, reset together), so the
// readable computes it once (CStarTerms) and each channel keeps only its own derivative state (CStarChannel). The
// differential checker pairs every generated copy with the one readable state.

/** The C* inputs shared by every channel. */
struct CStarTerms {
  double qkDot;          // pitch acceleration from the qk derivative (deg/s^2)
  double pitchRateTerm;  // V/g q with V = 100 m/s (q in rad/s)
  double nzError;        // nz - cos(theta)/cos(phi)
  double cosRatio;       // cos(theta)/cos(phi): 1 g in level flight at any bank
  double nzGain;         // 1 + 100 / V_tas (m/s, limited to 100..2000)
  double pGain;          // 13.5 (PLUT, constant table)
  double dGain;          // 0.5 (DLUT, constant table)
  double vTasDot;        // V_tas rate (kt/s), lagged and limited to +-1
  double spoilerTerm;    // washed out spoiler deflection (+-1) x gain
};

/** The shared C* filters: the qk and V_tas derivatives, the V_tas rate lag and the spoiler washout. */
struct CStarFilters {
  DiscreteDerivative qkRate;    // DiscreteDerivativeVariableTs1 (Delay of qk)
  DiscreteDerivative vTasRate;  // DiscreteDerivativeVariableTs2 (Delay of V_tas)
  LagFilter<double> vTasRateLag;
  WashoutFilter spoilerWashout;

  /** init(): the Delay states (the filters keep their state). */
  void init() {
    qkRate.reset();
    vTasRate.reset();
  }

  void reset() {
    init();
    vTasRateLag.reset();
    spoilerWashout.reset();
  }

  CStarTerms update(double qkDegS, double nzG, double cosRatio, double vTasKn, double spoilerPos, double spoilerGain,
                    double dt) {
    static constexpr double kSpeedBp[7] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0};
    static constexpr double kSpeedGain[7] = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
    static constexpr double kPdBp[2] = {0.0, 350.0};
    static constexpr double kP[2] = {13.5, 13.5};
    static constexpr double kD[2] = {0.5, 0.5};
    static constexpr double kOneOverG = 0.1019367991845056;
    static constexpr double kVmMs = 100.0;

    CStarTerms t;
    t.qkDot = qkRate.update(qkDegS, dt);
    t.pitchRateTerm = kOneOverG * kVmMs * (kDegToRad * qkDegS);
    t.nzError = nzG - cosRatio;
    t.cosRatio = cosRatio;
    t.nzGain = lookup1(vTasKn, kSpeedBp, kSpeedGain) / (kKtToMs * limit(vTasKn, 100.0, 2000.0)) + 1.0;
    t.pGain = lookup1(vTasKn, kPdBp, kP);
    t.dGain = lookup1(vTasKn, kPdBp, kD);
    t.vTasDot = limit(vTasRateLag.update(vTasRate.update(vTasKn, dt), 0.3, dt), -1.0, 1.0);
    t.spoilerTerm = limit(spoilerWashout.update(spoilerPos, 5.0, dt), -1.0, 1.0) * spoilerGain;
    return t;
  }
};

/** One C* channel: its load factor demand -> elevator rate (deg/s, +-30). Only the D term has a state of its own. */
struct CStarChannel {
  DiscreteDerivative dTerm;  // DiscreteDerivativeVariableTs (Delay of 0.5 y)

  void reset() { dTerm.reset(); }

  double update(const CStarTerms& t, double nzDemand, double dt) {
    const double y = (t.pitchRateTerm + t.nzError) - t.nzGain * (nzDemand - t.cosRatio);
    const double p = y * t.pGain;
    const double d = dTerm.update(y * t.dGain, dt);
    return limit((((t.qkDot + p) + d) + 0.06 * t.vTasDot) + t.spoilerTerm, -30.0, 30.0);
  }
};

/** Load factor increment (g) demanded by the stick / a protection term: -1 -> -2 g, 0 -> 0, +1 -> +1.5 g. */
inline double loadDemand(double u) {
  static constexpr double kBp[3] = {-1.0, 0.0, 1.0};
  static constexpr double kNz[3] = {-2.0, 0.0, 1.5};
  return lookup1(u, kBp, kNz);
}

/** Auto trim: eta (deg) / 13 as a THS rate (deg/s), 0 while frozen, limited to the flight phase's trim rate. */
inline double autoTrimRate(bool frozen, double etaDeg, const FlightPhaseChart::Limits& limits) {
  const double eta = frozen ? 0.0 : etaDeg;
  return limit(0.076923076923076927 * eta, limits.trimRateLo, limits.trimRateUp);
}

/** Integrator gain against the time step (s): 1 up to 0.06 s, 0.5 at 0.1 s, 0.3 from 0.2 s. */
inline double timeStepGain(double dt) {
  static constexpr double kBp[5] = {0.0, 0.06, 0.1, 0.2, 1.0};
  static constexpr double kGain[5] = {1.0, 1.0, 0.5, 0.3, 0.3};
  return lookup1(dt, kBp, kGain);
}

}  // namespace pitch_blocks

}  // namespace efcs::readable
