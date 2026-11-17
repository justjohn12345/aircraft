// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Input injectors for the FADEC differential check (src/check_fadec.cpp), applied on top of the drive
// (src/fadec_drive.h) at every step, to both FADECs, in this order: --script, --lever, --athr, --sim. Each one is
// reproducible from its seed. The generated FADEC is the oracle, so any input is a valid test input; inconsistent
// combinations are wanted.
//
//  --script SEED  a scripted tour, cycling (the seed shifts the start phase and jitters the values per cycle):
//                 lever sweeps -20..46 deg on the ground and in flight; every detent and band edge held (-20, -6, -3,
//                 0, 12, 24, 24.5, 25, 25.5, 26, 30, 34, 35, 36, 40, 45, 47; odd limits on every other cycle); A/THR
//                 engaged with the N1 command below idle, in range, above the lever, the lever out of the detents,
//                 alpha floor; instinctive disconnect
//                 short (thrust lock) and held 16 s (A/THR disabled) then reset; non-standard disconnect (ATS bit 13
//                 lost) in the MCT detent, lever moved inside then out of the band; ATS bit 14 and N1 command SSM lost
//                 while in control; FLX takeoff (TAT also equal to the flex temperature) with the latch cleared at CL
//                 and kept when the lever skips 25 deg; engine out and NaN lever on the ground and in flight;
//                 touchdown toggles and simulation time 0;
//                 invalid ATS / N1 command / flex words (SSM, huge, negative, NaN data); limit changes (negative IDLE,
//                 REV of both signs), commanded N1 offsets saturating the throttle integrators, N1 steps (trim reset).
//  --lever SEED   random lever episodes per engine or both: exact detents, random angles, ramps, NaN.
//  --athr SEED    random A/THR episodes: ATS bits 13/14/23 and SSM, N1 command SSM and value, instinctive disconnect
//                 pulses and holds (up to 20 s), reset-disable pulses, flex temperature word.
//  --sim SEED     random sim episodes: on ground, engine operative, TAT, limits, commanded N1 offsets, N1 actual
//                 offsets, N1 actual exactly 0.8 % below the N1 command, dt (0, tiny, large), simulation time jumps
//                 and 0.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

#include "FadecComputer_types.h"

namespace fadec_scenarios {

class Rng {
 public:
  explicit Rng(uint64_t seed) : rng_(seed) {}
  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng_); }
  bool chance(double p) { return u(0, 1) < p; }
  int irand(int n) { return static_cast<int>(rng_() % static_cast<uint64_t>(n)); }

 private:
  std::mt19937_64 rng_;
};

constexpr uint32_t kFw = 0, kNcd = 1, kFt = 2, kNo = 3;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline void setBit(base_arinc_429& w, int bit, bool v) {
  float r = std::round(w.Data);
  uint32_t x = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  x = v ? (x | (1U << (bit - 1))) : (x & ~(1U << (bit - 1)));
  w.Data = static_cast<float>(x);
}

/** A/THR words of the FCU bus: ATS bits 13 (engaged), 14 (active), 23 (alpha floor), N1 command. */
inline void athrBus(athr_in& in, bool engaged, bool active, bool alphaFloor, uint32_t cmdSsm, double cmd) {
  base_arinc_429& w = in.fcu_input.ats_discrete_word;
  w.SSM = kNo;
  setBit(w, 13, engaged);
  setBit(w, 14, active);
  setBit(w, 23, alphaFloor);
  in.fcu_input.n1_cmd_percent = {cmdSsm, static_cast<float>(cmd)};
}

inline void both(athr_in (&in)[2], double tla, bool onGround) {
  for (auto& i : in) {
    i.input.TLA_deg = tla;
    i.data.on_ground = onGround;
    i.data.is_engine_operative = true;
  }
}

/** Scripted tour (--script). */
class ScriptScenario {
 public:
  explicit ScriptScenario(uint64_t seed) : rng_(seed * 7919 + 3) { phase_ = rng_.irand(kPhases); }
  long episodes() const { return episodes_; }

  void apply(athr_in (&in)[2], double dt) {
    if (t_ >= duration(phase_)) {
      t_ = 0;
      phase_ = (phase_ + 1) % kPhases;
      jitter_ = rng_.u(-0.4, 0.4);
      episodes_++;
    }
    run(in, t_);
    t_ += dt > 0 && std::isfinite(dt) ? dt : 0.02;
  }

 private:
  static constexpr int kPhases = 12;
  static double duration(int p) {
    static const double d[kPhases] = {22, 22, 17, 16, 10, 24, 16, 20, 12, 10, 10, 10};
    return d[p];
  }

  void run(athr_in (&in)[2], double t) {
    static const double detents[17] = {-20, -6, -3, 0, 12, 24, 24.5, 25, 25.5, 26, 30, 34, 35, 36, 40, 45, 47};
    const double j = jitter_;
    for (auto& i : in) athrBus(i, false, false, false, kNo, 0.0);
    switch (phase_) {
      case 0:  // lever sweep on the ground
        both(in, -20 + 3 * t + j, true);
        break;
      case 1:  // lever sweep in flight
        both(in, -20 + 3 * t + j, false);
        break;
      case 2:  // every detent and band edge, exactly; odd limits on odd cycles (rounding at the segment ends)
        both(in, detents[std::min(16, static_cast<int>(t))], episodes_ % 2 == 0);
        if (episodes_ % 2 == 1) {
          for (auto& i : in) {
            auto& l = i.input;
            l.thrust_limit_CLB_percent += 3.7 * j + 0.123;
            l.thrust_limit_MCT_percent += 2.9 * j + 0.311;
            l.thrust_limit_FLEX_percent += 1.3 * j + 0.071;
            l.thrust_limit_TOGA_percent += 0.7 * j + 0.019;
          }
        }
        break;
      case 3: {  // A/THR engaged, command clamping
        both(in, t < 8 ? 25.0 : (t < 12 ? 30.0 : 10.0), false);
        static const double cmd[4] = {10.0, 60.0, 150.0, 70.0};
        double c = t < 12 ? cmd[static_cast<int>(t / 3) % 4] + j : (t < 14 ? 150.0 : 50.0);
        for (auto& i : in) athrBus(i, true, true, t >= 12, kNo, c);
        break;
      }
      case 4:  // instinctive disconnect, short, lever in CL: thrust lock; then lever moved
        both(in, t < 7 ? 25.0 : 20.0 + j, false);
        for (auto& i : in) {
          athrBus(i, t < 3.1, t < 3.1, false, kNo, 70.0 + j);
          i.input.ATHR_disconnect = t >= 3.0 && t < 3.3;
        }
        break;
      case 5:  // instinctive disconnect held 16 s: A/THR disabled; reset; engage again
        both(in, 25.0, false);
        for (auto& i : in) {
          athrBus(i, t < 1 || (t > 2 && t < 19.5) || t >= 20, true, false, kNo, 65.0);
          i.input.ATHR_disconnect = t >= 2 && t < 18.5;
          i.input.ATHR_reset_disable = t >= 19.0 && t < 19.1;
        }
        break;
      case 6:  // non-standard disconnect in the MCT detent; lever inside then outside the band; bit 14 and SSM lost
        both(in, t < 6 ? 35.0 : (t < 8 ? 34.5 : (t < 10 ? 33.0 : 26.0)), false);
        for (auto& i : in) {
          athrBus(i, t < 3 || t >= 10.5, t < 3 || (t >= 10.5 && t < 12.5), false, (t >= 14 && t < 15) ? kNcd : kNo,
                  80.0 + j);
        }
        break;
      case 7: {  // FLX: latch on the ground at 35, kept in flight, cleared at 25; kept when the lever skips 25
        static const double tla[10] = {0, 35, 35, 35, 25, 35, 35, 20, 35, 45};
        static const bool ground[10] = {true, true, false, false, false, true, false, false, false, false};
        int k = std::min(9, static_cast<int>(t / 2));
        both(in, tla[k], ground[k]);
        for (auto& i : in) {
          i.fcu_input.fcu_flex_to_temp_deg_c = {(k == 3 && episodes_ % 3 == 0) ? kNcd : kNo, 55.0F};
          // TAT above, equal to (flex not valid: strictly above is needed) or below the flex temperature
          i.data.TAT_degC = (k == 1 && episodes_ % 4 == 1) ? 60.0 : ((k == 1 && episodes_ % 4 == 2) ? 55.0 : 15.0);
        }
        break;
      }
      case 8:  // engine out on the ground and in flight; NaN lever
        both(in, -20 + 5 * t, t < 8);
        in[0].data.is_engine_operative = false;
        if (t > 4 && t < 4.5) in[0].input.TLA_deg = in[1].input.TLA_deg = kNaN;
        if (t > 10 && t < 10.5) in[0].input.TLA_deg = in[1].input.TLA_deg = kNaN;
        if (t > 11) in[0].data.is_engine_operative = true;
        if (t > 11.5) in[0].input.TLA_deg = in[1].input.TLA_deg = kNaN;
        break;
      case 9:  // touchdown timing, simulation time 0
        both(in, 0.0, static_cast<int>(t / 1.5) % 2 == 0);
        if (t < 0.1) {
          for (auto& i : in) {
            i.time.simulation_time = 0.0;
            i.data.on_ground = false;
          }
        }
        break;
      case 10: {  // invalid FCU words
        both(in, 25.0, false);
        int k = static_cast<int>(t);
        for (auto& i : in) {
          athrBus(i, true, true, k % 3 == 0, kNo, 60.0);
          base_arinc_429& w = i.fcu_input.ats_discrete_word;
          w.SSM = static_cast<uint32_t>(k % 4);
          if (k == 2) w.Data = 5.0e9F;
          if (k == 3) w.Data = -5.0F;
          if (k == 4) w.Data = std::numeric_limits<float>::quiet_NaN();
          i.fcu_input.n1_cmd_percent.SSM = static_cast<uint32_t>((k + 1) % 4);
          i.fcu_input.fcu_flex_to_temp_deg_c = {static_cast<uint32_t>(k % 4), 40.0F};
        }
        break;
      }
      case 11: {  // limits, commanded N1 offsets, N1 steps
        int k = static_cast<int>(t);
        both(in, k < 5 ? -15.0 : 25.0, k < 5);
        for (auto& i : in) {
          auto& l = i.input;
          if (k == 1) l.thrust_limit_IDLE_percent = -3.0;
          if (k == 2) l.thrust_limit_REV_percent = -l.thrust_limit_REV_percent;
          if (k == 6) l.thrust_limit_FLEX_percent = l.thrust_limit_TOGA_percent + 5.0;
          i.data.commanded_engine_N1_percent += k == 3 ? 80.0 : (k == 4 ? -80.0 : (k == 7 ? 120.0 : (k == 8 ? -120.0 : 0.0)));
          if (k == 9) i.data.engine_N1_percent += 5.0;
        }
        break;
      }
    }
  }

  Rng rng_;
  int phase_ = 0;
  double t_ = 0, jitter_ = 0;
  long episodes_ = 0;
};

/** Random lever episodes (--lever). */
class LeverScenario {
 public:
  explicit LeverScenario(uint64_t seed) : rng_(seed * 104729 + 11) {}
  long episodes() const { return episodes_; }

  void apply(athr_in (&in)[2], double dt) {
    if (remaining_ <= 0) {
      start(in);
    }
    remaining_ -= dt > 0 && std::isfinite(dt) ? dt : 0.02;
    elapsed_ += dt > 0 && std::isfinite(dt) ? dt : 0.02;
    for (int e = 0; e < 2; e++) {
      if (!(mask_ & (1 << e))) continue;
      double v = target_[e];
      if (ramp_ > 0) v = from_[e] + (target_[e] - from_[e]) * std::min(1.0, elapsed_ / ramp_);
      in[e].input.TLA_deg = v;
      last_[e] = v;
    }
  }

 private:
  void start(athr_in (&in)[2]) {
    static const double detents[] = {-20, -6, 0, 24, 25, 26, 34, 35, 36, 45};
    episodes_++;
    remaining_ = rng_.u(0.3, 6);
    elapsed_ = 0;
    mask_ = 1 + rng_.irand(3);
    ramp_ = rng_.chance(0.3) ? rng_.u(0.5, 5) : 0;
    for (int e = 0; e < 2; e++) {
      from_[e] = std::isfinite(last_[e]) ? last_[e] : in[e].input.TLA_deg;
      int k = rng_.irand(10);
      target_[e] = k < 4 ? detents[rng_.irand(10)] : (k < 9 ? rng_.u(-25, 50) : (rng_.chance(0.2) ? kNaN : 25.0));
      if (mask_ == 3 && e == 1 && rng_.chance(0.7)) target_[1] = target_[0];
    }
  }

  Rng rng_;
  double remaining_ = 0, elapsed_ = 0, ramp_ = 0;
  int mask_ = 0;
  double from_[2]{}, target_[2]{}, last_[2]{kNaN, kNaN};
  long episodes_ = 0;
};

/** Random A/THR episodes (--athr). */
class AthrScenario {
 public:
  explicit AthrScenario(uint64_t seed) : rng_(seed * 15485863 + 5) {}
  long episodes() const { return episodes_; }

  void apply(athr_in (&in)[2], double dt) {
    const double step = dt > 0 && std::isfinite(dt) ? dt : 0.02;
    if (remaining_ <= 0) {
      start();
    }
    remaining_ -= step;
    for (auto& i : in) {
      if (busOn_) {
        athrBus(i, engaged_, active_, alphaFloor_, cmdSsm_, cmd_);
        i.fcu_input.ats_discrete_word.SSM = atsSsm_;
      }
      if (flexOn_) i.fcu_input.fcu_flex_to_temp_deg_c = {flexSsm_, flex_};
      if (disconnect_ > 0) i.input.ATHR_disconnect = true;
      if (reset_ > 0) i.input.ATHR_reset_disable = true;
    }
    disconnect_ -= step;
    reset_ -= step;
  }

 private:
  void start() {
    episodes_++;
    remaining_ = rng_.u(0.3, 8);
    busOn_ = rng_.chance(0.8);
    engaged_ = rng_.chance(0.6);
    active_ = rng_.chance(0.6);
    alphaFloor_ = rng_.chance(0.15);
    cmdSsm_ = rng_.chance(0.75) ? kNo : static_cast<uint32_t>(rng_.irand(4));
    atsSsm_ = rng_.chance(0.85) ? kNo : static_cast<uint32_t>(rng_.irand(4));
    cmd_ = rng_.u(-10, 130);
    flexOn_ = rng_.chance(0.3);
    flexSsm_ = rng_.chance(0.6) ? kNo : static_cast<uint32_t>(rng_.irand(4));
    flex_ = static_cast<float>(rng_.u(-20, 80));
    int d = rng_.irand(10);
    disconnect_ = d < 2 ? rng_.u(0.01, 0.4) : (d < 3 ? rng_.u(10, 20) : 0);
    reset_ = rng_.chance(0.1) ? rng_.u(0.01, 0.2) : 0;
  }

  Rng rng_;
  double remaining_ = 0, cmd_ = 0, disconnect_ = 0, reset_ = 0;
  bool busOn_ = false, engaged_ = false, active_ = false, alphaFloor_ = false, flexOn_ = false;
  uint32_t cmdSsm_ = kNo, atsSsm_ = kNo, flexSsm_ = kNo;
  float flex_ = 0;
  long episodes_ = 0;
};

/** Random sim episodes (--sim). */
class SimScenario {
 public:
  explicit SimScenario(uint64_t seed) : rng_(seed * 32452843 + 7) {}
  long episodes() const { return episodes_; }

  /** The computer under test's previous outputs, for the "N1 error exactly 0.8" episodes (may be null). */
  const athr_out* feedback[2] = {nullptr, nullptr};

  void apply(athr_in (&in)[2], double dt) {
    const double step = dt > 0 && std::isfinite(dt) ? dt : 0.02;
    if (remaining_ <= 0) {
      start();
    }
    remaining_ -= step;
    timeOffset_ += timeJump_;
    timeJump_ = 0;
    for (int e = 0; e < 2; e++) {
      athr_in& i = in[e];
      i.time.simulation_time += timeOffset_;
      if (zeroTime_) i.time.simulation_time = 0.0;
      if (dtKind_ == 1) i.time.dt = 0.0;
      if (dtKind_ == 2) i.time.dt = dtValue_;
      if (groundOn_) i.data.on_ground = ground_;
      if (engineOut_ & (1 << e)) i.data.is_engine_operative = false;
      if (tatOn_) i.data.TAT_degC = tat_;
      i.data.commanded_engine_N1_percent += cmdOffset_;
      i.data.engine_N1_percent += n1Offset_;
      if (trimEdge_ && feedback[e]) {
        // N1 actual exactly 0.8 below the previous (steady) N1 command: the trim reset threshold itself
        const double c = feedback[e]->output.N1_c_percent;
        double n1 = c - 0.8;
        for (int k = 0; k < 8 && c - n1 != 0.8; k++) n1 = std::nextafter(n1, c - n1 > 0.8 ? c : -1e9);
        i.data.engine_N1_percent = n1;
      }
      if (limitsOn_) {
        auto& l = i.input;
        l.thrust_limit_IDLE_percent += limit_[0];
        l.thrust_limit_CLB_percent += limit_[1];
        l.thrust_limit_MCT_percent += limit_[2];
        l.thrust_limit_FLEX_percent += limit_[3];
        l.thrust_limit_TOGA_percent += limit_[4];
        l.thrust_limit_REV_percent += limit_[5];
      }
    }
    zeroTime_ = false;
    if (dtKind_ == 1) dtKind_ = 0;  // dt = 0 for one frame
  }

 private:
  void start() {
    episodes_++;
    remaining_ = rng_.u(0.3, 10);
    groundOn_ = rng_.chance(0.4);
    ground_ = rng_.chance(0.5);
    engineOut_ = rng_.chance(0.2) ? 1 + rng_.irand(3) : 0;
    tatOn_ = rng_.chance(0.3);
    tat_ = rng_.u(-60, 60);
    cmdOffset_ = rng_.chance(0.3) ? rng_.u(-60, 60) : 0;
    n1Offset_ = rng_.chance(0.2) ? (rng_.chance(0.05) ? kNaN : rng_.u(-10, 10)) : 0;
    trimEdge_ = rng_.chance(0.1);
    limitsOn_ = rng_.chance(0.3);
    for (double& l : limit_) l = rng_.u(-15, 15);
    int k = rng_.irand(20);
    dtKind_ = k == 0 ? 1 : (k == 1 ? 2 : 0);
    dtValue_ = rng_.chance(0.5) ? rng_.u(1e-6, 1e-3) : rng_.u(0.1, 1.0);
    timeJump_ = k == 2 ? rng_.u(-30, 30) : 0;
    zeroTime_ = k == 3;
  }

  Rng rng_;
  double remaining_ = 0, tat_ = 0, cmdOffset_ = 0, n1Offset_ = 0, dtValue_ = 0, timeOffset_ = 0, timeJump_ = 0;
  double limit_[6]{};
  bool groundOn_ = false, ground_ = false, tatOn_ = false, limitsOn_ = false, zeroTime_ = false, trimEdge_ = false;
  int engineOut_ = 0, dtKind_ = 0;
  long episodes_ = 0;
};

}  // namespace fadec_scenarios
