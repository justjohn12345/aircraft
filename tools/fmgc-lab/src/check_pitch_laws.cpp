// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable pitch laws (readable/efcs/pitch_*_law.*) against the generated PitchNormalLaw,
// PitchAlternateLaw and PitchDirectLaw. Each law runs as a generated / readable pair in lockstep on the same synthetic
// input trajectory (the readable laws never see a generated value); every step all four outputs are compared bit by
// bit (two NaNs match whatever their sign and payload), and so is every hidden memory (src/pitch_law_state.h: filters,
// rate limiters, delays, integrators, charts, the normal law's flare mode outputs).
//
// Inputs: seeded episodes of a few hundred to a few ten thousand steps, each a scenario: take-off (ground roll,
// rotation, lift-off, sometimes rejected), approach (descent through 50 / 30 ft, flare, touchdown, roll-out,
// sometimes a go-around), up-and-away (protections, autopilot, tracking mode, trim freeze conditions) or chaos (every
// input an independent random walk over a wide range). On top: dt 0.01-0.1 s (sometimes fixed, rarely 0 or large),
// rare injected NaN / inf / huge / tiny / threshold values on one input, rare reset() (an ELAC restart) and init().
//
// usage: efcs-pitch-law-check [--seed S] [--steps N] [--law normal|alternate|direct|all] [--list N]
// Exit status 0 when every output and memory matched on every step, 1 on any mismatch, 2 on usage errors.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <map>
#include <memory>
#include <string>

#include "pitch_law_state.h"  // first: opens the generated laws' private members

namespace R = efcs::readable;

namespace {

// ---------------------------------------------------------------------------------------------------------------------
// Random numbers (splitmix64: the same sequence on every platform)
// ---------------------------------------------------------------------------------------------------------------------

class Rng {
 public:
  // the seed is hashed so that different seeds start far apart in the splitmix sequence
  explicit Rng(uint64_t seed) : s_(seed) {
    s_ = next() ^ 0x5DEECE66DULL;
    s_ = next();
  }
  uint64_t next() {
    uint64_t z = (s_ += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  double uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }
  double range(double lo, double hi) { return lo + (hi - lo) * uniform(); }
  double sym() { return 2.0 * uniform() - 1.0; }
  bool chance(double p) { return uniform() < p; }
  int intRange(int lo, int hi) { return lo + static_cast<int>(next() % static_cast<uint64_t>(hi - lo + 1)); }
  template <size_t n>
  double pick(const double (&v)[n]) {
    return v[next() % n];
  }

 private:
  uint64_t s_;
};

// ---------------------------------------------------------------------------------------------------------------------
// Input trajectories
// ---------------------------------------------------------------------------------------------------------------------

/** Every input of the three laws (superset). */
struct Signals {
  double dt = 0.02;
  double nz = 1, theta = 0, phi = 0, qk = 0, qkDot = 0, eta = 0, etaTrim = 0, alpha = 0, vIas = 0, vTas = 0, mach = 0;
  double h = 0, flaps = 0, splL = 0, splR = 0, tl1 = 0, tl2 = 0, vls = 0, delta = 0, inFlight = 0;
  double alphaProt = 10, alphaMax = 14, hspHigh = 360, hspLow = 350, apTheta = 0;
  bool tailstrike = false, onGround = true, tracking = false, aoaProt = false, hspProt = false, ap = false;
  bool stabilities = true;
};

enum class Scenario { Takeoff, Approach, Cruise, Chaos };

class Trajectory {
 public:
  explicit Trajectory(Rng& rng) : rng_(rng) {}

  const Signals& next() {
    if (left_ <= 0) {
      newEpisode();
    }
    left_--;
    stepDt();
    switch (scenario_) {
      case Scenario::Takeoff:
        takeoff();
        break;
      case Scenario::Approach:
        approach();
        break;
      case Scenario::Cruise:
        cruise();
        break;
      case Scenario::Chaos:
        chaos();
        break;
    }
    common();
    out_ = s_;
    inject();
    return out_;
  }

  bool episodeStarted() const { return started_; }

 private:
  // v moves by up to `step` per call, clamped to [lo, hi]
  void walk(double& v, double step, double lo, double hi) {
    if (!(v == v) || std::isinf(v)) v = rng_.range(lo, hi);
    v += step * rng_.sym();
    if (v < lo) v = lo;
    if (v > hi) v = hi;
  }
  // moves v towards target by at most `rate` x dt (plus noise)
  void toward(double& v, double target, double rate, double noise = 0.0) {
    const double d = target - v;
    const double m = rate * s_.dt;
    v += (d > m ? m : (d < -m ? -m : d)) + noise * rng_.sym();
  }
  void flip(bool& b, double p) {
    if (rng_.chance(p)) b = !b;
  }

  void newEpisode() {
    started_ = true;
    const double r = rng_.uniform();
    scenario_ = r < 0.25   ? Scenario::Takeoff
                : r < 0.55 ? Scenario::Approach
                : r < 0.85 ? Scenario::Cruise
                           : Scenario::Chaos;
    left_ = rng_.intRange(300, 25000);
    const double dm = rng_.uniform();
    dtMode_ = dm < 0.6 ? 0 : dm < 0.9 ? 1 : 2;
    fixedDt_ = rng_.pick({0.01, 0.02, 0.025, 1.0 / 30.0, 0.05, 0.06, 0.1});
    s_.vls = rng_.chance(0.3) ? 0.0 : rng_.range(100.0, 160.0);
    s_.tailstrike = rng_.chance(0.7);
    s_.stabilities = rng_.chance(0.8);
    s_.tracking = rng_.chance(0.03);
    s_.alphaProt = rng_.range(7.0, 15.0);
    s_.alphaMax = s_.alphaProt + rng_.range(1.0, 5.0);
    s_.hspLow = rng_.range(330.0, 370.0);
    s_.hspHigh = s_.hspLow + rng_.range(-5.0, 25.0);
    phase_ = 0;
    timer_ = 0;
    switch (scenario_) {
      case Scenario::Takeoff:
        s_.onGround = true;
        s_.inFlight = 0;
        s_.h = rng_.range(-4.0, 0.0);
        s_.vTas = rng_.chance(0.5) ? 0.0 : rng_.range(0.0, 80.0);
        s_.theta = rng_.range(-1.0, 1.0);
        s_.phi = rng_.range(-1.0, 1.0);
        s_.qk = 0;
        s_.qkDot = 0;
        s_.nz = 1;
        s_.flaps = rng_.intRange(0, 3);
        s_.tl1 = s_.tl2 = rng_.pick({0.0, 25.0, 35.0, 35.0, 45.0, 45.0});
        if (rng_.chance(0.2)) s_.tl2 = rng_.pick({0.0, 25.0, 35.0, 45.0});
        s_.delta = rng_.chance(0.3) ? rng_.range(-1.0, 0.3) : 0.0;
        s_.ap = false;
        s_.aoaProt = s_.hspProt = false;
        s_.etaTrim = rng_.range(-3.0, 3.0);
        accel_ = rng_.range(1.5, 5.0);
        vr_ = rng_.range(110.0, 170.0);
        reject_ = rng_.chance(0.2) ? rng_.range(50.0, 160.0) : 1e9;
        pull_ = rng_.range(0.2, 1.0);
        break;
      case Scenario::Approach:
        s_.onGround = false;
        s_.inFlight = 1;
        s_.h = rng_.range(60.0, 700.0);
        s_.vTas = rng_.range(115.0, 170.0);
        s_.vIas = s_.vTas;
        s_.flaps = rng_.chance(0.5) ? 5 : rng_.intRange(2, 4);
        s_.theta = rng_.range(-2.0, 6.0);
        s_.phi = rng_.range(-10.0, 10.0);
        s_.nz = 1;
        s_.ap = rng_.chance(0.5);
        s_.tl1 = s_.tl2 = rng_.range(0.0, 30.0);
        s_.delta = 0;
        s_.aoaProt = rng_.chance(0.1);
        s_.hspProt = false;
        sink_ = rng_.range(5.0, 18.0);
        apOff_ = rng_.range(-10.0, 400.0);
        goAround_ = rng_.chance(0.15) ? rng_.range(-1.0, 70.0) : -1e9;
        flareAt_ = rng_.range(20.0, 45.0);
        pull_ = rng_.range(0.1, 0.8);
        touchdownDelay_ = rng_.intRange(0, 300);
        break;
      case Scenario::Cruise:
        s_.onGround = false;
        s_.inFlight = rng_.chance(0.95) ? 1 : 0;
        s_.h = rng_.chance(0.3) ? rng_.range(30.0, 500.0) : rng_.range(400.0, 3000.0);
        s_.vTas = rng_.range(130.0, 520.0);
        s_.vIas = s_.vTas * rng_.range(0.55, 1.0);
        s_.flaps = rng_.intRange(0, 5);
        s_.onGround = false;
        break;
      case Scenario::Chaos:
        break;
    }
  }

  void stepDt() {
    if (dtMode_ == 0) {
      s_.dt = rng_.range(0.01, 0.1);
    } else if (dtMode_ == 1) {
      s_.dt = fixedDt_;
    } else {
      s_.dt = rng_.range(0.001, 0.3);
    }
    if (rng_.chance(0.001)) s_.dt = rng_.range(0.1, 1.5);
  }

  void takeoff() {
    const double dt = s_.dt;
    timer_++;
    if (s_.vTas > reject_ && phase_ < 2) {
      phase_ = 9;  // rejected take-off
    }
    if (phase_ == 9) {
      s_.tl1 = s_.tl2 = 0;
      s_.vTas = std::fmax(0.0, s_.vTas - 5.0 * dt);
      toward(s_.delta, 0.0, 1.0);
    } else {
      s_.vTas = std::fmin(320.0, s_.vTas + accel_ * dt);
      if (s_.vTas > vr_) {
        toward(s_.delta, pull_, 0.8, 0.02);
        toward(s_.qk, s_.onGround ? 3.0 : 1.0, 4.0, 0.3);
      } else {
        walk(s_.delta, 0.02, -0.3, 0.3);
        toward(s_.qk, 0.0, 4.0, 0.1);
      }
      if (s_.onGround && phase_ == 0 && s_.theta > rng_.range(6.0, 9.0) && s_.vTas > vr_) {
        phase_ = 1;
        s_.onGround = false;
        liftoffTimer_ = rng_.intRange(0, 200);
      }
      if (phase_ >= 1) {
        s_.h += (s_.theta - 2.0) * s_.vTas * 0.02 * dt + rng_.sym() * 0.3;
        if (liftoffTimer_-- <= 0) s_.inFlight = 1;
        if (s_.h > 100 && phase_ == 1) phase_ = 2;
      }
    }
    s_.theta += s_.qk * dt;
    if (s_.onGround && s_.theta < -1) s_.theta = -1;
    s_.vIas = s_.vTas * rng_.range(0.98, 1.0);
    s_.alpha = s_.onGround ? s_.theta : s_.theta - 2.0 + rng_.sym();
    s_.nz = 1.0 + 0.05 * rng_.sym() + (s_.vTas > vr_ ? 0.15 : 0.0);
    s_.qkDot = rng_.sym() * 2.0;
    s_.mach = s_.vTas / 650.0;
    walk(s_.phi, 0.2, -15.0, 15.0);
    walk(s_.etaTrim, 0.01, -4.0, 4.0);
    walk(s_.eta, 1.0, -30.0, 17.0);
    if (rng_.chance(0.002)) s_.tl1 = rng_.pick({0.0, 25.0, 35.0, 45.0});
    if (rng_.chance(0.002)) s_.tl2 = rng_.pick({0.0, 25.0, 35.0, 45.0});
  }

  void approach() {
    const double dt = s_.dt;
    timer_++;
    if (s_.h < apOff_) s_.ap = false;
    if (phase_ == 0 && s_.h < goAround_) phase_ = 5;
    if (phase_ == 5) {  // go-around
      s_.h += 15.0 * dt + rng_.sym() * 0.5;
      s_.tl1 = s_.tl2 = 45.0;
      toward(s_.theta, 15.0, 3.0, 0.1);
      toward(s_.delta, 0.0, 1.0, 0.05);
      if (s_.h > 300.0 && rng_.chance(0.01)) phase_ = 0, goAround_ = rng_.chance(0.3) ? rng_.range(0.0, 70.0) : -1e9;
    } else if (phase_ == 0) {
      s_.h -= sink_ * dt;
      s_.h += rng_.sym() * 0.4;
      if (s_.h < flareAt_) {
        toward(s_.delta, pull_, 0.5, 0.03);
        toward(s_.theta, 6.0, 1.0, 0.05);
        s_.tl1 = s_.tl2 = std::fmax(0.0, s_.tl1 - 10.0 * dt);
      } else {
        walk(s_.delta, 0.03, -0.4, 0.4);
        walk(s_.theta, 0.05, -3.0, 8.0);
      }
      if (s_.h <= 0.0) {
        phase_ = 1;
        s_.onGround = true;
        s_.h = rng_.range(-3.0, 0.0);
        timer_ = 0;
      }
    } else {  // roll-out
      s_.onGround = true;
      walk(s_.h, 0.1, -4.0, 0.0);
      if (timer_ > touchdownDelay_) s_.inFlight = 0;
      s_.vTas = std::fmax(0.0, s_.vTas - 3.0 * dt);
      toward(s_.theta, 0.0, 2.0, 0.02);
      toward(s_.delta, rng_.chance(0.5) ? 0.0 : -0.5, 0.5, 0.03);
      s_.tl1 = s_.tl2 = 0;
      if (timer_ > 1500 && rng_.chance(0.002)) s_.etaTrim = 0.0;
      if (timer_ > 600 && rng_.chance(0.001)) {  // touch and go
        phase_ = 5;
        s_.onGround = false;
        s_.inFlight = 1;
      }
    }
    s_.qk = rng_.sym() * 3.0 + (s_.delta > 0.1 ? 2.0 : 0.0);
    s_.qkDot = rng_.sym() * 3.0;
    s_.nz = 1.0 + 0.1 * rng_.sym();
    walk(s_.phi, 0.3, -20.0, 20.0);
    walk(s_.vTas, 0.3, 0.0, 200.0);
    s_.vIas = s_.vTas * rng_.range(0.98, 1.0);
    s_.mach = s_.vTas / 650.0;
    s_.alpha = s_.theta + rng_.range(1.0, 5.0);
    walk(s_.etaTrim, 0.01, -4.0, 4.0);
    walk(s_.eta, 1.0, -30.0, 17.0);
    walk(s_.apTheta, 0.2, -5.0, 15.0);
    flip(s_.aoaProt, 0.0005);
    if (rng_.chance(0.0005)) s_.flaps = rng_.intRange(0, 5);
  }

  void cruise() {
    walk(s_.h, 4.0, -5.0, 3000.0);
    walk(s_.vTas, 1.0, 50.0, 560.0);
    s_.vIas = std::fmin(s_.vTas, s_.vIas + rng_.sym() * 1.0);
    if (s_.vIas < 0.5 * s_.vTas) s_.vIas = 0.5 * s_.vTas;
    s_.mach = s_.vTas / rng_.range(590.0, 670.0);
    walk(s_.theta, 0.3, -25.0, 40.0);
    walk(s_.phi, 0.6, -75.0, 75.0);
    walk(s_.nz, 0.04, -1.5, 3.0);
    toward(s_.nz, 1.0, 0.2);
    walk(s_.qk, 0.5, -15.0, 15.0);
    walk(s_.qkDot, 1.0, -20.0, 20.0);
    walk(s_.alpha, 0.2, -5.0, 25.0);
    walk(s_.delta, 0.04, -1.0, 1.0);
    if (rng_.chance(0.002)) s_.delta = rng_.pick({-1.0, 0.0, 0.0, 1.0});
    walk(s_.eta, 0.5, -30.0, 17.0);
    walk(s_.etaTrim, 0.02, -11.0, 3.5);
    walk(s_.apTheta, 0.3, -15.0, 25.0);
    if (rng_.chance(0.003)) s_.splL = rng_.chance(0.5) ? 0.0 : rng_.range(0.0, 45.0);
    if (rng_.chance(0.003)) s_.splR = rng_.chance(0.5) ? s_.splL : rng_.range(0.0, 45.0);
    walk(s_.tl1, 0.5, 0.0, 45.0);
    walk(s_.tl2, 0.5, 0.0, 45.0);
    if (rng_.chance(0.0005)) s_.flaps = rng_.intRange(0, 5);
    flip(s_.aoaProt, 0.002);
    flip(s_.hspProt, 0.002);
    flip(s_.ap, 0.001);
    flip(s_.tracking, 0.0003);
    flip(s_.stabilities, 0.0005);
    flip(s_.tailstrike, 0.0005);
    if (rng_.chance(0.0003)) s_.inFlight = s_.inFlight == 0 ? 1 : 0;
    if (rng_.chance(0.0003)) s_.onGround = !s_.onGround;
    if (rng_.chance(0.001)) {
      s_.hspLow = rng_.range(300.0, 380.0);
      s_.hspHigh = s_.hspLow + rng_.range(-5.0, 25.0);
    }
    if (rng_.chance(0.001)) {
      s_.alphaProt = rng_.range(5.0, 15.0);
      s_.alphaMax = s_.alphaProt + rng_.range(-1.0, 5.0);
    }
  }

  void chaos() {
    walk(s_.h, 30.0, -10.0, 2600.0);
    walk(s_.vTas, 5.0, -10.0, 700.0);
    walk(s_.vIas, 5.0, -10.0, 500.0);
    walk(s_.mach, 0.01, -0.1, 1.0);
    walk(s_.theta, 1.0, -40.0, 50.0);
    walk(s_.phi, 3.0, -180.0, 180.0);
    walk(s_.nz, 0.1, -3.0, 4.0);
    walk(s_.qk, 2.0, -40.0, 40.0);
    walk(s_.qkDot, 5.0, -60.0, 60.0);
    walk(s_.alpha, 1.0, -20.0, 40.0);
    walk(s_.delta, 0.2, -1.2, 1.2);
    walk(s_.eta, 3.0, -35.0, 25.0);
    walk(s_.etaTrim, 0.5, -15.0, 6.0);
    walk(s_.apTheta, 2.0, -30.0, 40.0);
    walk(s_.splL, 3.0, -5.0, 50.0);
    walk(s_.splR, 3.0, -5.0, 50.0);
    walk(s_.tl1, 3.0, -5.0, 50.0);
    walk(s_.tl2, 3.0, -5.0, 50.0);
    walk(s_.vls, 3.0, 0.0, 200.0);
    walk(s_.alphaProt, 0.5, -5.0, 25.0);
    walk(s_.alphaMax, 0.5, -5.0, 30.0);
    walk(s_.hspLow, 3.0, 200.0, 450.0);
    walk(s_.hspHigh, 3.0, 200.0, 450.0);
    if (rng_.chance(0.01)) s_.flaps = rng_.intRange(0, 5);
    if (rng_.chance(0.003)) s_.flaps = rng_.range(-1.0, 6.0);
    if (rng_.chance(0.01)) s_.inFlight = rng_.chance(0.5) ? 0 : 1;
    if (rng_.chance(0.002)) s_.inFlight = rng_.range(-1.0, 2.0);
    flip(s_.onGround, 0.01);
    flip(s_.tracking, 0.003);
    flip(s_.aoaProt, 0.01);
    flip(s_.hspProt, 0.01);
    flip(s_.ap, 0.01);
    flip(s_.tailstrike, 0.01);
    flip(s_.stabilities, 0.01);
  }

  void common() {
    // spoilers in the take-off / approach scenarios (ground spoilers, speed brakes)
    if (scenario_ != Scenario::Chaos && scenario_ != Scenario::Cruise) {
      if (rng_.chance(0.002)) s_.splL = rng_.chance(0.6) ? 0.0 : rng_.range(0.0, 45.0);
      if (rng_.chance(0.002)) s_.splR = rng_.chance(0.5) ? s_.splL : rng_.range(0.0, 45.0);
      flip(s_.tracking, 0.0002);
      if (rng_.chance(0.0005)) s_.tailstrike = !s_.tailstrike;
    }
  }

  // one input of this step replaced by a special value
  void inject() {
    // a failed radio height around the flare mode thresholds (the flare chart's NaN paths)
    if (scenario_ == Scenario::Approach && std::abs(s_.h - 50.0) < 8.0 && rng_.chance(0.05)) {
      out_.h = NAN;
    }
    if (!rng_.chance(0.0015)) return;
    static constexpr double kSpecial[] = {NAN, INFINITY, -INFINITY, 1e300, -1e300, 0.0, -0.0, 1e-300, -1e-300, 1.0};
    const int field = rng_.intRange(0, 32);
    const bool generic = rng_.chance(0.5);
    const double v = rng_.pick(kSpecial);
    auto set = [&](double& x, std::initializer_list<double> thresholds) {
      if (generic || thresholds.size() == 0) {
        x = v;
      } else {
        x = *(thresholds.begin() + rng_.next() % thresholds.size());
      }
    };
    switch (field) {
      case 0: set(out_.dt, {0.0, 0.0, 0.06, 0.1, 0.2, 1.0, 2.0, -0.02}); break;
      case 1: set(out_.nz, {0.5, 1.25}); break;
      case 2: set(out_.theta, {22.5, -15.0, 30.0, 0.0, 6.0, 11.0, 15.0}); break;
      case 3: set(out_.phi, {30.0, -30.0, 33.0, 3.0, 90.0}); break;
      case 4: set(out_.qk, {3.0, 3.0, 0.0, -10.0, 10.0}); break;
      case 5: set(out_.qkDot, {}); break;
      case 6: set(out_.eta, {-3.0, 0.0, 3.0, 10.0, -10.0}); break;
      case 7: set(out_.etaTrim, {0.0}); break;
      case 8: set(out_.alpha, {}); break;
      case 9: set(out_.vIas, {163.0, 243.0, 344.0, 400.0}); break;
      case 10: set(out_.vTas, {70.0, 100.0, 350.0, 400.0, 2000.0, 3000.0}); break;
      case 11: set(out_.mach, {0.0}); break;
      case 12: set(out_.h, {50.0, 30.0, 400.0, 100.0, 0.0, 51.0}); break;
      case 13: set(out_.flaps, {0.0, 5.0, 2.5, -1.0, 6.0}); break;
      case 14: set(out_.splL, {}); break;
      case 15: set(out_.splR, {}); break;
      case 16: set(out_.tl1, {35.0}); break;
      case 17: set(out_.tl2, {35.0}); break;
      case 18: set(out_.vls, {}); break;
      case 19: set(out_.delta, {0.0, -1.0, 1.0, 2.0, -2.0}); break;
      case 20: set(out_.inFlight, {0.0, 1.0, 0.5, 2.0, -1.0}); break;
      case 21: set(out_.alphaProt, {}); break;
      case 22: set(out_.alphaMax, {}); break;
      case 23: set(out_.hspHigh, {}); break;
      case 24: set(out_.hspLow, {}); break;
      case 25: set(out_.apTheta, {}); break;
      case 26: out_.tailstrike = !out_.tailstrike; break;
      case 27: out_.onGround = !out_.onGround; break;
      case 28: out_.tracking = !out_.tracking; break;
      case 29: out_.aoaProt = !out_.aoaProt; break;
      case 30: out_.hspProt = !out_.hspProt; break;
      case 31: out_.ap = !out_.ap; break;
      default: out_.stabilities = !out_.stabilities; break;
    }
  }

  Rng& rng_;
  Signals s_, out_;
  Scenario scenario_ = Scenario::Chaos;
  long left_ = 0;
  bool started_ = false;
  int dtMode_ = 0;
  double fixedDt_ = 0.02;
  int phase_ = 0;
  long timer_ = 0, liftoffTimer_ = 0, touchdownDelay_ = 0;
  double accel_ = 3, vr_ = 140, reject_ = 1e9, pull_ = 0.5, sink_ = 10, apOff_ = 0, goAround_ = -1e9, flareAt_ = 30;
};

R::PitchNormalLawInputs normalInputs(const Signals& s) {
  R::PitchNormalLawInputs in;
  in.time_dt = s.dt;
  in.nz_g = s.nz;
  in.Theta_deg = s.theta;
  in.Phi_deg = s.phi;
  in.qk_deg_s = s.qk;
  in.qk_dot_deg_s2 = s.qkDot;
  in.eta_deg = s.eta;
  in.eta_trim_deg = s.etaTrim;
  in.alpha_deg = s.alpha;
  in.V_ias_kn = s.vIas;
  in.V_tas_kn = s.vTas;
  in.H_radio_ft = s.h;
  in.flaps_handle_index = s.flaps;
  in.spoilers_left_pos = s.splL;
  in.spoilers_right_pos = s.splR;
  in.thrust_lever_1_pos = s.tl1;
  in.thrust_lever_2_pos = s.tl2;
  in.tailstrike_protection_on = s.tailstrike;
  in.VLS_kn = s.vls;
  in.delta_eta_pos = s.delta;
  in.on_ground = s.onGround;
  in.in_flight = s.inFlight;
  in.tracking_mode_on = s.tracking;
  in.high_aoa_prot_active = s.aoaProt;
  in.high_speed_prot_active = s.hspProt;
  in.alpha_prot = s.alphaProt;
  in.alpha_max = s.alphaMax;
  in.high_speed_prot_high_kn = s.hspHigh;
  in.high_speed_prot_low_kn = s.hspLow;
  in.ap_theta_c_deg = s.apTheta;
  in.any_ap_engaged = s.ap;
  return in;
}

R::PitchAlternateLawInputs alternateInputs(const Signals& s) {
  R::PitchAlternateLawInputs in;
  in.time_dt = s.dt;
  in.nz_g = s.nz;
  in.Theta_deg = s.theta;
  in.Phi_deg = s.phi;
  in.qk_deg_s = s.qk;
  in.eta_deg = s.eta;
  in.eta_trim_deg = s.etaTrim;
  in.V_ias_kn = s.vIas;
  in.mach = s.mach;
  in.V_tas_kn = s.vTas;
  in.flaps_handle_index = s.flaps;
  in.spoilers_left_pos = s.splL;
  in.spoilers_right_pos = s.splR;
  in.delta_eta_pos = s.delta;
  in.in_flight = s.inFlight;
  in.tracking_mode_on = s.tracking;
  in.stabilities_available = s.stabilities;
  return in;
}

// ---------------------------------------------------------------------------------------------------------------------
// Steppers: the generated law called the way the ELAC calls it
// ---------------------------------------------------------------------------------------------------------------------

R::PitchLawOutputs stepGenerated(::PitchNormalLaw& g, const R::PitchNormalLawInputs& i) {
  R::PitchLawOutputs o;
  const boolean_T tailstrike = i.tailstrike_protection_on, onGround = i.on_ground, tracking = i.tracking_mode_on;
  const boolean_T aoaProt = i.high_aoa_prot_active, hspProt = i.high_speed_prot_active, ap = i.any_ap_engaged;
  g.step(&i.time_dt, &i.nz_g, &i.Theta_deg, &i.Phi_deg, &i.qk_deg_s, &i.qk_dot_deg_s2, &i.eta_deg, &i.eta_trim_deg,
         &i.alpha_deg, &i.V_ias_kn, &i.V_tas_kn, &i.H_radio_ft, &i.flaps_handle_index, &i.spoilers_left_pos,
         &i.spoilers_right_pos, &i.thrust_lever_1_pos, &i.thrust_lever_2_pos, &tailstrike, &i.VLS_kn,
         &i.delta_eta_pos, &onGround, &i.in_flight, &tracking, &aoaProt, &hspProt, &i.alpha_prot, &i.alpha_max,
         &i.high_speed_prot_high_kn, &i.high_speed_prot_low_kn, &i.ap_theta_c_deg, &ap, &o.eta_deg,
         &o.eta_trim_dot_deg_s, &o.eta_trim_limit_lo, &o.eta_trim_limit_up);
  return o;
}

R::PitchLawOutputs stepGenerated(::PitchAlternateLaw& g, const R::PitchAlternateLawInputs& i) {
  R::PitchLawOutputs o;
  const boolean_T tracking = i.tracking_mode_on, stabilities = i.stabilities_available;
  g.step(&i.time_dt, &i.nz_g, &i.Theta_deg, &i.Phi_deg, &i.qk_deg_s, &i.eta_deg, &i.eta_trim_deg, &i.V_ias_kn,
         &i.mach, &i.V_tas_kn, &i.flaps_handle_index, &i.spoilers_left_pos, &i.spoilers_right_pos, &i.delta_eta_pos,
         &i.in_flight, &tracking, &stabilities, &o.eta_deg, &o.eta_trim_dot_deg_s,
         &o.eta_trim_limit_lo, &o.eta_trim_limit_up);
  return o;
}

R::PitchLawOutputs stepGenerated(::PitchDirectLaw& g, const R::PitchDirectLawInputs& i) {
  R::PitchLawOutputs o;
  g.step(&i.time_dt, &i.delta_eta_pos, &o.eta_deg, &o.eta_trim_dot_deg_s, &o.eta_trim_limit_lo, &o.eta_trim_limit_up);
  return o;
}

// ---------------------------------------------------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------------------------------------------------

bool same(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0 || (a != a && b != b); }

struct FirstMismatch {
  long step;
  double generated, readable;
  long count;
};

struct LawReport {
  const char* law;
  long steps = 0, badSteps = 0, compares = 0, nanEta = 0;
  std::map<std::string, FirstMismatch> fields;  // insertion-ordered by name; first mismatch per field

  void check(long step, const char* name, double g, double r, bool& bad) {
    compares++;
    if (same(g, r)) return;
    bad = true;
    auto it = fields.find(name);
    if (it == fields.end()) {
      fields.emplace(name, FirstMismatch{step, g, r, 1});
    } else {
      it->second.count++;
    }
  }

  void compareOutputs(long step, const R::PitchLawOutputs& g, const R::PitchLawOutputs& r, bool& bad) {
    nanEta += g.eta_deg != g.eta_deg;
    check(step, "out.eta_deg", g.eta_deg, r.eta_deg, bad);
    check(step, "out.eta_trim_dot_deg_s", g.eta_trim_dot_deg_s, r.eta_trim_dot_deg_s, bad);
    check(step, "out.eta_trim_limit_lo", g.eta_trim_limit_lo, r.eta_trim_limit_lo, bad);
    check(step, "out.eta_trim_limit_up", g.eta_trim_limit_up, r.eta_trim_limit_up, bad);
  }
};

template <class G, class Rd, class Pairs>
void compareState(LawReport& rep, long step, const G& g, const Rd& r, Pairs pairs, bool& bad) {
  pairs(g, r, [&](const char* name, double gv, double rv) { rep.check(step, name, gv, rv, bad); });
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 1;
  long steps = 100000;
  std::string law = "all";
  int maxListed = 40;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--seed" && i + 1 < argc) {
      seed = std::strtoull(argv[++i], nullptr, 10);
    } else if (a == "--steps" && i + 1 < argc) {
      steps = std::atol(argv[++i]);
    } else if (a == "--law" && i + 1 < argc) {
      law = argv[++i];
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr,
                   "usage: efcs-pitch-law-check [--seed S] [--steps N] [--law normal|alternate|direct|all] "
                   "[--list N]\n");
      return 2;
    }
  }
  const bool runNormal = law == "all" || law == "normal";
  const bool runAlternate = law == "all" || law == "alternate";
  const bool runDirect = law == "all" || law == "direct";
  if (!runNormal && !runAlternate && !runDirect) {
    std::fprintf(stderr, "unknown law %s\n", law.c_str());
    return 2;
  }

  auto gNormal = std::make_unique<::PitchNormalLaw>();
  auto gAlternate = std::make_unique<::PitchAlternateLaw>();
  auto gDirect = std::make_unique<::PitchDirectLaw>();
  auto rNormal = std::make_unique<R::PitchNormalLaw>();
  auto rAlternate = std::make_unique<R::PitchAlternateLaw>();
  auto rDirect = std::make_unique<R::PitchDirectLaw>();
  LawReport reps[3] = {{"normal"}, {"alternate"}, {"direct"}};

  Rng rng(seed);
  Rng eventRng(seed ^ 0xA5A5A5A5DEADBEEFULL);
  Trajectory traj(rng);

  // ElacComputer::initialize(): init() of the normal and alternate laws (the direct law has none)
  gNormal->init();
  rNormal->init();
  gAlternate->init();
  rAlternate->init();
  long resets = 0, inits = 0;

  for (long step = 0; step < steps; step++) {
    // an ELAC restart (always on the first running frame, which may or may not be the first step)
    const bool doReset = step == 0 ? eventRng.chance(0.5) : eventRng.chance(1.0 / 20000.0);
    if (doReset) {
      resets++;
      gNormal->reset();
      rNormal->reset();
      gAlternate->reset();
      rAlternate->reset();
      gDirect->reset();
      rDirect->reset();
    }
    if (step > 0 && eventRng.chance(1.0 / 20000.0)) {  // init() again mid-run (not a full reset)
      inits++;
      gNormal->init();
      rNormal->init();
      gAlternate->init();
      rAlternate->init();
    }
    const Signals& s = traj.next();

    if (runNormal) {
      LawReport& rep = reps[0];
      bool bad = false;
      const R::PitchNormalLawInputs in = normalInputs(s);
      const R::PitchLawOutputs go = stepGenerated(*gNormal, in);
      const R::PitchLawOutputs ro = rNormal->step(in);
      rep.compareOutputs(step, go, ro, bad);
      compareState(rep, step, *gNormal, *rNormal,
                   [](const auto& g, const auto& r, auto&& add) { pitchNormalLawPairs(g, r, add); }, bad);
      rep.steps++;
      rep.badSteps += bad;
    }
    if (runAlternate) {
      LawReport& rep = reps[1];
      bool bad = false;
      const R::PitchAlternateLawInputs in = alternateInputs(s);
      const R::PitchLawOutputs go = stepGenerated(*gAlternate, in);
      const R::PitchLawOutputs ro = rAlternate->step(in);
      rep.compareOutputs(step, go, ro, bad);
      compareState(rep, step, *gAlternate, *rAlternate,
                   [](const auto& g, const auto& r, auto&& add) { pitchAlternateLawPairs(g, r, add); }, bad);
      rep.steps++;
      rep.badSteps += bad;
    }
    if (runDirect) {
      LawReport& rep = reps[2];
      bool bad = false;
      R::PitchDirectLawInputs in;
      in.time_dt = s.dt;
      in.delta_eta_pos = s.delta;
      const R::PitchLawOutputs go = stepGenerated(*gDirect, in);
      const R::PitchLawOutputs ro = rDirect->step(in);
      rep.compareOutputs(step, go, ro, bad);
      compareState(rep, step, *gDirect, *rDirect,
                   [](const auto& g, const auto& r, auto&& add) { pitchDirectLawPairs(g, r, add); }, bad);
      rep.steps++;
      rep.badSteps += bad;
    }
  }

  long totalBad = 0;
  std::printf("seed %llu, %ld steps, %ld resets, %ld extra inits\n", static_cast<unsigned long long>(seed), steps,
              resets, inits);
  for (const LawReport& rep : reps) {
    if (rep.steps == 0) continue;
    totalBad += rep.badSteps;
    std::printf("%-9s %ld steps (eta NaN on %ld), %ld compares/step, %ld mismatching steps, %zu mismatching fields\n",
                rep.law, rep.steps, rep.nanEta, rep.compares / rep.steps, rep.badSteps, rep.fields.size());
    // first mismatches in step order
    std::multimap<long, std::pair<std::string, FirstMismatch>> byStep;
    for (const auto& [name, m] : rep.fields) byStep.emplace(m.step, std::make_pair(name, m));
    int listed = 0;
    for (const auto& [st, nm] : byStep) {
      if (listed++ >= maxListed) {
        std::printf("  ...\n");
        break;
      }
      std::printf("  %-70s first at step %ld: generated %.17g, readable %.17g (%ld steps)\n", nm.first.c_str(), st,
                  nm.second.generated, nm.second.readable, nm.second.count);
    }
  }
  std::printf("%s\n", totalBad == 0 ? "OK: all outputs and memories matched" : "MISMATCH");
  return totalBad == 0 ? 0 : 1;
}
