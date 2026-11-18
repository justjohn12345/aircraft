// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Generic input injectors for the EFCS differential checks (src/check_elac.cpp, src/check_sec.cpp), applied on top of
// the drive at every step. They work on any model input struct with `time`, `sim_data`, `discrete_inputs` (booleans),
// `analog_inputs` (doubles) and `bus_inputs` (ARINC words only), for N computers. The generated model is the oracle,
// so any input is a valid test input; inconsistent combinations are wanted. Each is reproducible from its seed.
//
//  --failures SEED   episodes (0.1-20 s, one computer or all): ARINC words get a random SSM and optionally random
//                    data (NaN, infinities, 0, huge, scaled, random discrete bits)
//  --discretes SEED  episodes: discrete inputs flipped (the unit discretes included), analog inputs replaced (NaN,
//                    infinities, 0, +-full scale, scaled), bits of ARINC discrete words flipped
//  --power SEED      computer_running false for 1 frame, 2-10 frames, 0.2-5 s or 5-60 s, flicker, one or all
//  --sim SEED        dt 0, tiny, 0.2-2 s, NaN; simulation / monotonic time jumps; slew / pause / tracking override /
//                    tailstrike protection flags

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace efcs_scenarios {

class Rng {
 public:
  explicit Rng(uint64_t seed) : rng_(seed) {}
  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng_); }
  bool chance(double p) { return u(0, 1) < p; }
  int irand(int n) { return static_cast<int>(rng_() % static_cast<uint64_t>(n)); }
  uint64_t raw() { return rng_(); }

 private:
  std::mt19937_64 rng_;
};

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

template <class In>
base_arinc_429* words(In& in) {
  return reinterpret_cast<base_arinc_429*>(&in.bus_inputs);
}
template <class In>
constexpr size_t wordCount() {
  return sizeof(In::bus_inputs) / sizeof(base_arinc_429);
}
template <class In>
unsigned char* discretes(In& in) {
  return reinterpret_cast<unsigned char*>(&in.discrete_inputs);
}
template <class In>
constexpr size_t discreteCount() {
  return sizeof(In::discrete_inputs);
}
template <class In>
double* analogs(In& in) {
  return reinterpret_cast<double*>(&in.analog_inputs);
}
template <class In>
constexpr size_t analogCount() {
  return sizeof(In::analog_inputs) / sizeof(double);
}

/** A random replacement for a value: special values or a scaled / offset copy. */
inline double weird(Rng& r, double v) {
  switch (r.irand(9)) {
    case 0: return kNaN;
    case 1: return -kNaN;
    case 2: return kInf;
    case 3: return -kInf;
    case 4: return 0.0;
    case 5: return r.chance(0.5) ? 1e9 : -1e9;
    case 6: return v * r.u(-3, 3);
    case 7: return v + r.u(-100, 100);
    default: return std::round(r.u(0, 4294967295.0));  // random discrete bits
  }
}

/** One episode: a set of targets held for a duration on one computer (or all). */
struct Episode {
  double remaining = 0;
  int computer = -1;  // -1: all
  std::vector<int> targets;
  std::vector<uint32_t> ssm;
  std::vector<double> data;
  std::vector<bool> keepData;
};

template <class In, int N>
class FailureScenario {
 public:
  explicit FailureScenario(uint64_t seed) : rng_(seed) {}
  long episodes() const { return episodes_; }

  void apply(In (&in)[N], double dt) {
    if (e_.remaining <= 0 && rng_.chance(0.01)) start();
    if (e_.remaining <= 0) return;
    e_.remaining -= std::isfinite(dt) && dt > 0 ? dt : 0.02;
    for (int c = 0; c < N; c++) {
      if (e_.computer >= 0 && e_.computer != c) continue;
      base_arinc_429* w = words(in[c]);
      for (size_t k = 0; k < e_.targets.size(); k++) {
        base_arinc_429& x = w[e_.targets[k]];
        x.SSM = e_.ssm[k];
        if (!e_.keepData[k]) x.Data = static_cast<float>(e_.data[k]);
      }
    }
  }

 private:
  void start() {
    episodes_++;
    e_ = Episode{};
    e_.remaining = rng_.chance(0.3) ? rng_.u(0.1, 1.0) : rng_.u(1.0, 20.0);
    e_.computer = rng_.chance(0.5) ? -1 : rng_.irand(N);
    const int n = rng_.chance(0.5) ? 1 : 1 + rng_.irand(12);
    for (int k = 0; k < n; k++) {
      e_.targets.push_back(rng_.irand(static_cast<int>(wordCount<In>())));
      e_.ssm.push_back(static_cast<uint32_t>(rng_.chance(0.1) ? rng_.irand(16) : rng_.irand(4)));
      const bool keep = rng_.chance(0.5);
      e_.keepData.push_back(keep);
      e_.data.push_back(weird(rng_, rng_.u(-500, 500)));
    }
  }

  Rng rng_;
  Episode e_;
  long episodes_ = 0;
};

template <class In, int N>
class DiscreteScenario {
 public:
  explicit DiscreteScenario(uint64_t seed) : rng_(seed) {}
  long episodes() const { return episodes_; }

  void apply(In (&in)[N], double dt) {
    if (remaining_ <= 0 && rng_.chance(0.01)) start();
    if (remaining_ <= 0) return;
    remaining_ -= std::isfinite(dt) && dt > 0 ? dt : 0.02;
    for (int c = 0; c < N; c++) {
      if (computer_ >= 0 && computer_ != c) continue;
      unsigned char* d = discretes(in[c]);
      for (int k : flips_) d[k] = !d[k];
      double* a = analogs(in[c]);
      for (size_t k = 0; k < analogTargets_.size(); k++) a[analogTargets_[k]] = analogValues_[k];
      base_arinc_429* w = words(in[c]);
      for (size_t k = 0; k < bitWords_.size(); k++) {
        float r = std::round(w[bitWords_[k]].Data);
        uint32_t x = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
        w[bitWords_[k]].Data = static_cast<float>(x ^ (1U << bitNumbers_[k]));
      }
    }
  }

 private:
  void start() {
    episodes_++;
    remaining_ = rng_.chance(0.3) ? rng_.u(0.02, 1.0) : rng_.u(1.0, 40.0);
    computer_ = rng_.chance(0.5) ? -1 : rng_.irand(N);
    flips_.clear();
    analogTargets_.clear();
    analogValues_.clear();
    bitWords_.clear();
    bitNumbers_.clear();
    const int kind = rng_.irand(4);
    if (kind == 0 || kind == 3) {
      const int n = 1 + rng_.irand(rng_.chance(0.5) ? 2 : 6);
      for (int k = 0; k < n; k++) flips_.push_back(rng_.irand(static_cast<int>(discreteCount<In>())));
    }
    if (kind == 1 || kind == 3) {
      const int n = 1 + rng_.irand(3);
      for (int k = 0; k < n; k++) {
        analogTargets_.push_back(rng_.irand(static_cast<int>(analogCount<In>())));
        const double scale[] = {1.0, 0.5, 0.25, 30.0, 3000.0};
        analogValues_.push_back(rng_.chance(0.3) ? weird(rng_, 1.0) : rng_.u(-1, 1) * scale[rng_.irand(5)]);
      }
    }
    if (kind == 2 || kind == 3) {
      const int n = 1 + rng_.irand(4);
      for (int k = 0; k < n; k++) {
        bitWords_.push_back(rng_.irand(static_cast<int>(wordCount<In>())));
        bitNumbers_.push_back(10 + rng_.irand(19));  // ARINC bits 11..29
      }
    }
  }

  Rng rng_;
  double remaining_ = 0;
  int computer_ = -1;
  std::vector<int> flips_, analogTargets_, bitWords_, bitNumbers_;
  std::vector<double> analogValues_;
  long episodes_ = 0;
};

template <class In, int N>
class PowerScenario {
 public:
  explicit PowerScenario(uint64_t seed) : rng_(seed) {}
  long episodes() const { return episodes_; }

  void apply(In (&in)[N], double dt) {
    if (remaining_ <= 0 && frames_ <= 0 && rng_.chance(0.002)) start();
    const bool off = frames_ > 0 || remaining_ > 0;
    if (frames_ > 0) frames_--;
    if (remaining_ > 0) remaining_ -= std::isfinite(dt) && dt > 0 ? dt : 0.02;
    if (flicker_ && off) flickerState_ = !flickerState_;
    if (!off) return;
    for (int c = 0; c < N; c++) {
      if (computer_ >= 0 && computer_ != c) continue;
      in[c].sim_data.computer_running = flicker_ ? flickerState_ : false;
    }
  }

 private:
  void start() {
    episodes_++;
    computer_ = rng_.chance(0.5) ? -1 : rng_.irand(N);
    flicker_ = rng_.chance(0.15);
    switch (rng_.irand(4)) {
      case 0: frames_ = 1; break;
      case 1: frames_ = 2 + rng_.irand(9); break;
      case 2: remaining_ = rng_.u(0.2, 5.0); break;
      default: remaining_ = rng_.u(5.0, 60.0); break;
    }
  }

  Rng rng_;
  double remaining_ = 0;
  int frames_ = 0, computer_ = -1;
  bool flicker_ = false, flickerState_ = false;
  long episodes_ = 0;
};

template <class In, int N>
class SimScenario {
 public:
  explicit SimScenario(uint64_t seed) : rng_(seed) {}
  long episodes() const { return episodes_; }

  void apply(In (&in)[N], double dt) {
    if (remaining_ <= 0 && rng_.chance(0.005)) start();
    for (int c = 0; c < N; c++) {
      in[c].time.simulation_time += timeOffset_;
      in[c].time.monotonic_time += timeOffset_;
      in[c].sim_data.tailstrike_protection_on = in[c].sim_data.tailstrike_protection_on != tailstrike_;
    }
    if (remaining_ <= 0) return;
    remaining_ -= std::isfinite(dt) && dt > 0 ? dt : 0.02;
    for (int c = 0; c < N; c++) {
      switch (kind_) {
        case 0: in[c].time.dt = 0.0; break;
        case 1: in[c].time.dt = 1e-6; break;
        case 2: in[c].time.dt = dtValue_; break;
        case 3: in[c].time.dt = kNaN; break;
        case 4: in[c].sim_data.slew_on = true; break;
        case 5: in[c].sim_data.pause_on = true; break;
        default: in[c].sim_data.tracking_mode_on_override = true; break;
      }
    }
  }

 private:
  void start() {
    episodes_++;
    kind_ = rng_.irand(8);
    remaining_ = kind_ <= 3 ? rng_.u(0.02, 0.3) : rng_.u(0.05, 5.0);
    dtValue_ = rng_.u(0.2, 2.0);
    if (rng_.chance(0.15)) timeOffset_ += rng_.chance(0.5) ? rng_.u(0.0, 30.0) : -rng_.u(0.0, 5.0);
    if (rng_.chance(0.1)) tailstrike_ = !tailstrike_;
  }

  Rng rng_;
  double remaining_ = 0, dtValue_ = 0.5, timeOffset_ = 0;
  int kind_ = 0;
  bool tailstrike_ = false;
  long episodes_ = 0;
};

}  // namespace efcs_scenarios
