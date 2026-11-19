// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic, seeded input trajectories for the law-level differential checkers: bounded random walks with occasional
// jumps, booleans with random dwell times, frame times, and rare special values (NaN, +-inf, huge, 0, -0).

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace law_trajectory {

class Rng {
 public:
  explicit Rng(uint64_t seed) : g_(seed) {}
  double uniform(double a, double b) { return std::uniform_real_distribution<double>(a, b)(g_); }
  bool chance(double p) { return uniform(0.0, 1.0) < p; }
  int integer(int a, int b) { return std::uniform_int_distribution<int>(a, b)(g_); }

 private:
  std::mt19937_64 g_;
};

/** A special floating-point value (NaN, +-inf, huge, tiny, zero, -0). */
inline double special(Rng& rng) {
  switch (rng.integer(0, 7)) {
    case 0: return std::numeric_limits<double>::quiet_NaN();
    case 1: return -std::numeric_limits<double>::quiet_NaN();
    case 2: return std::numeric_limits<double>::infinity();
    case 3: return -std::numeric_limits<double>::infinity();
    case 4: return 1e300;
    case 5: return -1e300;
    case 6: return 1e-310;
    default: return rng.chance(0.5) ? 0.0 : -0.0;
  }
}

/**
 * A bounded random walk: each step moves by up to `step`; with probability `jump` it jumps anywhere in [lo, hi], with
 * `hold` it holds exactly, with `exact` it lands on one of the `marks` (thresholds) exactly; `specialRate` gives a
 * special value for one frame.
 */
struct Walk {
  double lo, hi, step;
  double value;
  std::vector<double> marks;
  double jump = 0.002, specialRate = 0.0;
  bool integer = false;

  Walk(double lo_, double hi_, double step_, std::initializer_list<double> marks_ = {}, double specialRate_ = 0.0)
      : lo(lo_), hi(hi_), step(step_), value((lo_ + hi_) / 2), marks(marks_), specialRate(specialRate_) {}

  double next(Rng& rng) {
    if (specialRate > 0 && rng.chance(specialRate)) return special(rng);
    const double p = rng.uniform(0.0, 1.0);
    if (p < jump) {
      value = rng.uniform(lo, hi);
    } else if (p < jump + 0.01 && marks.size() > 0) {
      value = marks[rng.integer(0, static_cast<int>(marks.size()) - 1)];
    } else if (p < jump + 0.2) {
      // hold
    } else {
      value += rng.uniform(-step, step);
      if (value < lo) value = lo + (lo - value);
      if (value > hi) value = hi - (value - hi);
      if (value < lo || value > hi) value = (lo + hi) / 2;
    }
    if (integer) return std::round(value);
    return value;
  }
};

/** A boolean that flips after random dwell times (mean `meanDwell` steps). */
struct Toggle {
  double meanDwell;
  bool value = false;
  explicit Toggle(double meanDwell_, bool initial = false) : meanDwell(meanDwell_), value(initial) {}
  bool next(Rng& rng) {
    if (rng.chance(1.0 / meanDwell)) value = !value;
    return value;
  }
};

/** Frame times: mostly 1/30 .. 1/60 s, sometimes 0, tiny, large; rarely special. */
inline double frameTime(Rng& rng, double specialRate = 0.0) {
  if (specialRate > 0 && rng.chance(specialRate)) return special(rng);
  const double p = rng.uniform(0.0, 1.0);
  if (p < 0.002) return 0.0;
  if (p < 0.004) return 1e-6;
  if (p < 0.01) return rng.uniform(0.1, 2.0);
  return rng.uniform(1.0 / 70, 1.0 / 20);
}

/** Bit-exact comparison of two doubles (two NaNs match whatever their sign and payload). */
inline bool same(double a, double b) {
  if (a != a && b != b) return true;
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

}  // namespace law_trajectory
