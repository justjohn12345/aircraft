// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Leaf-by-leaf comparison of recorded and replayed outputs keeping the largest deviation per leaf (units in the last
// place and absolute), for the EFCS replays (the same helpers as src/fac_replay.cpp). Include after fcu_leaves.h.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace replay_compare {

/** Distance in units in the last place between two floats or doubles (across zero too); 1e18 for other kinds. */
inline double ulps(const FcuLeaf& l, const void* a, const void* b) {
  const char* x = static_cast<const char*>(a) + l.offset;
  const char* y = static_cast<const char*>(b) + l.offset;
  if (l.kind != FieldKind::Float) return 1e18;
  __int128 i, j;
  if (l.size == 4) {
    int32_t u, v;
    std::memcpy(&u, x, 4);
    std::memcpy(&v, y, 4);
    i = u < 0 ? -static_cast<__int128>(u & 0x7fffffff) : u;
    j = v < 0 ? -static_cast<__int128>(v & 0x7fffffff) : v;
  } else {
    int64_t u, v;
    std::memcpy(&u, x, 8);
    std::memcpy(&v, y, 8);
    i = u < 0 ? -static_cast<__int128>(u & 0x7fffffffffffffffLL) : u;
    j = v < 0 ? -static_cast<__int128>(v & 0x7fffffffffffffffLL) : v;
  }
  return static_cast<double>(i > j ? i - j : j - i);
}

inline double numeric(const FcuLeaf& l, const void* p) {
  const char* c = static_cast<const char*>(p) + l.offset;
  if (l.kind == FieldKind::Float && l.size == 4) {
    float v;
    std::memcpy(&v, c, 4);
    return v;
  }
  if (l.kind == FieldKind::Float && l.size == 8) {
    double v;
    std::memcpy(&v, c, 8);
    return v;
  }
  return 0.0;
}

struct Deviation {
  double ulp = 0;
  double abs = 0;
};

/** compareFcuLeaves, also keeping the largest deviation of each leaf's mismatches (indexed like the leaves). */
template <typename S>
int compareWithUlps(std::vector<FcuLeaf>& leaves, std::vector<Deviation>& dev, const S& expected, const S& got,
                    long frame, double t) {
  if (dev.size() != leaves.size()) dev.assign(leaves.size(), Deviation{});
  const int n = compareFcuLeaves(leaves, expected, got, frame, t, true);
  if (n == 0) return 0;
  for (size_t k = 0; k < leaves.size(); k++) {
    const FcuLeaf& l = leaves[k];
    if (std::memcmp(reinterpret_cast<const char*>(&expected) + l.offset, reinterpret_cast<const char*>(&got) + l.offset,
                    l.size) != 0) {
      dev[k].ulp = std::fmax(dev[k].ulp, ulps(l, &expected, &got));
      dev[k].abs = std::fmax(dev[k].abs, std::fabs(numeric(l, &expected) - numeric(l, &got)));
    }
  }
  return n;
}

}  // namespace replay_compare
