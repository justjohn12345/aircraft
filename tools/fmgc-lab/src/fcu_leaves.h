// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Every scalar leaf of an FCU struct (byte offset, size, kind, name), from the visitor generated from
// FcuComputer_types.h (fcu_fields.gen.h), for bit-exact field-by-field comparison (padding ignored): bools, enums and
// SSMs as integers, floats and doubles by bit pattern, so NaNs compare equal only with the same payload.

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

// The field visitor: fcu_fields.gen.h unless the includer has already included another one (LEAVES_FIELDS_INCLUDED).
#ifndef LEAVES_FIELDS_INCLUDED
#include "fcu_fields.gen.h"
#endif

struct FcuLeaf {
  std::string name;
  size_t offset;
  size_t size;
  FieldKind kind;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  std::string firstExpected, firstGot;
  // coverage of the expected values (running steps)
  long nonzero = 0;
  long changes = 0;
  unsigned char prev[8]{};
  bool havePrev = false;
};

struct FcuLeafCollector {
  std::vector<FcuLeaf>& leaves;
  const void* base;
  template <typename T>
  void field(const char* p, const char* name, T& a, T& /*b*/, FieldKind kind) {
    leaves.push_back({std::string(p) + name,
                      static_cast<size_t>(reinterpret_cast<const char*>(&a) - static_cast<const char*>(base)),
                      sizeof(T), kind});
  }
};

template <typename S>
std::vector<FcuLeaf> fcuLeaves(const char* prefix = "") {
  std::vector<FcuLeaf> leaves;
  auto a = std::make_unique<S>();
  auto b = std::make_unique<S>();
  FcuLeafCollector c{leaves, a.get()};
  visit(c, prefix, *a, *b);
  return leaves;
}

inline std::string showFcuLeaf(const FcuLeaf& l, const void* p) {
  const char* c = static_cast<const char*>(p) + l.offset;
  char buf[64];
  if (l.kind == FieldKind::Bool) {
    std::snprintf(buf, sizeof buf, "%d", *reinterpret_cast<const bool*>(c) ? 1 : 0);
  } else if (l.kind == FieldKind::Float && l.size == 4) {
    float v;
    std::memcpy(&v, c, 4);
    std::snprintf(buf, sizeof buf, "%.9g", v);
  } else if (l.kind == FieldKind::Float && l.size == 8) {
    double v;
    std::memcpy(&v, c, 8);
    std::snprintf(buf, sizeof buf, "%.17g", v);
  } else {
    int32_t v = 0;
    std::memcpy(&v, c, l.size < 4 ? l.size : 4);
    if (l.size == 1) v = *reinterpret_cast<const int8_t*>(c);
    std::snprintf(buf, sizeof buf, "%d", v);
  }
  return buf;
}

/**
 * Compares `got` with `expected` leaf by leaf; counts mismatches (first one: step, time, values). Optionally records
 * coverage of the expected values. Returns the number of differing leaves.
 */
template <typename S>
int compareFcuLeaves(std::vector<FcuLeaf>& leaves, const S& expected, const S& got, long step, double time,
                     bool coverage) {
  const char* e = reinterpret_cast<const char*>(&expected);
  const char* g = reinterpret_cast<const char*>(&got);
  int n = 0;
  static const unsigned char zero[8]{};
  for (FcuLeaf& l : leaves) {
    if (coverage) {
      l.nonzero += std::memcmp(e + l.offset, zero, l.size) != 0;
      l.changes += l.havePrev && std::memcmp(e + l.offset, l.prev, l.size) != 0;
      std::memcpy(l.prev, e + l.offset, l.size);
      l.havePrev = true;
    }
    if (std::memcmp(e + l.offset, g + l.offset, l.size) != 0) {
      if (l.firstStep < 0) {
        l.firstStep = step;
        l.firstTime = time;
        l.firstExpected = showFcuLeaf(l, e);
        l.firstGot = showFcuLeaf(l, g);
      }
      l.mismatches++;
      n++;
    }
  }
  return n;
}
