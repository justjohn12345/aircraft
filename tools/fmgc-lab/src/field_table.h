// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Flat field tables (byte offset, size, kind, full name) of the generated model's I/O structs, built once from the
// visitor in fields.gen.h. The visitor allocates a std::string per nested struct on every call, which is far too slow
// for per-frame work in the fuzzer; these tables let the fuzzer address input fields by index and compare output structs
// field by field (bit-exact, padding ignored) at a few nanoseconds per field.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include "FmgcComputer_types.h"
#include "fields.gen.h"

struct FieldInfo {
  std::string name;
  size_t offset;
  size_t size;
  FieldKind kind;
  bool isSsm;   // ARINC word SSM
  bool isData;  // ARINC word data
};

template <typename S>
class FieldTable {
 public:
  static const FieldTable& get() {
    static const FieldTable table;
    return table;
  }

  const std::vector<FieldInfo>& fields() const { return fields_; }

  /** Index of the first field whose name equals `name`, or -1. */
  int find(const std::string& name) const {
    for (size_t i = 0; i < fields_.size(); i++) {
      if (fields_[i].name == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  /** Bit-exact comparison field by field; returns the index of the first differing field, or -1. */
  int firstDifference(const S& a, const S& b) const {
    const auto* pa = reinterpret_cast<const unsigned char*>(&a);
    const auto* pb = reinterpret_cast<const unsigned char*>(&b);
    for (size_t i = 0; i < fields_.size(); i++) {
      const FieldInfo& f = fields_[i];
      if (std::memcmp(pa + f.offset, pb + f.offset, f.size) != 0) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  /** Value of a field as a double (for reports and mutations). */
  double value(const S& s, size_t i) const {
    const FieldInfo& f = fields_[i];
    const auto* p = reinterpret_cast<const unsigned char*>(&s) + f.offset;
    return read(p, f);
  }

  void setValue(S& s, size_t i, double v) const {
    const FieldInfo& f = fields_[i];
    auto* p = reinterpret_cast<unsigned char*>(&s) + f.offset;
    write(p, f, v);
  }

 private:
  FieldTable() {
    static S probe;  // only addresses are used
    Collector c{fields_, reinterpret_cast<const unsigned char*>(&probe)};
    visit(c, "", probe, probe);
  }

  struct Collector {
    std::vector<FieldInfo>& out;
    const unsigned char* base;
    template <typename T>
    void field(const char* prefix, const char* name, T& a, T&, FieldKind kind) {
      std::string full = std::string(prefix) + name;
      size_t n = full.size();
      bool ssm = n >= 4 && full.compare(n - 4, 4, ".SSM") == 0;
      bool data = n >= 5 && full.compare(n - 5, 5, ".Data") == 0;
      size_t off = static_cast<size_t>(reinterpret_cast<const unsigned char*>(&a) - base);
      out.push_back({full, off, sizeof(T), kind, ssm, data});
    }
  };

  static double read(const unsigned char* p, const FieldInfo& f) {
    switch (f.kind) {
      case FieldKind::Bool: {
        bool v;
        std::memcpy(&v, p, 1);
        return v ? 1.0 : 0.0;
      }
      case FieldKind::Float:
        if (f.size == 4) {
          float v;
          std::memcpy(&v, p, 4);
          return v;
        } else {
          double v;
          std::memcpy(&v, p, 8);
          return v;
        }
      default:
        if (f.size == 1) {
          int8_t v;
          std::memcpy(&v, p, 1);
          return v;
        } else if (f.isSsm) {
          uint32_t v;
          std::memcpy(&v, p, 4);
          return v;
        } else {
          int32_t v;  // int32 enums
          std::memcpy(&v, p, 4);
          return v;
        }
    }
  }

  static void write(unsigned char* p, const FieldInfo& f, double v) {
    switch (f.kind) {
      case FieldKind::Bool: {
        bool b = v != 0;
        std::memcpy(p, &b, 1);
        break;
      }
      case FieldKind::Float:
        if (f.size == 4) {
          float x = static_cast<float>(v);
          std::memcpy(p, &x, 4);
        } else {
          std::memcpy(p, &v, 8);
        }
        break;
      default:
        if (f.size == 1) {
          int8_t x = static_cast<int8_t>(v);
          std::memcpy(p, &x, 1);
        } else if (f.isSsm) {
          uint32_t x = static_cast<uint32_t>(static_cast<int64_t>(v));
          std::memcpy(p, &x, 4);
        } else {
          int32_t x = static_cast<int32_t>(static_cast<int64_t>(v));
          std::memcpy(p, &x, 4);
        }
    }
  }

  std::vector<FieldInfo> fields_;
};
