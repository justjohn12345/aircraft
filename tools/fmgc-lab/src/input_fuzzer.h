// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Perturbs the FMGC inputs of a replay: random "episodes" during which one input field (discrete input, FMS input or
// any ARINC bus field) is stuck at a mutated value for 0.5–30 s. Used by differential checks, where the generated
// model is the oracle, so any input is a valid test input; starting from recorded flights keeps the rest realistic.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include "FmgcComputer_types.h"
#include "fields.gen.h"

class InputFuzzer {
 public:
  explicit InputFuzzer(uint64_t seed, double episodesPerSecond = 0.2) : rng_(seed), rate_(episodesPerSecond) {}

  /** Applies active episodes to `in` (call before every step) and starts/ends episodes. */
  void apply(fmgc_inputs& in, double dt) {
    if (fields_.empty()) {
      Collector c{fields_};
      visit(c, "", in, in);  // the field pointers refer to `in`: always pass the same struct
    }
    // end finished episodes
    for (size_t i = 0; i < active_.size();) {
      active_[i].remaining -= dt;
      if (active_[i].remaining <= 0) {
        active_[i] = active_.back();
        active_.pop_back();
      } else {
        i++;
      }
    }
    // start new ones
    if (active_.size() < 3 && uniform(0, 1) < rate_ * dt) {
      size_t idx = static_cast<size_t>(rng_() % fields_.size());
      Episode e;
      e.field = idx;
      e.remaining = uniform(0, 1) < 0.5 ? uniform(0.5, 5) : uniform(5, 30);
      e.value = fields_[idx].mutate(in, rng_);
      active_.push_back(e);
      episodes_++;
    }
    for (const Episode& e : active_) {
      fields_[e.field].set(in, e.value);
    }
  }

  long episodes() const { return episodes_; }

 private:
  struct Field {
    std::string name;
    std::function<double(fmgc_inputs&, std::mt19937_64&)> mutate;  // returns the new value (as double bits holder)
    std::function<void(fmgc_inputs&, double)> set;
  };

  struct Episode {
    size_t field;
    double remaining;
    double value;
  };

  struct Collector {
    std::vector<Field>& out;

    template <typename T>
    void field(const char* prefix, const char* name, T& a, T&, FieldKind kind) {
      std::string full = std::string(prefix) + name;
      // keep time and sim state (computer_running etc.) as recorded
      if (full.rfind("time.", 0) == 0 || full.rfind("sim_data.", 0) == 0) {
        return;
      }
      Field f;
      f.name = full;
      T* ptr = &a;
      bool isWord = full.size() > 5 && full.compare(full.size() - 5, 5, ".Data") == 0;
      bool isSsm = full.size() > 4 && full.compare(full.size() - 4, 4, ".SSM") == 0;
      f.set = [ptr](fmgc_inputs&, double v) { *ptr = fromDouble<T>(v); };
      f.mutate = [ptr, kind, isWord, isSsm](fmgc_inputs&, std::mt19937_64& rng) -> double {
        auto u = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
        double cur = toDouble(*ptr);
        if (kind == FieldKind::Bool) {
          return cur != 0 ? 0.0 : 1.0;
        }
        if (isSsm) {
          return static_cast<double>(rng() % 4);
        }
        if (kind == FieldKind::Enum || kind == FieldKind::Int) {
          return u(0, 1) < 0.5 ? cur + (rng() % 2 ? 1 : -1) : static_cast<double>(rng() % 8);
        }
        // floats
        if (isWord && cur >= 1024 && cur == std::floor(cur) && cur < 4294967296.0) {
          // probably a discrete word: flip a bit in 10..29
          uint32_t w = static_cast<uint32_t>(cur) ^ (1u << (10 + rng() % 20));
          return static_cast<double>(w);
        }
        switch (rng() % 5) {
          case 0:
            return 0.0;
          case 1:
            return -cur;
          case 2:
            return cur + u(-1, 1) * std::fmax(1.0, std::fabs(cur) * 0.05);
          case 3:
            return cur + u(-1, 1) * std::fmax(10.0, std::fabs(cur) * 0.5);
          default:
            return cur * u(0.5, 1.5);
        }
      };
      out.push_back(std::move(f));
    }

    template <typename T>
    static double toDouble(const T& v) {
      if constexpr (std::is_enum_v<T>) {
        return static_cast<double>(static_cast<int>(v));
      } else {
        return static_cast<double>(v);
      }
    }

    template <typename T>
    static T fromDouble(double v) {
      if constexpr (std::is_enum_v<T>) {
        return static_cast<T>(static_cast<int>(v));
      } else if constexpr (std::is_same_v<T, bool>) {
        return v != 0;
      } else {
        return static_cast<T>(v);
      }
    }
  };

  double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng_); }

  std::mt19937_64 rng_;
  double rate_;
  std::vector<Field> fields_;
  std::vector<Episode> active_;
  long episodes_ = 0;
};
