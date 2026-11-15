// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Oracles the fuzzer (src/fuzz_fmgc.cpp), the corpus driver (src/drive.cpp) and the bank builder (src/fuzz_bank.cpp)
// call around every step of the generated FMGC. An oracle sees the step's inputs and the generated outputs and reports
// findings to a sink. Implemented:
//   - PropertyOracle: the behavioural properties of src/properties.h on the generated outputs (findings about stock
//     behaviour);
//   - OutputPackingOracle: differential, stateless: the readable output packing (readable/output_packing.cpp) applied
//     to this frame's generated stage results must reproduce every field of fmgc_outputs bit-exact;
//   - AthrResyncOracle: differential, resync: the readable autothrust (readable/athr*.cpp), its state taken from the
//     generated model before the step (src/athr_resync.h), must reproduce out.athr bit-exact;
//   - WholeComputerOracle: STUB for the whole-computer differential (readable::FmgcComputer from the integration
//     branch). See the comment on the class for the exact change.
//
// Include src/model_access.h (or this header) before anything else that includes FmgcComputer.h.

#pragma once

#include "model_access.h"

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "athr.h"
#include "athr_resync.h"
#include "field_table.h"
#include "output_packing.h"
#include "properties.h"

#ifndef FMGC_WHOLE_COMPUTER
#define FMGC_WHOLE_COMPUTER 0
#endif
#if FMGC_WHOLE_COMPUTER
#include "fmgc.h"  // readable/fmgc.h (integration branch)
#endif

namespace fmgc::fuzz {

using DWork = FmgcComputer::D_Work_FmgcComputer_T;

struct Finding {
  std::string oracle;  // "property", "output_packing", "athr_resync", "whole_computer"
  std::string check;   // property id or first differing field
  bool differential;   // a readable-vs-generated mismatch (a rewrite bug) rather than a stock-behaviour finding
  long step;
  double t;
  std::string detail;
  int propId = -1;  // fmgc::props::Id for property findings
};
using FindingSink = std::function<void(const Finding&)>;

/** Everything an oracle can look at for one step. */
struct FrameContext {
  long step;                 // step index in this run (0 = first step after the snapshot)
  const fmgc_inputs& in;     // the inputs of this step (after all fuzz events)
  const fmgc_outputs& out;   // generated outputs after the step (unchanged when the computer is not running)
  bool running;              // in.sim_data.computer_running
  const DWork* pre;          // generated state before the step (only when some oracle needsPreStepState())
  const FmgcComputer& computer;  // generated model after the step
};

class FuzzOracle {
 public:
  virtual ~FuzzOracle() = default;
  virtual const char* name() const = 0;
  /** True when onFrame needs FrameContext::pre (costs one copy of the generated state per step). */
  virtual bool needsPreStepState() const { return false; }

  // ---- warm-up (bank builder): called for every recorded step from the start of a recording, so stateful oracles
  // can reach the state the generated model has at each snapshot.
  virtual void warmBegin() {}
  virtual void warmStep(const fmgc_inputs& /*in*/, const fmgc_outputs& /*generated*/) {}
  /** Appends this oracle's state at the current warm-up point (stored with the snapshot in the bank). */
  virtual void saveState(std::vector<uint8_t>& /*blob*/) const {}

  // ---- runs (fuzzer / driver)
  /** A new run starts from a snapshot; `state` is what saveState() stored for it (empty if nothing). */
  virtual void begin(const uint8_t* /*state*/, size_t /*size*/) {}
  virtual void onFrame(const FrameContext& f, const FindingSink& sink) = 0;
};

/** The behavioural properties (src/properties.h) as an oracle. */
class PropertyOracle : public FuzzOracle {
 public:
  const char* name() const override { return "property"; }
  void begin(const uint8_t*, size_t) override { props_.reset(); }
  void onFrame(const FrameContext& f, const FindingSink& sink) override {
    sink_ = &sink;
    props_.onViolation = [this](int id, long step, double t, const std::string& detail) {
      (*sink_)({name(), props::list()[id].id, false, step, t, detail, id});
    };
    props_.onFrame(f.step, f.in, f.out, f.running);
  }
  props::Properties& properties() { return props_; }

 private:
  props::Properties props_;
  const FindingSink* sink_ = nullptr;
};

/** Stateless differential: readable output packing on this frame's generated stage results. */
class OutputPackingOracle : public FuzzOracle {
 public:
  const char* name() const override { return "output_packing"; }
  void onFrame(const FrameContext& f, const FindingSink& sink) override {
    if (!f.running) {
      return;  // the generated step leaves its outputs untouched
    }
    namespace O = fmgc::readable::output;
    O::StageResults r{f.out.data, f.out.logic, f.out.ap_fd_logic, f.out.ap_fd_outer_loops, f.out.athr};
    fmgc_outputs readable = O::assemble(r);
    const auto& table = FieldTable<fmgc_outputs>::get();
    int i = table.firstDifference(f.out, readable);
    if (i >= 0) {
      char buf[128];
      std::snprintf(buf, sizeof buf, "generated %.9g readable %.9g", table.value(f.out, i), table.value(readable, i));
      sink({name(), table.fields()[i].name, true, f.step, f.in.time.simulation_time, buf});
    }
  }
};

/** Differential in resync mode: the readable autothrust starts every step from the generated state. */
class AthrResyncOracle : public FuzzOracle {
 public:
  const char* name() const override { return "athr_resync"; }
  bool needsPreStepState() const override { return true; }
  void onFrame(const FrameContext& f, const FindingSink& sink) override {
    if (!f.running || !f.pre) {
      return;
    }
    namespace A = fmgc::readable::athr;
    const DWork& pre = *f.pre;
    const base_fmgc_athr_outputs& gen = f.out.athr;
    bool restarted = !pre.Runtime_MODE;
    base_fmgc_athr_outputs last = restarted ? FmgcComputer::FmgcComputer_P.Delay1_InitialCondition : pre.Delay1_DSTATE;
    if (restarted) {
      athr_.reset();
    } else {
      resyncAthrModes(athr_.modesState(), pre);
      resyncAthrN1(athr_.n1State(), pre);
    }
    A::Inputs in{f.out.data.time.dt, f.out.data, f.out.logic, f.out.ap_fd_logic, last};
    A::Result r;
    athr_.updateModes(in, r);
    const auto& table = FieldTable<base_fmgc_athr_outputs>::get();
    static const int n1Field = table.find("n1_c_percent");
    for (size_t i = 0; i < table.fields().size(); i++) {
      if (static_cast<int>(i) == n1Field || !r.rewritten[i]) {
        continue;
      }
      if (std::memcmp(reinterpret_cast<const char*>(&gen) + table.fields()[i].offset,
                      reinterpret_cast<const char*>(&r.out) + table.fields()[i].offset, table.fields()[i].size) != 0) {
        report(sink, f, table, static_cast<int>(i), gen, r.out);
        return;
      }
    }
    // the N1 stage sees the generated mode outputs (resync)
    double n1 = r.out.n1_c_percent;
    r.out = gen;
    r.out.n1_c_percent = n1;
    athr_.updateN1(in, r);
    if (r.rewritten[A::N1Command] &&
        std::memcmp(&gen.n1_c_percent, &r.out.n1_c_percent, sizeof(gen.n1_c_percent)) != 0) {
      report(sink, f, table, n1Field, gen, r.out);
    }
  }

 private:
  void report(const FindingSink& sink, const FrameContext& f, const FieldTable<base_fmgc_athr_outputs>& table, int i,
              const base_fmgc_athr_outputs& gen, const base_fmgc_athr_outputs& readable) {
    char buf[128];
    std::snprintf(buf, sizeof buf, "generated %.9g readable %.9g", table.value(gen, i), table.value(readable, i));
    sink({name(), "athr." + table.fields()[i].name, true, f.step, f.in.time.simulation_time, buf});
  }
  fmgc::readable::athr::Autothrust athr_;
};

/**
 * Whole-computer differential: STUB until the integration branch (readable/fmgc.{h,cpp}) merges.
 *
 * To wire it (the only changes needed):
 *   1. CMakeLists.txt: add readable/fmgc.cpp (and the stage .cpp files it needs) to FUZZ_READABLE_SOURCES and set
 *      FMGC_WHOLE_COMPUTER=1 in FUZZ_DEFINES (one place, the "fuzzing and coverage" block).
 *   2. Below, in the `#if FMGC_WHOLE_COMPUTER` branches: adapt the three calls marked ADAPT to the readable computer's
 *      API (assumed: `fmgc::readable::FmgcComputer` with `initialize()`, `fmgc_outputs step(const fmgc_inputs&)`, and a
 *      trivially copyable state; if it is not trivially copyable, give it save/load functions and call them in
 *      saveState()/begin()).
 *   3. Rebuild the bank (scripts/fuzz.sh --rebuild-bank): the bank builder runs the readable computer alongside the
 *      generated one from frame 0 of every recording and stores its state with each snapshot, so every fuzz run starts
 *      both computers from the same point. The bank header records the oracle state size; a bank built without it is
 *      rejected by the harness when this oracle is enabled.
 * Mismatches are differential findings: the harness aborts on them by default (FMGC_FUZZ_ABORT=diff), so libFuzzer
 * keeps and minimises the input.
 */
class WholeComputerOracle : public FuzzOracle {
 public:
  const char* name() const override { return "whole_computer"; }
  static constexpr bool enabled() { return FMGC_WHOLE_COMPUTER != 0; }

#if FMGC_WHOLE_COMPUTER
  void warmBegin() override { readable_.initialize(); }                                         // ADAPT
  void warmStep(const fmgc_inputs& in, const fmgc_outputs&) override { (void)readable_.step(in); }  // ADAPT
  void saveState(std::vector<uint8_t>& blob) const override {
    static_assert(std::is_trivially_copyable_v<decltype(readable_)>, "give the readable computer save/load functions");
    const auto* p = reinterpret_cast<const uint8_t*>(&readable_);
    blob.insert(blob.end(), p, p + sizeof(readable_));
  }
  void begin(const uint8_t* state, size_t size) override {
    if (state == nullptr) {
      return;  // the corpus driver: the readable computer was started with warmBegin() alongside the generated one
    }
    if (size == sizeof(readable_)) {
      std::memcpy(static_cast<void*>(&readable_), state, size);
    } else {
      std::fprintf(stderr, "whole_computer: the bank has no readable computer state (rebuild the bank)\n");
      std::abort();
    }
  }
  void onFrame(const FrameContext& f, const FindingSink& sink) override {
    fmgc_outputs r = readable_.step(f.in);  // ADAPT
    const auto& table = FieldTable<fmgc_outputs>::get();
    int i = table.firstDifference(f.out, r);
    if (i >= 0) {
      char buf[128];
      std::snprintf(buf, sizeof buf, "generated %.9g readable %.9g", table.value(f.out, i), table.value(r, i));
      sink({name(), table.fields()[i].name, true, f.step, f.in.time.simulation_time, buf});
    }
  }

 private:
  fmgc::readable::FmgcComputer readable_;  // ADAPT (type name)
#else
  void onFrame(const FrameContext&, const FindingSink&) override {}
#endif
};

/** The oracles in use, called in order. Owns them. */
class OracleSet {
 public:
  struct Options {
    bool properties = true;
    bool outputPacking = true;
    bool athrResync = true;
    bool wholeComputer = WholeComputerOracle::enabled();
  };

  explicit OracleSet(Options opt) {
    if (opt.properties) {
      auto p = std::make_unique<PropertyOracle>();
      properties_ = p.get();
      oracles_.push_back(std::move(p));
    }
    if (opt.outputPacking) {
      oracles_.push_back(std::make_unique<OutputPackingOracle>());
    }
    if (opt.athrResync) {
      oracles_.push_back(std::make_unique<AthrResyncOracle>());
    }
    if (opt.wholeComputer) {
      auto w = std::make_unique<WholeComputerOracle>();
      whole_ = w.get();
      oracles_.push_back(std::move(w));
    }
    for (auto& o : oracles_) {
      needsPre_ = needsPre_ || o->needsPreStepState();
    }
  }

  bool needsPreStepState() const { return needsPre_; }
  PropertyOracle* properties() { return properties_; }

  /** Only the whole-computer oracle keeps state across a snapshot; its blob is the bank's oracle state. */
  void warmBegin() {
    if (whole_) {
      whole_->warmBegin();
    }
  }
  void warmStep(const fmgc_inputs& in, const fmgc_outputs& out) {
    if (whole_) {
      whole_->warmStep(in, out);
    }
  }
  void saveState(std::vector<uint8_t>& blob) const {
    if (whole_) {
      whole_->saveState(blob);
    }
  }
  bool needsBankState() const { return whole_ != nullptr; }

  void begin(const uint8_t* state, size_t size) {
    for (auto& o : oracles_) {
      o->begin(state, size);
    }
  }
  void onFrame(const FrameContext& f, const FindingSink& sink) {
    for (auto& o : oracles_) {
      o->onFrame(f, sink);
    }
  }

 private:
  std::vector<std::unique_ptr<FuzzOracle>> oracles_;
  PropertyOracle* properties_ = nullptr;
  WholeComputerOracle* whole_ = nullptr;
  bool needsPre_ = false;
};

}  // namespace fmgc::fuzz
