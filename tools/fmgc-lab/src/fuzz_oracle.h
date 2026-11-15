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
//   - WholeComputerOracle: differential, free-running: the readable whole computer (readable/fmgc.{h,cpp}) on the
//     same inputs; every leaf of fmgc_outputs and the hidden memories (mode latches, other latches) bit-exact.
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
#include "fmgc.h"              // readable/fmgc.h: the readable whole computer
#include "vertical_latches.h"  // generated latch memory of each mode signal
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
 * Whole-computer differential: the readable computer (readable/fmgc.{h,cpp}, fmgc::readable::FmgcComputer) steps on
 * the same inputs as the generated model, free-running (it never sees a generated value), and every leaf of
 * fmgc_outputs (1017 leaves, by bit pattern, so -0/+0 and NaN payloads count) is compared every step, running or not.
 * The hidden memories that check_fmgc compares (every mode latch, the engagement / targets / A/THR / TCAS latches, the
 * memories that survive a restart) are compared too, so a divergence is caught before it reaches an output.
 *
 * State: the readable computer has no heap and no pointers, so its bytes are a snapshot. The bank builder warms it up
 * from frame 0 of every recording alongside the generated model (warmBegin/warmStep) and stores its bytes with each
 * snapshot (saveState); a fuzz run restores them (begin). The corpus driver runs it from frame 0 (begin with no state).
 * Mismatches are differential findings: the harness aborts on them by default (FMGC_FUZZ_ABORT=diff), so libFuzzer
 * keeps and minimises the input. Built with FMGC_WHOLE_COMPUTER=1 (CMakeLists.txt FUZZ_DEFINES).
 */
class WholeComputerOracle : public FuzzOracle {
 public:
  const char* name() const override { return "whole_computer"; }
  static constexpr bool enabled() { return FMGC_WHOLE_COMPUTER != 0; }

#if FMGC_WHOLE_COMPUTER
  WholeComputerOracle() { readable_.initialize(); }
  void warmBegin() override { readable_.initialize(); }
  void warmStep(const fmgc_inputs& in, const fmgc_outputs&) override { (void)readable_.step(in); }
  void saveState(std::vector<uint8_t>& blob) const override {
    const auto* p = reinterpret_cast<const uint8_t*>(&readable_);  // no heap, no pointers: the bytes are the state
    blob.insert(blob.end(), p, p + sizeof(readable_));
  }
  void begin(const uint8_t* state, size_t size) override {
    hiddenDiverged_ = false;
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
    const fmgc_outputs& r = readable_.step(f.in);
    steps_++;
    const auto& table = FieldTable<fmgc_outputs>::get();
    int i = table.firstDifference(f.out, r);
    if (i >= 0) {
      mismatchSteps_++;
      char buf[160];
      std::snprintf(buf, sizeof buf, "generated %.17g readable %.17g", table.value(f.out, i), table.value(r, i));
      sink({name(), table.fields()[i].name, true, f.step, f.in.time.simulation_time, buf});
    }
    if (!hiddenDiverged_) {
      const char* h = firstHiddenDivergence(f.computer);
      if (h) {
        hiddenDiverged_ = true;  // reported once per run: a diverged memory usually stays diverged
        sink({name(), std::string("hidden.") + h, true, f.step, f.in.time.simulation_time, "hidden memory diverged"});
      }
    }
  }
  long steps() const { return steps_; }
  long mismatchSteps() const { return mismatchSteps_; }

 private:
  /** check_fmgc's hidden-memory list (src/check_fmgc.cpp collectHidden): the first generated/readable difference. */
  const char* firstHiddenDivergence(const FmgcComputer& gen) const {
    namespace R = fmgc::readable;
    const DWork& d = gen.FmgcComputer_DWork;
    const auto& b = gen.FmgcComputer_B;
    const R::Signals& s = readable_.modeSignals();
    for (size_t i = 0; i < R::kSignalCount; i++) {
      if (const boolean_T* l = latchOf(d, static_cast<R::Signal>(i))) {
        if ((*l != 0) != s.v[i]) {
          return R::signalName(static_cast<R::Signal>(i));
        }
      }
    }
    const auto& es = readable_.engagementStage().state();
    const R::targets::Values& tv = readable_.targetValues();
    const auto& am = readable_.autothrustStage().modesState();
    const auto& ts = readable_.tcasStage().state();
    const auto& cs = readable_.conditioningStage().state();
    const R::OuterLoopInputState& os = readable_.outerLoopInputState();
    struct H {
      const char* name;
      double g, r;
    };
    auto D = [](auto v) { return static_cast<double>(v); };
    const H hs[] = {
        {"engagement.apEngageRequest", D(d.Memory_PreviousInput), D(es.apEngageRequest.q)},
        {"engagement.apEngaged", D(d.Delay_DSTATE_p), D(es.apEngaged.q)},
        {"engagement.athrInhibited", D(d.Memory_PreviousInput_g1), D(es.athrInhibited.q)},
        {"engagement.athrEngaged", D(d.Delay_DSTATE_k), D(es.athrEngaged.q)},
        {"engagement.land3FailOpHeld", D(d.pLand3FailOp), D(es.land3FailOpHeld)},
        {"engagement.land3FailPassiveHeld", D(d.pLand3FailPass), D(es.land3FailPassiveHeld)},
        {"targets.manualSpeed", D(d.Memory_PreviousInput_cu), D(tv.manualSpeed)},
        {"targets.autoSpeed", D(d.Memory_PreviousInput_hk), D(tv.autoSpeed)},
        {"targets.machMode", D(d.Delay_DSTATE_c), D(tv.machMode)},
        {"targets.lowerMarginActive", D(d.Memory_PreviousInput_dh), D(tv.lowerMarginActive)},
        {"targets.vappMemoKts", D(b.u_l), D(tv.vappMemoKts)},
        {"athr.tcasLatch", D(d.Memory_PreviousInput_jb), D(am.tcasLatch.q)},
        {"athr.alphaFloor", D(d.Memory_PreviousInput_hu), D(am.alphaFloor.q)},
        {"athr.thrustMode", D(d.Memory_PreviousInput_bh), D(am.thrustMode.q)},
        {"athr.speedMode", D(d.Memory_PreviousInput_cm), D(am.speedMode.q)},
        {"athr.retardMode", D(d.Memory_PreviousInput_ol), D(am.retardMode.q)},
        {"athr.lvrClbArmed", D(d.Memory_PreviousInput_kr), D(am.lvrClbArmed.q)},
        {"athr.lvrMctArmed", D(d.Memory_PreviousInput_km), D(am.lvrMctArmed.q)},
        {"athr.n1CommandPercent", D(d.Delay_DSTATE_l), D(readable_.autothrustStage().n1State().n1CommandPercent)},
        {"tcas.hdgVsDuringTcas", D(d.Memory_PreviousInput_e), D(ts.hdgVsDuringTcas.q)},
        {"tcas.tcasEndedAfterHdgVs", D(d.Delay_DSTATE_o), D(ts.tcasEndedAfterHdgVs)},
        {"conditioning.rwyHdgMemo", D(b.u_lyjj), D(cs.rwyHdgMemo)},
        {"conditioning.inFlight5s", D(d.sf_MATLABFunction_fw.output), D(cs.inFlight5s.output)},
        {"outerLoopInputs.locDistanceMemoNmi", D(b.u), D(os.locDistanceMemoNmi)},
        {"outerLoopInputs.trackMemoDeg", D(b.u_lyjjl), D(os.trackMemoDeg)},
        {"running(Runtime_MODE)", D(d.Runtime_MODE), D(readable_.running())},
    };
    for (const H& h : hs) {
      if (std::memcmp(&h.g, &h.r, sizeof(double)) != 0) {
        return h.name;
      }
    }
    return nullptr;
  }

  fmgc::readable::FmgcComputer readable_;
  bool hiddenDiverged_ = false;
  long steps_ = 0, mismatchSteps_ = 0;
#else
  void onFrame(const FrameContext&, const FindingSink&) override {}
  long steps() const { return 0; }
  long mismatchSteps() const { return 0; }
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
  const WholeComputerOracle* wholeComputer() const { return whole_; }

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
