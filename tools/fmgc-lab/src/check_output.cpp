// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable output packing (readable/output_packing.cpp) against the generated FmgcComputer.
//
// The packing has no state, so there is no resync or free-running mode: on every running frame, the readable
// assemble() gets this frame's generated stage results (out.data, out.logic, out.ap_fd_logic, out.ap_fd_outer_loops,
// out.athr) and every field of its fmgc_outputs is compared with the generated FmgcComputer_Y.out, bit-exact (bools,
// enums and SSMs as integers, floats and doubles by their bit pattern). Two further groups check what the packing
// passes through and what the step stores for the next frame:
//   - copy.*: out.data is the step's inputs unchanged (field by field);
//   - delay.*: the end-of-step delays (generated 5107–5128) hold what the docs say they hold (previous ap_fd_logic,
//     outer-loop and A/THR outputs, the constraint for the DetectChange blocks, the latch copies).
// Input perturbations, combinable: --fuzz SEED (src/input_fuzzer.h), --approach SEED, --athr SEED, --lateral SEED,
// --engagement SEED, --targets SEED (src/scenario_*.h).
//
// Coverage report: for every discrete word, the steps and rises of each data-field bit (generated values), and for
// every word the SSM values seen.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// The generated model keeps its state and parameters private; the checker reads them (test-only access).
#define private public
#include "FmgcComputer.h"
#undef private

#include "fields.gen.h"
#include "input_fuzzer.h"
#include "output_packing.h"
#include "replay_core.h"
#include "scenario_approach.h"
#include "scenario_athr.h"
#include "scenario_engagement.h"
#include "scenario_lateral.h"
#include "scenario_targets.h"

namespace O = fmgc::readable::output;
using DWork = FmgcComputer::D_Work_FmgcComputer_T;

namespace {

struct Stats {
  std::string name;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  std::string firstValues;  // generated / readable at the first mismatch
};

template <typename T>
bool sameBits(const T& a, const T& b) {
  return std::memcmp(&a, &b, sizeof(T)) == 0;
}

template <typename T>
std::string show(const T& v) {
  if constexpr (std::is_floating_point_v<T>) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.9g", static_cast<double>(v));
    return buf;
  } else {
    return std::to_string(static_cast<long long>(v));
  }
}

/** Compares fields in visit order; the field list is registered on the first step. */
class Comparator {
 public:
  explicit Comparator(std::string group) : group_(std::move(group)) {}
  void begin(long step, double t) {
    index_ = 0;
    step_ = step;
    t_ = t;
  }
  template <typename T>
  void field(const char* prefix, const char* name, T& generated, T& readable, FieldKind /*kind*/) {
    if (index_ == stats_.size()) {
      stats_.push_back({group_ + std::string(prefix) + name});
    }
    Stats& st = stats_[index_++];
    if (sameBits(generated, readable)) {
      return;
    }
    if (st.firstStep < 0) {
      st.firstStep = step_;
      st.firstTime = t_;
      st.firstValues = show(generated) + " / " + show(readable);
    }
    st.mismatches++;
  }
  /** A named scalar check outside the visitor. */
  template <typename T>
  void check(const char* name, T generated, T readable) {
    field("", name, generated, readable, FieldKind::Float);
  }
  const std::vector<Stats>& stats() const { return stats_; }

 private:
  std::string group_;
  std::vector<Stats> stats_;
  size_t index_ = 0;
  long step_ = 0;
  double t_ = 0;
};

/** Per-word coverage: SSM values seen; for discrete words, steps and rises per data-field bit. */
struct WordCoverage {
  const char* name;
  bool discrete;
  long ssm[4]{};
  long bitSteps[33]{};
  long bitRises[33]{};
  long nonZero = 0;
  uint32_t prevBits = 0;
  void add(const base_arinc_429& w) {
    if (w.SSM < 4) {
      ssm[w.SSM]++;
    }
    nonZero += w.Data != 0.0F;
    if (!discrete) {
      return;
    }
    uint32_t bits = static_cast<uint32_t>(w.Data);
    for (int b = 1; b <= 32; b++) {
      bool on = (bits >> (b - 1)) & 1U;
      bitSteps[b] += on;
      bitRises[b] += on && !((prevBits >> (b - 1)) & 1U);
    }
    prevBits = bits;
  }
};

}  // namespace

int main(int argc, char** argv) {
  long fuzzSeed = -1, approachSeed = -1, athrSeed = -1, lateralSeed = -1, engagementSeed = -1, targetsSeed = -1;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto seed = [&](long& v) {
      if (i + 1 < argc) {
        v = std::atol(argv[++i]);
        return true;
      }
      return false;
    };
    if ((a == "--fuzz" && seed(fuzzSeed)) || (a == "--approach" && seed(approachSeed)) ||
        (a == "--athr" && seed(athrSeed)) || (a == "--lateral" && seed(lateralSeed)) ||
        (a == "--engagement" && seed(engagementSeed)) || (a == "--targets" && seed(targetsSeed))) {
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr,
                 "usage: fmgc-check-output [--fuzz SEED] [--approach SEED] [--athr SEED] [--lateral SEED] "
                 "[--engagement SEED] [--targets SEED] <file.fdr>\n");
    return 2;
  }

  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    return 2;
  }
  std::unique_ptr<InputFuzzer> fuzzer;
  if (fuzzSeed >= 0) {
    fuzzer = std::make_unique<InputFuzzer>(static_cast<uint64_t>(fuzzSeed));
  }
  std::unique_ptr<ApproachScenario> approach;
  if (approachSeed >= 0) {
    approach = std::make_unique<ApproachScenario>(static_cast<uint64_t>(approachSeed));
  }
  std::unique_ptr<LateralScenario> lateral;
  if (lateralSeed >= 0) {
    lateral = std::make_unique<LateralScenario>(static_cast<uint64_t>(lateralSeed));
  }
  std::unique_ptr<AthrScenario> athrScenario;
  if (athrSeed >= 0) {
    athrScenario = std::make_unique<AthrScenario>(static_cast<uint64_t>(athrSeed));
  }
  std::unique_ptr<EngagementScenario> engagement;
  if (engagementSeed >= 0) {
    engagement = std::make_unique<EngagementScenario>(static_cast<uint64_t>(engagementSeed));
  }
  std::unique_ptr<TargetsScenario> targets;
  if (targetsSeed >= 0) {
    targets = std::make_unique<TargetsScenario>(static_cast<uint64_t>(targetsSeed));
  }

  Comparator packed("");
  Comparator copies("copy.");
  Comparator delays("delay.");
  auto in = std::make_unique<fmgc_inputs>();
  long steps = 0, checkedSteps = 0;

  // coverage, A bus words then the B bus value words
  std::vector<WordCoverage> coverage;
  auto addWords = [&](base_fmgc_a_bus* a, base_fmgc_b_bus* b, bool collect) {
    size_t k = 0;
    auto w = [&](const char* name, bool discrete, const base_arinc_429& word) {
      if (k == coverage.size()) {
        coverage.push_back({name, discrete});
      }
      if (collect) {
        coverage[k].add(word);
      }
      k++;
    };
    w("a.pfd_sel_spd_kts", false, a->pfd_sel_spd_kts);
    w("a.runway_hdg_memorized_deg", false, a->runway_hdg_memorized_deg);
    w("a.preset_mach_from_mcdu", false, a->preset_mach_from_mcdu);
    w("a.preset_speed_from_mcdu_kts", false, a->preset_speed_from_mcdu_kts);
    w("a.roll_fd_command", false, a->roll_fd_command);
    w("a.pitch_fd_command", false, a->pitch_fd_command);
    w("a.yaw_fd_command", false, a->yaw_fd_command);
    w("a.discrete_word_5", true, a->discrete_word_5);
    w("a.discrete_word_4", true, a->discrete_word_4);
    w("a.fm_alt_constraint_ft", false, a->fm_alt_constraint_ft);
    w("a.altitude_ft", false, a->altitude_ft);
    w("a.mach", false, a->mach);
    w("a.cas_kts", false, a->cas_kts);
    w("a.flx_to_temp_deg_c", false, a->flx_to_temp_deg_c);
    w("a.ats_discrete_word", true, a->ats_discrete_word);
    w("a.ats_fma_discrete_word", true, a->ats_fma_discrete_word);
    w("a.discrete_word_3", true, a->discrete_word_3);
    w("a.discrete_word_1", true, a->discrete_word_1);
    w("a.discrete_word_2", true, a->discrete_word_2);
    w("a.discrete_word_6", true, a->discrete_word_6);
    w("a.synchro_spd_mach_value", false, a->synchro_spd_mach_value);
    w("a.low_target_speed_margin_kts", false, a->low_target_speed_margin_kts);
    w("a.high_target_speed_margin_kts", false, a->high_target_speed_margin_kts);
    w("a.delta_p_ail_voted_cmd_deg", false, a->delta_p_ail_voted_cmd_deg);
    w("a.delta_p_splr_voted_cmd_deg", false, a->delta_p_splr_voted_cmd_deg);
    w("a.delta_r_voted_cmd_deg", false, a->delta_r_voted_cmd_deg);
    w("a.delta_nosewheel_voted_cmd_deg", false, a->delta_nosewheel_voted_cmd_deg);
    w("a.delta_q_voted_cmd_deg", false, a->delta_q_voted_cmd_deg);
    w("a.track_deg", false, a->track_deg);
    w("a.heading_deg", false, a->heading_deg);
    w("a.fpa_deg", false, a->fpa_deg);
    w("a.n1_command_percent", false, a->n1_command_percent);
    w("a.vertical_speed_ft_min", false, a->vertical_speed_ft_min);
    w("a.discrete_word_7", true, a->discrete_word_7);
    w("b.fac_weight_lbs", false, b->fac_weight_lbs);
    w("b.fm_weight_lbs", false, b->fm_weight_lbs);
    w("b.fac_cg_percent", false, b->fac_cg_percent);
    w("b.fm_cg_percent", false, b->fm_cg_percent);
    w("b.fg_radio_height_ft", false, b->fg_radio_height_ft);
    w("b.approach_spd_target_kn", false, b->approach_spd_target_kn);
    w("b.delta_p_ail_cmd_deg", false, b->delta_p_ail_cmd_deg);
    w("b.delta_p_splr_cmd_deg", false, b->delta_p_splr_cmd_deg);
    w("b.delta_r_cmd_deg", false, b->delta_r_cmd_deg);
    w("b.delta_nose_wheel_cmd_deg", false, b->delta_nose_wheel_cmd_deg);
    w("b.delta_q_cmd_deg", false, b->delta_q_cmd_deg);
    w("b.n1_left_percent", false, b->n1_left_percent);
    w("b.n1_right_percent", false, b->n1_right_percent);
  };

  replayer.beforeStep = [&]() {
    double dt = replayer.inputs().time.dt;
    if (approach) {
      approach->apply(replayer.mutableInputs(), dt);
    }
    if (lateral) {
      lateral->apply(replayer.mutableInputs(), dt);
    }
    if (athrScenario) {
      athrScenario->apply(replayer.mutableInputs(), dt);
    }
    if (targets) {
      targets->apply(replayer.mutableInputs(), dt);
    }
    if (engagement) {
      engagement->apply(replayer.mutableInputs(), dt, computer->getExternalOutputs().out);
    }
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), dt);
    }
    *in = replayer.inputs();
  };
  replayer.afterStep = [&]() {
    steps++;
    if (!replayer.inputs().sim_data.computer_running) {
      return;  // the generated step leaves its outputs untouched
    }
    fmgc_outputs& out = computer->FmgcComputer_Y.out;
    const DWork& post = computer->FmgcComputer_DWork;
    double t = out.data.time.simulation_time;

    O::StageResults r{out.data, out.logic, out.ap_fd_logic, out.ap_fd_outer_loops, out.athr};
    fmgc_outputs readable = O::assemble(r);
    packed.begin(steps, t);
    visit(packed, "", out, readable);

    copies.begin(steps, t);
    visit(copies, "data.", out.data, *in);

    delays.begin(steps, t);
    {
      auto apFd = post.Delay_DSTATE;  // previous ap_fd_logic for the next frame
      visit(delays, "Delay_DSTATE.", out.ap_fd_logic, apFd);
      auto outer = post.Delay2_DSTATE;  // previous outer-loop outputs
      visit(delays, "Delay2_DSTATE.", out.ap_fd_outer_loops, outer);
      auto athr = post.Delay1_DSTATE;  // previous A/THR outputs
      visit(delays, "Delay1_DSTATE.", out.athr, athr);
      delays.check("DelayInput1_DSTATE(next_alt_cstr_ft)", in->fms_inputs.next_alt_cstr_ft, post.DelayInput1_DSTATE);
      delays.check("DelayInput1_DSTATE_n(next_alt_cstr_ft)", in->fms_inputs.next_alt_cstr_ft,
                   post.DelayInput1_DSTATE_n);
      delays.check("Delay_DSTATE_fe(first frame = 0)", 0.0, static_cast<double>(post.Delay_DSTATE_fe));
      delays.check("Memory_PreviousInput_g(AP engaged)", static_cast<int>(post.Delay_DSTATE_p),
                   static_cast<int>(post.Memory_PreviousInput_g));
      delays.check("Memory_PreviousInput_p(A/THR engaged)", static_cast<int>(post.Delay_DSTATE_k),
                   static_cast<int>(post.Memory_PreviousInput_p));
      delays.check("Memory_PreviousInput_bo(Mach latch)", static_cast<int>(post.Delay_DSTATE_c),
                   static_cast<int>(post.Memory_PreviousInput_bo));
      delays.check("icLoad(false)", 0, static_cast<int>(post.icLoad));
      delays.check("Delay_DSTATE_f(N1 integrator)", post.Delay_DSTATE_l, post.Delay_DSTATE_f);
      delays.check("athr.n1_c_percent(= N1 command bus)", static_cast<real32_T>(out.athr.n1_c_percent),
                   out.bus_outputs.fmgc_a_bus.n1_command_percent.Data);
    }

    addWords(&out.bus_outputs.fmgc_a_bus, &out.bus_outputs.fmgc_b_bus, true);
    checkedSteps++;
  };

  Replayer::Frame fr;
  while (replayer.next(fr)) {
  }

  std::string mode = "stateless";
  if (fuzzer) {
    mode += ", fuzzed (seed " + std::to_string(fuzzSeed) + ", " + std::to_string(fuzzer->episodes()) + " episodes)";
  }
  if (approach) {
    mode += ", approach scenarios (seed " + std::to_string(approachSeed) + ")";
  }
  if (lateral) {
    mode += ", lateral scenarios (seed " + std::to_string(lateralSeed) + ")";
  }
  if (athrScenario) {
    mode += ", autothrust scenarios (seed " + std::to_string(athrSeed) + ")";
  }
  if (engagement) {
    mode += ", engagement scenarios (seed " + std::to_string(engagementSeed) + ")";
  }
  if (targets) {
    mode += ", targets scenarios (seed " + std::to_string(targetsSeed) + ")";
  }
  std::printf("file     %s\nmode     %s\nsteps    %ld (%ld checked, computer running)\n", path.c_str(), mode.c_str(),
              steps, checkedSteps);

  long fields = 0, failing = 0;
  std::printf("\n%-62s %9s %10s %10s  %s\n", "field", "mismatch", "first", "t_first", "generated / readable");
  for (const Comparator* c : {&packed, &copies, &delays}) {
    for (const Stats& st : c->stats()) {
      fields++;
      if (st.mismatches == 0) {
        continue;
      }
      failing++;
      std::printf("%-62s %9ld %10ld %10.2f  %s\n", st.name.c_str(), st.mismatches, st.firstStep, st.firstTime,
                  st.firstValues.c_str());
    }
  }

  // coverage (generated values): "word NAME ssm FW NCD FT NO nonzero N" and "bit NAME BIT steps rises"
  std::printf("\n");
  for (const WordCoverage& w : coverage) {
    std::printf("word %-32s ssm %ld %ld %ld %ld nonzero %ld\n", w.name, w.ssm[0], w.ssm[1], w.ssm[2], w.ssm[3],
                w.nonZero);
  }
  for (const WordCoverage& w : coverage) {
    if (!w.discrete) {
      continue;
    }
    for (int b = 11; b <= 29; b++) {
      std::printf("bit %-32s %2d %10ld %7ld\n", w.name, b, w.bitSteps[b], w.bitRises[b]);
    }
    for (int b = 1; b <= 32; b++) {
      if ((b < 11 || b > 29) && w.bitSteps[b]) {
        std::printf("bit %-32s %2d %10ld %7ld OUTSIDE DATA FIELD\n", w.name, b, w.bitSteps[b], w.bitRises[b]);
      }
    }
  }
  std::printf("\n%ld fields compared, %ld with mismatches\n", fields, failing);
  return failing == 0 ? 0 : 1;
}
