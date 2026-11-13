// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable vertical mode logic (readable/vertical*.cpp) against the generated FmgcComputer,
// over a flight data recording. The generated model is the oracle (not the recording), so use the stock variant.
//
// Modes:
//   resync (default): every signal starts the frame from the generated value of the previous frame, and after each
//     readable update the signal is compared and then reset to the generated value, so a mismatch points at exactly
//     one signal's logic.
//   --free: the readable logic keeps its own state (its own previous outputs feed `last`), as it would in the sim.
// Signals that are not rewritten yet always take the generated value.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define private public
#include "FmgcComputer.h"
#undef private

#include "input_fuzzer.h"
#include "replay_core.h"
#include "vertical.h"

#include "external_approach.h"
#include "external_climb_descent.h"
#include "external_vs_tcas.h"

using fmgc::readable::External;
using fmgc::readable::Frame;
using fmgc::readable::kSignalCount;
using fmgc::readable::Signal;
using fmgc::readable::Signals;
using fmgc::readable::VerticalModeLogic;

namespace {

using DWork = FmgcComputer::D_Work_FmgcComputer_T;

/** Generated latch memory for a latch signal, or nullptr for combinational signals. */
const boolean_T* latchOf(const DWork& d, Signal s) {
  switch (s) {
    case Signal::SrsTakeoff: return &d.Memory_PreviousInput_k;
    case Signal::LandArmed: return &d.Memory_PreviousInput_l;
    case Signal::Land: return &d.Memory_PreviousInput_d;
    case Signal::FlareMemory: return &d.Memory_PreviousInput_bc;
    case Signal::FinalArmed: return &d.Memory_PreviousInput_dv;
    case Signal::Final: return &d.Memory_PreviousInput_f;
    case Signal::SrsGoAround: return &d.Memory_PreviousInput_i;
    case Signal::AltArmPossible: return &d.Memory_PreviousInput_n;
    case Signal::AltCapture: return &d.Memory_PreviousInput_ne;
    case Signal::AltHold: return &d.Memory_PreviousInput_cb;
    case Signal::AltCruiseMemory: return &d.Memory_PreviousInput_no;
    case Signal::VsFpa: return &d.Memory_PreviousInput_fg;
    case Signal::ClbArmed: return &d.Memory_PreviousInput_ma;
    case Signal::Clb: return &d.Memory_PreviousInput_ec;
    case Signal::DesArmed: return &d.Memory_PreviousInput_nt;
    case Signal::Des: return &d.Memory_PreviousInput_b3;
    case Signal::OpClb: return &d.Memory_PreviousInput_ae;
    case Signal::OpDes: return &d.Memory_PreviousInput_ev;
    case Signal::ExpClb: return &d.Memory_PreviousInput_mx;
    case Signal::ExpDes: return &d.Memory_PreviousInput_o;
    case Signal::GlideArmed: return &d.Memory_PreviousInput_fm;
    case Signal::GsCapture: return &d.Memory_PreviousInput_nu;
    case Signal::GsTrack: return &d.Memory_PreviousInput_as;
    case Signal::TcasArmed: return &d.Memory_PreviousInput_n0;
    case Signal::Tcas: return &d.Memory_PreviousInput_i5;
    case Signal::TcasAltAcq: return &d.Memory_PreviousInput_h;
    case Signal::TcasAltHold: return &d.Memory_PreviousInput_cp;
    case Signal::AltSoft: return &d.Memory_PreviousInput_ak;
    default: return nullptr;
  }
}

/** The ap_fd_logic output field of a signal, or nullptr for internal latches. */
boolean_T* outputOf(base_fmgc_ap_fd_logic_outputs& o, Signal s) {
  auto& m = o.longitudinal_modes;
  auto& a = o.armed_modes;
  switch (s) {
    case Signal::SrsTakeoff: return &m.pitch_takeoff_active;
    case Signal::LandArmed: return &a.land_armed;
    case Signal::Land: return &o.lateral_modes.land_active;
    case Signal::FinalArmed: return &a.final_des_armed;
    case Signal::Final: return &m.final_des_active;
    case Signal::SrsGoAround: return &m.pitch_goaround_active;
    case Signal::AltArmPossible: return &a.alt_acq_arm_possible;
    case Signal::AltArmed: return &a.alt_acq_armed;
    case Signal::AltCapture: return &m.alt_acq_active;
    case Signal::AltHold: return &m.alt_hold_active;
    case Signal::AltCruise: return &m.fma_dash_display;
    case Signal::ReversionVs: return &o.longitudinal_mode_reversion_vs;
    case Signal::ClbArmed: return &a.clb_armed;
    case Signal::Clb: return &m.clb_active;
    case Signal::DesArmed: return &a.des_armed;
    case Signal::Des: return &m.des_active;
    case Signal::OpClb: return &m.op_clb_active;
    case Signal::ReversionOpClb: return &o.longitudinal_mode_reversion_op_clb;
    case Signal::OpDes: return &m.op_des_active;
    case Signal::ExpClb: return &m.exp_clb_active;
    case Signal::ExpDes: return &m.exp_des_active;
    case Signal::GlideArmed: return &a.glide_armed;
    case Signal::GsCapture: return &m.gs_capt_active;
    case Signal::GsTrack: return &m.gs_trk_active;
    case Signal::TcasArmed: return &a.tcas_armed;
    case Signal::Tcas: return &m.tcas_active;
    case Signal::AltSoft: return &o.alt_soft_mode_active;
    case Signal::VsActive: return &m.vs_active;
    case Signal::FpaActive: return &m.fpa_active;
    case Signal::Flare: return &m.flare_active;
    default: return nullptr;
  }
}

/** Generated value of a signal after this step. */
bool generatedValue(const DWork& post, const fmgc_outputs& out, Signal s) {
  if (const boolean_T* l = latchOf(post, s)) {
    return *l;
  }
  auto o = out.ap_fd_logic;
  boolean_T* field = outputOf(o, s);
  return field ? *field : false;
}

/** Fills the not-yet-rewritten parts of the FMGC from the generated model's outputs of this step. */
void fillExternal(External& e, const fmgc_outputs& out, const DWork& pre, const DWork& post) {
  e.logic = &out.logic;
  e.oppModeSync = out.ap_fd_logic.fmgc_opp_mode_sync;
  e.anyApFdEngaged = out.ap_fd_logic.any_ap_fd_engaged;
  e.modeReset = out.ap_fd_logic.longitudinal_mode_reset;
  e.altCstrApplicable = out.ap_fd_logic.alt_cstr_applicable;
  e.altSelOrCstr = out.ap_fd_logic.alt_sel_or_cstr;
  e.tcasVsTarget = out.ap_fd_logic.tcas_vs_target;
  e.tcasRaCorrective = out.ap_fd_logic.tcas_ra_corrective;
  e.tcasModeAvailable = out.logic.tcas_mode_available;
  fillVsTcasExternal(e.vsTcas, out, pre, post);
  fillClimbDescentExternal(e.climbDescent, out, pre, post);
  fillApproachExternal(e.approach, out, pre, post);
}

struct Stats {
  bool implemented = false;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  bool firstGenerated = false;
  long lastStep = -1;
};

}  // namespace

int main(int argc, char** argv) {
  bool freeRunning = false;
  long fuzzSeed = -1;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--free") {
      freeRunning = true;
    } else if (a == "--fuzz" && i + 1 < argc) {
      fuzzSeed = std::atol(argv[++i]);
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      std::fprintf(stderr, "usage: fmgc-check-vertical [--free] [--fuzz SEED] <file.fdr>\n");
      return 2;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: fmgc-check-vertical [--free] [--fuzz SEED] <file.fdr>\n");
    return 2;
  }

  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    return 2;
  }

  auto pre = std::make_unique<DWork>();
  VerticalModeLogic logic;
  Signals s;
  Signals lastReadable;  // free mode: readable outputs of the previous step
  bool haveLastReadable = false;
  std::vector<Stats> stats(kSignalCount);
  long steps = 0, checkedSteps = 0;
  double simTime = 0;

  std::unique_ptr<InputFuzzer> fuzzer;
  if (fuzzSeed >= 0) {
    fuzzer = std::make_unique<InputFuzzer>(static_cast<uint64_t>(fuzzSeed));
  }
  replayer.beforeStep = [&]() {
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    *pre = computer->FmgcComputer_DWork;
  };
  replayer.afterStep = [&]() {
    steps++;
    const fmgc_inputs& in = replayer.inputs();
    simTime = in.time.simulation_time;
    if (!in.sim_data.computer_running) {
      haveLastReadable = false;
      return;
    }
    const DWork& post = computer->FmgcComputer_DWork;
    const fmgc_outputs& out = computer->getExternalOutputs().out;
    bool restarted = !pre->Runtime_MODE;

    // previous frame's outputs as the logic sees them
    base_fmgc_ap_fd_logic_outputs last = restarted ? FmgcComputer::FmgcComputer_P.Delay_InitialCondition
                                                   : pre->Delay_DSTATE;
    if (restarted) {
      logic.reset(s);
    } else if (!freeRunning || !haveLastReadable) {
      for (size_t i = 0; i < kSignalCount; i++) {
        if (const boolean_T* l = latchOf(*pre, static_cast<Signal>(i))) {
          s.v[i] = *l;
        }
      }
    }
    if (freeRunning && haveLastReadable && !restarted) {
      for (size_t i = 0; i < kSignalCount; i++) {
        if (stats[i].implemented) {
          if (boolean_T* field = outputOf(last, static_cast<Signal>(i))) {
            *field = lastReadable.v[i];
          }
        }
      }
    }

    External ext;
    fillExternal(ext, out, *pre, post);
    Frame f{in.time.dt, in, last, ext};
    for (size_t i = 0; i < kSignalCount; i++) {
      Signal id = static_cast<Signal>(i);
      bool generated = generatedValue(post, out, id);
      if (!logic.update(id, f, s)) {
        s.v[i] = generated;
        continue;
      }
      Stats& st = stats[i];
      st.implemented = true;
      if (s.v[i] != generated) {
        if (st.firstStep < 0) {
          st.firstStep = steps;
          st.firstTime = simTime;
          st.firstGenerated = generated;
        }
        st.mismatches++;
        st.lastStep = steps;
      }
      if (!freeRunning) {
        s.v[i] = generated;
      }
    }
    lastReadable = s;
    haveLastReadable = true;
    checkedSteps++;
  };

  Replayer::Frame fr;
  while (replayer.next(fr)) {
  }

  long implemented = 0, failing = 0;
  std::printf("file     %s\nmode     %s%s\nsteps    %ld (%ld checked, computer running)\n", path.c_str(),
              freeRunning ? "free-running" : "resync",
              fuzzer ? (", fuzzed inputs (seed " + std::to_string(fuzzSeed) + ", " + std::to_string(fuzzer->episodes()) +
                        " episodes)").c_str()
                     : "",
              steps, checkedSteps);
  std::printf("\n%-16s %6s %9s %10s %10s %10s %6s\n", "signal", "impl", "mismatch", "first", "t_first", "last",
              "gen@1st");
  for (size_t i = 0; i < kSignalCount; i++) {
    const Stats& st = stats[i];
    if (!st.implemented) {
      continue;
    }
    implemented++;
    if (st.mismatches) {
      failing++;
    }
    std::printf("%-16s %6s %9ld %10ld %10.2f %10ld %6d\n", fmgc::readable::signalName(static_cast<Signal>(i)), "yes",
                st.mismatches, st.firstStep, st.firstTime, st.lastStep, st.firstGenerated);
  }
  std::printf("\n%ld signals rewritten, %ld with mismatches\n", implemented, failing);
  return failing == 0 ? 0 : 1;
}
