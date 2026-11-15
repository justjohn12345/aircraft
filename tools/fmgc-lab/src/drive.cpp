// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Corpus driver: replays a recording through the generated FMGC with any combination of the scenario injectors and
// the input fuzzer, and runs the fuzz oracles (properties + differential checks, src/fuzz_oracle.h) on every step.
// Used for the coverage measurement of the generated code (scripts/coverage.sh, linked against a profile-instrumented
// model) and to check the properties over the corpus (scripts/properties-corpus.sh).
//
// usage: fmgc-drive [--fuzz S] [--approach S] [--athr S] [--lateral S] [--engagement S] [--targets S] [--failures S]
//                   [--tcas S] [--situation S] [--no-oracles] [--quiet] [--trace T0 T1] file.fdr
// --trace prints the modes, laws, A/THR and key inputs between simulation times T0 and T1 whenever they change.

#include "fuzz_oracle.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>

#include "input_fuzzer.h"
#include "replay_core.h"
#include "scenario_approach.h"
#include "scenario_athr.h"
#include "scenario_conditioning.h"
#include "scenario_engagement.h"
#include "scenario_lateral.h"
#include "scenario_targets.h"
#include "scenario_tcas.h"

using namespace fmgc::fuzz;

int main(int argc, char** argv) {
  std::map<std::string, long> seeds;
  bool oraclesOn = true, quiet = false;
  double traceFrom = 1e18, traceTo = -1;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--no-oracles") {
      oraclesOn = false;
    } else if (a == "--quiet") {
      quiet = true;
    } else if (a == "--trace" && i + 2 < argc) {
      traceFrom = std::atof(argv[++i]);
      traceTo = std::atof(argv[++i]);
    } else if (a.rfind("--", 0) == 0 && i + 1 < argc) {
      seeds[a.substr(2)] = std::atol(argv[++i]);
    } else {
      path = a;
    }
  }
  static const char* known[] = {"fuzz", "approach", "athr", "lateral", "engagement", "targets", "failures", "tcas",
                                "situation"};
  for (auto& [k, v] : seeds) {
    bool ok = false;
    for (const char* n : known) {
      ok = ok || k == n;
    }
    if (!ok) {
      path.clear();
    }
  }
  if (path.empty()) {
    std::fprintf(stderr,
                 "usage: fmgc-drive [--fuzz S] [--approach S] [--athr S] [--lateral S] [--engagement S] [--targets S] "
                 "[--failures S] [--tcas S] [--situation S] [--no-oracles] [--quiet] [--trace T0 T1] file.fdr\n");
    return 2;
  }
  auto has = [&](const char* k) { return seeds.count(k) != 0; };
  auto seed = [&](const char* k) { return static_cast<uint64_t>(seeds[k]); };

  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    return 2;
  }
  std::unique_ptr<ApproachScenario> approach;
  std::unique_ptr<LateralScenario> lateral;
  std::unique_ptr<AthrScenario> athr;
  std::unique_ptr<TargetsScenario> targets;
  std::unique_ptr<EngagementScenario> engagement;
  std::unique_ptr<TcasScenario> tcas;
  std::unique_ptr<SituationScenario> situation;
  std::unique_ptr<ConditioningScenario> failures;
  std::unique_ptr<InputFuzzer> fuzzer;
  if (has("approach")) approach = std::make_unique<ApproachScenario>(seed("approach"));
  if (has("lateral")) lateral = std::make_unique<LateralScenario>(seed("lateral"));
  if (has("athr")) athr = std::make_unique<AthrScenario>(seed("athr"));
  if (has("targets")) targets = std::make_unique<TargetsScenario>(seed("targets"));
  if (has("engagement")) engagement = std::make_unique<EngagementScenario>(seed("engagement"));
  if (has("tcas")) tcas = std::make_unique<TcasScenario>(seed("tcas"));
  if (has("situation")) situation = std::make_unique<SituationScenario>(seed("situation"));
  if (has("failures")) failures = std::make_unique<ConditioningScenario>(seed("failures"));
  if (has("fuzz")) fuzzer = std::make_unique<InputFuzzer>(seed("fuzz"));

  OracleSet::Options opt;
  if (!oraclesOn) {
    opt = {false, false, false, false};
  }
  OracleSet oracles(opt);
  if (oracles.needsBankState()) {
    oracles.warmBegin();  // the whole-computer differential runs from frame 0 alongside the generated model
  }
  oracles.begin(nullptr, 0);
  auto pre = std::make_unique<DWork>();
  std::map<std::string, long> findings;
  std::map<std::string, std::string> firstFinding;
  long steps = 0;
  FindingSink sink = [&](const Finding& f) {
    if (findings[f.check]++ == 0) {
      char buf[512];
      std::snprintf(buf, sizeof buf, "%s step %ld t %.3f %s", f.oracle.c_str(), f.step, f.t, f.detail.c_str());
      firstFinding[f.check] = buf;
    }
  };

  replayer.beforeStep = [&]() {
    fmgc_inputs& in = replayer.mutableInputs();
    double dt = in.time.dt;
    const fmgc_outputs& last = computer->getExternalOutputs().out;
    if (approach) approach->apply(in, dt);
    if (lateral) lateral->apply(in, dt);
    if (athr) athr->apply(in, dt);
    if (targets) targets->apply(in, dt);
    if (tcas) tcas->apply(in, dt, last);
    if (situation) situation->apply(in, dt, last);
    if (engagement) engagement->apply(in, dt, last);
    if (failures) failures->apply(in, dt);
    if (fuzzer) fuzzer->apply(in, dt);
    if (oracles.needsPreStepState()) {
      std::memcpy(static_cast<void*>(pre.get()), &computer->FmgcComputer_DWork, sizeof(DWork));
    }
  };
  replayer.afterStep = [&]() {
    const fmgc_inputs& in = replayer.inputs();
    FrameContext f{steps, in, computer->getExternalOutputs().out, in.sim_data.computer_running != 0, pre.get(), *computer};
    oracles.onFrame(f, sink);
    double t = in.time.simulation_time;
    if (t >= traceFrom && t <= traceTo && oracles.properties()) {
      const fmgc_outputs& out = computer->getExternalOutputs().out;
      const auto& ap = out.ap_fd_logic;
      const auto& pr = oracles.properties()->properties();
      const auto& am = ap.armed_modes;
      char buf[768];
      std::snprintf(buf, sizeof buf,
                    "%s | %s | armed %s%s%s%s%s%s%s%s | law %d/%d | athr fma %d%s T%d S%d R%d F%d | AP %d FD %d ATHR %d sync %d gnd %d",
                    in.sim_data.computer_running ? fmgc::props::Properties::vertNames(pr.vertMask()).c_str() : "(off)",
                    fmgc::props::Properties::latNames(pr.latMask()).c_str(), am.alt_acq_armed ? "ALT " : "",
                    am.nav_armed ? "NAV " : "", am.loc_armed ? "LOC " : "", am.glide_armed ? "GS " : "",
                    am.land_armed ? "LAND " : "", am.clb_armed ? "CLB " : "", am.des_armed ? "DES " : "",
                    am.final_des_armed ? "FINAL " : "", static_cast<int>(ap.active_longitudinal_law),
                    static_cast<int>(ap.active_lateral_law), static_cast<int>(out.athr.fma_mode),
                    out.athr.athr_active ? "*" : "", out.athr.thrust_mode_active, out.athr.speed_mach_mode_active,
                    out.athr.retard_mode_active, out.athr.alpha_floor_mode_active, out.logic.ap_own_engaged,
                    out.logic.fd_own_engaged, out.logic.athr_own_engaged, ap.fmgc_opp_mode_sync, out.logic.on_ground);
      static std::string prev;
      if (prev != buf) {
        std::printf("  %7ld t=%9.3f H %7.0f RA %6.0f FCU %6.0f tgt %6.0f V_c %6.1f | %s\n", steps, t,
                    out.logic.altitude_indicated_ft.Data, out.logic.ra_computation_data.radio_height_ft.Data,
                    in.bus_inputs.fcu_bus.selected_alt_ft.Data, ap.alt_sel_or_cstr, ap.spd_target_kts, buf);
        prev = buf;
      }
    }
    steps++;
  };
  Replayer::Frame fr;
  while (replayer.next(fr)) {
  }

  if (!quiet) {
    std::printf("file   %s\nsteps  %ld\nconfig", path.c_str(), steps);
    for (auto& [k, v] : seeds) {
      std::printf(" --%s %ld", k.c_str(), v);
    }
    std::printf("\n\n");
    for (auto& [k, n] : findings) {
      std::printf("finding %-40s %8ld  first: %s\n", k.c_str(), n, firstFinding[k].c_str());
    }
    if (oracles.properties()) {
      std::printf("\n");
      oracles.properties()->properties().print(stdout);
    }
  }
  return 0;
}
