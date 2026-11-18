// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable FAC (readable/fac/) against the generated FacComputer over a flight data
// recording. Two pairs (FAC 1 and 2) run in lockstep on the same inputs, free-running (the readable FACs never see a
// generated value); every step the whole fac_outputs (data, laws, logic, flight envelope, discretes, analog orders,
// bus) is compared leaf by leaf, bit-exact (floats by bit pattern; two NaNs match whatever their sign and payload: x86
// takes them from whichever operand the compiler put first in a commutative operation, and no FAC output depends on
// a NaN's sign), and so are the hidden memories (confirm nodes, filters, rate limiters, latches, the flight phase
// chart, the rudder trim integrator).
//
// Inputs (src/fac_drive.h): both FACs' inputs rebuilt from any recording (no format records them). Injectors
// (src/fac_scenarios.h), combinable: --script, --failures, --discretes, --power, --sim SEED. --warm FILE: run both over
// FILE first, then re-construct the generated FACs and initialize() both (checks that the readable initialize() is a
// complete reset). --time: step times only. --dump STEP: every generated output leaf (inputs included) of both FACs at
// STEP - 1 and STEP.
//
// Exit status 0 when every leaf and hidden memory matched on every step, 1 on any mismatch, 2 on usage/file errors.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "fac_drive.h"  // includes FacComputer.h with test-only access to its state
#include "fac_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"

#include "fac.h"
#include "fac_scenarios.h"

namespace R = fac::readable;
namespace S = fac_scenarios;
using Clock = std::chrono::steady_clock;

namespace {

struct Hidden {
  std::string name;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0, firstReadable = 0;
};

/** Pairs (generated, readable) of every hidden memory, as doubles (exact for floats, bools, enums). */
void collectHidden(const FacComputer& g, const R::FacComputer& r, const char* prefix, std::vector<Hidden>& hidden,
                   std::vector<std::pair<double, double>>& values, size_t& k) {
  const auto& d = g.FacComputer_DWork;
  auto add = [&](const std::string& name, double gv, double rv) {
    if (k == hidden.size()) hidden.push_back({std::string(prefix) + name});
    if (values.size() <= k) values.resize(k + 1);
    values[k++] = {gv, rv};
  };
  using Confirm = FacComputer::rtDW_MATLABFunction_FacComputer_g_T;
  auto confirm = [&](const char* name, const Confirm& gc, const R::blocks::ConfirmNode& rc) {
    add(std::string(name) + ".time", gc.timeSinceCondition, rc.timeSinceCondition);
    add(std::string(name) + ".output", gc.output, rc.output);
  };
  using Trig = FacComputer::rtDW_MATLABFunction_FacComputer_d_T;
  auto trig = [&](const char* name, const Trig& gt, const R::blocks::Monostable& rt) {
    add(std::string(name) + ".remaining", gt.remainingTriggerTime, rt.remainingTriggerTime);
    add(std::string(name) + ".previous", gt.previousInput, rt.previousInput);
  };
  auto lag = [&](const char* name, const auto& gl, const auto& rl) {
    add(std::string(name) + ".pY", gl.pY, rl.pY);
    add(std::string(name) + ".pU", gl.pU, rl.pU);
    add(std::string(name) + ".pY_not_empty", gl.pY_not_empty, rl.primed);
    add(std::string(name) + ".pU_not_empty", gl.pU_not_empty, rl.primed);
  };
  auto rate = [&](const char* name, const auto& gr, const auto& rr) {
    add(std::string(name) + ".pY", gr.pY, rr.pY);
    add(std::string(name) + ".pY_not_empty", gr.pY_not_empty, rr.primed);
  };

  add("running(Runtime_MODE)", d.Runtime_MODE, r.running());
  const R::InputConditioning& c = r.sensors();
  confirm("noseGearAgreement(ax)", d.sf_MATLABFunction_ax, c.noseGearAgreement());
  confirm("sfccAgreement(jf)", d.sf_MATLABFunction_jf, c.sfccAgreement());
  confirm("raOwnValid(cu)", d.sf_MATLABFunction_cu, c.raOwnValid());
  confirm("raOppValid(md)", d.sf_MATLABFunction_md, c.raOppValid());
  confirm("onGround(le)", d.sf_MATLABFunction_le, c.onGroundConfirm());

  const R::AlphaFloor& a = r.alphaFloor();
  lag("alphaFilter(LagFilter_c)", d.sf_LagFilter_c, a.alphaFilter());
  rate("alphaFloorThreshold(RateLimiter)", d.sf_RateLimiter, a.threshold());
  lag("speedRate(LagFilter)", d.sf_LagFilter, a.speedRateFilter());
  add("previousHalfCas(Delay_DSTATE)", d.Delay_DSTATE, a.previousHalfCas());
  add("chartActive(is_active_c15)", d.is_active_c15_FacComputer, a.chartActive());
  add("phase(is_c15)", d.is_c15_FacComputer, static_cast<double>(a.phase()));
  add("alphaFloor(sAlphaFloor)", d.sAlphaFloor, a.active());

  const R::CharacteristicSpeeds& v = r.speeds();
  rate("alpha0(RateLimiter_c)", d.sf_RateLimiter_c, v.alpha0());
  rate("alphaMax(RateLimiter_a)", d.sf_RateLimiter_a, v.alphaMax());
  rate("alphaProt(RateLimiter_n)", d.sf_RateLimiter_n, v.alphaProt());
  rate("alphaStallWarn(RateLimiter_j)", d.sf_RateLimiter_j, v.alphaStallWarn());
  rate("vStall(RateLimiter_g0)", d.sf_RateLimiter_g0, v.vStallRate());
  rate("vlsFactor(RateLimiter_g)", d.sf_RateLimiter_g, v.vlsFactorRate());
  add("takeoffConfig(takeoff_config)", d.takeoff_config, v.takeoffConfig());
  confirm("speedScaleVisible(p4)", d.sf_MATLABFunction_p4, v.speedScaleConfirm());
  lag("trendLag(LagFilter_fb)", d.sf_LagFilter_fb, v.trendLag());
  lag("trendWashout(WashoutFilter)", d.sf_WashoutFilter, v.trendWashout());

  const R::FlightPath& f = r.flightPath();
  lag("sideslip(LagFilter_d)", d.sf_LagFilter_d, f.sideslipLag());
  lag("n1RightMinusLeft(LagFilter_k)", d.sf_LagFilter_k, f.n1DiffRightLag());
  lag("betaAlpha(LagFilter_f)", d.sf_LagFilter_f, f.betaAlphaLag());
  lag("n1LeftMinusRight(LagFilter_d5)", d.sf_LagFilter_d5, f.n1DiffLeftLag());
  lag("gammaA(LagFilter_m)", d.sf_LagFilter_m, f.gammaALag());
  lag("nz(LagFilter_c4)", d.sf_LagFilter_c4, f.nzLag());
  lag("gammaT(LagFilter_mt)", d.sf_LagFilter_mt, f.gammaTLag());
  lag("inertialFpa(LagFilter_fi)", d.sf_LagFilter_fi, f.inertialFpaLag());
  lag("deceleration(LagFilter_e)", d.sf_LagFilter_e, f.decelerationLag());
  confirm("raNcd(o)", d.sf_MATLABFunction_o, f.raNcdConfirm());
  trig("goAroundInhibit(ns)", d.sf_MATLABFunction_ns, f.goAroundInhibit());
  confirm("lowEnergyConfirm(n)", d.sf_MATLABFunction_n, f.lowEnergyConfirm());
  trig("lowEnergyHold(a)", d.sf_MATLABFunction_a, f.lowEnergyHold());

  const R::RudderTrim& t = r.rudderTrim();
  add("trimResetLatch(Memory_PreviousInput)", d.Memory_PreviousInput, t.resetLatch());
  add("trimResetButton.previous(previousInput)", d.previousInput, t.resetButtonEdge().previousInput);
  add("trimResetButton.primed(previousInput_not_empty)", d.previousInput_not_empty, t.resetButtonEdge().primed);
  add("trimIntegrator(Delay_DSTATE_d)", d.Delay_DSTATE_d, t.integrator());
  add("trimReload(icLoad)", d.icLoad, t.reload());
  rate("trimOrder(RateLimiter_l)", d.sf_RateLimiter_l, t.orderRate());
  lag("yawDamperWashout(WashoutFilter_i)", d.sf_WashoutFilter_i, r.yawDamper().washout());
  rate("yawDamperOrder(RateLimiter_f)", d.sf_RateLimiter_f, r.yawDamper().orderRate());
  rate("travelLimit(RateLimiter_p)", d.sf_RateLimiter_p, r.travelLimiter().orderRate());
}

/** The SR flip-flop table against the readable latch. Returns the number of differing entries. */
int checkTable() {
  const auto& p = FacComputer::FacComputer_P;
  int bad = 0;
  for (int i = 0; i < 8; i++) {
    R::blocks::SrLatch l;
    l.q = i & 1;
    const bool s = i & 4, rs = i & 2;
    l.update(s, rs);
    bad += p.Logic_table[i] != l.q;
  }
  return bad;
}

/** Event counts of one generated FAC (from its outputs, step by step). */
struct Events {
  long restarts = 0, alphaFloor = 0, lowEnergy = 0, pitchPitch = 0, trimMoves = 0, trimResets = 0;
  long phaseChanges = 0, ydAlternate = 0, sfccFallback = 0, lgciuFallback = 0, adrFallback = 0, irFallback = 0;
  bool prevRunning = false, prevAlphaFloor = false, prevLowEnergy = false, prevPitch = false, prevLatch = false;
  double prevTrim = 0;
  int prevPhase = 0;

  void update(const FacComputer& g) {
    const fac_outputs& o = g.FacComputer_Y.out;
    const auto& d = g.FacComputer_DWork;
    restarts += d.Runtime_MODE && !prevRunning;
    prevRunning = d.Runtime_MODE;
    if (!d.Runtime_MODE) return;
    alphaFloor += o.flight_envelope.alpha_floor_condition && !prevAlphaFloor;
    lowEnergy += o.flight_envelope.low_energy_warning_active && !prevLowEnergy;
    pitchPitch += o.flight_envelope.pitch_pitch_warning_active && !prevPitch;
    trimMoves += o.laws.rudder_trim_command_deg != prevTrim;
    trimResets += d.Memory_PreviousInput && !prevLatch;
    phaseChanges += d.is_c15_FacComputer != prevPhase;
    sfccFallback += !o.logic.sfcc_own_valid;
    lgciuFallback += !o.logic.lgciu_own_valid;
    adrFallback += o.data.bus_inputs.adr_own_bus.airspeed_computed_kn.SSM == 0;
    irFallback += o.data.bus_inputs.ir_own_bus.body_yaw_rate_deg_s.SSM != 3;
    prevAlphaFloor = o.flight_envelope.alpha_floor_condition;
    prevLowEnergy = o.flight_envelope.low_energy_warning_active;
    prevPitch = o.flight_envelope.pitch_pitch_warning_active;
    prevTrim = o.laws.rudder_trim_command_deg;
    prevLatch = d.Memory_PreviousInput;
    prevPhase = d.is_c15_FacComputer;
  }
};

}  // namespace

int main(int argc, char** argv) {
  long scriptSeed = -1, failuresSeed = -1, discretesSeed = -1, powerSeed = -1, simSeed = -1;
  int maxListed = 40;
  long dumpStep = -1;
  bool timeOnly = false;
  std::string path, warmPath;
  bool usage = false;
  for (int i = 1; i < argc && !usage; i++) {
    std::string a = argv[i];
    auto seed = [&](long& v) {
      if (i + 1 < argc) {
        v = std::atol(argv[++i]);
        return true;
      }
      return false;
    };
    if ((a == "--script" && seed(scriptSeed)) || (a == "--failures" && seed(failuresSeed)) ||
        (a == "--discretes" && seed(discretesSeed)) || (a == "--power" && seed(powerSeed)) ||
        (a == "--sim" && seed(simSeed))) {
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else if (a == "--warm" && i + 1 < argc) {
      warmPath = argv[++i];
    } else if (a == "--dump" && i + 1 < argc) {
      dumpStep = std::atol(argv[++i]);
    } else if (a == "--time") {
      timeOnly = true;
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      usage = true;
    }
  }
  if (path.empty() || usage) {
    std::fprintf(stderr,
                 "usage: fac-check [--script SEED] [--failures SEED] [--discretes SEED] [--power SEED] [--sim SEED] "
                 "[--warm FILE] [--time] [--list N] [--dump STEP] <file.fdr>\n");
    return 2;
  }

  std::unique_ptr<FacComputer> generated[2] = {std::make_unique<FacComputer>(), std::make_unique<FacComputer>()};
  R::FacComputer readable[2];
  for (int e = 0; e < 2; e++) {
    generated[e]->initialize();
    readable[e].initialize();
  }
  auto ext = std::make_unique<FacComputer::ExternalInputs_FacComputer_T>();
  FacInputDrive::Options opt;

  if (!warmPath.empty()) {
    FacInputDrive warm(warmPath, opt);
    if (!warm.ok()) {
      std::fprintf(stderr, "error: %s\n", warm.error().c_str());
      return 2;
    }
    S::ScriptScenario script(99);
    while (warm.next([&](fac_inputs (&in)[2], const FacInputDrive::StepInfo&) {
      script.apply(in, in[0].time.dt);
      for (int e = 0; e < 2; e++) {
        ext->in = in[e];
        generated[e]->setExternalInputs(ext.get());
        generated[e]->step();
        readable[e].step(in[e]);
      }
    })) {
    }
    // the generated initialize() is not a complete reset of a used instance: re-construct it in place
    for (int e = 0; e < 2; e++) {
      generated[e]->~FacComputer();
      new (generated[e].get()) FacComputer();
      generated[e]->initialize();
      readable[e].initialize();
    }
    std::printf("warm     %s (%ld frames, script 99), then initialize()\n", warmPath.c_str(), warm.frames());
  }

  FacInputDrive drive(path, opt);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }

  std::unique_ptr<S::ScriptScenario> script;
  if (scriptSeed >= 0) script = std::make_unique<S::ScriptScenario>(scriptSeed);
  std::unique_ptr<S::FailureScenario> failures;
  if (failuresSeed >= 0) failures = std::make_unique<S::FailureScenario>(failuresSeed);
  std::unique_ptr<S::DiscreteScenario> discretes;
  if (discretesSeed >= 0) discretes = std::make_unique<S::DiscreteScenario>(discretesSeed);
  std::unique_ptr<S::PowerScenario> power;
  if (powerSeed >= 0) power = std::make_unique<S::PowerScenario>(powerSeed);
  std::unique_ptr<S::SimScenario> sim;
  if (simSeed >= 0) sim = std::make_unique<S::SimScenario>(simSeed);

  std::vector<FcuLeaf> leaves[2] = {fcuLeaves<fac_outputs>("fac1."), fcuLeaves<fac_outputs>("fac2.")};

  // power-up outputs and the latch table
  long initMismatches = checkTable();
  if (initMismatches) std::printf("init mismatch: SR flip-flop table differs from the readable latch\n");
  for (int e = 0; e < 2; e++) {
    std::vector<FcuLeaf> l0 = fcuLeaves<fac_outputs>();
    initMismatches += compareFcuLeaves(l0, generated[e]->FacComputer_Y.out, readable[e].outputs(), 0, 0, false);
    for (const FcuLeaf& l : l0) {
      if (l.mismatches) std::printf("init mismatch: fac%d.%s\n", e + 1, l.name.c_str());
    }
  }

  std::vector<Hidden> hidden;
  std::vector<std::pair<double, double>> hiddenValues;
  long steps = 0, mismatchSteps = 0, firstMismatch = -1, firstHidden = -1;
  double genSeconds = 0, readSeconds = 0;
  Events events[2];

  auto step = [&](fac_inputs (&in)[2], const FacInputDrive::StepInfo& info) {
    const double dt = in[0].time.dt;
    if (script) script->apply(in, dt);
    if (failures) failures->apply(in, dt);
    if (discretes) discretes->apply(in, dt);
    if (power) power->apply(in, dt);
    if (sim) sim->apply(in, dt);
    steps++;

    int n = 0;
    for (int e = 0; e < 2; e++) {
      ext->in = in[e];
      auto t0 = Clock::now();
      generated[e]->setExternalInputs(ext.get());
      generated[e]->step();
      auto t1 = Clock::now();
      const fac_outputs& r = readable[e].step(in[e]);
      auto t2 = Clock::now();
      genSeconds += std::chrono::duration<double>(t1 - t0).count();
      readSeconds += std::chrono::duration<double>(t2 - t1).count();
      if (timeOnly) continue;
      n += compareFcuLeaves(leaves[e], generated[e]->FacComputer_Y.out, r, steps, in[e].time.simulation_time, true,
                            true);
      events[e].update(*generated[e]);
    }
    if (timeOnly) return;
    if (steps == dumpStep || steps == dumpStep - 1) {
      // every leaf of both FACs' generated outputs (inputs included) at this step and the one before
      for (int e = 0; e < 2; e++) {
        std::vector<FcuLeaf> all = fcuLeaves<fac_outputs>();
        for (const FcuLeaf& l : all) {
          std::printf("dump %ld fac%d.%s %s\n", steps, e + 1, l.name.c_str(),
                      showFcuLeaf(l, &generated[e]->FacComputer_Y.out).c_str());
        }
      }
    }

    size_t k = 0;
    collectHidden(*generated[0], readable[0], "fac1.", hidden, hiddenValues, k);
    collectHidden(*generated[1], readable[1], "fac2.", hidden, hiddenValues, k);
    for (size_t h = 0; h < hidden.size(); h++) {
      auto [hg, hr] = hiddenValues[h];
      if (std::memcmp(&hg, &hr, sizeof(double)) != 0 && !(hg != hg && hr != hr)) {
        Hidden& x = hidden[h];
        if (x.firstStep < 0) {
          x.firstStep = steps;
          x.firstTime = in[0].time.simulation_time;
          x.firstGenerated = hg;
          x.firstReadable = hr;
        }
        x.mismatches++;
        if (firstHidden < 0) firstHidden = steps;
      }
    }
    if (n == 0) return;
    mismatchSteps++;
    if (firstMismatch >= 0) return;
    firstMismatch = steps;
    std::printf("FIRST MISMATCH at step %ld (frame %ld), t = %.3f s\n", steps, info.frame, in[0].time.simulation_time);
    int listed = 0;
    for (const auto& ls : leaves) {
      for (const FcuLeaf& l : ls) {
        if (l.firstStep != steps) continue;
        if (listed++ < maxListed) {
          std::printf("  %-64s generated %-16s readable %-16s\n", l.name.c_str(), l.firstExpected.c_str(),
                      l.firstGot.c_str());
        }
      }
    }
    if (listed > maxListed) std::printf("  ... %d fields in all\n", listed);
    if (firstHidden >= 0) {
      std::printf("  hidden state diverged first at step %ld:", firstHidden);
      for (const Hidden& h : hidden) {
        if (h.firstStep == firstHidden) {
          std::printf(" %s (generated %.17g readable %.17g)", h.name.c_str(), h.firstGenerated, h.firstReadable);
        }
      }
      std::printf("\n");
    } else {
      std::printf("  hidden state: no divergence before this step\n");
    }
  };

  while (drive.next(step)) {
  }

  std::string mode = "inputs rebuilt from the record";
  auto addMode = [&](const char* name, long seed, long episodes) {
    if (seed >= 0) mode += std::string(", ") + name + " (seed " + std::to_string(seed) + ", " + std::to_string(episodes) +
                           " episodes)";
  };
  addMode("script", scriptSeed, script ? script->episodes() : 0);
  addMode("failures", failuresSeed, failures ? failures->episodes() : 0);
  addMode("discretes", discretesSeed, discretes ? discretes->episodes() : 0);
  addMode("power", powerSeed, power ? power->episodes() : 0);
  addMode("sim", simSeed, sim ? sim->episodes() : 0);
  std::printf("file     %s (interface %llu)\nmode     %s\n", path.c_str(),
              static_cast<unsigned long long>(drive.version()), mode.c_str());
  std::printf("steps    %ld (%ld frames), 2 FACs per step\n", steps, drive.frames());
  std::printf("time     generated %.3f us/step, readable %.3f us/step (ratio %.2f)\n", genSeconds / steps / 2 * 1e6,
              readSeconds / steps / 2 * 1e6, readSeconds / genSeconds);
  if (timeOnly) return 0;
  for (int e = 0; e < 2; e++) {
    const Events& v = events[e];
    std::printf("events   fac%d: restarts %ld, alpha floor %ld, low energy %ld, pitch-pitch %ld, trim moving %ld steps, "
                "trim resets %ld, phase changes %ld; steps with own SFCC invalid %ld, own LGCIU invalid %ld, own ADR "
                "failed %ld, own IR not normal %ld\n",
                e + 1, v.restarts, v.alphaFloor, v.lowEnergy, v.pitchPitch, v.trimMoves, v.trimResets, v.phaseChanges,
                v.sfccFallback, v.lgciuFallback, v.adrFallback, v.irFallback);
  }

  long failing = 0;
  std::printf("\n%-64s %9s %10s  %s\n", "field", "mismatch", "first", "first values (generated / readable)");
  for (const auto& ls : leaves) {
    for (const FcuLeaf& l : ls) {
      if (l.mismatches == 0) continue;
      failing++;
      std::printf("%-64s %9ld %10ld  t %.2f %s / %s\n", l.name.c_str(), l.mismatches, l.firstStep, l.firstTime,
                  l.firstExpected.c_str(), l.firstGot.c_str());
    }
  }
  // coverage of the generated outputs: "cov NAME nonzero changes" (not the input copy)
  std::printf("\n");
  for (const auto& ls : leaves) {
    for (const FcuLeaf& l : ls) {
      if (l.name.find(".data.") != std::string::npos) continue;
      std::printf("cov %-64s %10ld %8ld\n", l.name.c_str(), l.nonzero, l.changes);
    }
  }
  long hiddenFailing = 0;
  for (const Hidden& h : hidden) {
    if (h.mismatches == 0) continue;
    hiddenFailing++;
    std::printf("hidden.%-56s %9ld %10ld  t %.2f generated %.17g readable %.17g\n", h.name.c_str(), h.mismatches,
                h.firstStep, h.firstTime, h.firstGenerated, h.firstReadable);
  }
  std::printf("\n%zu fields compared, %ld with mismatches, %ld mismatching steps; %zu hidden memories, %ld diverged; "
              "%ld init mismatches\n",
              leaves[0].size() + leaves[1].size(), failing, mismatchSteps, hidden.size(), hiddenFailing, initMismatches);
  return failing == 0 && hiddenFailing == 0 && initMismatches == 0 ? 0 : 1;
}
