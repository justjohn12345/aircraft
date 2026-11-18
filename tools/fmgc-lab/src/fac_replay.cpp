// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Replays the generated FACs over a flight data recording: both FACs' inputs are taken from the record (3209005) or
// rebuilt from it (older formats; src/fac_drive.h), the generated FacComputer is stepped with them, and its outputs
// are compared with the recorded ones (bus, discretes, analog orders), bit-exact field by field, for both FACs. On
// frames where a FAC was not healthy the model is stepped with computer_running false and the recording is checked
// against what the wrapper sends then (every bus word FailureWarning with 0, no function engaged, 0 orders).
//
// What the comparison proves: the native harness reproduces the sim's FACs from the recorded inputs (3209005), or from
// inputs rebuilt from the record. When rebuilt, the rudder trim switches and reset button are not recorded and are
// chosen per frame so that the rudder trim outputs match (the drive's search); the FAC pushbutton and hydraulic
// pressure come from the recorded engaged / available discretes; the SFCC and LGCIU buses are rebuilt from what the FAC
// reported. Every other output is independent of the recorded FAC outputs.
//
// Usage: fac-replay [--from N] [--to N] [--list N] [--no-trim-search] [--rebuild] <file.fdr>
//   --rebuild: rebuild the inputs even when the recording holds them
// Exit status: 0 = every compared field matched, 1 = mismatches, 2 = usage/file error.

#include <xmmintrin.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <utility>
#include <string>
#include <vector>

#include "fac_drive.h"
#include "fac_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"

namespace {

/** The recorded outputs of a FAC that was not healthy, as the wrapper (Fac::get*Outputs) sends them. */
bool isWrapperUnhealthy(const FacRecord& r) {
  const base_arinc_429* w = reinterpret_cast<const base_arinc_429*>(&r.bus_outputs);
  for (size_t k = 0; k < sizeof(base_fac_bus) / sizeof(base_arinc_429); k++) {
    if (w[k].SSM != fac_drive::kFw || w[k].Data != 0.0F) return false;
  }
  const base_fac_discrete_outputs& d = r.discrete_outputs;
  return !d.yaw_damper_engaged && !d.rudder_trim_engaged && !d.rudder_travel_lim_engaged &&
         !d.yaw_damper_avail_for_norm_law && r.analog_outputs.yaw_damper_order_deg == 0.0 &&
         r.analog_outputs.rudder_trim_order_deg == 0.0 && r.analog_outputs.rudder_travel_limit_order_deg == 0.0;
}

/** Distance in units in the last place between two floats or doubles (across zero too); 1e18 for other kinds. */
double ulps(const FcuLeaf& l, const void* a, const void* b) {
  const char* x = static_cast<const char*>(a) + l.offset;
  const char* y = static_cast<const char*>(b) + l.offset;
  if (l.kind != FieldKind::Float) return 1e18;
  // map the bit patterns to a monotonic integer scale (negative values mirrored below zero)
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

double numeric(const FcuLeaf& l, const void* p) {
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
  double ulp = 0;  // largest distance in units in the last place
  double abs = 0;  // largest absolute difference
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

/** After frames with approximate inputs, filter states differ for a while: lag filters with C1 >= 0.4 rad/s. */
constexpr double kSettleSeconds = 60.0;

}  // namespace

int main(int argc, char** argv) {
  long from = 0, to = -1;
  double traceFrom = 1, traceTo = 0;
  int maxListed = 60;
  bool ftz = true;
  FacInputDrive::Options opt;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--from" && i + 1 < argc) {
      from = std::atol(argv[++i]);
    } else if (a == "--to" && i + 1 < argc) {
      to = std::atol(argv[++i]);
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else if (a == "--trace" && i + 2 < argc) {
      traceFrom = std::atof(argv[++i]);
      traceTo = std::atof(argv[++i]);
    } else if (a == "--ieee") {
      ftz = false;
    } else if (a == "--no-trim-search") {
      opt.inferTrim = false;
    } else if (a == "--rebuild") {
      opt.rebuild = true;
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: fac-replay [--from N] [--to N] [--list N] [--trace T0 T1] [--no-trim-search] [--rebuild] [--ieee] "
                         "<file.fdr>\n");
    return 2;
  }

  // The sim runs with denormals flushed to zero (MXCSR FTZ and DAZ): a float output of ~1e-38 reads 0 there.
  if (ftz) _mm_setcsr(_mm_getcsr() | 0x8040);

  FacInputDrive drive(path, opt);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }

  // [0]: frames with every input known; [1]: frames before FMGC 1 first runs (stand-in sensor data, approximate)
  std::vector<FcuLeaf> bus[2][2], disc[2][2], analog[2][2];
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 2; i++) {
      std::string p = std::string(k ? "approx." : "") + "fac" + std::to_string(i + 1) + ".";
      // (approx.: frames with approximate inputs, or within kSettleSeconds after them, while filters converge)
      bus[k][i] = fcuLeaves<base_fac_bus>((p + "bus_outputs.").c_str());
      disc[k][i] = fcuLeaves<base_fac_discrete_outputs>((p + "discrete_outputs.").c_str());
      analog[k][i] = fcuLeaves<base_fac_analog_outputs>((p + "analog_outputs.").c_str());
    }
  }
  std::vector<Deviation> ulpBus[2][2], ulpDisc[2][2], ulpAnalog[2][2];
  long approxCompared = 0, approxMismatchFrames = 0;

  long compared[2] = {0, 0}, unhealthyBad[2] = {0, 0}, mismatchFrames = 0, firstMismatch = -1, lastMismatch = -1;
  long episodes = 0, nearApprox = 0, restarts[2] = {0, 0};
  long lastApprox = -1000;
  double lastApproxTime = -1e9;  // simulation time of the last frame with approximate inputs
  bool prevMismatch = false, prevHealthy[2] = {false, false};

  auto step = [&](fac_inputs (&)[2], const FacInputDrive::StepInfo& info) {
    if (info.fmgc2Approx || !info.fmgc1Running) lastApprox = info.frame;
    if (info.approxInputs) lastApproxTime = info.fdr->base.simulation_time_s;
    const bool approx = info.fdr->base.simulation_time_s - lastApproxTime <= kSettleSeconds;
    int n = 0;
    for (int i = 0; i < 2; i++) {
      restarts[i] += info.healthy[i] && !prevHealthy[i];
      prevHealthy[i] = info.healthy[i];
    }
    if (info.frame < from || (to >= 0 && info.frame > to)) return;
    const double t = info.fdr->base.simulation_time_s;
    if (t >= traceFrom && t <= traceTo) {
      const fac_outputs& g = drive.reference(0).getExternalOutputs().out;
      const FacRecord& r = info.fdr->fac[0];
      std::printf("trace %ld t %.3f dt %.17g Vias %.9g trend rec %.9g gen %.9g yd rec %.17g gen %.17g r %.9g\n", info.frame,
                  t, info.fdr->base.simulation_delta_time_s, g.logic.adr_computation_data.V_ias_kn,
                  r.bus_outputs.speed_trend_kn.Data, g.bus_outputs.speed_trend_kn.Data,
                  r.analog_outputs.yaw_damper_order_deg, g.analog_outputs.yaw_damper_order_deg,
                  g.logic.ir_computation_data.r_deg_s);
    }
    for (int i = 0; i < 2; i++) {
      const FacRecord& rec = info.fdr->fac[i];
      if (!info.healthy[i]) {
        unhealthyBad[i] += !isWrapperUnhealthy(rec);
        continue;
      }
      const int k = approx ? 1 : 0;
      (k ? approxCompared : compared[i])++;
      const fac_outputs& g = drive.reference(i).getExternalOutputs().out;
      n += compareWithUlps(bus[k][i], ulpBus[k][i], rec.bus_outputs, g.bus_outputs, info.frame, t);
      n += compareWithUlps(disc[k][i], ulpDisc[k][i], rec.discrete_outputs, g.discrete_outputs, info.frame, t);
      n += compareWithUlps(analog[k][i], ulpAnalog[k][i], rec.analog_outputs, g.analog_outputs, info.frame, t);
    }
    if (n > 0 && approx) {
      approxMismatchFrames++;
      n = 0;
    }
    if (n > 0) {
      mismatchFrames++;
      episodes += !prevMismatch;
      nearApprox += info.frame - lastApprox <= 2;
      if (firstMismatch < 0) firstMismatch = info.frame;
      lastMismatch = info.frame;
    }
    prevMismatch = n > 0;
  };
  while (drive.next(step)) {
  }

  std::printf("file     %s (interface %llu)\n", path.c_str(), static_cast<unsigned long long>(drive.version()));
  std::printf("frames   %ld recorded; healthy fac1 %ld, fac2 %ld (restarts %ld, %ld); compared fac1 %ld, fac2 %ld\n",
              drive.frames(), drive.healthyFrames(0), drive.healthyFrames(1), restarts[0], restarts[1], compared[0],
              compared[1]);
  if (drive.recordedInputs()) {
    std::printf("inputs   recorded (3209005); not stepped (power outage) fac1 %ld, fac2 %ld frames; %ld active pause gaps\n",
                drive.notStepped(0), drive.notStepped(1), drive.gaps());
    std::printf("trim     rudder trim inputs on %ld frames (recorded)\n", drive.trimFrames());
  } else {
    std::printf("inputs   rebuilt; fmgc1 not running %ld frames, fmgc2 held %ld frames, %ld active pause gaps, A/B word "
                "check %ld\n",
                drive.fmgc1Frozen(), drive.fmgc2ApproxFrames(), drive.gaps(), drive.abMismatch());
    std::printf("trim     inputs inferred on %ld frames (search on %ld, unresolved %ld)%s\n", drive.trimFrames(),
                drive.trimSearched(), drive.trimUnresolved(), opt.inferTrim ? "" : " (search off)");
  }
  std::printf("unhealthy frames not as the wrapper sends them: fac1 %ld, fac2 %ld\n", unhealthyBad[0], unhealthyBad[1]);

  long failing = 0, approxFailing = 0;
  double worstUlp = 0;
  size_t total = 0;
  std::printf("\n%-58s %9s %9s %8s %8s  %s\n", "field", "mismatch", "first", "max ulp", "max diff",
              "first values (recorded / replay)");
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 2; i++) {
      std::pair<std::vector<FcuLeaf>*, std::vector<Deviation>*> groups[] = {
          {&bus[k][i], &ulpBus[k][i]}, {&disc[k][i], &ulpDisc[k][i]}, {&analog[k][i], &ulpAnalog[k][i]}};
      for (auto [leaves, ulp] : groups) {
        total += k ? 0 : leaves->size();
        for (size_t j = 0; j < leaves->size(); j++) {
          const FcuLeaf& l = (*leaves)[j];
          if (l.mismatches == 0) continue;
          (k ? approxFailing : failing)++;
          if (!k) worstUlp = std::fmax(worstUlp, (*ulp)[j].ulp);
          if (failing + approxFailing <= maxListed) {
            std::printf("%-58s %9ld %9ld %8.3g %8.2g  t %.2f %s / %s\n", l.name.c_str(), l.mismatches, l.firstStep,
                        (*ulp)[j].ulp, (*ulp)[j].abs, l.firstTime, l.firstExpected.c_str(), l.firstGot.c_str());
          }
        }
      }
    }
  }
  std::printf("\napproximate inputs (FMGC 1 not running, and %.0f s after): %ld FAC frames compared, %ld fields with "
              "mismatches on %ld frames\n",
              kSettleSeconds,
              approxCompared, approxFailing, approxMismatchFrames);
  std::printf("largest difference on frames with known inputs: %.3g ulp\n", worstUlp);
  std::printf("%zu fields compared, %ld with mismatches, %ld mismatching frames in %ld episodes (%ld within 2 frames "
              "of approximate FMGC inputs); first %ld, last %ld\n",
              total, failing, mismatchFrames, episodes, nearApprox, firstMismatch, lastMismatch);
  return failing == 0 && unhealthyBad[0] == 0 && unhealthyBad[1] == 0 ? 0 : 1;
}
