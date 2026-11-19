// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Replays the generated SECs over a flight data recording: the three SECs' inputs are rebuilt from the record
// (src/efcs_drive.h), the generated SecComputer is stepped with them, and its outputs are compared with the recorded
// ones (bus, discretes, analog orders), bit-exact field by field. On frames where a SEC was not healthy the recording
// is checked against what the wrapper sends then (every bus word but one FailureWarning with 0, no discrete but
// sec_failed, 0 orders). The discrete sec_failed is the wrapper's (self test light) and is not compared.
//
// Frames are classed "approximate" when an input is known to be estimated: FMGC 1 not running yet (stand-in sensor
// data) or frozen to the end, an SFCC not known from its FAC, a hydraulic pressure transition (the low pressure
// discretes are estimated), a frame longer than 0.5 s (a slew before it: slew_on unknown), each with the following
// kSettleSeconds while filters and integrators converge; for the SECs also frames where the wheel speeds (not
// recorded, estimated) are near the 23 kt threshold. The spoiler and elevator positions are the float copies on the
// SEC bus.
//
// Usage: sec-replay [--from N] [--to N] [--list N] [--ieee] <file.fdr>
// Exit status: 0 = every compared field matched, 1 = mismatches, 2 = usage/file error.

#include <xmmintrin.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "efcs_drive.h"
#include "sec_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"
#include "replay_compare.h"

using namespace replay_compare;

namespace {

/** The recorded outputs of an ELAC that was not healthy, as the wrapper (Elac::get*Outputs) sends them. */
bool isWrapperUnhealthy(const SecRecord& r) {
  const base_arinc_429* w = reinterpret_cast<const base_arinc_429*>(&r.bus_outputs);
  for (size_t k = 0; k < sizeof(base_sec_out_bus) / sizeof(base_arinc_429); k++) {
    if (w[k].SSM != efcs_drive::kFw || w[k].Data != 0.0F) return false;
  }
  const base_sec_discrete_outputs& d = r.discrete_outputs;
  const base_sec_analog_outputs& a = r.analog_outputs;
  return !d.thr_reverse_selected && !d.left_elevator_ok && !d.right_elevator_ok && !d.ground_spoiler_out &&
         !d.left_elevator_damping_mode && !d.right_elevator_damping_mode && !d.ths_active && !d.batt_power_supply &&
         a.left_elev_pos_order_deg == 0 && a.right_elev_pos_order_deg == 0 && a.ths_pos_order_deg == 0 &&
         a.left_spoiler_1_pos_order_deg == 0 && a.right_spoiler_1_pos_order_deg == 0 &&
         a.left_spoiler_2_pos_order_deg == 0 && a.right_spoiler_2_pos_order_deg == 0;
}

constexpr double kSettleSeconds = 60.0;

}  // namespace

int main(int argc, char** argv) {
  long from = 0, to = -1;
  int maxListed = 60;
  bool ftz = true;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--from" && i + 1 < argc) {
      from = std::atol(argv[++i]);
    } else if (a == "--to" && i + 1 < argc) {
      to = std::atol(argv[++i]);
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else if (a == "--ieee") {
      ftz = false;
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: sec-replay [--from N] [--to N] [--list N] [--ieee] <file.fdr>\n");
    return 2;
  }
  // the sim runs with denormals flushed to zero (MXCSR FTZ and DAZ)
  if (ftz) _mm_setcsr(_mm_getcsr() | 0x8040);

  EfcsInputDrive drive(path);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }

  // [0]: frames with every input known (but the surface positions); [1]: approximate
  std::vector<FcuLeaf> bus[2][3], disc[2][3], analog[2][3];
  std::vector<Deviation> dBus[2][3], dDisc[2][3], dAnalog[2][3];
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 3; i++) {
      std::string p = std::string(k ? "approx." : "") + "sec" + std::to_string(i + 1) + ".";
      bus[k][i] = fcuLeaves<base_sec_out_bus>((p + "bus_outputs.").c_str());
      disc[k][i] = fcuLeaves<base_sec_discrete_outputs>((p + "discrete_outputs.").c_str());
      analog[k][i] = fcuLeaves<base_sec_analog_outputs>((p + "analog_outputs.").c_str());
    }
  }
  long compared[2][3] = {{0, 0, 0}, {0, 0, 0}}, mismatchFrames[2] = {0, 0}, unhealthyBad[3] = {0, 0, 0};
  long exactFrames[2] = {0, 0}, firstMismatch = -1;
  double lastApprox = -1e9;

  auto elacStep = [&](elac_inputs (&)[2], const EfcsInputDrive::StepInfo&) {};
  auto step = [&](sec_inputs (&)[3], const EfcsInputDrive::StepInfo& info) {
    const double t = info.fdr->base.simulation_time_s;
    if (info.approxInputs || info.sfccUnknown[0] || info.sfccUnknown[1] || info.hydTransition || info.longFrame ||
        info.wheelSpeedApprox) {
      lastApprox = t;
    }
    const int k = t - lastApprox <= kSettleSeconds ? 1 : 0;
    if (info.frame < from || (to >= 0 && info.frame > to)) return;
    int n = 0;
    for (int i = 0; i < 3; i++) {
      const SecRecord& rec = info.fdr->sec[i];
      if (!info.secHealthy[i]) {
        unhealthyBad[i] += !isWrapperUnhealthy(rec);
        continue;
      }
      compared[k][i]++;
      sec_outputs g = drive.sec(i).getExternalOutputs().out;
      g.discrete_outputs.sec_failed = rec.discrete_outputs.sec_failed;  // the wrapper's
      const int ni = compareWithUlps(bus[k][i], dBus[k][i], rec.bus_outputs, g.bus_outputs, info.frame, t) +
                     compareWithUlps(disc[k][i], dDisc[k][i], rec.discrete_outputs, g.discrete_outputs, info.frame, t) +
                     compareWithUlps(analog[k][i], dAnalog[k][i], rec.analog_outputs, g.analog_outputs, info.frame, t);
      exactFrames[k] += ni == 0;
      n += ni;
    }
    if (n) {
      mismatchFrames[k]++;
      if (k == 0 && firstMismatch < 0) firstMismatch = info.frame;
    }
  };
  while (drive.next(elacStep, step)) {
  }

  std::printf("file     %s (interface %llu)\n", path.c_str(), static_cast<unsigned long long>(drive.version()));
  std::printf("frames   %ld recorded; healthy sec1 %ld, sec2 %ld, sec3 %ld\n", drive.frames(), drive.secHealthyFrames(0),
              drive.secHealthyFrames(1), drive.secHealthyFrames(2));
  std::printf("inputs   fmgc1 not running %ld frames, fmgc2 held %ld, A/B order words differing %ld; SFCC unknown %ld, "
              "hydraulic transitions %ld, long frames %ld, sidestick priority bits %ld; takeover pushbuttons inferred on %ld "
              "frames (unresolved %ld)\n",
              drive.fmgc1Frozen(), drive.fmgc2ApproxFrames(), drive.abMismatch(), drive.sfccUnknownFrames(),
              drive.hydTransitionFrames(), drive.longFrames(), drive.priorityFrames(), drive.priorityPressedFrames(),
              drive.priorityUnresolvedFrames());
  std::printf("wheel speeds estimated near a threshold on %ld frames\n", drive.wheelSpeedApproxFrames());
  std::printf("unhealthy frames not as the wrapper sends them: sec1 %ld, sec2 %ld, sec3 %ld\n", unhealthyBad[0],
              unhealthyBad[1], unhealthyBad[2]);

  long failing[2] = {0, 0};
  int listed = 0;
  std::printf("\n%-62s %9s %9s %8s %8s  %s\n", "field", "mismatch", "first", "max ulp", "max diff",
              "first values (recorded / replay)");
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 3; i++) {
      std::pair<std::vector<FcuLeaf>*, std::vector<Deviation>*> groups[] = {
          {&bus[k][i], &dBus[k][i]}, {&disc[k][i], &dDisc[k][i]}, {&analog[k][i], &dAnalog[k][i]}};
      for (auto [leaves, dev] : groups) {
        for (size_t j = 0; j < leaves->size(); j++) {
          const FcuLeaf& l = (*leaves)[j];
          if (l.mismatches == 0) continue;
          failing[k]++;
          if (listed++ < maxListed) {
            std::printf("%-62s %9ld %9ld %8.3g %8.2g  t %.2f %s / %s\n", l.name.c_str(), l.mismatches, l.firstStep,
                        (*dev)[j].ulp, (*dev)[j].abs, l.firstTime, l.firstExpected.c_str(), l.firstGot.c_str());
          }
        }
      }
    }
  }
  std::printf("\nknown inputs:       SEC frames compared %ld + %ld + %ld, bit-exact %ld; %ld fields with mismatches on %ld "
              "frames (first %ld)\n",
              compared[0][0], compared[0][1], compared[0][2], exactFrames[0], failing[0], mismatchFrames[0], firstMismatch);
  std::printf("approximate inputs: SEC frames compared %ld + %ld + %ld, bit-exact %ld; %ld fields with mismatches on %ld "
              "frames\n",
              compared[1][0], compared[1][1], compared[1][2], exactFrames[1], failing[1], mismatchFrames[1]);
  return failing[0] == 0 && unhealthyBad[0] == 0 && unhealthyBad[1] == 0 && unhealthyBad[2] == 0 ? 0 : 1;
}
