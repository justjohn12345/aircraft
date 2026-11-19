// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Replays the generated ELACs over a flight data recording: both ELACs' inputs are rebuilt from the record
// (src/efcs_drive.h), the generated ElacComputer is stepped with them, and its outputs are compared with the recorded
// ones (bus, discretes, analog orders), bit-exact field by field. On frames where an ELAC was not healthy the recording
// is checked against what the wrapper sends then (every bus word but two FailureWarning with 0, no discrete, 0 orders).
//
// Frames are classed "approximate" when an input is known to be estimated: FMGC 1 not running yet (stand-in sensor
// data) or frozen to the end, an SFCC not known from its FAC, a hydraulic pressure transition (the low pressure
// discretes are estimated), a frame longer than 0.5 s (a slew before it: slew_on unknown), each with the following
// kSettleSeconds while filters and integrators converge. The other frames have every input known except the elevator
// and aileron positions (the float copies on the ELAC bus; the sim's values are doubles), which feed the laws: the
// remaining mismatches measure that.
//
// Usage: elac-replay [--from N] [--to N] [--list N] [--ieee] <file.fdr>
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
#include "elac_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"
#include "replay_compare.h"

using namespace replay_compare;

namespace {

/** The recorded outputs of an ELAC that was not healthy, as the wrapper (Elac::get*Outputs) sends them. */
bool isWrapperUnhealthy(const ElacRecord& r) {
  const base_arinc_429* w = reinterpret_cast<const base_arinc_429*>(&r.bus_outputs);
  for (size_t k = 0; k < sizeof(base_elac_out_bus) / sizeof(base_arinc_429); k++) {
    if (w[k].SSM != efcs_drive::kFw || w[k].Data != 0.0F) return false;
  }
  const base_elac_discrete_outputs& d = r.discrete_outputs;
  const base_elac_analog_outputs& a = r.analog_outputs;
  return !d.pitch_axis_ok && !d.left_aileron_ok && !d.right_aileron_ok && !d.digital_output_validated &&
         !d.ap_1_authorised && !d.ap_2_authorised && !d.left_aileron_active_mode && !d.right_aileron_active_mode &&
         !d.left_elevator_damping_mode && !d.right_elevator_damping_mode && !d.ths_active && !d.batt_power_supply &&
         a.left_elev_pos_order_deg == 0 && a.right_elev_pos_order_deg == 0 && a.ths_pos_order == 0 &&
         a.left_aileron_pos_order == 0 && a.right_aileron_pos_order == 0;
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
    std::fprintf(stderr, "usage: elac-replay [--from N] [--to N] [--list N] [--ieee] <file.fdr>\n");
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
  std::vector<FcuLeaf> bus[2][2], disc[2][2], analog[2][2];
  std::vector<Deviation> dBus[2][2], dDisc[2][2], dAnalog[2][2];
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 2; i++) {
      std::string p = std::string(k ? "approx." : "") + "elac" + std::to_string(i + 1) + ".";
      bus[k][i] = fcuLeaves<base_elac_out_bus>((p + "bus_outputs.").c_str());
      disc[k][i] = fcuLeaves<base_elac_discrete_outputs>((p + "discrete_outputs.").c_str());
      analog[k][i] = fcuLeaves<base_elac_analog_outputs>((p + "analog_outputs.").c_str());
    }
  }
  long compared[2][2] = {{0, 0}, {0, 0}}, mismatchFrames[2] = {0, 0}, unhealthyBad[2] = {0, 0};
  long exactFrames[2] = {0, 0}, firstMismatch = -1;
  double lastApprox = -1e9;

  auto step = [&](elac_inputs (&)[2], const EfcsInputDrive::StepInfo& info) {
    const double t = info.fdr->base.simulation_time_s;
    if (info.approxInputs || info.sfccUnknown[0] || info.sfccUnknown[1] || info.hydTransition || info.longFrame) {
      lastApprox = t;
    }
    const int k = t - lastApprox <= kSettleSeconds ? 1 : 0;
    if (info.frame < from || (to >= 0 && info.frame > to)) return;
    int n = 0;
    for (int i = 0; i < 2; i++) {
      const ElacRecord& rec = info.fdr->elac[i];
      if (!info.elacHealthy[i]) {
        unhealthyBad[i] += !isWrapperUnhealthy(rec);
        continue;
      }
      compared[k][i]++;
      const elac_outputs& g = drive.elac(i).getExternalOutputs().out;
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
  while (drive.next(step)) {
  }

  std::printf("file     %s (interface %llu)\n", path.c_str(), static_cast<unsigned long long>(drive.version()));
  std::printf("frames   %ld recorded; healthy elac1 %ld, elac2 %ld\n", drive.frames(), drive.elacHealthyFrames(0),
              drive.elacHealthyFrames(1));
  std::printf("inputs   fmgc1 not running %ld frames, fmgc2 held %ld, A/B order words differing %ld; SFCC unknown %ld, "
              "hydraulic transitions %ld, long frames %ld, sidestick priority bits %ld; takeover pushbuttons inferred on %ld "
              "frames (unresolved %ld)\n",
              drive.fmgc1Frozen(), drive.fmgc2ApproxFrames(), drive.abMismatch(), drive.sfccUnknownFrames(),
              drive.hydTransitionFrames(), drive.longFrames(), drive.priorityFrames(), drive.priorityPressedFrames(),
              drive.priorityUnresolvedFrames());
  std::printf("unhealthy frames not as the wrapper sends them: elac1 %ld, elac2 %ld\n", unhealthyBad[0], unhealthyBad[1]);

  long failing[2] = {0, 0};
  int listed = 0;
  std::printf("\n%-62s %9s %9s %8s %8s  %s\n", "field", "mismatch", "first", "max ulp", "max diff",
              "first values (recorded / replay)");
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 2; i++) {
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
  std::printf("\nknown inputs:       ELAC frames compared %ld + %ld, bit-exact %ld; %ld fields with mismatches on %ld "
              "frames (first %ld)\n",
              compared[0][0], compared[0][1], exactFrames[0], failing[0], mismatchFrames[0], firstMismatch);
  std::printf("approximate inputs: ELAC frames compared %ld + %ld, bit-exact %ld; %ld fields with mismatches on %ld "
              "frames\n",
              compared[1][0], compared[1][1], exactFrames[1], failing[1], mismatchFrames[1]);
  return failing[0] == 0 && unhealthyBad[0] == 0 && unhealthyBad[1] == 0 ? 0 : 1;
}
