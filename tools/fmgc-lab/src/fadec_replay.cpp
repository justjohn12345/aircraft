// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Replays the generated FADECs over a 3209004 flight data recording: each recorded frame's FADEC model inputs (as given
// to their steps in the sim) go through FadecComputer::step(), and the outputs (data_computed, output, ECU bus) are
// compared with the recorded ones, bit-exact field by field, for both FADECs. FADEC 1's output and bus are also in the
// stock part of the record (FadecRecord) and are compared too. Unrecorded frames (active pause) are filled by stepping
// with the inputs held; mismatches right after such a gap can come from inputs that changed during the pause.
//
// A match proves the native harness reproduces the sim's FADECs (same inputs -> same outputs). The recording usually
// starts with the FADECs running: their memories (latches, timers, integrators) are unknown on frame 0. --sync sets
// them from frame 0's recorded outputs and inputs (exact for the latches; the integrators from the throttle output,
// the trim integrator 0, timers from the recorded times) and compares from frame 1.
//
// On a recording without the FADEC model record it says so and exits 0; fadec-check drives the FADECs from such files.
//
// Usage: fadec-replay [--from N] [--to N] [--list N] [--sync] <file.fdr>
// Exit status: 0 = every compared field matched (or no FADEC data), 1 = mismatches, 2 = usage/file error.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#define private public
#include "FadecComputer.h"
#undef private

#include "fadec_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"

#include "fadec_drive.h"

namespace {

/** Sets the generated FADEC's memories from a recorded frame (--sync). */
void syncFromRecord(FadecComputer& c, const FadecModelRecord& rec) {
  auto& d = c.FadecComputer_DWork;
  const athr_in& in = rec.in;
  const double t = in.time.simulation_time;
  d.Memory_PreviousInput = rec.data_computed.ATHR_disabled;
  d.Memory_PreviousInput_p = rec.output.athr_control_active;
  d.Memory_PreviousInput_j = rec.output.memo_thrust_active;
  d.latch = !in.data.on_ground && rec.data_computed.is_FLX_active;
  d.pU = rec.output.memo_thrust_active ? rec.output.N1_c_percent : in.data.engine_N1_percent;
  d.pU_not_empty = true;
  d.Delay_DSTATE = 0.0;
  d.Delay_DSTATE_n = rec.output.is_in_reverse ? 0.0 : rec.output.sim_throttle_lever_pos;
  d.Delay_DSTATE_l = rec.output.is_in_reverse ? rec.output.sim_throttle_lever_pos : 0.0;
  d.sf_TimeSinceCondition.eventTime = t;  // a disconnect held across the file start restarts its 15 s
  d.sf_TimeSinceCondition.eventTime_not_empty = true;
  d.sf_TimeSinceCondition1.eventTime = t - rec.data_computed.time_since_touchdown;
  d.sf_TimeSinceCondition1.eventTime_not_empty = true;
  const bool engaged = ((static_cast<uint32_t>(in.fcu_input.ats_discrete_word.Data) >> 12) & 1U) &&
                       in.fcu_input.n1_cmd_percent.SSM == static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
  d.sf_MATLABFunction_f.previousInput = engaged;
  d.sf_MATLABFunction_f.previousInput_not_empty = true;
  d.sf_MATLABFunction_b.previousInput = !rec.output.athr_control_active;
  d.sf_MATLABFunction_b.previousInput_not_empty = true;
  auto& y = c.FadecComputer_Y.out;
  y.time = in.time, y.data = in.data, y.input = in.input, y.fcu_input = in.fcu_input;
  y.data_computed = rec.data_computed;
  y.output = rec.output;
  y.fadec_bus_output = rec.bus_outputs;
}

}  // namespace

int main(int argc, char** argv) {
  long from = 0, to = -1;
  int maxListed = 40;
  bool sync = false;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--from" && i + 1 < argc) {
      from = std::atol(argv[++i]);
    } else if (a == "--to" && i + 1 < argc) {
      to = std::atol(argv[++i]);
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else if (a == "--sync") {
      sync = true;
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: fadec-replay [--from N] [--to N] [--list N] [--sync] <file.fdr>\n");
    return 2;
  }

  FadecInputDrive drive(path, {});
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }
  if (!drive.fileHasFadec()) {
    std::printf("file     %s (interface %llu)\nno FADEC model data in this recording (it starts with interface %llu); "
                "nothing to replay. fadec-check drives the FADECs synthetically from such files.\n",
                path.c_str(), static_cast<unsigned long long>(drive.version()),
                static_cast<unsigned long long>(FDR_INTERFACE_VERSION_FADEC));
    return 0;
  }

  std::unique_ptr<FadecComputer> computer[2] = {std::make_unique<FadecComputer>(), std::make_unique<FadecComputer>()};
  for (auto& c : computer) c->initialize();
  auto ext = std::make_unique<FadecComputer::ExternalInputs_FadecComputer_T>();

  std::vector<FcuLeaf> computed[2], output[2], bus[2];
  for (int e = 0; e < 2; e++) {
    std::string p = "fadec" + std::to_string(e + 1) + ".";
    computed[e] = fcuLeaves<athr_data_computed>((p + "data_computed.").c_str());
    output[e] = fcuLeaves<athr_output>((p + "output.").c_str());
    bus[e] = fcuLeaves<base_ecu_bus>((p + "fadec_bus_output.").c_str());
  }
  std::vector<FcuLeaf> stockOutput = fcuLeaves<athr_output>("stock_record.fadec1.output.");
  std::vector<FcuLeaf> stockBus = fcuLeaves<base_ecu_bus>("stock_record.fadec1.fadec_bus_output.");

  long steps = 0, compared = 0, mismatchFrames = 0, firstMismatch = -1, lastMismatch = -1, afterGap = 0, episodes = 0;
  long lastGapFrame = -1000;
  bool prevMismatch = false, synced = false;

  auto step = [&](athr_in (&in)[2], const FadecInputDrive::StepInfo& info) {
    if (info.gapFill) lastGapFrame = info.frame;
    if (sync && !synced && !info.gapFill) {
      synced = true;
      for (int e = 0; e < 2; e++) syncFromRecord(*computer[e], info.fdr->fadecModel[e]);
      return;  // frame 0's outputs are the synced state
    }
    steps++;
    for (int e = 0; e < 2; e++) {
      ext->in = in[e];
      computer[e]->setExternalInputs(ext.get());
      computer[e]->step();
    }
    if (info.gapFill || info.frame < from || (to >= 0 && info.frame > to)) return;
    compared++;
    const double t = in[0].time.simulation_time;
    int n = 0;
    for (int e = 0; e < 2; e++) {
      const athr_out& g = computer[e]->FadecComputer_Y.out;
      const FadecModelRecord& rec = info.fdr->fadecModel[e];
      n += compareFcuLeaves(computed[e], rec.data_computed, g.data_computed, info.frame, t, true);
      n += compareFcuLeaves(output[e], rec.output, g.output, info.frame, t, true);
      n += compareFcuLeaves(bus[e], rec.bus_outputs, g.fadec_bus_output, info.frame, t, true);
    }
    const athr_out& g1 = computer[0]->FadecComputer_Y.out;
    n += compareFcuLeaves(stockOutput, info.fdr->fadec[0].outputs, g1.output, info.frame, t, true);
    n += compareFcuLeaves(stockBus, info.fdr->fadec[0].bus_outputs, g1.fadec_bus_output, info.frame, t, true);
    if (n > 0) {
      mismatchFrames++;
      episodes += !prevMismatch;
      afterGap += info.frame - lastGapFrame <= 40;
      if (firstMismatch < 0) firstMismatch = info.frame;
      lastMismatch = info.frame;
    }
    prevMismatch = n > 0;
  };
  while (drive.next(step)) {
  }

  std::printf("file     %s (interface %llu)\n", path.c_str(), static_cast<unsigned long long>(drive.version()));
  std::printf("frames   %ld recorded, %ld stepped (%ld gap-fill steps in %ld gaps), %ld compared%s\n", drive.frames(),
              steps, drive.gapSteps(), drive.gaps(), compared, sync ? " (synced on frame 0)" : "");
  long failing = 0;
  size_t total = 0;
  std::printf("\n%-58s %9s %9s  %s\n", "field", "mismatch", "first", "first values (recorded / replay)");
  std::vector<std::vector<FcuLeaf>*> all = {&computed[0], &output[0], &bus[0],    &computed[1],
                                            &output[1],   &bus[1],    &stockOutput, &stockBus};
  for (auto* leaves : all) {
    total += leaves->size();
    int listed = 0;
    for (const FcuLeaf& l : *leaves) {
      if (l.mismatches == 0) continue;
      failing++;
      if (listed++ < maxListed) {
        std::printf("%-58s %9ld %9ld  t %.2f %s / %s\n", l.name.c_str(), l.mismatches, l.firstStep, l.firstTime,
                    l.firstExpected.c_str(), l.firstGot.c_str());
      }
    }
  }
  std::printf("\n%zu fields compared, %ld with mismatches, %ld mismatching frames in %ld episodes (%ld within 40 frames "
              "after a gap); first %ld, last %ld\n",
              total, failing, mismatchFrames, episodes, afterGap, firstMismatch, lastMismatch);
  return failing == 0 ? 0 : 1;
}
