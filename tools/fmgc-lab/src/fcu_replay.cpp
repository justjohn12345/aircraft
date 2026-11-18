// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Replays the generated FCU over a 3209001 flight data recording: each recorded frame's FCU model inputs (as given to
// its step in the sim) go through FcuComputer::step(), and the outputs are compared with the recorded ones (logic,
// discrete outputs, bus outputs: the model's own outputs, before the sim wrapper's changes), bit-exact field by field.
// Unrecorded frames (active pause) are filled by stepping with the level inputs held and no events; mismatches right
// after such a gap can come from pilot actions during the pause.
//
// A match proves the native harness reproduces the sim's FCU (same inputs -> same outputs). On a 3200006 recording
// (no FCU record) it says so and exits 0; fcu-check --synthetic drives the FCU from such files.
//
// Usage: fcu-replay [--from N] [--to N] [--list N] [--sync] <file.fdr>
//   --from/--to   compare only recorded frames in [from, to] (all frames are still stepped)
//   --sync        the recording starts with the FCU already running (e.g. a file split): before the first frame, set
//                 the generated FCU's memories from that frame's recorded outputs (window values, TRK/FPA, METRIC, EFIS
//                 panel states, dash states, change detectors, edge detector inputs) and start comparing at frame 1.
//                 Approximate for timers and pulses in progress. Ignored when the FCU is not running on frame 0.
// Exit status: 0 = every compared field matched (or no FCU data), 1 = mismatches, 2 = usage/file error.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#define private public
#include "FcuComputer.h"
#undef private

#include "fcu_drive.h"
#include "fcu_leaves.h"

namespace {

/** Sets the generated FCU's memories from a recorded frame (--sync). */
void syncFromRecord(FcuComputer& c, const FcuRecord& rec) {
  auto& d = c.FcuComputer_DWork;
  const fcu_inputs& in = rec.inputs;
  const base_afs_logic_outputs& a = rec.logic.afs;
  const double t = in.time.simulation_time;
  d.Runtime_MODE = true;
  d.p_trk_fpa_active = a.trk_fpa_active;
  d.p_metric_alt_active = a.metric_alt_active;
  d.pValue_p = a.spd_mach_display_value;
  d.pValue_not_empty_l = true;
  d.prevMachActive = a.mach_active;
  d.prevMachActive_not_empty = true;
  d.eventTime = a.spd_mach_dashes ? t - 11.0 : t;
  d.eventTime_not_empty = true;
  d.pValue_m = a.hdg_trk_display_value;
  d.pValue_not_empty_p = true;
  d.prevTrkFpaActive_m = a.trk_fpa_active;
  d.prevTrkFpaActive_not_empty_j = true;
  d.sf_MATLABFunction1.eventTime = a.hdg_trk_dashes ? t - 46.0 : t;
  d.sf_MATLABFunction1.eventTime_not_empty = true;
  d.pValue_e = a.alt_display_value;
  d.pValue_not_empty_j = true;
  d.pValue = a.vs_fpa_display_value;
  d.pValue_not_empty = true;
  d.prevTrkFpaActive = a.trk_fpa_active;
  d.prevTrkFpaActive_not_empty = true;
  d.sf_MATLABFunction1_o.eventTime = a.vs_fpa_dashes ? t - 46.0 : t;
  d.sf_MATLABFunction1_o.eventTime_not_empty = true;
  d.DelayInput1_DSTATE[0] = a.hdg_trk_display_value;
  d.DelayInput1_DSTATE[1] = a.alt_display_value;
  d.DelayInput1_DSTATE[2] = a.vs_fpa_display_value;
  d.DelayInput1_DSTATE[3] = a.spd_mach_display_value;
  auto edge = [](FcuComputer::rtDW_MATLABFunction_FcuComputer_e_T& e, bool previous) {
    e.previousInput = previous;
    e.previousInput_not_empty = true;
  };
  const auto& di = in.discrete_inputs;
  const bool anyApFd = a.any_ap_fd_engaged;
  edge(d.sf_MATLABFunction_a4, di.afs_inputs.trk_fpa_button_pressed);
  edge(d.sf_MATLABFunction_ma, di.afs_inputs.metric_alt_button_pressed);
  edge(d.sf_MATLABFunction_kl, a.selected_speed_control);
  edge(d.sf_MATLABFunction_du, !a.hdg_trk_selected && anyApFd);
  edge(d.sf_MATLABFunction_dc, !a.hdg_trk_selected && !a.hdg_trk_preset_available && anyApFd);
  edge(d.sf_MATLABFunction_kw, !a.lvl_ch_vs_fpa && anyApFd);
  edge(d.sf_MATLABFunction_ar, false);
  edge(d.sf_MATLABFunction_df, false);
  edge(d.sf_MATLABFunction_ch, di.ap_1_engaged || di.ap_2_engaged);
  edge(d.sf_MATLABFunction_kj, rec.logic.capt_efis.fd_auto_activate);
  edge(d.sf_MATLABFunction_kq, rec.logic.capt_efis.fd_auto_deactivate);
  const base_fcu_efis_panel_inputs* panels[2] = {&di.capt_efis_inputs, &di.fo_efis_inputs};
  const base_fcu_efis_logic_outputs* logic[2] = {&rec.logic.capt_efis, &rec.logic.fo_efis};
  FcuComputer::rtDW_MATLABFunction_FcuComputer_e_T* edges[2][8] = {
      {&d.sf_MATLABFunction_e1, &d.sf_MATLABFunction_lq, &d.sf_MATLABFunction_ome, &d.sf_MATLABFunction_cl,
       &d.sf_MATLABFunction_gi, &d.sf_MATLABFunction_np, &d.sf_MATLABFunction_hk, &d.sf_MATLABFunction_fv},
      {&d.sf_MATLABFunction_ai, &d.sf_MATLABFunction_mav, &d.sf_MATLABFunction_am, &d.sf_MATLABFunction_m2,
       &d.sf_MATLABFunction_dp, &d.sf_MATLABFunction_hd, &d.sf_MATLABFunction_aa0, &d.sf_MATLABFunction_f0}};
  FcuComputer::rtDW_EFISFilterLogic_FcuComputer_T* filters[2] = {&d.sf_EFISFilterLogic, &d.sf_EFISFilterLogic_k};
  FcuComputer::rtDW_MATLABFunction_FcuComputer_gf_T* fds[2] = {&d.sf_MATLABFunction_ml, &d.sf_MATLABFunction_bx};
  FcuComputer::rtDW_MATLABFunction_FcuComputer_k_T* lss[2] = {&d.sf_MATLABFunction_bs, &d.sf_MATLABFunction_ho3};
  FcuComputer::rtDW_MATLABFunction_FcuComputer_d_T* refs[2] = {&d.sf_MATLABFunction_ofc, &d.sf_MATLABFunction_o3};
  FcuComputer::rtDW_MATLABFunction1_FcuComputer_j_T* baros[2] = {&d.sf_MATLABFunction1_ou, &d.sf_MATLABFunction1_g};
  for (int s = 0; s < 2; s++) {
    const auto& p = *panels[s];
    const auto& l = *logic[s];
    const boolean_T inputs[8] = {p.cstr_button_pushed, p.wpt_button_pushed, p.vord_button_pushed, p.ndb_button_pushed,
                            p.arpt_button_pushed, p.ls_button_pushed, p.baro_knob.pushed, p.baro_knob.pulled};
    for (int i = 0; i < 8; i++) edge(*edges[s][i], inputs[i]);
    filters[s]->pEfisFilter = l.efis_filter;
    fds[s]->fdActive = l.fd_on;
    lss[s]->pY = l.ls_on;
    lss[s]->pY_not_empty = true;
    refs[s]->std_active = l.baro_std;
    refs[s]->qnh_active = l.baro_std ? true : l.baro_qnh;  // the QNH/QFE choice under STD is not on the outputs
    refs[s]->qfe_active = l.baro_qfe;
    baros[s]->pValueHpa = l.baro_value_hpa;
    baros[s]->pValueInhg = l.baro_value_inhg;
  }
  // monostables: previous inputs from this frame, no pulse in progress
  auto mtrig = [](FcuComputer::rtDW_MATLABFunction_FcuComputer_T& m, bool previous) {
    m.previousInput = previous;
    m.remainingTriggerTime = 0.0;
  };
  const auto& afs = di.afs_inputs;
  mtrig(d.sf_MATLABFunction_e, afs.hdg_trk_knob.pushed);
  mtrig(d.sf_MATLABFunction_k, afs.hdg_trk_knob.pulled);
  mtrig(d.sf_MATLABFunction_n, afs.hdg_trk_knob.turns != 0);
  mtrig(d.sf_MATLABFunction_b, afs.spd_knob.pushed);
  mtrig(d.sf_MATLABFunction_l, afs.spd_knob.pulled);
  mtrig(d.sf_MATLABFunction_m, afs.spd_knob.turns != 0);
  mtrig(d.sf_MATLABFunction_kr, afs.alt_knob.pushed);
  mtrig(d.sf_MATLABFunction_f, afs.alt_knob.pulled);
  mtrig(d.sf_MATLABFunction_c, afs.alt_knob.turns != 0);
  mtrig(d.sf_MATLABFunction_a, afs.vs_fpa_knob.pushed);
  mtrig(d.sf_MATLABFunction_o, afs.vs_fpa_knob.pulled);
  mtrig(d.sf_MATLABFunction_kh, afs.vs_fpa_knob.turns != 0);
  mtrig(d.sf_MATLABFunction_d, afs.loc_button_pressed);
  mtrig(d.sf_MATLABFunction_i, afs.exped_button_pressed);
  mtrig(d.sf_MATLABFunction_k0, afs.appr_button_pressed);
  mtrig(d.sf_MATLABFunction_g, afs.spd_mach_button_pressed);
  mtrig(d.sf_MATLABFunction_hh, a.vs_fpa_buttons.pushed);
  mtrig(d.sf_MATLABFunction_ey, false);
  mtrig(d.sf_MATLABFunction_av, false);
  mtrig(d.sf_MATLABFunction_p5r, false);
  mtrig(d.sf_MATLABFunction_lh, false);
  mtrig(d.sf_MATLABFunction_oi, false);
  c.FcuComputer_Y.out.logic = rec.logic;
  c.FcuComputer_Y.out.discrete_outputs = rec.discrete_outputs;
  c.FcuComputer_Y.out.bus_outputs = rec.bus_outputs;
  c.FcuComputer_Y.out.data = in;
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
    std::fprintf(stderr, "usage: fcu-replay [--from N] [--to N] [--list N] [--sync] <file.fdr>\n");
    return 2;
  }

  FcuInputDrive drive(path, {});
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }
  if (!drive.fileHasFcu()) {
    std::printf("file     %s (interface %llu)\nno FCU data in this recording (the FCU record starts with interface "
                "%llu); nothing to replay. fcu-check --synthetic drives the FCU from the FMGC record instead.\n",
                path.c_str(), static_cast<unsigned long long>(drive.version()),
                static_cast<unsigned long long>(FDR_INTERFACE_VERSION_FCU));
    return 0;
  }

  auto computer = std::make_unique<FcuComputer>();
  computer->initialize();
  auto ext = std::make_unique<FcuComputer::ExternalInputs_FcuComputer_T>();

  std::vector<FcuLeaf> logicLeaves = fcuLeaves<base_fcu_logic_outputs>("logic.");
  std::vector<FcuLeaf> discreteLeaves = fcuLeaves<base_fcu_discrete_outputs>("discrete_outputs.");
  std::vector<FcuLeaf> busLeaves = fcuLeaves<base_fcu_bus>("bus_outputs.");

  long steps = 0, compared = 0, mismatchFrames = 0, firstMismatch = -1, lastMismatch = -1, afterGap = 0;
  long notRunning = 0, episodes = 0;
  long lastGapFrame = -1000;
  bool prevMismatch = false;
  bool synced = false;

  auto step = [&](fcu_inputs& in, const FcuInputDrive::StepInfo& info) {
    if (info.gapFill) lastGapFrame = info.frame;
    if (sync && !synced && !info.gapFill) {
      synced = true;
      if (info.fdr->fcu.inputs.sim_data.computer_running) {
        syncFromRecord(*computer, info.fdr->fcu);
        return;  // frame 0's outputs are the synced state
      }
      sync = false;  // the FCU is off on frame 0: it restarts in the recording, nothing to sync
    }
    steps++;
    notRunning += !in.sim_data.computer_running;
    ext->in = in;
    computer->setExternalInputs(ext.get());
    computer->step();
    if (info.gapFill || info.frame < from || (to >= 0 && info.frame > to)) return;
    compared++;
    const fcu_outputs& g = computer->FcuComputer_Y.out;
    const FcuRecord& rec = info.fdr->fcu;
    const double t = in.time.simulation_time;
    int n = compareFcuLeaves(logicLeaves, rec.logic, g.logic, info.frame, t, true);
    n += compareFcuLeaves(discreteLeaves, rec.discrete_outputs, g.discrete_outputs, info.frame, t, true);
    n += compareFcuLeaves(busLeaves, rec.bus_outputs, g.bus_outputs, info.frame, t, true);
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
  std::printf("frames   %ld recorded, %ld stepped (%ld gap-fill steps in %ld gaps, %ld not running), %ld compared%s\n",
              drive.frames(), steps, drive.gapSteps(), drive.gaps(), notRunning, compared,
              sync ? " (synced on frame 0)" : "");
  long failing = 0;
  std::printf("\n%-58s %9s %9s  %s\n", "field", "mismatch", "first", "first values (recorded / replay)");
  for (auto* leaves : {&logicLeaves, &discreteLeaves, &busLeaves}) {
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
              logicLeaves.size() + discreteLeaves.size() + busLeaves.size(), failing, mismatchFrames, episodes, afterGap,
              firstMismatch, lastMismatch);
  return failing == 0 ? 0 : 1;
}
