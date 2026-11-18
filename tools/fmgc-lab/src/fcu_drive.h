// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FCU inputs from a flight data recording, for the FCU replay (src/fcu_replay.cpp) and the differential check
// (src/check_fcu.cpp).
//
// Two sources:
//  - Recorded (personal format 3209001): the FCU model inputs exactly as given to its step in the sim
//    (FcuRecord::inputs), with the recorded outputs to compare against.
//  - Synthetic (any recording, e.g. 3200006, which has no FCU record): built the way FlyByWireInterface::updateFcu()
//    builds them, from what the FMGC 1 record holds. The FCU runs before the FMGCs in a frame, so it sees the FMGCs'
//    previous-frame outputs:
//      fmgc_1_bus        = FMGC 1's A bus output of the previous recorded frame (fmgc1.bus_outputs.fmgc_a_bus)
//      fmgc_2_bus        = FMGC 2's A bus as FMGC 1 received it this frame (fmgc1.bus_inputs.fmgc_opp_bus), which is
//                          also FMGC 2's previous-frame output
//      AP/FD/A/THR 1     = FMGC 1's discrete outputs of the previous recorded frame
//      AP/FD/A/THR 2     = FMGC 1's opposite-FMGC discrete inputs this frame
//      time              = the recorded dt and simulation time (as the FMGC replay)
//      panel, sim_input  = idle (no knob/pb action, no value set from the sim, -1), EFIS panels at a fixed default,
//                          unless --reconstruct infers the pilot's actions from the recorded FCU bus (below) or a
//                          scenario injector drives them
//    FMGC 2 variants (--fmgc2): opp (default, as above), copy (FMGC 1's bus on both), invalid (every FMGC 2 word SSM
//    failure warning, data 0), ncd (SSM no computed data, data 0), swapped (FMGC 1 and 2 buses and discretes swapped).
//
// Unrecorded frames (active pause: the recorder stops, the FCU keeps running) show as a simulation-time jump; they are
// filled with steps at the last dt that hold the level inputs and clear the one-frame events (knob turns, pushes,
// sim_input values).
//
// Reconstruction (--reconstruct, synthetic source only): FMGC 1 records the FCU bus it receives (bus_inputs.fcu_bus,
// this frame's FCU output). Its pulse bits (knob pushes/pulls, LOC/APPR/EXPED/SPD-MACH pbs) give the presses; the EIS
// words give the EFIS selectors (mode, range, navaids, baro unit); the TRK-FPA, METRIC, LS, FD, filter and baro
// reference bits, and the selected values (HDG/TRK, ALT, SPD/MACH, V/S-FPA, baro setting), are followed by pressing
// or turning when the computer under test differs from the recording (feedback on its previous outputs). FCU health
// (fmgc1.discrete_inputs.fcu_own_healthy) drives computer_running. It is a realistic drive, not an exact replay.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "FcuComputer_types.h"
#include "fdr.h"

enum class Fmgc2Mode { Opp, Copy, Invalid, Ncd, Swapped };

inline bool parseFmgc2Mode(const std::string& s, Fmgc2Mode& m) {
  if (s == "opp") m = Fmgc2Mode::Opp;
  else if (s == "copy") m = Fmgc2Mode::Copy;
  else if (s == "invalid") m = Fmgc2Mode::Invalid;
  else if (s == "ncd") m = Fmgc2Mode::Ncd;
  else if (s == "swapped") m = Fmgc2Mode::Swapped;
  else return false;
  return true;
}

namespace fcu_drive {

inline bool arincBit(const base_arinc_429& w, int bit) {
  float r = std::round(w.Data);
  uint32_t v = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  return ((v >> (bit - 1)) & 1U) != 0;
}
inline bool isNormal(const base_arinc_429& w) {
  return w.SSM == static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
}

/** Every word of an FMGC A bus set to `ssm` with data 0. */
inline void deadBus(base_fmgc_a_bus& b, uint32_t ssm) {
  auto* w = reinterpret_cast<base_arinc_429*>(&b);
  for (size_t i = 0; i < sizeof(base_fmgc_a_bus) / sizeof(base_arinc_429); i++) {
    w[i].SSM = ssm;
    w[i].Data = 0.0F;
  }
}

/** Clears the one-frame events of the inputs (for gap-fill steps): knob turns/pushes/pulls, pbs, sim_input values. */
inline void clearEvents(fcu_inputs& in) {
  in.sim_input = {-1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F};
  auto& a = in.discrete_inputs.afs_inputs;
  a.loc_button_pressed = a.exped_button_pressed = a.appr_button_pressed = false;
  a.spd_mach_button_pressed = a.trk_fpa_button_pressed = a.metric_alt_button_pressed = false;
  for (base_fcu_knob_inputs* k : {&a.spd_knob, &a.hdg_trk_knob, &a.alt_knob, &a.vs_fpa_knob}) {
    *k = {false, false, 0};
  }
  for (base_fcu_efis_panel_inputs* e : {&in.discrete_inputs.capt_efis_inputs, &in.discrete_inputs.fo_efis_inputs}) {
    e->baro_knob = {false, false, 0};
    e->fd_button_pushed = e->ls_button_pushed = e->cstr_button_pushed = e->wpt_button_pushed = false;
    e->vord_button_pushed = e->ndb_button_pushed = e->arpt_button_pushed = false;
  }
}

/** The default EFIS panel of the synthetic drive: ARC, 40 nm, no navaid, hPa. */
inline base_fcu_efis_panel_inputs defaultEfisPanel() {
  base_fcu_efis_panel_inputs p{};
  p.efis_range = efis_range_selection::RANGE_40;
  p.efis_mode = efis_mode_selection::ARC;
  p.efis_navaid_1 = efis_navaid_selection::NONE;
  p.efis_navaid_2 = efis_navaid_selection::NONE;
  return p;
}

}  // namespace fcu_drive

class FcuInputDrive {
 public:
  struct Options {
    bool synthetic = false;  // build the inputs from the FMGC record even when the file has the FCU record
    Fmgc2Mode fmgc2 = Fmgc2Mode::Opp;
    bool reconstruct = false;
    bool fillGaps = true;
  };

  struct StepInfo {
    long frame;           // recorded frame index
    bool gapFill;         // an unrecorded step (active pause) before that frame
    const FdrFrame* fdr;  // the recorded frame
    bool recordedInputs;  // the inputs are the recorded FCU inputs (3209001)
  };

  FcuInputDrive(const std::string& path, Options opt) : reader_(path), opt_(opt) {
    frame_ = std::make_unique<FdrFrame>();
    prevFmgc_ = std::make_unique<FmgcRecord>();
    prevFcuBus_ = std::make_unique<base_fcu_bus>();
    in_ = std::make_unique<fcu_inputs>();
    std::memset(in_.get(), 0, sizeof(fcu_inputs));
    fcu_drive::clearEvents(*in_);
    in_->discrete_inputs.capt_efis_inputs = fcu_drive::defaultEfisPanel();
    in_->discrete_inputs.fo_efis_inputs = fcu_drive::defaultEfisPanel();
    in_->sim_data.computer_running = true;
  }

  bool ok() const { return reader_.ok(); }
  const std::string& error() const { return reader_.error(); }
  bool fileHasFcu() const { return reader_.hasFcu(); }
  uint64_t version() const { return reader_.version(); }
  bool recordedInputs() const { return reader_.hasFcu() && !opt_.synthetic; }
  long frames() const { return frames_; }
  long gapSteps() const { return gapSteps_; }
  long gaps() const { return gaps_; }

  /** The computer under test's last outputs, for --reconstruct feedback (may be null). */
  const fcu_outputs* feedback = nullptr;

  /**
   * Reads the next recorded frame and calls `step` for each step it implies (gap fills, then the frame), with the
   * inputs to use (the callee may modify them for this step only: they are rebuilt from the recording every frame,
   * and gap fills copy them first). False at the end of the file.
   */
  bool next(const std::function<void(fcu_inputs&, const StepInfo&)>& step) {
    if (!reader_.next(*frame_)) {
      return false;
    }
    const FdrFrame& f = *frame_;
    const BaseData& base = f.base;
    if (frames_ > 0 && opt_.fillGaps) {
      double gap = base.simulation_time_s - lastTime_ - base.simulation_delta_time_s;
      double fillDt = lastDt_ > 0 ? lastDt_ : base.simulation_delta_time_s;
      if (gap > 0.1 && fillDt > 0) {
        long steps = std::lround(gap / fillDt);
        fillDt = gap / steps;
        fcu_inputs held = last_;
        fcu_drive::clearEvents(held);
        for (long i = 0; i < steps; i++) {
          monotonic_ += fillDt;
          held.time.dt = fillDt;
          held.time.simulation_time += fillDt;
          held.time.monotonic_time = monotonic_;
          fcu_inputs s = held;
          step(s, {frames_, true, &f, recordedInputs()});
        }
        gapSteps_ += steps;
        gaps_++;
      }
    }

    if (recordedInputs()) {
      *in_ = f.fcu.inputs;
      monotonic_ = in_->time.monotonic_time;
    } else {
      buildSynthetic(f);
    }
    last_ = *in_;
    lastTime_ = base.simulation_time_s;
    lastDt_ = base.simulation_delta_time_s;
    fcu_inputs s = *in_;
    step(s, {frames_, false, &f, recordedInputs()});
    *prevFmgc_ = f.fmgc1;
    *prevFcuBus_ = f.fmgc1.bus_inputs.fcu_bus;
    frames_++;
    return true;
  }

  /** Reconstruction statistics. */
  long reconstructedPresses = 0, reconstructedTurns = 0;

 private:
  void buildSynthetic(const FdrFrame& f) {
    fcu_inputs& in = *in_;
    const FmgcRecord& rec = f.fmgc1;
    const FmgcRecord& prev = frames_ > 0 ? *prevFmgc_ : rec;
    monotonic_ += f.base.simulation_delta_time_s;
    in.time.dt = f.base.simulation_delta_time_s;
    in.time.simulation_time = f.base.simulation_time_s;
    in.time.monotonic_time = monotonic_;
    in.sim_data = {};
    in.sim_data.computer_running = true;
    fcu_drive::clearEvents(in);

    auto& d = in.discrete_inputs;
    base_fmgc_a_bus own = prev.bus_outputs.fmgc_a_bus;
    base_fmgc_a_bus opp = rec.bus_inputs.fmgc_opp_bus;
    bool ap1 = prev.discrete_outputs.ap_own_engaged, fd1 = prev.discrete_outputs.fd_own_engaged,
         athr1 = prev.discrete_outputs.athr_own_engaged;
    bool ap2 = rec.discrete_inputs.ap_opp_engaged, fd2 = rec.discrete_inputs.fd_opp_engaged,
         athr2 = rec.discrete_inputs.athr_opp_engaged;
    switch (opt_.fmgc2) {
      case Fmgc2Mode::Opp:
        break;
      case Fmgc2Mode::Copy:
        opp = own;
        ap2 = ap1, fd2 = fd1, athr2 = athr1;
        break;
      case Fmgc2Mode::Invalid:
        fcu_drive::deadBus(opp, static_cast<uint32_t>(SignStatusMatrix::FailureWarning));
        ap2 = fd2 = athr2 = false;
        break;
      case Fmgc2Mode::Ncd:
        fcu_drive::deadBus(opp, static_cast<uint32_t>(SignStatusMatrix::NoComputedData));
        ap2 = fd2 = athr2 = false;
        break;
      case Fmgc2Mode::Swapped:
        std::swap(own, opp);
        std::swap(ap1, ap2), std::swap(fd1, fd2), std::swap(athr1, athr2);
        break;
    }
    in.bus_inputs.fmgc_1_bus = own;
    in.bus_inputs.fmgc_2_bus = opp;
    d.ap_1_engaged = ap1, d.fd_1_engaged = fd1, d.athr_1_engaged = athr1;
    d.ap_2_engaged = ap2, d.fd_2_engaged = fd2, d.athr_2_engaged = athr2;
    d.lights_test = false;
    d.pin_prog_qfe_avail = false;
    if (opt_.reconstruct) {
      reconstruct(f);
    }
  }

  void press(boolean_T& b) {
    b = true;
    reconstructedPresses++;
  }
  void turn(int8_T& t, double clicks) {
    double c = std::fmax(std::fmin(std::round(clicks), 127.0), -127.0);
    if (c != 0) {
      t = static_cast<int8_T>(c);
      reconstructedTurns++;
    }
  }
  static float dataIfNormal(const base_arinc_429& w, bool& normal) {
    normal = fcu_drive::isNormal(w);
    return w.Data;
  }

  void reconstruct(const FdrFrame& f) {
    using fcu_drive::arincBit;
    using fcu_drive::isNormal;
    fcu_inputs& in = *in_;
    const base_fcu_bus& r = f.fmgc1.bus_inputs.fcu_bus;
    const base_fcu_bus& p = *prevFcuBus_;
    in.sim_data.computer_running = f.fmgc1.discrete_inputs.fcu_own_healthy;
    if (!isNormal(r.fcu_discrete_word_1)) {
      return;  // FCU bus dead in the recording (FCU unhealthy): nothing to read
    }
    auto rising = [&](const base_arinc_429& now, const base_arinc_429& before, int bit) {
      return arincBit(now, bit) && !(frames_ > 0 && arincBit(before, bit));
    };
    auto& a = in.discrete_inputs.afs_inputs;
    const auto& w1 = r.fcu_discrete_word_1;
    const auto& w2 = r.fcu_discrete_word_2;
    if (rising(w1, p.fcu_discrete_word_1, 11)) press(a.spd_knob.pushed);
    if (rising(w1, p.fcu_discrete_word_1, 12)) press(a.spd_knob.pulled);
    if (rising(w1, p.fcu_discrete_word_1, 16)) press(a.vs_fpa_knob.pushed);
    if (rising(w1, p.fcu_discrete_word_1, 17)) press(a.alt_knob.pushed);
    if (rising(w1, p.fcu_discrete_word_1, 18)) press(a.alt_knob.pulled);
    if (rising(w1, p.fcu_discrete_word_1, 19)) press(a.vs_fpa_knob.pulled);
    if (rising(w1, p.fcu_discrete_word_1, 21)) press(a.spd_mach_button_pressed);
    if (rising(w1, p.fcu_discrete_word_1, 22)) press(a.exped_button_pressed);
    if (rising(w1, p.fcu_discrete_word_1, 23)) press(a.appr_button_pressed);
    if (rising(w2, p.fcu_discrete_word_2, 11)) press(a.hdg_trk_knob.pushed);
    if (rising(w2, p.fcu_discrete_word_2, 12)) press(a.hdg_trk_knob.pulled);
    if (rising(w2, p.fcu_discrete_word_2, 13)) press(a.loc_button_pressed);

    // EFIS selectors from the EIS words (range 320 has no bit: none set -> 320)
    auto efis = [&](base_fcu_efis_panel_inputs& e, const base_arinc_429& eis1, const base_arinc_429& eis2) {
      if (!isNormal(eis1) || !isNormal(eis2)) return;
      e.baro_is_inhg = arincBit(eis1, 11);
      e.efis_range = efis_range_selection::RANGE_320;
      for (int i = 0; i < 5; i++) {
        if (arincBit(eis1, 25 + i)) e.efis_range = static_cast<efis_range_selection>(i);
      }
      static const efis_mode_selection modes[5] = {efis_mode_selection::PLAN, efis_mode_selection::ARC,
                                                   efis_mode_selection::ROSE_NAV, efis_mode_selection::ROSE_VOR,
                                                   efis_mode_selection::ROSE_ILS};
      for (int i = 0; i < 5; i++) {
        if (arincBit(eis2, 11 + i)) e.efis_mode = modes[i];
      }
      e.efis_navaid_1 = arincBit(eis2, 24) ? efis_navaid_selection::ADF
                                           : (arincBit(eis2, 26) ? efis_navaid_selection::VOR : efis_navaid_selection::NONE);
      e.efis_navaid_2 = arincBit(eis2, 25) ? efis_navaid_selection::ADF
                                           : (arincBit(eis2, 27) ? efis_navaid_selection::VOR : efis_navaid_selection::NONE);
    };
    efis(in.discrete_inputs.capt_efis_inputs, r.eis_discrete_word_1_left, r.eis_discrete_word_2_left);
    efis(in.discrete_inputs.fo_efis_inputs, r.eis_discrete_word_1_right, r.eis_discrete_word_2_right);

    if (!feedback) return;
    const fcu_outputs& g = *feedback;
    // panel latches: press when the computer under test differs from the recording (not twice in a row: edges)
    const bool recTrk = arincBit(w1, 25), recMetric = arincBit(w1, 20);
    if (recTrk != g.logic.afs.trk_fpa_active && !lastTrkPress_) press(a.trk_fpa_button_pressed);
    lastTrkPress_ = a.trk_fpa_button_pressed;
    if (recMetric != g.logic.afs.metric_alt_active && !lastMetricPress_) press(a.metric_alt_button_pressed);
    lastMetricPress_ = a.metric_alt_button_pressed;

    // EFIS latches and baro
    auto efisFollow = [&](base_fcu_efis_panel_inputs& e, const base_fcu_efis_logic_outputs& gl, const base_arinc_429& eis2,
                          const base_arinc_429& hpa, const base_arinc_429& inhg, bool& lastPress) {
      if (!isNormal(eis2) || lastPress) {
        lastPress = false;
        return;
      }
      bool any = false;
      auto pressIf = [&](bool cond, boolean_T& b) {
        if (cond) {
          press(b);
          any = true;
        }
      };
      pressIf(arincBit(eis2, 22) != gl.ls_on, e.ls_button_pushed);
      pressIf(!arincBit(eis2, 23) != gl.fd_on, e.fd_button_pushed);
      static const efis_filter_selection filters[5] = {efis_filter_selection::CSTR, efis_filter_selection::WPT,
                                                       efis_filter_selection::VORD, efis_filter_selection::NDB,
                                                       efis_filter_selection::ARPT};
      boolean_T* buttons[5] = {&e.cstr_button_pushed, &e.wpt_button_pushed, &e.vord_button_pushed, &e.ndb_button_pushed,
                          &e.arpt_button_pushed};
      for (int i = 0; i < 5; i++) {
        bool rec = arincBit(eis2, 17 + i), mine = gl.efis_filter == filters[i];
        if (rec && !mine) pressIf(true, *buttons[i]);  // select it
        else if (!rec && mine) pressIf(true, *buttons[i]);  // clear it
      }
      const bool recStd = arincBit(eis2, 28);
      pressIf(recStd && !gl.baro_std, e.baro_knob.pulled);
      pressIf(!recStd && gl.baro_std, e.baro_knob.pushed);
      if (!gl.baro_std && !recStd) {
        if (e.baro_is_inhg && isNormal(inhg)) {
          turn(e.baro_knob.turns, (inhg.Data - gl.baro_value_inhg) * 100.0);
        } else if (!e.baro_is_inhg && isNormal(hpa)) {
          turn(e.baro_knob.turns, hpa.Data - gl.baro_value_hpa);
        }
      }
      lastPress = any;
    };
    efisFollow(in.discrete_inputs.capt_efis_inputs, g.logic.capt_efis, r.eis_discrete_word_2_left,
               r.baro_setting_left_hpa, r.baro_setting_left_inhg, lastEfisPress_[0]);
    efisFollow(in.discrete_inputs.fo_efis_inputs, g.logic.fo_efis, r.eis_discrete_word_2_right,
               r.baro_setting_right_hpa, r.baro_setting_right_inhg, lastEfisPress_[1]);

    // selected values: turn towards the recorded value when it is shown (normal) in the recording and in the unit the
    // computer under test shows
    const auto& ga = g.logic.afs;
    bool n;
    float v = dataIfNormal(ga.trk_fpa_active ? r.selected_trk_deg : r.selected_hdg_deg, n);
    if (n && !ga.hdg_trk_dashes) {
      double diff = v - ga.hdg_trk_display_value;
      diff = diff > 180 ? diff - 360 : (diff < -180 ? diff + 360 : diff);
      turn(a.hdg_trk_knob.turns, diff);
    }
    v = dataIfNormal(r.selected_alt_ft, n);
    if (n) turn(a.alt_knob.turns, (v - ga.alt_display_value) / 100.0);
    v = dataIfNormal(ga.mach_active ? r.selected_mach : r.selected_spd_kts, n);
    if (n && !ga.spd_mach_dashes) {
      turn(a.spd_knob.turns, (v - ga.spd_mach_display_value) / (ga.mach_active ? 0.01 : 1.0));
    }
    v = dataIfNormal(ga.trk_fpa_active ? r.selected_fpa_deg : r.selected_vz_ft_min, n);
    if (n && !ga.vs_fpa_dashes) {
      turn(a.vs_fpa_knob.turns, (v - ga.vs_fpa_display_value) / (ga.trk_fpa_active ? 0.1 : 100.0));
    }
  }

  FdrReader reader_;
  Options opt_;
  std::unique_ptr<FdrFrame> frame_;
  std::unique_ptr<FmgcRecord> prevFmgc_;
  std::unique_ptr<base_fcu_bus> prevFcuBus_;
  std::unique_ptr<fcu_inputs> in_;
  fcu_inputs last_{};
  double lastTime_ = 0, lastDt_ = 0, monotonic_ = 0;
  long frames_ = 0, gapSteps_ = 0, gaps_ = 0;
  bool lastTrkPress_ = false, lastMetricPress_ = false, lastEfisPress_[2]{};
};
