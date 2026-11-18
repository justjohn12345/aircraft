// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FADEC inputs from a flight data recording, for the FADEC replay (src/fadec_replay.cpp) and the differential check
// (src/check_fadec.cpp). Both FADECs (engine 1 and 2) every step.
//
// Two sources:
//  - Recorded (personal format 3209004): each FADEC's model inputs exactly as given to its step in the sim
//    (FadecModelRecord::in), with the recorded outputs to compare against.
//  - Synthetic (any recording): built the way FlyByWireInterface::updateFadec() builds them, from what the record holds.
//    The FADECs run after the FMGCs and the FCU, so they see this frame's FCU bus:
//      time                 recorded dt and simulation time
//      data                 speeds, altitudes, alpha, TAT and flap handle from the base record; on_ground = both main
//                           gear compressions > 0.01 (the LGCIU threshold); engine operative = !eng_own/opp_stop
//                           (FMGC 1 discrete inputs); N1 actual = FADEC 1's recorded bus word (this frame) and FADEC 2's
//                           bus as FMGC 1 received it (previous frame); N2 = 0 (as the wrapper)
//      commanded N1         not recorded: a static plant, commanded N1 = IDLE + |throttle pos| / 100 x (104 - IDLE),
//                           fed back from the computer under test's previous throttle output (`feedback`)
//      ATHR_disconnect      FMGC 1's athr_instinctive_disc (the same wrapper expression)
//      ATHR_reset_disable   false (injectors drive it)
//      TLA                  the recorded thrust lever angles
//      thrust limits        not recorded: learned from FADEC 1's recorded outputs (the limit of the recorded limit type,
//                           exact; IDLE = N1_TLA when TLA is exactly 0; TOGA also from the N1 maximum word); REV =
//                           TOGA x 0.813 (the wrapper's default); typical values until learned
//      fcu_input            the FCU bus FMGC 1 received this frame
//
// Unrecorded frames (active pause: the recorder stops, the FADECs keep running) show as a simulation-time jump; they are
// filled with steps at the last dt that hold the inputs and clear the instinctive disconnect and reset inputs.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "FadecComputer_types.h"
#include "fdr.h"

namespace fadec_drive {

/** The wrapper's default THRUST_LIMIT_REVERSE_PERCENTAGE_TOGA (A32NX ini). */
constexpr double kReversePercentageToga = 0.813;
/** Plant: commanded N1 at full throttle (%). */
constexpr double kPlantMaxN1 = 104.0;

inline bool onGround(const AircraftSpecificData& s) {
  return s.aircraft_gear_main_left_compression_percent > 0.01 && s.aircraft_gear_main_right_compression_percent > 0.01;
}

/** The commanded N1 the sim engine would give for a throttle position (the drive's plant). */
inline double plantCommandedN1(double throttlePos, double idle) {
  return idle + std::fabs(throttlePos) / 100.0 * (kPlantMaxN1 - idle);
}

}  // namespace fadec_drive

class FadecInputDrive {
 public:
  struct Options {
    bool synthetic = false;  // build the inputs from the record even when the file has the FADEC model record
    bool fillGaps = true;
  };

  struct StepInfo {
    long frame;           // recorded frame index
    bool gapFill;         // an unrecorded step (active pause) before that frame
    const FdrFrame* fdr;  // the recorded frame
    bool recordedInputs;  // the inputs are the recorded FADEC inputs (3209004)
  };

  /** The learned thrust limits (synthetic drive). */
  struct Limits {
    double idle = 19.5, clb = 89.0, mct = 91.0, flex = 86.0, toga = 93.0;
    bool idleKnown = false;
  };

  FadecInputDrive(const std::string& path, Options opt) : reader_(path), opt_(opt) {
    frame_ = std::make_unique<FdrFrame>();
    for (auto& i : in_) std::memset(&i, 0, sizeof(athr_in));
  }

  bool ok() const { return reader_.ok(); }
  const std::string& error() const { return reader_.error(); }
  bool fileHasFadec() const { return reader_.hasFadecModel(); }
  uint64_t version() const { return reader_.version(); }
  bool recordedInputs() const { return reader_.hasFadecModel() && !opt_.synthetic; }
  long frames() const { return frames_; }
  long gapSteps() const { return gapSteps_; }
  long gaps() const { return gaps_; }
  const Limits& limits() const { return limits_; }

  /** The computer under test's last outputs per engine, for the commanded N1 plant (may be null). */
  const athr_out* feedback[2] = {nullptr, nullptr};

  /**
   * Reads the next recorded frame and calls `step` for each step it implies (gap fills, then the frame), with the inputs
   * of both FADECs (the callee may modify them for this step only). False at the end of the file.
   */
  bool next(const std::function<void(athr_in (&)[2], const StepInfo&)>& step) {
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
        athr_in held[2] = {in_[0], in_[1]};
        for (auto& h : held) {
          h.input.ATHR_disconnect = false;
          h.input.ATHR_reset_disable = false;
        }
        for (long i = 0; i < steps; i++) {
          athr_in s[2];
          for (int e = 0; e < 2; e++) {
            held[e].time.dt = fillDt;
            held[e].time.simulation_time += fillDt;
            if (!recordedInputs()) held[e].data.commanded_engine_N1_percent = plant(e);
            s[e] = held[e];
          }
          step(s, {frames_, true, &f, recordedInputs()});
        }
        gapSteps_ += steps;
        gaps_++;
      }
    }

    if (recordedInputs()) {
      in_[0] = f.fadecModel[0].in;
      in_[1] = f.fadecModel[1].in;
    } else {
      buildSynthetic(f);
    }
    lastTime_ = base.simulation_time_s;
    lastDt_ = base.simulation_delta_time_s;
    athr_in s[2] = {in_[0], in_[1]};
    step(s, {frames_, false, &f, recordedInputs()});
    frames_++;
    return true;
  }

 private:
  double plant(int e) const {
    const double pos = feedback[e] ? feedback[e]->output.sim_throttle_lever_pos : 0.0;
    return fadec_drive::plantCommandedN1(pos, limits_.idle);
  }

  void learnLimits(const FdrFrame& f) {
    using T = athr_thrust_limit_type;
    const athr_output& o = f.fadec[0].outputs;
    const base_ecu_bus& b = f.fadec[0].bus_outputs;
    if (b.n1_maximum_percent.Data > 0.0F) limits_.toga = b.n1_maximum_percent.Data;
    switch (o.thrust_limit_type) {
      case T::CLB: limits_.clb = o.thrust_limit_percent; break;
      case T::MCT: limits_.mct = o.thrust_limit_percent; break;
      case T::FLEX: limits_.flex = o.thrust_limit_percent; break;
      case T::TOGA: limits_.toga = o.thrust_limit_percent; break;
      default: break;
    }
    if (f.specific.simulation_input_throttle_lever_1_angle == 0.0 && o.N1_TLA_percent > 0.0) {
      limits_.idle = o.N1_TLA_percent;
      limits_.idleKnown = true;
    }
  }

  void buildSynthetic(const FdrFrame& f) {
    learnLimits(f);
    const BaseData& base = f.base;
    const FmgcRecord& m = f.fmgc1;
    for (int e = 0; e < 2; e++) {
      athr_in& in = in_[e];
      in.time.dt = base.simulation_delta_time_s;
      in.time.simulation_time = base.simulation_time_s;
      athr_data& d = in.data;
      d.V_ias_kn = base.aircraft_V_indicated_kn;
      d.V_tas_kn = base.aircraft_V_true_kn;
      d.V_mach = base.aircraft_Ma_mach;
      d.V_gnd_kn = base.aircraft_V_ground_kn;
      d.alpha_deg = base.aircraft_alpha_deg;
      d.H_ft = base.aircraft_H_pressure_ft;
      d.H_ind_ft = base.aircraft_H_indicated_ft;
      d.H_radio_ft = base.aircraft_H_radio_ft;
      d.H_dot_fpm = 0.0;
      d.on_ground = fadec_drive::onGround(f.specific);
      d.flap_handle_index = base.simulation_input_flaps_handle_index;
      d.is_engine_operative = e == 0 ? !m.discrete_inputs.eng_own_stop : !m.discrete_inputs.eng_opp_stop;
      d.engine_N1_percent = e == 0 ? f.fadec[0].bus_outputs.selected_n1_actual_percent.Data
                                   : m.bus_inputs.fadec_opp_bus.selected_n1_actual_percent.Data;
      d.engine_N2_percent = 0.0;
      d.commanded_engine_N1_percent = plant(e);
      d.TAT_degC = base.aircraft_total_air_temperature_deg_celsius;
      d.OAT_degC = base.aircraft_total_air_temperature_deg_celsius;
      d.ISA_degC = 0.0;
      d.ambient_density_kg_per_m3 = 0.0;

      athr_input& i = in.input;
      i.ATHR_disconnect = m.discrete_inputs.athr_instinctive_disc;
      i.TLA_deg = e == 0 ? f.specific.simulation_input_throttle_lever_1_angle
                         : f.specific.simulation_input_throttle_lever_2_angle;
      i.thrust_limit_REV_percent = limits_.toga * fadec_drive::kReversePercentageToga;
      i.thrust_limit_IDLE_percent = limits_.idle;
      i.thrust_limit_CLB_percent = limits_.clb;
      i.thrust_limit_MCT_percent = limits_.mct;
      i.thrust_limit_FLEX_percent = limits_.flex;
      i.thrust_limit_TOGA_percent = limits_.toga;
      i.is_anti_ice_active = f.specific.aircraft_is_wing_anti_ice_active != 0.0;
      i.is_air_conditioning_active = true;
      i.ATHR_reset_disable = false;
      i.tracking_mode_on_override = false;

      in.fcu_input = m.bus_inputs.fcu_bus;
    }
  }

  FdrReader reader_;
  Options opt_;
  std::unique_ptr<FdrFrame> frame_;
  athr_in in_[2];
  Limits limits_;
  double lastTime_ = 0, lastDt_ = 0;
  long frames_ = 0, gapSteps_ = 0, gaps_ = 0;
};
