// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ELAC (and SEC) inputs from a flight data recording, for the replays (src/elac_replay.cpp) and the differential checks
// (src/check_elac.cpp). Both ELACs every frame.
//
// The recordings up to 3209004 hold only the ELAC outputs as the wrapper (elac/Elac.cpp) gives them (bus, discretes,
// analog orders). The inputs are rebuilt the way FlyByWireInterface::updateElac() (~1350) builds them, from what the
// record holds. Order of a frame in the sim: sensors, FCU, FMGC 1, FMGC 2, ELAC 1, ELAC 2, SEC 1-3, FAC 1-2, FCDC 1-2,
// FADECs, recorder; the EFCS computers do not run in active pause (no recorded frame either). `recordedInputs()` is the
// place for a format that records the model inputs (3209005): when it returns true the rebuild is skipped.
//
//   time.dt, simulation_time    the frame's recorded dt and time; monotonic_time = the sum of the recorded dt
//   sim_data                    all false: the laws do not run in pause or slew (the frame after a slew has slew_on true
//                               in the sim; not detectable, counted as `longFrames`); computer_running = the recorded
//                               digital_output_validated (the wrapper's monitoringHealthy)
//   ADR 1-3, IR 1-3, RA 1-2     FMGC 1's recorded bus inputs of the same frame (ADR/IR/RA own = 1, opp = 2)
//   FMGC 1 / 2 B bus, AP        as src/fac_drive.h: FMGC 1's record when it ran, FMGC 2 from the next frame's FMGC 1
//                               inputs (the A bus voted orders stand for the B bus orders; checked on FMGC 1)
//   opposite ELAC bus/discretes ELAC 1: ELAC 2's record of the previous frame (zeros on frame 0); ELAC 2: ELAC 1's of this
//                               frame
//   SEC 1/2 bus, SEC discretes  the previous frame's records (SECs run after the ELACs); ground spoilers: SEC 1 and SEC
//                               2 (ELAC 1) / SEC 3 (ELAC 2)
//   FAC yaw control lost        !yaw_damper_avail_for_norm_law of the previous frame's FAC records
//   FCDC 1/2 bus                only the bits the ELAC reads, as fcdc/Fcdc.cpp packs them from the previous frame's
//                               SEC records: status word 1 bit 29 = SEC 3 failed, status word 3 bits 23/24 = SEC 1
//                               discrete word 1 bits 15/16 (when NO); the FCDCs are taken to be healthy
//   SFCC 1 / 2 bus              from FAC 1 / 2's recorded bus (src/fac_drive.h): with its own SFCC valid, the actual
//                               position word it copies and a status word with the bits of the configuration it
//                               reported; else FailureWarning (`sfccUnknown` while the ELAC runs)
//   LGCIU main gear discretes   FAC 1 / 2 discrete word 5 bits 18 / 19 (the other FAC's when that one is not healthy)
//   sidesticks, rudder pedal    -(the recorded simulation inputs), F/O stick 0: exact
//   THS position                -(recorded eta_trim_deg): exact
//   elevator / aileron pos      not recorded as doubles: the float copies on this ELAC's bus (the other ELAC's when this
//                               one is not healthy): float precision (`positionsFromBus`)
//   servo failures              FailureWarning on the ELAC's position word for that surface
//   hydraulic pressure          the recorded psi (the same variables); low pressure discretes (the Rust pressure
//                               switches, filtered and relative to ambient) estimated with their 1750/1450 psi
//                               hysteresis on the recorded psi: approximate at pressure transitions
//   takeover pushbuttons        not recorded: searched per frame (inferPriority) so that discrete status word 2 (the
//                               sidestick disabled / priority bits) matches the recording
//   THS override, ELAC pb       false / = healthy
//
// The reference ELACs (generated ElacComputer) are stepped here with the final inputs; elac-replay compares their
// outputs with the recorded ones.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#define private public
#include "ElacComputer.h"
#undef private

#include "fdr.h"

namespace efcs_drive {

constexpr uint32_t kFw = static_cast<uint32_t>(SignStatusMatrix::FailureWarning);
constexpr uint32_t kNcd = static_cast<uint32_t>(SignStatusMatrix::NoComputedData);
constexpr uint32_t kNo = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);

/** Bit n (1-based ARINC bit) of a discrete word's value, ignoring the SSM. */
inline bool bitOf(const base_arinc_429& w, int n) {
  float r = std::round(w.Data);
  uint32_t v = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  return (v >> (n - 1)) & 1U;
}

inline void setBit(base_arinc_429& w, int n, bool on) {
  uint32_t v = static_cast<uint32_t>(w.Data);
  v = on ? (v | (1U << (n - 1))) : (v & ~(1U << (n - 1)));
  w.Data = static_cast<float>(v);
}

/** The FMGC's radio height selection (FmgcComputer 1076-1100): own unless failed, else opposite, both failed 2500 ft. */
inline base_arinc_429 fmgcRadioHeight(const base_arinc_429& own, const base_arinc_429& opp) {
  if (own.SSM != kFw) return own;
  if (opp.SSM != kFw) return opp;
  return {own.SSM, 2500.0F};
}

/** The SFCC bus as a FAC saw it, from what that FAC reported (src/fac_drive.h sfccFromOutputs). */
inline base_sfcc_bus sfccFromFac(const base_fac_bus& o) {
  base_sfcc_bus s{};
  if (!bitOf(o.discrete_word_5, 21)) {
    s.slat_flap_component_status_word.SSM = kFw;
    s.slat_flap_system_status_word.SSM = kFw;
    s.slat_flap_actual_position_word.SSM = kFw;
    s.slat_actual_position_deg.SSM = kFw;
    s.flap_actual_position_deg.SSM = kFw;
    return s;
  }
  s.slat_flap_component_status_word.SSM = kNo;
  s.slat_flap_system_status_word.SSM = kNo;
  int config = 0;
  for (int k = 1; k <= 5; k++) {
    if (bitOf(o.discrete_word_5, 10 + k)) config = k;
  }
  static const int kConfigBit[6] = {17, 18, 18, 19, 20, 21};
  setBit(s.slat_flap_system_status_word, kConfigBit[config], true);
  setBit(s.slat_flap_system_status_word, 26, config == 1);
  s.slat_flap_actual_position_word = {kNo, o.discrete_word_1.Data};
  s.slat_actual_position_deg = {kNo, o.fac_slat_angle_deg.Data};
  s.flap_actual_position_deg = {kNo, o.fac_flap_angle_deg.Data};
  return s;
}

/** A hydraulic pressure switch estimate: the Rust switch's 1750 / 1450 psi hysteresis on the recorded pressure. */
struct PressureSwitch {
  bool pressurised = false;
  bool update(double psi) {
    if (psi <= 1450.0) {
      pressurised = false;
    } else if (psi >= 1750.0) {
      pressurised = true;
    }
    return pressurised;
  }
};

}  // namespace efcs_drive

class EfcsInputDrive {
 public:
  struct Options {};

  struct StepInfo {
    long frame;
    const FdrFrame* fdr;      // the recorded frame
    bool elacHealthy[2];      // the ELAC was healthy (model stepped with computer_running)
    bool fmgc1Running;        // FMGC 1 ran this frame (its record changed)
    bool fmgc2Approx;         // FMGC 2's bus and AP discrete held from an earlier frame
    bool approxInputs;        // FMGC 1 has not run yet (stand-in sensor data) or its record is frozen to the end
    bool sfccUnknown[2];      // SFCC 1 / 2 not known from a FAC while the ELACs run
    bool hydTransition;       // a hydraulic pressure between 1450 and 1750 psi or a switch change within 2 s
    bool longFrame;           // dt > 0.5 s (a slew or a loading pause before: slew_on unknown)
    int priorityMask;         // the inferred takeover pushbuttons (bit 0 captain, bit 1 F/O)
  };

  EfcsInputDrive(const std::string& path, Options opt = {}) : reader_(path), opt_(opt) {
    for (auto& b : buf_) {
      b = std::make_unique<FdrFrame>();
      std::memset(b.get(), 0, sizeof(FdrFrame));
    }
    for (auto& r : elac_) {
      r = std::make_unique<ElacComputer>();
      r->initialize();
    }
    for (auto& r : snap_) r = std::make_unique<ElacComputer>();
    elacExt_ = std::make_unique<ElacComputer::ExternalInputs_ElacComputer_T>();
    for (auto& i : elacIn_) std::memset(&i, 0, sizeof(elac_inputs));
    std::memset(&fmgc2B_, 0, sizeof(fmgc2B_));
    std::memset(&fmgc1B_, 0, sizeof(fmgc1B_));
    std::memset(&fmgc2A_, 0, sizeof(fmgc2A_));
    proxy_ = std::make_unique<FmgcRecord>();
    std::memset(proxy_.get(), 0, sizeof(FmgcRecord));
    if (reader_.ok()) {
      prepass(path);
      haveNext_ = reader_.next(*buf_[2]);
    }
  }

  bool ok() const { return reader_.ok(); }
  const std::string& error() const { return reader_.error(); }
  uint64_t version() const { return reader_.version(); }
  long frames() const { return frames_; }
  long elacHealthyFrames(int i) const { return elacHealthy_[i]; }
  long fmgc1Frozen() const { return fmgc1Frozen_; }
  long fmgc2ApproxFrames() const { return fmgc2Approx_; }
  long abMismatch() const { return abMismatch_; }
  long sfccUnknownFrames() const { return sfccUnknown_; }
  long hydTransitionFrames() const { return hydTransition_; }
  long longFrames() const { return longFrames_; }
  long priorityFrames() const { return priorityFrames_; }
  long priorityPressedFrames() const { return priorityPressed_; }
  long priorityUnresolvedFrames() const { return priorityUnresolved_; }

  /** The reference (generated) ELACs, stepped with the final inputs of each frame. */
  const ElacComputer& elac(int i) const { return *elac_[i]; }

  /**
   * Reads the next recorded frame, builds both ELACs' inputs, steps the reference ELACs and calls `step` with the
   * inputs (the callee may modify its copy for its own computers). False at the end of the file.
   */
  bool next(const std::function<void(elac_inputs (&)[2], const StepInfo&)>& step) {
    if (!haveNext_) return false;
    std::swap(buf_[0], buf_[1]);
    std::swap(buf_[1], buf_[2]);
    haveNext_ = reader_.next(*buf_[2]);
    const FdrFrame& prev = *buf_[0];
    const FdrFrame& f = *buf_[1];
    const FdrFrame* nxt = haveNext_ ? buf_[2].get() : nullptr;
    const bool first = frames_ == 0;

    StepInfo info{};
    info.frame = frames_;
    info.fdr = &f;
    info.fmgc1Running = running_[frames_];
    fmgc1Frozen_ += !info.fmgc1Running;
    const FmgcRecord& m = info.fmgc1Running || frames_ >= firstRunning_ || firstRunning_ < 0 ? f.fmgc1 : *proxy_;
    info.approxInputs = &m == proxy_.get() || approxEnd_[frames_];
    for (int i = 0; i < 2; i++) {
      info.elacHealthy[i] = f.elac[i].discrete_outputs.digital_output_validated;
      elacHealthy_[i] += info.elacHealthy[i];
    }
    info.longFrame = f.base.simulation_delta_time_s > 0.5;
    longFrames_ += info.longFrame;

    // FMGC 2 from the next frame's FMGC 1 inputs (src/fac_drive.h)
    const bool nextUsable = nxt && running_[frames_ + 1] &&
                            nxt->base.simulation_time_s - f.base.simulation_time_s -
                                    nxt->base.simulation_delta_time_s <= 0.1;
    if (nextUsable) {
      fmgc2Healthy_ = nxt->fmgc1.discrete_inputs.fmgc_opp_healthy;
      fmgc2Ap_ = nxt->fmgc1.discrete_inputs.ap_opp_engaged;
      fmgc2A_ = nxt->fmgc1.bus_inputs.fmgc_opp_bus;
      fmgc2Known_ = true;
    } else {
      info.fmgc2Approx = true;
      fmgc2Approx_++;
    }
    buildFmgcBuses(f.fmgc1, m, info.fmgc1Running);

    // hydraulic pressure switches (estimate)
    const double psi[3] = {f.specific.aircraft_hydraulic_system_green_pressure_psi,
                           f.specific.aircraft_hydraulic_system_blue_pressure_psi,
                           f.specific.aircraft_hydraulic_system_yellow_pressure_psi};
    bool transition = false;
    for (int k = 0; k < 3; k++) {
      const bool before = hyd_[k].pressurised;
      transition |= hyd_[k].update(psi[k]) != before || (psi[k] > 1450.0 && psi[k] < 1750.0);
    }
    if (transition) lastHydTransition_ = f.base.simulation_time_s;
    info.hydTransition = f.base.simulation_time_s - lastHydTransition_ <= 2.0;
    hydTransition_ += info.hydTransition;

    for (int i = 0; i < 2; i++) {
      if (!recordedInputs(i, f, elacIn_[i])) buildElacInputs(i, f, m, prev, first, info);
    }
    priorityFrames_ += (bitOf(f.elac[0].bus_outputs.discrete_status_word_2, 17) ||
                        bitOf(f.elac[0].bus_outputs.discrete_status_word_2, 18));

    info.priorityMask = inferPriority(f, info);
    elac_inputs s[2] = {elacIn_[0], elacIn_[1]};
    step(s, info);
    frames_++;
    return true;
  }

 private:
  static bool bitOf(const base_arinc_429& w, int n) { return efcs_drive::bitOf(w, n); }

  void stepElacs() {
    for (int i = 0; i < 2; i++) {
      elacExt_->in = elacIn_[i];
      elac_[i]->setExternalInputs(elacExt_.get());
      elac_[i]->step();
    }
  }

  /**
   * The sidestick takeover pushbuttons (the AP instinctive disconnect buttons) are not recorded: steps the reference
   * ELACs with the candidates (the previous frame's, none, captain, F/O, both) until discrete status word 2 (sidestick
   * disabled / priority locked bits) matches the recording on every healthy ELAC. Bit 0 captain, bit 1 F/O.
   */
  int inferPriority(const FdrFrame& f, StepInfo& info) {
    auto apply = [&](int mask) {
      for (auto& in : elacIn_) {
        in.discrete_inputs.capt_priority_takeover_pressed = mask & 1;
        in.discrete_inputs.fo_priority_takeover_pressed = mask & 2;
      }
    };
    auto matches = [&]() {
      for (int i = 0; i < 2; i++) {
        if (!info.elacHealthy[i]) continue;
        if (std::memcmp(&elac_[i]->ElacComputer_Y.out.bus_outputs.discrete_status_word_2,
                        &f.elac[i].bus_outputs.discrete_status_word_2, sizeof(base_arinc_429)) != 0) {
          return false;
        }
      }
      return true;
    };
    for (int i = 0; i < 2; i++) std::memcpy(static_cast<void*>(snap_[i].get()), elac_[i].get(), sizeof(ElacComputer));
    const int order[] = {lastPriority_, 0, 1, 2, 3};
    for (size_t k = 0; k < 5; k++) {
      const int mask = order[k];
      if (k > 0 && mask == lastPriority_) continue;
      if (k > 0) {
        for (int i = 0; i < 2; i++) std::memcpy(static_cast<void*>(elac_[i].get()), snap_[i].get(), sizeof(ElacComputer));
      }
      apply(mask);
      stepElacs();
      if (matches()) {
        priorityPressed_ += mask != 0;
        lastPriority_ = mask;
        return mask;
      }
    }
    priorityUnresolved_++;
    for (int i = 0; i < 2; i++) std::memcpy(static_cast<void*>(elac_[i].get()), snap_[i].get(), sizeof(ElacComputer));
    apply(lastPriority_);
    stepElacs();
    return lastPriority_;
  }

  /** A recording format with the ELAC model inputs (3209005) fills them here. */
  bool recordedInputs(int, const FdrFrame&, elac_inputs&) { return false; }

  /** `own`: FMGC 1's record of this frame; `m`: the record standing in for the sensor buses and FMS words. */
  void buildFmgcBuses(const FmgcRecord& own, const FmgcRecord& m, bool fmgc1Running) {
    if (fmgc1Running) {
      fmgc1B_ = own.bus_outputs.fmgc_b_bus;
      fmgc1Ap_ = own.discrete_outputs.ap_own_engaged;
      // the B bus orders and the A bus voted orders: the same values on FMGC 1 (stand-ins for FMGC 2's B bus)
      const base_fmgc_a_bus& a = own.bus_outputs.fmgc_a_bus;
      const base_fmgc_b_bus& b = own.bus_outputs.fmgc_b_bus;
      abMismatch_ += std::memcmp(&a.delta_p_ail_voted_cmd_deg, &b.delta_p_ail_cmd_deg, sizeof(base_arinc_429)) != 0 ||
                     std::memcmp(&a.delta_q_voted_cmd_deg, &b.delta_q_cmd_deg, sizeof(base_arinc_429)) != 0 ||
                     std::memcmp(&a.delta_r_voted_cmd_deg, &b.delta_r_cmd_deg, sizeof(base_arinc_429)) != 0;
    } else {
      std::memset(&fmgc1B_, 0, sizeof(fmgc1B_));
      fmgc1Ap_ = false;
    }
    if (!fmgc2Known_) {
      fmgc2Healthy_ = m.discrete_inputs.fmgc_opp_healthy;
      fmgc2Ap_ = m.discrete_inputs.ap_opp_engaged;
      fmgc2A_ = m.bus_inputs.fmgc_opp_bus;
    }
    if (!fmgc2Healthy_) {
      std::memset(&fmgc2B_, 0, sizeof(fmgc2B_));
      return;
    }
    base_fmgc_b_bus& b = fmgc2B_;
    const base_fmgc_b_bus& b1 = m.bus_outputs.fmgc_b_bus;
    b.fac_weight_lbs = b1.fac_weight_lbs;
    b.fm_weight_lbs = b1.fm_weight_lbs;
    b.fac_cg_percent = b1.fac_cg_percent;
    b.fm_cg_percent = b1.fm_cg_percent;
    b.fg_radio_height_ft = efcs_drive::fmgcRadioHeight(m.bus_inputs.ra_opp_bus.radio_height_ft,
                                                        m.bus_inputs.ra_own_bus.radio_height_ft);
    b.discrete_word_4 = fmgc2A_.discrete_word_4;
    b.ats_discrete_word = fmgc2A_.ats_discrete_word;
    b.discrete_word_3 = fmgc2A_.discrete_word_3;
    b.discrete_word_1 = fmgc2A_.discrete_word_1;
    b.discrete_word_2 = fmgc2A_.discrete_word_2;
    b.approach_spd_target_kn = b1.approach_spd_target_kn;
    b.delta_p_ail_cmd_deg = fmgc2A_.delta_p_ail_voted_cmd_deg;
    b.delta_p_splr_cmd_deg = fmgc2A_.delta_p_splr_voted_cmd_deg;
    b.delta_r_cmd_deg = fmgc2A_.delta_r_voted_cmd_deg;
    b.delta_nose_wheel_cmd_deg = fmgc2A_.delta_nosewheel_voted_cmd_deg;
    b.delta_q_cmd_deg = fmgc2A_.delta_q_voted_cmd_deg;
    b.n1_left_percent = b1.n1_left_percent;
    b.n1_right_percent = b1.n1_right_percent;
  }

  void buildElacInputs(int i, const FdrFrame& f, const FmgcRecord& m, const FdrFrame& prev, bool first,
                       StepInfo& info) {
    using namespace efcs_drive;
    const int opp = 1 - i;
    elac_inputs& in = elacIn_[i];
    in.time.dt = f.base.simulation_delta_time_s;
    in.time.simulation_time = f.base.simulation_time_s;
    if (i == 0) monotonic_ += f.base.simulation_delta_time_s;
    in.time.monotonic_time = monotonic_;
    in.sim_data = {};
    in.sim_data.computer_running = info.elacHealthy[i];
    in.sim_data.tailstrike_protection_on = false;

    // previous frame's SEC and FAC records (zero on frame 0); the opposite ELAC: ELAC 2's previous outputs for ELAC
    // 1, ELAC 1's of this frame for ELAC 2
    static const FdrFrame* zero = [] {
      auto* z = new FdrFrame;
      std::memset(static_cast<void*>(z), 0, sizeof(FdrFrame));
      return z;
    }();
    const FdrFrame& p = first ? *zero : prev;
    const ElacRecord& oppElac = i == 1 ? f.elac[0] : p.elac[1];

    base_elac_discrete_inputs& d = in.discrete_inputs;
    d = {};
    d.ground_spoilers_active_1 = p.sec[0].discrete_outputs.ground_spoiler_out;
    d.ground_spoilers_active_2 = i == 0 ? p.sec[1].discrete_outputs.ground_spoiler_out
                                        : p.sec[2].discrete_outputs.ground_spoiler_out;
    d.is_unit_1 = i == 0;
    d.is_unit_2 = i == 1;
    d.opp_axis_pitch_failure = !oppElac.discrete_outputs.pitch_axis_ok;
    d.ap_1_disengaged = !fmgc1Ap_;
    d.ap_2_disengaged = !fmgc2Ap_;
    d.opp_left_aileron_lost = !oppElac.discrete_outputs.left_aileron_ok;
    d.opp_right_aileron_lost = !oppElac.discrete_outputs.right_aileron_ok;
    d.fac_1_yaw_control_lost = !p.fac[0].discrete_outputs.yaw_damper_avail_for_norm_law;
    d.fac_2_yaw_control_lost = !p.fac[1].discrete_outputs.yaw_damper_avail_for_norm_law;
    // LGCIU main gear: the FAC of the same side reports its LGCIU (word 5 bits 18/19); the other FAC when not healthy
    for (int k = 0; k < 2; k++) {
      const base_fac_bus& fb = f.fac[k].bus_outputs.discrete_word_2.SSM != kFw ? f.fac[k].bus_outputs
                                                                               : f.fac[1 - k].bus_outputs;
      const bool left = bitOf(fb.discrete_word_5, 18), right = bitOf(fb.discrete_word_5, 19);
      (k == 0 ? d.lgciu_1_left_main_gear_pressed : d.lgciu_2_left_main_gear_pressed) = left;
      (k == 0 ? d.lgciu_1_right_main_gear_pressed : d.lgciu_2_right_main_gear_pressed) = right;
      (k == 0 ? d.lgciu_1_nose_gear_pressed : d.lgciu_2_nose_gear_pressed) = left || right;
    }
    // servo failures: FailureWarning on this ELAC's position word (the other ELAC's bus while this one is not healthy)
    const base_elac_out_bus& posBus = info.elacHealthy[i] ? f.elac[i].bus_outputs : f.elac[opp].bus_outputs;
    if (info.elacHealthy[i]) {
      d.l_ail_servo_failed = posBus.left_aileron_position_deg.SSM == kFw;
      d.r_ail_servo_failed = posBus.right_aileron_position_deg.SSM == kFw;
      d.l_elev_servo_failed = posBus.left_elevator_position_deg.SSM == kFw;
      d.r_elev_servo_failed = posBus.right_elevator_position_deg.SSM == kFw;
    }
    d.green_low_pressure = !hyd_[0].pressurised;
    d.blue_low_pressure = !hyd_[1].pressurised;
    d.yellow_low_pressure = !hyd_[2].pressurised;
    d.elac_engaged_from_switch = info.elacHealthy[i];

    base_elac_analog_inputs& a = in.analog_inputs;
    a = {};
    a.capt_pitch_stick_pos = -f.base.simulation_input_sidestick_pitch_pos;
    a.capt_roll_stick_pos = -f.base.simulation_input_sidestick_roll_pos;
    a.rudder_pedal_pos = -f.base.simulation_input_rudder_pos;
    a.ths_pos_deg = -f.base.aircraft_eta_trim_deg;
    if (info.elacHealthy[i] || info.elacHealthy[opp]) {
      a.left_elevator_pos_deg = posBus.left_elevator_position_deg.Data;
      a.right_elevator_pos_deg = posBus.right_elevator_position_deg.Data;
      a.left_aileron_pos_deg = posBus.left_aileron_position_deg.Data;
      a.right_aileron_pos_deg = posBus.right_aileron_position_deg.Data;
    }
    a.green_hyd_pressure_psi = f.specific.aircraft_hydraulic_system_green_pressure_psi;
    a.blue_hyd_pressure_psi = f.specific.aircraft_hydraulic_system_blue_pressure_psi;
    a.yellow_hyd_pressure_psi = f.specific.aircraft_hydraulic_system_yellow_pressure_psi;

    base_elac_bus_inputs& b = in.bus_inputs;
    const FmgcRecord& adr = m;
    b.adr_1_bus = adr.bus_inputs.adr_own_bus;
    b.adr_2_bus = adr.bus_inputs.adr_opp_bus;
    b.adr_3_bus = adr.bus_inputs.adr_3_bus;
    b.ir_1_bus = adr.bus_inputs.ir_own_bus;
    b.ir_2_bus = adr.bus_inputs.ir_opp_bus;
    b.ir_3_bus = adr.bus_inputs.ir_3_bus;
    b.ra_1_bus = adr.bus_inputs.ra_own_bus;
    b.ra_2_bus = adr.bus_inputs.ra_opp_bus;
    b.fmgc_1_bus = fmgc1B_;
    b.fmgc_2_bus = fmgc2B_;
    for (int k = 0; k < 2; k++) {
      const bool facHealthy = f.fac[k].bus_outputs.discrete_word_2.SSM != kFw;
      base_sfcc_bus& s = k == 0 ? b.sfcc_1_bus : b.sfcc_2_bus;
      if (facHealthy) {
        s = sfccFromFac(f.fac[k].bus_outputs);
      } else {
        s = {};
      }
      info.sfccUnknown[k] = !facHealthy && (info.elacHealthy[0] || info.elacHealthy[1]);
    }
    if (i == 1) sfccUnknown_ += info.sfccUnknown[0] || info.sfccUnknown[1];
    // FCDC: the bits the ELAC reads (Fcdc.cpp 551, 601-602), from the previous frame's SECs
    for (int k = 0; k < 2; k++) {
      base_fcdc_bus& c = k == 0 ? b.fcdc_1_bus : b.fcdc_2_bus;
      c = {};
      c.efcs_status_word_1.SSM = kNo;
      c.efcs_status_word_3.SSM = kNo;
      setBit(c.efcs_status_word_1, 29, p.sec[2].discrete_outputs.sec_failed);
      const base_arinc_429& s1 = p.sec[0].bus_outputs.discrete_status_word_1;
      setBit(c.efcs_status_word_3, 23, s1.SSM == kNo && bitOf(s1, 15));
      setBit(c.efcs_status_word_3, 24, s1.SSM == kNo && bitOf(s1, 16));
    }
    b.sec_1_bus = p.sec[0].bus_outputs;
    b.sec_2_bus = p.sec[1].bus_outputs;
    b.elac_opp_bus = oppElac.bus_outputs;
  }

  /**
   * Reads the whole file once for FMGC 1's running state per frame (src/fac_drive.h prepass): a frame where its record
   * did not change is a frame it did not run when it belongs to a frozen stretch from the start of the file or lasting
   * at least 25 s; the first running frame's record stands in for the start-up frames.
   */
  void prepass(const std::string& path) {
    FdrReader r(path);
    auto cur = std::make_unique<FdrFrame>(), prv = std::make_unique<FdrFrame>();
    std::memset(cur.get(), 0, sizeof(FdrFrame));
    std::memset(prv.get(), 0, sizeof(FdrFrame));
    std::vector<double> time;
    long n = 0;
    while (r.next(*cur)) {
      running_.push_back(n > 0 && std::memcmp(&cur->fmgc1, &prv->fmgc1, sizeof(FmgcRecord)) != 0);
      approxEnd_.push_back(false);
      time.push_back(cur->base.simulation_time_s);
      if (firstRunning_ < 0 && running_.back()) {
        firstRunning_ = n;
        std::memcpy(static_cast<void*>(proxy_.get()), &cur->fmgc1, sizeof(FmgcRecord));
      }
      std::swap(cur, prv);
      n++;
    }
    if (n > 1) running_[0] = running_[1];
    for (long a = 0; a < n;) {
      if (running_[a]) {
        a++;
        continue;
      }
      long e = a;
      while (e + 1 < n && !running_[e + 1]) e++;
      const bool atStart = a <= 1;
      if (!atStart && time[e] - time[a] < 25.0) {
        for (long k = a; k <= e; k++) running_[k] = true;
      }
      if (!atStart && e == n - 1) {
        for (long k = a; k <= e; k++) approxEnd_[k] = true;
      }
      a = e + 1;
    }
    running_.push_back(false);
  }

  FdrReader reader_;
  Options opt_;
  std::vector<uint8_t> running_, approxEnd_;
  long firstRunning_ = -1;
  std::unique_ptr<FmgcRecord> proxy_;
  std::unique_ptr<FdrFrame> buf_[3];
  bool haveNext_ = false;
  std::unique_ptr<ElacComputer> elac_[2], snap_[2];
  std::unique_ptr<ElacComputer::ExternalInputs_ElacComputer_T> elacExt_;
  elac_inputs elacIn_[2];
  double monotonic_ = 0;
  base_fmgc_b_bus fmgc1B_, fmgc2B_;
  base_fmgc_a_bus fmgc2A_;
  bool fmgc1Ap_ = false, fmgc2Ap_ = false, fmgc2Healthy_ = false, fmgc2Known_ = false;
  efcs_drive::PressureSwitch hyd_[3];
  double lastHydTransition_ = -1e9;
  long frames_ = 0, elacHealthy_[2] = {0, 0}, fmgc1Frozen_ = 0, fmgc2Approx_ = 0, abMismatch_ = 0;
  long sfccUnknown_ = 0, hydTransition_ = 0, longFrames_ = 0, priorityFrames_ = 0;
  int lastPriority_ = 0;
  long priorityPressed_ = 0, priorityUnresolved_ = 0;
};
