// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FAC inputs from a flight data recording, for the FAC replay (src/fac_replay.cpp) and the differential check
// (src/check_fac.cpp). Both FACs every frame.
//
// No recording format holds the FAC model inputs: every one (3200006 and the personal 3209001-3209004) has only the FAC
// outputs as the wrapper (fac/Fac.cpp) gives them (bus, discretes, analog orders). The inputs are rebuilt the way
// FlyByWireInterface::updateFac() builds them, from what the record holds. Order of a frame in the sim: sensors (RA,
// LGCIU, SFCC, ILS, ADIRS), FCU, FMGC 1, FMGC 2, ELACs, SECs, FAC 1, FAC 2, FCDCs, FADECs, recorder; the FACs do not run
// in active pause (no recorded frame either), the FMGCs do.
//
//   time.dt, simulation_time   the frame's recorded dt and simulation time
//   sim_data                   false (the laws are not run in slew or pause); computer_running = the FAC was healthy:
//                              the wrapper sends FailureWarning (SSM 0) on every bus word when it is not, and the model
//                              writes NormalOperation to discrete word 2 whenever it runs
//   ADR / IR own, opp, 3       FMGC 1's recorded bus inputs of the same frame (the same adrBusOutputs/irBusOutputs;
//                              FAC 1 own = FMGC 1 own, FAC 2 own = FMGC 1 opp)
//   ELAC 1 / 2 bus, healthy    the recorded ELAC buses and digital_output_validated discretes (ELACs run before)
//   FAC opp bus, discretes     FAC 1: FAC 2's recorded outputs of the previous frame (zero on frame 0); FAC 2: FAC 1's of
//                              this frame
//   FMGC 1 B bus, AP engaged   FMGC 1's recorded model outputs when it ran this frame (its record changes), else what the
//                              wrapper sends while it does not run: a zero bus and AP not engaged
//   FMGC 2 B bus, AP engaged   not recorded. FMGC 1 receives FMGC 2's A bus, AP discrete and health one frame later, so
//                              they are taken from the next frame's FMGC 1 inputs: discrete words 1-4 and the ATS word
//                              are the A bus words (the same packing); the radio height is FMGC 2's own selection from
//                              RA 2 / RA 1 (the RA words FMGC 1 received); weight, CG, FAC weight/CG and N1 words are
//                              FMGC 1's (the same sources). Without a usable next frame (end of file, active pause gap,
//                              FMGC 1 not running) the last known values are held (`fmgc2Approx` frames)
//   SFCC own bus               rebuilt from this FAC's recorded outputs: when the FAC reports its own SFCC valid, the
//                              flap/slat angles and the actual position word are the bus values it copied, the system
//                              status word carries the bits of the configuration it reported (discrete word 5 bits
//                              11-15); when it reports it invalid, the status word is FailureWarning
//   LGCIU own bus, nose gear   rebuilt the same way: own LGCIU valid -> the gear bits it reported (discrete word 5 bits
//                              18-20), else discrete word 1 FailureWarning; the nose gear discrete equals the LGCIU nose
//                              bit (both come from the same LGCIU)
//   slats_extended             as the wrapper: !(actual position word bit 12 when NormalOperation)
//   FAC pushbutton, hydraulics from this FAC's recorded discretes: FAC 1 has priority, so its rudder trim engaged = its
//                              pushbutton and yaw_damper_avail_for_norm_law = pushbutton and pressure; FAC 2 likewise when
//                              FAC 1 does not hold the function; when no output shows them they cannot matter (true,
//                              and the pressure from the recorded green/yellow psi)
//   rudder trim, travel limit  the positions the FAC copies to its bus (rudder_trim_pos_deg, rudder_travel_limit_
//   positions                  command_deg): exact to float precision (the sim value is a double)
//   rudder trim switches,      not recorded. Inferred per frame by stepping the reference FACs (the generated model,
//   reset button               `reference`) with the candidates (none, previous frame's, left, right, reset, the rest)
//                              until the rudder trim order and bus word match the recording (`trimSearched`,
//                              `trimUnresolved` counts); both FACs share the inputs
//   engine stopped, actuator   constants as the wrapper (true); ADR/IR 3 switches false, yaw damper position 0
//   healthy, switches
//
// The reference FACs (generated FacComputer) are stepped here with the final inputs; fac-replay compares their outputs
// with the recorded ones.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#define private public
#include "FacComputer.h"
#undef private

#include "fdr.h"

namespace fac_drive {

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

/** The wrapper's bus when the FAC is not healthy: every word FailureWarning with 0 data. */
inline bool isUnhealthyBus(const base_fac_bus& b) { return b.discrete_word_2.SSM == kFw; }

/** The FMGC's radio height selection (FmgcComputer 1076-1100): own unless failed, else opposite, both failed 2500 ft. */
inline base_arinc_429 fmgcRadioHeight(const base_arinc_429& own, const base_arinc_429& opp) {
  if (own.SSM != kFw) return own;
  if (opp.SSM != kFw) return opp;
  return {own.SSM, 2500.0F};
}

/** Rudder trim input candidates: bit 0 left, bit 1 right, bit 2 reset button. */
inline void applyTrimMask(base_fac_discrete_inputs& d, int mask) {
  d.rudder_trim_switch_left = mask & 1;
  d.rudder_trim_switch_right = mask & 2;
  d.rudder_trim_reset_button = mask & 4;
}

}  // namespace fac_drive

class FacInputDrive {
 public:
  struct Options {
    bool inferTrim = true;
  };

  struct StepInfo {
    long frame;
    const FdrFrame* fdr;   // the recorded frame
    bool healthy[2];       // the FAC was healthy (model stepped with computer_running)
    bool fmgc1Running;     // FMGC 1 ran this frame (its record changed)
    bool fmgc2Approx;      // FMGC 2's bus and AP discrete held from an earlier frame
    bool approxInputs;     // FMGC 1 has not run yet (ADIRS, RA and FMS words from its first running frame), or its record
                           // stays frozen to the end of the file (stale sensor buses)
    int trimMask;          // the inferred rudder trim inputs (bit 0 left, 1 right, 2 reset)
    bool trimUnresolved;   // no candidate reproduced the recorded rudder trim outputs
  };

  FacInputDrive(const std::string& path, Options opt) : reader_(path), opt_(opt) {
    for (auto& b : buf_) {
      b = std::make_unique<FdrFrame>();
      std::memset(b.get(), 0, sizeof(FdrFrame));  // padding stays zero, so whole records compare with memcmp
    }
    for (auto& r : ref_) {
      r = std::make_unique<FacComputer>();
      r->initialize();
    }
    for (auto& s : snap_) s = std::make_unique<FacComputer>();
    ext_ = std::make_unique<FacComputer::ExternalInputs_FacComputer_T>();
    for (auto& i : in_) std::memset(&i, 0, sizeof(fac_inputs));
    std::memset(&fmgc2B_, 0, sizeof(fmgc2B_));
    std::memset(&proxy_, 0, sizeof(proxy_));
    if (reader_.ok()) {
      prepass(path);
      haveNext_ = reader_.next(*buf_[2]);
    }
  }

  bool ok() const { return reader_.ok(); }
  const std::string& error() const { return reader_.error(); }
  uint64_t version() const { return reader_.version(); }
  long frames() const { return frames_; }
  long healthyFrames(int i) const { return healthyFrames_[i]; }
  long fmgc1Frozen() const { return fmgc1Frozen_; }
  /** First frame FMGC 1 runs (its record changes after the start-up freeze), -1 if never. */
  long fmgc1FirstRunning() const { return firstRunning_; }
  long fmgc2ApproxFrames() const { return fmgc2Approx_; }
  long gaps() const { return gaps_; }
  long trimFrames() const { return trimFrames_; }
  long trimSearched() const { return trimSearched_; }
  long trimUnresolved() const { return trimUnresolved_; }
  long abMismatch() const { return abMismatch_; }

  /** The reference (generated) FACs, stepped with the final inputs of each frame. */
  const FacComputer& reference(int i) const { return *ref_[i]; }

  /**
   * Reads the next recorded frame, builds both FACs' inputs, steps the reference FACs and calls `step` with the inputs
   * (the callee may modify them for its own computers). False at the end of the file.
   */
  bool next(const std::function<void(fac_inputs (&)[2], const StepInfo&)>& step) {
    if (!haveNext_) return false;
    // rotate: prev <- cur <- next, read a new next
    std::swap(buf_[0], buf_[1]);
    std::swap(buf_[1], buf_[2]);
    haveNext_ = reader_.next(*buf_[2]);
    const FdrFrame& prev = *buf_[0];
    const FdrFrame& f = *buf_[1];
    const FdrFrame* nxt = haveNext_ ? buf_[2].get() : nullptr;

    StepInfo info{};
    info.frame = frames_;
    info.fdr = &f;
    const bool first = frames_ == 0;
    if (!first && f.base.simulation_time_s - prev.base.simulation_time_s - f.base.simulation_delta_time_s > 0.1) gaps_++;
    info.fmgc1Running = running_[frames_];
    fmgc1Frozen_ += !info.fmgc1Running;
    // the FMGC 1 record whose sensor buses (ADIRS, RA) and FMS words stand in for this frame's: its own while it runs;
    // before it first runs, the first running frame's (the aircraft is parked during the start-up self-test)
    const FmgcRecord& m = info.fmgc1Running || frames_ >= firstRunning_ || firstRunning_ < 0 ? f.fmgc1 : proxy_;
    info.approxInputs = &m == &proxy_ || approxEnd_[frames_];
    for (int i = 0; i < 2; i++) {
      info.healthy[i] = !fac_drive::isUnhealthyBus(f.fac[i].bus_outputs);
      healthyFrames_[i] += info.healthy[i];
    }

    // FMGC 2 as the FACs see it this frame: from the next frame's FMGC 1 inputs when usable
    const bool nextUsable = nxt && running_[frames_ + 1] &&
                            nxt->base.simulation_time_s - f.base.simulation_time_s -
                                    nxt->base.simulation_delta_time_s <= 0.1;
    if (nextUsable) {
      const FmgcRecord& n = nxt->fmgc1;
      fmgc2Healthy_ = n.discrete_inputs.fmgc_opp_healthy;
      fmgc2Ap_ = n.discrete_inputs.ap_opp_engaged;
      fmgc2A_ = n.bus_inputs.fmgc_opp_bus;
      fmgc2Known_ = true;
    } else {
      info.fmgc2Approx = true;
      fmgc2Approx_++;
    }
    if (!nextUsable && info.approxInputs && !fmgc2Known_) {
      fmgc2Healthy_ = proxy_.discrete_inputs.fmgc_opp_healthy;
      fmgc2Ap_ = proxy_.discrete_inputs.ap_opp_engaged;
      fmgc2A_ = proxy_.bus_inputs.fmgc_opp_bus;
    }
    buildFmgcBuses(f.fmgc1, m, info.fmgc1Running);

    for (int i = 0; i < 2; i++) {
      const base_fac_bus& o = f.fac[i].bus_outputs;
      approxAdrOn_[i] = info.healthy[i] && o.v_max_kn.Data != 0.0F;
      approxIrOn_[i] = info.healthy[i] && (o.gamma_a_deg.Data != 0.0F || o.gamma_t_deg.Data != 0.0F);
    }
    for (int i = 0; i < 2; i++) buildInputs(i, f, m, prev, first, info);

    // reference FACs, with the rudder trim inputs inferred from the recorded outputs
    info.trimMask = inferTrim(f, info);
    info.trimUnresolved = trimUnresolvedThisFrame_;
    for (auto& i : in_) fac_drive::applyTrimMask(i.discrete_inputs, info.trimMask);

    fac_inputs s[2] = {in_[0], in_[1]};
    step(s, info);
    frames_++;
    return true;
  }

 private:
  void stepRef(int i) {
    ext_->in = in_[i];
    ref_[i]->setExternalInputs(ext_.get());
    ref_[i]->step();
  }

  /** Steps the reference FACs with candidate trim inputs until the recorded rudder trim outputs are reproduced. */
  int inferTrim(const FdrFrame& f, const StepInfo& info) {
    trimUnresolvedThisFrame_ = false;
    auto matches = [&]() {
      for (int i = 0; i < 2; i++) {
        if (!info.healthy[i]) continue;
        const fac_outputs& o = ref_[i]->FacComputer_Y.out;
        const FacRecord& r = f.fac[i];
        if (std::memcmp(&o.analog_outputs.rudder_trim_order_deg, &r.analog_outputs.rudder_trim_order_deg,
                        sizeof(double)) != 0 ||
            std::memcmp(&o.bus_outputs.delta_r_rudder_trim_deg, &r.bus_outputs.delta_r_rudder_trim_deg,
                        sizeof(base_arinc_429)) != 0) {
          return false;
        }
      }
      return true;
    };
    auto run = [&](int mask) {
      for (int i = 0; i < 2; i++) {
        fac_drive::applyTrimMask(in_[i].discrete_inputs, mask);
        stepRef(i);
      }
    };
    if (!opt_.inferTrim) {
      run(0);
      return 0;
    }
    for (int i = 0; i < 2; i++) std::memcpy(static_cast<void*>(snap_[i].get()), ref_[i].get(), sizeof(FacComputer));
    const int order[] = {0, lastTrim_, 1, 2, 4, 3, 5, 6, 7};
    bool searched = false;
    for (size_t k = 0; k < sizeof(order) / sizeof(order[0]); k++) {
      const int mask = order[k];
      if ((k == 1 && mask == 0) || (k > 1 && mask == lastTrim_)) continue;
      if (searched) {
        for (int i = 0; i < 2; i++) std::memcpy(static_cast<void*>(ref_[i].get()), snap_[i].get(), sizeof(FacComputer));
      }
      run(mask);
      if (matches()) {
        trimSearched_ += searched;
        trimFrames_ += mask != 0;
        lastTrim_ = mask;
        return mask;
      }
      searched = true;
    }
    // none reproduces the recording: keep "no input"
    trimSearched_++;
    trimUnresolved_++;
    trimUnresolvedThisFrame_ = true;
    for (int i = 0; i < 2; i++) std::memcpy(static_cast<void*>(ref_[i].get()), snap_[i].get(), sizeof(FacComputer));
    run(0);
    lastTrim_ = 0;
    return 0;
  }

  /** `own`: FMGC 1's record of this frame; `m`: the record standing in for the sensor buses and FMS words. */
  void buildFmgcBuses(const FmgcRecord& own, const FmgcRecord& m, bool fmgc1Running) {
    // FMGC 1 as the wrapper sends it: the model's B bus while it runs (monitoring healthy), else zeros
    if (fmgc1Running) {
      fmgc1B_ = own.bus_outputs.fmgc_b_bus;
      fmgc1Ap_ = own.discrete_outputs.ap_own_engaged;
      // the A and B bus discrete words are packed by the same functions
      abMismatch_ += std::memcmp(&own.bus_outputs.fmgc_a_bus.discrete_word_1,
                                 &own.bus_outputs.fmgc_b_bus.discrete_word_1, sizeof(base_arinc_429)) != 0 ||
                     std::memcmp(&own.bus_outputs.fmgc_a_bus.ats_discrete_word,
                                 &own.bus_outputs.fmgc_b_bus.ats_discrete_word, sizeof(base_arinc_429)) != 0;
    } else {
      std::memset(&fmgc1B_, 0, sizeof(fmgc1B_));
      fmgc1Ap_ = false;
    }
    // FMGC 2 (see the file comment)
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
    b.fg_radio_height_ft =
        fac_drive::fmgcRadioHeight(m.bus_inputs.ra_opp_bus.radio_height_ft, m.bus_inputs.ra_own_bus.radio_height_ft);
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

  /** The own SFCC bus as this FAC saw it, from what it reported. */
  static base_sfcc_bus sfccFromOutputs(const base_fac_bus& o) {
    using namespace fac_drive;
    base_sfcc_bus s{};
    const bool valid = bitOf(o.discrete_word_5, 21);
    if (!valid) {
      s.slat_flap_system_status_word.SSM = kFw;
      s.slat_flap_actual_position_word.SSM = kFw;
      s.slat_actual_position_deg.SSM = kFw;
      s.flap_actual_position_deg.SSM = kFw;
      s.slat_flap_component_status_word.SSM = kFw;
      return s;
    }
    s.slat_flap_component_status_word.SSM = kNo;
    s.slat_flap_system_status_word.SSM = kNo;
    // configuration index -> status word bits read by the FAC: 0 bit 17; 1 bits 18 + 26; 2 bit 18; 3 bit 19; 4 bit 20;
    // 5 bit 21
    base_arinc_429& w = s.slat_flap_system_status_word;
    int config = 0;
    for (int k = 1; k <= 5; k++) {
      if (bitOf(o.discrete_word_5, 10 + k)) config = k;
    }
    static const int kConfigBit[6] = {17, 18, 18, 19, 20, 21};
    setBit(w, kConfigBit[config], true);
    setBit(w, 26, config == 1);
    s.slat_flap_actual_position_word = {kNo, o.discrete_word_1.Data};
    s.slat_actual_position_deg = {kNo, o.fac_slat_angle_deg.Data};
    s.flap_actual_position_deg = {kNo, o.fac_flap_angle_deg.Data};
    return s;
  }

  /** The own LGCIU bus as this FAC saw it (and the nose gear discrete, equal to its nose bit). */
  static base_lgciu_bus lgciuFromOutputs(const base_fac_bus& o, bool& noseGear) {
    using namespace fac_drive;
    base_lgciu_bus l{};
    const bool valid = bitOf(o.discrete_word_5, 16);
    l.discrete_word_1.SSM = valid ? kNo : kFw;
    l.discrete_word_2.SSM = valid ? kNo : kFw;
    l.discrete_word_3.SSM = valid ? kNo : kFw;
    l.discrete_word_4.SSM = valid ? kNo : kFw;
    noseGear = false;
    if (valid) {
      const bool left = bitOf(o.discrete_word_5, 18), right = bitOf(o.discrete_word_5, 19);
      noseGear = left || right;
      setBit(l.discrete_word_2, 12, noseGear);
      setBit(l.discrete_word_2, 13, left);
      setBit(l.discrete_word_2, 14, right);
      setBit(l.discrete_word_3, 11, bitOf(o.discrete_word_5, 20));
    }
    return l;
  }

  void buildInputs(int i, const FdrFrame& f, const FmgcRecord& m, const FdrFrame& prev, bool first,
                   const StepInfo& info) {
    using namespace fac_drive;
    const int opp = 1 - i;
    fac_inputs& in = in_[i];
    const FacRecord& own = f.fac[i];
    in.time.dt = f.base.simulation_delta_time_s;
    in.time.simulation_time = f.base.simulation_time_s;
    monotonic_[i] += f.base.simulation_delta_time_s;
    in.time.monotonic_time = monotonic_[i];
    in.sim_data = {};
    in.sim_data.computer_running = info.healthy[i];

    // the opposite FAC as this one sees it: FAC 2's previous outputs for FAC 1 (zero before its first run), FAC 1's
    // outputs of this frame for FAC 2
    base_fac_bus oppBus{};
    base_fac_discrete_outputs oppDisc{};
    if (i == 1) {
      oppBus = f.fac[0].bus_outputs;
      oppDisc = f.fac[0].discrete_outputs;
    } else if (!first) {
      oppBus = prev.fac[1].bus_outputs;
      oppDisc = prev.fac[1].discrete_outputs;
    }

    base_fac_discrete_inputs& d = in.discrete_inputs;
    const bool fmgcOwnAp = i == 0 ? fmgc1Ap_ : fmgc2Ap_;
    const bool fmgcOppAp = i == 0 ? fmgc2Ap_ : fmgc1Ap_;
    d.ap_own_engaged = fmgcOwnAp;
    d.ap_opp_engaged = fmgcOppAp;
    d.yaw_damper_opp_engaged = oppDisc.yaw_damper_engaged;
    d.rudder_trim_opp_engaged = oppDisc.rudder_trim_engaged;
    d.rudder_travel_lim_opp_engaged = oppDisc.rudder_travel_lim_engaged;
    d.elac_1_healthy = f.elac[0].discrete_outputs.digital_output_validated;
    d.elac_2_healthy = f.elac[1].discrete_outputs.digital_output_validated;
    d.engine_1_stopped = true;
    d.engine_2_stopped = true;
    d.fac_opp_healthy = oppDisc.fac_healthy;
    d.is_unit_1 = i == 0;
    d.rudder_trim_actuator_healthy = true;
    d.rudder_travel_lim_actuator_healthy = true;
    d.ir_3_switch = false;
    d.adr_3_switch = false;

    // pushbutton and hydraulic pressure from this FAC's discretes (see the file comment)
    const double psi = i == 0 ? f.specific.aircraft_hydraulic_system_green_pressure_psi
                              : f.specific.aircraft_hydraulic_system_yellow_pressure_psi;
    bool pb = true, hyd = psi > 1450.0;
    if (info.healthy[i]) {
      const base_fac_discrete_outputs& o = own.discrete_outputs;
      if (o.yaw_damper_avail_for_norm_law) {
        pb = hyd = true;
      } else if (i == 0 || !d.rudder_trim_opp_engaged) {
        pb = o.rudder_trim_engaged;
        hyd = pb ? false : hyd;
      } else if (!d.rudder_travel_lim_opp_engaged) {
        pb = o.rudder_travel_lim_engaged;
        hyd = pb ? false : hyd;
      } else {
        hyd = false;
      }
    }
    d.fac_engaged_from_switch = pb;
    d.yaw_damper_has_hyd_press = hyd;

    // analog positions: the copies on this FAC's bus (the other FAC's when this one is not healthy)
    const base_fac_bus& posBus = info.healthy[i] ? own.bus_outputs : f.fac[opp].bus_outputs;
    in.analog_inputs.yaw_damper_position_deg = 0.0;
    if (info.healthy[i] || info.healthy[opp]) {
      in.analog_inputs.rudder_trim_position_deg = posBus.rudder_trim_pos_deg.Data;
      in.analog_inputs.rudder_travel_lim_position_deg = posBus.rudder_travel_limit_command_deg.Data;
    }

    base_fac_bus_inputs& b = in.bus_inputs;
    b.fac_opp_bus = oppBus;
    // before FMGC 1 first runs, its frozen record (initial, all failed) stands for the ADRs while the FACs report
    // VMAX 0 (no static pressure: ADRs off) and for the IRs while they report flight path angles of exactly 0
    const FmgcRecord& adr = info.approxInputs && !approxAdrOn_[i] ? f.fmgc1 : m;
    const FmgcRecord& ir = info.approxInputs && !approxIrOn_[i] ? f.fmgc1 : m;
    b.adr_own_bus = i == 0 ? adr.bus_inputs.adr_own_bus : adr.bus_inputs.adr_opp_bus;
    b.adr_opp_bus = i == 0 ? adr.bus_inputs.adr_opp_bus : adr.bus_inputs.adr_own_bus;
    b.adr_3_bus = adr.bus_inputs.adr_3_bus;
    b.ir_own_bus = i == 0 ? ir.bus_inputs.ir_own_bus : ir.bus_inputs.ir_opp_bus;
    b.ir_opp_bus = i == 0 ? ir.bus_inputs.ir_opp_bus : ir.bus_inputs.ir_own_bus;
    b.ir_3_bus = ir.bus_inputs.ir_3_bus;
    b.fmgc_own_bus = i == 0 ? fmgc1B_ : fmgc2B_;
    b.fmgc_opp_bus = i == 0 ? fmgc2B_ : fmgc1B_;
    b.elac_1_bus = f.elac[0].bus_outputs;
    b.elac_2_bus = f.elac[1].bus_outputs;
    if (info.healthy[i]) {
      b.sfcc_own_bus = sfccFromOutputs(own.bus_outputs);
      bool nose = false;
      b.lgciu_own_bus = lgciuFromOutputs(own.bus_outputs, nose);
      d.nose_gear_pressed = nose;
    }
    const base_arinc_429& pos = b.sfcc_own_bus.slat_flap_actual_position_word;
    d.slats_extended = !(pos.SSM == kNo && bitOf(pos, 12));
  }

  /**
   * Reads the whole file once for FMGC 1's running state per frame. A frame where the FMGC 1 record did not change is a
   * frame it did not run when it belongs to a frozen stretch that starts with the file (the start-up self-test; frame
   * 0 holds the initial outputs) or lasts at least 25 s (a self-test after a power loss on the ground lasts 30 s);
   * shorter stretches are a running FMGC whose inputs and outputs did not change (parked). Keeps the first running
   * frame's record as the stand-in for the start-up frames.
   */
  void prepass(const std::string& path) {
    FdrReader r(path);
    auto cur = std::make_unique<FdrFrame>(), prev = std::make_unique<FdrFrame>();
    std::memset(cur.get(), 0, sizeof(FdrFrame));
    std::memset(prev.get(), 0, sizeof(FdrFrame));
    std::vector<double> time;
    long n = 0;
    while (r.next(*cur)) {
      running_.push_back(n > 0 && std::memcmp(&cur->fmgc1, &prev->fmgc1, sizeof(FmgcRecord)) != 0);
      approxEnd_.push_back(false);
      time.push_back(cur->base.simulation_time_s);
      if (firstRunning_ < 0 && running_.back()) {
        firstRunning_ = n;
        std::memcpy(static_cast<void*>(&proxy_), &cur->fmgc1, sizeof(FmgcRecord));
      }
      std::swap(cur, prev);
      n++;
    }
    // frame 0 cannot be compared with an earlier record: it ran if frame 1 did
    if (n > 1) running_[0] = running_[1];
    // frozen stretches
    for (long a = 0; a < n;) {
      if (running_[a]) {
        a++;
        continue;
      }
      long b = a;
      while (b + 1 < n && !running_[b + 1]) b++;
      const bool atStart = a <= 1;
      const bool longEnough = time[b] - time[a] >= 25.0;
      if (!atStart && !longEnough) {
        for (long k = a; k <= b; k++) running_[k] = true;
      }
      if (!atStart && b == n - 1) {
        // frozen until the end of the file: either parked with nothing changing or FMGC 1 switched off first at the
        // end of the session; its sensor buses may be stale (approximate inputs)
        for (long k = a; k <= b; k++) approxEnd_[k] = true;
      }
      a = b + 1;
    }
    running_.push_back(false);  // past the end
  }

  FdrReader reader_;
  Options opt_;
  std::vector<uint8_t> running_, approxEnd_;
  long firstRunning_ = -1;
  FmgcRecord proxy_;
  std::unique_ptr<FdrFrame> buf_[3];  // previous, current, next
  bool haveNext_ = false;
  std::unique_ptr<FacComputer> ref_[2], snap_[2];
  std::unique_ptr<FacComputer::ExternalInputs_FacComputer_T> ext_;
  fac_inputs in_[2];
  double monotonic_[2] = {0, 0};
  base_fmgc_b_bus fmgc1B_{}, fmgc2B_{};
  base_fmgc_a_bus fmgc2A_{};
  bool approxAdrOn_[2] = {false, false}, approxIrOn_[2] = {false, false};
  bool fmgc1Ap_ = false, fmgc2Ap_ = false, fmgc2Healthy_ = false, fmgc2Known_ = false;
  int lastTrim_ = 0;
  bool trimUnresolvedThisFrame_ = false;
  long frames_ = 0, healthyFrames_[2] = {0, 0}, fmgc1Frozen_ = 0, fmgc2Approx_ = 0, gaps_ = 0;
  long trimFrames_ = 0, trimSearched_ = 0, trimUnresolved_ = 0, abMismatch_ = 0;
};
