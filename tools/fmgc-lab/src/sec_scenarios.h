// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The scripted tour of the SEC differential check (src/check_sec.cpp --script SEED), applied on top of the drive
// (src/efcs_drive.h) before the generic injectors (src/efcs_scenarios.h). Phases of 8-45 s (the seed picks the first
// one and jitters values), each overriding inputs on every SEC:
//   0 touchdown: main gears through both LGCIUs, lever armed / extended, thrust levers idle / reverse / above 20 deg
//   1 rejected take-off: wheel speeds ramp through 23 and 72 kt, lever armed, thrust levers to idle or reverse
//   2 partial lift dumping: one main gear, lever armed, thrust levers at 25 deg or less
//   3 speed brakes: lever sweeps, AP (half deflection), inhibition by alpha protection, flaps FULL, thrust lever MCT,
//     elevators lost, then lever retracted for 10 s
//   4 roll spoilers: ELAC roll spoiler orders (either ELAC, or none: the SEC's own direct law), the enable bits,
//     both ELACs' digital outputs failed
//   5 LGCIU words: gears not uplocked, handle bits, one / both LGCIUs failed, disagreement beyond 10 s
//   6 SEC pitch: both ELACs' pitch lost, the other SEC's elevators lost, THS ground setting after touchdown
//   7 ADR / IR selection: every failure pattern; abnormal attitude values
//   8 inputs held at and beyond the thresholds (thrust levers, lever, wheel speed, attitude, speed, alpha)

#pragma once

#include <cmath>
#include <cstdint>

#include "SecComputer_types.h"
#include "efcs_scenarios.h"

namespace sec_scenarios {

using efcs_scenarios::Rng;

constexpr uint32_t kFw = 0, kNcd = 1, kNo = 3;

inline void setBit(base_arinc_429& w, int bit, bool v) {
  float r = std::round(w.Data);
  uint32_t x = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  x = v ? (x | (1U << (bit - 1))) : (x & ~(1U << (bit - 1)));
  w.Data = static_cast<float>(x);
}

inline base_arinc_429 no(double v) { return {kNo, static_cast<float>(v)}; }

class ScriptScenario {
 public:
  explicit ScriptScenario(uint64_t seed) : rng_(seed), phase_(static_cast<int>(seed % kPhases)) {}
  long episodes() const { return episodes_; }

  void apply(sec_inputs (&in)[3], double dt) {
    const double step = std::isfinite(dt) && dt > 0 ? dt : 0.02;
    t_ += step;
    if (t_ >= duration_) {
      t_ = 0;
      phase_ = (phase_ + 1) % kPhases;
      duration_ = rng_.u(8.0, 45.0);
      jitter_ = rng_.u(-1, 1);
      variant_ = rng_.irand(8);
      episodes_++;
    }
    for (auto& i : in) applyPhase(i);
  }

 private:
  static constexpr int kPhases = 9;

  static void hydraulics(sec_inputs& i) {
    auto& d = i.discrete_inputs;
    d.green_low_pressure = d.blue_low_pressure = d.yellow_low_pressure = false;
  }
  /** Main gears compressed (word 2 bits 13 / 14) on both LGCIUs, words valid. */
  static void gear(sec_inputs& i, bool left, bool right, bool notUplocked) {
    for (base_lgciu_bus* l : {&i.bus_inputs.lgciu_1_bus, &i.bus_inputs.lgciu_2_bus}) {
      l->discrete_word_2 = no(0);
      l->discrete_word_3 = no(0);
      setBit(l->discrete_word_2, 13, left);
      setBit(l->discrete_word_2, 14, right);
      setBit(l->discrete_word_3, 11, notUplocked);
      setBit(l->discrete_word_3, 12, notUplocked);
      setBit(l->discrete_word_3, 13, notUplocked);
      setBit(l->discrete_word_3, 14, !notUplocked);
    }
  }
  static void levers(sec_inputs& i, double tla1, double tla2, double spoilerLever) {
    i.analog_inputs.thr_lever_1_pos = tla1;
    i.analog_inputs.thr_lever_2_pos = tla2;
    i.analog_inputs.spd_brk_lever_pos = spoilerLever;
  }

  void applyPhase(sec_inputs& i) {
    auto& d = i.discrete_inputs;
    auto& a = i.analog_inputs;
    auto& b = i.bus_inputs;
    const double ramp = std::fmod(t_, 20.0) / 20.0;
    switch (phase_) {
      case 0: {  // touchdown
        hydraulics(i);
        const bool down = t_ > 4;
        gear(i, down || (variant_ & 1), down, true);
        const double tla = (variant_ & 2) ? -10.0 : ((variant_ & 4) ? 30.0 : 0.0);
        levers(i, tla, (variant_ == 5) ? 40.0 : tla, (variant_ & 1) ? -0.05 : 0.4 * ramp);
        a.wheel_speed_left = a.wheel_speed_right = down ? 120.0 * (1.0 - ramp) : 0.0;
        break;
      }
      case 1: {  // rejected take-off
        hydraulics(i);
        gear(i, true, true, true);
        a.wheel_speed_left = 150.0 * ramp;
        a.wheel_speed_right = 150.0 * ramp + 5.0 * jitter_;
        const bool rto = ramp > 0.6;
        levers(i, rto ? ((variant_ & 1) ? -5.0 : 0.0) : 45.0, rto ? 0.0 : 45.0, (variant_ & 2) ? -0.05 : 0.0);
        break;
      }
      case 2:  // partial lift dumping: one main gear
        hydraulics(i);
        gear(i, (variant_ & 1) && t_ > 3, !(variant_ & 1) && t_ > 3, true);
        if (variant_ & 2) setBit(b.lgciu_2_bus.discrete_word_2, 13, false);
        levers(i, (variant_ & 4) ? 22.0 : 0.0, 0.0, -0.05);
        a.wheel_speed_left = a.wheel_speed_right = 40.0;
        break;
      case 3: {  // speed brakes and their inhibition
        hydraulics(i);
        gear(i, false, false, false);
        const double lever = t_ < 0.7 * duration_ ? 1.2 * ramp - 0.1 : 0.0;
        levers(i, (variant_ == 3) ? 36.0 : 25.0, 25.0, lever);
        setBit(b.elac_1_bus.discrete_status_word_2, 22, variant_ & 1);
        setBit(b.elac_2_bus.discrete_status_word_2, 22, false);
        setBit(b.elac_1_bus.discrete_status_word_2, 23, variant_ == 2 || variant_ == 1);
        setBit(b.elac_2_bus.discrete_status_word_2, 23, variant_ == 6);
        b.elac_1_bus.discrete_status_word_2.SSM = variant_ == 1 ? kFw : kNo;
        b.elac_2_bus.discrete_status_word_2.SSM = kNo;
        setBit(b.sfcc_1_bus.slat_flap_actual_position_word, 23, variant_ == 4);
        b.sfcc_1_bus.slat_flap_actual_position_word.SSM = kNo;
        setBit(b.elac_1_bus.discrete_status_word_1, 17, variant_ != 5);
        setBit(b.elac_2_bus.discrete_status_word_1, 18, variant_ != 5);
        if (variant_ == 7) {
          d.pitch_not_avail_elac_1 = d.pitch_not_avail_elac_2 = true;
          d.left_elev_not_avail_sec_opp = true;
          d.l_elev_servo_failed = true;
        }
        break;
      }
      case 4:  // roll spoilers
        hydraulics(i);
        gear(i, false, false, false);
        b.elac_1_bus.roll_spoiler_command_deg = {(variant_ & 1) ? kNo : kNcd, static_cast<float>(30 * jitter_)};
        b.elac_2_bus.roll_spoiler_command_deg = {(variant_ & 2) ? kNo : kFw, static_cast<float>(-25 * jitter_)};
        setBit(b.elac_1_bus.discrete_status_word_1, 28, variant_ & 4);
        setBit(b.elac_1_bus.discrete_status_word_1, 29, !(variant_ & 4));
        setBit(b.elac_2_bus.discrete_status_word_1, 28, ramp > 0.5);
        setBit(b.elac_2_bus.discrete_status_word_1, 29, ramp < 0.5);
        d.digital_output_failed_elac_1 = d.digital_output_failed_elac_2 = variant_ == 0;
        a.capt_roll_stick_pos = std::sin(t_);
        levers(i, 25.0, 25.0, (variant_ & 4) ? 0.5 : 0.0);
        break;
      case 5: {  // LGCIU words
        const int k = (variant_ * 3 + static_cast<int>(t_ / 4)) & 15;
        gear(i, k & 1, k & 2, k & 4);
        if (k & 8) {
          setBit(b.lgciu_1_bus.discrete_word_3, 14, true);
          setBit(b.lgciu_1_bus.discrete_word_3, 11, false);
          setBit(b.lgciu_1_bus.discrete_word_3, 12, false);
          setBit(b.lgciu_1_bus.discrete_word_3, 13, false);
        }
        if (variant_ & 1) b.lgciu_1_bus.discrete_word_3.SSM = kFw;
        if (variant_ & 2) b.lgciu_2_bus.discrete_word_3.SSM = kFw;
        if (variant_ >= 5) {  // single gear bits (11, 12, 13 one at a time)
          const int bit = 11 + (static_cast<int>(t_ / 2) % 3);
          for (base_lgciu_bus* l : {&b.lgciu_1_bus, &b.lgciu_2_bus}) {
            setBit(l->discrete_word_3, 11, bit == 11);
            setBit(l->discrete_word_3, 12, bit == 12);
            setBit(l->discrete_word_3, 13, bit == 13 && (l == &b.lgciu_1_bus || (variant_ & 1)));
          }
        }
        if (variant_ == 4) {
          setBit(b.lgciu_2_bus.discrete_word_3, 14, true);
          setBit(b.lgciu_2_bus.discrete_word_3, 11, false);
          setBit(b.lgciu_2_bus.discrete_word_3, 12, false);
          setBit(b.lgciu_2_bus.discrete_word_3, 13, false);
          setBit(b.lgciu_1_bus.discrete_word_3, 14, true);
          setBit(b.lgciu_1_bus.discrete_word_3, 11, true);
        }
        setBit(b.sfcc_2_bus.slat_flap_actual_position_word, 20, variant_ & 4);
        b.sfcc_2_bus.slat_flap_actual_position_word.SSM = variant_ == 6 ? kFw : kNo;
        break;
      }
      case 6:  // SEC pitch: both ELACs' pitch lost, the other SEC's elevators, THS ground setting after touchdown
        hydraulics(i);
        d.pitch_not_avail_elac_1 = d.pitch_not_avail_elac_2 = !(variant_ == 7);
        d.left_elev_not_avail_sec_opp = (variant_ & 1) != 0;
        d.right_elev_not_avail_sec_opp = (variant_ & 1) != 0;
        gear(i, t_ > 6, t_ > 6, true);
        for (base_ir_bus* r : {&b.ir_1_bus, &b.ir_2_bus}) r->pitch_angle_deg = no(t_ < 3 ? 10.0 : 1.0);
        a.ths_pos_deg = 3.0 * jitter_ * (1.0 - ramp);
        d.ths_override_active = variant_ == 6;
        a.capt_pitch_stick_pos = 0.5 * std::sin(0.5 * t_);
        b.elac_1_bus.elevator_double_pressurization_command_deg = {(variant_ & 2) ? kNo : kNcd, 7.0F};
        b.elac_2_bus.elevator_double_pressurization_command_deg = {(variant_ & 4) ? kNo : kFw, -7.0F};
        setBit(b.elac_1_bus.discrete_status_word_1, 20, variant_ & 2);
        setBit(b.elac_2_bus.discrete_status_word_1, 20, variant_ & 4);
        b.elac_2_bus.discrete_status_word_1.SSM = variant_ == 4 ? kFw : kNo;
        if (variant_ == 3) d.green_low_pressure = d.yellow_low_pressure = true;  // SEC 1 in pitch without the THS
        break;
      case 8: {  // thresholds: thrust levers 20 / 25 / 35 / 0, lever 0.05 / -0.02, wheels 23, abnormal attitude values
        hydraulics(i);
        const int k = (variant_ * 5 + static_cast<int>(t_ / 1.5)) % 12;
        static const double kTla[12] = {20, 25, 30, 35, 0, -5, 20, 25, 30, 0, 37, 10};
        gear(i, (k & 1) || t_ > 6, t_ > 6, true);
        levers(i, (variant_ & 1) ? kTla[k] : 0.0, (variant_ & 1) ? 0.0 : kTla[k],
               (variant_ & 2) ? -0.02 : ((variant_ & 4) ? 0.05 : -0.6));
        a.wheel_speed_left = a.wheel_speed_right = (variant_ & 4) ? 23.0 : 80.0;
        a.ths_pos_deg = 0.02;
        for (base_ir_bus* r : {&b.ir_1_bus, &b.ir_2_bus}) {
          r->pitch_angle_deg = no(k == 2 ? 50.0 : (k == 3 ? 60.0 : (k == 4 ? -30.0 : (k == 5 ? 9.0 : 3.0))));
          r->roll_angle_deg = no(k == 6 ? 125.0 : (k == 7 ? 150.0 : 0.0));
        }
        for (base_adr_bus* r : {&b.adr_1_bus, &b.adr_2_bus}) {
          r->airspeed_computed_kn = no(k == 8 ? 440.0 : (k == 9 ? 60.0 : 200.0));
          r->aoa_corrected_deg = no(k == 10 ? 40.0 : (k == 11 ? -10.0 : 3.0));
        }
        break;
      }
      default: {  // ADR / IR selection, abnormal attitude values
        const int k = (variant_ + static_cast<int>(t_ / 3)) & 15;
        if (k & 1) b.adr_1_bus.mach.SSM = kFw;
        if (k & 2) b.adr_2_bus.aoa_corrected_deg.SSM = kFw;
        if (k & 4) b.ir_1_bus.body_lat_accel_g.SSM = kNcd;
        if (k & 8) b.ir_2_bus.roll_angle_deg.SSM = kFw;
        gear(i, false, false, false);
        hydraulics(i);
        if (variant_ & 1) {
          for (base_ir_bus* r : {&b.ir_1_bus, &b.ir_2_bus}) {
            r->pitch_angle_deg.Data = static_cast<float>(60 * jitter_);
            r->roll_angle_deg.Data = static_cast<float>(140 * jitter_);
          }
          for (base_adr_bus* r : {&b.adr_1_bus, &b.adr_2_bus}) {
            r->airspeed_computed_kn.Data = static_cast<float>(250 + 250 * jitter_);
            r->mach.Data = static_cast<float>(0.6 + 0.4 * jitter_);
            r->aoa_corrected_deg.Data = static_cast<float>(30 * jitter_);
          }
        }
        break;
      }
    }
  }

  Rng rng_;
  int phase_;
  double t_ = 0, duration_ = 10.0, jitter_ = 0;
  int variant_ = 0;
  long episodes_ = 0;
};

}  // namespace sec_scenarios
