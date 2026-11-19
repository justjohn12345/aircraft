// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The scripted tour of the ELAC differential check (src/check_elac.cpp --script SEED), applied on top of the drive
// (src/efcs_drive.h) before the generic injectors (src/efcs_scenarios.h). It cycles through phases of 8-45 s (the
// seed picks the first one and jitters values); every phase overrides a set of inputs on both ELACs to reach a
// function the recordings do not exercise:
//   0 aileron cross-command (ELAC 2 with ELAC 1's ailerons lost)    6 touchdown with the THS away from 0, override
//   1 radio altimeters: disagreement, flaps FULL, one / both failed  7 double pressurisation, the other ELAC's order
//   2 high speed protection, diving, AP                             8 ADR / IR voting: every failure pattern
//   3 high angle of attack protection, stick inputs                 9 law capabilities: SEC / FCDC / FAC / ELAC words
//   4 abnormal attitude and envelope limits                         10 aileron droop / anti-droop
//   5 sidestick priority, locked after 30 s

#pragma once

#include <cmath>
#include <cstdint>

#include "ElacComputer_types.h"
#include "efcs_scenarios.h"

namespace elac_scenarios {

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

  void apply(elac_inputs (&in)[2], double dt) {
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
    for (int u = 0; u < 2; u++) applyPhase(in[u], u);
  }

 private:
  static constexpr int kPhases = 11;

  static void allAdr(elac_inputs& i, double cas, double mach, double alpha) {
    for (base_adr_bus* a : {&i.bus_inputs.adr_1_bus, &i.bus_inputs.adr_2_bus, &i.bus_inputs.adr_3_bus}) {
      a->airspeed_computed_kn = no(cas);
      a->airspeed_true_kn = no(cas * 1.3);
      a->mach = no(mach);
      a->aoa_corrected_deg = no(alpha);
    }
  }
  static void allIr(elac_inputs& i, double theta, double phi) {
    for (base_ir_bus* b : {&i.bus_inputs.ir_1_bus, &i.bus_inputs.ir_2_bus, &i.bus_inputs.ir_3_bus}) {
      b->pitch_angle_deg = no(theta);
      b->roll_angle_deg = no(phi);
      b->body_yaw_rate_deg_s = no(0.5);
      b->body_long_accel_g = no(0.0);
      b->body_lat_accel_g = no(0.0);
      b->body_normal_accel_g = no(1.0);
      b->pitch_att_rate_deg_s = no(0.2);
      b->roll_att_rate_deg_s = no(0.1);
      b->body_pitch_rate_deg_s = no(0.2);
    }
  }
  static void ra(elac_inputs& i, double h1, double h2) {
    i.bus_inputs.ra_1_bus.radio_height_ft = no(h1);
    i.bus_inputs.ra_2_bus.radio_height_ft = no(h2);
  }
  static void hydraulics(elac_inputs& i) {
    auto& d = i.discrete_inputs;
    d.green_low_pressure = d.blue_low_pressure = d.yellow_low_pressure = false;
    i.analog_inputs.green_hyd_pressure_psi = 3000.0;
    i.analog_inputs.blue_hyd_pressure_psi = 3000.0;
    i.analog_inputs.yellow_hyd_pressure_psi = 3000.0;
  }
  static void inFlight(elac_inputs& i) {
    auto& d = i.discrete_inputs;
    d.lgciu_1_left_main_gear_pressed = d.lgciu_1_right_main_gear_pressed = false;
    d.lgciu_2_left_main_gear_pressed = d.lgciu_2_right_main_gear_pressed = false;
    d.ground_spoilers_active_1 = d.ground_spoilers_active_2 = false;
  }
  static void onGround(elac_inputs& i) {
    auto& d = i.discrete_inputs;
    d.lgciu_1_left_main_gear_pressed = d.lgciu_1_right_main_gear_pressed = true;
    d.lgciu_2_left_main_gear_pressed = d.lgciu_2_right_main_gear_pressed = true;
  }
  static void sfccWord(elac_inputs& i, int bit, bool on) {
    setBit(i.bus_inputs.sfcc_1_bus.slat_flap_actual_position_word, bit, on);
    setBit(i.bus_inputs.sfcc_2_bus.slat_flap_actual_position_word, bit, on);
    i.bus_inputs.sfcc_1_bus.slat_flap_actual_position_word.SSM = kNo;
    i.bus_inputs.sfcc_2_bus.slat_flap_actual_position_word.SSM = kNo;
  }

  void applyPhase(elac_inputs& i, int unit) {
    auto& d = i.discrete_inputs;
    auto& a = i.analog_inputs;
    auto& b = i.bus_inputs;
    const double ramp = std::fmod(t_, 20.0) / 20.0;  // 0..1 every 20 s
    switch (phase_) {
      case 0:  // aileron cross-command: ELAC 2 with ELAC 1's ailerons lost (one or both) and its aileron order
        hydraulics(i);
        if (unit == 1) {
          d.opp_left_aileron_lost = variant_ & 1;
          d.opp_right_aileron_lost = (variant_ & 2) || !(variant_ & 1);
          b.elac_opp_bus.aileron_command_deg = {(variant_ & 4) ? kNcd : kNo, static_cast<float>(20 * jitter_)};
        } else {
          d.l_ail_servo_failed = variant_ & 1;
          d.r_ail_servo_failed = variant_ & 2;
        }
        break;
      case 1:  // radio altimeters: disagreement, flaps FULL for 10 s, one / both failed, CAS above 180 / 200 kt
        hydraulics(i);
        inFlight(i);
        ra(i, 300 + 200 * ramp, 300 + ((variant_ & 1) ? 90 : 10) * jitter_);
        if (variant_ & 2) b.ra_1_bus.radio_height_ft.SSM = kFw;
        if (variant_ & 4) b.ra_2_bus.radio_height_ft.SSM = kFw;
        sfccWord(i, 23, t_ > 3);
        allAdr(i, 150 + 100 * ramp, 0.3, 4.0);
        if (variant_ == 7) {
          b.adr_1_bus.mach.SSM = kFw;
          b.adr_2_bus.mach.SSM = kFw;
          b.adr_3_bus.mach.SSM = kFw;
        }
        break;
      case 2: {  // high speed protection: CAS / Mach sweeps through VMO / MMO + margins, diving or not, AP off / on
        hydraulics(i);
        inFlight(i);
        ra(i, 2500, 2500);
        const double cas = 330 + 60 * ramp;
        allAdr(i, cas, (variant_ & 1) ? 0.78 + 0.1 * ramp : cas / 450, 2.0);
        allIr(i, (variant_ & 2) ? -6.0 : 2.0, 5.0 * jitter_);
        d.ap_1_disengaged = d.ap_2_disengaged = !(variant_ & 4);
        a.capt_pitch_stick_pos = jitter_ > 0 ? 0.0 : -0.6 * ramp;
        break;
      }
      case 3:  // high angle of attack: alpha ramps through alpha prot / max, stick pulled / released / pushed
        hydraulics(i);
        inFlight(i);
        ra(i, (variant_ & 1) ? 150 : 1500, (variant_ & 1) ? 150 : 1500);
        allAdr(i, 140, 0.3, 4 + 14 * ramp);
        allIr(i, 10, 0);
        d.ap_1_disengaged = d.ap_2_disengaged = true;
        a.capt_pitch_stick_pos = (variant_ & 2) ? (ramp > 0.7 ? 0.8 : -0.2) : (ramp > 0.8 ? -0.7 : 0.0);
        break;
      case 4: {  // abnormal attitude and envelope limits, in flight, then on the ground
        hydraulics(i);
        inFlight(i);
        ra(i, 3000, 3000);
        const int v = variant_;
        allAdr(i, v == 0 ? 450 : (v == 1 ? 50 : 250), v == 2 ? 0.95 : 0.5, v == 3 ? 45 : (v == 4 ? -15 : 3));
        allIr(i, v == 5 ? 55 : (v == 6 ? -35 : 3), v == 7 ? 130 : 0);
        if (t_ > 0.6 * duration_) {
          onGround(i);
          ra(i, 0, 0);
        }
        break;
      }
      case 5:  // sidestick priority: takeover pushbuttons held beyond 30 s (locked), released, the other one
        hydraulics(i);
        d.capt_priority_takeover_pressed = (variant_ & 1) ? t_ < 35 : (t_ > 2 && t_ < 6);
        d.fo_priority_takeover_pressed = (variant_ & 2) ? (t_ > 4 && t_ < 40) : false;
        a.capt_roll_stick_pos = 0.5 * jitter_;
        a.fo_roll_stick_pos = -0.4;
        a.fo_pitch_stick_pos = 0.3;
        break;
      case 6:  // touchdown with the THS away from 0 (THS ground setting), override, THS motor fault
        hydraulics(i);
        if (t_ < 3) {
          inFlight(i);
          ra(i, 500, 500);
          allIr(i, 9, 0);
        } else {
          onGround(i);
          ra(i, 0, 0);
          allIr(i, 1, 0);
        }
        a.ths_pos_deg = 2.0 * jitter_ * (1.0 - ramp) + ((variant_ & 1) ? 0.01 : 0.0);
        d.ths_override_active = variant_ == 3;
        d.ths_motor_fault = variant_ == 5;
        d.opp_axis_pitch_failure = variant_ & 4;
        break;
      case 7:  // double pressurisation: large elevator orders at speed; ELAC not in pitch with the other's order
        hydraulics(i);
        inFlight(i);
        ra(i, 2500, 2500);
        allAdr(i, 180 + 250 * ramp, 0.6, 3);
        a.capt_pitch_stick_pos = (variant_ & 1) ? 1.0 : -1.0;
        d.opp_axis_pitch_failure = variant_ & 2;
        b.elac_opp_bus.elevator_double_pressurization_command_deg = {(variant_ & 4) ? kNo : kNcd,
                                                                    static_cast<float>(-12 * jitter_)};
        setBit(b.elac_opp_bus.discrete_status_word_1, 17, variant_ & 1);
        setBit(b.elac_opp_bus.discrete_status_word_1, 18, variant_ & 2);
        setBit(b.sec_1_bus.discrete_status_word_1, 17, variant_ & 4);
        setBit(b.sec_2_bus.discrete_status_word_1, 18, variant_ & 4);
        break;
      case 8: {  // ADR / IR voting: every pattern of failed ADRs and IRs
        const int adr = (variant_ + static_cast<int>(t_ / 3)) & 7, ir = (variant_ * 3 + static_cast<int>(t_ / 5)) & 7;
        base_adr_bus* adrs[3] = {&b.adr_1_bus, &b.adr_2_bus, &b.adr_3_bus};
        base_ir_bus* irs[3] = {&b.ir_1_bus, &b.ir_2_bus, &b.ir_3_bus};
        for (int k = 0; k < 3; k++) {
          if (adr & (1 << k)) adrs[k]->airspeed_computed_kn.SSM = kFw;
          if (ir & (1 << k)) irs[k]->roll_att_rate_deg_s.SSM = kNcd;
        }
        break;
      }
      case 9: {  // law capabilities: SEC / FCDC / FAC / opposite ELAC words, SEC gear bits
        const int k = (variant_ * 5 + static_cast<int>(t_ / 2)) & 31;
        setBit(b.sec_1_bus.discrete_status_word_1, 15, k & 1);
        setBit(b.sec_1_bus.discrete_status_word_1, 16, k & 2);
        setBit(b.sec_2_bus.discrete_status_word_1, 15, k & 4);
        b.sec_1_bus.discrete_status_word_1.SSM = (k & 8) ? kNo : kFw;
        b.sec_2_bus.discrete_status_word_1.SSM = (k & 8) ? kNo : kNcd;
        setBit(b.fcdc_1_bus.efcs_status_word_1, 29, k & 16);
        setBit(b.fcdc_2_bus.efcs_status_word_1, 29, k & 16);
        setBit(b.fcdc_1_bus.efcs_status_word_3, 23, k == 3);
        setBit(b.fcdc_2_bus.efcs_status_word_3, 24, k == 5);
        d.fac_1_yaw_control_lost = d.fac_2_yaw_control_lost = k == 7;
        setBit(b.sec_1_bus.discrete_status_word_2, 17, k & 1);
        setBit(b.sec_2_bus.discrete_status_word_2, 17, k & 2);
        setBit(b.sec_1_bus.discrete_status_word_2, 18, k & 4);
        b.sec_1_bus.discrete_status_word_2.SSM = (k & 8) ? kNo : kFw;
        b.sec_2_bus.discrete_status_word_2.SSM = (k & 16) ? kNo : kFw;
        sfccWord(i, 20, k & 2);
        setBit(b.elac_opp_bus.discrete_status_word_2, 11, variant_ & 1);
        setBit(b.elac_opp_bus.discrete_status_word_2, 12, variant_ & 2);
        setBit(b.elac_opp_bus.discrete_status_word_2, 13, k & 4);
        setBit(b.elac_opp_bus.discrete_status_word_2, 14, variant_ & 4);
        if (k & 1) b.ir_1_bus.pitch_angle_deg.SSM = b.ir_2_bus.pitch_angle_deg.SSM = kNcd;
        if (k & 2) d.l_elev_servo_failed = true;
        hydraulics(i);
        inFlight(i);
        ra(i, 3000, 3000);
        break;
      }
      default:  // aileron droop / anti-droop: flaps extended, ground spoilers, pitch below 2.5 deg, AP
        hydraulics(i);
        sfccWord(i, 19, variant_ & 1);
        d.ground_spoilers_active_1 = d.ground_spoilers_active_2 = variant_ & 2;
        allIr(i, (variant_ & 4) ? 1.0 : 5.0, 0);
        d.ap_1_disengaged = d.ap_2_disengaged = jitter_ > -0.5;
        d.opp_left_aileron_lost = jitter_ > 0.6;
        d.l_ail_servo_failed = jitter_ < -0.8;
        break;
    }
  }

  Rng rng_;
  int phase_;
  double t_ = 0, duration_ = 10.0, jitter_ = 0;
  int variant_ = 0;
  long episodes_ = 0;
};

}  // namespace elac_scenarios
