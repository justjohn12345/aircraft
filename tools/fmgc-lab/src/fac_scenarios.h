// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Input injectors for the FAC differential check (src/check_fac.cpp), applied on top of the drive (src/fac_drive.h)
// at every step, in this order: --script, --failures, --discretes, --power, --sim. Each one is reproducible from its
// seed. The generated FAC is the oracle, so any input is a valid test input; inconsistent combinations are wanted.
//
//  --script SEED     a scripted tour, cycling through phases (the seed picks the first one and jitters values):
//                    take-off configuration memory and VLS factors; alpha floor (alpha ramps through the thresholds
//                    in every configuration, Mach above 0.6, ELAC normal law / high AoA protection bits, radio height
//                    crossing 100 ft both ways, touchdown, deceleration in CONF 3/FULL); low energy (CONF 2..FULL,
//                    radio height window, CAS below VLS and VLS - 10, SRS take-off / go-around bits, alpha floor bit,
//                    ELAC bits, RA NCD / both failed); pitch-pitch; rudder trim (switches, both, reset with and
//                    without AP, auto trim above and below 200 ft, saturation, disengaged); yaw damper (alternate law
//                    on a yaw rate sine, ELAC orders beyond the limits, hydraulics, priority); travel limiter (CAS
//                    sweep, disengaged); speeds (weight / CG words, FAC weight fallback, altitude 0-45000 ft, speed
//                    brakes on either ELAC, gear down); source fallbacks (LGCIU, nose gear disagreement, SFCC with the
//                    opposite FAC's copy and the gear-down CONF FULL default, slat bit agreement, ADR / IR own -> 3 ->
//                    none, FMGC RA failed, FMGC selection with every AP combination); SFCC status bit combinations;
//                    inputs held exactly at the comparison thresholds (radio heights 20 / 100 / 200 ft, 20000 ft, alpha
//                    at the alpha floor threshold, pitch + pitch rate 10 deg, CAS on breakpoints) and beyond limits
//  --failures SEED   random episodes (0.1-20 s, one or both FACs): any ARINC word the FAC reads gets a random SSM and
//                    optionally random data (NaN, infinities, 0, huge, scaled, random discrete bits)
//  --discretes SEED  random episodes: any discrete input flipped (unit 1 included), analog positions (incl. NaN),
//                    random bits of the discrete words the FAC decodes
//  --power SEED      computer_running false for 1 frame, 2-10 frames, 0.2-5 s or 5-60 s, flicker, one or both FACs
//  --sim SEED        dt 0, tiny, 0.2-2 s; simulation time jumps; slew / pause / tracking flags

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>

#include "FacComputer_types.h"

namespace fac_scenarios {

class Rng {
 public:
  explicit Rng(uint64_t seed) : rng_(seed) {}
  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng_); }
  bool chance(double p) { return u(0, 1) < p; }
  int irand(int n) { return static_cast<int>(rng_() % static_cast<uint64_t>(n)); }

 private:
  std::mt19937_64 rng_;
};

constexpr uint32_t kFw = 0, kNcd = 1, kFt = 2, kNo = 3;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

inline void setBit(base_arinc_429& w, int bit, bool v) {
  float r = std::round(w.Data);
  uint32_t x = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  x = v ? (x | (1U << (bit - 1))) : (x & ~(1U << (bit - 1)));
  w.Data = static_cast<float>(x);
}

inline base_arinc_429 no(double v) { return {kNo, static_cast<float>(v)}; }

/** Standard atmosphere static pressure (hPa) at a pressure altitude (ft). */
inline double pressureAt(double altFt) {
  const double h = altFt * 0.3048;
  return h < 11000 ? 1013.25 * std::pow(1.0 - 2.25577e-5 * h, 5.25588) : 226.32 * std::exp(-(h - 11000) / 6341.6);
}

// --- helpers that set a consistent picture on both FACs ---------------------------------------------------------------

/** Landing gear through the own LGCIU (valid, consistent nose gear) on both FACs. */
inline void gear(fac_inputs (&in)[2], bool left, bool right, bool down) {
  for (auto& i : in) {
    base_lgciu_bus& l = i.bus_inputs.lgciu_own_bus;
    l.discrete_word_1 = no(0);
    l.discrete_word_2 = no(0);
    l.discrete_word_3 = no(0);
    l.discrete_word_4 = no(0);
    setBit(l.discrete_word_2, 12, left || right);
    setBit(l.discrete_word_2, 13, left);
    setBit(l.discrete_word_2, 14, right);
    setBit(l.discrete_word_3, 11, down);
    i.discrete_inputs.nose_gear_pressed = left || right;
  }
}

/** Slats/flaps through the own SFCC (valid): configuration index 0..5, the matching FPPU angles. */
inline void config(fac_inputs (&in)[2], int idx, double fraction = 1.0) {
  static const double flap[6] = {0.0, 0.0, 120.23, 145.62, 168.35, 251.97};
  static const double slat[6] = {0.0, 222.27, 222.27, 272.27, 272.27, 334.16};
  static const int bits[6] = {17, 18, 18, 19, 20, 21};
  for (auto& i : in) {
    base_sfcc_bus& s = i.bus_inputs.sfcc_own_bus;
    s.slat_flap_system_status_word = no(0);
    setBit(s.slat_flap_system_status_word, bits[idx], true);
    setBit(s.slat_flap_system_status_word, 26, idx == 1);
    s.slat_flap_actual_position_word = no(0);
    setBit(s.slat_flap_actual_position_word, 12, idx > 0);
    s.flap_actual_position_deg = no(flap[idx] * fraction);
    s.slat_actual_position_deg = no(slat[idx] * fraction);
    i.discrete_inputs.slats_extended = !(idx > 0);
  }
}

/** Air data on every ADR (normal): CAS, Mach, alpha, altitude (and the matching static pressure). */
inline void airData(fac_inputs (&in)[2], double cas, double mach, double alpha, double alt) {
  for (auto& i : in) {
    for (base_adr_bus* a : {&i.bus_inputs.adr_own_bus, &i.bus_inputs.adr_opp_bus, &i.bus_inputs.adr_3_bus}) {
      a->airspeed_computed_kn = no(cas);
      a->airspeed_true_kn = no(cas * (1.0 + alt / 60000.0));
      a->mach = no(mach);
      a->aoa_corrected_deg = no(alpha);
      a->altitude_standard_ft = no(alt);
      a->corrected_average_static_pressure = no(pressureAt(alt));
    }
  }
}

/** Inertial data on every IR (normal). */
inline void inertial(fac_inputs (&in)[2], double theta, double phi, double r, double ny, double nz, double nx,
                     double thetaDot, double gs, double vz) {
  for (auto& i : in) {
    for (base_ir_bus* b : {&i.bus_inputs.ir_own_bus, &i.bus_inputs.ir_opp_bus, &i.bus_inputs.ir_3_bus}) {
      b->pitch_angle_deg = no(theta);
      b->roll_angle_deg = no(phi);
      b->body_yaw_rate_deg_s = no(r);
      b->body_pitch_rate_deg_s = no(thetaDot);
      b->body_lat_accel_g = no(ny);
      b->body_normal_accel_g = no(nz);
      b->body_long_accel_g = no(nx);
      b->pitch_att_rate_deg_s = no(thetaDot);
      b->roll_att_rate_deg_s = no(0.1 * phi);
      b->ground_speed_kn = no(gs);
      b->inertial_vertical_speed_ft_s = no(vz);
    }
  }
}

/** Radio height on both FMGC buses (own and opposite, both FACs). */
inline void radioHeight(fac_inputs (&in)[2], double ft, uint32_t ssm = kNo) {
  for (auto& i : in) {
    i.bus_inputs.fmgc_own_bus.fg_radio_height_ft = {ssm, static_cast<float>(ft)};
    i.bus_inputs.fmgc_opp_bus.fg_radio_height_ft = {ssm, static_cast<float>(ft)};
  }
}

/** FM weight and CG on both FMGC buses. */
inline void weight(fac_inputs (&in)[2], double lbs, double cg, uint32_t ssm = kNo) {
  for (auto& i : in) {
    for (base_fmgc_b_bus* b : {&i.bus_inputs.fmgc_own_bus, &i.bus_inputs.fmgc_opp_bus}) {
      b->fm_weight_lbs = {ssm, static_cast<float>(lbs)};
      b->fm_cg_percent = {ssm, static_cast<float>(cg)};
      b->fac_weight_lbs = no(0);
      b->fac_cg_percent = no(0);
    }
  }
}

/** ELAC discrete status words: bit 20 (FAC yaw damper source) and 26 (normal law) in word 1, high AoA bit 23 in word 2. */
inline void elac(fac_inputs (&in)[2], bool normal1, bool normal2, bool highAoa, double ydOrder = 0.0) {
  for (auto& i : in) {
    base_elac_out_bus* e[2] = {&i.bus_inputs.elac_1_bus, &i.bus_inputs.elac_2_bus};
    const bool normal[2] = {normal1, normal2};
    for (int k = 0; k < 2; k++) {
      e[k]->discrete_status_word_1 = no(0);
      setBit(e[k]->discrete_status_word_1, 20, normal[k]);
      setBit(e[k]->discrete_status_word_1, 26, normal[k]);
      e[k]->discrete_status_word_2 = no(0);
      setBit(e[k]->discrete_status_word_2, 23, highAoa && normal[k]);
      e[k]->yaw_damper_command_deg = no(ydOrder * (k ? 0.5 : 1.0));
    }
    i.discrete_inputs.elac_1_healthy = true;
    i.discrete_inputs.elac_2_healthy = true;
  }
}

/** FMGC discrete word 1 / ATS word bits on both FMGC buses. */
inline void fmgcBits(fac_inputs (&in)[2], bool srsTo, bool srsGa, bool togaLever, bool alphaFloor) {
  for (auto& i : in) {
    for (base_fmgc_b_bus* b : {&i.bus_inputs.fmgc_own_bus, &i.bus_inputs.fmgc_opp_bus}) {
      b->discrete_word_1 = no(0);
      setBit(b->discrete_word_1, 15, srsTo);
      setBit(b->discrete_word_1, 16, srsGa);
      setBit(b->discrete_word_1, 29, togaLever);
      b->ats_discrete_word = no(0);
      setBit(b->ats_discrete_word, 23, alphaFloor);
    }
  }
}

inline void ap(fac_inputs (&in)[2], bool ap1, bool ap2) {
  in[0].discrete_inputs.ap_own_engaged = ap1;
  in[0].discrete_inputs.ap_opp_engaged = ap2;
  in[1].discrete_inputs.ap_own_engaged = ap2;
  in[1].discrete_inputs.ap_opp_engaged = ap1;
}

inline void running(fac_inputs (&in)[2]) {
  for (auto& i : in) {
    i.sim_data.computer_running = true;
    i.discrete_inputs.fac_engaged_from_switch = true;
    i.discrete_inputs.yaw_damper_has_hyd_press = true;
    i.discrete_inputs.rudder_trim_actuator_healthy = true;
    i.discrete_inputs.rudder_travel_lim_actuator_healthy = true;
  }
  in[0].discrete_inputs.is_unit_1 = true;
  in[1].discrete_inputs.is_unit_1 = false;
  // FAC 2 sees FAC 1 engaged (FAC 1 has priority)
  in[1].discrete_inputs.yaw_damper_opp_engaged = in[1].discrete_inputs.rudder_trim_opp_engaged =
      in[1].discrete_inputs.rudder_travel_lim_opp_engaged = true;
  in[0].discrete_inputs.yaw_damper_opp_engaged = in[0].discrete_inputs.rudder_trim_opp_engaged =
      in[0].discrete_inputs.rudder_travel_lim_opp_engaged = false;
}

/** A clean in-flight picture to start each phase from. */
inline void cruise(fac_inputs (&in)[2], double cas, double alt, double alpha) {
  running(in);
  gear(in, false, false, false);
  config(in, 0);
  airData(in, cas, cas / 600.0 * (1.0 + alt / 50000.0), alpha, alt);
  inertial(in, alpha + 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, cas, 0.0);
  radioHeight(in, std::min(alt, 2500.0));
  weight(in, 140000.0, 0.27);
  elac(in, true, true, false);
  fmgcBits(in, false, false, false, false);
  ap(in, false, false);
  for (auto& i : in) {
    for (base_fmgc_b_bus* b : {&i.bus_inputs.fmgc_own_bus, &i.bus_inputs.fmgc_opp_bus}) {
      b->n1_left_percent = no(80);
      b->n1_right_percent = no(80);
    }
    i.discrete_inputs.rudder_trim_switch_left = i.discrete_inputs.rudder_trim_switch_right =
        i.discrete_inputs.rudder_trim_reset_button = false;
    i.bus_inputs.elac_1_bus.speedbrake_extension_deg = no(0);
    i.bus_inputs.elac_2_bus.speedbrake_extension_deg = no(0);
    // the opposite FAC's copy: valid LGCIU and SFCC, CONF 0
    base_fac_bus& o = i.bus_inputs.fac_opp_bus;
    o.discrete_word_5 = no(0);
    setBit(o.discrete_word_5, 16, true);
    setBit(o.discrete_word_5, 21, true);
  }
}

// --- --script -------------------------------------------------------------------------------------------------------

class ScriptScenario {
 public:
  explicit ScriptScenario(uint64_t seed) : rng_(seed * 7919 + 5) { phase_ = rng_.irand(kPhases); }
  long episodes() const { return episodes_; }

  void apply(fac_inputs (&in)[2], double dt) {
    if (t_ >= duration(phase_)) {
      t_ = 0;
      phase_ = (phase_ + 1) % kPhases;
      jitter_ = rng_.u(-1.0, 1.0);
      episodes_++;
    }
    run(in, t_);
    t_ += dt > 0 && std::isfinite(dt) ? dt : 0.02;
  }

 private:
  static constexpr int kPhases = 11;
  static double duration(int p) {
    static const double d[kPhases] = {40, 60, 56, 20, 60, 30, 40, 50, 60, 25, 92};
    return d[p];
  }

  void run(fac_inputs (&in)[2], double t) {
    const double j = jitter_;
    switch (phase_) {
      case 0: {  // take-off configuration memory, VLS factors, gear down VMO
        cruise(in, 0.0, 0.0, 0.0);
        const bool ground = t < 12;
        gear(in, ground, ground && t > 1, t < 25);
        config(in, t < 4 ? 0 : (t < 30 ? 2 + (episodes_ % 2) : (t < 35 ? 4 : 0)));
        airData(in, t < 12 ? 10.0 * t : 140.0 + 2.0 * t, 0.2, 5.0 + j, 1000.0 + 50.0 * t);
        radioHeight(in, t < 12 ? 0.0 : 30.0 * (t - 12));
        break;
      }
      case 1: {  // alpha floor
        cruise(in, 180.0 + 20.0 * j, 3000.0, 2.0);
        const int c = static_cast<int>(t / 10) % 6;
        config(in, c);
        const double s = std::fmod(t, 10.0);
        const double alpha = s < 5 ? 3.0 * s : 15.0 - 3.0 * (s - 5);
        const bool highAoa = s > 3 && s < 8.5;
        elac(in, !(t > 50 && t < 53), true, highAoa);
        airData(in, 200.0 - (c >= 4 ? 4.0 * s : 0.0), t > 45 && t < 48 ? 0.65 : 0.3, alpha, 3000.0);
        // radio height through 100 ft both ways, touchdown at the end
        radioHeight(in, t < 20 ? 300.0 : (t < 30 ? 50.0 : (t < 40 ? 150.0 : 300.0)), t > 33 && t < 35 ? kFw : kNo);
        if (t > 55) gear(in, true, true, true);
        for (auto& i : in) {  // ELAC status words failed with their bits still set
          if (t > 52 && t < 54) i.bus_inputs.elac_1_bus.discrete_status_word_1.SSM = kFw;
          if (t > 53 && t < 54) i.bus_inputs.elac_2_bus.discrete_status_word_1.SSM = kFw;
        }
        break;
      }
      case 2: {  // low energy
        cruise(in, 140.0, 1500.0, 9.0);
        config(in, 3 + static_cast<int>(t / 12) % 3);
        weight(in, 160000.0 + 10000.0 * j, 0.3);
        elac(in, !(t > 44 && t < 46) && t < 48, !(t > 44 && t < 48), false);
        if (t > 48 && t < 50) in[0].discrete_inputs.elac_2_healthy = in[1].discrete_inputs.elac_2_healthy = false;
        for (auto& i : in) {
          if (t > 50 && t < 52) setBit(i.bus_inputs.elac_2_bus.discrete_status_word_1, 26, false);
        }
        const double s = std::fmod(t, 12.0);
        const double cas = 150.0 - 6.0 * s;  // through VLS and VLS - 10
        airData(in, cas, 0.22, 4.0 + s, 1500.0);
        inertial(in, 2.0, 5.0 * j, 0.0, 0.0, 1.0, -0.05 * s, 0.5, cas, -10.0 * s);
        radioHeight(in, t < 30 ? 500.0 : (t < 33 ? 80.0 : (t < 36 ? 2200.0 : 500.0)),
                    t > 36 && t < 38 ? kNcd : (t > 38 && t < 40 ? kFw : kNo));
        fmgcBits(in, t > 24 && t < 25, (t > 26 && t < 26.2) || (t > 40 && t < 40.1), false, t > 28 && t < 29);
        break;
      }
      case 3: {  // pitch-pitch
        cruise(in, 140.0, 0.0, 5.0);
        gear(in, t < 10, t < 10, true);
        inertial(in, 4.0 + 0.6 * t, 0.0, 0.0, 0.0, 1.0, 0.0, 2.0 + j, 140.0, 0.0);
        radioHeight(in, t < 10 ? 0.0 : 2.0 * (t - 10), t > 6 && t < 8.5 ? kNcd : (t > 9 && t < 11.5 ? kFw : kNo));
        ap(in, t > 14 && t < 15, false);
        fmgcBits(in, false, false, t > 16 && t < 17, false);
        break;
      }
      case 4: {  // rudder trim
        cruise(in, 250.0, 10000.0, 3.0);
        for (auto& i : in) {
          auto& d = i.discrete_inputs;
          d.rudder_trim_switch_left = (t > 1 && t < 6) || (t > 10 && t < 34) || (t > 40 && t < 41);
          d.rudder_trim_switch_right = (t > 6 && t < 9) || (t > 40 && t < 41);
          d.rudder_trim_reset_button = (t > 9.5 && t < 9.6) || (t > 35 && t < 36) || (t > 42 && t < 42.1) ||
                                       (t > 47 && t < 47.1);
          d.fac_engaged_from_switch = !(t > 52 && t < 55);
          i.analog_inputs.rudder_trim_position_deg = 3.0 * j + 0.1 * t;
        }
        ap(in, t > 43 && t < 51, t > 45 && t < 47);
        radioHeight(in, t < 48 ? 300.0 : 150.0);
        elac(in, !(t > 49 && t < 50), true, false, 12.0 * std::sin(t));
        break;
      }
      case 5: {  // yaw damper
        cruise(in, 220.0, 8000.0, 3.0);
        const bool elacOk = t > 15;
        elac(in, elacOk, elacOk && t > 20, false, 30.0 * std::sin(2.0 * t) + j);
        inertial(in, 3.0, 0.0, 15.0 * std::sin(3.0 * t), 0.02, 1.0, 0.0, 0.0, 220.0, 0.0);
        for (auto& i : in) {
          i.discrete_inputs.yaw_damper_has_hyd_press = !(t > 8 && t < 10);
          i.analog_inputs.yaw_damper_position_deg = j;
        }
        in[1].discrete_inputs.yaw_damper_opp_engaged = !(t > 5 && t < 12);
        in[0].discrete_inputs.elac_1_healthy = !(t > 24 && t < 26);
        in[1].discrete_inputs.elac_2_healthy = !(t > 25 && t < 27);
        break;
      }
      case 6: {  // travel limiter
        cruise(in, 0.0, 5000.0, 2.0);
        const double cas = t < 20 ? 25.0 * t : 500.0 - 25.0 * (t - 20);
        airData(in, cas, cas / 600.0, 2.0, 5000.0);
        for (auto& i : in) {
          i.discrete_inputs.rudder_travel_lim_actuator_healthy = !(t > 30 && t < 33);
          i.analog_inputs.rudder_travel_lim_position_deg = 10.0 + 5.0 * j;
        }
        in[1].discrete_inputs.rudder_travel_lim_opp_engaged = !(t > 34 && t < 37);
        break;
      }
      case 7: {  // speeds: weight words, altitude sweep, speed brakes, gear down
        const double alt = t < 25 ? 1800.0 * t : 45000.0 - 1800.0 * (t - 25);
        cruise(in, 280.0, alt, 2.0);
        airData(in, 280.0 - alt / 400.0, 0.4 + alt / 60000.0, 2.0 + j, alt);
        weight(in, 120000.0 + 4000.0 * t, 0.2 + 0.004 * t + 0.01 * j,
               t > 10 && t < 14 ? kNcd : (t > 14 && t < 16 ? kFw : kNo));
        for (auto& i : in) {
          if (t > 10 && t < 16) i.bus_inputs.fmgc_own_bus.fac_weight_lbs = no(150000.0);
          i.bus_inputs.elac_1_bus.speedbrake_extension_deg = {t > 30 && t < 40 ? kNo : kNcd,
                                                              static_cast<float>(-6.0 * (t - 30))};
          i.bus_inputs.elac_2_bus.speedbrake_extension_deg = no(t > 40 ? 10.0 - 2.0 * (t - 40) : 0.0);
        }
        config(in, static_cast<int>(t / 8) % 6);
        if (t > 42) gear(in, false, false, true);
        break;
      }
      case 8: {  // source fallbacks
        cruise(in, 200.0, 3000.0, 3.0);
        for (auto& i : in) {
          auto& b = i.bus_inputs;
          auto& d = i.discrete_inputs;
          if (t > 2 && t < 8) b.lgciu_own_bus.discrete_word_1.SSM = kFw;  // opposite FAC's copy
          if (t > 4 && t < 6) {
            setBit(b.fac_opp_bus.discrete_word_5, 18, true);
            setBit(b.fac_opp_bus.discrete_word_5, 20, true);
          }
          if (t > 6 && t < 8) b.fac_opp_bus.discrete_word_5.SSM = kFw;  // all LGCIU lost
          if ((t > 9 && t < 9.5) || (t > 10 && t < 12)) d.nose_gear_pressed = true;  // nose disagreement
          if (t > 13 && t < 19) b.sfcc_own_bus.slat_flap_system_status_word.SSM = kFw;
          if (t > 13 && t < 15) setBit(b.fac_opp_bus.discrete_word_5, 11 + static_cast<int>(t * 3) % 5, true);
          if (t > 15 && t < 19) b.fac_opp_bus.discrete_word_5.SSM = kFw;
          if (t > 17 && t < 19) setBit(b.lgciu_own_bus.discrete_word_3, 12, true);  // gear down: CONF FULL default
          if ((t > 20 && t < 20.5) || (t > 21 && t < 23)) d.slats_extended = false;  // agrees with bit 12 = 0
          if (t > 24 && t < 30) b.adr_own_bus.airspeed_computed_kn.SSM = kFw;
          if (t > 26 && t < 28) b.adr_3_bus.aoa_corrected_deg.SSM = kFw;
          if (t > 29 && t < 31) b.adr_opp_bus.airspeed_computed_kn.SSM = kFw;
          if (t > 32 && t < 38) b.ir_own_bus.body_lat_accel_g.SSM = kNcd;
          if (t > 34 && t < 36) b.ir_3_bus.body_yaw_rate_deg_s.SSM = kFt;
          if (t > 37 && t < 39) b.ir_opp_bus.body_yaw_rate_deg_s.SSM = kFw;
          if ((t > 40 && t < 40.3) || (t > 41 && t < 44)) b.fmgc_own_bus.fg_radio_height_ft.SSM = kFw;
          if (t > 42 && t < 43.5) b.fmgc_opp_bus.fg_radio_height_ft.SSM = kFw;
          if (t > 45) {
            // FMGC selection: weight words and every AP combination
            const int k = static_cast<int>((t - 45) / 1.5);
            b.fmgc_own_bus.fm_weight_lbs.SSM = k % 2 ? kNcd : kNo;
            b.fmgc_own_bus.fm_cg_percent.SSM = k % 3 ? kNo : kNcd;
            setBit(b.fmgc_opp_bus.discrete_word_1, 29, true);
            setBit(b.fmgc_own_bus.discrete_word_1, 15, true);
          }
        }
        if (t > 45) {
          const int k = static_cast<int>((t - 45) / 1.5);
          ap(in, k & 1, k & 2);
          if (k & 4) in[1].discrete_inputs.is_unit_1 = true;
        }
        break;
      }
      case 10:  // exact thresholds
        boundaries(in, t);
        break;
      default: {  // SFCC status word bit combinations, configuration at intermediate angles
        cruise(in, 180.0, 2000.0, 4.0);
        const int k = static_cast<int>(t * 2);
        for (auto& i : in) {
          base_arinc_429& w = i.bus_inputs.sfcc_own_bus.slat_flap_system_status_word;
          w = no(0);
          for (int b = 17; b <= 21; b++) setBit(w, b, (k >> (b - 17)) & 1);
          setBit(w, 26, (k >> 5) & 1);
          i.bus_inputs.sfcc_own_bus.flap_actual_position_deg = no(5.0 * t + 20.0 * j);
          i.bus_inputs.sfcc_own_bus.slat_actual_position_deg = no(7.0 * t);
        }
        break;
      }
    }
  }

  /** Inputs held exactly at the thresholds (and just beyond the limits) of the FAC's comparisons. */
  static void boundaries(fac_inputs (&in)[2], double t) {
    if (t < 16) {  // alpha floor flight phase: radio height exactly 100 ft in Landing100ft, then in Takeoff100ft
      cruise(in, 180.0, 3000.0, 5.0);
      config(in, 1);  // alpha floor 12.5 deg
      elac(in, true, true, true);
      const bool takeoff = t >= 8;
      if (takeoff && t < 10) gear(in, true, true, true);
      const double ra = !takeoff ? (t < 2 ? 300.0 : (t < 4 ? 50.0 : 100.0)) : (t < 10 ? 0.0 : (t < 12 ? 100.0 : 50.0));
      radioHeight(in, ra);
      const double alpha = (!takeoff && t >= 4) || (takeoff && t >= 12) ? 14.0 : 5.0;
      airData(in, 180.0, 0.3, alpha, 3000.0);
    } else if (t < 28) {  // filtered alpha exactly at the alpha floor threshold (12.5 deg, CONF 1)
      cruise(in, 180.0, 3000.0, 12.5);
      config(in, 1);
      elac(in, true, true, true);
      radioHeight(in, 300.0);
      airData(in, 180.0, 0.3, 12.5, 3000.0);
    } else if (t < 34) {  // exactly 20000 ft, heavy: VLS from the buffet Mach
      cruise(in, 250.0, 20000.0, 3.0);
      weight(in, 180000.0, 0.25);
      airData(in, 250.0, 0.55, 3.0, 20000.0);
    } else if (t < 40) {  // CAS beyond the trend (660 kt) and sideslip target (1000 kt) limits, with an N1 difference
      cruise(in, t < 37 ? 800.0 : 1200.0, 10000.0, 2.0);
      for (auto& i : in) {
        i.bus_inputs.fmgc_own_bus.n1_left_percent = no(90.0);
        i.bus_inputs.fmgc_own_bus.n1_right_percent = no(40.0);
      }
    } else if (t < 46) {  // CAS exactly on travel limit breakpoints, then exactly 30 kt with a lateral load factor
      cruise(in, t < 42 ? 240.0 : (t < 44 ? 320.0 : 30.0), 5000.0, 2.0);
      inertial(in, 3.0, 0.0, 0.0, 0.05, 1.0, 0.0, 0.0, 200.0, 0.0);
    } else if (t < 50) {  // pitch-pitch: pitch + pitch rate exactly 10 deg, then radio height exactly 20 ft
      cruise(in, 140.0, 0.0, 5.0);
      gear(in, false, false, true);
      const bool first = t < 48;
      inertial(in, first ? 8.0 : 9.0, 0.0, 0.0, 0.0, 1.0, 0.0, 2.0, 140.0, 0.0);
      radioHeight(in, first ? 10.0 : 20.0);
    } else if (t < 70) {  // low energy: deceleration terms beyond their limits; radio height exactly 100 ft, then 120
      cruise(in, 143.0, 1500.0, 3.0);
      config(in, 4);
      weight(in, 160000.0, 0.3);
      const double alpha = t < 56 ? 3.0 : 10.3;
      const double nx = t < 56 ? -0.3 : (t < 62 ? 0.3 : 0.0);
      const double cas = t < 62 ? 143.0 : 120.0;
      airData(in, cas, 0.22, alpha, 1500.0);
      inertial(in, 2.0, 0.0, 0.0, 0.0, 1.0, nx, 0.0, cas, 0.0);
      radioHeight(in, t < 62 ? 500.0 : (t < 66 ? 100.0 : 120.0));
    } else if (t < 80) {  // auto trim: radio height exactly 200 ft, then ELAC 2 with bit 26 off / unhealthy
      cruise(in, 250.0, 3000.0, 3.0);
      ap(in, true, false);
      radioHeight(in, 200.0);
      elac(in, t < 73, true, false, 10.0);
      for (auto& i : in) {
        if (t >= 73 && t < 76) setBit(i.bus_inputs.elac_2_bus.discrete_status_word_1, 26, false);
        if (t >= 76) i.discrete_inputs.elac_2_healthy = false;
      }
    } else {  // yaw damper: ELAC 2 with only bit 20 / only bit 26, then yaw rate steps with no ELAC order
      cruise(in, 220.0, 8000.0, 3.0);
      elac(in, false, t < 88, false, 8.0);
      for (auto& i : in) {
        base_arinc_429& w = i.bus_inputs.elac_2_bus.discrete_status_word_1;
        if (t < 84) setBit(w, 26, false);
        if (t >= 84 && t < 88) setBit(w, 20, false);
      }
      inertial(in, 3.0, 0.0, std::fmod(t, 1.0) < 0.5 ? 20.0 : -20.0, 0.0, 1.0, 0.0, 0.0, 220.0, 0.0);
    }
  }

  Rng rng_;
  int phase_ = 0;
  double t_ = 0, jitter_ = 0;
  long episodes_ = 0;
};

// --- --failures -----------------------------------------------------------------------------------------------------

class FailureScenario {
 public:
  explicit FailureScenario(uint64_t seed) : rng_(seed * 104729 + 11) {}
  long episodes() const { return episodes_; }

  void apply(fac_inputs (&in)[2], double dt) {
    constexpr int kWords = sizeof(base_fac_bus_inputs) / sizeof(base_arinc_429);
    static_assert(sizeof(base_fac_bus_inputs) % sizeof(base_arinc_429) == 0);
    if (left_ <= 0) {
      left_ = rng_.u(0.1, 20.0);
      if (rng_.chance(0.3)) left_ = 0;  // a gap
      active_ = left_ > 0;
      word_ = rng_.irand(kWords);
      which_ = rng_.irand(3);
      ssm_ = static_cast<uint32_t>(rng_.irand(4));
      dataMode_ = rng_.irand(8);
      scale_ = rng_.u(-2.0, 3.0);
      bits_ = static_cast<float>(static_cast<uint32_t>(rng_.u(0, 1 << 29)) & ~0x3FFU);
      episodes_ += active_;
      if (!active_) left_ = rng_.u(0.5, 10.0);
    }
    left_ -= dt > 0 && std::isfinite(dt) ? dt : 0.02;
    if (!active_) {
      if (left_ <= 0) left_ = 0;
      return;
    }
    for (int e = 0; e < 2; e++) {
      if (which_ != 2 && which_ != e) continue;
      base_arinc_429* w = reinterpret_cast<base_arinc_429*>(&in[e].bus_inputs) + word_;
      w->SSM = ssm_;
      switch (dataMode_) {
        case 0: w->Data = static_cast<float>(kNaN); break;
        case 1: w->Data = static_cast<float>(scale_ > 0 ? kInf : -kInf); break;
        case 2: w->Data = 0.0F; break;
        case 3: w->Data = static_cast<float>(scale_ * 1e6); break;
        case 4: w->Data = bits_; break;
        case 5: w->Data = static_cast<float>(w->Data * scale_ + 10.0 * scale_); break;
        default: break;  // SSM only
      }
    }
  }

 private:
  Rng rng_;
  double left_ = 0, scale_ = 1;
  bool active_ = false;
  int word_ = 0, which_ = 0, dataMode_ = 0;
  uint32_t ssm_ = 0;
  float bits_ = 0;
  long episodes_ = 0;
};

// --- --discretes ----------------------------------------------------------------------------------------------------

class DiscreteScenario {
 public:
  explicit DiscreteScenario(uint64_t seed) : rng_(seed * 15485863 + 7) {}
  long episodes() const { return episodes_; }

  void apply(fac_inputs (&in)[2], double dt) {
    constexpr int kDiscretes = sizeof(base_fac_discrete_inputs);
    if (left_ <= 0) {
      left_ = rng_.u(0.05, 15.0);
      kind_ = rng_.irand(4);
      index_ = rng_.irand(kind_ == 0 ? kDiscretes : (kind_ == 1 ? 3 : 7));
      which_ = rng_.irand(3);
      value_ = rng_.chance(0.5);
      analog_ = rng_.chance(0.1) ? kNaN : rng_.u(-35.0, 35.0);
      bit_ = 11 + rng_.irand(19);
      episodes_++;
    }
    left_ -= dt > 0 && std::isfinite(dt) ? dt : 0.02;
    for (int e = 0; e < 2; e++) {
      if (which_ != 2 && which_ != e) continue;
      fac_inputs& i = in[e];
      if (kind_ == 0) {
        reinterpret_cast<bool*>(&i.discrete_inputs)[index_] = value_;
      } else if (kind_ == 1) {
        reinterpret_cast<double*>(&i.analog_inputs)[index_] = analog_;
      } else {
        base_arinc_429* words[7] = {&i.bus_inputs.elac_1_bus.discrete_status_word_1,
                                    &i.bus_inputs.elac_2_bus.discrete_status_word_1,
                                    &i.bus_inputs.elac_1_bus.discrete_status_word_2,
                                    &i.bus_inputs.fac_opp_bus.discrete_word_5,
                                    &i.bus_inputs.fmgc_own_bus.discrete_word_1,
                                    &i.bus_inputs.sfcc_own_bus.slat_flap_system_status_word,
                                    &i.bus_inputs.lgciu_own_bus.discrete_word_2};
        setBit(*words[index_], bit_, value_);
        if (kind_ == 3) setBit(*words[index_], bit_ == 29 ? 11 : bit_ + 1, !value_);
      }
    }
  }

 private:
  Rng rng_;
  double left_ = 0, analog_ = 0;
  int kind_ = 0, index_ = 0, which_ = 0, bit_ = 11;
  bool value_ = false;
  long episodes_ = 0;
};

// --- --power --------------------------------------------------------------------------------------------------------

class PowerScenario {
 public:
  explicit PowerScenario(uint64_t seed) : rng_(seed * 49979687 + 13) {}
  long episodes() const { return episodes_; }

  void apply(fac_inputs (&in)[2], double dt) {
    const double step = dt > 0 && std::isfinite(dt) ? dt : 0.02;
    if (!active_) {
      wait_ -= step;
      if (wait_ > 0) return;
      start();
    }
    bool cut = true;
    if (kind_ == 0) {  // frames
      if (--frames_ <= 0) end();
    } else if (kind_ == 1) {  // seconds
      seconds_ -= step;
      if (seconds_ <= 0) end();
    } else if (gap_ > 0) {  // flicker: one frame off, 1-3 on
      cut = false;
      gap_--;
    } else {
      gap_ = 1 + rng_.irand(3);
      if (--frames_ <= 0) end();
    }
    if (!cut) return;
    for (int e = 0; e < 2; e++) {
      if (which_ == 2 || which_ == e) in[e].sim_data.computer_running = false;
    }
  }

 private:
  void start() {
    active_ = true;
    episodes_++;
    which_ = rng_.irand(3);
    const int k = rng_.irand(5);
    kind_ = k <= 1 ? 0 : (k <= 3 ? 1 : 2);
    frames_ = k == 0 ? 1 : (k == 1 ? 2 + rng_.irand(9) : 1 + rng_.irand(20));
    seconds_ = k == 2 ? rng_.u(0.2, 5.0) : rng_.u(5.0, 60.0);
    gap_ = 0;
  }
  void end() {
    active_ = false;
    wait_ = rng_.u(5.0, 120.0);
  }

  Rng rng_;
  bool active_ = false;
  double wait_ = 3.0, seconds_ = 0;
  int kind_ = 0, frames_ = 0, gap_ = 0, which_ = 0;
  long episodes_ = 0;
};

// --- --sim ----------------------------------------------------------------------------------------------------------

class SimScenario {
 public:
  explicit SimScenario(uint64_t seed) : rng_(seed * 32452843 + 17) {}
  long episodes() const { return episodes_; }

  void apply(fac_inputs (&in)[2], double dt) {
    if (left_ <= 0) {
      left_ = rng_.u(0.05, 8.0);
      kind_ = rng_.irand(7);
      value_ = kind_ == 0 ? 0.0 : (kind_ == 1 ? 1e-6 : rng_.u(0.2, 2.0));
      episodes_++;
    }
    left_ -= dt > 0 && std::isfinite(dt) ? dt : 0.02;
    for (auto& i : in) {
      if (kind_ <= 2) {
        i.time.dt = value_;
      } else if (kind_ == 3) {
        i.time.simulation_time += 1000.0;
      } else if (kind_ == 4) {
        i.sim_data.slew_on = true;
      } else if (kind_ == 5) {
        i.sim_data.pause_on = true;
      } else {
        i.sim_data.tracking_mode_on_override = true;
      }
    }
  }

 private:
  Rng rng_;
  double left_ = 2.0, value_ = 0;
  int kind_ = 0;
  long episodes_ = 0;
};

}  // namespace fac_scenarios
