// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic sensor and equipment failure episodes injected on top of a replay (after the other scenarios, before the
// fuzzer), for differential checks of the input conditioning (readable/conditioning.cpp): the recordings have almost
// no failures, so most source-selection branches and failure flags would never be exercised. Episodes of 3–60 s; each
// holds 1–4 faults, each with its own start and duration (one frame, 0.05–0.5 s glitches, 4.8–5.2 s around the
// in-flight confirmation, or 1–30 s):
//   - ADR / IR: SSM failure warning, NCD or functional test on own / opposite / 3 alone, in pairs or all three, on one
//     voted label, all voted labels, or a label that does not vote (ADR TAS, V/S, corrected altitudes, static
//     pressure; IR ground speed, magnetic heading, FPA, wind);
//   - RA: own, opposite or both failed / NCD / FT / normal again; radio heights at and around 700 and 900 ft;
//   - ILS: own, opposite or both G/S and/or LOC word failed / NCD; the runway heading word not normal or changed;
//     back beam selected;
//   - FAC: own / opposite / both unhealthy; discrete word 2 yaw damper (bit 11) and rudder trim (bit 13) bits cleared
//     or set in one or both FACs, or engaged in one FAC only; discrete word 5 failed / NCD, all LGCIUs lost (17),
//     all SFCCs lost (22), random flap lever bits (11–15); main gear bits (18/19) set or cleared (ground/flight transitions around the 5 s confirm);
//     VLS / VMAX / VMAN / CG / weight words failed or NCD;
//   - FCU: own / opposite / both unhealthy; EIS discrete word 2 (left and right) STD bit 28 set or cleared, word failed
//     / NCD;
//     this FMGC as unit 2;
//   - engines own / opposite / both stopped or running;
//   - TCAS mode word failed / NCD, STBY / TA / TA-RA bit patterns;
//   - opposite FMGC unhealthy (not read by the conditioning; for the downstream logic).
// Physically inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "FmgcComputer_types.h"

struct ConditioningScenario {
  enum Kind : int {
    Adr = 0,
    Ir,
    Ra,
    RaHeight,
    Ils,
    IlsRunway,
    Backbeam,
    FacHealth,
    FacWord2,
    FacWord5,
    FacGear,
    FacSpeeds,
    Fcu,
    FcuEis,
    Unit2,
    Engines,
    Tcas,
    OppFmgc,
    kKinds
  };
  struct Fault {
    int kind = 0;
    double at = 0, len = 0;
    int mask = 0;     // which sources (bit 0 own, 1 opposite, 2 third)
    int scope = 0;    // kind-specific: which label(s)
    uint32_t ssm = 0;  // SSM to force
    float value = 0;  // kind-specific value
    uint32_t bits = 0;
  };

  std::mt19937_64 rng;
  double te = 1e9, epLen = 0;
  std::vector<Fault> faults;
  long episodes = 0;
  long faultCount[kKinds]{};

  explicit ConditioningScenario(uint64_t seed) : rng(seed) {}

  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  bool chance(double p) { return u(0, 1) < p; }
  int irand(int n) { return static_cast<int>(rng() % static_cast<uint64_t>(n)); }
  template <typename T, size_t N>
  T pick(const T (&a)[N]) {
    return a[rng() % N];
  }
  static uint32_t word(float data) { return data >= 0.0F && data < 4294967296.0F ? static_cast<uint32_t>(data) : 0u; }
  static void setBit(base_arinc_429& w, int b, bool v) {
    uint32_t x = word(w.Data);
    x = v ? x | (1u << (b - 1)) : x & ~(1u << (b - 1));
    w.Data = static_cast<float>(x);
  }
  uint32_t badSsm() {
    static const uint32_t s[]{0, 0, 1, 2};  // failure warning (twice as likely), NCD, functional test
    return pick(s);
  }

  double duration(double dt) {
    double r = u(0, 1);
    if (r < 0.2) {
      return dt * 0.5;  // a single frame
    }
    if (r < 0.4) {
      return u(0.05, 0.5);
    }
    if (r < 0.6) {
      return u(4.8, 5.2);
    }
    return u(1, 30);
  }

  void newEpisode(double dt) {
    te = 0;
    epLen = u(3, 60);
    episodes++;
    faults.clear();
    int n = 1 + irand(4);
    for (int i = 0; i < n; i++) {
      Fault f;
      f.kind = irand(kKinds);
      f.at = chance(0.3) ? 0 : u(0, epLen);
      f.len = duration(dt);
      f.mask = 1 + irand(7);
      f.ssm = badSsm();
      switch (f.kind) {
        case Adr:
          f.scope = irand(7);  // 0..3 one voted label, 4 all voted labels, 5 non-voted labels, 6 everything
          break;
        case Ir:
          f.scope = irand(11);  // 0..7 one voted label, 8 all voted, 9 non-voted, 10 everything
          break;
        case Ra:
          f.mask = 1 + irand(3);
          f.ssm = chance(0.2) ? 3 : badSsm();  // sometimes normal again (after a recorded failure)
          break;
        case RaHeight: {
          static const float h[]{699.0F, 699.9F, 700.0F, 700.1F, 701.0F, 899.0F, 899.9F, 900.0F, 900.1F, 901.0F,
                                 0.0F, 2500.0F};
          f.value = pick(h);
          f.mask = 1 + irand(3);
          f.ssm = chance(0.7) ? 3 : badSsm();
          break;
        }
        case Ils:
          f.mask = 1 + irand(3);
          f.scope = irand(3);  // 0 G/S, 1 LOC, 2 both
          f.ssm = chance(0.2) ? 3 : badSsm();
          break;
        case IlsRunway:
          f.mask = 1 + irand(3);
          f.scope = irand(2);  // 0 SSM not normal, 1 heading changed
          f.value = static_cast<float>(u(0, 360));
          break;
        case FacHealth:
        case Fcu:
          f.mask = 1 + irand(3);
          break;
        case FacWord2:
          f.mask = 1 + irand(3);
          f.scope = irand(6);  // 0 yaw damper cleared, 1 rudder trim cleared, 2 both cleared, 3 both set,
                               // 4/5 yaw damper/rudder trim engaged in the masked FACs only (cleared in the others)
          break;
        case FacWord5:
          f.mask = 1 + irand(3);
          f.scope = irand(5);  // 0 SSM, 1 all LGCIU lost, 2 all SFCC lost, 3 flap lever bits, 4 normal SSM
          f.bits = static_cast<uint32_t>(rng() % 32);
          break;
        case FacGear:
          f.mask = 1 + irand(3);
          f.scope = irand(4);  // 0 both gears set (ground), 1 both cleared (flight), 2 left only, 3 right only
          break;
        case FacSpeeds:
          f.mask = 1 + irand(3);
          f.scope = irand(6);  // 0 VMAX, 1 VLS, 2 VMAN, 3 CG, 4 weight, 5 all
          break;
        case FcuEis:
          f.mask = 1 + irand(3);  // left / right / both
          f.scope = irand(3);     // 0 STD bit cleared, 1 SSM, 2 STD bit set
          break;
        case Engines:
          f.bits = static_cast<uint32_t>(rng() % 4);  // bit 0 own stopped, bit 1 opposite stopped
          break;
        case Tcas:
          f.scope = irand(2);  // 0 SSM, 1 bits
          f.bits = static_cast<uint32_t>(rng() % 8);
          break;
        default:
          break;
      }
      faults.push_back(f);
      faultCount[f.kind]++;
    }
  }

  void apply(fmgc_inputs& in, double dt) {
    if (te >= epLen) {
      newEpisode(dt);
    }
    te += dt;
    auto& b = in.bus_inputs;
    auto& di = in.discrete_inputs;
    for (const Fault& f : faults) {
      if (te < f.at || te >= f.at + f.len) {
        continue;
      }
      switch (f.kind) {
        case Adr: {
          base_adr_bus* adrs[]{&b.adr_own_bus, &b.adr_opp_bus, &b.adr_3_bus};
          for (int i = 0; i < 3; i++) {
            if (!(f.mask & (1 << i))) {
              continue;
            }
            base_adr_bus& a = *adrs[i];
            base_arinc_429* voted[]{&a.airspeed_computed_kn, &a.aoa_corrected_deg, &a.mach, &a.altitude_standard_ft};
            base_arinc_429* other[]{&a.airspeed_true_kn, &a.vertical_speed_ft_min, &a.altitude_corrected_1_ft,
                                    &a.altitude_corrected_2_ft, &a.corrected_average_static_pressure};
            if (f.scope < 4) {
              voted[f.scope]->SSM = f.ssm;
            }
            if (f.scope == 4 || f.scope == 6) {
              for (auto* w : voted) w->SSM = f.ssm;
            }
            if (f.scope >= 5) {
              for (auto* w : other) w->SSM = f.ssm;
            }
          }
          break;
        }
        case Ir: {
          base_ir_bus* irs[]{&b.ir_own_bus, &b.ir_opp_bus, &b.ir_3_bus};
          for (int i = 0; i < 3; i++) {
            if (!(f.mask & (1 << i))) {
              continue;
            }
            base_ir_bus& r = *irs[i];
            base_arinc_429* voted[]{&r.body_yaw_rate_deg_s, &r.body_lat_accel_g, &r.pitch_angle_deg, &r.roll_angle_deg,
                                    &r.heading_true_deg, &r.track_angle_true_deg, &r.inertial_vertical_speed_ft_s,
                                    &r.latitude_deg};
            base_arinc_429* other[]{&r.ground_speed_kn, &r.heading_magnetic_deg, &r.flight_path_angle_deg,
                                    &r.wind_speed_kn, &r.wind_direction_true_deg, &r.longitude_deg};
            if (f.scope < 8) {
              voted[f.scope]->SSM = f.ssm;
            }
            if (f.scope == 8 || f.scope == 10) {
              for (auto* w : voted) w->SSM = f.ssm;
            }
            if (f.scope >= 9) {
              for (auto* w : other) w->SSM = f.ssm;
            }
          }
          break;
        }
        case Ra:
          if (f.mask & 1) b.ra_own_bus.radio_height_ft.SSM = f.ssm;
          if (f.mask & 2) b.ra_opp_bus.radio_height_ft.SSM = f.ssm;
          break;
        case RaHeight:
          for (int i = 0; i < 2; i++) {
            if (f.mask & (1 << i)) {
              base_arinc_429& w = i == 0 ? b.ra_own_bus.radio_height_ft : b.ra_opp_bus.radio_height_ft;
              w.SSM = f.ssm;
              w.Data = f.value;
            }
          }
          break;
        case Ils:
          for (int i = 0; i < 2; i++) {
            if (f.mask & (1 << i)) {
              base_ils_bus& l = i == 0 ? b.ils_own_bus : b.ils_opp_bus;
              if (f.scope != 1) l.glideslope_deviation_deg.SSM = f.ssm;
              if (f.scope != 0) l.localizer_deviation_deg.SSM = f.ssm;
            }
          }
          break;
        case IlsRunway:
          for (int i = 0; i < 2; i++) {
            if (f.mask & (1 << i)) {
              base_ils_bus& l = i == 0 ? b.ils_own_bus : b.ils_opp_bus;
              if (f.scope == 0) {
                l.runway_heading_deg.SSM = f.ssm;
              } else {
                l.runway_heading_deg.Data = f.value;
              }
            }
          }
          break;
        case Backbeam:
          in.fms_inputs.backbeam_selected = true;  // (values, not toggles: gap-fill steps reuse the inputs)
          break;
        case FacHealth:
          if (f.mask & 1) di.fac_own_healthy = false;
          if (f.mask & 2) di.fac_opp_healthy = false;
          break;
        case FacWord2:
          for (int i = 0; i < 2; i++) {
            base_arinc_429& w = i == 0 ? b.fac_own_bus.discrete_word_2 : b.fac_opp_bus.discrete_word_2;
            if (f.scope >= 4) {
              setBit(w, f.scope == 4 ? 11 : 13, (f.mask & (1 << i)) != 0);
            } else if (f.mask & (1 << i)) {
              if (f.scope == 0 || f.scope == 2) setBit(w, 11, false);
              if (f.scope == 1 || f.scope == 2) setBit(w, 13, false);
              if (f.scope == 3) {
                setBit(w, 11, true);
                setBit(w, 13, true);
              }
            }
          }
          break;
        case FacWord5:
          for (int i = 0; i < 2; i++) {
            if (f.mask & (1 << i)) {
              base_arinc_429& w = i == 0 ? b.fac_own_bus.discrete_word_5 : b.fac_opp_bus.discrete_word_5;
              switch (f.scope) {
                case 0: w.SSM = f.ssm; break;
                case 1: setBit(w, 17, true); break;
                case 2: setBit(w, 22, true); break;
                case 3:
                  for (int k = 0; k < 5; k++) setBit(w, 11 + k, (f.bits >> k) & 1u);
                  break;
                default: w.SSM = 3; break;
              }
            }
          }
          break;
        case FacGear:
          for (int i = 0; i < 2; i++) {
            if (f.mask & (1 << i)) {
              base_arinc_429& w = i == 0 ? b.fac_own_bus.discrete_word_5 : b.fac_opp_bus.discrete_word_5;
              setBit(w, 18, f.scope == 0 || f.scope == 2);
              setBit(w, 19, f.scope == 0 || f.scope == 3);
            }
          }
          break;
        case FacSpeeds:
          for (int i = 0; i < 2; i++) {
            if (f.mask & (1 << i)) {
              base_fac_bus& fb = i == 0 ? b.fac_own_bus : b.fac_opp_bus;
              base_arinc_429* w[]{&fb.v_max_kn, &fb.v_ls_kn, &fb.v_man_kn, &fb.center_of_gravity_pos_percent,
                                  &fb.total_weight_lbs};
              if (f.scope < 5) {
                w[f.scope]->SSM = f.ssm;
              } else {
                for (auto* x : w) x->SSM = f.ssm;
              }
            }
          }
          break;
        case Fcu:
          if (f.mask & 1) di.fcu_own_healthy = false;
          if (f.mask & 2) di.fcu_opp_healthy = false;
          break;
        case FcuEis:
          for (int i = 0; i < 2; i++) {
            if (f.mask & (1 << i)) {
              base_arinc_429& w = i == 0 ? b.fcu_bus.eis_discrete_word_2_left : b.fcu_bus.eis_discrete_word_2_right;
              if (f.scope == 0) {
                setBit(w, 28, false);
              } else if (f.scope == 1) {
                w.SSM = f.ssm;
              } else {
                setBit(w, 28, true);
              }
            }
          }
          break;
        case Unit2:
          di.is_unit_1 = false;
          break;
        case Engines:
          di.eng_own_stop = f.bits & 1u;
          di.eng_opp_stop = (f.bits >> 1) & 1u;
          break;
        case Tcas: {
          base_arinc_429& w = b.tcas_bus.sensitivity_level;
          if (f.scope == 0) {
            w.SSM = f.ssm;
          } else {
            w.SSM = 3;
            setBit(w, 25, f.bits & 1u);
            setBit(w, 26, (f.bits >> 1) & 1u);
            setBit(w, 27, (f.bits >> 2) & 1u);
          }
          break;
        }
        case OppFmgc:
          di.fmgc_opp_healthy = false;
          break;
        default:
          break;
      }
    }
  }
};
