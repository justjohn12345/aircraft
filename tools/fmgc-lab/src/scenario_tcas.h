// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic TCAS RA episodes and "situations" injected on top of a replay. Ported from the V/S-TCAS agent's scratch
// driver (~/fmgc-lab/scratch-drivers/vs_tcas_scratch, Phase 2), which caught mutations the other scenarios missed:
//   TcasScenario: TCAS sensitivity level TA/RA (sometimes lost, sometimes not normal), RA word with random combined
//     control / sign / rate bits (RA active = bit 20 most of the time), TA display, the FCU altitude around the
//     aircraft (and moved by 200-300 ft during the RA), sometimes a fixed radio height;
//   SituationScenario: a held Mach cruise (cruise phase, Mach requested, CAS on the speed target; the only way the ALT
//     soft mode and its timers see long periods), VMAX brought within 3.5-5.5 kt of the CAS and CAS steps of 6-14 kt
//     while ALT soft is active, V/S knob pulses (word 1 bits 16/19), both engines stopped on the ground.
// Physically inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "FmgcComputer_types.h"

struct TcasScenario {
  explicit TcasScenario(uint64_t seed) : rng_(seed * 7919 + 13) {}

  long episodes = 0;

  void apply(fmgc_inputs& in, double dt, const fmgc_outputs& prev) {
    float lastH = prev.logic.altitude_indicated_ft.Data;
    if (remaining_ <= 0 && clear_ <= 0) {
      if (u(0, 1) < dt / 40.0 || episodes == 0) {
        episodes++;
        remaining_ = u(0, 1) < 0.3 ? u(0.5, 4) : u(4, 40);
        clear_ = u(0, 25);
        sensData_ = static_cast<float>((1u << 25) | (1u << 26) | (u(0, 1) < 0.08 ? (1u << 24) : 0u));
        sensSsm_ = u(0, 1) < 0.05 ? 0u : 3u;
        newRa();
        sensLost_ = false;
        ta_ = u(0, 1) < 0.85;
        fcuAlt_ = u(0, 1) < 0.6;
        fcuAltValue_ = std::round((lastH + static_cast<float>(u(-2500, 2500))) / 100.0F) * 100.0F;
        if (fcuAltValue_ < 100.0F) {
          fcuAltValue_ = 100.0F;
        }
        ra_ = u(0, 1) < 0.3;
        raValue_ = static_cast<float>(u(0, 3000));
      }
    }
    if (remaining_ <= 0 && clear_ <= 0) {
      return;
    }
    bool raPhase = remaining_ > 0;
    if (u(0, 1) < 0.15 * dt) {
      fcuAltValue_ += std::round(static_cast<float>(u(-1, 1) < 0 ? -u(200, 300) : u(200, 300)) / 10.0F) * 10.0F;
    }
    if (raPhase) {
      remaining_ -= dt;
      if (u(0, 1) < 0.3 * dt) {
        newRa();
      }
      if (u(0, 1) < (sensLost_ ? 0.5 : 0.08) * dt) {
        sensLost_ = !sensLost_;
      }
    } else {
      clear_ -= dt;
    }
    in.bus_inputs.tcas_bus.sensitivity_level.SSM = sensSsm_;
    in.bus_inputs.tcas_bus.sensitivity_level.Data = sensLost_ ? static_cast<float>(1u << 24) : sensData_;
    in.bus_inputs.tcas_bus.vertical_resolution_advisory.SSM = 3u;
    in.bus_inputs.tcas_bus.vertical_resolution_advisory.Data = raPhase ? raData_ : 0.0F;
    in.discrete_inputs.tcas_ta_display = ta_;
    if (fcuAlt_) {
      in.bus_inputs.fcu_bus.selected_alt_ft.SSM = 3u;
      in.bus_inputs.fcu_bus.selected_alt_ft.Data = fcuAltValue_;
    }
    if (ra_) {
      in.bus_inputs.ra_own_bus.radio_height_ft.SSM = 3u;
      in.bus_inputs.ra_own_bus.radio_height_ft.Data = raValue_;
      in.bus_inputs.ra_opp_bus.radio_height_ft.SSM = 3u;
      in.bus_inputs.ra_opp_bus.radio_height_ft.Data = raValue_;
    }
  }

 private:
  void newRa() {
    uint32_t d = 0;
    if (u(0, 1) < 0.85) {
      d |= 1u << 19;  // RA active (bit 20)
    }
    if (u(0, 1) < 0.5) {
      d |= 1u << 17;
    }
    if (u(0, 1) < 0.5) {
      d |= 1u << 18;
    }
    if (u(0, 1) < 0.5) {
      d |= 1u << 16;  // sign
    }
    d |= (static_cast<uint32_t>(u(0, 64)) & 63u) << 10;
    raData_ = static_cast<float>(d);
  }
  double u(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng_); }

  std::mt19937_64 rng_;
  double remaining_ = 0, clear_ = 0;
  float sensData_ = 0, raData_ = 0, fcuAltValue_ = 0, raValue_ = 0;
  uint32_t sensSsm_ = 3;
  bool sensLost_ = false, ta_ = false, fcuAlt_ = false, ra_ = false;
};

struct SituationScenario {
  explicit SituationScenario(uint64_t seed) : rng_(seed * 104729 + 7) {}

  bool cruiseActive() const { return cruiseT_ > 0; }

  void apply(fmgc_inputs& in, double dt, const fmgc_outputs& prev) {
    bool soft = prev.ap_fd_logic.alt_soft_mode_active;
    float cas = prev.logic.adr_computation_data.airspeed_computed_kn.Data;
    // stable Mach cruise: cruise phase, Mach mode, CAS on the speed target
    if (cruiseT_ <= 0 && prev.ap_fd_logic.longitudinal_modes.alt_hold_active && prev.logic.athr_own_engaged &&
        (u(0, 1) < dt / 30.0 || !started_)) {
      started_ = true;
      cruiseT_ = u(130, 400);
      cruiseStart_ = true;
      fcu_ = in.bus_inputs.fcu_bus;
      uint32_t w = static_cast<uint32_t>(fcu_.fcu_discrete_word_1.Data);
      w &= ~((1u << 12) | (1u << 14) | (1u << 15) | (1u << 18) | (1u << 22));
      fcu_.fcu_discrete_word_1.Data = static_cast<float>(w);
    }
    if (cruiseT_ > 0) {
      cruiseT_ -= dt;
      in.bus_inputs.fcu_bus = fcu_;
      in.fms_inputs.fms_flight_phase = fmgc_flight_phase::Cruise;
      in.fms_inputs.fms_mach_mode_activate = cruiseStart_ || !prev.ap_fd_logic.mach_control_active;
      cruiseStart_ = false;
      float target = static_cast<float>(prev.ap_fd_logic.spd_target_kts + u(-1.5, 1.5));
      in.bus_inputs.adr_own_bus.airspeed_computed_kn.Data = target;
      in.bus_inputs.adr_opp_bus.airspeed_computed_kn.Data = target;
      in.bus_inputs.adr_3_bus.airspeed_computed_kn.Data = target;
    }
    if (vmaxT_ <= 0 && casT_ <= 0 && soft && u(0, 1) < dt / 60.0) {
      vmaxT_ = u(0.5, 3);
      vmax_ = cas + static_cast<float>(u(3.5, 5.5));
    }
    if (casT_ <= 0 && vmaxT_ <= 0 && soft && u(0, 1) < dt / 60.0) {
      casT_ = u(2, 12);
      casOff_ = static_cast<float>(u(0, 1) < 0.5 ? u(-14, -6) : u(6, 14));
    }
    if (knobT_ <= 0 && cruiseT_ <= 0 && u(0, 1) < dt / 20.0) {
      knobT_ = u(0.05, 0.5);
      knobBit_ = u(0, 1) < 0.5 ? 16 : 19;
    }
    if (engT_ <= 0 && prev.logic.on_ground && u(0, 1) < dt / 60.0) {
      engT_ = u(2, 20);
    }
    if (vmaxT_ > 0) {
      vmaxT_ -= dt;
      in.bus_inputs.fac_own_bus.v_max_kn.Data = vmax_;
      in.bus_inputs.fac_opp_bus.v_max_kn.Data = vmax_;
    }
    if (casT_ > 0) {
      casT_ -= dt;
      in.bus_inputs.adr_own_bus.airspeed_computed_kn.Data += casOff_;
      in.bus_inputs.adr_opp_bus.airspeed_computed_kn.Data += casOff_;
      in.bus_inputs.adr_3_bus.airspeed_computed_kn.Data += casOff_;
    }
    if (knobT_ > 0) {
      knobT_ -= dt;
      uint32_t w = static_cast<uint32_t>(in.bus_inputs.fcu_bus.fcu_discrete_word_1.Data);
      w |= 1u << (knobBit_ - 1);
      in.bus_inputs.fcu_bus.fcu_discrete_word_1.Data = static_cast<float>(w);
    }
    if (engT_ > 0) {
      engT_ -= dt;
      in.discrete_inputs.eng_own_stop = true;
      in.discrete_inputs.eng_opp_stop = true;
    }
  }

 private:
  double u(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng_); }
  std::mt19937_64 rng_;
  double vmaxT_ = 0, casT_ = 0, knobT_ = 0, engT_ = 0, cruiseT_ = 0;
  bool cruiseStart_ = false, started_ = false;
  base_fcu_bus fcu_{};
  float vmax_ = 0, casOff_ = 0;
  int knobBit_ = 16;
};
