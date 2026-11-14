// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic autothrust episodes injected on top of a replay, for differential checks of the autothrust modes: the
// recordings fly with the levers in CL and both engines running, and the input fuzzer rarely produces coherent
// thrust-lever situations. Each 15–120 s episode independently picks:
//   - a thrust lever pattern: both levers at one angle, asymmetric (one in the CL detent), independent jumps, or a slow
//     sweep, over angles at and around every band edge the autothrust uses (0, 24, 25, 26, 34, 35, 36, 44), reverse
//     included; sometimes a lever word with a non-normal SSM (the code reads both the validated and the raw angle);
//   - FLEX temperature set or unset on each FADEC;
//   - one engine out (eng_own_stop xor eng_opp_stop), sometimes toggling;
//   - alpha floor from the FAC (discrete word 5 bit 29 on both FACs), in one or more windows;
//   - a TCAS RA (TCAS in TA/RA above 900 ft RA, RA word with bit 20 and a V/S rate around the 500 fpm threshold, climb
//     or descent, corrective or preventive);
//   - A/THR pushbutton presses and instinctive disconnects;
//   - FAC VMAX around IAS + 20 kt (the climbing-RA LVR CLB condition);
//   - the thrust reduction altitude around the current altitude, sometimes in the go-around phase;
//   - the opposite FMGC in control (unit 2 with the opposite AP/FD/A/THR engaged) with its THR/SPEED/RETARD ATS bits.
// Physically inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "FmgcComputer_types.h"

struct AthrScenario {
  enum class Levers { Recorded, Same, Asymmetric, Independent, Sweep };

  std::mt19937_64 rng;
  double te = 1e9, epLen = 0;

  Levers levers = Levers::Recorded;
  float tlaOwn = 25, tlaOpp = 25, sweepFrom = 0, sweepTo = 0, sweepRate = 1;
  double nextLeverChange = 0;
  int ssmOwn = 3, ssmOpp = 3;

  int flexOwn = -1, flexOpp = -1;  // -1 recorded, 0 unset (NCD), 1 set
  int oneEngineOut = -1;           // -1 recorded, 0 both running, 1 own stopped, 2 opp stopped
  double engineToggleAt = -1;

  double afStart[3]{-1, -1, -1}, afLen[3]{0, 0, 0};

  bool tcas = false;
  double raStart = 0, raLen = 0;
  uint32_t raWord = 0;

  double athrPushAt[3]{-1, -1, -1}, instinctiveAt = -1;

  double vmaxOffset = 0;
  bool vmax = false;

  bool thrRed = false, goAround = false;
  double thrRedOffset = 0;
  bool thrRedPinned = false;
  double thrRedFt = 0;

  bool oppControl = false;
  uint32_t oppAts = 0;
  double nextOppChange = 0;

  explicit AthrScenario(uint64_t seed) : rng(seed) {}

  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  bool chance(double p) { return u(0, 1) < p; }
  static uint32_t word(float data) { return data >= 0.0F && data < 4294967296.0F ? static_cast<uint32_t>(data) : 0u; }

  float leverAngle() {
    static const float angles[]{-20.0F, -5.0F, 0.0F, 0.5F, 10.0F, 23.9F, 24.0F, 24.5F, 25.0F, 25.5F, 26.0F, 26.1F,
                                30.0F, 33.9F, 34.0F, 34.5F, 35.0F, 35.5F, 36.0F, 36.1F, 40.0F, 44.0F, 44.1F, 45.0F};
    return angles[rng() % (sizeof(angles) / sizeof(angles[0]))];
  }

  void newLevers() {
    switch (levers) {
      case Levers::Same:
        tlaOwn = tlaOpp = leverAngle();
        break;
      case Levers::Asymmetric: {
        static const float detent[]{24.5F, 25.0F, 25.5F};
        float a = detent[rng() % 3];
        float b = leverAngle();
        if (chance(0.5)) {
          tlaOwn = a, tlaOpp = b;
        } else {
          tlaOwn = b, tlaOpp = a;
        }
        break;
      }
      case Levers::Independent:
        tlaOwn = leverAngle();
        tlaOpp = chance(0.3) ? tlaOwn : leverAngle();
        break;
      default:
        break;
    }
    nextLeverChange = te + u(1, 12);
  }

  void newEpisode() {
    te = 0;
    epLen = u(15, 120);

    int l = static_cast<int>(rng() % 6);
    levers = l == 0 ? Levers::Recorded : l == 1 ? Levers::Same : l == 2 ? Levers::Asymmetric
             : l == 3 ? Levers::Independent : l == 4 ? Levers::Sweep : Levers::Same;
    if (levers == Levers::Sweep) {
      sweepFrom = static_cast<float>(u(-5, 47));
      sweepTo = static_cast<float>(u(-5, 47));
      sweepRate = static_cast<float>(u(0.5, 6));
    }
    newLevers();
    ssmOwn = chance(0.12) ? static_cast<int>(rng() % 3) : 3;
    ssmOpp = chance(0.12) ? static_cast<int>(rng() % 3) : 3;

    auto flex = [&]() { return chance(0.4) ? -1 : (chance(0.5) ? 1 : 0); };
    flexOwn = flex();
    flexOpp = chance(0.6) ? flexOwn : flex();

    oneEngineOut = chance(0.55) ? -1 : static_cast<int>(rng() % 3);
    engineToggleAt = chance(0.25) ? u(0, epLen) : -1;

    for (int i = 0; i < 3; i++) {
      bool on = i == 0 ? chance(0.35) : chance(0.4) && afStart[i - 1] >= 0;
      afStart[i] = on ? (i == 0 ? u(0, epLen * 0.6) : afStart[i - 1] + afLen[i - 1] + u(0.1, 8)) : -1;
      afLen[i] = chance(0.3) ? u(0.05, 0.5) : u(0.5, 15);
    }

    tcas = chance(0.3);
    if (tcas) {
      raStart = u(0.5, epLen * 0.5);
      raLen = u(2, 40);
      static const int rates[]{0, 2, 3, 3, 4, 8, 15};  // x100 fpm (+200 margin when not 0): 0, 400, 500, 600, 1000, 1700
      uint32_t rate = static_cast<uint32_t>(rates[rng() % 7]);
      uint32_t control = static_cast<uint32_t>(4 + rng() % 3);  // 4, 5 corrective, 6 preventive
      bool descend = chance(0.4);
      raWord = (rate << 10) | (descend ? 1u << 16 : 0u) | (control << 17);
    }

    for (double& at : athrPushAt) {
      at = chance(0.3) ? u(0, epLen) : -1;
    }
    instinctiveAt = chance(0.15) ? u(0, epLen) : -1;

    vmax = chance(0.3);
    static const double vmaxOffsets[]{-5, 10, 19.5, 20, 20.5, 30, 60};
    vmaxOffset = vmaxOffsets[rng() % 7];

    thrRed = chance(0.35);
    thrRedPinned = chance(0.5);
    thrRedOffset = chance(0.3) ? 0.0 : u(-1500, 1500);
    thrRedFt = -1;
    goAround = chance(0.3);

    oppControl = chance(0.12);
    oppAts = 0;
    nextOppChange = 0;
  }

  void apply(fmgc_inputs& in, double dt) {
    if (te >= epLen) {
      newEpisode();
    }
    te += dt;
    auto& bus = in.bus_inputs;
    auto& di = in.discrete_inputs;
    auto window = [&](double start, double len) { return start >= 0 && te >= start && te < start + len; };
    auto setBit = [](base_arinc_429& w, int b, bool v) {
      uint32_t x = word(w.Data);
      x = v ? x | (1u << (b - 1)) : x & ~(1u << (b - 1));
      w.Data = static_cast<float>(x);
    };

    // thrust levers
    if (levers != Levers::Recorded) {
      if (levers == Levers::Sweep) {
        float a = sweepFrom + static_cast<float>((sweepTo > sweepFrom ? 1 : -1) * sweepRate * te);
        a = sweepTo > sweepFrom ? std::fmin(a, sweepTo) : std::fmax(a, sweepTo);
        tlaOwn = a;
        tlaOpp = a + (te > epLen / 2 ? 0.0F : 1.0F);
      } else if (te >= nextLeverChange) {
        newLevers();
      }
      bus.fadec_own_bus.selected_tla_deg.Data = tlaOwn;
      bus.fadec_opp_bus.selected_tla_deg.Data = tlaOpp;
      bus.fadec_own_bus.selected_tla_deg.SSM = static_cast<uint32_t>(ssmOwn);
      bus.fadec_opp_bus.selected_tla_deg.SSM = static_cast<uint32_t>(ssmOpp);
    }

    // FLEX temperature
    auto flex = [](base_arinc_429& w, int mode) {
      if (mode >= 0) {
        w.SSM = mode ? 3u : 1u;
        w.Data = mode ? 45.0F : 0.0F;
      }
    };
    flex(bus.fadec_own_bus.selected_flex_temp_deg, flexOwn);
    flex(bus.fadec_opp_bus.selected_flex_temp_deg, flexOpp);

    // one engine out
    if (oneEngineOut >= 0) {
      int mode = oneEngineOut;
      if (engineToggleAt >= 0 && te >= engineToggleAt) {
        mode = mode == 0 ? 1 : 0;
      }
      di.eng_own_stop = mode == 1;
      di.eng_opp_stop = mode == 2;
    }

    // alpha floor from the FACs
    bool af = window(afStart[0], afLen[0]) || window(afStart[1], afLen[1]) || window(afStart[2], afLen[2]);
    if (afStart[0] >= 0) {
      setBit(bus.fac_own_bus.discrete_word_5, 29, af);
      setBit(bus.fac_opp_bus.discrete_word_5, 29, af);
    }

    // TCAS RA
    if (tcas) {
      auto& sens = bus.tcas_bus.sensitivity_level;
      sens.SSM = 3;
      uint32_t s = word(sens.Data);
      s = (s | (1u << 25) | (1u << 26)) & ~(1u << 24);  // bits 26, 27 (TA/RA), not 25 (STBY)
      sens.Data = static_cast<float>(s);
      for (auto* ra : {&bus.ra_own_bus.radio_height_ft, &bus.ra_opp_bus.radio_height_ft}) {
        ra->SSM = 3;
        ra->Data = std::fmax(ra->Data, 2500.0F);
      }
      di.tcas_ta_display = true;
      auto& adv = bus.tcas_bus.vertical_resolution_advisory;
      adv.SSM = 3;
      adv.Data = static_cast<float>(window(raStart, raLen) ? raWord : 0u);
    }

    // A/THR pushbutton (0.2 s presses) and instinctive disconnect (0.5 s)
    for (double at : athrPushAt) {
      if (window(at, 0.2)) {
        di.fcu_athr_button = true;
      }
    }
    if (window(instinctiveAt, 0.5)) {
      di.athr_instinctive_disc = true;
    }

    // VMAX relative to IAS
    if (vmax) {
      float ias = bus.adr_own_bus.airspeed_computed_kn.Data;
      for (auto* w : {&bus.fac_own_bus.v_max_kn, &bus.fac_opp_bus.v_max_kn}) {
        w->SSM = 3;
        w->Data = ias + static_cast<float>(vmaxOffset);
      }
    }

    // thrust reduction altitude around the current altitude (pinned at episode start, or following the aircraft)
    if (thrRed) {
      double h = bus.adr_own_bus.altitude_corrected_1_ft.Data;
      if (thrRedFt < 0 || !thrRedPinned) {
        thrRedFt = h + thrRedOffset;
      }
      in.fms_inputs.thrust_reduction_alt_ft = thrRedFt;
      if (goAround) {
        in.fms_inputs.fms_flight_phase = fmgc_flight_phase::Goaround;
      }
    }

    // opposite FMGC in control, with its own thrust mode bits
    if (oppControl) {
      di.is_unit_1 = false;
      di.ap_opp_engaged = true;
      di.fd_opp_engaged = true;
      di.athr_opp_engaged = true;
      if (te >= nextOppChange) {
        oppAts = 0;
        int pick = static_cast<int>(rng() % 5);  // none, SPEED, RETARD, THR, two at once
        if (pick == 1 || pick == 4) oppAts |= 1u << 17;
        if (pick == 2) oppAts |= 1u << 19;
        if (pick == 3 || pick == 4) oppAts |= 1u << 20;
        nextOppChange = te + u(0.3, 6);
      }
      auto& w = bus.fmgc_opp_bus.ats_discrete_word;
      uint32_t x = word(w.Data) & ~((1u << 17) | (1u << 19) | (1u << 20));
      w.Data = static_cast<float>(x | oppAts);
    }
  }
};
