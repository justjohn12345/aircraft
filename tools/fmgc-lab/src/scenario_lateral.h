// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic lateral-mode episodes injected on top of a replay (after the approach scenario, if any), for differential
// checks of the lateral modes: the recordings fly NAV almost all the time, so LOC arming by the LOC button, LOC*
// geometry, HDG/TRK knob pushes and pulls, presets, DIR TO, flight plan loss, back-to-back pushes, the opposite FMGC as
// master, and touchdown/roll-out are otherwise rare. Each 10–90 s episode picks a random subset of:
//   - ILS: course = heading + offset (small, large, near 360/0), decaying/oscillating localizer deviation, sometimes
//     invalid, sometimes back-beam selected;
//   - radio height: held near a threshold (30, 50, 100, 400 ft), climbing through 30 ft, or descending;
//   - roll angle (all three IRs);
//   - FCU: HDG/TRK knob push/pull, LOC push (word 2 bits 11/12/13, sometimes twice), HDG-V/S/TRK-FPA toggle (word 1
//     bit 24), selected heading/track valid or not;
//   - FMS: DIR TO, NAV capture condition, lateral flight plan valid, flight phase;
//   - AP/FD: everything disengaged for a while (then re-engaged by the recording);
//   - this FMGC as unit 2 with AP 1 engaged, so it copies random opposite-FMGC lateral mode bits;
//   - wheel spin-up and BSCU validity below 80 ft RA;
//   - on the ground (half the episodes starting below 5 ft RA): AP/FD off, then a TOGA edge with V2 and flaps 1 (SRS
//     takeoff and RWY), runway course within 26° of the heading (sometimes ±360° unwrapped), localizer lost later.
// Physically inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "FmgcComputer_types.h"

struct LateralScenario {
  std::mt19937_64 rng;
  double te = 1e9, epLen = 0;
  // episode parameters
  bool ils = false, locInvalid = false, backbeam = false, raOn = false, rollOn = false, fcuOn = false, fmsOn = false,
       noApFd = false, oppMaster = false, wheels = false, presetToggle = false, trkToggle = false;
  double course = 0, L0 = 0, tauL = 1, w = 0, raMode = 0, ra0 = 0, raRate = 0, roll0 = 0, rollW = 0;
  int phase = 0;
  uint32_t oppW2 = 0, oppW3 = 0;
  double nextFcuIn = 0, fcuLeft = 0, nextFmsIn = 0, toggleIn = 0;
  bool takeoff = false;
  double togaAt = 0, locInvalidAt = -1;
  bool snapHeading = false;
  double hdgInt = 0;
  int fcuBit = 0;
  bool navCapture = true, fpValid = true, dirTo = false, hdgSelValid = true, trkSelValid = false, hdgVs = true;
  bool bscuValid = true, wheelSpin = false;

  explicit LateralScenario(uint64_t seed) : rng(seed) {}
  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  bool p(double prob) { return u(0, 1) < prob; }

  void startEpisode(const fmgc_inputs& in) {
    te = 0;
    epLen = u(10, 90);
    takeoff = in.bus_inputs.ra_own_bus.radio_height_ft.Data < 5 && p(0.5);
    locInvalidAt = -1;
    ils = p(0.6);
    locInvalid = p(0.1);
    backbeam = p(0.05);
    float hdg = in.bus_inputs.ir_own_bus.heading_magnetic_deg.Data;
    double r = u(0, 1);
    double offset = r < 0.4 ? u(-12, 12) : (r < 0.75 ? u(-60, 60) : u(-180, 180));
    course = std::fmod(hdg + offset + 720.0, 360.0);
    if (p(0.1)) {
      course = p(0.5) ? u(0, 3) : u(357, 360);  // near the 360/0 wrap
    }
    L0 = r < 0.3 ? u(-0.5, 0.5) : (r < 0.7 ? u(-3, 3) : u(-12, 12));
    tauL = u(2, 60);
    w = p(0.3) ? u(0, 0.5) : 0;
    raOn = p(0.5);
    raMode = u(0, 4);
    static const double thresholds[] = {30, 50, 100, 400};
    ra0 = thresholds[static_cast<int>(u(0, 4)) % 4] + u(-15, 15);
    raRate = u(-30, 30);
    rollOn = p(0.4);
    roll0 = u(-30, 30);
    rollW = u(0, 0.5);
    fcuOn = p(0.7);
    fmsOn = p(0.5);
    phase = static_cast<int>(u(0, 8));
    noApFd = p(0.15);
    oppMaster = p(0.15);
    oppW2 = 0;
    oppW3 = 0;
    for (int b : {11, 12, 13, 14, 15, 16, 17, 20, 23}) {
      if (p(0.25)) oppW2 |= 1u << (b - 1);
    }
    for (int b : {14, 16}) {
      if (p(0.3)) oppW3 |= 1u << (b - 1);
    }
    wheels = p(0.5);
    presetToggle = p(0.4);
    trkToggle = p(0.3);
    nextFcuIn = u(0, 10);
    nextFmsIn = u(0, 10);
    toggleIn = u(0, 10);
    if (takeoff) {
      // On the ground: AP/FD off (resets the modes), then a TOGA thrust lever edge (SRS takeoff, RWY) with a runway
      // course near the heading (sometimes off by about 360°, unwrapped) and a small localizer deviation.
      epLen = u(12, 40);
      ils = true;
      raOn = false;
      rollOn = false;
      oppMaster = p(0.1);
      noApFd = true;
      double off = u(-26, 26);
      course = hdg + (p(0.15) ? (p(0.5) ? 360.0 : -360.0) + off : off);
      L0 = u(-0.6, 0.6);
      tauL = u(5, 100);
      w = 0;
      locInvalid = p(0.1);
      togaAt = u(3, 6);
      locInvalidAt = p(0.25) ? u(togaAt + 0.5, epLen) : -1;
    }
    // Exact thresholds (integer-valued ones are representable, so `<` vs `<=` can be told apart): an integer heading
    // with the course at an exact offset, a constant deviation of exactly 10° or 0°, a roll of exactly ±5°, a radio
    // height exactly at a threshold.
    snapHeading = p(0.3);
    if (snapHeading) {
      static const double offsets[] = {0, 15, 20, 25, 115, 114, 116, -15, -20, -25, -115, 360, -340};
      hdgInt = std::round(hdg);
      course = hdgInt + offsets[static_cast<int>(u(0, 13)) % 13];
      if (p(0.3)) {
        L0 = p(0.5) ? 10.0 : (p(0.5) ? -10.0 : 0.0);
        tauL = 1e9;
        w = 0;
      }
    }
    if (p(0.2)) {
      roll0 = p(0.5) ? 5.0 : -5.0;
      rollW = 0;
    }
    if (p(0.4)) {
      ra0 = thresholds[static_cast<int>(u(0, 4)) % 4];
      raMode = u(0, 2);
    }
  }

  void apply(fmgc_inputs& in, double dt) {
    if (te >= epLen) {
      startEpisode(in);
    }
    te += dt;
    auto word = [](base_arinc_429& wd, float v) { wd.SSM = 3; wd.Data = v; };
    auto setBit = [](base_arinc_429& wd, int b) {
      uint32_t v = static_cast<uint32_t>(wd.Data);
      v |= 1u << (b - 1);
      wd.Data = static_cast<float>(v);
    };
    auto clearBit = [](base_arinc_429& wd, int b) {
      uint32_t v = static_cast<uint32_t>(wd.Data);
      v &= ~(1u << (b - 1));
      wd.Data = static_cast<float>(v);
    };
    auto& b = in.bus_inputs;

    if (ils) {
      float loc = static_cast<float>(L0 * std::exp(-te / tauL) * std::cos(w * te));
      for (auto* bus : {&b.ils_own_bus, &b.ils_opp_bus}) {
        word(bus->runway_heading_deg, static_cast<float>(course));
        word(bus->ils_frequency_mhz, 109.5F);
        word(bus->localizer_deviation_deg, loc);
        if (locInvalid) bus->localizer_deviation_deg.SSM = 0;
      }
      in.fms_inputs.backbeam_selected = backbeam;
    }
    if (raOn) {
      double ra = ra0;
      if (raMode >= 2 && raMode < 3) ra = std::fmax(0.0, ra0 + raRate * te);  // climbing or descending through
      if (raMode >= 3) ra = std::fmax(0.0, u(0, 1) < 0.5 ? ra0 : ra0 + 10 * std::sin(te));
      word(b.ra_own_bus.radio_height_ft, static_cast<float>(ra));
      word(b.ra_opp_bus.radio_height_ft, static_cast<float>(ra));
    }
    if (snapHeading && ils) {
      for (auto* ir : {&b.ir_own_bus, &b.ir_opp_bus, &b.ir_3_bus}) {
        ir->heading_magnetic_deg.Data = static_cast<float>(hdgInt);
      }
    }
    if (rollOn) {
      float roll = static_cast<float>(roll0 * std::cos(rollW * te));
      for (auto* ir : {&b.ir_own_bus, &b.ir_opp_bus, &b.ir_3_bus}) {
        ir->roll_angle_deg.Data = roll;
      }
    }
    if (fcuOn) {
      if (fcuLeft <= 0) {
        nextFcuIn -= dt;
        if (nextFcuIn <= 0) {
          static const int bits[] = {11, 12, 13, 13};
          fcuBit = bits[static_cast<int>(u(0, 4)) % 4];
          fcuLeft = u(0.05, 0.5);
          nextFcuIn = p(0.3) ? u(0.2, 1.5) : u(2, 20);  // sometimes a quick second push
        }
      }
      if (fcuLeft > 0) {
        fcuLeft -= dt;
        setBit(b.fcu_bus.fcu_discrete_word_2, fcuBit);
      }
    }
    toggleIn -= dt;
    if (toggleIn <= 0) {
      toggleIn = u(0.5, 15);
      if (presetToggle) {
        hdgSelValid = p(0.5);
        trkSelValid = p(0.3);
      }
      if (trkToggle) hdgVs = p(0.5);
      bscuValid = p(0.7);
      wheelSpin = p(0.6);
    }
    if (presetToggle) {
      b.fcu_bus.selected_hdg_deg.SSM = hdgSelValid ? 3 : 1;
      b.fcu_bus.selected_trk_deg.SSM = trkSelValid ? 3 : 1;
    }
    if (trkToggle) {
      if (hdgVs) setBit(b.fcu_bus.fcu_discrete_word_1, 24);
      else clearBit(b.fcu_bus.fcu_discrete_word_1, 24);
    }
    if (fmsOn) {
      nextFmsIn -= dt;
      if (nextFmsIn <= 0) {
        nextFmsIn = u(0.3, 15);
        navCapture = p(0.6);
        fpValid = p(0.8);
        dirTo = p(0.2);
      }
      in.fms_inputs.nav_capture_condition = navCapture;
      in.fms_inputs.lateral_flight_plan_valid = fpValid;
      in.fms_inputs.direct_to_nav_engage = dirTo;
      in.fms_inputs.fms_flight_phase = static_cast<fmgc_flight_phase>(phase);
    }
    if (takeoff) {
      if (te >= togaAt && te < togaAt + 3) {
        word(b.fadec_own_bus.selected_tla_deg, 45.0F);
        word(b.fadec_opp_bus.selected_tla_deg, 45.0F);
      }
      if (in.fms_inputs.v_2_kts <= 0) in.fms_inputs.v_2_kts = 140;
      for (auto* fac : {&b.fac_own_bus, &b.fac_opp_bus}) setBit(fac->discrete_word_5, 11);  // flap handle 1
      if (locInvalidAt >= 0 && te >= locInvalidAt) {
        b.ils_own_bus.localizer_deviation_deg.SSM = 0;
        b.ils_opp_bus.localizer_deviation_deg.SSM = 0;
      }
    }
    if (noApFd && te < (takeoff ? 2 : 8)) {
      setBit(b.fcu_bus.fcu_discrete_word_2, 26);
      setBit(b.fcu_bus.fcu_discrete_word_2, 27);
      in.discrete_inputs.fd_opp_engaged = false;
      in.discrete_inputs.ap_opp_engaged = false;
      in.discrete_inputs.ap_instinctive_disc = true;
    }
    if (oppMaster) {
      in.discrete_inputs.is_unit_1 = false;
      in.discrete_inputs.ap_opp_engaged = true;
      b.fmgc_opp_bus.discrete_word_2.SSM = 3;
      b.fmgc_opp_bus.discrete_word_2.Data = static_cast<float>(oppW2);
      b.fmgc_opp_bus.discrete_word_3.SSM = 3;
      b.fmgc_opp_bus.discrete_word_3.Data = static_cast<float>(oppW3);
    }
    if (wheels && b.ra_own_bus.radio_height_ft.Data < 80) {
      in.discrete_inputs.bscu_own_valid = bscuValid;
      in.discrete_inputs.bscu_opp_valid = bscuValid && p(0.5);
      in.discrete_inputs.left_wheel_spd_abv_70_kts = wheelSpin;
      in.discrete_inputs.right_wheel_spd_abv_70_kts = wheelSpin && p(0.9);
    }
  }
};
