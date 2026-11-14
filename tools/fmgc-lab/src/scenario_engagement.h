// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic AP / FD / A/THR engagement episodes injected on top of a replay (after the other scenarios, before the
// fuzzer), for differential checks of the engagement logic: the recordings engage the AP once or twice per flight, in
// the normal envelope, with FMGC 1 as master and both FDs on, so most engage/disengage paths are never exercised. Each
// 8–90 s episode independently picks:
//   - AP pushbutton presses (single, or two presses 0.05–0.4 s apart: the 0.2 s request drop), sometimes repeated;
//   - AP instinctive disconnect (short, or held), ELAC AP disconnect (own, opposite, both);
//   - FAC rudder control failure (both FACs unhealthy);
//   - the AP engage envelope: pitch, bank (all three IRs) and IAS against VLS/VMAX at and around the limits;
//   - FD pushbuttons (FCU word 2 bits 26/27) and the opposite FD;
//   - the opposite FMGC: AP/FD/A/THR engaged patterns, AP engagement edges, unhealthy, this FMGC as unit 2;
//   - on the ground (FAC discrete word 5 on-ground bits): engines stopped/started, levers at and around TOGA/FLX
//     (36.7°), AP presses there;
//   - A/THR pushbutton presses with the radio height around 100 ft (or both RAs failed), instinctive disconnect held
//     past 15 s, both levers to idle (around 2.6°), one ECU not reporting A/THR engaged (status word 3 bit 19) for
//     0.3–2 s, FAC alpha floor;
//   - common condition losses around the 0.72 s confirmation: two ADRs or two IRs failed, FAC discrete word 5 failed,
//     FM not valid, FAC speeds/weights words failed, both RAs failed, both FCU channels unhealthy; FM lost in G/S
//     between 450 and 950 ft RA (with the approach scenario);
//   - the runway course: true heading 15–30° off the ILS runway heading low down (ROLL OUT off course, with the approach
//     scenario), sometimes alternating above and below 20° every second (retriggers of the 3 s monostable);
//   - FD auto disengage / V/S not held: VMAX just below, or VLS just above, the IAS (IAS exactly VLS + 3 / VMAX - 3
//     included), and the IR inertial V/S at the FCU V/S target +- 49/50/51 fpm;
//   - landing capability: the opposite AP engaged and in LAND (its bus bits) whenever this FMGC has LAND armed/LAND,
//     with the bus tie open (LAND 3 FAIL OP possible); a failure (opposite RA, ELAC, bus tie, ADR 3, AP or A/THR
//     instinctive disconnect) as the radio height passes 60–130 ft; random LAND capacity bits on the opposite bus.
// Physically inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "FmgcComputer_types.h"

struct EngagementScenario {
  std::mt19937_64 rng;
  double te = 1e9, epLen = 0;

  // AP
  double apPushAt[3]{-1, -1, -1}, apPushLen = 0.2, apSecondGap = -1;
  double apInstAt = -1, apInstLen = 0;
  int elac = 0;  // 0 recorded, 1 own disc, 2 opp disc, 3 both
  double elacAt = 0, elacLen = 0;
  bool facFail = false;
  double facFailAt = 0, facFailLen = 0;
  // envelope
  bool attitude = false, speeds = false;
  float pitch = 0, roll = 0;
  float vlsOffset = 0, vmaxOffset = 0;  // relative to IAS
  // FD
  int fdPattern = -1;  // -1 recorded, else bit 0: FD1 off, bit 1: FD2 off, bit 2: opp FD engaged
  double fdToggleAt = -1;
  // opposite FMGC
  int opp = -1;  // -1 recorded, else bit 0 AP, bit 1 FD, bit 2 A/THR
  double oppToggleAt = -1;
  int oppToggleBits = 0;
  bool unit2 = false, oppUnhealthy = false;
  // ground
  bool ground = false;
  int engines = -1;  // -1 recorded, 0 both stopped, 1 own stopped, 2 both running
  double engineStartAt = -1;
  float groundLever = -1;
  double leverAt = -1;
  bool oppLeverToo = false, oppLeverOnly = false;
  bool fmLossInGs = false;
  // landing capability
  bool dualLand = false;  // opposite AP engaged and in LAND whenever this FMGC has LAND armed/LAND
  int landFailure = 0;    // failure injected when the radio height passes landFailRa in LAND: 1 opp RA, 2 ELAC,
                          // 3 bus tie, 4 ADR 3, 5 AP instinctive, 6 A/THR instinctive
  float landFailRa = 100;
  double landFailAt = -1, landFailLen = 0;
  bool oppCapacity = false;  // random LAND capacity bits on the opposite FMGC's word 4
  uint32_t oppCapacityBits = 0;
  double nextCapacityChange = 0;
  double fmLossAt = -1, fmLossLen = 0;
  // A/THR
  double athrPushAt[3]{-1, -1, -1};
  float raFt = -1;
  bool raFailed = false;
  double athrInstAt = -1, athrInstLen = 0;
  float idleLever = -1;
  double idleAt = -1;
  int ecuDrop = 0;  // 0 none, 1 own, 2 opp
  double ecuAt = 0, ecuLen = 0;
  double afAt = -1, afLen = 0;
  // common conditions
  int failure = 0;  // 0 none, 1 2 ADRs, 2 2 IRs, 3 FAC word 5, 4 FM, 5 FAC speeds, 6 FAC weights, 7 RAs, 8 FCU
  double failAt = 0, failLen = 0;
  // runway course
  bool offCourse = false;
  float courseOffset = 0;
  // FD auto disengage / V/S not held
  bool speedLimit = false;
  float vsOffset = 0;
  bool offCourseToggle = false;  // heading deviation alternating above/below 20° every second (monostable retrigger)

  double total = 0, lockAfter = 1e18;
  bool lockUsed = false;

  explicit EngagementScenario(uint64_t seed) : rng(seed) { lockAfter = chance(0.5) ? u(300, 3000) : 1e18; }

  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  bool chance(double p) { return u(0, 1) < p; }
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

  void newEpisode() {
    te = 0;
    epLen = u(8, 90);
    auto at = [&](double p) { return chance(p) ? u(0, epLen) : -1.0; };

    for (double& a : apPushAt) {
      a = at(0.35);
    }
    apPushLen = chance(0.5) ? u(0.05, 0.3) : u(0.3, 1.5);
    apSecondGap = chance(0.3) ? u(0.05, 0.4) : -1;
    apInstAt = at(0.15);
    apInstLen = chance(0.7) ? u(0.05, 0.5) : u(0.5, 4);
    elac = chance(0.15) ? static_cast<int>(1 + rng() % 3) : 0;
    elacAt = u(0, epLen);
    elacLen = u(0.1, 10);
    facFail = chance(0.08);
    facFailAt = u(0, epLen);
    facFailLen = u(0.1, 6);

    attitude = chance(0.2);
    static const float pitches[]{-10.5F, -10.0F, -9.5F, 0.0F, 5.0F, 21.5F, 22.0F, 22.5F};
    static const float rolls[]{-40.5F, -40.0F, 0.0F, 20.0F, 39.5F, 40.0F, 40.5F};
    pitch = pick(pitches);
    roll = pick(rolls);
    speeds = chance(0.2);
    static const float offsets[]{-1.0F, 0.0F, 0.0F, 1.0F, 5.0F, -5.0F};
    vlsOffset = chance(0.5) ? pick(offsets) : -30.0F;
    vmaxOffset = chance(0.5) ? pick(offsets) : 60.0F;

    fdPattern = chance(0.35) ? static_cast<int>(rng() % 8) : -1;
    fdToggleAt = at(0.3);

    opp = chance(0.35) ? static_cast<int>(rng() % 8) : -1;
    oppToggleAt = at(0.4);
    oppToggleBits = static_cast<int>(1 + rng() % 7);
    unit2 = chance(0.3);
    oppUnhealthy = chance(0.1);

    ground = chance(0.2);
    engines = ground && chance(0.7) ? static_cast<int>(rng() % 3) : -1;
    engineStartAt = engines >= 0 && engines < 2 && chance(0.5) ? u(0, epLen) : -1;
    static const float levers[]{25.0F, 35.0F, 36.6F, 36.7F, 36.8F, 45.0F};
    groundLever = ground && chance(0.5) ? pick(levers) : -1;
    leverAt = u(0, epLen);

    for (double& a : athrPushAt) {
      a = at(0.3);
    }
    static const float ras[]{50.0F, 99.0F, 100.0F, 100.5F, 150.0F, 2500.0F};
    raFt = chance(0.35) ? pick(ras) : -1;
    raFailed = chance(0.08);
    athrInstAt = at(0.15);
    // held past 15 s locks the A/THR out until the FMGC restarts: at most once per run, late, so that the rest of the
    // A/THR logic keeps its coverage
    bool lock = athrInstAt >= 0 && !lockUsed && total >= lockAfter && chance(0.3);
    lockUsed = lockUsed || lock;
    athrInstLen = lock ? u(14.5, 18) : (chance(0.1) ? u(13, 14.9) : u(0.05, 2));
    static const float idle[]{0.0F, 2.5F, 2.6F, 2.7F, 10.0F};
    idleLever = chance(0.15) ? pick(idle) : -1;
    idleAt = u(0, epLen);
    ecuDrop = chance(0.12) ? static_cast<int>(1 + rng() % 2) : 0;
    ecuAt = u(0, epLen);
    ecuLen = chance(0.5) ? u(0.3, 0.95) : u(0.95, 3);
    afAt = at(0.1);
    afLen = u(0.1, 3);

    failure = chance(0.25) ? static_cast<int>(1 + rng() % 8) : 0;
    failAt = u(0, epLen);
    failLen = chance(0.5) ? u(0.3, 1.0) : u(1, 8);

    offCourse = chance(0.25);
    courseOffset = static_cast<float>((chance(0.5) ? 1 : -1) * u(15, 30));
    offCourseToggle = chance(0.4);

    speedLimit = chance(0.15);
    if (speedLimit) {
      // FD auto disengage (VMAX + 4 < IAS, IAS < VLS - 2) and V/S not held (IAS within 3 kt of VLS / VMAX)
      static const float vlsNear[]{1.0F, 2.0F, 3.5F, -3.0F, -30.0F};
      static const float vmaxNear[]{-4.5F, -4.0F, 2.0F, 2.5F, 3.0F, 60.0F};
      vlsOffset = pick(vlsNear);
      vmaxOffset = pick(vmaxNear);
      static const float vsNear[]{0.0F, -50.0F, -49.0F, -51.0F, 50.0F, 49.0F, 51.0F};
      vsOffset = pick(vsNear);
    } else {
      vsOffset = 0;
    }
    oppLeverToo = chance(0.5);
    oppLeverOnly = !oppLeverToo && chance(0.4);
    fmLossInGs = chance(0.5);
    fmLossAt = -1;
    fmLossLen = u(0.5, 4);
    dualLand = chance(0.4);
    landFailure = chance(0.6) ? static_cast<int>(1 + rng() % 6) : 0;
    static const float failRas[]{60.0F, 90.0F, 99.0F, 101.0F, 110.0F, 130.0F};
    landFailRa = pick(failRas);
    landFailAt = -1;
    landFailLen = u(0.3, 5);
    oppCapacity = chance(0.4);
    oppCapacityBits = 0;
    nextCapacityChange = 0;
  }

  /** `last`: the generated outputs of the previous step. */
  void apply(fmgc_inputs& in, double dt, const fmgc_outputs& last) {
    if (te >= epLen) {
      newEpisode();
    }
    te += dt;
    total += dt;
    auto& bus = in.bus_inputs;
    auto& di = in.discrete_inputs;
    auto window = [&](double start, double len) { return start >= 0 && te >= start && te < start + len; };

    // FM lost in G/S around 700 ft RA (the common condition tolerates it below 700 ft): needs the approach scenario
    float lastRa = last.logic.ra_computation_data.radio_height_ft.Data;
    if (fmLossInGs && fmLossAt < 0 && last.ap_fd_logic.longitudinal_modes.gs_trk_active && lastRa > 450.0F &&
        lastRa < 950.0F) {
      fmLossAt = te;
    }
    if (window(fmLossAt, fmLossLen)) {
      in.fms_inputs.fm_valid = false;
    }

    // AP pushbutton, instinctive and ELAC disconnects, FAC rudder control
    for (double a : apPushAt) {
      if (window(a, apPushLen) || (apSecondGap >= 0 && window(a + apPushLen + apSecondGap, apPushLen))) {
        di.fcu_ap_button = true;
      }
    }
    if (window(apInstAt, apInstLen)) {
      di.ap_instinctive_disc = true;
    }
    if (elac && window(elacAt, elacLen)) {
      di.elac_own_ap_disc = (elac & 1) != 0;
      di.elac_opp_ap_disc = (elac & 2) != 0;
    }
    if (facFail && window(facFailAt, facFailLen)) {
      di.fac_own_healthy = false;
      di.fac_opp_healthy = false;
    }

    // AP engage envelope
    if (attitude) {
      for (auto* ir : {&bus.ir_own_bus, &bus.ir_opp_bus, &bus.ir_3_bus}) {
        ir->pitch_angle_deg.Data = pitch;
        ir->roll_angle_deg.Data = roll;
      }
    }
    if (speeds || speedLimit) {
      float ias = bus.adr_own_bus.airspeed_computed_kn.Data;
      for (auto* adr : {&bus.adr_opp_bus, &bus.adr_3_bus}) {
        adr->airspeed_computed_kn.Data = ias;
      }
      float vls = ias + vlsOffset;
      float vmax = ias + vmaxOffset;
      // offsets -3 / +3 are made exact in float: IAS == VLS + 3, IAS == VMAX - 3 (the V/S target not held bounds)
      if (vlsOffset == -3.0F) {
        while (vls + 3.0F > ias) vls = std::nextafter(vls, -INFINITY);
        while (vls + 3.0F < ias) vls = std::nextafter(vls, INFINITY);
      }
      if (vmaxOffset == 3.0F) {
        while (vmax - 3.0F > ias) vmax = std::nextafter(vmax, -INFINITY);
        while (vmax - 3.0F < ias) vmax = std::nextafter(vmax, INFINITY);
      }
      for (auto* fac : {&bus.fac_own_bus, &bus.fac_opp_bus}) {
        fac->v_ls_kn.Data = vls;
        fac->v_max_kn.Data = vmax;
      }
    }
    // inertial V/S at the FCU V/S target +- 50 fpm (the V/S target not held threshold)
    if (vsOffset != 0.0F) {
      float target = bus.fcu_bus.selected_vz_ft_min.Data;
      for (auto* ir : {&bus.ir_own_bus, &bus.ir_opp_bus, &bus.ir_3_bus}) {
        ir->inertial_vertical_speed_ft_s.Data = target + vsOffset;
      }
    }

    // FD pushbuttons and the opposite FD
    if (fdPattern >= 0) {
      int p = fdPattern ^ (fdToggleAt >= 0 && te >= fdToggleAt ? 3 : 0);
      setBit(bus.fcu_bus.fcu_discrete_word_2, 26, (p & 1) != 0);
      setBit(bus.fcu_bus.fcu_discrete_word_2, 27, (p & 2) != 0);
      di.fd_opp_engaged = (p & 4) != 0;
    }

    // opposite FMGC
    if (opp >= 0) {
      int p = opp ^ (oppToggleAt >= 0 && te >= oppToggleAt ? oppToggleBits : 0);
      di.ap_opp_engaged = (p & 1) != 0;
      di.fd_opp_engaged = (p & 2) != 0 || (fdPattern >= 0 && di.fd_opp_engaged);
      di.athr_opp_engaged = (p & 4) != 0;
    }
    if (unit2) {
      di.is_unit_1 = false;
    }
    if (oppUnhealthy) {
      di.fmgc_opp_healthy = false;
    }

    // on the ground
    if (ground) {
      for (auto* w : {&bus.fac_own_bus.discrete_word_5, &bus.fac_opp_bus.discrete_word_5}) {
        w->SSM = 3;
        setBit(*w, 17, false);
        setBit(*w, 18, true);
        setBit(*w, 19, true);
      }
      if (engines >= 0) {
        int e = engines;
        if (engineStartAt >= 0 && te >= engineStartAt) {
          e = 2;
        }
        di.eng_own_stop = e <= 1;
        di.eng_opp_stop = e == 0;
      }
      if (groundLever >= 0 && te >= leverAt) {
        bus.fadec_own_bus.selected_tla_deg.SSM = 3;
        if (!oppLeverOnly || te > leverAt + 2) {
          bus.fadec_own_bus.selected_tla_deg.Data = groundLever;
        }
        if (oppLeverToo || oppLeverOnly || te > leverAt + 2) {
          bus.fadec_opp_bus.selected_tla_deg.SSM = 3;
          bus.fadec_opp_bus.selected_tla_deg.Data = groundLever;
        }
      }
    }

    // A/THR
    for (double a : athrPushAt) {
      if (window(a, 0.2)) {
        di.fcu_athr_button = true;
      }
    }
    if (raFt >= 0) {
      for (auto* w : {&bus.ra_own_bus.radio_height_ft, &bus.ra_opp_bus.radio_height_ft}) {
        w->SSM = 3;
        w->Data = raFt;
      }
    }
    if (raFailed) {
      bus.ra_own_bus.radio_height_ft.SSM = 0;
      bus.ra_opp_bus.radio_height_ft.SSM = 0;
    }
    if (window(athrInstAt, athrInstLen)) {
      di.athr_instinctive_disc = true;
    }
    if (idleLever >= 0 && te >= idleAt) {
      for (auto* w : {&bus.fadec_own_bus.selected_tla_deg, &bus.fadec_opp_bus.selected_tla_deg}) {
        w->SSM = 3;
        w->Data = idleLever;
      }
    }
    if (ecuDrop && window(ecuAt, ecuLen)) {
      setBit(ecuDrop == 1 ? bus.fadec_own_bus.ecu_status_word_3 : bus.fadec_opp_bus.ecu_status_word_3, 19, false);
    }
    if (window(afAt, afLen)) {
      setBit(bus.fac_own_bus.discrete_word_5, 29, true);
      setBit(bus.fac_opp_bus.discrete_word_5, 29, true);
    }

    // common condition losses
    if (failure && window(failAt, failLen)) {
      switch (failure) {
        case 1:
          bus.adr_own_bus.airspeed_computed_kn.SSM = 0;
          bus.adr_opp_bus.mach.SSM = 0;
          break;
        case 2:
          bus.ir_own_bus.pitch_angle_deg.SSM = 0;
          bus.ir_3_bus.roll_angle_deg.SSM = 0;
          break;
        case 3:
          bus.fac_own_bus.discrete_word_5.SSM = 0;
          bus.fac_opp_bus.discrete_word_5.SSM = 0;
          break;
        case 4:
          in.fms_inputs.fm_valid = false;
          break;
        case 5:
          bus.fac_own_bus.v_ls_kn.SSM = 0;
          bus.fac_opp_bus.v_ls_kn.SSM = 0;
          break;
        case 6:
          bus.fac_own_bus.total_weight_lbs.SSM = 0;
          bus.fac_opp_bus.total_weight_lbs.SSM = 0;
          break;
        case 7:
          bus.ra_own_bus.radio_height_ft.SSM = 0;
          bus.ra_opp_bus.radio_height_ft.SSM = 0;
          break;
        default:
          di.fcu_own_healthy = false;
          di.fcu_opp_healthy = false;
          break;
      }
    }

    // ROLL OUT off the runway course: true heading away from the ILS runway heading low down
    if (offCourse && bus.ra_own_bus.radio_height_ft.Data < 10.0F) {
      float rwy = bus.ils_own_bus.runway_heading_deg.Data;
      for (auto* ir : {&bus.ir_own_bus, &bus.ir_opp_bus, &bus.ir_3_bus}) {
        float offset = offCourseToggle && std::fmod(te, 2.0) < 1.0 ? courseOffset * 0.5F : courseOffset;
        ir->heading_true_deg.Data = rwy + offset;
      }
    }

    // landing capability: the opposite AP in LAND with this FMGC's LAND (dual AP, LAND 3 FAIL OP possible), and a
    // failure injected as the radio height passes a height around the 100 ft freeze
    const auto& lastAp = last.ap_fd_logic;
    bool landLast = lastAp.armed_modes.land_armed || lastAp.lateral_modes.land_active;
    if (dualLand && landLast) {
      di.ap_opp_engaged = true;
      di.powersupply_split = true;
      bus.fmgc_opp_bus.discrete_word_3.SSM = 3;
      bus.fmgc_opp_bus.discrete_word_4.SSM = 3;
      setBit(bus.fmgc_opp_bus.discrete_word_3, 20, lastAp.armed_modes.land_armed);
      setBit(bus.fmgc_opp_bus.discrete_word_4, 14, lastAp.lateral_modes.land_active);
    }
    if (landFailure && landFailAt < 0 && landLast && lastRa < landFailRa) {
      landFailAt = te;
    }
    if (window(landFailAt, landFailLen)) {
      switch (landFailure) {
        case 1: bus.ra_opp_bus.radio_height_ft.SSM = 0; break;
        case 2: di.elac_opp_ap_disc = true; break;
        case 3: di.powersupply_split = false; break;
        case 4: bus.adr_3_bus.airspeed_computed_kn.SSM = 0; break;
        case 5: di.ap_instinctive_disc = true; break;
        default: di.athr_instinctive_disc = true; break;
      }
    }

    // LAND capacity bits of the opposite FMGC (bits 15-18 of its word 4: neighbours of the three capacity bits)
    if (oppCapacity) {
      if (te >= nextCapacityChange) {
        oppCapacityBits = static_cast<uint32_t>(rng() % 16);
        nextCapacityChange = te + u(0.5, 8);
      }
      auto& w4 = bus.fmgc_opp_bus.discrete_word_4;
      w4.SSM = chance(0.05) ? 0 : 3;
      for (int b = 0; b < 4; b++) {
        setBit(w4, 15 + b, ((oppCapacityBits >> b) & 1) != 0);
      }
    }
  }
};
