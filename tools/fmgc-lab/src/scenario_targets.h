// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic episodes for the targets block (selected/managed speed, SPD/MACH, speed targets, HDG/TRK preset, ALT soft
// mode), injected on top of a replay (after the approach scenario, if any). The recordings fly managed speed with the
// knobs untouched almost all the time. Each 10–150 s episode picks a random subset of:
//   - FCU word 1: SPD knob push (bit 11) and pull (bit 12), SPD/MACH pb (bit 21), bit 15, sometimes twice in a row;
//   - FCU selected speed / Mach values turning (ramps), sometimes not normal; static pressure on all ADRs;
//   - crossover: climb with an FMS Mach-mode request, descent with an FMS speed-mode request (one frame, a few frames,
//     or both requests at once), the managed speed ramping;
//   - MCDU presets: speed, Mach, both or neither, one frame or held, sometimes in SRS takeoff above/below the FCU
//     altitude;
//   - FMS speed lost (v_managed = 0) and V2 deleted/inserted;
//   - no AP/FD for a while (FD pbs off, opposite AP/FD off), then back;
//   - on the ground (FAC discrete word 5 bits 17/18/19), with an engine start (both engines stopped, then started);
//   - one engine out in flight, sometimes toggling;
//   - this FMGC as unit 2 with the opposite AP engaged (opposite-FMGC sync), with random opposite SPD/MACH bits;
//   - a TCAS RA (TCAS mode engagement selects the speed);
//   - DES speed margins: show_speed_margins, DES submodes SPEED_THRUST / VPATH_SPEED / others, margins and managed
//     speed around the CAS (integer values: the CAS exactly at a margin or at the managed speed);
//   - approach speeds: approach phase, tower headwind valid or not, IR wind, VFE next valid or not, VMAX, the flap
//     lever in CONF 3 (or another) with or without a CONF 3 landing, FAC manoeuvring speeds, VAPP changing;
//   - HDG/TRK preset: FCU selected heading/track valid or not, HDG knob push/pull (word 2 bits 11/12), HDG-V/S /
//     TRK-FPA toggle (word 1 bit 24);
//   - an FMGC restart (computer not running for 0.2–3 s), in flight or on the ground;
//   - a synthetic takeoff: on the ground with AP/FD off, then FDs on, V2 inserted, flaps 1, TOGA (SRS takeoff and
//     managed speed), sometimes an engine failure during the SRS climb, lift-off (FAC on-ground bits cleared), and
//     sometimes the FMS speed lost 5 s after lift-off;
//   - HDG/TRK preset windows on the ground or at a low radio height (< 30 ft) with NAV armed (HDG knob pull then push
//     with no NAV capture), and the FCU heading/track appearing (at 20 ft or exactly 30 ft);
//   - the radio height held at exactly 700 ft while it passes 650-750 ft (with --approach: LAND armed);
// and long (300–700 s) ALT soft episodes: cruise phase, a Mach request, the managed speed held within 4 kt of the CAS,
// then, every ~2 min, moved off by more than 4 or 10 kt (around the timer lengths), VMAX brought near CAS + 5, or an
// FCU word 1 bit 15 pulse; started mostly when the recording is in level flight above 10000 ft (ALT itself comes from the recording).
// Physically inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "FmgcComputer_types.h"

struct TargetsScenario {
  std::mt19937_64 rng;
  double te = 1e9, epLen = 0;

  // episode components
  bool fcuButtons = false, fcuValues = false, pressure = false, crossover = false, presets = false, speedsLost = false,
       noApFd = false, ground = false, engineOut = false, oppMaster = false, tcas = false, margins = false,
       approach = false, hdgPreset = false, restart = false, soft = false;

  // FCU buttons
  double nextPushIn = 0, pushLeft = 0;
  int pushBit = 0;
  // FCU values
  float spdBase = 250, spdRate = 0, machBase = 0.78F, machRate = 0;
  bool spdNcd = false, machNcd = false;
  float pressureHpa = 300;
  bool pressureNcd = false;
  // crossover
  bool climb = true;
  double crossAt = 0, crossLen = 0;
  int crossKind = 0;
  float vManaged0 = 300, vManagedRate = 0;
  // presets
  double presetAt = 0, presetLen = 0;
  float presetSpd = 0, presetMach = 0;
  bool presetSrs = false, presetBelowFcuAlt = false;
  // FMS speeds lost / V2
  double lostAt = 0, lostLen = 0, v2At = 0, v2Len = 0;
  bool v2Delete = false;
  // AP/FD off
  double apFdOffAt = 0, apFdOffLen = 0;
  // ground
  double groundAt = 0, groundLen = 0, engStopAt = -1, engStartAt = -1, groundSpeedsLost = -1;
  bool groundDropV2 = false;
  // engine out
  int engineOutWhich = 0;
  double engineToggleAt = -1;
  // opposite FMGC
  uint32_t oppWord5 = 0;
  double nextOppChange = 0;
  // TCAS
  double raStart = 0, raLen = 0;
  uint32_t raWord = 0;
  // margins
  int submode = 0;
  double nextMarginChange = 0;
  float upperOffset = 0, lowerOffset = 0, managedOffset = 0;
  bool showMargins = true, integerCas = false;
  // approach
  bool towerValid = true, vfeValid = true, flap3 = false, conf3Landing = false, approachPhase = true;
  float towerHw = 0, windDir = 0, windSpd = 0, vfeNext = 200, vapp = 135, vmax = 230, vman = 0, v3 = 0, v4 = 0,
        vls = 125;
  int flapBit = 0;
  double nextApproachChange = 0;
  // HDG preset
  double nextHdgIn = 0, hdgLeft = 0, toggleIn = 0;
  int hdgBit = 0;
  bool hdgSelValid = true, trkSelValid = false, hdgVs = true;
  // restart
  double restartAt = 0, restartLen = 0;
  // synthetic takeoff
  bool takeoff = false, takeoffEngineOut = false, liftoffNoManaged = false;
  double togaAt = 0, liftoffAt = 0, takeoffEoAt = 0;
  // HDG/TRK preset on the ground / low
  bool raSnap = false;  // radio height exactly 700 ft while passing 650-750 ft (the VAPP memo freeze threshold)
  float presetRa = 20;
  int presetWhere = 0;  // 0 none (only the FCU toggles), 1 on the ground, 2 low radio height
  double pullAt = 0, pushAt = 0;
  // ALT soft
  double softOffAt = -1, softOffLen = 0, softVmaxAt = -1, softBit15At = -1, softExitEnd = 0;
  float softOffset = 0, softOffKt = 0, softVmaxOffset = 5;

  explicit TargetsScenario(uint64_t seed) : rng(seed) {}
  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  bool p(double prob) { return u(0, 1) < prob; }
  bool window(double at, double len) const { return at >= 0 && te >= at && te < at + len; }
  static uint32_t word(float data) { return data >= 0.0F && data < 4294967296.0F ? static_cast<uint32_t>(data) : 0u; }
  static void setBit(base_arinc_429& w, int b, bool v) {
    uint32_t x = word(w.Data);
    x = v ? (x | (1u << (b - 1))) : (x & ~(1u << (b - 1)));
    w.Data = static_cast<float>(x);
  }
  static void normal(base_arinc_429& w, float v) {
    w.SSM = 3;
    w.Data = v;
  }

  void startEpisode(const fmgc_inputs& in) {
    te = 0;
    // ALT soft needs ALT from the recording: mostly started in level flight above 10000 ft
    bool level = std::fabs(in.bus_inputs.adr_own_bus.vertical_speed_ft_min.Data) < 300 &&
                 in.bus_inputs.adr_own_bus.altitude_standard_ft.Data > 10000;
    soft = p(level ? 0.5 : 0.05);
    epLen = soft ? u(300, 700) : u(10, 150);
    float cas = in.bus_inputs.adr_own_bus.airspeed_computed_kn.Data;

    fcuButtons = p(0.6);
    nextPushIn = u(0, 6);
    fcuValues = p(0.5);
    spdBase = static_cast<float>(u(100, 399));
    spdRate = static_cast<float>(p(0.5) ? u(-5, 5) : 0);
    machBase = static_cast<float>(u(0.3, 0.85));
    machRate = static_cast<float>(p(0.5) ? u(-0.01, 0.01) : 0);
    spdNcd = p(0.15);
    machNcd = p(0.15);
    pressure = p(0.4);
    pressureHpa = static_cast<float>(u(150, 1050));
    pressureNcd = p(0.15);

    crossover = p(0.35);
    climb = p(0.5);
    crossAt = u(0, epLen);
    static const double lens[]{0, 0, 0.1, 0.5, 3};  // 0 = one frame
    crossLen = lens[rng() % 5];
    crossKind = static_cast<int>(rng() % 6);  // 0-3 the phase's request, 4 the other one, 5 both
    vManaged0 = static_cast<float>(u(150, 340));
    vManagedRate = static_cast<float>(u(-1, 1));

    presets = p(0.3);
    presetAt = u(0, epLen);
    presetLen = p(0.6) ? 0 : u(0.1, 3);
    int pk = static_cast<int>(rng() % 4);
    presetSpd = pk == 0 || pk == 2 ? static_cast<float>(u(150, 330)) : 0.0F;
    presetMach = pk == 1 || pk == 2 ? static_cast<float>(u(0.5, 0.82)) : 0.0F;
    presetSrs = p(0.3);
    presetBelowFcuAlt = p(0.5);

    speedsLost = p(0.3);
    lostAt = p(0.7) ? u(0, epLen) : -1;
    lostLen = u(0.1, 20);
    v2At = p(0.6) ? u(0, epLen) : -1;
    v2Len = u(0.1, 20);
    v2Delete = p(0.5);

    noApFd = p(0.2);
    apFdOffAt = u(0, epLen * 0.7);
    apFdOffLen = u(0.5, 15);

    ground = p(0.2);
    groundAt = u(0, epLen * 0.5);
    groundLen = u(3, epLen);
    engStopAt = p(0.6) ? groundAt + u(0, 3) : -1;
    engStartAt = engStopAt >= 0 ? engStopAt + u(0.5, 8) : -1;
    groundSpeedsLost = p(0.4) ? groundAt + u(1, 10) : -1;
    groundDropV2 = p(0.5);

    engineOut = p(0.2);
    engineOutWhich = 1 + static_cast<int>(rng() % 2);
    engineToggleAt = p(0.4) ? u(0, epLen) : -1;

    oppMaster = p(0.12);
    nextOppChange = 0;

    tcas = p(0.15);
    raStart = u(0.5, epLen * 0.6);
    raLen = u(2, 30);
    {
      static const int rates[]{0, 3, 8, 15};
      uint32_t rate = static_cast<uint32_t>(rates[rng() % 4]);
      uint32_t control = static_cast<uint32_t>(4 + rng() % 3);
      raWord = (rate << 10) | (p(0.4) ? 1u << 16 : 0u) | (control << 17);
    }

    margins = p(0.3);
    showMargins = p(0.8);
    integerCas = p(0.6);
    nextMarginChange = 0;

    approach = p(0.3);
    approachPhase = p(0.7);
    nextApproachChange = 0;

    hdgPreset = p(0.3);
    nextHdgIn = u(0, 8);
    toggleIn = 0;

    restart = p(0.08);
    restartAt = u(0, epLen * 0.8);
    restartLen = u(0.2, 3);

    takeoff = p(0.12);
    if (takeoff) {
      epLen = u(25, 90);
      togaAt = u(4, 8);
      liftoffAt = togaAt + u(3, 15);
      takeoffEngineOut = p(0.4);
      takeoffEoAt = u(togaAt + 0.5, liftoffAt + 20);
      liftoffNoManaged = p(0.3);
      ground = engineOut = noApFd = restart = false;
    }
    presetWhere = hdgPreset ? static_cast<int>(rng() % 3) : 0;
    raSnap = p(0.3);
    presetRa = p(0.4) ? 30.0F : 20.0F;
    pullAt = u(0.5, 4);
    pushAt = pullAt + u(1, 4);

    if (soft) {
      // ALT soft: on target for a long time, then a way out, again every ~2 min
      fcuButtons = fcuValues = crossover = presets = speedsLost = noApFd = ground = engineOut = oppMaster = tcas =
          margins = approach = hdgPreset = restart = false;
      softOffset = static_cast<float>(p(0.3) ? (p(0.5) ? 4.0 : -4.0) : u(-3, 3));
      planSoftExit(0);
    }
    (void)cas;
  }

  /** The next ALT soft exit, a little over 2 min after `from` (the soft mode needs 120 s to engage). */
  void planSoftExit(double from) {
    softOffAt = softVmaxAt = softBit15At = -1;
    softExitEnd = from + u(122, 135);
    int way = static_cast<int>(rng() % 5);
    // off by just over 4 kt (the 10 s timer) or around 10 kt (the 4 s timer), for about the timer's length
    static const float small[]{4.5F, 5, 6};
    static const float large[]{8, 9, 10, 10.5F, 12, 13, 14};
    softOffKt = way == 0 ? small[rng() % 3] : large[rng() % 7];
    if (p(0.5)) softOffKt = -softOffKt;
    softOffLen = way == 0 ? u(7, 13) : u(2, 6);
    static const float vmaxOffsets[]{3.5F, 4, 4.5F, 5, 5.5F, 6, 7};
    softVmaxOffset = vmaxOffsets[rng() % 7];
    if (way == 0 || way == 1) {
      softOffAt = softExitEnd;
      softExitEnd += softOffLen;
    } else if (way == 2) {
      softVmaxAt = softExitEnd;
      softExitEnd += 8;
    } else if (way == 3) {
      softBit15At = softExitEnd;
      softExitEnd += 0.3;
    }  // way 4: no exit this time
  }

  void apply(fmgc_inputs& in, double dt) {
    if (te >= epLen) {
      startEpisode(in);
    }
    te += dt;
    auto& b = in.bus_inputs;
    auto& fms = in.fms_inputs;
    auto& di = in.discrete_inputs;
    float cas = b.adr_own_bus.airspeed_computed_kn.Data;

    if (fcuButtons) {
      if (pushLeft <= 0) {
        nextPushIn -= dt;
        if (nextPushIn <= 0) {
          static const int bits[]{11, 12, 21, 21, 21, 15};
          pushBit = bits[rng() % 6];
          pushLeft = u(0.03, 0.4);
          nextPushIn = p(0.3) ? u(0.1, 1.5) : u(1, 15);  // sometimes a quick second press
        }
      }
      if (pushLeft > 0) {
        pushLeft -= dt;
        setBit(b.fcu_bus.fcu_discrete_word_1, pushBit, true);
      }
    }
    if (fcuValues) {
      b.fcu_bus.selected_spd_kts.Data = static_cast<float>(std::fmin(399.0, std::fmax(100.0, spdBase + spdRate * te)));
      b.fcu_bus.selected_spd_kts.SSM = spdNcd ? 1 : 3;
      b.fcu_bus.selected_mach.Data = static_cast<float>(std::fmin(0.99, std::fmax(0.1, machBase + machRate * te)));
      b.fcu_bus.selected_mach.SSM = machNcd ? 1 : 3;
    }
    if (pressure) {
      for (auto* adr : {&b.adr_own_bus, &b.adr_opp_bus, &b.adr_3_bus}) {
        normal(adr->corrected_average_static_pressure, pressureHpa);
        if (pressureNcd) adr->corrected_average_static_pressure.SSM = 1;
      }
    }
    if (crossover) {
      fms.fms_flight_phase = climb ? fmgc_flight_phase::Climb : fmgc_flight_phase::Descent;
      fms.v_managed_kts = std::fmax(0.0, vManaged0 + vManagedRate * te);
      bool on = te >= crossAt && (crossLen == 0 ? te - dt < crossAt : te < crossAt + crossLen);
      if (on) {
        bool machReq = crossKind <= 3 ? climb : (crossKind == 4 ? !climb : true);
        bool spdReq = crossKind <= 3 ? !climb : (crossKind == 4 ? climb : true);
        fms.fms_mach_mode_activate = machReq;
        fms.fms_spd_mode_activate = spdReq;
      }
    }
    if (presets) {
      bool on = te >= presetAt && (presetLen == 0 ? te - dt < presetAt : te < presetAt + presetLen);
      fms.preset_spd_kts = presetSpd;
      fms.preset_mach = presetMach;
      if (on) fms.preset_spd_mach_activate = true;
      if (presetSrs && te >= presetAt - 1 && te < presetAt + presetLen + 1) {
        // SRS takeoff needs the takeoff conditions; instead, put the FCU altitude above or below the aircraft so the
        // preset's altitude condition sees both sides when SRS takeoff happens to be active (recorded takeoffs).
        float alt = b.adr_own_bus.altitude_corrected_1_ft.Data;
        normal(b.fcu_bus.selected_alt_ft, presetBelowFcuAlt ? alt + 1000 : alt - 1000);
      }
    }
    if (speedsLost) {
      if (window(lostAt, lostLen)) fms.v_managed_kts = 0;
      if (window(v2At, v2Len)) fms.v_2_kts = v2Delete ? 0 : 140;
    }
    if (noApFd && window(apFdOffAt, apFdOffLen)) {
      setBit(b.fcu_bus.fcu_discrete_word_2, 26, true);
      setBit(b.fcu_bus.fcu_discrete_word_2, 27, true);
      di.fd_opp_engaged = false;
      di.ap_opp_engaged = false;
      di.ap_instinctive_disc = true;
    }
    if (ground && window(groundAt, groundLen)) {
      for (auto* w : {&b.fac_own_bus.discrete_word_5, &b.fac_opp_bus.discrete_word_5}) {
        w->SSM = 3;
        setBit(*w, 17, false);
        setBit(*w, 18, true);
        setBit(*w, 19, true);
      }
      if (engStopAt >= 0 && te >= engStopAt && te < engStartAt) {
        di.eng_own_stop = true;
        di.eng_opp_stop = true;
      }
      if (groundSpeedsLost >= 0 && te >= groundSpeedsLost) {
        if (groundDropV2) fms.v_2_kts = 0;
        else fms.v_managed_kts = 0;
      }
    }
    if (engineOut) {
      bool out = engineToggleAt < 0 || te < engineToggleAt || std::fmod(te - engineToggleAt, 8.0) < 4.0;
      if (out) {
        di.eng_own_stop = engineOutWhich == 1;
        di.eng_opp_stop = engineOutWhich == 2;
      }
    }
    if (oppMaster) {
      nextOppChange -= dt;
      if (nextOppChange <= 0) {
        nextOppChange = u(0.5, 12);
        oppWord5 = (p(0.5) ? 1u << 18 : 0u) | (p(0.5) ? 1u << 19 : 0u);
      }
      di.is_unit_1 = false;
      di.ap_opp_engaged = true;
      b.fmgc_opp_bus.discrete_word_5.SSM = 3;
      b.fmgc_opp_bus.discrete_word_5.Data = static_cast<float>(oppWord5);
    }
    if (tcas) {
      auto& sens = b.tcas_bus.sensitivity_level;
      sens.SSM = 3;
      uint32_t s = word(sens.Data);
      s = (s | (1u << 25) | (1u << 26)) & ~(1u << 24);
      sens.Data = static_cast<float>(s);
      for (auto* ra : {&b.ra_own_bus.radio_height_ft, &b.ra_opp_bus.radio_height_ft}) {
        ra->SSM = 3;
        ra->Data = std::fmax(ra->Data, 2500.0F);
      }
      di.tcas_ta_display = true;
      auto& adv = b.tcas_bus.vertical_resolution_advisory;
      adv.SSM = 3;
      adv.Data = static_cast<float>(window(raStart, raLen) ? raWord : 0u);
    }
    if (margins) {
      if (integerCas) {
        cas = std::round(cas);
        for (auto* adr : {&b.adr_own_bus, &b.adr_opp_bus, &b.adr_3_bus}) adr->airspeed_computed_kn.Data = cas;
      }
      nextMarginChange -= dt;
      if (nextMarginChange <= 0) {
        nextMarginChange = u(1, 20);
        static const int submodes[]{1, 1, 3, 3, 3, 0, 2, 4, 5};
        submode = submodes[rng() % 9];
        static const float offs[]{-20, -5, -1, 0, 1, 5, 20};
        upperOffset = offs[rng() % 7];
        lowerOffset = offs[rng() % 7];
        managedOffset = offs[rng() % 7];
        if (!integerCas) {
          upperOffset += static_cast<float>(u(-0.5, 0.5));
          lowerOffset += static_cast<float>(u(-0.5, 0.5));
        }
      }
      fms.show_speed_margins = showMargins;
      fms.requested_des_submode = static_cast<fmgc_des_submode>(submode);
      fms.v_upper_margin_kts = cas + upperOffset;
      fms.v_lower_margin_kts = cas + lowerOffset;
      fms.v_managed_kts = cas + managedOffset;
    }
    if (approach) {
      nextApproachChange -= dt;
      if (nextApproachChange <= 0) {
        nextApproachChange = u(1, 20);
        towerValid = p(0.75);
        towerHw = static_cast<float>(u(-15, 30));
        windDir = static_cast<float>(u(0, 360));
        windSpd = static_cast<float>(p(0.2) ? 0 : u(0, 60));
        vfeValid = p(0.7);
        vfeNext = static_cast<float>(u(140, 230));
        vapp = static_cast<float>(u(120, 160));
        vmax = static_cast<float>(u(150, 350));
        vls = static_cast<float>(u(110, 160));
        vman = static_cast<float>(p(0.5) ? u(120, 250) : 0);
        v3 = static_cast<float>(p(0.5) ? u(120, 200) : 0);
        v4 = static_cast<float>(p(0.5) ? u(120, 220) : 0);
        static const int flapBits[]{11, 12, 13, 14, 14, 15, 0};
        flapBit = flapBits[rng() % 7];
        conf3Landing = p(0.5);
      }
      if (approachPhase) fms.fms_flight_phase = fmgc_flight_phase::Approach;
      normal(fms.tower_headwind_kts, towerHw);
      if (!towerValid) fms.tower_headwind_kts.SSM = 1;
      for (auto* ir : {&b.ir_own_bus, &b.ir_opp_bus, &b.ir_3_bus}) {
        normal(ir->wind_direction_true_deg, windDir);
        normal(ir->wind_speed_kn, windSpd);
      }
      fms.v_app_kts = vapp;
      fms.flap_3_approach_selected = conf3Landing;
      for (auto* fac : {&b.fac_own_bus, &b.fac_opp_bus}) {
        normal(fac->v_fe_next_kn, vfeNext);
        if (!vfeValid) fac->v_fe_next_kn.SSM = 1;
        normal(fac->v_max_kn, vmax);
        normal(fac->v_ls_kn, vls);
        normal(fac->v_man_kn, vman);
        if (vman == 0) fac->v_man_kn.SSM = 1;
        normal(fac->v_3_kn, v3);
        if (v3 == 0) fac->v_3_kn.SSM = 1;
        normal(fac->v_4_kn, v4);
        if (v4 == 0) fac->v_4_kn.SSM = 1;
        for (int bit = 11; bit <= 15; bit++) setBit(fac->discrete_word_5, bit, bit == flapBit);
      }
    }
    if (hdgPreset) {
      if (hdgLeft <= 0) {
        nextHdgIn -= dt;
        if (nextHdgIn <= 0) {
          hdgBit = p(0.5) ? 11 : 12;
          hdgLeft = u(0.05, 0.5);
          nextHdgIn = p(0.3) ? u(0.2, 1.5) : u(2, 20);
        }
      }
      if (hdgLeft > 0) {
        hdgLeft -= dt;
        setBit(b.fcu_bus.fcu_discrete_word_2, hdgBit, true);
      }
      toggleIn -= dt;
      if (toggleIn <= 0) {
        toggleIn = presetWhere != 0 ? u(0.3, 4) : u(0.5, 15);
        hdgSelValid = p(0.5);
        trkSelValid = p(0.3);
        hdgVs = p(0.6);
      }
      b.fcu_bus.selected_hdg_deg.SSM = hdgSelValid ? 3 : 1;
      b.fcu_bus.selected_trk_deg.SSM = trkSelValid ? 3 : 1;
      setBit(b.fcu_bus.fcu_discrete_word_1, 24, hdgVs);
      setBit(b.fcu_bus.fcu_discrete_word_1, 25, !hdgVs);
    }
    if (restart && window(restartAt, restartLen)) {
      in.sim_data.computer_running = false;
    }
    auto onGround = [&](bool g) {
      for (auto* w : {&b.fac_own_bus.discrete_word_5, &b.fac_opp_bus.discrete_word_5}) {
        w->SSM = 3;
        setBit(*w, 17, false);
        setBit(*w, 18, g);
        setBit(*w, 19, g);
      }
    };
    if (takeoff) {
      // on the ground, AP/FD off (modes reset), then FDs on with V2, flaps 1, TOGA; lift-off later
      bool onGnd = te < liftoffAt;
      onGround(onGnd);
      for (auto* ra : {&b.ra_own_bus.radio_height_ft, &b.ra_opp_bus.radio_height_ft}) {
        normal(*ra, onGnd ? 0.0F : static_cast<float>(10 * (te - liftoffAt)));
      }
      if (te < togaAt - 2) {
        setBit(b.fcu_bus.fcu_discrete_word_2, 26, true);
        setBit(b.fcu_bus.fcu_discrete_word_2, 27, true);
        di.fd_opp_engaged = false;
        di.ap_opp_engaged = false;
        di.ap_instinctive_disc = true;
      }
      fms.v_2_kts = te < togaAt - 1 ? 0 : 140;
      for (auto* fac : {&b.fac_own_bus, &b.fac_opp_bus}) {
        // flap lever index 2 (CONF 1+F)
        for (int bit = 11; bit <= 15; bit++) setBit(fac->discrete_word_5, bit, bit == 12);
      }
      if (te >= togaAt) {
        normal(b.fadec_own_bus.selected_tla_deg, 45.0F);
        normal(b.fadec_opp_bus.selected_tla_deg, 45.0F);
      }
      fms.fms_flight_phase = te < togaAt ? fmgc_flight_phase::Preflight : fmgc_flight_phase::Takeoff;
      if (takeoffEngineOut && te >= takeoffEoAt) di.eng_opp_stop = true;
      if (liftoffNoManaged && te >= liftoffAt + 4.5) fms.v_managed_kts = 0;
    }
    if (presetWhere != 0) {
      // NAV armed: HDG knob pulled, then pushed with no NAV capture; on the ground or below 30 ft
      fms.nav_capture_condition = false;
      if (presetWhere == 1) onGround(true);
      for (auto* ra : {&b.ra_own_bus.radio_height_ft, &b.ra_opp_bus.radio_height_ft}) {
        normal(*ra, presetWhere == 1 ? 0.0F : presetRa);
      }
      if (window(pullAt, 0.3)) setBit(b.fcu_bus.fcu_discrete_word_2, 12, true);
      if (window(pushAt, 0.3)) setBit(b.fcu_bus.fcu_discrete_word_2, 11, true);
    }
    if (raSnap) {
      float ra = b.ra_own_bus.radio_height_ft.Data;
      if (ra > 650 && ra < 750) {
        b.ra_own_bus.radio_height_ft.Data = 700.0F;
        b.ra_opp_bus.radio_height_ft.Data = 700.0F;
      }
    }
    if (soft) {
      // cruise at the recorded altitude; Mach requested at the start; the managed speed on the (integer) CAS
      fms.fms_flight_phase = fmgc_flight_phase::Cruise;
      if (te < 0.2) fms.fms_mach_mode_activate = true;
      cas = std::round(cas);
      for (auto* adr : {&b.adr_own_bus, &b.adr_opp_bus, &b.adr_3_bus}) adr->airspeed_computed_kn.Data = cas;
      float off = window(softOffAt, softOffLen) ? softOffKt : softOffset;
      fms.v_managed_kts = cas + off;
      fms.show_speed_margins = false;
      float vmaxNow = window(softVmaxAt, 8) ? cas + softVmaxOffset : cas + 60;
      for (auto* fac : {&b.fac_own_bus, &b.fac_opp_bus}) normal(fac->v_max_kn, vmaxNow);
      if (window(softBit15At, 0.3)) setBit(b.fcu_bus.fcu_discrete_word_1, 15, true);
      if (te >= softExitEnd) planSoftExit(te);
    }
  }
};
