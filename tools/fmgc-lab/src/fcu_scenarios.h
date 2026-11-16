// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Input injectors for the FCU differential check (src/check_fcu.cpp), applied on top of the drive (src/fcu_drive.h) at
// every step, in this order: --script, --panel, --priority, --bus, --sim, --power. Each one is reproducible from its
// seed. The generated FCU is the oracle, so any input is a valid test input; inconsistent combinations are wanted.
//
//  --script SEED   a scripted tour of every control, one action every 0.6-2.5 s, cycling through the list below with
//                  turn sizes / hold times / values varied per cycle (the seed shifts the start, spacing and values):
//                  SPD/HDG/ALT/V-S knob turns (+-1, +-2, +-5, multi-frame, +-127/-128), push, pull (1-5 frames);
//                  SPD/MACH, TRK/FPA, METRIC, LOC, APPR, EXPED pbs; ALT 100/1000; per EFIS panel: FD (1 frame and
//                  held), LS, the five filters, baro push/pull/turns, hPa/inHg, ND mode, range (incl. 320), navaids;
//                  sim_input sets (SPD/MACH, HDG/TRK, ALT, V/S-FPA, both baros; including odd values: 359.6 deg,
//                  12345 ft, 0, -1 = none, out of range); lights test; QFE pin programming.
//  --panel SEED    the same actions at random (about one every 1.5 s, sometimes several at once).
//  --priority SEED episodes of AP/FD/A/THR engaged discretes of both FMGCs forced to random combinations (FMGC priority
//                  changes, any AP/FD engaged changes, AP engagement edges).
//  --bus SEED      episodes on the FMGC A buses: a random word the FCU reads (discrete words 1-7, ATS words, CAS,
//                  Mach, heading, track, altitude, V/S, FPA, presets, altitude constraint, N1, flex) with a random SSM,
//                  a bit set/cleared or a random value; and structured ones: end of TCAS into V/S-FPA with the RA bits
//                  (24, 27, 28) varied, V/S knob push followed by V/S-FPA engagement within 0-1.5 s (level-off), SRS
//                  TO/GA and TCAS on FMGC 1 (TRK/FPA reset, FD auto on), FD auto disengage, FMGC Mach mode, managed <->
//                  selected speed and lateral, HDG preset, MCDU speed/Mach preset with random preset values.
//  --sim SEED      pause_on/slew_on/tracking/tailstrike flags toggled (unused by the model), dt = 0 frames, simulation
//                  time steps backwards or forwards.
//  --power SEED    computer_running false for 1 frame, 2-10 frames, 0.2-5 s or 5-60 s, flicker, late power-up.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <random>
#include <vector>

#include "FcuComputer_types.h"
#include "fcu_drive.h"

namespace fcu_scenarios {

class Rng {
 public:
  explicit Rng(uint64_t seed) : rng_(seed) {}
  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng_); }
  bool chance(double p) { return u(0, 1) < p; }
  int irand(int n) { return static_cast<int>(rng_() % static_cast<uint64_t>(n)); }
  uint64_t raw() { return rng_(); }

 private:
  std::mt19937_64 rng_;
};

inline void setBit(base_arinc_429& w, int bit, bool v) {
  float r = std::round(w.Data);
  uint32_t x = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  x = v ? (x | (1U << (bit - 1))) : (x & ~(1U << (bit - 1)));
  w.Data = static_cast<float>(x);
}

// ---------------------------------------------------------------------------------------------------------------------
// Panel actions (shared by --script and --panel)
// ---------------------------------------------------------------------------------------------------------------------

enum ActionKind : int {
  SpdTurn = 0, SpdPush, SpdPull, SpdMachPb, HdgTurn, HdgPush, HdgPull, TrkFpaPb, AltTurn, AltPush, AltPull,
  AltIncrement, MetricPb, VsTurn, VsPush, VsPull, LocPb, ApprPb, ExpedPb,
  EfisFd, EfisLs, EfisCstr, EfisWpt, EfisVord, EfisNdb, EfisArpt, BaroPush, BaroPull, BaroTurn, BaroUnit, EfisRange,
  EfisMode, Navaid1, Navaid2,
  SimSpd, SimHdg, SimAlt, SimVs, SimBaroLeft, SimBaroRight, LightsTest, QfePin,
  kActionKinds
};

struct Action {
  int kind;
  int side;      // EFIS panel 0 = captain, 1 = F/O
  int value;     // turns, or selector value
  float fvalue;  // sim_input value
  int frames;    // how many frames the input is applied
};

/** Applies active actions to the inputs each frame and keeps the panel's level selectors once changed. */
class PanelDriver {
 public:
  void start(const Action& a) {
    active_.push_back(a);
    started_++;
    counts_[a.kind]++;
  }

  void apply(fcu_inputs& in) {
    auto& d = in.discrete_inputs;
    // level selectors set by an earlier action
    for (int s = 0; s < 2; s++) {
      auto& e = s == 0 ? d.capt_efis_inputs : d.fo_efis_inputs;
      if (rangeSet_[s]) e.efis_range = static_cast<efis_range_selection>(range_[s]);
      if (modeSet_[s]) e.efis_mode = static_cast<efis_mode_selection>(mode_[s]);
      if (nav1Set_[s]) e.efis_navaid_1 = static_cast<efis_navaid_selection>(nav1_[s]);
      if (nav2Set_[s]) e.efis_navaid_2 = static_cast<efis_navaid_selection>(nav2_[s]);
      if (inhgSet_[s]) e.baro_is_inhg = inhg_[s];
    }
    if (incrementSet_) d.afs_inputs.alt_increment_1000 = increment1000_;
    if (lightsSet_) d.lights_test = lights_;
    if (qfeSet_) d.pin_prog_qfe_avail = qfe_;

    for (size_t i = 0; i < active_.size();) {
      Action& a = active_[i];
      act(a, in);
      if (--a.frames <= 0) {
        active_[i] = active_.back();
        active_.pop_back();
      } else {
        i++;
      }
    }
  }

  long started() const { return started_; }
  long count(int kind) const { return counts_[kind]; }

 private:
  void act(const Action& a, fcu_inputs& in) {
    auto& d = in.discrete_inputs;
    auto& afs = d.afs_inputs;
    auto& e = a.side == 0 ? d.capt_efis_inputs : d.fo_efis_inputs;
    const int8_T turns = static_cast<int8_T>(a.value);
    switch (a.kind) {
      case SpdTurn: afs.spd_knob.turns = turns; break;
      case SpdPush: afs.spd_knob.pushed = true; break;
      case SpdPull: afs.spd_knob.pulled = true; break;
      case SpdMachPb: afs.spd_mach_button_pressed = true; break;
      case HdgTurn: afs.hdg_trk_knob.turns = turns; break;
      case HdgPush: afs.hdg_trk_knob.pushed = true; break;
      case HdgPull: afs.hdg_trk_knob.pulled = true; break;
      case TrkFpaPb: afs.trk_fpa_button_pressed = true; break;
      case AltTurn: afs.alt_knob.turns = turns; break;
      case AltPush: afs.alt_knob.pushed = true; break;
      case AltPull: afs.alt_knob.pulled = true; break;
      case AltIncrement:
        incrementSet_ = true;
        increment1000_ = a.value != 0;
        afs.alt_increment_1000 = increment1000_;
        break;
      case MetricPb: afs.metric_alt_button_pressed = true; break;
      case VsTurn: afs.vs_fpa_knob.turns = turns; break;
      case VsPush: afs.vs_fpa_knob.pushed = true; break;
      case VsPull: afs.vs_fpa_knob.pulled = true; break;
      case LocPb: afs.loc_button_pressed = true; break;
      case ApprPb: afs.appr_button_pressed = true; break;
      case ExpedPb: afs.exped_button_pressed = true; break;
      case EfisFd: e.fd_button_pushed = true; break;
      case EfisLs: e.ls_button_pushed = true; break;
      case EfisCstr: e.cstr_button_pushed = true; break;
      case EfisWpt: e.wpt_button_pushed = true; break;
      case EfisVord: e.vord_button_pushed = true; break;
      case EfisNdb: e.ndb_button_pushed = true; break;
      case EfisArpt: e.arpt_button_pushed = true; break;
      case BaroPush: e.baro_knob.pushed = true; break;
      case BaroPull: e.baro_knob.pulled = true; break;
      case BaroTurn: e.baro_knob.turns = turns; break;
      case BaroUnit:
        inhgSet_[a.side] = true;
        inhg_[a.side] = a.value != 0;
        e.baro_is_inhg = inhg_[a.side];
        break;
      case EfisRange:
        rangeSet_[a.side] = true;
        range_[a.side] = a.value;
        e.efis_range = static_cast<efis_range_selection>(a.value);
        break;
      case EfisMode:
        modeSet_[a.side] = true;
        mode_[a.side] = a.value;
        e.efis_mode = static_cast<efis_mode_selection>(a.value);
        break;
      case Navaid1:
        nav1Set_[a.side] = true;
        nav1_[a.side] = a.value;
        e.efis_navaid_1 = static_cast<efis_navaid_selection>(a.value);
        break;
      case Navaid2:
        nav2Set_[a.side] = true;
        nav2_[a.side] = a.value;
        e.efis_navaid_2 = static_cast<efis_navaid_selection>(a.value);
        break;
      case SimSpd: in.sim_input.spd_mach = a.fvalue; break;
      case SimHdg: in.sim_input.hdg_trk = a.fvalue; break;
      case SimAlt: in.sim_input.alt = a.fvalue; break;
      case SimVs: in.sim_input.vs_fpa = a.fvalue; break;
      case SimBaroLeft: in.sim_input.left_baro_setting_hpa = a.fvalue; break;
      case SimBaroRight: in.sim_input.right_baro_setting_hpa = a.fvalue; break;
      case LightsTest:
        lightsSet_ = true;
        lights_ = a.value != 0;
        d.lights_test = lights_;
        break;
      case QfePin:
        qfeSet_ = true;
        qfe_ = a.value != 0;
        d.pin_prog_qfe_avail = qfe_;
        break;
      default: break;
    }
  }

  std::vector<Action> active_;
  bool rangeSet_[2]{}, modeSet_[2]{}, nav1Set_[2]{}, nav2Set_[2]{}, inhgSet_[2]{};
  int range_[2]{}, mode_[2]{}, nav1_[2]{}, nav2_[2]{};
  bool inhg_[2]{};
  bool incrementSet_ = false, increment1000_ = false, lightsSet_ = false, lights_ = false, qfeSet_ = false, qfe_ = false;
  long started_ = 0;
  long counts_[kActionKinds]{};
};

/** A random action of the given kind; `cycle` varies the sizes deterministically for the script. */
inline Action makeAction(int kind, Rng& r, int cycle) {
  Action a{kind, r.irand(2), 0, -1.0F, 1};
  static const int turnSizes[] = {1, -1, 2, -2, 5, -5, 1, -1, 127, -128, 3, -3, 10, -10};
  const int nTurnSizes = sizeof(turnSizes) / sizeof(turnSizes[0]);
  switch (kind) {
    case SpdTurn:
    case HdgTurn:
    case AltTurn:
    case VsTurn:
    case BaroTurn:
      a.value = turnSizes[(cycle + r.irand(nTurnSizes)) % nTurnSizes];
      if (std::abs(a.value) > 10 && r.chance(0.7)) a.value = a.value > 0 ? 1 : -1;
      a.frames = r.chance(0.6) ? 1 : 2 + r.irand(30);  // single click or a sustained turn
      break;
    case SpdPush: case SpdPull: case SpdMachPb: case HdgPush: case HdgPull: case TrkFpaPb: case AltPush: case AltPull:
    case MetricPb: case VsPush: case VsPull: case LocPb: case ApprPb: case ExpedPb: case EfisLs: case EfisCstr:
    case EfisWpt: case EfisVord: case EfisNdb: case EfisArpt: case BaroPush: case BaroPull:
      a.frames = r.chance(0.7) ? 1 : 2 + r.irand(8);
      break;
    case EfisFd:
      a.frames = r.chance(0.8) ? 1 : 2 + r.irand(4);  // a held FD button toggles every frame
      break;
    case AltIncrement: case BaroUnit: case LightsTest: case QfePin:
      a.value = r.irand(2);
      break;
    case EfisRange:
      a.value = r.chance(0.97) ? r.irand(6) : 6 + r.irand(3);  // incl. out-of-range enum values
      break;
    case EfisMode:
      a.value = r.chance(0.97) ? r.irand(5) : 5 + r.irand(3);
      break;
    case Navaid1: case Navaid2:
      a.value = r.chance(0.97) ? r.irand(3) : 3 + r.irand(3);
      break;
    case SimSpd: {
      static const float v[] = {250.0F, 78.0F, 140.0F, 0.0F, 99.0F, 400.0F, 82.0F, 300.5F, -1.0F, 1000.0F};
      a.fvalue = r.chance(0.8) ? v[(cycle + r.irand(10)) % 10] : static_cast<float>(r.u(-50, 500));
      break;
    }
    case SimHdg: {
      static const float v[] = {0.0F, 359.6F, 359.4F, 180.0F, 90.5F, 360.0F, -0.4F, 270.0F, -1.0F, 721.0F};
      a.fvalue = r.chance(0.8) ? v[(cycle + r.irand(10)) % 10] : static_cast<float>(r.u(-400, 800));
      break;
    }
    case SimAlt: {
      static const float v[] = {12345.0F, 5000.0F, 100.0F, 0.0F, 49000.0F, 60000.0F, 35050.0F, 4950.0F, -1.0F, 10.0F};
      a.fvalue = r.chance(0.8) ? v[(cycle + r.irand(10)) % 10] : static_cast<float>(r.u(-1000, 60000));
      break;
    }
    case SimVs: {
      static const float v[] = {2000.0F, -1500.0F, 0.0F, 6500.0F, -7000.0F, 35.0F, -33.0F, 99.0F, -1.0F, 150.0F};
      a.fvalue = r.chance(0.8) ? v[(cycle + r.irand(10)) % 10] : static_cast<float>(r.u(-8000, 8000));
      break;
    }
    case SimBaroLeft:
    case SimBaroRight: {
      static const float v[] = {1013.25F, 1013.0F, 990.0F, 745.0F, 1100.0F, 700.0F, 1200.0F, 1029.0F, -1.0F, 0.0F};
      a.fvalue = r.chance(0.8) ? v[(cycle + r.irand(10)) % 10] : static_cast<float>(r.u(900, 1060));
      break;
    }
    default:
      break;
  }
  return a;
}

/** --script: one action every 0.6-2.5 s, cycling through every kind. */
class ScriptScenario {
 public:
  explicit ScriptScenario(uint64_t seed) : rng_(seed) {
    next_ = rng_.u(0.5, 5);
    kind_ = rng_.irand(kActionKinds);
  }
  void apply(fcu_inputs& in, double dt) {
    t_ += dt;
    if (t_ >= next_) {
      driver_.start(makeAction(kind_, rng_, cycle_));
      if (++kind_ == kActionKinds) {
        kind_ = 0;
        cycle_++;
      }
      next_ = t_ + rng_.u(0.6, 2.5);
    }
    driver_.apply(in);
  }
  long episodes() const { return driver_.started(); }

 private:
  Rng rng_;
  PanelDriver driver_;
  double t_ = 0, next_ = 0;
  int kind_ = 0, cycle_ = 0;
};

/** --panel: random actions. */
class RandomPanelScenario {
 public:
  explicit RandomPanelScenario(uint64_t seed) : rng_(seed) {}
  void apply(fcu_inputs& in, double dt) {
    if (rng_.u(0, 1) < dt / 1.5) {
      int n = rng_.chance(0.85) ? 1 : 2 + rng_.irand(3);
      for (int i = 0; i < n; i++) {
        driver_.start(makeAction(rng_.irand(kActionKinds), rng_, rng_.irand(1000)));
      }
    }
    driver_.apply(in);
  }
  long episodes() const { return driver_.started(); }

 private:
  Rng rng_;
  PanelDriver driver_;
};

// ---------------------------------------------------------------------------------------------------------------------
// --priority
// ---------------------------------------------------------------------------------------------------------------------

class PriorityScenario {
 public:
  explicit PriorityScenario(uint64_t seed) : rng_(seed) { next_ = rng_.u(2, 60); }
  void apply(fcu_inputs& in, double dt) {
    t_ += dt;
    if (left_ <= 0 && t_ >= next_) {
      left_ = rng_.chance(0.3) ? rng_.u(0.05, 1) : rng_.u(1, 40);
      mask_ = static_cast<int>(rng_.raw() & 63);
      which_ = static_cast<int>(rng_.raw() & 63);  // which discretes are forced (others recorded)
      if (which_ == 0) which_ = 63;
      episodes_++;
      next_ = t_ + left_ + rng_.u(2, 90);
    }
    if (left_ > 0) {
      left_ -= dt;
      auto& d = in.discrete_inputs;
      boolean_T* f[6] = {&d.ap_1_engaged, &d.fd_1_engaged, &d.athr_1_engaged, &d.ap_2_engaged, &d.fd_2_engaged,
                    &d.athr_2_engaged};
      for (int i = 0; i < 6; i++) {
        if (which_ >> i & 1) *f[i] = (mask_ >> i) & 1;
      }
    }
  }
  long episodes() const { return episodes_; }

 private:
  Rng rng_;
  double t_ = 0, next_ = 0, left_ = 0;
  int mask_ = 0, which_ = 0;
  long episodes_ = 0;
};

// ---------------------------------------------------------------------------------------------------------------------
// --bus
// ---------------------------------------------------------------------------------------------------------------------

class BusScenario {
 public:
  enum Kind : int {
    WordSsm = 0, WordBit, WordValue, TcasEndToVs, LevelOffPush, SrsOrTcasOnFmgc1, FdAutoDisengage, MachMode,
    SpeedManagedSelected, LateralModes, Preset, kKinds
  };

  explicit BusScenario(uint64_t seed) : rng_(seed) { next_ = rng_.u(1, 20); }

  void apply(fcu_inputs& in, double dt) {
    t_ += dt;
    if (t_ >= next_ && active_.size() < 3) {
      start();
      next_ = t_ + rng_.u(0.5, 15);
    }
    for (size_t i = 0; i < active_.size();) {
      Episode& e = active_[i];
      e.age += dt;
      run(e, in);
      if (e.age >= e.length) {
        active_[i] = active_.back();
        active_.pop_back();
      } else {
        i++;
      }
    }
  }
  long episodes() const { return episodes_; }
  long count(int k) const { return counts_[k]; }

 private:
  struct Episode {
    int kind;
    int bus;    // 0 = FMGC 1, 1 = FMGC 2, 2 = both
    int word;   // index of the word in base_fmgc_a_bus
    int bit;
    uint32_t ssm;
    float value;
    double age = 0, length = 0, split = 0;  // split: phase change time
    int bits;   // random bit pattern for structured episodes
  };

  // words the FCU reads, as indices into base_fmgc_a_bus (an array of base_arinc_429)
  static int wordIndex(const base_arinc_429 base_fmgc_a_bus::*m) {
    base_fmgc_a_bus b{};
    return static_cast<int>(&(b.*m) - reinterpret_cast<base_arinc_429*>(&b));
  }
  static const std::vector<int>& readWords() {
    static const std::vector<int> w = {
        wordIndex(&base_fmgc_a_bus::discrete_word_1), wordIndex(&base_fmgc_a_bus::discrete_word_2),
        wordIndex(&base_fmgc_a_bus::discrete_word_3), wordIndex(&base_fmgc_a_bus::discrete_word_4),
        wordIndex(&base_fmgc_a_bus::discrete_word_5), wordIndex(&base_fmgc_a_bus::discrete_word_7),
        wordIndex(&base_fmgc_a_bus::ats_discrete_word), wordIndex(&base_fmgc_a_bus::ats_fma_discrete_word),
        wordIndex(&base_fmgc_a_bus::cas_kts), wordIndex(&base_fmgc_a_bus::mach),
        wordIndex(&base_fmgc_a_bus::heading_deg), wordIndex(&base_fmgc_a_bus::track_deg),
        wordIndex(&base_fmgc_a_bus::altitude_ft), wordIndex(&base_fmgc_a_bus::vertical_speed_ft_min),
        wordIndex(&base_fmgc_a_bus::fpa_deg), wordIndex(&base_fmgc_a_bus::preset_mach_from_mcdu),
        wordIndex(&base_fmgc_a_bus::preset_speed_from_mcdu_kts), wordIndex(&base_fmgc_a_bus::fm_alt_constraint_ft),
        wordIndex(&base_fmgc_a_bus::n1_command_percent), wordIndex(&base_fmgc_a_bus::flx_to_temp_deg_c)};
    return w;
  }
  static base_arinc_429& word(base_fmgc_a_bus& b, int i) { return reinterpret_cast<base_arinc_429*>(&b)[i]; }

  void start() {
    Episode e{};
    e.kind = rng_.irand(kKinds);
    e.bus = rng_.irand(3);
    e.length = rng_.chance(0.3) ? rng_.u(0.05, 1) : rng_.u(1, 25);
    e.word = readWords()[rng_.irand(static_cast<int>(readWords().size()))];
    e.bit = 11 + rng_.irand(19);
    e.ssm = static_cast<uint32_t>(rng_.irand(4));
    e.bits = static_cast<int>(rng_.raw() & 0xffff);
    if (e.word >= wordIndex(&base_fmgc_a_bus::cas_kts) || rng_.chance(0.1)) {
      static const float special[] = {0.0F, -1.0F, 0.78F, 250.0F, 359.9F, 35000.0F, -2000.0F, 1e9F, -1e9F, 0.5F};
      e.value = rng_.chance(0.5) ? special[rng_.irand(10)] : static_cast<float>(rng_.u(-500, 50000));
      if (rng_.chance(0.02)) e.value = std::nanf("");
    } else {
      e.value = static_cast<float>(rng_.raw() & 0x1ffffc00U);  // bits 11-29
    }
    e.split = rng_.u(0.0, 1.5);
    if (e.kind == TcasEndToVs || e.kind == LevelOffPush) {
      e.length = e.split + rng_.u(1, 12);
    }
    if (e.kind == TcasEndToVs) {
      e.split = rng_.u(0.5, 10);
      e.length = e.split + rng_.u(0.0, 1.0) + rng_.u(1, 10);
    }
    active_.push_back(e);
    episodes_++;
    counts_[e.kind]++;
  }

  void forEachBus(const Episode& e, fcu_inputs& in, const std::function<void(base_fmgc_a_bus&)>& f) {
    if (e.bus != 1) f(in.bus_inputs.fmgc_1_bus);
    if (e.bus != 0) f(in.bus_inputs.fmgc_2_bus);
  }

  void run(Episode& e, fcu_inputs& in) {
    const uint32_t kNormal = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
    switch (e.kind) {
      case WordSsm:
        forEachBus(e, in, [&](base_fmgc_a_bus& b) { word(b, e.word).SSM = e.ssm; });
        break;
      case WordBit:
        forEachBus(e, in, [&](base_fmgc_a_bus& b) { setBit(word(b, e.word), e.bit, e.bits & 1); });
        break;
      case WordValue:
        forEachBus(e, in, [&](base_fmgc_a_bus& b) {
          word(b, e.word).Data = e.value;
          if (e.bits & 2) word(b, e.word).SSM = kNormal;
        });
        break;
      case TcasEndToVs:
        // TCAS (dw7 b13) with RA bits, then V/S or FPA (dw1 b17/18) within up to 1 s
        forEachBus(e, in, [&](base_fmgc_a_bus& b) {
          base_arinc_429& w7 = b.discrete_word_7;
          w7.SSM = kNormal;
          setBit(w7, 13, e.age < e.split);
          setBit(w7, 24, e.bits & 1);
          setBit(w7, 27, e.bits & 2);
          setBit(w7, 28, e.bits & 4);
          const double vsAt = e.split + ((e.bits >> 3 & 7) / 7.0);
          if (e.age >= vsAt) {
            base_arinc_429& w1 = b.discrete_word_1;
            w1.SSM = kNormal;
            setBit(w1, (e.bits & 64) ? 18 : 17, true);
            if (e.bits & 128) b.altitude_ft = {kNormal, 31000.0F};
          }
        });
        break;
      case LevelOffPush:
        // V/S knob push at the start, V/S-FPA engaged after `split` s
        if (e.age == 0 || e.age <= in.time.dt) in.discrete_inputs.afs_inputs.vs_fpa_knob.pushed = true;
        if (e.age >= e.split) {
          forEachBus(e, in, [&](base_fmgc_a_bus& b) {
            b.discrete_word_1.SSM = kNormal;
            setBit(b.discrete_word_1, (e.bits & 1) ? 18 : 17, true);
          });
        }
        break;
      case SrsOrTcasOnFmgc1: {
        base_arinc_429& w = (e.bits & 3) == 0 ? in.bus_inputs.fmgc_1_bus.discrete_word_7
                                              : in.bus_inputs.fmgc_1_bus.discrete_word_1;
        setBit(w, (e.bits & 3) == 0 ? 13 : ((e.bits & 3) == 1 ? 15 : 16), true);
        break;
      }
      case FdAutoDisengage:
        setBit(in.bus_inputs.fmgc_1_bus.discrete_word_4, 27, true);
        break;
      case MachMode:
        setBit(in.bus_inputs.fmgc_1_bus.ats_discrete_word, 19, e.bits & 1);
        break;
      case SpeedManagedSelected:
        forEachBus(e, in, [&](base_fmgc_a_bus& b) {
          b.discrete_word_5.SSM = (e.bits & 4) ? e.ssm : kNormal;
          setBit(b.discrete_word_5, 19, e.bits & 1);
          setBit(b.discrete_word_5, 20, e.bits & 2);
        });
        break;
      case LateralModes:
        forEachBus(e, in, [&](base_fmgc_a_bus& b) {
          static const int dw2Bits[] = {11, 12, 13, 14, 15, 16, 17};
          for (int i = 0; i < 7; i++) setBit(b.discrete_word_2, dw2Bits[i], (e.bits >> i) & 1);
          setBit(b.discrete_word_3, 14, e.bits >> 7 & 1);
          setBit(b.discrete_word_3, 16, e.bits >> 8 & 1);
          setBit(b.discrete_word_3, 20, e.bits >> 9 & 1);
          setBit(b.discrete_word_4, 14, e.bits >> 10 & 1);
          if (e.bits >> 11 & 1) b.discrete_word_2.SSM = e.ssm;
        });
        break;
      case Preset:
        forEachBus(e, in, [&](base_fmgc_a_bus& b) {
          setBit(b.discrete_word_3, 11, e.bits & 1);
          setBit(b.discrete_word_5, 28, e.bits & 2);
          b.preset_speed_from_mcdu_kts = {(e.bits & 4) ? kNormal : e.ssm, (e.bits & 8) ? e.value : 220.0F};
          b.preset_mach_from_mcdu = {(e.bits & 16) ? kNormal : e.ssm, (e.bits & 32) ? e.value : 0.76F};
        });
        break;
      default:
        break;
    }
  }

  Rng rng_;
  double t_ = 0, next_ = 0;
  std::vector<Episode> active_;
  long episodes_ = 0;
  long counts_[kKinds]{};
};

// ---------------------------------------------------------------------------------------------------------------------
// --sim
// ---------------------------------------------------------------------------------------------------------------------

class SimScenario {
 public:
  explicit SimScenario(uint64_t seed) : rng_(seed) { next_ = rng_.u(1, 30); }
  void apply(fcu_inputs& in, double dt) {
    t_ += dt;
    if (t_ >= next_) {
      switch (rng_.irand(4)) {
        case 0:
          flags_ = static_cast<int>(rng_.raw() & 15);
          flagsLeft_ = rng_.u(0.05, 20);
          break;
        case 1:
          zeroDtFrames_ = 1 + rng_.irand(5);
          break;
        case 2:
          timeOffset_ += rng_.chance(0.5) ? -rng_.u(0.01, 100) : rng_.u(0.01, 100);
          break;
        default:
          dtScale_ = rng_.u(0.1, 4);
          dtScaleLeft_ = rng_.u(0.5, 10);
          break;
      }
      episodes_++;
      next_ = t_ + rng_.u(2, 40);
    }
    if (flagsLeft_ > 0) {
      flagsLeft_ -= dt;
      in.sim_data.slew_on = flags_ & 1;
      in.sim_data.pause_on = flags_ & 2;
      in.sim_data.tracking_mode_on_override = flags_ & 4;
      in.sim_data.tailstrike_protection_on = flags_ & 8;
    }
    if (zeroDtFrames_ > 0) {
      zeroDtFrames_--;
      in.time.dt = 0.0;
    }
    if (dtScaleLeft_ > 0) {
      dtScaleLeft_ -= dt;
      in.time.dt *= dtScale_;
    }
    in.time.simulation_time += timeOffset_;
  }
  long episodes() const { return episodes_; }

 private:
  Rng rng_;
  double t_ = 0, next_ = 0, flagsLeft_ = 0, timeOffset_ = 0, dtScale_ = 1, dtScaleLeft_ = 0;
  int flags_ = 0, zeroDtFrames_ = 0;
  long episodes_ = 0;
};

// ---------------------------------------------------------------------------------------------------------------------
// --power
// ---------------------------------------------------------------------------------------------------------------------

class PowerScenario {
 public:
  explicit PowerScenario(uint64_t seed) : rng_(seed) {
    lateStart_ = rng_.chance(0.3) ? rng_.u(0.1, 20) : 0;
    next_ = lateStart_ + rng_.u(5, 120);
  }
  void apply(fcu_inputs& in, double dt) {
    t_ += dt;
    bool off = t_ < lateStart_;
    if (left_ <= 0 && framesLeft_ <= 0 && t_ >= next_) {
      kind_ = rng_.irand(5);
      switch (kind_) {
        case 0: framesLeft_ = 1; break;
        case 1: framesLeft_ = 2 + rng_.irand(9); break;
        case 2: left_ = rng_.u(0.2, 5); break;
        case 3: left_ = rng_.u(5, 60); break;
        default:
          left_ = rng_.u(0.5, 3);
          period_ = 1 + rng_.irand(5);
          break;
      }
      frame_ = 0;
      episodes_++;
      next_ = t_ + rng_.u(20, 240);
    }
    if (framesLeft_ > 0) {
      framesLeft_--;
      off = true;
    } else if (left_ > 0) {
      left_ -= dt;
      off = kind_ == 4 ? ((frame_++ / period_) % 2 == 0) : true;
    }
    if (off) {
      in.sim_data.computer_running = false;
      offFrames_++;
    }
  }
  long episodes() const { return episodes_; }
  long offFrames() const { return offFrames_; }

 private:
  Rng rng_;
  double t_ = 0, next_ = 0, left_ = 0, lateStart_ = 0;
  long framesLeft_ = 0, frame_ = 0, episodes_ = 0, offFrames_ = 0;
  int kind_ = 0, period_ = 1;
};

}  // namespace fcu_scenarios
