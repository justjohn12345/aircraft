// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Fuzz input encoding: fuzzer bytes -> a base (snapshot, start offset, length, time scale) and a sequence of timed
// events from the scenario-injector vocabulary, applied on top of the snapshot's recorded inputs.
//
//   header, 8 bytes:
//     [0..1] snapshot index (little endian, modulo the bank size)
//     [2]    start offset in the snapshot's window (x / 256 of the window; the model state is the snapshot's, so a
//            non-zero offset joins the recording later: a "jump", a valid input like any other)
//     [3]    length: 32 + 8 x value steps (32..2072)
//     [4]    bits 0-1: time scale - 1 (1..4 window frames per step, dt summed); other bits reserved
//     [5..7] global seed (noise, scenario seeds)
//   events, 8 bytes each (trailing partial record ignored, at most kMaxEvents):
//     [0] gap: the event starts 2 x gap steps after the previous one
//     [1] duration: d < 128 ? d + 1 : 128 + 16 x (d - 128) steps (1..2160)
//     [2] kind (modulo kKindCount)
//     [3] a, [4..5] b (LE), [6..7] c (LE): kind-specific parameters
//
// Past the end of the window the last frame's inputs are held (as the replay does in active pause). Every byte string
// decodes to a valid case, and the same bytes always give the same run (all randomness is seeded from the bytes).

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "FmgcComputer_types.h"
#include "field_table.h"
#include "scenario_approach.h"
#include "scenario_athr.h"
#include "scenario_conditioning.h"
#include "scenario_engagement.h"
#include "scenario_lateral.h"
#include "scenario_targets.h"
#include "scenario_tcas.h"

namespace fmgc::fuzz {

enum class Kind : uint8_t {
  FcuPush,        // a: button (kFcuButtons); pulse for the duration
  FcuHdgVsTrkFpa, // a&1: 1 = HDG-V/S (word 1 bit 24), 0 = TRK-FPA (bit 25), held for the duration
  FcuAltitude,    // a&1: relative (H + int16 b, rounded to 100 ft) or absolute (b x 10 ft); + ALT value changed pulse
  FcuVs,          // selected V/S = int16 b rounded to 100 fpm (+-6000), FPA = int8 a / 10 deg
  FcuHeading,     // selected heading and track = b mod 360; a&3 = SSM of both
  FcuSpeed,       // selected speed = 100 + b mod 300 kt, Mach = 0.10 + (c mod 80) / 100; a&3 = SSM
  Buttons,        // a: AP pb, A/THR pb, AP instinctive disc, A/THR instinctive disc, FD own off, both FDs off,
                  //    ELAC own AP disc, ELAC opp AP disc
  FmsConstraint,  // next_alt_cstr_ft: a&1 ? 0 : H + int16 b rounded to 100 ft
  FmsPhase,       // fms_flight_phase = a mod 8
  FmsSpeed,       // v_managed: a&1 ? 0 : 120 + b mod 250; a bits 1/2: FMS speed / Mach mode request pulses
  FmsFlag,        // a mod 10 selects a bool (fm_valid, lat fp valid, vert fp valid, nav capture, DIR TO, FINAL can
                  //    engage, speed margins, preset activate, back beam, flap 3 approach); b&1 = value
  FmsValue,       // a mod 10 selects des submode, approach type, ACC ALT, THR RED ALT, V2, VAPP, cruise alt, bank
                  //    limit, xtk, track error; from b/c
  Ssm,            // SSM field (b mod n) = a mod 4
  Discrete,       // discrete input (b mod n) = !recorded
  Field,          // any input field (b mod n): a mod 6 = zero, negate, +-5 %, +-int16 c, x (c/32768+0.5), = int16 c
  Noise,          // float input field (b mod n) + uniform noise of amplitude max(1, |x|) x c / 65536 every step
  Power,          // computer not running for the duration (power cycle / self test)
  Levers,         // TLA: a&3 selects both / own / opp / asymmetric, b indexes an angle table; c&3 = SSM of one
  RadioHeight,    // both RAs: c mod 3000 ft descending at (a / 4) ft/s; b&3 = SSM
  OnGround,       // both FACs discrete word 5: a&1 ? on ground (bits 18, 19) : in flight
  Scenario,       // a mod 9: approach, athr, lateral, engagement, targets, failures, tcas, situation, approach+lateral;
                  //    seed from b, c and the global seed; runs the injector for the duration
  OppFmgc,        // this FMGC as unit 2 (a&1), opposite AP/FD/A/THR engaged (a bits 1-3), opposite bus mode bits
                  //    from b (discrete word 1) and c (discrete word 3) when a bit 4 is set
  Engines,        // a&3: own stopped, opp stopped, both stopped, both running
  Tcas,           // RA word: a&1 RA active (bit 20), b -> combined control bits, c&1 TA display, c&2 sensitivity
  Ils,            // both ILS receivers: LOC dev int8(a)/50 deg, G/S dev int8(b)/50 deg, course = heading + int8(c)
  SimFlags,       // a bits: slew, pause, tracking mode override, tailstrike protection
  Dt,             // time step override: dt x {0.25, 0.5, 2, 4, 8, 0 (zero dt), 1/60 s, 0.2 s}[a mod 8]
  kKindCount
};

constexpr size_t kHeaderSize = 8;
constexpr size_t kEventSize = 8;
constexpr size_t kMaxEvents = 64;

struct Event {
  uint32_t start;     // step
  uint32_t duration;  // steps
  Kind kind;
  uint8_t a;
  uint16_t b, c;
};

struct FuzzCase {
  uint32_t snapshot = 0;
  uint8_t start = 0;
  uint32_t steps = 32;
  uint32_t timeScale = 1;
  uint32_t seed = 0;
  std::vector<Event> events;
};

inline const char* kindName(Kind k) {
  static const char* names[] = {"FcuPush", "FcuHdgVsTrkFpa", "FcuAltitude", "FcuVs", "FcuHeading", "FcuSpeed",
                                "Buttons", "FmsConstraint", "FmsPhase", "FmsSpeed", "FmsFlag", "FmsValue", "Ssm",
                                "Discrete", "Field", "Noise", "Power", "Levers", "RadioHeight", "OnGround",
                                "Scenario", "OppFmgc", "Engines", "Tcas", "Ils", "SimFlags", "Dt"};
  return static_cast<size_t>(k) < sizeof(names) / sizeof(names[0]) ? names[static_cast<size_t>(k)] : "?";
}

inline FuzzCase decode(const uint8_t* data, size_t size, uint32_t snapshots) {
  FuzzCase c;
  uint8_t h[kHeaderSize] = {};
  std::memcpy(h, data, size < kHeaderSize ? size : kHeaderSize);
  c.snapshot = snapshots ? static_cast<uint32_t>(h[0] | (h[1] << 8)) % snapshots : 0;
  c.start = h[2];
  c.steps = 32 + 8u * h[3];
  c.timeScale = 1 + (h[4] & 3u);
  c.seed = static_cast<uint32_t>(h[5] | (h[6] << 8) | (h[7] << 16));
  uint32_t at = 0;
  for (size_t off = kHeaderSize; off + kEventSize <= size && c.events.size() < kMaxEvents; off += kEventSize) {
    const uint8_t* e = data + off;
    at += 2u * e[0];
    Event ev;
    ev.start = at;
    ev.duration = e[1] < 128 ? e[1] + 1u : 128u + 16u * (e[1] - 128u);
    ev.kind = static_cast<Kind>(e[2] % static_cast<uint8_t>(Kind::kKindCount));
    ev.a = e[3];
    ev.b = static_cast<uint16_t>(e[4] | (e[5] << 8));
    ev.c = static_cast<uint16_t>(e[6] | (e[7] << 8));
    c.events.push_back(ev);
  }
  return c;
}

/** Inverse of decode (for the seed generator and the minimiser). Event starts must be non-decreasing. */
inline std::vector<uint8_t> encode(const FuzzCase& c) {
  std::vector<uint8_t> out(kHeaderSize, 0);
  out[0] = static_cast<uint8_t>(c.snapshot & 0xff);
  out[1] = static_cast<uint8_t>((c.snapshot >> 8) & 0xff);
  out[2] = c.start;
  uint32_t len = c.steps < 32 ? 0 : (c.steps - 32 + 7) / 8;
  out[3] = static_cast<uint8_t>(len > 255 ? 255 : len);
  out[4] = static_cast<uint8_t>((c.timeScale - 1) & 3u);
  out[5] = static_cast<uint8_t>(c.seed & 0xff);
  out[6] = static_cast<uint8_t>((c.seed >> 8) & 0xff);
  out[7] = static_cast<uint8_t>((c.seed >> 16) & 0xff);
  uint32_t at = 0;
  for (const Event& e : c.events) {
    uint32_t gap = (e.start - at + 1) / 2;
    while (gap > 255) {
      // a gap longer than one record can hold (after the minimiser removed events): insert filler records,
      // SimFlags with a = 0 for one step (the replay inputs have those flags false anyway)
      uint8_t filler[kEventSize] = {255, 0, static_cast<uint8_t>(Kind::SimFlags), 0, 0, 0, 0, 0};
      out.insert(out.end(), filler, filler + kEventSize);
      at += 2 * 255;
      gap = (e.start - at + 1) / 2;
    }
    at += 2 * gap;
    uint8_t d = e.duration <= 128 ? static_cast<uint8_t>(e.duration - 1)
                                  : static_cast<uint8_t>(128 + std::min<uint32_t>(127, (e.duration - 128 + 15) / 16));
    uint8_t rec[kEventSize] = {static_cast<uint8_t>(gap), d, static_cast<uint8_t>(e.kind), e.a,
                               static_cast<uint8_t>(e.b & 0xff), static_cast<uint8_t>(e.b >> 8),
                               static_cast<uint8_t>(e.c & 0xff), static_cast<uint8_t>(e.c >> 8)};
    out.insert(out.end(), rec, rec + kEventSize);
  }
  return out;
}

/** Input field groups the generic events address by index. */
struct InputFieldIndex {
  std::vector<int> ssm, discrete, numeric, floats;
  static const InputFieldIndex& get() {
    static const InputFieldIndex idx;
    return idx;
  }

 private:
  InputFieldIndex() {
    const auto& t = FieldTable<fmgc_inputs>::get();
    for (size_t i = 0; i < t.fields().size(); i++) {
      const FieldInfo& f = t.fields()[i];
      if (f.name.rfind("time.", 0) == 0 || f.name.rfind("sim_data.", 0) == 0) {
        continue;  // time and sim state have their own events
      }
      int k = static_cast<int>(i);
      if (f.isSsm) {
        ssm.push_back(k);
      } else if (f.name.rfind("discrete_inputs.", 0) == 0) {
        discrete.push_back(k);
      }
      if (!f.isSsm) {
        numeric.push_back(k);
      }
      if (f.kind == FieldKind::Float) {
        floats.push_back(k);
      }
    }
  }
};

/** Applies a case's events to the inputs of each step. One instance per run (scenario injectors keep state). */
class EventApplier {
 public:
  explicit EventApplier(const FuzzCase& c) : case_(c), scen_(c.events.size()), noise_(c.events.size()) {}

  /**
   * Applies the events active at `step` to `in` (the window's inputs for this step). `last` = the generated outputs
   * before this step (for the injectors that look at them). Returns false if the computer must not run this step.
   */
  void apply(uint32_t step, fmgc_inputs& in, const fmgc_outputs& last) {
    for (size_t i = 0; i < case_.events.size(); i++) {
      const Event& e = case_.events[i];
      if (step < e.start || step >= e.start + e.duration) {
        continue;
      }
      applyOne(i, e, step - e.start, in, last);
    }
  }

 private:
  struct ScenarioState {
    std::unique_ptr<ApproachScenario> approach;
    std::unique_ptr<AthrScenario> athr;
    std::unique_ptr<LateralScenario> lateral;
    std::unique_ptr<EngagementScenario> engagement;
    std::unique_ptr<TargetsScenario> targets;
    std::unique_ptr<ConditioningScenario> failures;
    std::unique_ptr<TcasScenario> tcas;
    std::unique_ptr<SituationScenario> situation;
  };

  static uint32_t word(const base_arinc_429& w) {
    return w.Data >= 0.0F && w.Data < 4294967296.0F ? static_cast<uint32_t>(w.Data) : 0u;
  }
  static void setBit(base_arinc_429& w, int b, bool on = true) {
    uint32_t v = word(w);
    v = on ? (v | (1u << (b - 1))) : (v & ~(1u << (b - 1)));
    w.Data = static_cast<float>(v);
  }
  static void value(base_arinc_429& w, float v, uint32_t ssm = 3) {
    w.SSM = ssm;
    w.Data = v;
  }
  static double indicatedAlt(const fmgc_inputs& in, const fmgc_outputs& last) {
    const auto& w = last.logic.altitude_indicated_ft;
    if (w.SSM == 3) {
      return w.Data;
    }
    return in.bus_inputs.adr_own_bus.altitude_corrected_1_ft.Data;
  }

  void applyOne(size_t i, const Event& e, uint32_t rel, fmgc_inputs& in, const fmgc_outputs& last) {
    auto& bus = in.bus_inputs;
    auto& fcu = bus.fcu_bus;
    auto& fms = in.fms_inputs;
    auto& di = in.discrete_inputs;
    int16_t sb = static_cast<int16_t>(e.b);
    int16_t sc = static_cast<int16_t>(e.c);
    switch (e.kind) {
      case Kind::FcuPush: {
        // word, bit: SPD push/pull, ALT value changed, w1 bit 14/15, V/S push, ALT push/pull, V/S pull, SPD/MACH,
        // EXPED, APPR, HDG push/pull, LOC push, lateral value changed
        static const uint8_t buttons[][2] = {{1, 11}, {1, 12}, {1, 13}, {1, 14}, {1, 15}, {1, 16}, {1, 17}, {1, 18},
                                             {1, 19}, {1, 21}, {1, 22}, {1, 23}, {2, 11}, {2, 12}, {2, 13}, {2, 14}};
        const auto& btn = buttons[e.a % 16];
        setBit(btn[0] == 1 ? fcu.fcu_discrete_word_1 : fcu.fcu_discrete_word_2, btn[1]);
        break;
      }
      case Kind::FcuHdgVsTrkFpa:
        setBit(fcu.fcu_discrete_word_1, 24, e.a & 1);
        setBit(fcu.fcu_discrete_word_1, 25, !(e.a & 1));
        break;
      case Kind::FcuAltitude: {
        if (rel == 0) {
          double v = (e.a & 1) ? std::round((indicatedAlt(in, last) + sb) / 100.0) * 100.0 : (e.b % 4900) * 10.0;
          held_[i] = static_cast<float>(std::fmin(49000.0, std::fmax(100.0, v)));
        }
        value(fcu.selected_alt_ft, held_[i]);
        if (rel < 3) {
          setBit(fcu.fcu_discrete_word_1, 13);
        }
        break;
      }
      case Kind::FcuVs:
        value(fcu.selected_vz_ft_min, static_cast<float>(std::fmax(-6000.0, std::fmin(6000.0, std::round(sb / 100.0) * 100.0))));
        value(fcu.selected_fpa_deg, static_cast<float>(static_cast<int8_t>(e.a)) / 10.0F);
        break;
      case Kind::FcuHeading:
        value(fcu.selected_hdg_deg, static_cast<float>(e.b % 360), e.a & 3u);
        value(fcu.selected_trk_deg, static_cast<float>(e.b % 360), e.a & 3u);
        break;
      case Kind::FcuSpeed:
        value(fcu.selected_spd_kts, static_cast<float>(100 + e.b % 300), e.a & 3u);
        value(fcu.selected_mach, 0.10F + static_cast<float>(e.c % 80) / 100.0F, e.a & 3u);
        break;
      case Kind::Buttons:
        switch (e.a % 8) {
          case 0: di.fcu_ap_button = true; break;
          case 1: di.fcu_athr_button = true; break;
          case 2: di.ap_instinctive_disc = true; break;
          case 3: di.athr_instinctive_disc = true; break;
          case 4: setBit(fcu.fcu_discrete_word_2, 26); break;
          case 5: setBit(fcu.fcu_discrete_word_2, 26); setBit(fcu.fcu_discrete_word_2, 27); break;
          case 6: di.elac_own_ap_disc = true; break;
          default: di.elac_opp_ap_disc = true; break;
        }
        break;
      case Kind::FmsConstraint:
        if (rel == 0) {
          held_[i] = (e.a & 1) ? 0.0F : static_cast<float>(std::fmax(0.0, std::round((indicatedAlt(in, last) + sb) / 100.0) * 100.0));
        }
        fms.next_alt_cstr_ft = held_[i];
        break;
      case Kind::FmsPhase:
        fms.fms_flight_phase = static_cast<fmgc_flight_phase>(e.a % 8);
        break;
      case Kind::FmsSpeed:
        fms.v_managed_kts = (e.a & 1) ? 0.0 : 120.0 + e.b % 250;
        if (rel < 2) {
          fms.fms_spd_mode_activate = fms.fms_spd_mode_activate || (e.a & 2);
          fms.fms_mach_mode_activate = fms.fms_mach_mode_activate || (e.a & 4);
        }
        break;
      case Kind::FmsFlag: {
        bool v = e.b & 1;
        switch (e.a % 10) {
          case 0: fms.fm_valid = v; break;
          case 1: fms.lateral_flight_plan_valid = v; break;
          case 2: fms.vertical_flight_plan_valid = v; break;
          case 3: fms.nav_capture_condition = v; break;
          case 4: fms.direct_to_nav_engage = v; break;
          case 5: fms.final_app_can_engage = v; break;
          case 6: fms.show_speed_margins = v; break;
          case 7: fms.preset_spd_mach_activate = v; break;
          case 8: fms.backbeam_selected = v; break;
          default: fms.flap_3_approach_selected = v; break;
        }
        break;
      }
      case Kind::FmsValue: {
        double H = indicatedAlt(in, last);
        switch (e.a % 10) {
          case 0: fms.requested_des_submode = static_cast<fmgc_des_submode>(e.b % 6); break;
          case 1: fms.selected_approach_type = static_cast<fmgc_approach_type>(e.b % 3); break;
          case 2: fms.acceleration_alt_ft = std::fmax(0.0, std::round((H + sc) / 10.0) * 10.0); break;
          case 3: fms.thrust_reduction_alt_ft = std::fmax(0.0, std::round((H + sc) / 10.0) * 10.0); break;
          case 4: fms.v_2_kts = e.b % 2 ? 0.0 : 100.0 + e.c % 100; break;
          case 5: fms.v_app_kts = 100.0 + e.c % 100; break;
          case 6: fms.cruise_alt_ft = std::round((H + sc) / 100.0) * 100.0; break;
          case 7: fms.phi_limit_deg = e.c % 40; break;
          case 8: fms.xtk_nmi = sc / 1000.0; break;
          default: fms.tke_deg = sc / 100.0; break;
        }
        break;
      }
      case Kind::Ssm: {
        const auto& idx = InputFieldIndex::get().ssm;
        FieldTable<fmgc_inputs>::get().setValue(in, static_cast<size_t>(idx[e.b % idx.size()]), e.a % 4);
        break;
      }
      case Kind::Discrete: {
        const auto& idx = InputFieldIndex::get().discrete;
        const auto& t = FieldTable<fmgc_inputs>::get();
        size_t f = static_cast<size_t>(idx[e.b % idx.size()]);
        if (rel == 0) {
          held_[i] = t.value(in, f) != 0 ? 0.0F : 1.0F;
        }
        t.setValue(in, f, held_[i]);
        break;
      }
      case Kind::Field: {
        const auto& idx = InputFieldIndex::get().numeric;
        const auto& t = FieldTable<fmgc_inputs>::get();
        size_t f = static_cast<size_t>(idx[e.b % idx.size()]);
        double x = t.value(in, f);
        double v;
        switch (e.a % 6) {
          case 0: v = 0; break;
          case 1: v = -x; break;
          case 2: v = x * (1.0 + (sc / 32768.0) * 0.05); break;
          case 3: v = x + sc; break;
          case 4: v = x * (e.c / 32768.0 + 0.5); break;
          default: v = sc; break;
        }
        if (t.fields()[f].kind == FieldKind::Bool) {
          v = (e.a & 1) ? 1 : 0;
        }
        t.setValue(in, f, v);
        break;
      }
      case Kind::Noise: {
        const auto& idx = InputFieldIndex::get().floats;
        const auto& t = FieldTable<fmgc_inputs>::get();
        size_t f = static_cast<size_t>(idx[e.b % idx.size()]);
        if (!noise_[i]) {
          noise_[i] = std::make_unique<std::mt19937_64>(case_.seed * 1000003ULL + i);
        }
        double x = t.value(in, f);
        double amp = std::fmax(1.0, std::fabs(x)) * e.c / 65536.0;
        double u = std::uniform_real_distribution<double>(-1, 1)(*noise_[i]);
        t.setValue(in, f, x + amp * u);
        break;
      }
      case Kind::Power:
        in.sim_data.computer_running = false;
        break;
      case Kind::Levers: {
        static const float angles[] = {-20.0F, -5.0F, 0.0F,  0.5F,  10.0F, 23.9F, 24.0F, 24.5F, 25.0F, 25.5F, 26.0F, 30.0F,
                                       33.9F,  34.0F, 35.0F, 35.5F, 36.0F, 36.7F, 40.0F, 44.0F, 45.0F, 2.6F,  22.0F, 25.0F};
        float ang = angles[e.b % 24];
        uint32_t ssm = (e.c & 3u) == 0 ? 3u : (e.c & 3u) - 1u;  // mostly normal
        switch (e.a & 3) {
          case 0: value(bus.fadec_own_bus.selected_tla_deg, ang); value(bus.fadec_opp_bus.selected_tla_deg, ang); break;
          case 1: value(bus.fadec_own_bus.selected_tla_deg, ang, ssm); break;
          case 2: value(bus.fadec_opp_bus.selected_tla_deg, ang, ssm); break;
          default: value(bus.fadec_own_bus.selected_tla_deg, ang); value(bus.fadec_opp_bus.selected_tla_deg, 25.0F); break;
        }
        break;
      }
      case Kind::RadioHeight: {
        double ra = std::fmax(0.0, (e.c % 3000) - (e.a / 4.0) * rel * in.time.dt);
        uint32_t ssm = (e.b & 3u) == 0 ? 3u : (e.b & 3u) - 1u;
        value(bus.ra_own_bus.radio_height_ft, static_cast<float>(ra), ssm);
        value(bus.ra_opp_bus.radio_height_ft, static_cast<float>(ra), ssm);
        break;
      }
      case Kind::OnGround:
        for (auto* fac : {&bus.fac_own_bus, &bus.fac_opp_bus}) {
          fac->discrete_word_5.SSM = 3;
          setBit(fac->discrete_word_5, 17, false);
          setBit(fac->discrete_word_5, 18, e.a & 1);
          setBit(fac->discrete_word_5, 19, e.a & 1);
        }
        break;
      case Kind::Scenario: {
        ScenarioState& s = scen_[i];
        uint64_t seed = (static_cast<uint64_t>(case_.seed) << 32) ^ (static_cast<uint64_t>(e.b) << 16) ^ e.c;
        double dt = in.time.dt;
        switch (e.a % 9) {
          case 0: if (!s.approach) s.approach = std::make_unique<ApproachScenario>(seed); s.approach->apply(in, dt); break;
          case 1: if (!s.athr) s.athr = std::make_unique<AthrScenario>(seed); s.athr->apply(in, dt); break;
          case 2: if (!s.lateral) s.lateral = std::make_unique<LateralScenario>(seed); s.lateral->apply(in, dt); break;
          case 3: if (!s.engagement) s.engagement = std::make_unique<EngagementScenario>(seed); s.engagement->apply(in, dt, last); break;
          case 4: if (!s.targets) s.targets = std::make_unique<TargetsScenario>(seed); s.targets->apply(in, dt); break;
          case 5: if (!s.failures) s.failures = std::make_unique<ConditioningScenario>(seed); s.failures->apply(in, dt); break;
          case 6: if (!s.tcas) s.tcas = std::make_unique<TcasScenario>(seed); s.tcas->apply(in, dt, last); break;
          case 7: if (!s.situation) s.situation = std::make_unique<SituationScenario>(seed); s.situation->apply(in, dt, last); break;
          default:
            if (!s.approach) s.approach = std::make_unique<ApproachScenario>(seed);
            if (!s.lateral) s.lateral = std::make_unique<LateralScenario>(seed + 1);
            s.approach->apply(in, dt);
            s.lateral->apply(in, dt);
            break;
        }
        break;
      }
      case Kind::OppFmgc:
        di.is_unit_1 = !(e.a & 1);
        di.ap_opp_engaged = e.a & 2;
        di.fd_opp_engaged = e.a & 4;
        di.athr_opp_engaged = e.a & 8;
        if (e.a & 16) {
          bus.fmgc_opp_bus.discrete_word_1.SSM = 3;
          bus.fmgc_opp_bus.discrete_word_1.Data = static_cast<float>(static_cast<uint32_t>(e.b) << 10);
          bus.fmgc_opp_bus.discrete_word_3.SSM = 3;
          bus.fmgc_opp_bus.discrete_word_3.Data = static_cast<float>(static_cast<uint32_t>(e.c) << 10);
        }
        break;
      case Kind::Engines:
        switch (e.a & 3) {
          case 0: di.eng_own_stop = true; break;
          case 1: di.eng_opp_stop = true; break;
          case 2: di.eng_own_stop = di.eng_opp_stop = true; break;
          default: di.eng_own_stop = di.eng_opp_stop = false; break;
        }
        break;
      case Kind::Tcas: {
        uint32_t ra = (e.a & 1) ? (1u << 19) : 0u;
        ra |= (static_cast<uint32_t>(e.b) & 0x1ffu) << 10;
        value(bus.tcas_bus.vertical_resolution_advisory, static_cast<float>(ra));
        di.tcas_ta_display = e.c & 1;
        if (e.c & 2) {
          value(bus.tcas_bus.sensitivity_level, static_cast<float>((1u << 25) | (1u << 26)));
        }
        break;
      }
      case Kind::Ils: {
        float hdg = bus.ir_own_bus.heading_magnetic_deg.Data;
        for (auto* ils : {&bus.ils_own_bus, &bus.ils_opp_bus}) {
          value(ils->localizer_deviation_deg, static_cast<float>(static_cast<int8_t>(e.a)) / 50.0F);
          value(ils->glideslope_deviation_deg, static_cast<float>(static_cast<int8_t>(e.b & 0xff)) / 50.0F);
          value(ils->runway_heading_deg, std::fmod(hdg + static_cast<float>(static_cast<int8_t>(e.c & 0xff)) + 360.0F, 360.0F));
          value(ils->ils_frequency_mhz, 110.3F);
        }
        break;
      }
      case Kind::SimFlags:
        in.sim_data.slew_on = e.a & 1;
        in.sim_data.pause_on = e.a & 2;
        in.sim_data.tracking_mode_on_override = e.a & 4;
        in.sim_data.tailstrike_protection_on = e.a & 8;
        break;
      case Kind::Dt: {
        static const double factor[] = {0.25, 0.5, 2, 4, 8, 0, -1, -2};
        double f = factor[e.a % 8];
        in.time.dt = f >= 0 ? in.time.dt * f : (f == -1 ? 1.0 / 60.0 : 0.2);
        break;
      }
      default:
        break;
    }
  }

  const FuzzCase& case_;
  std::vector<ScenarioState> scen_;
  std::vector<std::unique_ptr<std::mt19937_64>> noise_;
  std::vector<float> held_ = std::vector<float>(kMaxEvents, 0.0F);
};

}  // namespace fmgc::fuzz
