// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Behavioural properties of the FMGC, checked every frame on the generated (stock) model's inputs and outputs only.
// They come from PLAN.md (Phase 4 and the docs-derived oracle list), the FBW pilot docs, the FCOM, and the mode docs in
// docs/modes/*.md, docs/athr-modes.md, docs/conditioning.md. They describe intended behaviour, not the generated code:
// a violation is a finding about stock behaviour (record it with a reproducer, docs/properties.md), not a test failure.
//
// Every property is "bad condition must not hold", with a tolerance: a violation is reported when the bad condition has
// held for longer than the tolerance (0 = on the first bad frame). Several properties exist in a strict form and a
// "_1s" form because the generated mode logic hands over between latches with a deliberate one-frame overlap and a
// 0.72 s "set recently" guard (docs/modes/climb-descent.md S2): the strict form records that the overlap exists, the
// tolerant form whether an overlap ever persists.
//
// Edge-based properties need the previous frame; after reset() (a new run from a snapshot) the first frame only fills
// the history.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "FmgcComputer_types.h"

namespace fmgc::props {

struct PropertyInfo {
  const char* id;
  double toleranceS;  // bad condition must hold longer than this (s) to be a violation; < 0: -N frames
  const char* source;
  const char* text;
};

// clang-format off
enum Id : int {
  VertSingle, VertSingle1s, LatSingle, LatSingle1s, ArmedActive, ArmedActive1s,
  ModesNeedApFd, LawVertConsistent, LawLatConsistent, ApImpliesAnyApFd,
  AthrActiveEngaged, AthrModesEngaged, AthrSingleMode, AthrSingleMode1s, AthrActiveHasMode, AthrComplementary, RetardInLand, RetardHeight30,
  FlareImpliesLand, RolloutImpliesLand, FlareAfterLand, RolloutAfterFlare, LandEngageConditions, FlareHeight,
  VsEngageReason, VsAirborne5s, OpenModesAirborne5s, AltStarTargetChangeVs, AltCapture20ft, AltCapture40ft,
  ClbEngageConditions, DesEngageConditions,
  SpdTargetRange, SpdTargetLeVmax, SpdTargetSrsTo, SrsPitchLimit18,
  DoubleFailureDisengages, FrozenWhenOff, CleanRestart, OutputsFinite,
  kCount
};

inline const std::vector<PropertyInfo>& list() {
  static const std::vector<PropertyInfo> l = {
    {"vert.single_active", 0, "PLAN Phase 4", "at most one vertical (longitudinal) mode active (CLB, DES, OP CLB/DES, EXP CLB/DES, SRS TO/GA, V/S, FPA, ALT*, ALT, G/S*, G/S, FINAL, FLARE, TCAS)"},
    {"vert.single_active_1s", 1.0, "PLAN Phase 4 + climb-descent.md S2", "two vertical modes never active together for more than 1 s (own logic: not counted while synchronised to the opposite FMGC)"},
    {"lat.single_active", 0, "task list", "at most one lateral mode active (RWY, RWY TRK, NAV, LOC*, LOC, GA TRK, HDG, TRK, LAND)"},
    {"lat.single_active_1s", 1.0, "task list + lateral.md", "two lateral modes never active together for more than 1 s (not while synchronised)"},
    {"armed_active.exclusive", 0, "task list", "a mode is never armed and active at once (NAV, LOC, G/S, FINAL, CLB, DES, LAND, ALT, TCAS)"},
    {"armed_active.exclusive_1s", 1.0, "task list", "armed and active together never last more than 1 s (not while synchronised)"},
    {"modes.need_ap_fd", -1, "climb-descent.md S3 (mode reset without AP/FD)", "no lateral or vertical mode active while no AP/FD is engaged for more than one frame (the resets read the previous frame)"},
    {"law.vertical_consistent", 0, "structure.md law selection", "active_longitudinal_law is the law of an active vertical mode (NONE only without one, or DES with no submode)"},
    {"law.lateral_consistent", 0, "structure.md law selection", "active_lateral_law is the law of an active lateral mode (NONE only without one)"},
    {"engaged.ap_implies_any", 0, "engagement.md", "own AP engaged implies any_ap_fd_engaged"},
    {"athr.active_implies_engaged", 0, "task list, athr-modes.md", "A/THR active implies A/THR (own) engaged"},
    {"athr.modes_imply_engaged", 0, "athr-modes.md (latch reset by !engaged)", "THR, SPEED/MACH, RETARD and alpha floor modes only with A/THR engaged"},
    {"athr.single_mode", 0, "athr-modes.md", "at most one of alpha floor, THR, SPEED/MACH, RETARD"},
    {"athr.single_mode_1s", 1.0, "athr-modes.md (0.72 s handover guard)", "two A/THR modes never together for more than 1 s (not while synchronised)"},
    {"athr.active_has_mode", 1.0, "FBW docs / FCOM (an active A/THR always has a mode on the FMA)", "A/THR active => one of alpha floor, THR, SPEED/MACH, RETARD (after 1 s)"},
    {"athr.complementary", 1.5, "PLAN docs oracle (FBW docs)", "A/THR active and settled: pitch on speed (SPD_MACH, SRS law) => THR; pitch on path (ALT, ALT*, V/S, FPA, G/S law) => SPEED/MACH"},
    {"athr.retard_in_land", 0, "PLAN docs oracle (RETARD in FLARE)", "RETARD only with LAND or FLARE active (this or the previous frame)"},
    {"athr.retard_height_30ft", 0, "PLAN docs oracle (FBW docs: RETARD at 30 ft)", "RETARD engages at or below 30 ft RA"},
    {"land.flare_implies_land", -1, "takeoff-ga-approach.md FLARE", "FLARE only with LAND active (one frame of lag allowed: FLARE reads the previous frame's LAND)"},
    {"land.rollout_implies_land", -1, "lateral.md ROLL OUT", "ROLL OUT and ALIGN only with LAND active (one frame of lag allowed)"},
    {"land.flare_after_land", 0, "task list (LAND/FLARE/ROLL OUT order)", "FLARE engages only when LAND was active on the previous frame"},
    {"land.rollout_after_flare", 0, "task list (LAND/FLARE/ROLL OUT order)", "ROLL OUT engages only after FLARE has been active in this LAND engagement"},
    {"land.engage_conditions", 0, "PLAN docs oracle (LAND below 400 ft RA with LOC and G/S)", "LAND (own) engages below 400 ft RA with LOC and G/S tracking on the previous frame"},
    {"land.flare_height", 0, "PLAN docs oracle (FLARE about 40 ft)", "FLARE engages below 80 ft RA"},
    {"vs.engage_reason", 0, "PLAN Phase 4 + FBW docs/FCOM reversion list", "V/S-FPA never engages without a listed reason (knob, AP/FD engagement with no mode, 5 s airborne with no mode, G/S/FINAL/DES loss, ALT* target change >= 250 ft, CLB/DES family with FCU on the wrong side, TCAS end, opposite sync)"},
    {"vs.airborne_5s", 0, "PLAN docs oracle (V/S needs > 5 s airborne)", "V/S-FPA engages only after 5 s airborne (or both engines stopped on the ground)"},
    {"open_modes.airborne_5s", 0, "PLAN docs oracle (OP CLB/OP DES/EXPED need > 5 s airborne)", "OP CLB, OP DES, EXP CLB, EXP DES engage only after 5 s airborne (or engines stopped on the ground)"},
    {"alt_star.target_change_reverts_vs", 0, "PLAN Phase 4 (FCU change during ALT* reverts to V/S)", "ALT* active (own logic, not synchronised) and its target moved >= 250 ft from the value at ALT* engagement => V/S-FPA within 2 frames"},
    {"alt.capture_20ft", 0, "PLAN docs oracle (FBW docs: ALT engages within 20 ft)", "ALT (own) engages only within 20 ft of the target"},
    {"alt.capture_40ft", 0, "alt-vs-fpa-tcas.md (40 ft for 0.8 s)", "ALT (own) engages only within 40 ft of the target"},
    {"clb.engage_conditions", 0, "PLAN docs oracle (CLB needs NAV, FCU above, not DES/APPR/GA)", "CLB (own) engages only with NAV, FCU altitude above the aircraft, phase not DES/APPR/GA"},
    {"des.engage_conditions", 0, "PLAN docs oracle (DES needs FCU below, NAV/LOC, valid path, not TO/GA/G/S/LAND/FINAL)", "DES (own) engages only with FCU below, NAV/LOC*/LOC, valid vertical path, not TO/GA phase, not from G/S/LAND/FINAL"},
    {"spd_target.range", 0, "FCU speed range 100-399 kt, targets.md findings 5/6", "with an AP/FD engaged, the speed target is within 90-400 kt"},
    {"spd_target.le_vmax", 0, "FCOM DSC-22_30-40 note 4 (targets.md finding 8)", "managed speed target (outside SRS) not above VMAX"},
    {"spd_target.srs_to", 0, "PLAN docs oracle (SRS: V2+10, max V2+15)", "SRS takeoff with managed speed: V2 <= target <= V2 + 15"},
    {"srs.pitch_limit_18", 0.5, "PLAN docs oracle (SRS 18 deg pitch limit)", "SRS law active: FD pitch command not above 18 deg"},
    {"cond.double_failure_disengages", 1.0, "conditioning.md finding 2 + engagement.md (common condition)", "dual ADR or dual IR failure: AP and A/THR (own) not engaged after 1 s"},
    {"running.frozen_when_off", 0, "task list (no mode engages while the computer is not running)", "computer not running: outputs unchanged"},
    {"running.clean_restart", 0, "task list", "first running frame after a power cycle: no lateral or vertical mode active (unless synchronised from the opposite FMGC)"},
    {"outputs.finite", 0, "robustness", "outer-loop commands, N1 command and targets are finite"},
  };
  return l;
}
// clang-format on

/** Per-property counters. */
struct Stats {
  long checked = 0;     // frames where the property's precondition applied
  long badFrames = 0;   // frames where the bad condition held (before tolerance)
  long violations = 0;  // episodes beyond the tolerance
  long firstStep = -1;
  double firstTime = 0;
  std::string firstDetail;
};

class Properties {
 public:
  /** Called once per violation episode: property id, step, sim time, detail. */
  std::function<void(int, long, double, const std::string&)> onViolation;

  Properties() : stats_(kCount), badFor_(kCount, 0.0), inViolation_(kCount, false) {
    if (list().size() != static_cast<size_t>(kCount)) {
      std::fprintf(stderr, "properties.h: list() and Id out of step\n");
      std::abort();
    }
  }

  /** A new run (e.g. restored snapshot): forget the history. Counters are kept. */
  void reset() {
    havePrev_ = false;
    havePrev2_ = false;
    std::fill(badFor_.begin(), badFor_.end(), 0.0);
    std::fill(badFrames_.begin(), badFrames_.end(), 0);
    std::fill(inViolation_.begin(), inViolation_.end(), false);
    airborneS_ = -2;      // not seeded yet
    sinceStartS_ = 1e9;   // a restored snapshot is not a restart of the model
    wasOff_ = false;
    offS_ = 0;
    altStarTargetKnown_ = false;
    vsPendingSteps_ = -1;
    flareSinceLand_ = false;
    lawStableS_ = 0;
  }

  void clearStats() {
    for (auto& s : stats_) {
      s = Stats{};
    }
  }

  const std::vector<Stats>& stats() const { return stats_; }

  void onFrame(long step, const fmgc_inputs& in, const fmgc_outputs& out, bool running) {
    step_ = step;
    t_ = in.time.simulation_time;
    dt_ = in.time.dt;
    if (!running) {
      if (havePrev_) {
        bad(FrozenWhenOff, std::memcmp(&out, &prev_, sizeof(out)) != 0, [] { return std::string("outputs changed while not running"); });
      }
      wasOff_ = true;
      offS_ += dt_;
      return;  // the generated step does nothing; keep the history of the last running frame
    }
    const auto& ap = out.ap_fd_logic;
    const auto& lm = ap.longitudinal_modes;
    const auto& la = ap.lateral_modes;
    const auto& am = ap.armed_modes;
    const auto& lg = out.logic;
    const auto& at = out.athr;

    bool onGround = lg.on_ground;
    bool engStoppedGround = in.discrete_inputs.eng_own_stop && in.discrete_inputs.eng_opp_stop && onGround;
    // Time airborne (own model of it): unknown (-1) when a run or a power-up starts in flight without the conditioning's
    // 5 s in-flight confirm (which restarts at every power-up, docs/conditioning.md finding 9). Unknown counts as
    // airborne for more than 5 s (the aircraft was flying). sinceStartS_: time since the model (re)started running, for
    // the mode logic's own 5 s confirmation nodes, which restart with it.
    if (wasOff_) {
      sinceStartS_ = 0;
      if (!onGround && airborneS_ >= 0 && airborneS_ < 1e8) {
        airborneS_ += offS_;  // still flying during the power cut
      }
    } else {
      sinceStartS_ += dt_;
    }
    if (airborneS_ == -2) {
      airborneS_ = onGround ? 0 : (lg.gnd_eng_stop_flt_5s ? 1e9 : -1);
    } else if (onGround) {
      airborneS_ = 0;
    } else if (airborneS_ >= 0) {
      airborneS_ += dt_;
    }
    offS_ = 0;
    constexpr double kFrameTol = 0.05;  // one frame of slack on the 5 s timers
    bool airborneKnown = airborneS_ >= 0;
    bool airborne5 = (airborneKnown ? airborneS_ >= 5.0 - kFrameTol : true) || engStoppedGround;
    bool confirm5 = !onGround && sinceStartS_ >= 5.0 - kFrameTol &&
                    (airborneKnown ? airborneS_ >= 5.0 - kFrameTol : true);
    double H = valueOr(lg.altitude_indicated_ft, 0);
    double fcuAlt = valueOr(in.bus_inputs.fcu_bus.selected_alt_ft, 0);
    double ra = lg.ra_computation_data.radio_height_ft.Data;
    auto phase = in.fms_inputs.fms_flight_phase;
    uint32_t w1 = word(in.bus_inputs.fcu_bus.fcu_discrete_word_1);

    // ---- mode exclusivity
    {
      const uint8_t v[] = {lm.clb_active, lm.des_active, lm.op_clb_active, lm.op_des_active, lm.exp_clb_active,
                        lm.exp_des_active, lm.pitch_takeoff_active, lm.pitch_goaround_active, lm.vs_active,
                        lm.fpa_active, lm.alt_acq_active, lm.alt_hold_active, lm.gs_capt_active, lm.gs_trk_active,
                        lm.final_des_active, lm.flare_active, lm.tcas_active};
      vertMask_ = mask(v, sizeof(v) / sizeof(v[0]));
      int n = __builtin_popcount(vertMask_);
      bad(VertSingle, n > 1, [&] { return vertNames(vertMask_); });
      bad(VertSingle1s, n > 1 && !ap.fmgc_opp_mode_sync, [&] { return vertNames(vertMask_); });
    }
    {
      bool rwyTrkOnly = la.rwy_trk_submode_active && !la.rwy_active;  // rwy_active includes RWY TRK
      const uint8_t v[] = {la.rwy_active, rwyTrkOnly, la.nav_active, la.loc_cpt_active, la.loc_trk_active,
                        la.roll_goaround_active, la.hdg_active, la.trk_active, la.land_active};
      latMask_ = mask(v, sizeof(v) / sizeof(v[0]));
      int n = __builtin_popcount(latMask_);
      bad(LatSingle, n > 1, [&] { return latNames(latMask_); });
      bad(LatSingle1s, n > 1 && !ap.fmgc_opp_mode_sync, [&] { return latNames(latMask_); });
    }
    {
      const uint8_t both[] = {am.nav_armed && la.nav_active, am.loc_armed && (la.loc_cpt_active || la.loc_trk_active),
                           am.glide_armed && (lm.gs_capt_active || lm.gs_trk_active),
                           am.final_des_armed && lm.final_des_active, am.clb_armed && lm.clb_active,
                           am.des_armed && lm.des_active, am.land_armed && la.land_active,
                           am.alt_acq_armed && (lm.alt_acq_active || lm.alt_hold_active), am.tcas_armed && lm.tcas_active};
      uint32_t m = mask(both, sizeof(both) / sizeof(both[0]));
      static const char* names[] = {"NAV", "LOC", "GS", "FINAL", "CLB", "DES", "LAND", "ALT", "TCAS"};
      bad(ArmedActive, m != 0, [&] { return "armed+active: " + maskNames(m, names); });
      bad(ArmedActive1s, m != 0 && !ap.fmgc_opp_mode_sync, [&] { return "armed+active: " + maskNames(m, names); });
    }
    bool anyVert = vertMask_ != 0;
    bool anyLat = latMask_ != 0;
    bad(ModesNeedApFd, (anyVert || anyLat) && !ap.any_ap_fd_engaged, [&] { return "modes " + vertNames(vertMask_) + " / " + latNames(latMask_); });

    // ---- law consistency
    {
      bool ok = true;
      bool tcasVs = lm.tcas_active && ap.active_tcas_submode == tcas_submode::VS;
      bool tcasAcq = lm.tcas_active && ap.active_tcas_submode == tcas_submode::ALT_ACQ;
      bool tcasHold = lm.tcas_active && ap.active_tcas_submode == tcas_submode::ALT_HOLD;
      auto sub = in.fms_inputs.requested_des_submode;
      switch (ap.active_longitudinal_law) {
        case vertical_law::NONE: ok = !anyVert || (lm.des_active && !lm.vs_active && !lm.fpa_active); break;
        case vertical_law::FLARE: ok = lm.flare_active; break;
        case vertical_law::GS: ok = lm.gs_capt_active || lm.gs_trk_active || la.land_active; break;
        case vertical_law::SRS: ok = lm.pitch_takeoff_active || lm.pitch_goaround_active; break;
        case vertical_law::FPA: ok = lm.fpa_active || (lm.des_active && sub == fmgc_des_submode::FPA_SPEED); break;
        case vertical_law::VS: ok = lm.vs_active || tcasVs || (lm.des_active && sub == fmgc_des_submode::VS_SPEED); break;
        case vertical_law::SPD_MACH:
          ok = lm.clb_active || lm.op_clb_active || lm.op_des_active || lm.exp_clb_active || lm.exp_des_active ||
               (lm.des_active && sub == fmgc_des_submode::SPEED_THRUST);
          break;
        case vertical_law::ALT_ACQ: ok = lm.alt_acq_active || tcasAcq; break;
        case vertical_law::ALT_HOLD: ok = lm.alt_hold_active || tcasHold; break;
        case vertical_law::VPATH:
          ok = lm.final_des_active || (lm.des_active && (sub == fmgc_des_submode::VPATH_THRUST ||
                                                         sub == fmgc_des_submode::VPATH_SPEED));
          break;
        default: ok = false;
      }
      bad(LawVertConsistent, !ok, [&] { return "law " + std::to_string(static_cast<int>(ap.active_longitudinal_law)) + " modes " + vertNames(vertMask_); });
      switch (ap.active_lateral_law) {
        case lateral_law::NONE: ok = !anyLat; break;
        case lateral_law::ROLL_OUT: ok = la.rollout_submode_active || la.rwy_active; break;
        case lateral_law::LOC_TRACK: ok = la.loc_trk_active || la.land_active; break;
        case lateral_law::LOC_CPT: ok = la.loc_cpt_active; break;
        case lateral_law::HPATH: ok = la.nav_active; break;
        case lateral_law::TRACK: ok = la.trk_active || la.rwy_trk_submode_active || la.roll_goaround_active; break;
        case lateral_law::HDG: ok = la.hdg_active; break;
        default: ok = false;
      }
      bad(LawLatConsistent, !ok, [&] { return "law " + std::to_string(static_cast<int>(ap.active_lateral_law)) + " modes " + latNames(latMask_); });
    }
    bad(ApImpliesAnyApFd, lg.ap_own_engaged && !ap.any_ap_fd_engaged, [] { return std::string(); });

    // ---- autothrust
    bad(AthrActiveEngaged, at.athr_active && !lg.athr_own_engaged, [] { return std::string(); });
    bool anyAthrMode = at.alpha_floor_mode_active || at.thrust_mode_active || at.speed_mach_mode_active ||
                       at.retard_mode_active;
    bad(AthrModesEngaged, anyAthrMode && !lg.athr_own_engaged, [&] { return athrNames(at); });
    {
      int n = at.alpha_floor_mode_active + at.thrust_mode_active + at.speed_mach_mode_active + at.retard_mode_active;
      bad(AthrSingleMode, n > 1, [&] { return athrNames(at); });
      bad(AthrSingleMode1s, n > 1 && !ap.fmgc_opp_mode_sync, [&] { return athrNames(at); });
      bad(AthrActiveHasMode, at.athr_active && n == 0, [&] { return "FMA " + std::to_string(static_cast<int>(at.fma_mode)); });
    }
    {
      if (havePrev_ && ap.active_longitudinal_law == prev_.ap_fd_logic.active_longitudinal_law) {
        lawStableS_ += dt_;
      } else {
        lawStableS_ = 0;
      }
      auto law = ap.active_longitudinal_law;
      bool settled = at.athr_active && !at.alpha_floor_mode_active && !at.retard_mode_active && !lm.tcas_active &&
                     !ap.fmgc_opp_mode_sync && lawStableS_ > 1.5;
      bool speedLaw = law == vertical_law::SPD_MACH || law == vertical_law::SRS;
      bool pathLaw = law == vertical_law::ALT_HOLD || law == vertical_law::ALT_ACQ || law == vertical_law::VS ||
                     law == vertical_law::FPA || law == vertical_law::GS;
      bool badC = settled && ((speedLaw && !at.thrust_mode_active) || (pathLaw && !at.speed_mach_mode_active));
      if (settled) {
        bad(AthrComplementary, badC, [&] { return "law " + std::to_string(static_cast<int>(law)) + " " + athrNames(at); });
      } else {
        idle(AthrComplementary);
      }
    }
    if (havePrev_ && !wasOff_) {
      const auto& p = prev_.ap_fd_logic;
      bool land = la.land_active || lm.flare_active || p.lateral_modes.land_active || p.longitudinal_modes.flare_active;
      bad(RetardInLand, at.retard_mode_active && !land, [&] { return "lat " + latNames(latMask_) + " vert " + vertNames(vertMask_); });
      bool retardRise = at.retard_mode_active && !prev_.athr.retard_mode_active;
      if (retardRise) {
        stats_[RetardHeight30].checked++;
        bad(RetardHeight30, ra > 30.0, [&] { return std::string("RA " + num(ra)); }, false);
      }
    }

    // ---- LAND / FLARE / ROLL OUT
    bad(FlareImpliesLand, lm.flare_active && !la.land_active, [] { return std::string(); });
    bad(RolloutImpliesLand, (la.rollout_submode_active || la.align_submode_active) && !la.land_active,
        [&] { return std::string(la.rollout_submode_active ? "ROLL OUT" : "ALIGN"); });
    if (havePrev_ && !wasOff_) {
      const auto& p = prev_.ap_fd_logic;
      if (!la.land_active) {
        flareSinceLand_ = false;
      }
      bool flareRise = lm.flare_active && !p.longitudinal_modes.flare_active;
      if (flareRise) {
        stats_[FlareAfterLand].checked++;
        bad(FlareAfterLand, !p.lateral_modes.land_active, [&] { return std::string(""); }, false);
        stats_[FlareHeight].checked++;
        bad(FlareHeight, ra >= 80.0, [&] { return std::string("RA " + num(ra)); }, false);
      }
      bool rolloutRise = la.rollout_submode_active && !p.lateral_modes.rollout_submode_active;
      if (rolloutRise) {
        stats_[RolloutAfterFlare].checked++;
        bad(RolloutAfterFlare, !flareSinceLand_ && !lm.flare_active, [&] { return std::string("RA " + num(ra)); }, false);
      }
      if (lm.flare_active && la.land_active) {
        flareSinceLand_ = true;
      }
      bool landRise = la.land_active && !p.lateral_modes.land_active;
      if (landRise && !ap.fmgc_opp_mode_sync) {
        stats_[LandEngageConditions].checked++;
        bool ok = ra < 400.0 && p.lateral_modes.loc_trk_active &&
                  (p.longitudinal_modes.gs_trk_active || p.longitudinal_modes.gs_capt_active);
        bad(LandEngageConditions, !ok,
            [&] { return std::string("RA " + num(ra) + " prev LOC " + std::to_string(p.lateral_modes.loc_trk_active) + " prev GS " +
                std::to_string(p.longitudinal_modes.gs_trk_active)); },
            false);
      }
    } else {
      flareSinceLand_ = la.land_active && lm.flare_active;
    }

    // ---- vertical engagement rules (edges)
    if (havePrev_ && !wasOff_) {
      const auto& p = prev_.ap_fd_logic;
      const auto& pm = p.longitudinal_modes;
      const auto& pl = p.lateral_modes;
      bool vsNow = lm.vs_active || lm.fpa_active;
      bool vsPrev = pm.vs_active || pm.fpa_active;
      bool sync = ap.fmgc_opp_mode_sync;
      uint32_t oppW1 = word(in.bus_inputs.fmgc_opp_bus.discrete_word_1);
      if (vsNow && !vsPrev) {
        // listed reasons, observable from inputs and outputs
        bool knob = (bit(w1, 16) || bit(w1, 19)) && !(bit(prevW1_, 16) || bit(prevW1_, 19));
        bool apFdEngage = !onGround && ap.any_ap_fd_engaged && !p.any_ap_fd_engaged && !p.any_longitudinal_mode_engaged;
        bool liftoff = confirm5 && !p.any_longitudinal_mode_engaged;
        bool gsPrev = pm.gs_capt_active || pm.gs_trk_active;
        bool gsPrev2 = havePrev2_ && (prev2Gs_);
        bool landNow = am.land_armed || la.land_active;
        bool landPrev = p.armed_modes.land_armed || pl.land_active;
        bool gsLost = (gsPrev || gsPrev2) && ((prev2Land_ && !landPrev) || (landPrev && !landNow) ||
                                              !(lm.gs_capt_active || lm.gs_trk_active));
        bool navFell = havePrev2_ && prev2Nav_ && !pl.nav_active;
        bool apprPush = bit(w1, 23) && !bit(prevW1_, 23);
        bool finalLost = pm.final_des_active && (navFell || apprPush || !in.fms_inputs.vertical_flight_plan_valid);
        bool desInvalid = pm.des_active && (phase == fmgc_flight_phase::Takeoff || phase == fmgc_flight_phase::Climb ||
                                            phase == fmgc_flight_phase::Goaround ||
                                            !(pl.nav_active || pl.loc_cpt_active || pl.loc_trk_active) ||
                                            !in.fms_inputs.vertical_flight_plan_valid);
        bool altStarChange = pm.alt_acq_active &&
                             (!altStarTargetKnown_ || std::fabs(altStarTarget_ - ap.alt_sel_or_cstr) >= 250.0);
        bool desFcuAbove = (pm.des_active || pm.exp_des_active || pm.op_des_active) && fcuAlt - H > 0;
        bool clbFcuBelow = (pm.clb_active || pm.exp_clb_active || pm.op_clb_active) && fcuAlt - H < 0;
        uint32_t raw = word(in.bus_inputs.tcas_bus.vertical_resolution_advisory);
        bool raActive = in.bus_inputs.tcas_bus.vertical_resolution_advisory.SSM == 3 && bit(raw, 20);
        bool tcasEnd = pm.tcas_active && (!raActive || !lg.tcas_mode_available);
        bool oppSync = sync && (bit(oppW1, 17) || bit(oppW1, 18));
        // the loss terms are falling edges between the two previous frames: unknown on the first frames of a run
        bool ok = knob || apFdEngage || liftoff || gsLost || finalLost || desInvalid || altStarChange ||
                  desFcuAbove || clbFcuBelow || tcasEnd || oppSync || !havePrev2_;
        stats_[VsEngageReason].checked++;
        bad(VsEngageReason, !ok,
            [&] { return std::string("prev vert modes [" + vertNames(prevVertMask_) + "] lat [" + latNames(prevLatMask_) + "] onGround " +
                std::to_string(onGround) + " H " + num(H) + " FCU " + num(fcuAlt) + " target " + num(ap.alt_sel_or_cstr)); },
            false);
        stats_[VsAirborne5s].checked++;
        bad(VsAirborne5s, !airborne5 && !sync, [&] { return std::string("onGround " + std::to_string(onGround) + " airborne " + num(airborneS_) + " s"); },
            false);
      }
      bool openRise = (lm.op_clb_active && !pm.op_clb_active) || (lm.op_des_active && !pm.op_des_active) ||
                      (lm.exp_clb_active && !pm.exp_clb_active) || (lm.exp_des_active && !pm.exp_des_active);
      if (openRise) {
        stats_[OpenModesAirborne5s].checked++;
        bad(OpenModesAirborne5s, !airborne5 && !sync,
            [&] { return std::string(vertNames(vertMask_) + " onGround " + std::to_string(onGround) + " airborne " + num(airborneS_) + " s"); }, false);
      }
      // ALT* target sample (= the generated u_ly: target on the frame ALT* latched)
      if (lm.alt_acq_active && !pm.alt_acq_active) {  // (a restart frame is skipped: sampled below)
        altStarTarget_ = ap.alt_sel_or_cstr;
        altStarTargetKnown_ = true;
        altStarReported_ = false;
      } else if (!lm.alt_acq_active) {
        altStarTargetKnown_ = false;
      }
      // ALT* target change >= 250 ft must revert to V/S within 2 frames
      if (pm.alt_acq_active && altStarTargetKnown_ && std::fabs(altStarTarget_ - ap.alt_sel_or_cstr) >= 250.0 &&
          vsPendingSteps_ < 0 && !sync && !altStarReported_) {
        altStarReported_ = true;  // checked once per ALT* engagement
        stats_[AltStarTargetChangeVs].checked++;
        if (!vsNow) {
          vsPendingSteps_ = 2;
          vsPendingDetail_ = "ALT* target " + num(altStarTarget_) + " -> " + num(ap.alt_sel_or_cstr);
        }
      }
      if (vsPendingSteps_ >= 0) {
        if (vsNow) {
          vsPendingSteps_ = -1;
        } else if (vsPendingSteps_-- == 0) {
          bad(AltStarTargetChangeVs, true, [&] { return std::string(vsPendingDetail_ + ", modes now " + vertNames(vertMask_)); }, false);
          vsPendingSteps_ = -1;
        }
      }
      if (lm.alt_hold_active && !pm.alt_hold_active && !sync) {
        double d = std::fabs(H - ap.alt_sel_or_cstr);
        stats_[AltCapture20ft].checked++;
        stats_[AltCapture40ft].checked++;
        bad(AltCapture20ft, d >= 20.0, [&] { return std::string("|H - target| " + num(d)); }, false);
        bad(AltCapture40ft, d >= 40.0, [&] { return std::string("|H - target| " + num(d)); }, false);
      }
      if (lm.clb_active && !pm.clb_active && !sync) {
        stats_[ClbEngageConditions].checked++;
        bool ok = pl.nav_active && fcuAlt > H && phase != fmgc_flight_phase::Descent &&
                  phase != fmgc_flight_phase::Approach && phase != fmgc_flight_phase::Goaround;
        bad(ClbEngageConditions, !ok,
            [&] { return std::string("prev NAV " + std::to_string(pl.nav_active) + " FCU " + num(fcuAlt) + " H " + num(H) + " phase " +
                std::to_string(static_cast<int>(phase))); },
            false);
      }
      if (lm.des_active && !pm.des_active && !sync) {
        stats_[DesEngageConditions].checked++;
        bool ok = fcuAlt < H && (pl.nav_active || pl.loc_cpt_active || pl.loc_trk_active) &&
                  in.fms_inputs.vertical_flight_plan_valid && phase != fmgc_flight_phase::Takeoff &&
                  phase != fmgc_flight_phase::Goaround && !pm.gs_capt_active && !pm.gs_trk_active &&
                  !pl.land_active && !pm.final_des_active;
        bad(DesEngageConditions, !ok,
            [&] { return std::string("FCU " + num(fcuAlt) + " H " + num(H) + " prev lat [" + latNames(prevLatMask_) + "] prev vert [" + vertNames(prevVertMask_) +
                "] vfp " + std::to_string(in.fms_inputs.vertical_flight_plan_valid) + " phase " +
                std::to_string(static_cast<int>(phase))); },
            false);
      }
    }

    // ---- targets
    {
      double v = ap.spd_target_kts;
      if (ap.any_ap_fd_engaged) {
        bad(SpdTargetRange, !(v >= 90.0 && v <= 400.0), [&] { return std::string("V_c " + num(v)); });
      } else {
        idle(SpdTargetRange);
      }
      const auto& vmax = lg.chosen_fac_bus.v_max_kn;
      bool managedCruise = ap.auto_spd_control_active && !lm.pitch_takeoff_active && !lm.pitch_goaround_active;
      if (managedCruise && vmax.SSM == 3) {
        bad(SpdTargetLeVmax, v > vmax.Data + 1e-3, [&] { return std::string("V_c " + num(v) + " VMAX " + num(vmax.Data)); });
      } else {
        idle(SpdTargetLeVmax);
      }
      if (lm.pitch_takeoff_active && ap.auto_spd_control_active && in.fms_inputs.v_2_kts > 0) {
        double v2 = in.fms_inputs.v_2_kts;
        bad(SpdTargetSrsTo, v < v2 - 1e-9 || v > v2 + 15 + 1e-9, [&] { return std::string("V_c " + num(v) + " V2 " + num(v2)); });
      } else {
        idle(SpdTargetSrsTo);
      }
      if (ap.active_longitudinal_law == vertical_law::SRS) {
        double th = out.ap_fd_outer_loops.flight_director.Theta_c_deg;
        bad(SrsPitchLimit18, th > 18.0 + 1e-6, [&] { return std::string("FD Theta_c " + num(th)); });
      } else {
        idle(SrsPitchLimit18);
      }
    }

    bad(DoubleFailureDisengages, (lg.double_adr_failure || lg.double_ir_failure) && (lg.ap_own_engaged || lg.athr_own_engaged),
        [&] { return std::string(lg.double_adr_failure ? "dual ADR" : "dual IR") + " AP " + std::to_string(lg.ap_own_engaged) +
                     " ATHR " + std::to_string(lg.athr_own_engaged); });

    // ---- restart and robustness
    if (wasOff_) {
      altStarTarget_ = ap.alt_sel_or_cstr;
      altStarTargetKnown_ = lm.alt_acq_active;
      altStarReported_ = false;
      vsPendingSteps_ = -1;
      havePrev2_ = false;
    }
    if (wasOff_ && havePrev_) {
      stats_[CleanRestart].checked++;
      bad(CleanRestart, (anyVert || anyLat) && !ap.fmgc_opp_mode_sync, [&] { return "modes " + vertNames(vertMask_) + " / " + latNames(latMask_); }, false);
    }
    wasOff_ = false;
    {
      const auto& o = out.ap_fd_outer_loops;
      const double vals[] = {o.Phi_loc_c, o.Nosewheel_c, o.flight_director.Theta_c_deg, o.flight_director.Phi_c_deg,
                             o.flight_director.Beta_c_deg, o.autopilot.Theta_c_deg, o.autopilot.Phi_c_deg,
                             o.autopilot.Beta_c_deg, o.flare_law.H_dot_c_fpm, at.n1_c_percent, ap.spd_target_kts,
                             ap.pfd_spd_target_kts, ap.alt_sel_or_cstr, ap.tcas_vs_target};
      static const char* names[] = {"Phi_loc_c", "Nosewheel_c", "FD Theta_c", "FD Phi_c", "FD Beta_c", "AP Theta_c",
                                    "AP Phi_c", "AP Beta_c", "flare H_dot_c", "n1_c", "spd_target", "pfd_spd_target",
                                    "alt_sel_or_cstr", "tcas_vs_target"};
      int nonFinite = -1;
      for (size_t i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        if (!std::isfinite(vals[i]) && nonFinite < 0) {
          nonFinite = static_cast<int>(i);
        }
      }
      bad(OutputsFinite, nonFinite >= 0, [&] { return std::string(names[nonFinite]) + " = " + num(vals[nonFinite]); });
    }

    // ---- history
    if (havePrev_) {
      prev2Gs_ = prev_.ap_fd_logic.longitudinal_modes.gs_capt_active || prev_.ap_fd_logic.longitudinal_modes.gs_trk_active;
      prev2Land_ = prev_.ap_fd_logic.armed_modes.land_armed || prev_.ap_fd_logic.lateral_modes.land_active;
      prev2Nav_ = prev_.ap_fd_logic.lateral_modes.nav_active;
      havePrev2_ = true;
    }
    std::memcpy(&prev_, &out, sizeof(out));
    prevW1_ = w1;
    prevVertMask_ = vertMask_;
    prevLatMask_ = latMask_;
    havePrev_ = true;
  }

  /** Text report: one line per property. */
  void print(FILE* f) const {
    std::fprintf(f, "%-36s %10s %10s %10s %10s %10s  %s\n", "property", "checked", "bad", "violations", "first",
                 "t_first", "detail");
    for (int i = 0; i < kCount; i++) {
      const Stats& s = stats_[i];
      std::fprintf(f, "%-36s %10ld %10ld %10ld %10ld %10.2f  %s\n", list()[i].id, s.checked, s.badFrames, s.violations,
                   s.firstStep, s.firstTime, s.firstDetail.c_str());
    }
  }

 private:
  static double valueOr(const base_arinc_429& w, double d) { return w.SSM == 3 ? w.Data : d; }
  static uint32_t word(const base_arinc_429& w) {
    return w.Data >= 0.0F && w.Data < 4294967296.0F ? static_cast<uint32_t>(w.Data) : 0u;
  }
  static bool bit(uint32_t w, int n) { return (w >> (n - 1)) & 1u; }
  static std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.6g", v);
    return b;
  }
  static std::string athrNames(const base_fmgc_athr_outputs& a) {
    std::string s;
    if (a.alpha_floor_mode_active) s += "AFLOOR ";
    if (a.thrust_mode_active) s += "THR ";
    if (a.speed_mach_mode_active) s += "SPEED ";
    if (a.retard_mode_active) s += "RETARD ";
    return s;
  }

  static uint32_t mask(const uint8_t* v, size_t n) {
    uint32_t m = 0;
    for (size_t i = 0; i < n; i++) {
      m |= static_cast<uint32_t>(v[i] != 0) << i;
    }
    return m;
  }
  static std::string maskNames(uint32_t m, const char* const* names) {
    std::string s;
    for (int i = 0; (m >> i) != 0; i++) {
      if ((m >> i) & 1u) {
        s += std::string(s.empty() ? "" : "+") + names[i];
      }
    }
    return s;
  }

 public:
  static std::string vertNames(uint32_t m) {
    static const char* n[] = {"CLB", "DES", "OPCLB", "OPDES", "EXPCLB", "EXPDES", "SRSTO", "SRSGA", "VS",
                              "FPA", "ALT*", "ALT", "GS*", "GS", "FINAL", "FLARE", "TCAS"};
    return maskNames(m, n);
  }
  static std::string latNames(uint32_t m) {
    static const char* n[] = {"RWY", "RWYTRK", "NAV", "LOC*", "LOC", "GATRK", "HDG", "TRK", "LAND"};
    return maskNames(m, n);
  }
  /** This frame's vertical / lateral mode bitmasks (bit order of vertNames / latNames). */
  uint32_t vertMask() const { return vertMask_; }
  uint32_t latMask() const { return latMask_; }

 private:
  void idle(int id) {
    badFor_[id] = 0;
    badFrames_[id] = 0;
    inViolation_[id] = false;
  }

  /**
   * Feeds one frame's bad condition. `level` properties count every frame as checked; edge properties (level = false)
   * count their own checks.
   */
  template <typename F>
  void bad(int id, bool isBad, F&& detail, bool level = true) {
    Stats& s = stats_[id];
    if (level) {
      s.checked++;
    }
    if (!isBad) {
      badFor_[id] = 0;
      badFrames_[id] = 0;
      inViolation_[id] = false;
      return;
    }
    s.badFrames++;
    double tol = list()[id].toleranceS;
    badFrames_[id]++;
    badFor_[id] += dt_;
    bool over = tol < 0 ? badFrames_[id] > -tol : (tol == 0 || badFor_[id] > tol);
    if (over && !inViolation_[id]) {
      inViolation_[id] = true;
      s.violations++;
      if (s.firstStep < 0) {
        s.firstStep = step_;
        s.firstTime = t_;
        s.firstDetail = detail();
      }
      if (onViolation) {
        onViolation(id, step_, t_, s.firstStep == step_ ? s.firstDetail : detail());
      }
    }
    if (!level) {
      // edge properties: one bad frame is one episode
      badFor_[id] = 0;
      badFrames_[id] = 0;
      inViolation_[id] = false;
    }
  }

  std::vector<Stats> stats_;
  std::vector<double> badFor_;
  std::vector<int> badFrames_ = std::vector<int>(kCount, 0);
  std::vector<bool> inViolation_;
  fmgc_outputs prev_{};
  bool havePrev_ = false, havePrev2_ = false, wasOff_ = false;
  bool prev2Gs_ = false, prev2Land_ = false, prev2Nav_ = false;
  uint32_t prevW1_ = 0;
  long step_ = 0;
  double t_ = 0, dt_ = 0;
  double airborneS_ = -2, sinceStartS_ = 1e9, offS_ = 0;
  double altStarTarget_ = 0;
  bool altStarTargetKnown_ = false, altStarReported_ = false;
  int vsPendingSteps_ = -1;
  std::string vsPendingDetail_;
  bool flareSinceLand_ = false;
  double lawStableS_ = 0;
  uint32_t vertMask_ = 0, latMask_ = 0, prevVertMask_ = 0, prevLatMask_ = 0;
};

}  // namespace fmgc::props
