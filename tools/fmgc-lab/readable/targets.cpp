// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Targets block: HDG/TRK preset, selected/managed speed, Mach latch, speed targets, ALT soft mode.
// Spec: docs/modes/targets.md. Generated: FmgcComputer.cpp 3405-3434 (preset), 3436-3482 (selected speed), 3483-3522
// (managed speed), 3523-3536 (Mach), 3562-3712 (speed targets), 3713-3735 (ALT soft).

#include "targets.h"

#include <cmath>

namespace fmgc::readable::targets {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::rawValue;
using blocks::a429::valueOr;

constexpr double kPulseS = 0.72;                  // edge-triggered pulses (MTrigNode) last this long
constexpr double kOtherModeTakeoverDelayS = 0.72; // selected/managed replaces the other only after this

// HDG/TRK preset
constexpr double kPresetMaxRadioHeightFt = 30;

// Selected / managed speed
constexpr double kAirborneConfirmS = 5;           // "in flight": 5 s after lift-off
constexpr double kNoSpeed = 0;                    // the FMS sends 0 for "no managed speed" / "no V2"

// Speed target
constexpr double kSrsTakeoffIncrementKt = 10;     // SRS takeoff, all engines: V2 + 10
constexpr double kSrsTakeoffEoMaxIncrementKt = 15;// SRS takeoff, engine out: at most V2 + 15
constexpr int kSrsGaVlsIncrementKt = 25;          // SRS go-around: at most VLS + 25 (all engines) ...
constexpr int kSrsGaVlsIncrementEoKt = 15;        // ... VLS + 15 (engine out)
constexpr double kVmaxMarginKt = 5;               // SRS go-around and ground speed mini: at most VMAX - 5
constexpr double kGsMiniGain = 0.33333333333333331;  // ground speed mini: 1/3 of the headwind gust
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kApproachLandArmedMaxRaFt = 700; // VAPP / runway heading frozen below 700 ft RA with LAND armed
constexpr int kFlapLever3 = 4;                    // flap_slat_lever_position of CONF 3
constexpr float kMachToCasKt = 1479.1F;           // sqrt(5) x speed of sound at sea level (661.47 kt)
constexpr float kSeaLevelPressureHpa = 1013.25F;

// ALT soft mode
constexpr double kSoftModeSpeedBandKt = 4;        // speed within 4 kt of the target ...
constexpr double kSoftModeEngageS = 120;          // ... for 2 min engages it
constexpr double kSoftModeSpeedOffS = 10;         // speed off by more than 4 kt for 10 s drops it
constexpr double kSoftModeLargeSpeedOffKt = 10;   // speed off by more than 10 kt ...
constexpr double kSoftModeLargeSpeedOffS = 4;     // ... for 4 s drops it
constexpr double kSoftModeVmaxMarginKt = 5;       // CAS above VMAX - 5 kt drops it

// FCU discrete word 1 (bit = index + 11)
constexpr int kFcuWord1SpdKnobPushed = 11;
constexpr int kFcuWord1SpdKnobPulled = 12;
constexpr int kFcuWord1Bit15 = 15;                // resets the ALT soft mode (see docs/modes/targets.md)
constexpr int kFcuWord1SpdMachPushed = 21;        // SPD/MACH pushbutton
// FCU discrete word 2
constexpr int kFcuWord2HdgKnobPushed = 11;
constexpr int kFcuWord2HdgKnobPulled = 12;
// Opposite FMGC discrete word 5
constexpr int kOppWord5AutoSpeed = 19;
constexpr int kOppWord5ManualSpeed = 20;

bool latch(bool set, bool reset, bool q) {
  return reset ? false : (set ? true : q);
}

bool fcuWord1Bit(const fmgc_inputs& in, int n) {
  return bit(in.bus_inputs.fcu_bus.fcu_discrete_word_1, n) != 0;
}

bool fcuWord2Bit(const fmgc_inputs& in, int n) {
  return bit(in.bus_inputs.fcu_bus.fcu_discrete_word_2, n) != 0;
}

/** Selected radio height (ft), raw: 2500 ft when both radio altimeters failed. */
float radioHeight(const base_fmgc_logic_outputs& logic) {
  return rawValue(logic.ra_computation_data.radio_height_ft);
}

/** MATLAB mod(x, 360) (generated rt_modd with a divisor of 360): [0, 360), NaN for NaN or infinite x. */
double wrap360(double x) {
  if (x == 0.0) {
    return 0.0;
  }
  double y = std::fmod(x, 360.0);
  if (y == 0.0) {
    return 0.0;
  }
  return x < 0.0 ? y + 360.0 : y;
}

/** The smaller of a and b, as the generated `if (a > b) a = b` (b when a is NaN, a when b is NaN). */
double lowerOf(double a, double b) {
  return a > b ? b : a;
}

void mark(Trace* t, Cond c, bool v) {
  if (t != nullptr && v) {
    t->c[c] = true;
  }
}

/**
 * In final approach or on the runway (previous frame): RWY, LAND, or LAND armed below 700 ft RA. The VAPP memo (and
 * the runway heading memo, logic.rwy_hdg_memo) stop tracking then. [rtb_y_d at 1366; its negation rtb_y_e0]
 */
bool finalApproachOrRunway(const Inputs& in) {
  const auto& l = in.last;
  return l.lateral_modes.rwy_loc_submode_active || l.lateral_modes.land_active ||
         (l.armed_modes.land_armed && radioHeight(in.logic) <= kApproachLandArmedMaxRaFt);
}

}  // namespace

const char* fieldName(int field) {
  static const char* const names[] = {"hdg_trk_preset", "manual_spd",  "auto_spd",     "mach",
                                      "vapp_memo",      "ga_spd_memo", "eo_spd_memo",  "lower_margin",
                                      "spd_target",     "pfd_spd_target", "alt_soft"};
  static_assert(sizeof(names) / sizeof(names[0]) == kFieldCount, "one name per field");
  return names[field];
}

const char* condName(int cond) {
  static const char* const names[] = {
      "manual_set_opp_sync", "manual_set_power_up_in_flight", "manual_set_no_managed_at_liftoff",
      "manual_set_no_ap_fd", "manual_set_knob_pulled", "manual_set_preset", "manual_set_managed_lost",
      "manual_set_tcas", "manual_reset_by_auto", "manual_reset_ground",
      "auto_set_opp_sync", "auto_set_ground_ap_fd", "auto_set_srs", "auto_set_knob_pushed", "auto_set_exped",
      "auto_set_no_speeds", "auto_reset_by_manual", "auto_reset_ground",
      "mach_set_push", "mach_set_fms", "mach_set_preset", "mach_reset_push", "mach_reset_fms", "mach_reset_preset",
      "mach_set_and_reset", "mach_push_in_managed",
      "target_selected_speed", "target_selected_mach", "target_managed", "target_managed_upper_margin",
      "target_managed_lower_margin", "target_approach_gs_mini", "target_approach_vapp", "target_gs_mini_limited",
      "target_gs_mini_vmax_limit", "target_srs_takeoff", "target_srs_takeoff_engine_out", "target_srs_go_around",
      "target_floor_active", "target_f3_no_f_floor",
      "alt_soft_condition", "alt_soft_engage", "alt_soft_reset_speed_off", "alt_soft_reset_vmax",
      "alt_soft_reset_fcu"};
  static_assert(sizeof(names) / sizeof(names[0]) == kCondCount, "one name per condition");
  return names[cond];
}

double Values::get(int field) const {
  switch (field) {
    case HdgTrkPreset: return hdgTrkPreset;
    case ManualSpeed: return manualSpeed;
    case AutoSpeed: return autoSpeed;
    case MachMode: return machMode;
    case VappMemo: return vappMemoKts;
    case GaSpeedMemo: return gaSpeedMemoKts;
    case EoSpeedMemo: return eoSpeedMemoKts;
    case LowerMarginActive: return lowerMarginActive;
    case SpdTarget: return spdTargetKts;
    case PfdSpdTarget: return pfdSpdTargetKts;
    case AltSoft: return altSoft;
    default: return std::nan("");
  }
}

void Values::set(int field, double v) {
  switch (field) {
    case HdgTrkPreset: hdgTrkPreset = v != 0; break;
    case ManualSpeed: manualSpeed = v != 0; break;
    case AutoSpeed: autoSpeed = v != 0; break;
    case MachMode: machMode = v != 0; break;
    case VappMemo: vappMemoKts = v; break;
    case GaSpeedMemo: gaSpeedMemoKts = v; break;
    case EoSpeedMemo: eoSpeedMemoKts = v; break;
    case LowerMarginActive: lowerMarginActive = v != 0; break;
    case SpdTarget: spdTargetKts = v; break;
    case PfdSpdTarget: pfdSpdTargetKts = v; break;
    case AltSoft: altSoft = v != 0; break;
    default: break;
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// HDG/TRK preset [3405-3434]
// ---------------------------------------------------------------------------------------------------------------------

bool updateHdgTrkPreset(HdgTrkPresetState& st, double dt, const fmgc_inputs& in, const base_fmgc_logic_outputs& logic,
                        const PresetModes& m, bool anyApFdEngaged, bool q, Trace* /*trace*/) {
  // A heading/track preset (FCU selected value, or the knob pulled) is kept when made in a guided mode or on the
  // ground; it is cleared by HDG/TRK, NAV, the knob pushed, no AP/FD, or leaving those conditions.
  const auto& fcu = in.bus_inputs.fcu_bus;
  bool presetSelected =
      isNormal(fcu.selected_hdg_deg) || isNormal(fcu.selected_trk_deg) || fcuWord2Bit(in, kFcuWord2HdgKnobPulled);
  bool selectedPulse = st.selectPulse.update(presetSelected, dt, true, false, kPulseS);
  bool navArmedRecently = st.navArmedRecent.update(m.navArmed, dt, false, kPulseS);
  bool guidedMode = m.locCapture || m.locTrack || m.land || m.final || m.gaTrk || m.rwyTrk;
  bool presetPossible = navArmedRecently || guidedMode;
  bool guidedOrLow =
      guidedMode || logic.on_ground || (radioHeight(logic) < kPresetMaxRadioHeightFt && !logic.dual_ra_failure);
  bool set = selectedPulse && presetPossible && guidedOrLow;
  bool knobPush = st.knobPushPulse.update(fcuWord2Bit(in, kFcuWord2HdgKnobPushed), dt, true, false, kPulseS);
  bool noApFdRecently = st.noApFdRecent.update(!anyApFdEngaged, dt, false, kPulseS);
  bool reset = !guidedOrLow || m.hdg || m.trk || m.nav || knobPush || noApFdRecently;
  return latch(set, reset, q);
}

// ---------------------------------------------------------------------------------------------------------------------
// ALT soft mode [3713-3735]
// ---------------------------------------------------------------------------------------------------------------------

bool updateAltSoft(AltSoftState& st, double dt, const fmgc_inputs& in, const base_fmgc_logic_outputs& logic,
                   bool altHold, bool machMode, double spdTargetKts, bool q, Trace* trace) {
  // After 2 min of stable Mach cruise in ALT with A/THR, the soft altitude mode engages.
  bool cond = altHold && in.fms_inputs.fms_flight_phase == fmgc_flight_phase::Cruise && machMode &&
              logic.athr_own_engaged;
  float cas = rawValue(logic.adr_computation_data.airspeed_computed_kn);
  double speedError = std::abs(spdTargetKts - cas);
  bool speedOff = speedError > kSoftModeSpeedBandKt;
  bool speedOff10s = st.speedOff4kt10s.update(speedOff, dt, true, kSoftModeSpeedOffS);
  bool largeSpeedOff4s =
      st.speedOff10kt4s.update(speedError > kSoftModeLargeSpeedOffKt, dt, true, kSoftModeLargeSpeedOffS);
  float vmax = valueOr(logic.chosen_fac_bus.v_max_kn, 0.0F);
  bool aboveVmax = static_cast<double>(cas) > static_cast<double>(vmax) - kSoftModeVmaxMarginKt;
  bool fcuReset = fcuWord1Bit(in, kFcuWord1Bit15);
  bool reset = !cond || speedOff10s || largeSpeedOff4s || aboveVmax || fcuReset;
  bool set = st.softCondFor120s.update(cond && !speedOff && !reset, dt, true, kSoftModeEngageS);
  mark(trace, AltSoftCondition, cond);
  mark(trace, AltSoftEngage, set && !reset);
  mark(trace, AltSoftResetSpeedOff, cond && (speedOff10s || largeSpeedOff4s));
  mark(trace, AltSoftResetVmax, cond && aboveVmax);
  mark(trace, AltSoftResetFcu, cond && fcuReset);
  return latch(set, reset, q);
}

// ---------------------------------------------------------------------------------------------------------------------
// The block
// ---------------------------------------------------------------------------------------------------------------------

void Targets::initialize(Values& v) {
  reset(v);
  v.vappMemoKts = 0;  // FmgcComputer_B.u_l = Y_Y0_g, only in initialize()
}

void Targets::reset(Values& v) {
  double vapp = v.vappMemoKts;  // not reset on restart
  v = Values{};
  v.vappMemoKts = vapp;
  st_ = State{};
}

void Targets::update(Field id, const Inputs& f, Values& v, Trace& trace) {
  const fmgc_inputs& in = f.in;
  const auto& fms = in.fms_inputs;
  const auto& logic = f.logic;
  const auto& lm = f.modes.longitudinal_modes;
  const auto& lat = f.modes.lateral_modes;
  const auto& oppWord5 = in.bus_inputs.fmgc_opp_bus.discrete_word_5;
  bool oppSync = f.modes.fmgc_opp_mode_sync;   // this FMGC copies the opposite FMGC's modes
  bool anyApFd = f.modes.any_ap_fd_engaged;
  bool onGround = logic.on_ground;
  bool srsTakeoff = lm.pitch_takeoff_active;
  bool srsGoAround = lm.pitch_goaround_active;
  Trace* t = &trace;

  switch (id) {
    case HdgTrkPreset: {
      PresetModes m{};
      m.navArmed = f.modes.armed_modes.nav_armed;
      m.nav = lat.nav_active;
      m.locCapture = lat.loc_cpt_active;
      m.locTrack = lat.loc_trk_active;
      m.land = lat.land_active;
      m.final = lm.final_des_active;
      m.gaTrk = lat.roll_goaround_active;
      m.rwyTrk = lat.rwy_trk_submode_active;
      m.hdg = lat.hdg_active;
      m.trk = lat.trk_active;
      v.hdgTrkPreset = updateHdgTrkPreset(st_.preset, f.dt, in, logic, m, anyApFd, v.hdgTrkPreset, t);
      return;
    }

    case ManualSpeed: {
      // Selected speed: power-up in flight, no FMS speed, no AP/FD outside the approach phase, the SPD knob pulled, a
      // preselected speed, TCAS; managed speed replaces it 0.72 s after the last of these. On the ground, cleared at
      // engine start and at the first AP/FD engagement.
      bool airborne5s = st_.airborne5s.update(!onGround, f.dt, true, kAirborneConfirmS);
      bool fiveSecondsAfterLiftoff = st_.liftoff5sPulse.update(airborne5s, true);
      bool knobPulled =
          st_.knobPulled.update(fcuWord1Bit(in, kFcuWord1SpdKnobPulled), true) && logic.gnd_eng_stop_flt_5s;
      float altitude = valueOr(logic.altitude_indicated_ft, 0.0F);
      float fcuAltitude = valueOr(in.bus_inputs.fcu_bus.selected_alt_ft, 0.0F);
      // a preselected speed, except in SRS takeoff above the FCU altitude
      bool preset = fms.preset_spd_mach_activate && (!srsTakeoff || altitude <= fcuAltitude);
      bool managedLost = st_.managedLostManual.update(fms.v_managed_kts == kNoSpeed, true);
      bool fmSpeedLossReverts = !srsTakeoff && !srsGoAround && !lm.gs_capt_active && !lm.gs_trk_active &&
                                   !lat.land_active;  // SRS, G/S and LAND keep managed speed when the FMS speed is lost
      bool tcasEngaged = st_.tcasEngaged.update(lm.tcas_active, true);

      bool powerUpInFlight = f.firstFrame && !onGround;
      bool noManagedAtLiftoff = fms.v_managed_kts == kNoSpeed && fiveSecondsAfterLiftoff && fmSpeedLossReverts;
      bool noApFd = !anyApFd && fms.fms_flight_phase != fmgc_flight_phase::Approach;
      bool managedLostInFlight = managedLost && !onGround && fmSpeedLossReverts;
      bool oppManual = bit(oppWord5, kOppWord5ManualSpeed) != 0 && oppSync;
      bool set = oppManual || (!oppSync && (powerUpInFlight || noManagedAtLiftoff || noApFd || knobPulled || preset ||
                                            managedLostInFlight || tcasEngaged));

      bool setRecently = st_.manualSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool replacedByAuto = f.last.auto_spd_control_active && !setRecently;
      bool engineStart = st_.engineStartManual.update(logic.engine_running, true);
      bool apFdEngaged = st_.apFdEngagedManual.update(anyApFd, true);
      bool groundReset = onGround && (engineStart || apFdEngaged);
      bool reset = replacedByAuto || groundReset;
      v.manualSpeed = latch(set, reset, v.manualSpeed);

      mark(t, ManualSetOppSync, oppManual);
      bool own = !oppSync;
      mark(t, ManualSetPowerUpInFlight, own && powerUpInFlight);
      mark(t, ManualSetNoManagedAtLiftoff, own && noManagedAtLiftoff);
      mark(t, ManualSetNoApFd, own && noApFd);
      mark(t, ManualSetKnobPulled, own && knobPulled);
      mark(t, ManualSetPreset, own && preset);
      mark(t, ManualSetManagedLost, own && managedLostInFlight);
      mark(t, ManualSetTcas, own && tcasEngaged);
      mark(t, ManualResetByAuto, replacedByAuto);
      mark(t, ManualResetGround, groundReset);
      return;
    }

    case AutoSpeed: {
      // Managed speed: on the ground when an AP/FD and V2 are both there, SRS, the SPD knob pushed, EXPED; needs a
      // managed speed or V2 from the FMS. Selected speed replaces it 0.72 s after the last set condition. On the ground
      // outside SRS and the approach, cleared when the FMS speed or V2 is lost.
      bool apFdEngaged = st_.apFdEngagedAuto.update(anyApFd, true);
      bool hasV2 = fms.v_2_kts != kNoSpeed;
      bool engineStart = st_.engineStartAuto.update(logic.engine_running, true);
      bool v2Inserted = st_.v2Inserted.update(hasV2, true) && anyApFd;
      bool groundApFd = (apFdEngaged && hasV2) || (engineStart && hasV2 && anyApFd) || v2Inserted;
      bool srsEngaged = st_.srsEngaged.update(srsGoAround || srsTakeoff, true);
      bool knobPushed = st_.knobPushed.update(fcuWord1Bit(in, kFcuWord1SpdKnobPushed), true);
      bool expedEngaged = st_.expedEngaged.update(lm.exp_clb_active || lm.exp_des_active, true);
      bool cause = (onGround && groundApFd) || srsEngaged || knobPushed || expedEngaged;
      bool haveSpeeds = fms.v_managed_kts != kNoSpeed || fms.v_2_kts != kNoSpeed;
      bool oppAuto = bit(oppWord5, kOppWord5AutoSpeed) != 0 && oppSync;
      bool set = oppAuto || (!oppSync && cause && haveSpeeds);

      bool setRecently = st_.autoSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool replacedByManual = f.last.manual_spd_control_active && !setRecently;
      bool managedLost = st_.managedLostAuto.update(fms.v_managed_kts == kNoSpeed, true);
      bool v2Deleted = st_.v2Deleted.update(fms.v_2_kts == kNoSpeed, true);
      bool groundReset = onGround && (!srsGoAround && !srsTakeoff && !finalApproachOrRunway(f)) &&
                         (managedLost || v2Deleted);
      bool reset = replacedByManual || groundReset;
      v.autoSpeed = latch(set, reset, v.autoSpeed);

      mark(t, AutoSetOppSync, oppAuto);
      bool own = !oppSync;
      mark(t, AutoSetGroundApFd, own && onGround && groundApFd && haveSpeeds);
      mark(t, AutoSetSrs, own && srsEngaged && haveSpeeds);
      mark(t, AutoSetKnobPushed, own && knobPushed && haveSpeeds);
      mark(t, AutoSetExped, own && expedEngaged && haveSpeeds);
      mark(t, AutoSetNoSpeeds, own && cause && !haveSpeeds);
      mark(t, AutoResetByManual, replacedByManual);
      mark(t, AutoResetGround, groundReset);
      return;
    }

    case MachMode: {
      // SPD/MACH: the pushbutton toggles; the FMS (managed crossover) and the MCDU presets set it explicitly. Reset
      // wins. Independent of selected/managed speed and of the opposite FMGC.
      bool mach = f.last.mach_control_active;
      bool pushed = st_.spdMachPushed.update(fcuWord1Bit(in, kFcuWord1SpdMachPushed), true);
      bool presetMach = fms.preset_spd_mach_activate && fms.preset_mach > 0;
      bool presetSpeed = fms.preset_spd_mach_activate && fms.preset_spd_kts > 0;
      bool set = (!mach && pushed) || fms.fms_mach_mode_activate || presetMach;
      bool reset = (pushed && mach) || fms.fms_spd_mode_activate || presetSpeed;
      v.machMode = latch(set, reset, mach);

      mark(t, MachSetPush, !mach && pushed);
      mark(t, MachSetFms, fms.fms_mach_mode_activate);
      mark(t, MachSetPreset, presetMach);
      mark(t, MachResetPush, pushed && mach);
      mark(t, MachResetFms, fms.fms_spd_mode_activate);
      mark(t, MachResetPreset, presetSpeed);
      mark(t, MachSetAndReset, set && reset);
      mark(t, MachPushInManaged, pushed && v.autoSpeed);
      return;
    }

    case VappMemo:
      // VAPP, frozen in final approach and on the runway (the managed approach speed is then the memorised VAPP).
      if (!finalApproachOrRunway(f)) {
        v.vappMemoKts = fms.v_app_kts;
      }
      return;

    case GaSpeedMemo: {
      // CAS while SRS go-around is not engaged: at go-around, the speed when it was initiated.
      float cas = valueOr(logic.adr_computation_data.airspeed_computed_kn, 0.0F);
      if (!st_.gaSpeedMemoPrimed) {
        v.gaSpeedMemoKts = cas;
        st_.gaSpeedMemoPrimed = true;
      }
      if (!srsGoAround) {
        v.gaSpeedMemoKts = cas;
      }
      return;
    }

    case EoSpeedMemo: {
      // CAS while both engines run: with an engine out, the speed at the failure.
      float cas = valueOr(logic.adr_computation_data.airspeed_computed_kn, 0.0F);
      if (!st_.eoSpeedMemoPrimed) {
        v.eoSpeedMemoKts = cas;
        st_.eoSpeedMemoPrimed = true;
      }
      if (!logic.one_engine_out) {
        v.eoSpeedMemoKts = cas;
      }
      return;
    }

    case LowerMarginActive: {
      // DES in VPATH_SPEED: once the speed falls below the lower margin, the target is the lower margin until the speed
      // is back above the managed speed (or the submode changes).
      float cas = valueOr(logic.adr_computation_data.airspeed_computed_kn, 0.0F);
      bool vpathSpeed = fms.requested_des_submode == fmgc_des_submode::VPATH_SPEED;
      bool set = cas < fms.v_lower_margin_kts && vpathSpeed;
      bool reset = cas > fms.v_managed_kts || !vpathSpeed;
      v.lowerMarginActive = latch(set, reset, v.lowerMarginActive);
      return;
    }

    case SpdTarget: {
      // V_c (for the laws and the A/THR) and the PFD target. Managed: SRS, approach (ground speed mini), DES margins,
      // or the FMS speed, raised to the manoeuvring speed floor except in SRS; selected: the FCU speed or Mach (as
      // CAS).
      // The PFD shows the unfloored managed target (the FMS speed, not the margins).
      const auto& fac = logic.chosen_fac_bus;
      float cas = valueOr(logic.adr_computation_data.airspeed_computed_kn, 0.0F);
      float vls = valueOr(fac.v_ls_kn, 0.0F);
      float vmax = valueOr(fac.v_max_kn, 0.0F);

      // Manoeuvring speed floor: max(green dot, S, F), without F in CONF 3 with a CONF 3 landing in the approach phase.
      bool approachPhase = fms.fms_flight_phase == fmgc_flight_phase::Approach;
      bool conf3Landing =
          logic.flap_slat_lever_position == kFlapLever3 && fms.flap_3_approach_selected && approachPhase;
      double fSpeedFloor = conf3Landing ? 0.0 : valueOr(fac.v_3_kn, 0.0F);
      double floor = std::fmax(std::fmax(static_cast<double>(valueOr(fac.v_man_kn, 0.0F)),
                                         static_cast<double>(valueOr(fac.v_4_kn, 0.0F))),
                               fSpeedFloor);

      // SRS speeds (computed every frame; used when SRS is engaged in managed speed)
      double srsTarget, srsPfdTarget;
      if (srsGoAround) {
        // the speed at go-around, at least VAPP, at most VLS + 25 (15 engine out) and VMAX - 5
        int vlsIncrement = logic.one_engine_out ? kSrsGaVlsIncrementEoKt : kSrsGaVlsIncrementKt;
        double limit =
            lowerOf(lowerOf(vmax - kVmaxMarginKt, vls + static_cast<double>(vlsIncrement)), v.gaSpeedMemoKts);
        srsTarget = srsPfdTarget = std::fmax(v.vappMemoKts, limit);
      } else if (logic.one_engine_out) {
        // the speed at the failure, at least V2, at most V2 + 15
        srsTarget = srsPfdTarget =
            std::fmax(fms.v_2_kts, std::fmin(fms.v_2_kts + kSrsTakeoffEoMaxIncrementKt, v.eoSpeedMemoKts));
      } else {
        srsTarget = fms.v_2_kts + kSrsTakeoffIncrementKt;  // V2 + 10 flown, V2 shown
        srsPfdTarget = fms.v_2_kts;
      }

      double target, pfdTarget;
      if (v.autoSpeed) {
        bool approach = lat.land_active || f.modes.armed_modes.land_armed || approachPhase;
        if (approach) {
          // Ground speed mini: VAPP + 1/3 of (headwind along the memorised runway course - tower headwind), not below
          // VAPP, at most VFE next (VMAX - 5 when VFE next is not available).
          double approachTarget;
          if (isNormal(fms.tower_headwind_kts)) {
            float windDirection = valueOr(logic.ir_computation_data.wind_direction_true_deg, 0.0F);
            float windSpeed = valueOr(logic.ir_computation_data.wind_speed_kn, 0.0F);
            float towerHeadwind = rawValue(fms.tower_headwind_kts);
            double clockwise = wrap360((windDirection - (logic.rwy_hdg_memo + 360.0)) + 360.0);
            double counterClockwise = wrap360(360.0 - clockwise);
            double windAngle = clockwise < counterClockwise ? -clockwise : counterClockwise;
            double headwind = std::cos(kDegToRad * windAngle) * windSpeed;
            float limit = isNormal(fac.v_fe_next_kn)
                              ? valueOr(fac.v_fe_next_kn, 0.0F)
                              : static_cast<float>(valueOr(fac.v_max_kn, 0.0F) + -kVmaxMarginKt);
            double gsMini = std::fmax(headwind - towerHeadwind, 0.0) * kGsMiniGain + v.vappMemoKts;
            approachTarget = std::fmin(static_cast<double>(limit), gsMini);
            mark(t, TargetApproachGsMini, true);
            mark(t, TargetGsMiniLimited, static_cast<double>(limit) < gsMini);
            mark(t, TargetGsMiniVmaxLimit, !isNormal(fac.v_fe_next_kn));
          } else {
            approachTarget = v.vappMemoKts;
            mark(t, TargetApproachVapp, true);
          }
          target = std::fmax(floor, approachTarget);
          pfdTarget = approachTarget;
          mark(t, TargetFloorActive, floor > approachTarget);
        } else if (srsGoAround || srsTakeoff) {
          target = srsTarget;
          pfdTarget = srsPfdTarget;
          Cond srs = srsGoAround ? TargetSrsGoAround
                                 : (logic.one_engine_out ? TargetSrsTakeoffEngineOut : TargetSrsTakeoff);
          mark(t, srs, true);
        } else {
          // DES with speed margins: idle path (SPEED_THRUST) flies the upper margin; on the path with speed on thrust
          // (VPATH_SPEED) the lower margin once below it.
          bool margins = lm.des_active && fms.show_speed_margins;
          double managed;
          if (fms.requested_des_submode == fmgc_des_submode::SPEED_THRUST && margins) {
            managed = fms.v_upper_margin_kts;
            mark(t, TargetManagedUpperMargin, true);
          } else if (margins && v.lowerMarginActive) {
            managed = fms.v_lower_margin_kts;
            mark(t, TargetManagedLowerMargin, true);
          } else {
            managed = fms.v_managed_kts;
            mark(t, TargetManaged, true);
          }
          target = std::fmax(floor, managed);
          pfdTarget = fms.v_managed_kts;
          mark(t, TargetFloorActive, floor > managed);
        }
        mark(t, TargetF3NoFFloor, conf3Landing && !(srsGoAround || srsTakeoff));
      } else {
        // Selected: the FCU speed, or the FCU Mach converted to CAS at the current static pressure (both raw: no SSM
        // check).
        const auto& fcu = in.bus_inputs.fcu_bus;
        float selected;
        if (v.machMode) {
          float pressureHpa = valueOr(logic.adr_computation_data.corrected_average_static_pressure, 0.0F);
          float mach = fcu.selected_mach.Data;
          float y = std::pow((std::pow(mach * mach * 0.2F + 1.0F, 3.5F) - 1.0F) * (pressureHpa / kSeaLevelPressureHpa) +
                                 1.0F,
                             0.285714298F);
          selected = std::sqrt(y - 1.0F) * kMachToCasKt;
          mark(t, TargetSelectedMach, true);
        } else {
          selected = fcu.selected_spd_kts.Data;
          mark(t, TargetSelectedSpeed, true);
        }
        target = pfdTarget = selected;
      }
      v.spdTargetKts = target;
      v.pfdSpdTargetKts = pfdTarget;
      return;
    }

    case PfdSpdTarget:
      return;  // computed with SpdTarget

    case AltSoft:
      v.altSoft = updateAltSoft(st_.altSoft, f.dt, in, logic, lm.alt_hold_active, v.machMode, v.spdTargetKts,
                                v.altSoft, t);
      return;

    default:
      return;
  }
}

}  // namespace fmgc::readable::targets
