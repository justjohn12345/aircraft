// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Altitude target (alt_cstr_applicable / alt_sel_or_cstr), ALT arm possible, ALT armed, ALT* (altitude capture),
// ALT (altitude hold), ALT CRZ.
// Spec: docs/modes/alt-vs-fpa-tcas.md. Generated: FmgcComputer.cpp 1636–1664 (target), 2405–2610.

#include <cmath>

#include "vertical.h"

namespace fmgc::readable {

namespace {

using blocks::a429::bit;
using blocks::a429::rawValue;
using blocks::a429::valueOr;

constexpr double kAltArmPossibleResetBandFt = 40;   // ALT within this of the FCU altitude: no re-arming
constexpr double kAltArmPossibleSetDeviationFt = 250;
constexpr double kFcuChangeCaptureInhibitS = 3;     // no capture for 3 s after an FCU altitude change
constexpr double kOtherModeTakeoverDelayS = 0.72;   // another mode replaces ALT*/ALT only after this
constexpr double kAltHoldBandFt = 40;               // ALT engages within 40 ft ...
constexpr double kAltHoldConfirmS = 0.8;            // ... held for 0.8 s
constexpr double kSrsMinRadioHeightFt = 400;        // in SRS, ALT* only above 400 ft RA
constexpr double kCruiseAltBandFt = 60;
constexpr double kCstrWrongSideMarginFt = 250;      // a constraint up to 250 ft behind the aircraft still applies

constexpr int kFcuWord1AltChanged = 13;   // 0.5 s pulse when the FCU altitude value changes
constexpr int kOppWord1AltCaptOrHold = 19;  // opposite FMGC: ALT* or ALT
constexpr int kOppWord1AltHoldOrGs = 20;    // opposite FMGC: ALT or G/S
constexpr int kOppWord1AltCaptOrGsCapt = 21;  // opposite FMGC: ALT* or G/S*
constexpr int kOppWord3AltArmed = 12;
constexpr int kTcasRaWordRaActive = 20;   // TCAS RA word: RA in progress

/** Opposite-FMGC discrete words. */
const auto& oppBus(const Frame& f) {
  return f.in.bus_inputs.fmgc_opp_bus;
}

bool latch(bool set, bool reset, bool q) {
  return reset ? false : (set ? true : q);
}

int sign(double x) {
  return x < 0.0 ? -1 : (x > 0.0);
}

/**
 * Altitude capture distance (ft) for a vertical speed (fpm): the distance to level off with a normal load factor
 * f(|V/S|) (piecewise cubic, 0.05 g at low V/S up to 0.15 g), limited to 80..3000 ft. Float arithmetic as generated.
 */
float captureDistanceFt(float vsFpm) {
  static const int16_t breakpoints[7]{0, 1000, 3333, 4000, 6000, 8000, 10000};
  static const double coefs[24]{
      -3.7631613045100394E-12, -3.7631613045100418E-12, 6.2076488130688133E-12, 2.3375903616618146E-12,
      -2.9675180723323623E-12, -2.9675180723323619E-12, 2.2735910872868498E-8, 1.1446426959338374E-8,
      -1.4891939010927404E-8, -2.4704337359767112E-9, 1.1555108433994175E-8, -6.25E-9, -1.897274956835846E-5,
      1.520958826384842E-5, 7.1712086474912069E-6, -4.4094939746938354E-6, 1.3759855421341094E-5,
      2.4370072289329445E-5, 0.05, 0.05, 0.1, 0.1, 0.1, 0.15};
  float absVs = std::abs(vsFpm);
  // segment: binary search as generated (last segment extrapolated above 10000 fpm)
  int high = 7, k = 0, lowp1 = 2;
  while (high > lowp1) {
    int mid = ((k + high) + 1) >> 1;
    if (static_cast<double>(absVs) >= breakpoints[mid - 1]) {
      k = mid - 1;
      lowp1 = mid + 1;
    } else {
      high = mid;
    }
  }
  float dx = absVs - static_cast<float>(breakpoints[k]);
  float loadFactorG = ((dx * static_cast<float>(coefs[k]) + static_cast<float>(coefs[k + 6])) * dx +
                       static_cast<float>(coefs[k + 12])) *
                          dx +
                      static_cast<float>(coefs[k + 18]);
  float vsMs = vsFpm * 0.00508F;
  return std::fmin(3000.0F, std::fmax(80.0F, vsMs * vsMs / (loadFactorG * 9.81F) * 3.28084F));
}

// In TCAS mode, ALT* or ALT may only engage in the matching TCAS submode, with no RA in progress.
bool tcasAllows(const Frame& f, tcas_submode submode) {
  bool raActive = bit(f.in.bus_inputs.tcas_bus.vertical_resolution_advisory, kTcasRaWordRaActive) != 0;
  return !f.last.longitudinal_modes.tcas_active ||
         (!raActive && f.last.active_tcas_submode == submode && f.ext.tcasModeAvailable);
}

}  // namespace

void VerticalModeLogic::resetAlt(Signals& s) {
  s[Signal::AltArmPossible] = true;
  s[Signal::AltCapture] = false;
  s[Signal::AltHold] = false;
  s[Signal::AltCruiseMemory] = false;
  alt_.fcuAltChangedRecently.reset();
  alt_.captureCondRecent.reset();
  alt_.withinAltHoldBand.reset();
  alt_.altHoldCondRecent.reset();
}

bool VerticalModeLogic::updateAlt(Signal id, const Frame& f, Signals& s) {
  const auto& last = f.last.longitudinal_modes;
  const bool ownCompute = !f.ext.oppModeSync;
  const float h = indicatedAltitude(f);

  switch (id) {
    case Signal::AltCstrApplicable: {
      // The FMS altitude constraint is the ALT*/ALT target when a managed climb or descent is armed or active (or
      // ALT*/ALT is already levelling at a constraint), and the constraint lies between the aircraft and the FCU
      // altitude. All conditions read the previous frame's modes.
      const auto& lastLat = f.last.lateral_modes;
      const auto& lastArmed = f.last.armed_modes;
      const double cstr = f.in.fms_inputs.next_alt_cstr_ft;  // 0: no constraint
      const float fcuAlt = fcuAltitude(f);
      // ALT*/ALT on a constraint in NAV/LOC* /LOC with a vertical flight plan: keeps the constraint applicable through
      // the capture, also once the aircraft is within 250 ft behind it.
      bool levelAtCstr = (last.alt_acq_active || last.alt_hold_active) && f.last.alt_cstr_applicable &&
                         (lastLat.nav_active || lastLat.loc_cpt_active || lastLat.loc_trk_active) &&
                         f.in.fms_inputs.vertical_flight_plan_valid;
      bool between;  // constraint between the aircraft (- 250 ft) and the FCU altitude
      if (fcuAlt > h) {  // climbing towards the FCU altitude
        between = cstr < fcuAlt && (cstr + kCstrWrongSideMarginFt > h || levelAtCstr);
      } else {
        between = cstr > fcuAlt && (cstr - kCstrWrongSideMarginFt < h || levelAtCstr);
      }
      bool managed = lastArmed.clb_armed || lastArmed.des_armed || last.clb_active || last.des_active || levelAtCstr;
      s[id] = managed && cstr != 0.0 && between;
      return true;
    }

    case Signal::AltSelOrCstr:
      // The ALT*/ALT target: always the current constraint value (not the one captured), else the FCU altitude.
      s.altSelOrCstr = s[Signal::AltCstrApplicable] ? f.in.fms_inputs.next_alt_cstr_ft : fcuAltitude(f);
      s[id] = s.altSelOrCstr != 0.0;
      return true;

    case Signal::AltArmPossible: {
      // ALT can be armed again once the FCU altitude changes or the aircraft is more than 250 ft from it; not while
      // holding ALT at the FCU altitude.
      float deviation = std::abs(h - fcuAltitude(f));
      bool set = fcuWord1Bit(f, kFcuWord1AltChanged) || deviation > kAltArmPossibleSetDeviationFt;
      bool reset = last.alt_hold_active && deviation <= kAltArmPossibleResetBandFt;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::AltArmed: {
      // Vertical target of the current V/S, FPA or TCAS mode (0 otherwise), to know whether it points at the FCU
      // altitude.
      const auto& fcu = f.in.bus_inputs.fcu_bus;
      float target = valueOr(fcu.selected_fpa_deg, 0.0F);
      if (last.vs_active) {
        target = valueOr(fcu.selected_vz_ft_min, 0.0F);
      } else if (!last.fpa_active) {
        target = last.tcas_active && !f.ext.tcasRaCorrective ? static_cast<float>(f.ext.tcasVsTarget) : 0.0F;
      }
      float aboveFcuAlt = h - fcuAltitude(f);
      bool awayFromFcuAlt = aboveFcuAlt < 0.0F ? target <= 0.0F : (aboveFcuAlt > 0.0F && target >= 0.0F);
      bool inSrs = last.pitch_goaround_active || last.pitch_takeoff_active;

      bool oppArmed = f.ext.oppModeSync && oppFmgcBit(oppBus(f).discrete_word_3, kOppWord3AltArmed);
      bool ownArmed = ownCompute && !last.alt_acq_active && !last.alt_hold_active && !last.gs_capt_active &&
                      !last.gs_trk_active && !last.final_des_active && !f.last.lateral_modes.land_active &&
                      (!awayFromFcuAlt || (!last.vs_active && !last.fpa_active && !last.tcas_active)) &&
                      (!inSrs || aboveFcuAlt <= 0.0F) && f.ext.anyApFdEngaged && s[Signal::AltArmPossible];
      s[id] = oppArmed || ownArmed;
      return true;
    }

    case Signal::AltCapture: {
      // Capture: armed, no recent FCU altitude change, within the capture distance and moving towards the target.
      bool fcuChanged = alt_.fcuAltChangedRecently.update(fcuWord1Bit(f, kFcuWord1AltChanged), f.dt, true, true,
                                                          kFcuChangeCaptureInhibitS);
      float vs = valueOr(f.ext.logic->ir_computation_data.inertial_vertical_speed_ft_s, 0.0F);  // fpm
      double toTarget = s.altSelOrCstr - h;
      bool captureCond = f.last.armed_modes.alt_acq_armed && !fcuChanged &&
                         captureDistanceFt(vs) > std::abs(toTarget) && sign(toTarget) == sign(vs);

      float radioHeight = rawValue(f.ext.logic->ra_computation_data.radio_height_ft);
      bool srsAllows = radioHeight >= kSrsMinRadioHeightFt || (!last.pitch_goaround_active && !last.pitch_takeoff_active);

      const auto& opp = oppBus(f).discrete_word_1;
      bool oppCapture = f.ext.oppModeSync && oppFmgcBit(opp, kOppWord1AltCaptOrHold) &&
                        oppFmgcBit(opp, kOppWord1AltCaptOrGsCapt);
      bool set = oppCapture || (ownCompute && captureCond && srsAllows && tcasAllows(f, tcas_submode::ALT_ACQ));

      bool setRecently = alt_.captureCondRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = f.ext.modeReset || (anyOtherVerticalModeLast(f.last, Signal::AltCapture) && !setRecently);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::AltHold: {
      const auto& opp = oppBus(f).discrete_word_1;
      bool oppHold = f.ext.oppModeSync && oppFmgcBit(opp, kOppWord1AltCaptOrHold) && oppFmgcBit(opp, kOppWord1AltHoldOrGs);
      bool withinBand = alt_.withinAltHoldBand.update(std::abs(s.altSelOrCstr - h) < kAltHoldBandFt, f.dt, true,
                                                      kAltHoldConfirmS);
      bool set = oppHold ||
                 (ownCompute && last.alt_acq_active && withinBand && tcasAllows(f, tcas_submode::ALT_HOLD));

      bool setRecently = alt_.altHoldCondRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = f.ext.modeReset || (anyOtherVerticalModeLast(f.last, Signal::AltHold) && !setRecently);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::AltCruiseMemory: {
      bool atCruiseAlt = std::abs(f.in.fms_inputs.cruise_alt_ft - h) < kCruiseAltBandFt;
      s[id] = latch(atCruiseAlt, !last.alt_hold_active, s[id]);
      return true;
    }

    case Signal::AltCruise:
      // "ALT CRZ" on the FMA: holding ALT at the cruise altitude, not at a constraint.
      s[id] = s[Signal::AltHold] && !s[Signal::AltCstrApplicable] && s[Signal::AltCruiseMemory];
      return true;

    default:
      return false;
  }
}

}  // namespace fmgc::readable
