// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ALT_ACQ (ALT*) and ALT_HOLD laws. Generated: FmgcOuterLoops.cpp 2754–3067 (ALT_ACQ: capture law, VLS/VMAX speed
// protection) and 3069–3107 (ALT_HOLD); the ALT_HOLD AP command is limited at 4304. Both run every frame whatever the
// active law. Branch personal/fmgc-readable-fixes: stock except the ALT* entry gain fix (= personal/fmgc-fixes).

#include "laws.h"

namespace fmgc::readable::laws {

namespace {

constexpr double kKnotsToMs = 0.51444444444444448;
constexpr double kKnotsToMsRounded = 0.5144;  // used for the speed-derivative inputs of the speed-on-pitch law
constexpr double kFpmToMs = 0.0050800000000000003;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.295779513082323;
constexpr double kG = 9.81;
constexpr double kOneOverG = 0.1019367991845056;

// V/S command -> flight path angle change: asin((Hdot_c - Hdot) / V_tas), with V_tas limited to 1..2000 m/s
constexpr double kMinTasMs = 1;
constexpr double kMaxTasMs = 2000;

// Speed-on-pitch law (VLS / VMAX protection)
constexpr double kMinGsMs = 1;                       // ground speed limits for the flight path angle
constexpr double kMaxGsMs = 1000;
constexpr double kGsToCas = 0.90350790290525129;     // ground speed derivative scaled to a CAS derivative
constexpr double kGroundAccelGain = 2;               // washout gain ...
constexpr double kGroundAccelC1 = 2;                 // ... and corner (rad/s): derivative filtered with 0.5 s
constexpr double kInertialHpC1 = 3, kInertialHpC2 = 0, kInertialHpC3 = 3, kInertialHpC4 = 1;  // 3s/(3s+1)
constexpr double kIasRateC1 = 0.33333333333333331, kIasRateC2 = 0, kIasRateC3 = 1,
                 kIasRateC4 = 0.33333333333333331;  // (s/3)/(s+1/3): IAS derivative lagged by 3 s
constexpr double kSpeedErrorGain = 0.33333333333333331;  // deg per kt of speed error ...
constexpr double kSpeedErrorLimitDeg = 4;                // ... limited to ±4°

// ALT* capture law
constexpr double kCaptureArrivalVsBase = 500;     // dH_offset = |500/|k| - 100| ft
constexpr double kCaptureArrivalVsSlope = 100;
constexpr double kCaptureMinEntryErrorFt = 1;     // fix: entry altitude error kept at least this far from 0
constexpr double kCaptureMaxOffsetFt = 10000;     // fix: bound on dH_offset
constexpr double kAltAcqApGain = 0.35;            // AP gain on the capture law's FPA change
constexpr double kSpeedProtectionApGain = 0.36;   // AP gain on the VLS / VMAX commands
constexpr double kSpeedProtectionMarginKn = 5;    // protection active within 5 kt of VLS / VMAX

// ALT hold law
constexpr double kHoldErrorLagC1 = 1;      // rad/s
constexpr double kHoldGainFpmPerFt = 8;    // V/S command per ft of altitude error
constexpr double kHoldMaxVsFpm = 1500;
constexpr double kSoftModeGainFpmPerKt = -100;  // ALT soft mode: V/S command per kt above the target speed ...
constexpr double kSoftModeMaxVsFpm = 400;        // ... limited to ±400 fpm
constexpr double kAltHoldApGain = 0.7;

/** Simulink Saturation block: NaN passes through. */
double saturate(double u, double lo, double hi) {
  if (u > hi) {
    return hi;
  }
  if (u < lo) {
    return lo;
  }
  return u;
}

/** MATLAB sign(): -1, 0 or 1 (0 for NaN). */
double sign(double u) {
  return u < 0.0 ? -1.0 : (u > 0.0 ? 1.0 : 0.0);
}

/** Flight path angle change (deg) that turns the current V/S into hDotCmdFpm at the current TAS. */
double vsToFpaChangeDeg(double hDotCmdFpm, const ap_laws_output& in) {
  double tasMs = saturate(kKnotsToMs * in.data.V_tas_kn, kMinTasMs, kMaxTasMs);
  double sinDGamma = saturate((hDotCmdFpm - in.data.H_dot_ft_min) * kFpmToMs / tasMs, -1.0, 1.0);
  return kRadToDeg * std::asin(sinDGamma);
}

}  // namespace

/**
 * Speed-on-pitch law: pitch change (deg) = speed rate expressed as an angle (in g, i.e. the energy angle minus the
 * flight path angle) plus the speed error to the reference speed, 1/3 deg per kt limited to ±4°. The speed rate is
 * complementary: the ground speed derivative (minus a turn correction) high passed, plus the IAS derivative low passed.
 */
double SpeedOnPitchLaw::update(const ap_laws_output& in, double speedRefKn) {
  const double dt = in.time.dt;
  const double vGnd = in.data.V_gnd_kn;
  const double hDot = in.data.H_dot_ft_min;
  double gsMs = saturate(kKnotsToMs * vGnd, kMinGsMs, kMaxGsMs);
  double gammaDeg = std::atan(kFpmToMs * hDot / gsMs) * kRadToDeg;
  double phiRad = kDegToRad * in.data.Phi_deg;
  double accel = groundAccel.update(kGroundAccelGain * (kGsToCas * (kKnotsToMsRounded * vGnd)), kGroundAccelC1, dt);
  double gsMs2 = saturate(kKnotsToMs * vGnd, kMinGsMs, kMaxGsMs);  // a second, identical saturation block
  double thetaMinusGammaRad =
      kDegToRad * in.data.Theta_deg - kDegToRad * (kRadToDeg * std::atan(kFpmToMs * hDot / gsMs2));
  double turnCorrectionRad = thetaMinusGammaRad * (1.0 - std::cos(phiRad)) +
                             std::sin(phiRad) * std::sin(kDegToRad * in.data.Psi_magnetic_track_deg -
                                                         kDegToRad * in.data.Psi_magnetic_deg);
  double inertial = inertialHighPass.update(accel - kG * (kDegToRad * (kRadToDeg * turnCorrectionRad)), kInertialHpC1,
                                            kInertialHpC2, kInertialHpC3, kInertialHpC4, dt);
  double iasDot =
      iasRate.update(kKnotsToMsRounded * in.data.V_ias_kn, kIasRateC1, kIasRateC2, kIasRateC3, kIasRateC4, dt);
  double speedRateG = (inertial + iasDot) * kOneOverG;
  double gammaRad = kDegToRad * gammaDeg;
  double energyAngleRad = speedRateG + gammaRad;

  // The generated limits are switches between ±4° and 0.6 / -0.93 × (0.7·speedRateG + gamma) in degrees, on a constant
  // condition (1 - 0 > 0) that always selects ±4°.
  double speedErrorDeg = (in.data.V_ias_kn - speedRefKn) * kSpeedErrorGain;
  double correction;
  if (speedErrorDeg <= kSpeedErrorLimitDeg) {
    correction = speedErrorDeg >= -kSpeedErrorLimitDeg ? speedErrorDeg : -kSpeedErrorLimitDeg;
  } else {
    correction = kSpeedErrorLimitDeg;  // also NaN
  }
  return (kRadToDeg * energyAngleRad - gammaDeg) + correction;
}

void VerticalLaws::resetAlt() {
  alt_.captureWasActivePrimed = false;
  alt_.captureOffsetFt = 0.0;
  alt_.captureGain = 5.0;
  alt_.captureMaxHdot = 1500.0;
  alt_.altAcqVls.reset();
  alt_.altAcqVmax.reset();
  alt_.holdErrorLag.reset();
  // holdTargetH is not reset (generated: set in init() only)
}

void VerticalLaws::computeAlt(const ap_laws_output& in, LawResults& out) {
  const double law = in.input.vertical_law;
  const double hDot = in.data.H_dot_ft_min;

  // ---- ALT_ACQ: capture law. V/S command k·(dH ± offset), limited to the entry V/S; k and the offset are chosen at
  // law entry so the command starts at the current V/S.
  bool captureActive = law == AltAcq;
  if (!alt_.captureWasActivePrimed) {
    alt_.captureWasActive = captureActive;
    alt_.captureWasActivePrimed = true;
  }
  double altError = in.input.H_c_ft - in.data.H_ind_ft;
  double r = sign(altError) * alt_.captureOffsetFt + altError;
  if (!alt_.captureWasActive && captureActive) {
    // Fix (stock computes the entry gain from r, which still holds the previous capture's offset; an entry at ~0 V/S,
    // e.g. ALT* on the ground, made that offset huge and every later capture crawled at ~400 fpm, see
    // docs/laws/alt-capture.md): gain from the altitude error alone, kept away from 0, offset bounded. Same as stock on
    // a flight's first capture.
    double dhEntry = altError;
    if (std::abs(dhEntry) < kCaptureMinEntryErrorFt) {
      dhEntry = hDot < 0.0 ? -kCaptureMinEntryErrorFt : kCaptureMinEntryErrorFt;
    }
    alt_.captureGain = hDot / dhEntry;
    alt_.captureOffsetFt = std::fmin(
        std::abs(kCaptureArrivalVsBase / std::abs(alt_.captureGain) - kCaptureArrivalVsSlope), kCaptureMaxOffsetFt);
    r = dhEntry + (dhEntry < 0.0 ? -1.0 : 1.0) * alt_.captureOffsetFt;
    alt_.captureGain = hDot / r;
    alt_.captureMaxHdot = std::abs(hDot);
  }
  double captureVsCmd = r * alt_.captureGain;
  if (std::abs(captureVsCmd) > alt_.captureMaxHdot) {
    captureVsCmd = sign(captureVsCmd) * alt_.captureMaxHdot;
  }
  alt_.captureWasActive = captureActive;
  double captureFpaChange = vsToFpaChangeDeg(captureVsCmd, in);

  // speed protection: below VLS / above VMAX the speed-on-pitch commands take over
  double vlsCmd = alt_.altAcqVls.update(in, blocks::vlsWithMargin(in.input.V_c_kn, in.data.VLS_kn));
  double vmaxCmd = alt_.altAcqVmax.update(in, in.data.VMAX_kn);
  blocks::ProtectedCommand acq = blocks::speedProtection(
      in.input.V_c_kn, in.data.VLS_kn, in.data.V_ias_kn, in.data.VMAX_kn, captureFpaChange,
      kAltAcqApGain * captureFpaChange, vlsCmd, kSpeedProtectionApGain * vlsCmd, vmaxCmd,
      kSpeedProtectionApGain * vmaxCmd, kSpeedProtectionMarginKn);
  out.cmd[AltAcq] = {acq.fd, acq.ap};

  // ---- ALT_HOLD: V/S command proportional to the lagged altitude error to a target frozen while the law is active
  // (tracks H_c in the H_ft frame otherwise), plus the ALT soft mode speed term.
  if (law != AltHold) {
    alt_.holdTargetH = (in.input.H_c_ft + in.data.H_ft) - in.data.H_ind_ft;
  }
  double holdError = alt_.holdErrorLag.update(alt_.holdTargetH - in.data.H_ft, kHoldErrorLagC1, in.time.dt);
  double softModeVs = 0.0;
  if (in.input.ALT_soft_mode_active) {
    softModeVs = saturate((in.input.V_c_kn - in.data.V_ias_kn) * kSoftModeGainFpmPerKt, -kSoftModeMaxVsFpm,
                          kSoftModeMaxVsFpm);
  }
  double holdVsCmd = saturate(kHoldGainFpmPerFt * holdError + softModeVs, -kHoldMaxVsFpm, kHoldMaxVsFpm);
  double holdFpaChange = vsToFpaChangeDeg(holdVsCmd, in);
  out.cmd[AltHold] = {holdFpaChange, blocks::vsLimit(kAltHoldApGain * holdFpaChange, in.data.V_tas_kn)};
}

}  // namespace fmgc::readable::laws
