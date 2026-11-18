// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// G/S, VPATH (FINAL and DES on the FMS vertical profile) and FLARE laws, plus the flare law outputs.
// Generated: FmgcOuterLoops.cpp 3108–3186 (flare V/S law), 3474–3556 (G/S), 4070–4094 (VPATH), 4310–4319 (flare
// outputs, AP limits), 4560–4565 (flare condition); law selection at 4095 (FD) and 4484 (AP).
//
// Every law runs every step whichever law is active, so all filters below keep moving. Each law's pitch command is a
// change of flight path angle (delta theta, deg); the AP command is limited to a load-factor equivalent pitch change
// g/V·n (rad/s → deg).

#include <cstdint>

#include "laws.h"

namespace fmgc::readable::laws {

namespace {

// ---- units
constexpr double kKnToMs = 0.51444444444444448;
constexpr double kFpmToMs = 0.0050800000000000003;
constexpr double kFpmToFtS = 0.016666666666666666;  // ft/min -> ft/s
constexpr double kMToFt = 3.2808398950131235;
constexpr double kKnToFpm = 101.26859142607174;     // kt -> ft/min
constexpr double kRadToDeg = 57.295779513082323;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kG = 9.81;

// ---- G/S
// Track: beam deviation with a gain scheduled on radio height (smaller towards the runway as the beam narrows),
// plus damping by the inertial vertical acceleration; faded in when G/S TRK engages.
constexpr double kGsDeviationLagC1 = 10;                                     // rad/s
constexpr double kGsDeviationGainRaFt[8] = {0, 50, 100, 200, 400, 1000, 2500, 3000};
constexpr double kGsDeviationGain[8] = {0, 0, -0.14999999999999999, -0.40000000000000002, -0.77500000000000002,
                                        -1.6000000000000001, -3, -3};       // deg per deg of deviation
constexpr double kGsVsRateLagC1 = 10;                                         // rad/s, d(V/S)/dt
constexpr double kGsPitchRateAccelLagC1 = 1;                                  // rad/s, V·tan(q) (pitch-rate accel.)
constexpr double kGsAccelLagC1 = 4;                                           // rad/s
constexpr double kGsAccelWashoutC1 = 10;                                      // rad/s
constexpr double kGsAccelGain = -0.25;                                        // deg per ft/s²
constexpr double kGsAccelLimitDeg = 0.5;
constexpr double kGsTrackAccelWeight = 0.5;
constexpr double kGsTrackDeviationWeight = 1;
constexpr double kGsTrackFadeUp = 1;       // capture -> track over 1 s ...
constexpr double kGsTrackFadeDown = -1000; // ... back to capture at once
// Capture: lagged deviation plus 14 s of its rate (anticipates the beam), gain scheduled on radio height.
constexpr double kGsCaptureLagC1 = 2;      // rad/s
constexpr double kGsCaptureRateRaFt[5] = {0, 100, 200, 400, 1000};
constexpr double kGsCaptureRateGainS[5] = {14, 14, 14, 14, 14};
constexpr double kGsCaptureFilterC1 = 1;   // rad/s
constexpr double kGsCaptureGainRaFt[8] = {0, 50, 100, 200, 400, 1000, 2500, 3000};
constexpr double kGsCaptureGain[8] = {0, 0, -0.10000000000000001, -0.29999999999999999, -0.59999999999999998,
                                      -0.80000000000000004, -2, -2};
// Beam guidance is used above 60 ft RA with a valid G/S; in G/S TRK the flight path is also pulled to the beam angle.
constexpr double kGsMinRadioHeightFt = 60;
constexpr double kGsPathGain = 1;
constexpr double kGsBeamAngleMinDeg = -4;  // memorised beam angle limited to -4..-1 deg
constexpr double kGsBeamAngleMaxDeg = -1;
constexpr double kGsTrackFpaGain = 0.4;    // deg per deg of FPA vs beam angle (G/S TRK only)
constexpr double kGsFpaBandDeg = 2;        // flight path kept within ±2 deg of the beam angle ...
constexpr double kGsFpaBandGain = 0.34999999999999998;  // ... by a median with these two FPA commands
constexpr double kGsGndSpeedMinMs = 1;     // FPA from V/S and ground speed (1–1000 m/s)
constexpr double kGsGndSpeedMaxMs = 1000;
constexpr double kGsLawGainTasKn[7] = {0, 100, 150, 200, 250, 300, 400};
constexpr double kGsLawGain[7] = {2.5, 2.5, 2, 1, 1, 1, 1};  // overall gain on TAS (higher when slow)
constexpr double kGsFdGain = 2.8500000000000001;
constexpr double kGsApLimitG = 0.3;

// ---- VPATH: FMS profile V/S plus a correction from the lagged profile altitude error, as a flight path change
constexpr double kVpathAltErrorLagC1 = 1;  // rad/s
constexpr double kVpathAltErrorGain = 8;   // ft/min per ft
constexpr double kVpathMaxCorrectionFpm = 1500;
constexpr double kVpathTasMinMs = 1;
constexpr double kVpathTasMaxMs = 2000;
constexpr double kVpathApGain = 0.69999999999999996;
constexpr double kVpathApLimitG = 0.1;       // DES on path
constexpr double kVpathFinalApLimitG = 0.15; // FINAL APP

// ---- FLARE
// Radio height rate: 15 s/(s + 15) on RA, kept within a 3 % terrain slope of the slowly lagged V/S.
constexpr double kFlareVsLagC1 = 0.033333333333333333;  // rad/s (30 s)
constexpr double kFlareMaxTerrainSlope = 0.029999999999999999;
constexpr double kFlareRaRateC1 = 15, kFlareRaRateC2 = 0, kFlareRaRateC3 = 1, kFlareRaRateC4 = 15;
constexpr double kFtSToFpm = 60;
constexpr double kFlareRateCorrectionC1 = 1;  // fix: rad/s, lag on (radio height rate - V/S)
// V/S command: exponential flare H_dot = -(RA + bias)/tau, set at flare engagement so that it starts at the current
// radio height rate and reaches -2.5 ft/s at RA = 0.
constexpr double kFlareTouchdownVsFtS = 2.5;
constexpr double kFlareCmdRateC1 = 15, kFlareCmdRateC2 = 0, kFlareCmdRateC3 = 1, kFlareCmdRateC4 = 15;
constexpr double kFlareGndSpeedMinMs = 1;
constexpr double kFlareGndSpeedMaxMs = 2000;
constexpr double kFlareCmdRateWeight = 0;   // V/S command rate term: disabled (gain 0)
constexpr double kFlareVsErrorWeight = 1;
// Pitch terms
constexpr double kFlareBxWashoutC1 = 50;    // rad/s
constexpr double kFlareBzGain = -0.29999999999999999;  // deg per m/s²
constexpr double kFlareBxApGain = -5;
constexpr double kFlareBxFdGain = -50;
constexpr double kFlareVsApGain = 0.80000000000000004;
constexpr double kFlareFdGain = 4;
constexpr double kFlareBetaWashoutC1 = 6;   // rad/s
constexpr double kFlareBetaMaxDeg = 2;
constexpr double kFlareBetaGain = 0.29999999999999999;  // pitch up with |sideslip command changes| (decrab)
constexpr double kFlareWeightKg[4] = {0, 45000, 65000, 70000};
constexpr double kFlareWeightGain[4] = {1, 1, 1, 1};  // V/S term gain on weight (1 at all weights)
// On ground: pitch towards -2 deg (derotation)
constexpr double kDerotationThetaDeg = -2;
constexpr double kDerotationGain = -1.2;
constexpr double kDerotationFdGain = 2.5;
constexpr double kFlareApLimitG = 0.6;
// Flare condition: below 80 ft RA when RA·14 s ≤ |radio height rate| (14 s or less to touchdown), or at 42 ft
constexpr double kFlareCondMaxRaFt = 80;
constexpr double kFlareCondTimeToGoS = 14;
constexpr double kFlareCondMinRaFt = 42;
constexpr double kFlareCondRateLagC1 = 4;   // rad/s

/** Pitch change for a load factor increment n at TAS: g/V·n (rad/s), in deg. */
double loadFactorLimitDeg(double gOverV, double n) {
  return gOverV * n * kRadToDeg;
}

/** Saturation block: NaN passes through. */
double saturate(double x, double lo, double hi) {
  return x > hi ? hi : (x < lo ? lo : x);
}

/** 1-D lookup table, linear interpolation and extrapolation (generated look1_binlxpw). */
template <uint32_t N>
double lookup(double u, const double (&bp)[N], const double (&table)[N]) {
  constexpr uint32_t maxIndex = N - 1;
  uint32_t iLeft;
  double frac;
  if (u <= bp[0]) {
    iLeft = 0;
    frac = (u - bp[0]) / (bp[1] - bp[0]);
  } else if (u < bp[maxIndex]) {
    uint32_t bpIdx = maxIndex >> 1U;
    uint32_t iRight = maxIndex;
    iLeft = 0;
    while (iRight - iLeft > 1U) {
      if (u < bp[bpIdx]) {
        iRight = bpIdx;
      } else {
        iLeft = bpIdx;
      }
      bpIdx = (iRight + iLeft) >> 1U;
    }
    frac = (u - bp[iLeft]) / (bp[iLeft + 1] - bp[iLeft]);
  } else {
    iLeft = maxIndex - 1;
    frac = (u - bp[maxIndex - 1]) / (bp[maxIndex] - bp[maxIndex - 1]);
  }
  return (table[iLeft + 1] - table[iLeft]) * frac + table[iLeft];
}

}  // namespace

void VerticalLaws::resetPath() {
  // The generated reset() clears every one of these blocks (filters re-prime, delays to 0, Tau 1, H_bias 0).
  path_ = PathLawState{};
}

void VerticalLaws::computePath(const ap_laws_output& in, LawResults& out) {
  const double dt = in.time.dt;
  const auto& d = in.data;
  const double gOverV = kG / (d.V_tas_kn * kKnToMs);

  // ================================================================================================================
  // FLARE V/S law (3108–3186)
  // ================================================================================================================
  FlareLawState& fl = path_.flare;
  const double derotation = (d.Theta_deg - kDerotationThetaDeg) * kDerotationGain;
  const double bxWashed = fl.bxWashout.update(d.bx_m_s2, kFlareBxWashoutC1, dt);
  out.flare.delta_Theta_bz_deg = kFlareBzGain * d.bz_m_s2;
  const double weightGain = lookup(d.total_weight_kg, kFlareWeightKg, kFlareWeightGain);
  const bool flareActive = in.input.vertical_law == Flare;

  // radio height rate (ft/min), limited around the lagged V/S
  const double vsLagged = fl.vsLag.update(d.H_dot_ft_min, kFlareVsLagC1, dt);
  double slopeFpm = kKnToFpm * d.V_gnd_kn * kFlareMaxTerrainSlope;
  const double raRate =
      fl.radioRate.update(d.H_radio_ft, kFlareRaRateC1, kFlareRaRateC2, kFlareRaRateC3, kFlareRaRateC4, dt);
  // Fix (= personal/fg-flare-rate, findings F24): the inertial V/S plus the 1 s lagged difference to the radio height
  // derivative. The sim's radio altimeters refresh every 80 ms, so the derivative alone swings by ±400 fpm; stock used
  // it directly, taking the flare time constant from one noisy sample.
  const double rateCorrection = fl.rateCorrectionLag.update(kFtSToFpm * raRate - d.H_dot_ft_min, kFlareRateCorrectionC1, dt);
  const double hDotRadio =
      std::fmin(std::fmax(vsLagged - slopeFpm, d.H_dot_ft_min + rateCorrection), slopeFpm + vsLagged);
  out.flare.H_dot_radio_fpm = hDotRadio;

  // V/S command: exponential flare, its time constant and bias set on the step the law becomes active
  if (!fl.wasActivePrimed) {
    fl.wasActive = flareActive;
    fl.wasActivePrimed = true;
  }
  if (!fl.wasActive && flareActive) {
    double sinkFtS = std::abs(hDotRadio) / 60.0;
    fl.tau = d.H_radio_ft / (sinkFtS - kFlareTouchdownVsFtS);
    fl.heightBias = fl.tau * sinkFtS - d.H_radio_ft;
  }
  const double hDotCmd = flareActive ? -1.0 / fl.tau * (d.H_radio_ft + fl.heightBias) * 60.0 : hDotRadio;
  out.flare.H_dot_c_fpm = hDotCmd;
  fl.wasActive = flareActive;

  // V/S error (and the disabled V/S command rate term) as a flight path change
  const double cmdRate =
      fl.vsCommandRate.update(hDotCmd, kFlareCmdRateC1, kFlareCmdRateC2, kFlareCmdRateC3, kFlareCmdRateC4, dt);
  const double vGndMs = kKnToMs * d.V_gnd_kn;
  double sinCmdRate = kFpmToMs * cmdRate / saturate(vGndMs, kFlareGndSpeedMinMs, kFlareGndSpeedMaxMs);
  double sinVsError = (hDotCmd - hDotRadio) * kFpmToMs / saturate(vGndMs, kFlareGndSpeedMinMs, kFlareGndSpeedMaxMs);
  sinCmdRate = saturate(sinCmdRate, -1.0, 1.0);
  sinVsError = saturate(sinVsError, -1.0, 1.0);
  const double flareGammaDeg = kRadToDeg * std::asin(sinCmdRate) * kFlareCmdRateWeight +
                               kRadToDeg * std::asin(sinVsError) * kFlareVsErrorWeight;

  // ================================================================================================================
  // G/S (3474–3556)
  // ================================================================================================================
  GsLawState& gs = path_.gs;
  const double ra = d.H_radio_ft;
  // track: deviation term
  const double deviation = gs.deviationLag.update(d.nav_gs_error_deg, kGsDeviationLagC1, dt);
  const double deviationTerm = deviation * lookup(ra, kGsDeviationGainRaFt, kGsDeviationGain);
  // track: vertical acceleration (d(V/S)/dt minus the part due to pitch rate, V·q), washed out
  const double vsFtS = kFpmToFtS * d.H_dot_ft_min;
  const double vsRate = gs.vsRateLag.update((vsFtS - gs.prevVsFtS) / dt, kGsVsRateLagC1, dt);
  gs.prevVsFtS = vsFtS;  // (generated: stored at the end of step())
  const double pitchRateAccel = gs.pitchRateAccelLag.update(
      kMToFt * (std::tan(kDegToRad * d.qk_deg_s) * (kKnToMs * d.V_gnd_kn)), kGsPitchRateAccelLagC1, dt);
  const double accel = gs.accelLag.update(vsRate - pitchRateAccel, kGsAccelLagC1, dt);
  double accelTerm = kGsAccelGain * gs.accelWashout.update(accel, kGsAccelWashoutC1, dt);
  const double track =
      saturate(gs.trackBlend.update(in.input.GS_track_mode, kGsTrackFadeUp, kGsTrackFadeDown, dt, 0.0), 0.0, 1.0);
  // capture: lagged deviation plus its rate
  const double captureDeviation = gs.captureDeviationLag.update(d.nav_gs_error_deg, kGsCaptureLagC1, dt);
  const double prevCaptureDeviation = gs.prevCaptureDeviation;
  gs.prevCaptureDeviation = captureDeviation;
  const double capture = gs.captureLag.update(
      captureDeviation + (captureDeviation - prevCaptureDeviation) / dt *
                             lookup(ra, kGsCaptureRateRaFt, kGsCaptureRateGainS),
      kGsCaptureFilterC1, dt);
  accelTerm = saturate(accelTerm, -kGsAccelLimitDeg, kGsAccelLimitDeg);
  const double beamCmd = (kGsTrackAccelWeight * accelTerm + kGsTrackDeviationWeight * deviationTerm) * track +
                         (1.0 - track) * (capture * lookup(ra, kGsCaptureGainRaFt, kGsCaptureGain));
  // beam angle, memorised when G/S becomes the active law
  if (in.input.vertical_law != Gs || !gs.beamAnglePrimed) {
    gs.beamAngleDeg = d.nav_gs_deg;
    gs.beamAnglePrimed = true;
  }
  const double beamAngle = saturate(gs.beamAngleDeg, kGsBeamAngleMinDeg, kGsBeamAngleMaxDeg);
  const double fpa =
      std::atan(kFpmToMs * d.H_dot_ft_min / saturate(kKnToMs * d.V_gnd_kn, kGsGndSpeedMinMs, kGsGndSpeedMaxMs)) *
      kRadToDeg;
  const double pathTerm = ra > kGsMinRadioHeightFt && d.nav_gs_valid ? kGsPathGain * beamCmd : 0.0;
  const double trackFpaTerm = in.input.GS_track_mode ? (beamAngle - fpa) * kGsTrackFpaGain : 0.0;
  double gsCmd = blocks::median3(pathTerm + trackFpaTerm, kGsFpaBandGain * ((beamAngle + kGsFpaBandDeg) - fpa),
                                 kGsFpaBandGain * ((beamAngle - kGsFpaBandDeg) - fpa));
  gsCmd *= lookup(d.V_tas_kn, kGsLawGainTasKn, kGsLawGain);

  // ================================================================================================================
  // VPATH (4070–4094)
  // ================================================================================================================
  const double altError =
      path_.vpath.altitudeErrorLag.update(d.fms_H_c_profile_ft - d.H_ft, kVpathAltErrorLagC1, dt);
  const double vsCorrection = saturate(kVpathAltErrorGain * altError, -kVpathMaxCorrectionFpm, kVpathMaxCorrectionFpm);
  const double tasMs = saturate(kKnToMs * d.V_tas_kn, kVpathTasMinMs, kVpathTasMaxMs);
  const double sinGamma =
      saturate(((d.fms_H_dot_c_profile_ft_min + vsCorrection) - d.H_dot_ft_min) * kFpmToMs / tasMs, -1.0, 1.0);
  const double vpathCmd = kRadToDeg * std::asin(sinGamma);

  // ================================================================================================================
  // Flare law outputs (4310–4317) and FLARE commands
  // ================================================================================================================
  const double betaWashed = fl.betaWashout.update(in.output.autopilot.Beta_c_deg, kFlareBetaWashoutC1, dt);
  out.flare.delta_Theta_beta_c_deg = kFlareBetaGain * saturate(std::abs(betaWashed), -kFlareBetaMaxDeg, kFlareBetaMaxDeg);
  out.flare.delta_Theta_H_dot_deg = kFlareVsApGain * flareGammaDeg;
  out.flare.delta_Theta_bx_deg = kFlareBxApGain * bxWashed;

  // ================================================================================================================
  // Flare condition (4560–4565)
  // ================================================================================================================
  const double raRateLagged = fl.conditionRateLag.update(raRate, kFlareCondRateLagC1, dt);
  slopeFpm = kKnToFpm * d.V_gnd_kn * kFlareMaxTerrainSlope;
  out.flare.condition_Flare =
      flareActive ||
      (ra < kFlareCondMaxRaFt &&
       (ra * kFlareCondTimeToGoS <=
            std::abs(std::fmin(std::fmax(vsLagged - slopeFpm, kFtSToFpm * raRateLagged), slopeFpm + vsLagged)) ||
        ra <= kFlareCondMinRaFt));
  out.flareValid = true;

  // ================================================================================================================
  // Commands as the FD (4095) and AP (4484) law switches select them
  // ================================================================================================================
  const double gsApLimit = loadFactorLimitDeg(gOverV, kGsApLimitG);
  out.cmd[Gs].fd = kGsFdGain * gsCmd;
  out.cmd[Gs].ap = std::fmax(-gsApLimit, std::fmin(gsApLimit, gsCmd));

  const double vpathApLimit =
      loadFactorLimitDeg(gOverV, in.input.FINAL_DES_mode_active ? kVpathFinalApLimitG : kVpathApLimitG);
  out.cmd[Vpath].fd = vpathCmd;
  out.cmd[Vpath].ap = std::fmax(-vpathApLimit, std::fmin(vpathApLimit, kVpathApGain * vpathCmd));

  const double flareApLimit = loadFactorLimitDeg(gOverV, kFlareApLimitG);
  const auto& f = out.flare;
  out.cmd[Flare].fd = d.on_ground ? kDerotationFdGain * derotation
                                  : ((kFlareBxFdGain * bxWashed + f.delta_Theta_bz_deg) + flareGammaDeg * weightGain) *
                                        kFlareFdGain;
  const double flareAp = d.on_ground ? derotation
                                     : ((f.delta_Theta_bz_deg + f.delta_Theta_bx_deg) +
                                        weightGain * f.delta_Theta_H_dot_deg) +
                                           f.delta_Theta_beta_c_deg;
  out.cmd[Flare].ap = std::fmax(-flareApLimit, std::fmin(flareApLimit, flareAp));
}

}  // namespace fmgc::readable::laws
