// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Autothrust N1 command. Generated: FmgcComputer.cpp 4425-4640 on master (speed error 4425-4451, acceleration estimate
// 4452-4526, rate/derivative 4527-4544, load factor 4545-4569, ALT soft fade 4571-4579, rate selection 4580-4616,
// integrator 4617-4633, alpha floor 4635-4640; output 4956, end-of-step state 5126-5128).
//
// The FMGC does not compute an N1 target directly: every mode commands an N1 *rate* (%/s), which an integrator turns
// into the N1 command sent to the FADECs:
//
//   n1_c = limit[18 %, N1 ref](n1_c(previous) + rate * dt)
//
// The integrator is loaded with the actual N1 (the higher engine) only while A/THR is not engaged, not active, or in
// alpha floor; once A/THR is active it is never re-loaded, whatever the mode changes (see docs/athr-n1.md, "windup").
//
// Mode rates, in priority order:
//   tracking mode override (sim): 0 (hold)
//   RETARD:     -10 %/s
//   SPEED/MACH: predicted speed error and load factor feedback, scheduled on N1 and faded down in ALT soft mode, ±5 %/s
//   THR:        ±(10 %/s below 20 % actual N1, 1 %/s above 30 %), towards idle when thrust_target_idle, ±10 %/s
//   otherwise:  0
// Alpha floor overrides the output (not the integrator) with the N1 maximum (TOGA).

#include <cmath>
#include <cstdint>

#include "athr.h"

namespace fmgc::readable::athr {

namespace {

namespace a429 = blocks::a429;

constexpr double kKnToMs = 0.51444444444444448;
constexpr double kKnToMsAirspeed = 0.5144;   // used for the speed rates (slightly rounded)
constexpr double kMsToKn = 1.9440124416796269;
constexpr double kFpmToMs = 0.0050800000000000003;
constexpr double kRadToDeg = 57.295779513082323;
constexpr double kDegToRad = 0.017453292519943295;
constexpr float kDegToRadF = 0.0174532924F;  // float variant for the load factor term
constexpr double kG = 9.81;                  // m/s²

// ---------------------------------------------------------------------------------------------------------------------
// SPEED/MACH controller
// ---------------------------------------------------------------------------------------------------------------------

// Flight path angle: ground speed limited to [1, 1000] m/s.
constexpr double kMinGroundSpeedMs = 1, kMaxGroundSpeedMs = 1000;
// Speeds used for the rate estimates are limited to [60, 480] kt.
constexpr double kMinRateSpeedKn = 60, kMaxRateSpeedKn = 480;

// Ground speed rate: 2·V through the washout s/(s+2) = dV/dt with a 0.5 s lag.
constexpr double kGroundSpeedRateGain = 2;
constexpr double kGroundSpeedRateC1 = 2;  // rad/s
// Attitude-corrected inertial acceleration, high-passed 0.2s/(0.2s+1) (only its fast part is kept).
constexpr double kInertialHighPassC1 = 0.20000000000000001, kInertialHighPassC2 = 0;
constexpr double kInertialHighPassC3 = 0.20000000000000001, kInertialHighPassC4 = 1;
// IAS through 5s/(5s+1): 5 s times the IAS rate lagged 5 s (the "speed trend" part of the prediction).
constexpr double kAirspeedWashoutC1 = 5, kAirspeedWashoutC2 = 0;
constexpr double kAirspeedWashoutC3 = 5, kAirspeedWashoutC4 = 1;
// The trend (kt) is subtracted from the speed error: predicted error = error - trend x gain(error).
constexpr double kTrendSign = -1;
// Trend gain scheduled on the speed error (target - IAS, kt): 1.8 when 20 kt or more too fast, 1 on target, 1.2 when
// 10 kt or more too slow.
constexpr float kTrendGainErrorKn[5] = {-100, -20, 0, 10, 100};
constexpr float kTrendGain[5] = {1.8F, 1.8F, 1, 1.2F, 1.2F};

// Rate of the predicted speed error: discrete derivative, lagged 0.1 s.
constexpr double kDerivativeGain = 1;
constexpr double kErrorRateLagC1 = 10;  // rad/s

// Load factor feedback: lagged normal acceleration minus the 1 g of level flight at this attitude (cos θ / cos φ).
constexpr double kNormalAccelLagC1 = 3;     // rad/s
// N1 rate += 10 %/s per g of load factor increment, plus 3 x (10 x the increment) washed out with a 0.1 s time constant
// (an extra 30 %/s per g on load factor changes).
constexpr float kLoadFactorWashoutInputGain = 10;
constexpr double kLoadFactorWashoutC1 = 10;  // rad/s
constexpr float kLoadFactorDirectGain = 10;  // %/s per g
constexpr float kLoadFactorWashoutGain = 3;

// N1 rate = 0.35 %/s per kt of predicted error + the error rate (%/s per kt/s) + load factor terms.
constexpr double kSpeedErrorGain = 0.34999999999999998;

// Gain scheduled on the N1 command (%): x5 at low N1 (below 20 %), x1 above 45 %.
constexpr double kN1GainN1Percent[4] = {0, 20, 45, 100};
constexpr double kN1Gain[4] = {5, 5, 1, 1};
// ALT soft mode: the rate gain fades from 1 to 0.01 (thrust nearly frozen) as the blend goes 0 -> 1.
constexpr double kAltSoftBlendUp = 0.10000000000000001;  // /s: 10 s to fade in
constexpr double kAltSoftBlendDown = -2;                 // /s: 0.5 s to fade out
constexpr double kAltSoftBlendInit = 0;
constexpr double kAltSoftBlend[2] = {0, 1};
constexpr double kAltSoftGain[2] = {1, 0.01};

constexpr double kSpeedMaxRatePercentS = 5;

// ---------------------------------------------------------------------------------------------------------------------
// THR, RETARD, integrator
// ---------------------------------------------------------------------------------------------------------------------

// THR mode ramp: |rate| from the lower actual N1 of the two engines: 10 %/s up to 20 % N1, 1 %/s from 30 % N1.
constexpr float kThrRampActualN1Percent[7] = {0, 20, 30, 45, 60, 80, 100};
constexpr float kThrRampRatePercentS[7] = {10, 10, 1, 1, 1, 1, 1};
constexpr double kThrTowardsIdle = -1, kThrTowardsLimit = 1;
constexpr double kThrMaxRatePercentS = 10;

constexpr double kRetardRatePercentS = -10;
constexpr double kHoldRatePercentS = 0;  // tracking mode override, and no A/THR mode

constexpr double kIntegratorGain = 1;
constexpr double kMinN1CommandPercent = 18;

/** Saturation block: hi if x > hi, else lo if x < lo, else x (NaN passes). */
double saturate(double x, double lo, double hi) {
  return x > hi ? hi : (x < lo ? lo : x);
}

/** 1-D lookup table, linear interpolation and extrapolation (generated look1_binlxpw / look1_iflf_binlxpw). */
template <typename T, uint32_t N>
T lookup(T u, const T (&bp)[N], const T (&table)[N]) {
  constexpr uint32_t maxIndex = N - 1;
  uint32_t iLeft;
  T frac;
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

void Autothrust::updateN1(const Inputs& in, Result& r) {
  N1State& s = n1_;
  const double dt = in.dt;
  const auto& logic = in.logic;
  const auto& ir = logic.ir_computation_data;
  const auto& fadecOwn = in.data.bus_inputs.fadec_own_bus;
  const auto& fadecOpp = in.data.bus_inputs.fadec_opp_bus;
  const base_fmgc_athr_outputs& mode = r.out;  // this frame's mode outputs

  // ==================================================================================================================
  // SPEED/MACH controller (computed every frame, used in SPEED/MACH mode)
  // ==================================================================================================================

  // Speed error to the protected target median(VLS, target, VMAX) (float, kt). Positive: too slow.
  const float iasKn = a429::valueOr(logic.adr_computation_data.airspeed_computed_kn, 0);
  const float vlsKn = a429::valueOr(logic.chosen_fac_bus.v_ls_kn, 0);
  const float vmaxKn = a429::valueOr(logic.chosen_fac_bus.v_max_kn, 0);
  const float targetKn =
      static_cast<float>(blocks::median3(vlsKn, static_cast<float>(in.apFdLogic.spd_target_kts), vmaxKn));
  const float speedErrorKn = targetKn - iasKn;

  // Inertial acceleration: ground speed rate minus the gravity component of the attitude correction (pitch above the
  // flight path in a bank, and drift: track - heading).
  const double thetaRad = kDegToRad * static_cast<double>(a429::valueOr(ir.pitch_angle_deg, 0));
  const double vsFpm = a429::valueOr(ir.inertial_vertical_speed_ft_s, 0);
  const double groundSpeedKn = a429::valueOr(ir.ground_speed_kn, 0);
  const double vGndMs = saturate(kKnToMs * groundSpeedKn, kMinGroundSpeedMs, kMaxGroundSpeedMs);
  const double gammaRad = std::atan(kFpmToMs * vsFpm / vGndMs) * kRadToDeg * kDegToRad;
  const double phiRad = static_cast<double>(a429::valueOr(ir.roll_angle_deg, 0)) * kDegToRad;
  const double cosPhi = std::cos(phiRad);
  const double sinPhi = std::sin(phiRad);
  const double headingRad = kDegToRad * static_cast<double>(a429::valueOr(ir.heading_magnetic_deg, 0));
  const double trackDeg = a429::valueOr(ir.track_angle_magnetic_deg, 0);
  const double groundSpeedRate = s.groundSpeedRate.update(
      kKnToMsAirspeed * saturate(groundSpeedKn, kMinRateSpeedKn, kMaxRateSpeedKn) * kGroundSpeedRateGain,
      kGroundSpeedRateC1, dt);
  const double attitudeCorrectionRad =
      (thetaRad - gammaRad) * (1.0 - cosPhi) + sinPhi * std::sin(kDegToRad * trackDeg - headingRad);
  const double inertialAccel = s.inertialAccelHighPass.update(
      groundSpeedRate - kG * (kDegToRad * (kRadToDeg * attitudeCorrectionRad)), kInertialHighPassC1,
      kInertialHighPassC2, kInertialHighPassC3, kInertialHighPassC4, dt);
  const double iasTrend = s.airspeedWashout.update(
      kKnToMsAirspeed * saturate(a429::valueOr(logic.adr_computation_data.airspeed_computed_kn, 0), kMinRateSpeedKn,
                                 kMaxRateSpeedKn),
      kAirspeedWashoutC1, kAirspeedWashoutC2, kAirspeedWashoutC3, kAirspeedWashoutC4, dt);

  // Predicted speed error (kt): the error minus the speed trend, the trend weighted more when well below the target.
  const double speedTrendKn = (inertialAccel + iasTrend) * kMsToKn * kTrendSign;
  const double predictedErrorKn =
      speedTrendKn * lookup(speedErrorKn, kTrendGainErrorKn, kTrendGain) + speedErrorKn;

  // Its rate (kt/s), lagged.
  const double previousPredictedErrorKn = s.lastPredictedSpeedErrorKn;
  s.lastPredictedSpeedErrorKn = kDerivativeGain * predictedErrorKn;
  const double errorRate = (s.lastPredictedSpeedErrorKn - previousPredictedErrorKn) / dt;
  const double errorRateLagged = s.errorRateLag.update(errorRate, kErrorRateLagC1, dt);

  // Load factor increment (g, float): lagged normal acceleration minus the level-flight 1 g at this attitude.
  const float nzLagged =
      s.normalAccelLag.update(a429::valueOr(ir.body_normal_accel_g, 0), kNormalAccelLagC1, dt);
  const float levelNz = std::cos(kDegToRadF * a429::valueOr(ir.pitch_angle_deg, 0)) /
                        std::cos(kDegToRadF * a429::valueOr(ir.roll_angle_deg, 0));
  const float deltaNz = nzLagged - levelNz;
  const float deltaNzWashedOut = s.loadFactorWashout.update(kLoadFactorWashoutInputGain * deltaNz, kLoadFactorWashoutC1, dt);

  // ALT soft mode fade and the N1 command gain schedule (previous frame's integrator output).
  const double previousN1Percent = s.n1CommandPercent;
  const double altSoftBlend =
      s.altSoftBlend.update(in.apFdLogic.alt_soft_mode_active, kAltSoftBlendUp, kAltSoftBlendDown, dt, kAltSoftBlendInit);

  // ==================================================================================================================
  // N1 rate by mode (%/s)
  // ==================================================================================================================
  double ratePercentS;
  if (in.data.sim_data.tracking_mode_on_override) {
    ratePercentS = kHoldRatePercentS;
  } else if (mode.retard_mode_active) {
    ratePercentS = kRetardRatePercentS;
  } else if (mode.speed_mach_mode_active) {
    ratePercentS = ((kSpeedErrorGain * predictedErrorKn + errorRateLagged) +
                    (kLoadFactorDirectGain * deltaNz + kLoadFactorWashoutGain * deltaNzWashedOut)) *
                   lookup(previousN1Percent, kN1GainN1Percent, kN1Gain) *
                   lookup(altSoftBlend, kAltSoftBlend, kAltSoftGain);
    ratePercentS = saturate(ratePercentS, -kSpeedMaxRatePercentS, kSpeedMaxRatePercentS);
  } else if (mode.thrust_mode_active) {
    const double direction = mode.thrust_target_idle ? kThrTowardsIdle : kThrTowardsLimit;
    const float lowerActualN1 = std::fmin(fadecOpp.selected_n1_actual_percent.Data,
                                          fadecOwn.selected_n1_actual_percent.Data);
    ratePercentS = direction * lookup(lowerActualN1, kThrRampActualN1Percent, kThrRampRatePercentS);
    ratePercentS = saturate(ratePercentS, -kThrMaxRatePercentS, kThrMaxRatePercentS);
  } else {
    ratePercentS = kHoldRatePercentS;
  }

  // ==================================================================================================================
  // Integrator
  // ==================================================================================================================
  double deltaN1 = ratePercentS * kIntegratorGain;
  deltaN1 *= dt;
  // Load from the higher actual N1 while A/THR is not engaged or not active, or in alpha floor (and on the first frame).
  s.loadIntegrator =
      !logic.athr_own_engaged || !mode.athr_active || mode.alpha_floor_mode_active || s.loadIntegrator;
  double integrator = s.n1CommandPercent;
  if (s.loadIntegrator) {
    integrator = static_cast<double>(std::fmax(fadecOpp.selected_n1_actual_percent.Data,
                                               fadecOwn.selected_n1_actual_percent.Data)) -
                 deltaN1;
  }
  double n1Percent = deltaN1 + integrator;
  // Limits: the FADEC N1 reference (the higher engine; current thrust lever detent limit), and 18 % below.
  const float n1RefPercent = std::fmax(fadecOpp.n1_ref_percent.Data, fadecOwn.n1_ref_percent.Data);
  if (n1Percent > n1RefPercent) {
    n1Percent = n1RefPercent;
  } else if (n1Percent < kMinN1CommandPercent) {
    n1Percent = kMinN1CommandPercent;
  }
  s.n1CommandPercent = n1Percent;
  s.loadIntegrator = false;

  // Alpha floor: the N1 maximum (TOGA), the integrator keeps tracking the actual N1.
  r.out.n1_c_percent = mode.alpha_floor_mode_active
                           ? static_cast<double>(std::fmax(fadecOwn.n1_maximum_percent.Data,
                                                           fadecOpp.n1_maximum_percent.Data))
                           : n1Percent;
  r.rewritten[N1Command] = true;
}

}  // namespace fmgc::readable::athr
