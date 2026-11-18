// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's characteristic speeds for the PFD speed scale and the FMGCs (generated FacComputer.cpp 688-689, 831-993):
//
//   V alpha max, V alpha prot, V stall warning   CAS x sqrt(|alpha - alpha0| / (alpha limit - alpha0)): the speed at
//                                which the filtered angle of attack would reach the limit at 1 g, alpha0 being the zero
//                                lift angle; limits for Mach and configuration, each rate limited to 1 deg/s
//   VS (1 g stall)               a + b x (weight - 40 t), per configuration, rate limited 5 kt/s (from 0 at power-up)
//   VLS                          max(1.3 g buffet CAS above 20000 ft (clean), VMCL 116 kt, k x VS) + up to 18 kt with
//                                speed brakes; k = 1.28 clean, 1.13 in the take-off configuration, 1.23 otherwise,
//                                rate limited 0.1/s (from 0 at power-up)
//   F (V3), S (V4)               1.26 VS(CONF 2) >= 131 kt, 1.23 VS(CONF 0) >= 136 kt; shown in CONF 2-3 and CONF 1-1+F
//   green dot (VMAN)             2 x weight (t) + 85 kt, + 1 kt per 1000 ft above 20000 ft; shown in CONF 0
//   VMAX                         min(VMO 350 kt (280 kt with the gear down), MMO 0.82, VFE of the configuration)
//   VFE next                     VFE of the next configuration; shown below 15000 ft and before CONF FULL
//   speed trend                  10 s x the washed-out (lagged) CAS
// The alpha-based speeds (and VLS) are sent as valid only 10 s after lift-off (speed_scale_visible).

#pragma once

#include "FacComputer_types.h"
#include "fac_blocks.h"
#include "sensors.h"

namespace fac::readable {

/** VMCL used as VLS floor (kt), and the F / S floors (VMCL + 15, VMCL + 20). */
constexpr double kVmclKn = 116.0;
constexpr double kFSpeedMarginKn = 15.0, kSSpeedMarginKn = 20.0;
constexpr double kFSpeedFactor = 1.26, kSSpeedFactor = 1.23;
/** VLS / VS factors: clean, take-off configuration, other configurations. */
constexpr double kVlsFactorClean = 1.28, kVlsFactorTakeoff = 1.13, kVlsFactorApproach = 1.23;
constexpr double kVlsFactorRate = 0.1;
constexpr double kVsRateKnS = 5.0;
/** VMO (kt) gear up / gear down, MMO. */
constexpr double kVmoKn = 350.0, kVmoGearDownKn = 280.0, kMmo = 0.82;
/** Altitudes (ft): above this VLS includes the buffet margin, green dot grows; below this VFE next is shown. */
constexpr double kHighAltitudeFt = 20000.0;
constexpr double kVfeNextMaxAltitudeFt = 15000.0;
/** Speed scale (alpha speeds, VLS) valid this long after lift-off (s). */
constexpr double kSpeedScaleDelaySeconds = 10.0;
/** Speed trend: CAS 30..660 kt, lag and washout 1 rad/s, 10 s ahead. */
constexpr double kTrendCasMinKn = 30.0, kTrendCasMaxKn = 660.0, kTrendSeconds = 10.0;

struct SpeedOutputs {
  double vAlphaMax, vAlphaProt, vStallWarn, vls, vStall, v3, v4, vMan, vMax, vFeNext, trend;
  bool v3Visible, v4Visible, vManVisible, vFeNextVisible, speedScaleVisible;
  float weightLbs;
};

class CharacteristicSpeeds {
 public:
  void reset();

  /** One frame. `alphaFilteredDeg` from the alpha floor detection. */
  const SpeedOutputs& update(const fac_inputs& in, const Sensors& s, double alphaFilteredDeg);

  // state access for the checker
  const blocks::RateLimiter& alpha0() const { return alpha0_; }
  const blocks::RateLimiter& alphaMax() const { return alphaMax_; }
  const blocks::RateLimiter& alphaProt() const { return alphaProt_; }
  const blocks::RateLimiter& alphaStallWarn() const { return alphaStallWarn_; }
  const blocks::RateLimiterFrom& vStallRate() const { return vStall_; }
  const blocks::RateLimiterFrom& vlsFactorRate() const { return vlsFactor_; }
  double takeoffConfig() const { return takeoffConfig_; }
  const blocks::ConfirmNode& speedScaleConfirm() const { return speedScale_; }
  const blocks::LagFilter<double>& trendLag() const { return trendLag_; }
  const blocks::WashoutFilter& trendWashout() const { return trendWashout_; }

 private:
  blocks::RateLimiter alpha0_;          // sf_RateLimiter_c
  blocks::RateLimiter alphaMax_;        // sf_RateLimiter_a
  blocks::RateLimiter alphaProt_;       // sf_RateLimiter_n
  blocks::RateLimiter alphaStallWarn_;  // sf_RateLimiter_j
  blocks::RateLimiterFrom vStall_;      // sf_RateLimiter_g0
  blocks::RateLimiterFrom vlsFactor_;   // sf_RateLimiter_g
  double takeoffConfig_ = 0.0;          // takeoff_config: the configuration set on the ground, -1 once changed
  blocks::ConfirmNode speedScale_;      // sf_MATLABFunction_p4
  blocks::LagFilter<double> trendLag_;  // sf_LagFilter_fb
  blocks::WashoutFilter trendWashout_;  // sf_WashoutFilter
  SpeedOutputs out_{};
};

}  // namespace fac::readable
