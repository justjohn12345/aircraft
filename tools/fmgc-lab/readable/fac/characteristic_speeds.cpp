// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's characteristic speeds (generated FacComputer.cpp 688-689, 831-993). See characteristic_speeds.h.

#include "characteristic_speeds.h"

#include <cmath>

#include "vls_buffet_table.h"

namespace fac::readable {

namespace {

namespace a429 = blocks::a429;

constexpr double kConfig[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};  // CONF 0, 1, 1+F, 2, 3, FULL
constexpr double kMach[4] = {0.0, 0.5, 0.9, 1.0};

/** Zero-lift angle of attack per configuration (deg). */
constexpr double kAlpha0Deg[6] = {-1.84, -1.84, -2.18, -4.72, -4.27, -6.94};
/** Angle of attack limits (deg): rows configuration, columns Mach 0, 0.5, 0.9, 1. alpha stall warning = alpha prot. */
constexpr double kAlphaMaxDeg[24] = {8.7,  8.7,  6.4,  6.4,  13.6, 13.6, 13.6, 13.6, 13.6, 13.6, 13.6, 13.6,
                                     14.2, 14.2, 14.2, 14.2, 13.1, 13.1, 13.1, 13.1, 13.0, 13.0, 13.0, 13.0};
constexpr double kAlphaProtDeg[24] = {6.5,  6.5,  4.6,  4.6,  11.7, 11.7, 11.7, 11.7, 11.7, 11.7, 11.7, 11.7,
                                      11.9, 11.9, 11.9, 11.9, 11.0, 11.0, 11.0, 11.0, 10.6, 10.6, 10.6, 10.6};
constexpr double kAlphaStallWarnDeg[24] = {6.5,  6.5,  4.6,  4.6,  11.7, 11.7, 11.7, 11.7, 11.7, 11.7, 11.7, 11.7,
                                           11.9, 11.9, 11.9, 11.9, 11.0, 11.0, 11.0, 11.0, 10.6, 10.6, 10.6, 10.6};
/** 1 g stall speed at 40 t (kt) and its slope (kt per t) per configuration. */
constexpr double kVs40tKn[6] = {125.64, 102.24, 93.76, 91.89, 91.89, 84.47};
constexpr double kVsSlopeKnPerT[6] = {1.2, 0.99, 0.91, 0.88, 0.88, 0.81};
/** VFE of the configuration (VMO in CONF 0) and of the next one (kt). */
constexpr double kVfeKn[6] = {350.0, 230.0, 215.0, 200.0, 185.0, 177.0};
constexpr double kVfeNextKn[6] = {230.0, 200.0, 200.0, 185.0, 177.0, 177.0};
/** VLS increment with speed brakes: ELAC speed brake extension (deg, negative = extended) -> kt. */
constexpr double kSpeedBrakeDeg[2] = {-40.0, 0.0};
constexpr double kSpeedBrakeVlsKn[2] = {18.0, 0.0};
/** VLS buffet: flight level range of the table, the weight/CG parameter (see buffetMach). */
constexpr double kBuffetMinFl = 200.0, kBuffetMaxFl = 410.0;

constexpr double kLbsPerTonne = 2205.0;

/** VS = a + b x (weight - 40 t) (MATLABFunction2). */
double stallSpeed(double a, double b, double weightLbs) { return (weightLbs / kLbsPerTonne - 40.0) * b + a; }

/** CAS (kt) for a Mach number at a static pressure (hPa), subsonic. */
double casForMach(double mach, double pressureRatio) {
  return std::sqrt(std::pow((std::pow(mach * mach * 0.2 + 1.0, 3.5) - 1.0) * pressureRatio + 1.0, 0.2857142857142857) -
                   1.0) *
         1479.1;
}

/** The speed at which the angle of attack would reach `alphaLimit` (CalculateV_alpha_max). */
double alphaSpeed(double cas, double alpha, double alpha0, double alphaLimit) {
  return std::sqrt(std::abs(alpha - alpha0) / (alphaLimit - alpha0)) * cas;
}

}  // namespace

void CharacteristicSpeeds::reset() {
  alpha0_.reset();
  alphaMax_.reset();
  alphaProt_.reset();
  alphaStallWarn_.reset();
  vStall_.reset();
  vlsFactor_.reset();
  takeoffConfig_ = 0.0;
  speedScale_.reset();
  trendLag_.reset();
  trendWashout_.reset();
}

const SpeedOutputs& CharacteristicSpeeds::update(const fac_inputs& in, const Sensors& s, double alphaFilteredDeg) {
  const double dt = in.time.dt;
  const base_fac_bus_inputs& b = in.bus_inputs;
  const double config = s.slatFlap.configIndex;
  const double mach = s.adr.mach;
  const double cas = s.adr.casKn;
  const float altitude = s.adr.altitudeFt;
  SpeedOutputs& o = out_;

  // 688-689: speeds valid 10 s after lift-off (any frame on the ground resets it)
  o.speedScaleVisible = speedScale_.update(!s.onGround, dt, true, kSpeedScaleDelaySeconds);

  // 831-848: alpha-based speeds from the filtered alpha; each limit rate limited to 1 deg/s
  const double alpha0 = alpha0_.update(blocks::lookup1(config, kConfig, kAlpha0Deg), 1.0, 1.0, dt);
  const double alphaMax = alphaMax_.update(blocks::lookup2(mach, config, kMach, kConfig, kAlphaMaxDeg), 1.0, 1.0, dt);
  o.vAlphaMax = alphaSpeed(cas, alphaFilteredDeg, alpha0, alphaMax);
  const double alphaProt = alphaProt_.update(blocks::lookup2(mach, config, kMach, kConfig, kAlphaProtDeg), 1.0, 1.0, dt);
  o.vAlphaProt = alphaSpeed(cas, alphaFilteredDeg, alpha0, alphaProt);
  const double alphaStallWarn =
      alphaStallWarn_.update(blocks::lookup2(mach, config, kMach, kConfig, kAlphaStallWarnDeg), 1.0, 1.0, dt);
  o.vStallWarn = alphaSpeed(cas, alphaFilteredDeg, alpha0, alphaStallWarn);

  // 851-863: weight from the own FMGC (FM weight when normal, else the FAC weight word it relays); VS rate limited
  o.weightLbs = a429::isNormal(b.fmgc_own_bus.fm_weight_lbs) ? b.fmgc_own_bus.fm_weight_lbs.Data
                                                              : b.fmgc_own_bus.fac_weight_lbs.Data;
  const double weight = o.weightLbs;
  o.vStall = vStall_.update(stallSpeed(blocks::lookup1(config, kConfig, kVs40tKn),
                                       blocks::lookup1(config, kConfig, kVsSlopeKnPerT), weight),
                            kVsRateKnS, kVsRateKnS, dt, 0.0, false);

  // 864-874: speed brake extension from the first ELAC with a normal word
  double speedBrake;
  if (a429::isNormal(b.elac_1_bus.speedbrake_extension_deg)) {
    speedBrake = b.elac_1_bus.speedbrake_extension_deg.Data;
  } else if (a429::isNormal(b.elac_2_bus.speedbrake_extension_deg)) {
    speedBrake = b.elac_2_bus.speedbrake_extension_deg.Data;
  } else {
    speedBrake = 0.0;
  }

  // 876-890: 1.3 g buffet Mach for the weight/CG parameter and flight level (weight in t, CG in % MAC)
  const double cgPercent = b.fmgc_own_bus.fm_cg_percent.Data * 100.0;
  const double tonnes = o.weightLbs / kLbsPerTonne;
  const double p = (tonnes * 0.19807001167721894 - 0.0347376119873668) * 1.3 - 5.9;
  const double flightLevel = blocks::limit(0.01 * altitude, kBuffetMinFl, kBuffetMaxFl);
  const double buffetParameter = std::fmax(p - (p - -6.5) / (cgPercent - -405.0) * (cgPercent - 25.0), 1.0);
  const double buffetMach = blocks::limit(
      blocks::lookup2(buffetParameter, flightLevel, kBuffetParameter, kBuffetFlightLevel, kBuffetMach), 0.0, 1.0);

  // 892-918: VLS factor. The take-off configuration is the one held on the ground (both main gears, an LGCIU); it is
  // forgotten (-1) as soon as the configuration changes in flight.
  if (!s.gear.allLgciuLost && s.gear.leftMainPressed && s.gear.rightMainPressed) {
    takeoffConfig_ = s.slatFlap.configIndex;
  } else if (takeoffConfig_ != s.slatFlap.configIndex) {
    takeoffConfig_ = -1.0;
  }
  double factor;
  if (s.slatFlap.configIndex == 0.0F) {
    factor = kVlsFactorClean;
  } else if (takeoffConfig_ != -1.0) {
    factor = kVlsFactorTakeoff;
  } else {
    factor = kVlsFactorApproach;
  }
  const double vlsFactor = vlsFactor_.update(factor, kVlsFactorRate, kVlsFactorRate, dt, 0.0, false);
  const double buffetCas =
      altitude >= kHighAltitudeFt ? casForMach(buffetMach, s.adr.staticPressureHpa / 1013.25) : 0.0;
  speedBrake = blocks::limit(speedBrake, -40.0, 0.0);
  o.vls = std::fmax(std::fmax(buffetCas, kVmclKn), vlsFactor * o.vStall) +
          blocks::lookup1(speedBrake, kSpeedBrakeDeg, kSpeedBrakeVlsKn);

  // 927-946: F, S and green dot
  const double vsConf2 = stallSpeed(blocks::lookup1(2.0, kConfig, kVs40tKn), blocks::lookup1(2.0, kConfig, kVsSlopeKnPerT),
                                    weight);
  o.v3 = std::fmax(kFSpeedFactor * vsConf2, kVmclKn + kFSpeedMarginKn);
  const double vsConf0 = stallSpeed(blocks::lookup1(0.0, kConfig, kVs40tKn), blocks::lookup1(0.0, kConfig, kVsSlopeKnPerT),
                                    weight);
  o.v4 = std::fmax(kSSpeedFactor * vsConf0, kVmclKn + kSSpeedMarginKn);
  o.vMan = (tonnes * 2.0 + 85.0) + std::fmax(altitude - kHighAltitudeFt, 0.0) / 1000.0;
  const float idx = s.slatFlap.configIndex;
  o.v4Visible = idx == 1.0F || idx == 2.0F;
  o.v3Visible = idx == 3.0F || idx == 4.0F;
  o.vManVisible = idx == 0.0F;

  // 948-963: VMAX and VFE next
  const double mmo = std::fmax(kMmo, 0.0);
  const double vmo = s.gear.mainGearOut ? kVmoGearDownKn : kVmoKn;
  o.vMax = std::fmin(std::fmin(vmo, casForMach(mmo, std::fmax(static_cast<double>(s.adr.staticPressureHpa), 0.0) / 1013.25)),
                     blocks::lookup1(config, kConfig, kVfeKn));
  o.vFeNext = blocks::lookup1(config, kConfig, kVfeNextKn);
  o.vFeNextVisible = idx < 5.0F && altitude <= kVfeNextMaxAltitudeFt;

  // 964-975: speed trend
  const double lagged = trendLag_.update(blocks::limit(cas, kTrendCasMinKn, kTrendCasMaxKn), 1.0, dt);
  o.trend = kTrendSeconds * trendWashout_.update(lagged, 1.0, dt);
  return o;
}

}  // namespace fac::readable
