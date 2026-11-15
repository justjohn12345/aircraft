// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Lateral laws (HDG, TRACK, HPATH = NAV, LOC_CPT, LOC_TRACK, ROLL_OUT) and the lateral output stage.
// Generated: FmgcOuterLoops.cpp 2232–2275 (bank limit), 2276–2289 (NAV feedforward), 2290–2330 (HDG), 2331–2370
// (LOC_CPT, Phi_loc_c), 2371–2412 (LOC_TRACK deviation and runway alignment), 2412–2442 (TRACK), 2443–2531 (roll law
// switch: HPATH and LOC_TRACK are computed in their cases), 2532–2540 (bank limit, FD roll), 2541–2626 (ROLL_OUT),
// 2627–2716 (sideslip and nosewheel switches), 2718–2750 (AP roll), 4566–4572 (the HDG / TRACK delay lines).
//
// Angles: the generated code wraps every heading difference with MATLAB mod(x, 360) (rt_modd) in a fixed pattern,
// kept here operation for operation (wrappedDifference, shortestAngle) because it decides the rounding.

#include <cmath>
#include <cstdint>

#include "laws_lateral.h"

namespace fmgc::readable::laws {

namespace {

constexpr double kDegToRad = 0.017453292519943295;

// ---- Bank limit (all laws): from TAS (15° at 0 kt, 30° from 150 to 300 kt, 19° from 450 kt), at most 25° except in
// LOC / roll-out where it is 10° below 700 ft RA; in NAV the FMS limit replaces it when non-zero. Rate limited 5°/s.
constexpr double kBankLimitTasKn[5] = {0, 150, 300, 450, 600};
constexpr double kBankLimitDeg[5] = {15, 30, 30, 19, 19};
constexpr double kBankLimitMaxDeg = 25;
constexpr double kBankLimitLocLowDeg = 10;
constexpr double kBankLimitLocLowRaFt = 700;
constexpr double kBankLimitRateDegS = 5;
constexpr double kBankLimitInitDeg = 25;

// ---- HDG / TRACK select: roll = 1.16 · gain(TAS) · heading error − 2 · yaw rate, within the heading bank limit
constexpr double kHeadingGainTasKn[7] = {100, 150, 200, 250, 300, 400, 500};
constexpr double kHeadingGain[7] = {1.6000000000000001, 1.6000000000000001, 2, 2.7999999999999998,
                                    3.2000000000000002, 4.2000000000000002, 4.5};  // deg roll per deg error
constexpr double kHeadingGainFactor = 1.1599999999999999;
constexpr double kHeadingYawRateGain = -2;  // deg roll per deg/s of yaw rate

// ---- HPATH (NAV): FMS roll order (rate limited, lagged) + track-angle error and cross-track error laws.
constexpr double kNavTauS = 3, kNavZeta = 0.80000000000000004;  // second-order path capture response
constexpr double kFmsPhiRateDegS = 10;
constexpr double kFmsPhiLagC1 = 4;  // rad/s
constexpr double kNavXtkMaxDeg = 45;  // cross-track term limited to a 45° intercept

// ---- LOC_CPT: same law shape as NAV on the beam, tau 2 s.
constexpr double kLocTauS = 2, kLocZeta = 0.80000000000000004;
constexpr double kLocDmeMinNm = 2, kLocDmeMaxNm = 100;  // distance to the localizer antenna used for the deviation
constexpr double kLocDeviationLimitMinDeg = 15;  // deviation term limited to the intercept angle, 15..115°
constexpr double kLocDeviationLimitMaxDeg = 115;

// ---- LOC_TRACK: lagged deviation with 10 s of lead (gain on TAS and RA) + course error + sideslip and alignment terms
constexpr double kLocDevLagC1 = 1;      // rad/s
constexpr double kLocDevLeadS = 10;     // deviation + 10 s · d(deviation)/dt ...
constexpr double kLocDevLeadLagC1 = 1;  // ... lagged at 1 rad/s
constexpr double kLocDevGainTasKn[7] = {100, 150, 200, 250, 300, 400, 500};
constexpr double kLocDevGainTas[7] = {1.6000000000000001, 1.6000000000000001, 2, 2.7999999999999998,
                                      3.2000000000000002, 4.2000000000000002, 4.5};
constexpr double kLocDevGainFactor = 1.1000000000000001;
constexpr double kLocDevGainRaFt[6] = {0, 50, 100, 1000, 2500, 3000};
constexpr double kLocDevGainRa[6] = {3.7000000000000002, 3.7000000000000002, 4.4000000000000004,
                                     7.7999999999999998, 15, 15};  // deg roll per deg of deviation (× TAS gain)
constexpr double kLocCourseErrorGain = 0.20000000000000001;  // deg roll per kt of cross-course speed
constexpr double kLocSideslipGainRaFt[5] = {0, 30, 35, 100, 200};
constexpr double kLocSideslipGainRa[5] = {0, 0, 0.80000000000000004, 0.80000000000000004, 0.80000000000000004};
constexpr double kLocAlignGainRaFt[6] = {-10, 0, 5, 10, 30, 100};
constexpr double kLocAlignGainRa[6] = {0, 0.40000000000000002, 0.40000000000000002, 0.40000000000000002,
                                       0.40000000000000002, 0.40000000000000002};
constexpr double kLocTrackRollMaxDeg = 15;
constexpr double kLocTrackFadeUp = 0.5;     // LOC_CPT -> LOC_TRACK roll command blend over 2 s ...
constexpr double kLocTrackFadeDown = -1000; // ... reset at once
// Runway alignment (decrab) sideslip command below 30 ft RA: 1.5 · (sideslip − (runway course − heading)), lagged
constexpr double kAlignMaxRaFt = 30;
constexpr double kAlignGain = 1.5;
constexpr double kAlignLagC1 = 0.69999999999999996;  // rad/s
constexpr double kAlignMaxDeg = 15;

// ---- ROLL_OUT: heading target = runway course + deviation terms; yaw order = rudder at engagement faded into yaw-rate
// damping, minus 1.2 · heading error; output 2.4 × that, faded in from the rudder at engagement.
constexpr double kRollOutDevGainFt = 1200;  // sin(deviation) · DME (nm) · 1200
constexpr double kRollOutDevMax = 70;
constexpr double kRollOutIntegralGain = 0.10000000000000001;  // per second
constexpr double kRollOutIntegralMaxDeg = 3;
constexpr double kRollOutDevGainGsKn[3] = {0, 150, 200};
constexpr double kRollOutDevGainGs[3] = {0.80000000000000004, 0.20000000000000001, 0.20000000000000001};
constexpr double kRollOutBeamHeadingGain = 1.8;
constexpr double kRollOutDampingFadeUp = 0.125;  // over 8 s
constexpr double kRollOutYawRateGain = 0.29999999999999999;
constexpr double kRollOutHeadingErrorGain = -1.2;
constexpr double kRollOutYawMaxDeg = 20;
constexpr double kRollOutCommandFadeUp = 0.33333333333333331;  // over 3 s
constexpr double kRollOutFadeDown = -1000;
constexpr double kRollOutCommandGain = 2.3999999999999999;  // two identical copies in the generated code (beta, nose)

// ---- Output stage
constexpr double kFdPhiLagC1 = 2;          // rad/s
constexpr double kNegate = -1;             // FD roll sign gains (see the FD roll command)
constexpr double kApRollRateDegS = 5;      // AP roll command rate ...
constexpr double kApRollRateLocDegS = 7.5; // ... in LOC_CPT, LOC_TRACK, ROLL_OUT
constexpr double kApRollRateChange = 2.5;  // deg/s per s
constexpr double kApPhiLagC1 = 1;          // rad/s
constexpr double kApEngageFadeUp = 1;      // AP roll command faded in over 1 s ...
constexpr double kApEngageFadeDown = -10;  // ... and out over 0.1 s
constexpr double kFdBetaLagC1 = 2;         // rad/s
constexpr double kNosewheelGain = -0.040000000000000001;  // deg -> normalised nosewheel order (±1)

/** Saturation block: NaN passes through. */
double saturate(double x, double lo, double hi) {
  return x > hi ? hi : (x < lo ? lo : x);
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

/** a − b wrapped to [0, 360): how far a is clockwise of b. Generated pattern: mod((a − (b + 360)) + 360, 360). */
double wrappedDifference(double a, double b) {
  return wrap360((a - (b + 360.0)) + 360.0);
}

/** From a clockwise difference a − b in [0, 360): the shorter signed turn from a to b (b − a in [−180, 180]). */
double shortestAngle(double clockwise) {
  double counterClockwise = wrap360(360.0 - clockwise);
  return clockwise < counterClockwise ? -clockwise : counterClockwise;
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

/** MATLAB interp1 (linear, clamped at both ends; NaN for NaN), as the generated embedded function computes it. */
template <int N>
double interpolateClamped(double u, const double (&bp)[N], const double (&table)[N]) {
  if (u > bp[N - 1]) {
    return table[N - 1];
  }
  int high = N, low = 0, lowPlus1 = 2;
  while (high > lowPlus1) {
    int mid = ((low + high) + 1) >> 1;
    if (u >= bp[mid - 1]) {
      low = mid - 1;
      lowPlus1 = mid + 1;
    } else {
      high = mid;
    }
  }
  double r = (u - bp[low]) / (bp[low + 1] - bp[low]);
  if (r == 0.0) {
    return table[low];
  }
  if (r == 1.0) {
    return table[low + 1];
  }
  if (table[low + 1] == table[low]) {
    return table[low];
  }
  return (1.0 - r) * table[low] + r * table[low + 1];
}

}  // namespace

const char* lateralLawName(int law) {
  static const char* names[kLateralLawCount] = {"none", "hdg", "track", "hpath", "loc_cpt", "loc_track", "roll_out"};
  return law >= 0 && law < kLateralLawCount ? names[law] : "?";
}

void FirstSecondDetector::reset() {
  for (bool& h : history) {
    h = false;
  }
}

bool FirstSecondDetector::update(bool active, double dt) {
  if (!active) {
    reset();
  }
  double samples = 1.0 / dt;  // delay of 1 s
  bool delayed;
  if (samples < 1.0) {
    delayed = active;
  } else {
    uint32_t n = samples > 100.0 ? 100U : static_cast<uint32_t>(std::fmod(std::trunc(samples), 4.294967296E+9));
    delayed = history[100U - n];
  }
  for (int i = 0; i < 99; i++) {
    history[i] = history[i + 1];
  }
  history[99] = active;
  return active != delayed;
}

void LateralLaws::reset() {
  nav_.fmsPhiPrimed = false;
  nav_.fmsPhiLag.reset();
  hdg_.reset();
  track_.reset();
  locCapture_.deviationLimitPrimed = false;
  locTrack_.deviationLag.reset();
  locTrack_.previousDeviation = 0;
  locTrack_.deviationLeadLag.reset();
  locTrack_.runwayHeadingError.reset();
  locTrack_.alignLag.reset();
  locTrack_.captureToTrack.reset();
  rollOut_.deviationIntegral = 0;
  rollOut_.coursePrimed = false;
  rollOut_.headingError.reset();
  rollOut_.rateDampingBlend.reset();
  rollOut_.rudderPrimed = false;
  rollOut_.commandBlend.reset();
  output_.bankLimitPrimed = false;
  output_.fdPhiLag.reset();
  output_.apPhiIntegratorLoad = true;
  output_.apRollRateLimitPrimed = false;
  output_.apPhiLag.reset();
  output_.apEngagedBlend.reset();
  output_.fdBetaLag.reset();
}

/**
 * HDG / TRACK select: error to the selected heading (track) the shorter way, except that a turn of 10–20° keeps its
 * direction while the target moves through the reciprocal (TurnDirectionChart; always the shorter way during the first
 * second of the mode). Roll = 1.16 · gain(TAS) · error − 2 · yaw rate, limited by the heading bank limit (5° for small
 * heading changes up to 30° for 20°+, until the roll command is back within it).
 */
double LateralLaws::headingSelect(HeadingSelectLaw& law, bool active, double targetDeg, double currentDeg,
                                  const ap_laws_output& in) {
  double clockwise = wrappedDifference(targetDeg, currentDeg);
  double counterClockwise = -wrap360(360.0 - clockwise);
  bool shortPath = law.justEngaged.update(active, in.time.dt);
  double error = law.turn.update(clockwise, counterClockwise, shortPath);
  double phi = error * lookup(in.data.V_tas_kn, kHeadingGainTasKn, kHeadingGain) * kHeadingGainFactor +
               kHeadingYawRateGain * in.data.rk_deg_s;
  double limit = law.bankLimit.update(targetDeg, error, phi);
  return phi > limit ? limit : (phi < -limit ? -limit : phi);
}

void LateralLaws::computeLaws(const ap_laws_output& in, LateralResults& out) {
  out = LateralResults{};
  const double dt = in.time.dt;
  const double vGnd = in.data.V_gnd_kn;
  const double ra = in.data.H_radio_ft;
  const double locDeviation = in.data.nav_loc_error_deg;
  const int law = static_cast<int>(in.input.lateral_law);

  // ---- HPATH (NAV): roll = FMS roll order − k2 · V_gnd · (−track-angle error + limited cross-track term)
  blocks::NavGains nav = blocks::navGains(kNavTauS, kNavZeta);
  if (!nav_.fmsPhiPrimed) {
    nav_.fmsPhiRateLimited = 0;
    nav_.fmsPhiPrimed = true;
  }
  nav_.fmsPhiRateLimited += std::fmax(std::fmin(in.data.fms_phi_deg - nav_.fmsPhiRateLimited, kFmsPhiRateDegS * dt),
                                      -kFmsPhiRateDegS * dt);
  double fmsPhi = nav_.fmsPhiLag.update(nav_.fmsPhiRateLimited, kFmsPhiLagC1, dt);
  double xtkTerm = saturate(-in.data.fms_xtk_nmi * nav.k1 / vGnd, -kNavXtkMaxDeg, kNavXtkMaxDeg);
  double hpathPhi = fmsPhi - (-in.data.fms_tae_deg + xtkTerm) * nav.k2 * vGnd;

  // ---- HDG
  double hdgPhi = headingSelect(hdg_, law == Hdg, in.input.Psi_c_deg, in.data.Psi_magnetic_deg, in);

  // ---- LOC_CPT: the NAV law on the beam. Track error to the beam bearing (course + deviation) plus the cross-beam
  // distance (sin(deviation) · DME) as a track angle, limited to the intercept angle at capture.
  blocks::NavGains loc = blocks::navGains(kLocTauS, kLocZeta);
  double courseTrue = in.data.nav_loc_deg - in.data.nav_loc_magvar_deg;
  double course = wrap360(wrap360(courseTrue) + 360.0);
  double beamBearing = wrap360(wrap360(locDeviation + course) + 360.0);
  double trackFromBeam = wrappedDifference(in.data.Chi_true_deg, beamBearing);
  double interceptAngle = std::abs(shortestAngle(wrappedDifference(in.data.Chi_true_deg, course)));
  bool locCptActive = law == LocCpt;
  if (!locCapture_.deviationLimitPrimed) {
    locCapture_.deviationLimitDeg = interceptAngle;
    locCapture_.deviationLimitPrimed = true;
  }
  if (!locCptActive) {
    locCapture_.deviationLimitDeg =
        std::fmin(std::fmax(interceptAngle, kLocDeviationLimitMinDeg), kLocDeviationLimitMaxDeg);
  }
  if (locCptActive && interceptAngle < kLocDeviationLimitMinDeg) {
    locCapture_.deviationLimitDeg = kLocDeviationLimitMinDeg;
  }
  double dme = saturate(in.data.nav_dme_nmi, kLocDmeMinNm, kLocDmeMaxNm);
  double locXtkTerm = std::sin(kDegToRad * locDeviation) * dme * loc.k1 / vGnd;
  locXtkTerm = saturate(locXtkTerm, -locCapture_.deviationLimitDeg, locCapture_.deviationLimitDeg);
  double phiLoc = (shortestAngle(trackFromBeam) + locXtkTerm) * loc.k2 * vGnd;

  // ---- LOC_TRACK filters: deviation lagged, plus 10 s of its rate, lagged
  double devLagged = locTrack_.deviationLag.update(locDeviation, kLocDevLagC1, dt);
  double previous = locTrack_.previousDeviation;
  locTrack_.previousDeviation = devLagged;
  double devLead =
      locTrack_.deviationLeadLag.update(devLagged + kLocDevLeadS * ((devLagged - previous) / dt), kLocDevLeadLagC1, dt);
  // runway alignment sideslip below 30 ft: turn the nose to the runway course, plus the current sideslip
  double toRunway = wrappedDifference(course, in.data.Psi_true_deg);
  double runwayHeadingError = locTrack_.runwayHeadingError.update(toRunway, -wrap360(360.0 - toRunway), true);
  double align = ra <= kAlignMaxRaFt ? (-runwayHeadingError + in.data.beta_deg) * kAlignGain : 0.0;
  double alignBeta = saturate(locTrack_.alignLag.update(align, kAlignLagC1, dt), -kAlignMaxDeg, kAlignMaxDeg);
  double locTrackBlend = locTrack_.captureToTrack.update(law == LocTrack, kLocTrackFadeUp, kLocTrackFadeDown, dt, 0.0);

  // ---- TRACK
  double trackPhi = headingSelect(track_, law == Track, in.input.Chi_c_deg, in.data.Psi_magnetic_track_deg, in);

  // ---- LOC_TRACK: roll = deviation (with lead) + course error (cross-course speed) + sideslip + alignment terms,
  // faded in from the LOC_CPT command
  double magneticVariation = shortestAngle(wrappedDifference(in.data.Psi_magnetic_deg, in.data.Psi_true_deg));
  double trackTrue = wrap360(wrap360(in.data.Psi_magnetic_track_deg + magneticVariation) + 360.0);
  double trackFromCourse = wrappedDifference(trackTrue, course);
  double blend = saturate(locTrackBlend, 0.0, 1.0);
  double courseError = shortestAngle(trackFromCourse);
  double locTrackRoll =
      (devLead * lookup(in.data.V_tas_kn, kLocDevGainTasKn, kLocDevGainTas) * kLocDevGainFactor *
           lookup(ra, kLocDevGainRaFt, kLocDevGainRa) +
       std::sin(kDegToRad * courseError) * vGnd * kLocCourseErrorGain) +
      (in.data.beta_deg * lookup(ra, kLocSideslipGainRaFt, kLocSideslipGainRa) +
       alignBeta * lookup(ra, kLocAlignGainRaFt, kLocAlignGainRa));
  locTrackRoll = saturate(locTrackRoll, -kLocTrackRollMaxDeg, kLocTrackRollMaxDeg);
  double locTrackPhi = (1.0 - blend) * phiLoc + locTrackRoll * blend;

  // ---- ROLL_OUT: heading target from the runway course memorised at engagement, the deviation (proportional +
  // integral) and the bearing of the beam; rudder command from the heading error, yaw rate damping faded in.
  double rollOutDev =
      saturate(std::sin(kDegToRad * locDeviation) * saturate(in.data.nav_dme_nmi, kLocDmeMinNm, kLocDmeMaxNm) *
                   kRollOutDevGainFt,
               -kRollOutDevMax, kRollOutDevMax);
  bool rollOutActive = law == RollOut;
  if (!rollOutActive) {
    rollOut_.deviationIntegral = 0;
  }
  rollOut_.deviationIntegral += kRollOutIntegralGain * rollOutDev * dt;
  rollOut_.deviationIntegral =
      saturate(rollOut_.deviationIntegral, -kRollOutIntegralMaxDeg, kRollOutIntegralMaxDeg);
  if (!rollOutActive || !rollOut_.coursePrimed) {
    rollOut_.courseDeg = wrap360(wrap360(courseTrue) + 360.0);
    rollOut_.coursePrimed = true;
  }
  double rollOutBeam = wrap360(wrap360(locDeviation + rollOut_.courseDeg) + 360.0);
  double headingToBeam = shortestAngle(wrappedDifference(in.data.Psi_true_deg, rollOutBeam));
  double headingTarget = wrap360(
      wrap360(((rollOutDev * lookup(vGnd, kRollOutDevGainGsKn, kRollOutDevGainGs) + rollOut_.deviationIntegral) +
               kRollOutBeamHeadingGain * headingToBeam) +
              in.data.Psi_true_deg) +
      360.0);
  double toTarget = wrappedDifference(headingTarget, in.data.Psi_true_deg);
  double headingError = rollOut_.headingError.update(toTarget, -wrap360(360.0 - toTarget), true);
  double damping = saturate(
      rollOut_.rateDampingBlend.update(rollOutActive, kRollOutDampingFadeUp, kRollOutFadeDown, dt, 0.0), 0.0, 1.0);
  if (!rollOutActive || !rollOut_.rudderPrimed) {
    rollOut_.rudderAtEngagementDeg = in.data.zeta_deg;
    rollOut_.rudderPrimed = true;
  }
  double yaw = (kRollOutYawRateGain * in.data.rk_deg_s * damping + (1.0 - damping) * rollOut_.rudderAtEngagementDeg) +
               kRollOutHeadingErrorGain * headingError;
  yaw = saturate(yaw, -kRollOutYawMaxDeg, kRollOutYawMaxDeg);
  double fade = saturate(
      rollOut_.commandBlend.update(rollOutActive, kRollOutCommandFadeUp, kRollOutFadeDown, dt, 0.0), 0.0, 1.0);
  double rollOutYaw = kRollOutCommandGain * yaw * fade + (1.0 - fade) * rollOut_.rudderAtEngagementDeg;

  out.cmd[LatNone] = {in.data.Phi_deg, 0.0, 0.0};  // no law: roll command = current roll
  out.cmd[Hdg] = {hdgPhi, 0.0, 0.0};
  out.cmd[Track] = {trackPhi, 0.0, 0.0};
  out.cmd[Hpath] = {hpathPhi, 0.0, 0.0};
  out.cmd[LocCpt] = {phiLoc, 0.0, 0.0};
  out.cmd[LocTrack] = {locTrackPhi, alignBeta, 0.0};
  out.cmd[RollOut] = {0.0, rollOutYaw, rollOutYaw};
  out.phiLocDeg = phiLoc;
  for (bool& r : out.rewritten) {
    r = true;
  }
}

bool LateralLaws::computeOutputs(const ap_laws_output& in, const LateralCommand (&cmd)[kLateralLawCount],
                                 LateralOutputs& out) {
  const double dt = in.time.dt;
  const double phi = in.data.Phi_deg;
  const int law = static_cast<int>(in.input.lateral_law);
  const int selected = law >= LatNone && law <= LocTrack ? law : RollOut;  // the generated switches' default case
  const bool locOrRollOut = law == LocCpt || law == LocTrack || law == RollOut;

  // ---- bank limit
  double limit = interpolateClamped(std::abs(in.data.V_tas_kn), kBankLimitTasKn, kBankLimitDeg);
  if (!locOrRollOut) {
    limit = std::fmin(kBankLimitMaxDeg, limit);
  } else if (in.data.H_radio_ft < kBankLimitLocLowRaFt) {
    limit = kBankLimitLocLowDeg;
  }
  double fmsLimit = std::abs(in.data.fms_phi_limit_deg);
  if (!output_.bankLimitPrimed) {
    output_.bankLimitDeg = kBankLimitInitDeg;
    output_.bankLimitPrimed = true;
  }
  if (law == Hpath && fmsLimit > 0.0) {
    limit = fmsLimit;
  }
  output_.bankLimitDeg += std::fmax(std::fmin(limit - output_.bankLimitDeg, kBankLimitRateDegS * dt),
                                    -kBankLimitRateDegS * dt);
  double phiCmd = cmd[selected].phi;
  double phiLimited = phiCmd > output_.bankLimitDeg
                          ? output_.bankLimitDeg
                          : (phiCmd >= -output_.bankLimitDeg ? phiCmd : -output_.bankLimitDeg);  // NaN -> -limit

  // ---- FD: roll error, lagged. The generated code negates the error into the lag filter and negates its output
  // (Gain_Gain_lv = Gain_Gain_on = -1): the same value except the sign of an exact zero (x + -x = +0 either way), so
  // it is kept to stay bit-exact (the filter holds the negated error).
  out.fdPhiDeg = kNegate * output_.fdPhiLag.update(kNegate * (phiLimited - phi), kFdPhiLagC1, dt);

  // ---- sideslip and nosewheel
  out.apBetaDeg = cmd[selected].beta;
  out.fdBetaDeg = output_.fdBetaLag.update(out.apBetaDeg, kFdBetaLagC1, dt);
  double nose = in.input.ap_engaged ? cmd[selected].nosewheel : 0.0;
  out.nosewheel = saturate(kNosewheelGain * nose, -1.0, 1.0);

  // ---- AP: roll command moved towards the limited command at the roll-rate limit, lagged, faded in with the AP
  output_.apPhiIntegratorLoad = !in.input.ap_engaged || output_.apPhiIntegratorLoad;
  if (output_.apPhiIntegratorLoad) {
    output_.apPhiIntegrator = phi;
  }
  double error = phiLimited - output_.apPhiIntegrator;
  if (!output_.apRollRateLimitPrimed) {
    output_.apRollRateLimit = kApRollRateDegS;
    output_.apRollRateLimitPrimed = true;
  }
  double rateTarget = locOrRollOut ? kApRollRateLocDegS : kApRollRateDegS;
  output_.apRollRateLimit += std::fmax(std::fmin(rateTarget - output_.apRollRateLimit, kApRollRateChange * dt),
                                       -kApRollRateChange * dt);
  output_.apPhiIntegrator +=
      std::fmax(std::fmin(error, output_.apRollRateLimit * dt), -output_.apRollRateLimit * dt);
  double apLagged = output_.apPhiLag.update(output_.apPhiIntegrator, kApPhiLagC1, dt);
  double engaged = saturate(
      output_.apEngagedBlend.update(in.input.ap_engaged, kApEngageFadeUp, kApEngageFadeDown, dt, 0.0), 0.0, 1.0);
  out.apPhiDeg = (1.0 - engaged) * phi + apLagged * engaged;
  output_.apPhiIntegratorLoad = false;

  return !std::isnan(phiCmd) && !std::isnan(out.apBetaDeg) && !std::isnan(nose);
}

}  // namespace fmgc::readable::laws
