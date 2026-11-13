// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Output stage of the vertical laws: picks the active law's FD and AP commands and turns them into the FD and AP pitch
// commands. Generated: FmgcOuterLoops.cpp 4095–4167 (FD) and 4484–4559 (AP).

#include <initializer_list>

#include "laws.h"

namespace fmgc::readable::laws {

namespace {

constexpr double kMaxPitchCommandDeg = 20;   // theta command limited to ±20°
constexpr double kFdRateLimitDegS = 15;      // FD command rate limit
constexpr double kFdThetaWashoutC1 = 1;      // rad/s
constexpr double kFdLagC1 = 2;               // rad/s
constexpr double kApLagC1 = 2;               // rad/s
constexpr double kApMaxPitchRateG = 0.6;     // AP theta command moves at most 0.6 g equivalent (g/V·n rad/s)
constexpr double kApEngageFadeUp = 1;        // AP command fades in over 1 s ...
constexpr double kApEngageFadeDown = -10;    // ... and out over 0.1 s

constexpr double kKnotsToMs = 0.51444444444444448;
constexpr double kRadToDeg = 57.295779513082323;

}  // namespace

const char* lawName(int law) {
  static const char* names[kLawCount] = {"none", "alt_hold", "alt_acq", "spd_mach", "vs",
                                         "fpa",  "gs",       "flare",   "srs",      "vpath"};
  return law >= 0 && law < kLawCount ? names[law] : "?";
}

void VerticalLaws::reset() {
  resetAlt();
  resetVsFpa();
  resetSpeed();
  resetPath();
  output_.fdThetaWashout.reset();
  output_.fdRateLimitedPrimed = false;
  output_.fdLag.reset();
  output_.apIntegratorLoad = true;
  output_.apLag.reset();
  output_.apEngagedBlend.reset();
}

void VerticalLaws::computeLaws(const ap_laws_output& in, LawResults& out) {
  out = LawResults{};
  computeAlt(in, out);
  computeVsFpa(in, out);
  computeSpeed(in, out);
  computePath(in, out);
  for (int law : {AltHold, AltAcq, SpdMach, Vs, Fpa, Gs, Flare, Srs, Vpath}) {
    out.rewritten[law] = true;
  }
}

bool VerticalLaws::computeOutputs(const ap_laws_output& in, const LawCommand (&cmd)[kLawCount], PitchOutputs& out) {
  const double dt = in.time.dt;
  const double theta = in.data.Theta_deg;
  int law = static_cast<int>(in.input.vertical_law);
  bool knownLaw = law >= 0 && law < kLawCount;

  // ---- flight director: limited command, minus washed-out theta, rate limited and lagged (sign: bar deflection)
  double fdCmd = !knownLaw || law == None ? 0.0 : cmd[law].fd;
  if (!knownLaw && law != None) {
    fdCmd = cmd[Vpath].fd;  // the generated switch's default case
  }
  double fdLimited;
  if (fdCmd > kMaxPitchCommandDeg) {
    fdLimited = kMaxPitchCommandDeg;
  } else {
    fdLimited = -kMaxPitchCommandDeg;
    if (fdCmd >= fdLimited) {
      fdLimited = fdCmd;
    }
  }
  double thetaWashed = output_.fdThetaWashout.update(theta, kFdThetaWashoutC1, dt);
  if (!output_.fdRateLimitedPrimed) {
    output_.fdRateLimited = 0;
    output_.fdRateLimitedPrimed = true;
  }
  output_.fdRateLimited += std::fmax(std::fmin((fdLimited - thetaWashed) * -1.0 - output_.fdRateLimited,
                                               std::abs(kFdRateLimitDegS) * dt),
                                     -std::abs(-kFdRateLimitDegS) * dt);
  out.fdThetaDeg = output_.fdLag.update(output_.fdRateLimited, kFdLagC1, dt);

  // ---- autopilot: theta + command, limited to ±20°, followed at ≤ 0.6 g, lagged, faded in with the AP
  double apCmd = !knownLaw || law == None ? 0.0 : cmd[law].ap;
  if (!knownLaw && law != None) {
    apCmd = cmd[Vpath].ap;
  }
  double target = apCmd + theta;
  if (target > kMaxPitchCommandDeg) {
    target = kMaxPitchCommandDeg;
  } else if (target < -1.0 * kMaxPitchCommandDeg) {
    target = -1.0 * kMaxPitchCommandDeg;
  }
  bool apDisengaged = !in.input.ap_engaged;
  output_.apIntegratorLoad = apDisengaged || output_.apIntegratorLoad;
  if (output_.apIntegratorLoad) {
    output_.apIntegrator = theta;
  }
  double gOverV = 9.81 / (in.data.V_tas_kn * kKnotsToMs);
  double maxRate = gOverV * kApMaxPitchRateG * kRadToDeg;
  double step = std::fmin(target - output_.apIntegrator, maxRate * dt);
  output_.apIntegrator += std::fmax(step, -1.0 * maxRate * dt);
  double apLagged = output_.apLag.update(output_.apIntegrator, kApLagC1, dt);
  double engaged = output_.apEngagedBlend.update(in.input.ap_engaged, kApEngageFadeUp, kApEngageFadeDown, dt, 0.0);
  engaged = engaged > 1.0 ? 1.0 : (engaged < 0.0 ? 0.0 : engaged);
  out.apThetaDeg = (1.0 - engaged) * theta + apLagged * engaged;
  output_.apIntegratorLoad = false;

  return !std::isnan(fdCmd) && !std::isnan(apCmd);
}

}  // namespace fmgc::readable::laws
