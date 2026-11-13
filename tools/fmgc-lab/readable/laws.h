// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC vertical control laws (FmgcOuterLoops, Phase 3). Stock behaviour: must reproduce the
// generated FmgcOuterLoops exactly (checked by src/check_laws.cpp); fixes belong on personal/fmgc-fixes.
//
// Every frame every vertical law computes its pitch command (delta theta, deg) for the flight director and for the
// autopilot, whichever law is active; the output stage then picks the active law's commands, limits and filters them
// into the FD and AP pitch commands. Inputs are the outer loops' input bus (ap_laws_output: time, data, input).

#pragma once

#include <cmath>
#include <limits>

#include "FmgcOuterLoops_types.h"
#include "blocks.h"
#include "laws_alt.h"
#include "laws_path.h"
#include "laws_speed.h"
#include "laws_vs_fpa.h"

namespace fmgc::readable::laws {

/** vertical_law enum values (FmgcComputer_types.h): NONE, ALT_HOLD, ALT_ACQ, SPD_MACH, VS, FPA, GS, FLARE, SRS, VPATH. */
enum Law : int { None = 0, AltHold, AltAcq, SpdMach, Vs, Fpa, Gs, Flare, Srs, Vpath, kLawCount };
const char* lawName(int law);

/** A law's pitch commands (delta theta, deg). NaN = not rewritten yet. */
struct LawCommand {
  double fd = std::numeric_limits<double>::quiet_NaN();
  double ap = std::numeric_limits<double>::quiet_NaN();
};

struct LawResults {
  LawCommand cmd[kLawCount];
  ap_raw_laws_flare flare{};  // flare law outputs (output.flare_law), valid when flareValid
  bool flareValid = false;
};

struct PitchOutputs {
  double fdThetaDeg;  // output.flight_director.Theta_c_deg
  double apThetaDeg;  // output.autopilot.Theta_c_deg
};

/** Output stage memories (laws_output.cpp). */
struct OutputStageState {
  blocks::WashoutFilter fdThetaWashout;
  double fdRateLimited = 0;  // FD command rate limiter (15 deg/s)
  bool fdRateLimitedPrimed = false;
  blocks::LagFilter<double> fdLag;
  double apIntegrator = 0;   // AP pitch command, moved towards theta + delta at a limited rate
  bool apIntegratorLoad = true;
  blocks::LagFilter<double> apLag;
  blocks::SwitchBlend apEngagedBlend;  // AP command faded in when the AP engages
};

class VerticalLaws {
 public:
  /** Power-up (generated init()) and computer restart (generated reset()). */
  void reset();

  /** Every law's commands for this step, plus the flare law outputs. */
  void computeLaws(const ap_laws_output& in, LawResults& out);

  /** Output stage: picks in.input.vertical_law's commands. Returns false if not rewritten yet. */
  bool computeOutputs(const ap_laws_output& in, const LawCommand (&cmd)[kLawCount], PitchOutputs& out);

 private:
  void resetAlt();
  void computeAlt(const ap_laws_output& in, LawResults& out);
  void resetVsFpa();
  void computeVsFpa(const ap_laws_output& in, LawResults& out);
  void resetSpeed();
  void computeSpeed(const ap_laws_output& in, LawResults& out);
  void resetPath();
  void computePath(const ap_laws_output& in, LawResults& out);

  AltLawState alt_;
  VsFpaLawState vsFpa_;
  SpeedLawState speed_;
  PathLawState path_;
  OutputStageState output_;
};

}  // namespace fmgc::readable::laws
