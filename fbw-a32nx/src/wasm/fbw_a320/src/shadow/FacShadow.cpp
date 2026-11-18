// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "FacShadow.h"

#include "FacOutputLeaves.gen.h"
// The readable FAC, compiled from tools/fmgc-lab/readable/fac (include path: tools/fmgc-lab). Qualified with its folder
// so it can never resolve to src/fac/Fac.h on a case-insensitive file system.
#include "readable/fac/fac.h"

FacShadow::FacShadow(int unit)
    : readable_(std::make_unique<fac::readable::FacComputer>()),
      checker_(unit, fac_shadow::kOutputLeaves, fac_shadow::kOutputLeafCount, sizeof(fac_outputs), /*hasSimData*/ true,
               /*nanEqual*/ true) {
  readable_->initialize();
}

FacShadow::~FacShadow() = default;

void FacShadow::step(const fac_inputs& in, const fac_outputs& generated) {
  const fac_outputs& readable = readable_->step(in);
  shadow::FrameInputs frame;
  frame.dt = in.time.dt;
  frame.simulationTime = in.time.simulation_time;
  frame.monotonicTime = in.time.monotonic_time;
  frame.slewOn = in.sim_data.slew_on != 0;
  frame.pauseOn = in.sim_data.pause_on != 0;
  frame.trackingModeOnOverride = in.sim_data.tracking_mode_on_override != 0;
  frame.tailstrikeProtectionOn = in.sim_data.tailstrike_protection_on != 0;
  frame.computerRunning = in.sim_data.computer_running != 0;
  checker_.step(&generated, &readable, frame);
}

std::unique_ptr<shadow::ShadowLog> FacShadow::makeLog(const std::string& directory) {
  return std::make_unique<shadow::ShadowLog>(
      directory, "FAC", ".fac-shadow.log",
      "readable FAC (tools/fmgc-lab/readable/fac) vs generated FacComputer, FAC 1 and 2, two NaNs equal");
}
