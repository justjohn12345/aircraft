// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "FcuShadow.h"

#include "FcuOutputLeaves.gen.h"
// The readable FCU, compiled from tools/fmgc-lab/readable/fcu (include path: tools/fmgc-lab). Qualified with its folder
// so it can never resolve to src/fcu/Fcu.h on a case-insensitive file system.
#include "readable/fcu/fcu.h"

FcuShadow::FcuShadow()
    : readable_(std::make_unique<fcu::readable::FcuComputer>()),
      checker_(1, fcu_shadow::kOutputLeaves, fcu_shadow::kOutputLeafCount, sizeof(fcu_outputs), /*hasSimData*/ true) {
  readable_->initialize();
}

FcuShadow::~FcuShadow() = default;

void FcuShadow::step(const fcu_inputs& in, const fcu_outputs& generated) {
  const fcu_outputs& readable = readable_->step(in);
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

std::unique_ptr<shadow::ShadowLog> FcuShadow::makeLog(const std::string& directory) {
  return std::make_unique<shadow::ShadowLog>(directory, "FCU", ".fcu-shadow.log",
                                             "readable FCU (tools/fmgc-lab/readable/fcu) vs generated FcuComputer");
}
