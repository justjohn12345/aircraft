// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "FadecShadow.h"

#include "FadecOutputLeaves.gen.h"
// The readable FADEC, compiled from tools/fmgc-lab/readable/fadec (include path: tools/fmgc-lab).
#include "readable/fadec/fadec.h"

FadecShadow::FadecShadow(int engine)
    : readable_(std::make_unique<fadec::readable::FadecComputer>()),
      checker_(engine, fadec_shadow::kOutputLeaves, fadec_shadow::kOutputLeafCount, sizeof(athr_out), /*hasSimData*/ false) {
  readable_->initialize();
}

FadecShadow::~FadecShadow() = default;

void FadecShadow::step(const athr_in& in, const athr_out& generated) {
  const athr_out& readable = readable_->step(in);
  shadow::FrameInputs frame;
  frame.dt = in.time.dt;
  frame.simulationTime = in.time.simulation_time;
  checker_.step(&generated, &readable, frame);
}

std::unique_ptr<shadow::ShadowLog> FadecShadow::makeLog(const std::string& directory) {
  return std::make_unique<shadow::ShadowLog>(directory, "FADEC", ".fadec-shadow.log",
                                             "readable FADEC (tools/fmgc-lab/readable/fadec) vs generated FadecComputer, engine 1 and 2");
}
