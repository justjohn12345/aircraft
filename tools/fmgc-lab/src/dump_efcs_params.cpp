// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Prints every parameter of the generated ELAC, SEC and control law models (block name = value), one per line, each
// model after a "# Model" header line. The block names locate each library block instance in step()
// (docs/params-ElacComputer.txt, docs/params-SecComputer.txt, docs/params-EfcsLaws.txt).
// Usage: efcs-dump-params [elac|sec|laws]   (default: all)

#include <cstdio>
#include <cstring>

// The parameter structs are private static members.
#define private public
#include "ElacComputer.h"
#include "SecComputer.h"
#undef private

#include "efcs_params.gen.h"

int main(int argc, char** argv) {
  const char* which = argc > 1 ? argv[1] : "all";
  const bool all = std::strcmp(which, "all") == 0;
  if (all || std::strcmp(which, "elac") == 0) {
    std::printf("# ElacComputer\n");
    dumpParams_ElacComputer(stdout, ElacComputer::ElacComputer_P);
  }
  if (all || std::strcmp(which, "sec") == 0) {
    std::printf("# SecComputer\n");
    dumpParams_SecComputer(stdout, SecComputer::SecComputer_P);
  }
  if (all || std::strcmp(which, "laws") == 0) {
    std::printf("# PitchNormalLaw\n");
    dumpParams_PitchNormalLaw(stdout, PitchNormalLaw::PitchNormalLaw_rtP);
    std::printf("# PitchAlternateLaw\n");
    dumpParams_PitchAlternateLaw(stdout, PitchAlternateLaw::PitchAlternateLaw_rtP);
    std::printf("# PitchDirectLaw\n");
    dumpParams_PitchDirectLaw(stdout, PitchDirectLaw::PitchDirectLaw_rtP);
    std::printf("# LateralNormalLaw\n");
    dumpParams_LateralNormalLaw(stdout, LateralNormalLaw::LateralNormalLaw_rtP);
    std::printf("# LateralDirectLaw\n");
    dumpParams_LateralDirectLaw(stdout, LateralDirectLaw::LateralDirectLaw_rtP);
  }
  return 0;
}
