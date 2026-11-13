// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Prints every parameter of the generated FmgcComputer and FmgcOuterLoops models (block name = value), one per line,
// prefixed with the model name. The block names locate each library block instance in step().

#include <cstdio>
#include <cstring>
#include <string>

// The parameter structs are private static members.
#define private public
#include "FmgcComputer.h"
#include "FmgcOuterLoops.h"
#undef private

#include "params.gen.h"

int main(int argc, char** argv) {
  const char* which = argc > 1 ? argv[1] : "all";
  if (!std::strcmp(which, "all") || !std::strcmp(which, "FmgcComputer")) {
    dumpParams_FmgcComputer(stdout, FmgcComputer::FmgcComputer_P);
  }
  if (!std::strcmp(which, "all") || !std::strcmp(which, "FmgcOuterLoops")) {
    dumpParams_FmgcOuterLoops(stdout, FmgcOuterLoops::FmgcOuterLoops_rtP);
  }
  return 0;
}
