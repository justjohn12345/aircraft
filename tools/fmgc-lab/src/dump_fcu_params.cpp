// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Prints every parameter of the generated FcuComputer model (block name = value), one per line. The block names locate
// each library block instance in step() (docs/params-FcuComputer.txt).

#include <cstdio>

// The parameter struct is a private static member.
#define private public
#include "FcuComputer.h"
#undef private

#include "fcu_params.gen.h"

int main() {
  dumpParams_FcuComputer(stdout, FcuComputer::FcuComputer_P);
  return 0;
}
