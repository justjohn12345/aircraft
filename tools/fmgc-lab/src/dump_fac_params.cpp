// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Prints every parameter of the generated FacComputer model (block name = value), one per line. The block names locate
// each library block instance in step() (docs/params-FacComputer.txt).

#include <cstdio>

// The parameter struct is a private static member.
#define private public
#include "FacComputer.h"
#undef private

#include "fac_params.gen.h"

int main() {
  dumpParams_FacComputer(stdout, FacComputer::FacComputer_P);
  return 0;
}
