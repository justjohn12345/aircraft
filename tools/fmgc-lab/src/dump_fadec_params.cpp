// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Prints every parameter of the generated FadecComputer model (block name = value), one per line. The block names locate
// each library block instance in step() (docs/params-FadecComputer.txt).

#include <cstdio>

// The parameter struct is a private static member.
#define private public
#include "FadecComputer.h"
#undef private

#include "fadec_params.gen.h"

int main() {
  dumpParams_FadecComputer(stdout, FadecComputer::FadecComputer_P);
  return 0;
}
