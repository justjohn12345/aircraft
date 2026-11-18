// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The ECU bus of one FADEC (generated FadecComputer.cpp 325-414): stateless, every word SSM normal operation except
// the flex temperature (NCD when FLX is not active). Values are the doubles of this frame rounded to float.
//
//   selected TLA            TLA (deg)
//   N1 ref                  N1 for the lever position (N1_TLA)
//   selected flex temp      the FCU flex temperature word's value when normal, else 0; SSM NO when FLX active
//   ECU status words 1, 2   0
//   ECU status word 3       15 thrust limit REVERSE, 19 A/THR control active, 21 alpha floor (ATS bit 23),
//                           22 TOGA, 23 FLEX, 24 MCT, 25 CLB (thrust limit type)
//   N1 limit                thrust limit (%)
//   N1 maximum              TOGA limit (%)
//   N1 command              N1_c
//   selected N2 actual      engine_N2_percent (the sim wrapper sends 0)
//   selected N1 actual      engine_N1_percent
//   ECU maintenance word 6  12 thrust lock (read by the PFD FMA: THR LK)

#pragma once

#include "FadecComputer_types.h"
#include "thrust_lever.h"

namespace fadec::readable {

struct EcuBusInputs {
  double n1ForLever;
  bool flexActive;
  ThrustLimit limit;
  bool controlActive;
  bool alphaFloor;
  double n1Command;
  bool thrustLock;
};

base_ecu_bus packEcuBus(const athr_in& in, const EcuBusInputs& v);

}  // namespace fadec::readable
