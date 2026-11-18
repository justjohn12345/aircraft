// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's outputs (generated FacComputer.cpp 1220-1563): the FAC bus to the FMGCs, the PFD, the opposite FAC and the
// ELACs/SECs, the discretes and analog orders to the rudder actuators, and the logic / flight envelope / laws copies
// of fac_outputs. Stateless.
//
// FAC bus (SSM: NO = NormalOperation, NCD = NoComputedData; ARINC bits 1-based):
//   discrete word 1        NO when the own SFCC is valid, else NCD; the selected SFCC actual position word's value
//   gamma_a, gamma_t       NO
//   total weight, CG       NO, constant 0 (the FMGCs relay these as their "FAC weight/CG")
//   sideslip target        NCD always (the beta target is never displayed)
//   slat / flap angle      as discrete word 1; the selected FPPU angles
//   discrete word 2        NO; bit 11 yaw damper engaged, 12 opposite yaw damper engaged, 13 rudder trim engaged,
//                          14 opposite, 15 rudder travel limiter engaged, 16 opposite
//   rudder travel limit    NO; the measured limiter position (not the order)
//   yaw damper order       NO, constant 0
//   estimated sideslip     NO
//   V alpha lim, VLS,      NO from 10 s after lift-off, else NCD
//   V alpha prot, VSW
//   VS, speed trend, VMAX  NO
//   F (V3), S (V4), green  NO when shown (10 s after lift-off and in the configurations they belong to), else NCD
//   dot (VMAN), VFE next
//   discrete word 3        NO; bit 11 low energy warning, 15 pitch-pitch warning
//   discrete word 4        NO, no bit set
//   discrete word 5        NO; bits 11-15 CONF 1, 1+F, 2, 3, FULL; 16 own LGCIU valid; 17 all LGCIU lost; 18/19 left /
//                          right main gear pressed; 20 main gear down; 21 own SFCC valid; 22 all SFCC lost; 29 alpha
//                          floor
//   rudder trim order      NO; the trim order (also while not engaged: then the measured position)
//   rudder trim position   NO; the measured trim position

#pragma once

#include "FacComputer_types.h"
#include "characteristic_speeds.h"
#include "flight_path.h"
#include "sensors.h"
#include "yaw_functions.h"

namespace fac::readable {

/** Everything the packing needs from one frame. */
struct FrameResults {
  const fac_inputs* in;
  const Sensors* sensors;
  YawEngagement engagement;
  double yawDamperOrder, rudderTrimOrder, travelLimitOrder;
  const SpeedOutputs* speeds;
  const FlightPathOutputs* flightPath;
  double alphaFilteredDeg;
  bool alphaFloor;
};

/** Fills every field of `out` (the generated step writes them all on a running frame). */
void assembleOutputs(const FrameResults& r, fac_outputs& out);

}  // namespace fac::readable
