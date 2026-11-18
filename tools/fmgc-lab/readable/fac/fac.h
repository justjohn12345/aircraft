// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable FAC (Flight Augmentation Computer): a rewrite of the generated A32NX FacComputer (fbw-a32nx/src/wasm/
// fbw_a320/src/model/FacComputer.cpp, 1587 lines) that reproduces it bit for bit, frame by frame (checked
// free-running by src/check_fac.cpp). Stock behaviour, quirks included; fixes never go here. Spec and findings:
// docs/fac.md.
//
// Two instances (FlyByWireInterface::updateFac, after the FMGCs and ELACs; FAC 1 first). Interface as the generated
// model's: initialize() once, then step() every frame. computer_running false (the wrapper's FAC health) leaves the
// outputs as they were; the next running frame restarts every function (filters, latches, rate limiters from their
// initial values), the rudder trim integrator excepted (it is reloaded instead).
//
// Pipeline of one frame:
//   sensors               landing gear, slats/flaps, ADR, IR, radio height and FMGC selection, on ground  392-670
//   yaw_functions         engagement of the yaw damper, rudder trim and travel limiter                    671-689
//   flight_path           estimated sideslip and the engine-out sideslip target                           690-724
//   alpha_floor           filtered alpha, alpha floor threshold, flight phase, alpha floor latch           727-830
//   characteristic_speeds V alpha max/prot, VSW, VS, VLS, F, S, green dot, VMAX, VFE next, trend          831-975
//   flight_path           gamma_a, gamma_t, pitch-pitch and low energy warnings                           1031-1100
//   yaw_functions         rudder trim, yaw damper, travel limiter orders                                  1102-1222
//   output_packing        FAC bus, discretes, analog orders, logic and flight envelope copies             1223-1586
//
// No heap allocation, no static mutable state: copyable, several instances can run side by side.

#pragma once

#include "FacComputer_types.h"
#include "alpha_floor.h"
#include "characteristic_speeds.h"
#include "flight_path.h"
#include "sensors.h"
#include "yaw_functions.h"

namespace fac::readable {

class FacComputer {
 public:
  /** Power-up: every memory to its initial value, outputs to zero (a complete reset of a used instance too). */
  void initialize();

  /** One frame. Returns the outputs (FacComputer_Y.out). */
  const fac_outputs& step(const fac_inputs& in);

  const fac_outputs& outputs() const { return out_; }

  // state access for the checker
  bool running() const { return running_; }
  const InputConditioning& sensors() const { return sensors_; }
  const AlphaFloor& alphaFloor() const { return alphaFloor_; }
  const CharacteristicSpeeds& speeds() const { return speeds_; }
  const FlightPath& flightPath() const { return flightPath_; }
  const RudderTrim& rudderTrim() const { return rudderTrim_; }
  const YawDamper& yawDamper() const { return yawDamper_; }
  const TravelLimiter& travelLimiter() const { return travelLimiter_; }

 private:
  /** The first running frame after power-up or a stop (generated Runtime_MODE block, 336-389). */
  void restart();

  bool running_ = false;  // Runtime_MODE
  InputConditioning sensors_;
  AlphaFloor alphaFloor_;
  CharacteristicSpeeds speeds_;
  FlightPath flightPath_;
  RudderTrim rudderTrim_;
  YawDamper yawDamper_;
  TravelLimiter travelLimiter_;
  fac_inputs in_{};
  fac_outputs out_{};
};

}  // namespace fac::readable
