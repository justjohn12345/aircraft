// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's input conditioning (generated FacComputer.cpp 392-718): which landing gear, slat/flap, air data, inertial,
// radio height and FMGC source the FAC uses, and the failure flags of the logic bus.
//
// Sources and fallbacks (ARINC bits 1-based):
//   landing gear   own LGCIU (discrete word 1 not failed, and its nose gear bit agreeing with the nose gear discrete:
//                  disagreement for 1 s invalidates it), else the opposite FAC's copy (its discrete word 5 bits 16-20),
//                  else "not pressed, gear not down"
//   slats / flaps  own SFCC (system status word not failed, and the slat-extended discrete disagreeing with the actual
//                  position word bit 12 as wired: agreement for 1 s invalidates it), else the opposite FAC's copy, else
//                  with the gear down CONF FULL at full flap/slat angles, else CONF 0
//   air data       own ADR unless its CAS or AoA word fails, else ADR 3, else zeros (the opposite ADR is never used)
//   inertial data  own IR unless its yaw rate or lateral acceleration word is not normal, else IR 3, else zeros
//   radio height   the own FMGC's word unless it has been failed for 0.6 s, else the opposite FMGC's
//   FMGC words     the own FMGC's discrete/ATS words when its AP is engaged, or with no AP engaged when its FM weight
//                  and CG words are normal; else the opposite FMGC's

#pragma once

#include "FacComputer_types.h"
#include "fac_blocks.h"

namespace fac::readable {

/** Nose gear agreement between the LGCIU word and the discrete before the own LGCIU is declared invalid (s). */
constexpr double kLgciuDisagreeSeconds = 1.0;
/** Agreement of the slat-extended discrete with the position word bit before the own SFCC is invalid (s). */
constexpr double kSfccDisagreeSeconds = 1.0;
/** Main gear compressed for this long before "on ground" (s). */
constexpr double kOnGroundConfirmSeconds = 0.5;
/** An FMGC radio height word failed for this long before the other FMGC's is used (s). */
constexpr double kRaFailConfirmSeconds = 0.6;
/** Slat/flap fallback with both SFCCs lost and the gear down: CONF FULL, flap and slat FPPU angles at full travel. */
constexpr float kGearDownConfigIndex = 5.0F;
constexpr float kGearDownFlapAngle = 251.97F;
constexpr float kGearDownSlatAngle = 334.16F;

/** Landing gear state as the FAC sees it (logic bus fields). */
struct GearState {
  bool lgciuOwnValid = false;
  bool allLgciuLost = false;  // neither the own LGCIU nor the opposite FAC's copy
  bool leftMainPressed = false;
  bool rightMainPressed = false;
  bool mainGearOut = false;  // downlocked (either of the two LGCIU bits)
};

/** Slats and flaps as the FAC sees them. The configuration index: 0 CONF 0, 1 CONF 1, 2 CONF 1+F, 3 CONF 2, 4 CONF 3,
 *  5 CONF FULL. Angles are the SFCC feedback position pick-off unit (FPPU) angles. */
struct SlatFlapState {
  bool sfccOwnValid = false;
  bool allSfccLost = false;
  float configIndex = 0.0F;
  float flapFppuDeg = 0.0F;
  float slatFppuDeg = 0.0F;
  float actualPositionWord = 0.0F;  // raw value of the SFCC actual position word (sent on as FAC discrete word 1)
  double flapSurfaceDeg = 0.0;
  double slatSurfaceDeg = 0.0;
};

/** The selected ADR's values (zero without one) and the failure flags. */
struct AirData {
  float casKn = 0.0F, tasKn = 0.0F, mach = 0.0F, alphaDeg = 0.0F, staticPressureHpa = 0.0F, altitudeFt = 0.0F;
  bool ownInvalid = false, oppInvalid = false, adr3Invalid = false;
};

/** The selected IR's values (zero without one) and the failure flags. */
struct InertialData {
  float thetaDeg = 0.0F, phiDeg = 0.0F, qDegS = 0.0F, rDegS = 0.0F, nxG = 0.0F, nyG = 0.0F, nzG = 0.0F,
        thetaDotDegS = 0.0F, phiDotDegS = 0.0F, groundSpeedKn = 0.0F, verticalSpeed = 0.0F;
  bool ownInvalid = false, oppInvalid = false, ir3Invalid = false;
};

/** Everything the functions read from the conditioning, one frame. */
struct Sensors {
  GearState gear;
  SlatFlapState slatFlap;
  AirData adr;
  InertialData ir;
  bool onGround = false;
  base_arinc_429 radioHeight{};  // the selected FMGC's radio height word
  bool allRaFailed = false;
  bool anyApEngaged = false;
  bool fmgcOwnSelected = false;
  const base_fmgc_b_bus* fmgc = nullptr;  // the selected FMGC's bus (discrete word 1, ATS word)
};

class InputConditioning {
 public:
  void reset();

  /** One frame (generated order: landing gear, slats/flaps, ADR, IR, RA, FMGC selection, on ground). */
  const Sensors& update(const fac_inputs& in);

  // state access for the checker
  const blocks::ConfirmNode& noseGearAgreement() const { return noseGearAgree_; }
  const blocks::ConfirmNode& sfccAgreement() const { return sfccAgree_; }
  const blocks::ConfirmNode& raOwnValid() const { return raOwn_; }
  const blocks::ConfirmNode& raOppValid() const { return raOpp_; }
  const blocks::ConfirmNode& onGroundConfirm() const { return onGround_; }

 private:
  void updateGear(const fac_inputs& in);
  void updateSlatFlap(const fac_inputs& in);
  void updateAirData(const base_fac_bus_inputs& b);
  void updateInertialData(const base_fac_bus_inputs& b);

  blocks::ConfirmNode noseGearAgree_;  // sf_MATLABFunction_ax
  blocks::ConfirmNode sfccAgree_;      // sf_MATLABFunction_jf
  blocks::ConfirmNode raOwn_;          // sf_MATLABFunction_cu
  blocks::ConfirmNode raOpp_;          // sf_MATLABFunction_md
  blocks::ConfirmNode onGround_;       // sf_MATLABFunction_le
  Sensors s_;
};

}  // namespace fac::readable
