// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Landing capability: LAND 2 / LAND 3 FAIL PASSIVE / LAND 3 FAIL OP capability, inop and capacity (the autoland
// category shown on the FMA). Spec: docs/modes/engagement.md. Generated: FmgcComputer.cpp 3796–3863, outputs 4049–4054
// and 3838–3863, init 671–672; FMGC A bus discrete word 4 bits 16–25 (4742–4751).
//
// Conditioned inputs from the logic bus: at 3812–3823 raOppInvalid = logic.both_ra_valid, rtb_OR1_fl =
// logic.all_adr_valid, rtb_adrOwnInvalid = logic.all_ir_valid, rtb_ir3Invalid = logic.both_fac_rudder_valid.

#include "engagement.h"

namespace fmgc::readable::engagement {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;

constexpr float kCapabilityFreezeRaFt = 100;  // below this, LAND 3 capabilities can no longer be gained or lost

// Bits (1-based) of the opposite FMGC's A bus
constexpr int kOppWord3LandArmed = 20;          // discrete word 3: LAND armed
constexpr int kOppWord4Land = 14;               // discrete word 4: LAND
constexpr int kOppWord4Land2Capacity = 16;      //                  LAND 2 capacity
constexpr int kOppWord4Land3PassCapacity = 17;  //                  LAND 3 FAIL PASSIVE capacity
constexpr int kOppWord4Land3OpCapacity = 18;    //                  LAND 3 FAIL OP capacity

}  // namespace

void Engagement::updateLandCapability(const Stage3Inputs& in, Outputs& out) {
  const auto& di = in.in.discrete_inputs;
  const auto& oppBus = in.in.bus_inputs.fmgc_opp_bus;
  const auto& logic = in.logic;
  State& s = s_;

  // The opposite AP engaged with LAND armed or LAND on its bus; this FMGC's AP with LAND armed or LAND (this frame).
  const bool oppLandArmed = bit(oppBus.discrete_word_3, kOppWord3LandArmed) != 0 && isNormal(oppBus.discrete_word_3);
  const bool oppLand = bit(oppBus.discrete_word_4, kOppWord4Land) != 0 && isNormal(oppBus.discrete_word_4);
  const bool oppApLand = (oppLandArmed || oppLand) && di.ap_opp_engaged;
  const bool ownApLand =
      out[ApOwnEngaged] && (in.modes.armed_modes.land_armed || in.modes.lateral_modes.land_active);

  // Equipment for each category (the "inop" outputs are their complements).
  const bool land2Equipment = !out[ApInop] && (di.fwc_own_valid || di.fwc_opp_valid) && di.pfd_own_valid &&
                              di.pfd_opp_valid && logic.both_ils_valid &&
                              isNormal(logic.ra_computation_data.radio_height_ft);
  const bool land3PassiveEquipment = land2Equipment && !out[AthrInop] && logic.both_ra_valid;
  const bool land3OpEquipment = di.fwc_own_valid && di.fwc_opp_valid && di.powersupply_split && di.pfd_own_valid &&
                                di.pfd_opp_valid && !di.elac_own_ap_disc && !di.elac_opp_ap_disc &&
                                logic.both_ra_valid && logic.both_ils_valid && logic.all_adr_valid &&
                                logic.all_ir_valid && di.bscu_own_valid && di.bscu_opp_valid &&
                                logic.both_fac_rudder_valid;

  // Capability: the best category the equipment allows with this AP in LAND (LAND 3 FAIL OP also needs the opposite AP
  // in LAND, both LAND 3 need the A/THR engaged). Below 100 ft RA the LAND 3 capabilities are held as they were.
  const bool athrEngaged = out[AthrOwnEngaged];
  const bool land3Op = s.land3FailOpHeld || (land3OpEquipment && ownApLand && oppApLand && athrEngaged);
  const bool land3Passive = s.land3FailPassiveHeld || (land3PassiveEquipment && ownApLand && athrEngaged && !land3Op);
  const bool land2 = land2Equipment && ownApLand && !land3Passive && !land3Op;
  const bool frozen = logic.ra_computation_data.radio_height_ft.Data < kCapabilityFreezeRaFt && (ownApLand || oppApLand);
  s.land3FailOpHeld = frozen && land3Op;
  s.land3FailPassiveHeld = frozen && land3Passive;

  // Capacity: from the opposite FMGC's bus, unless this FMGC computes it: FMGC 1 with both APs engaged, or, without
  // both APs, when the opposite FMGC is not master.
  const auto& w4 = oppBus.discrete_word_4;
  const bool w4Normal = isNormal(w4);
  bool land2Capacity = bit(w4, kOppWord4Land2Capacity) != 0 && w4Normal;
  bool land3PassiveCapacity = bit(w4, kOppWord4Land3PassCapacity) != 0 && w4Normal;
  bool land3OpCapacity = bit(w4, kOppWord4Land3OpCapacity) != 0 && w4Normal;
  const bool ownCapacity = out[ApOwnEngaged] && di.ap_opp_engaged ? di.is_unit_1 : !out[FmgcOppPriority];
  if (ownCapacity) {
    land2Capacity = land2;
    land3PassiveCapacity = land3Passive;
    land3OpCapacity = land3Op;
  }

  cond[CondLandOwnAp] = ownApLand;
  cond[CondLandOppAp] = oppApLand;
  cond[CondLand3FailOpHeld] = s.land3FailOpHeld;
  cond[CondLand3FailPassiveHeld] = s.land3FailPassiveHeld;
  cond[CondLandCapacityOwn] = ownCapacity;
  cond[CondLandCapacityOpp] = !ownCapacity;

  out[Land2Capability] = land2;
  out[Land3FailPassiveCapability] = land3Passive;
  out[Land3FailOpCapability] = land3Op;
  out[Land2Inop] = !land2Equipment;
  out[Land3FailPassiveInop] = !land3PassiveEquipment;
  out[Land3FailOpInop] = !land3OpEquipment;
  out[Land2Capacity] = land2Capacity;
  out[Land3FailPassiveCapacity] = land3PassiveCapacity;
  out[Land3FailOpCapacity] = land3OpCapacity;
}

}  // namespace fmgc::readable::engagement
