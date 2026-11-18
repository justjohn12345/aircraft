// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "fmgc_interface.h"

#include "fcu_blocks.h"

namespace fcu::readable {

namespace a429 = blocks::a429;

namespace {
bool bit(const base_arinc_429& w, int arincBit) {
  return a429::bit(w, arincBit) != 0U;
}
bool failed(const base_arinc_429& w) {
  return w.SSM == static_cast<uint32_t>(SignStatusMatrix::FailureWarning);
}
}  // namespace

bool fmgc1HasPriority(const base_fcu_discrete_inputs& d) {
  if (d.ap_1_engaged) return true;
  if (d.ap_2_engaged) return false;
  if (d.fd_1_engaged) return true;
  if (d.fd_2_engaged) return false;
  if (d.athr_1_engaged) return true;
  if (d.athr_2_engaged) return false;
  return true;
}

FmgcView readFmgcBuses(const fcu_inputs& in) {
  FmgcView v;
  v.fmgc1HasPriority = fmgc1HasPriority(in.discrete_inputs);
  const base_fmgc_a_bus& fmgc1 = in.bus_inputs.fmgc_1_bus;
  const base_fmgc_a_bus& b = v.fmgc1HasPriority ? fmgc1 : in.bus_inputs.fmgc_2_bus;

  // 471-480: aircraft data, 0 when not normal (A429ValueOrDefault, all defaults 0)
  v.data.v_cas_kts = a429::valueOr(b.cas_kts, 0.0F);
  v.data.v_mach = a429::valueOr(b.mach, 0.0F);
  v.data.hdg_deg = a429::valueOr(b.heading_deg, 0.0F);
  v.data.trk_deg = a429::valueOr(b.track_deg, 0.0F);
  v.data.alt_ft = a429::valueOr(b.altitude_ft, 0.0F);
  v.data.vs_ft_min = a429::valueOr(b.vertical_speed_ft_min, 0.0F);
  v.data.fpa_deg = a429::valueOr(b.fpa_deg, 0.0F);

  // FMGC 1 only (C6)
  v.machActive = bit(fmgc1.ats_discrete_word, 19);
  v.trkFpaReset = bit(fmgc1.discrete_word_1, 15) || bit(fmgc1.discrete_word_1, 16) || bit(fmgc1.discrete_word_7, 13);
  v.fdAutoActivateCondition = bit(fmgc1.discrete_word_1, 16) || bit(fmgc1.discrete_word_7, 13);
  v.fdAutoDeactivateCondition = bit(fmgc1.discrete_word_4, 27);

  // speed: discrete word 5
  const base_arinc_429& dw5 = b.discrete_word_5;
  const bool dw5Normal = a429::isNormal(dw5);
  v.autoSpeedControl = bit(dw5, 19) && dw5Normal;
  v.selectedSpeedControl = (dw5Normal && bit(dw5, 20)) || failed(dw5);
  if (bit(dw5, 28)) {
    v.presetSpeedOrMach = v.machActive ? a429::valueOr(b.preset_mach_from_mcdu, 0.0F)
                                       : a429::valueOr(b.preset_speed_from_mcdu_kts, 0.0F);
  } else {
    v.presetSpeedOrMach = 0.0F;  // Constant_Value
  }

  const base_arinc_429& dw1 = b.discrete_word_1;
  const base_arinc_429& dw2 = b.discrete_word_2;
  const base_arinc_429& dw3 = b.discrete_word_3;
  const base_arinc_429& dw4 = b.discrete_word_4;
  const bool dw1Normal = a429::isNormal(dw1);
  const bool dw2Normal = a429::isNormal(dw2);
  const bool dw3Normal = a429::isNormal(dw3);
  const bool dw4Normal = a429::isNormal(dw4);
  const bool landActive = bit(dw4, 14);

  // lateral
  v.lateralManagedArmed = (bit(dw3, 14) || bit(dw3, 16)) && dw3Normal;
  v.lateralManagedActive = (bit(dw2, 11) || bit(dw2, 12) || bit(dw2, 13) || bit(dw2, 14) || bit(dw2, 15) || landActive) &&
                           dw2Normal;
  v.hdgTrkSelected = dw2Normal && (bit(dw2, 16) || bit(dw2, 17));
  v.hdgTrkPresetAvailable = dw3Normal && bit(dw3, 11);

  // vertical
  v.verticalManagedArmed = (bit(dw3, 22) || bit(dw3, 23) || bit(dw3, 24) || bit(dw3, 25)) && dw3Normal;
  const bool altCstActive = a429::isNormal(b.fm_alt_constraint_ft) && bit(dw1, 19);
  const bool managedClimbDescent = (bit(dw1, 11) || bit(dw1, 12)) && !bit(dw1, 13) && !bit(dw1, 14) && !bit(dw1, 24);
  v.verticalManagedActive = altCstActive || managedClimbDescent || bit(dw1, 15) || bit(dw1, 16) || bit(dw1, 22) ||
                            bit(dw1, 23) || landActive;
  v.word1Normal = dw1Normal;
  v.vsOrFpaMode = bit(dw1, 17) || bit(dw1, 18);
  v.lvlChManaged = v.verticalManagedArmed || (v.verticalManagedActive && dw1Normal);
  v.lvlChVsFpa = dw1Normal && v.vsOrFpaMode;
  v.expedActive = bit(dw1, 24) && dw1Normal;

  // LOC / APPR lights
  const bool locArmedOrActive = bit(dw2, 13) || bit(dw2, 14) || bit(dw3, 16);
  v.landArmedOrActive = (landActive || bit(dw3, 20)) && dw4Normal && dw3Normal;
  v.locOnlyActive = dw2Normal && locArmedOrActive && dw3Normal && !v.landArmedOrActive;
  v.apprActive = v.landArmedOrActive || (dw3Normal && (bit(dw3, 23) || bit(dw1, 23)) && dw1Normal);

  // TCAS
  const base_arinc_429& dw7 = b.discrete_word_7;
  v.tcasActive = bit(dw7, 13);
  v.tcasRaInhibited = bit(dw7, 24);
  v.tcasRaCorrective = bit(dw7, 27);
  v.tcasVsTargetUp = bit(dw7, 28);
  return v;
}

}  // namespace fcu::readable
