// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// What the FCU reads from the two FMGCs (generated FcuComputer.cpp 444-480, 549-737, 974-982, 1079-1089): which FMGC
// has priority, the aircraft data it shows in dashed windows, and the guidance state decoded from the FMGC discrete
// words (managed/selected, the LOC/APPR/EXPED lights, TCAS). All stateless; bit numbers are ARINC bits (1-based), as
// in readable/output_packing.cpp (the FMGC side of the same words).
//
// Quirk kept from the generated code: a few words are read from FMGC 1's bus whatever the priority (TRK/FPA reset,
// Mach mode, FD auto on/off: findings C6 and U-items in docs/fcu.md), all the others from the priority FMGC.

#pragma once

#include "FcuComputer_types.h"

namespace fcu::readable {

/**
 * FMGC priority (444-452): the FCU listens to the FMGC with the "highest" engagement, AP > FD > A/THR, FMGC 1 winning
 * ties and when nothing is engaged:
 *   AP1 -> 1;  else AP2 -> 2;  else FD1 -> 1;  else FD2 -> 2;  else A/THR1 -> 1;  else A/THR2 -> 2;  else 1.
 */
bool fmgc1HasPriority(const base_fcu_discrete_inputs& d);

/** The FMGC data and guidance state the FCU uses this frame. */
struct FmgcView {
  bool fmgc1HasPriority = true;

  /** Aircraft data from the priority FMGC (0 when a word is not normal): logic.afs.chosen_fmgc_data (471-480). */
  base_afs_fmgc_data data{};

  // ---- read from FMGC 1 only, whatever the priority
  /** ATS discrete word bit 19 (FMGC Mach control active) (574-577): the SPD/MACH window shows Mach. */
  bool machActive = false;
  /** Discrete word 1 bit 15 (SRS TO) or 16 (SRS GA), or word 7 bit 13 (TCAS): forces HDG-V/S (554-566). */
  bool trkFpaReset = false;
  /** Discrete word 1 bit 16 (SRS GA) or word 7 bit 13 (TCAS): the FD comes on (rising edge, 1079-1085). */
  bool fdAutoActivateCondition = false;
  /** Discrete word 4 bit 27 (FD auto disengage): the FD goes off (rising edge, 1086-1089). */
  bool fdAutoDeactivateCondition = false;

  // ---- speed (discrete word 5)
  /** Bit 19 (managed speed) with the word normal: logic.afs.auto_speed_control, the SPD/MACH managed dot (584-586). */
  bool autoSpeedControl = false;
  /** Bit 20 (selected speed) with the word normal, or the word failed: logic.afs.selected_speed_control (587-589). */
  bool selectedSpeedControl = false;
  /**
   * The MCDU preset speed/Mach when bit 28 (preset activate) is set: the preset Mach in Mach mode, else the preset
   * speed (each 0 when its word is not normal); 0 without bit 28 (764-773). Above 0, it replaces the SPD/MACH memory.
   */
  float presetSpeedOrMach = 0.0F;

  // ---- lateral (discrete words 2, 3, 4)
  /** Word 3 bit 14 (NAV armed) or 16 (LOC armed), word normal (596-600). */
  bool lateralManagedArmed = false;
  /** Word 2 bits 11-15 (RWY, NAV, LOC*, LOC, GA TRK) with word 2 normal, or word 4 bit 14 (LAND, raw) (607-626). */
  bool lateralManagedActive = false;
  /** Word 2 bit 16 (HDG) or 17 (TRK), word normal: logic.afs.hdg_trk_selected (627-630). */
  bool hdgTrkSelected = false;
  /** Word 3 bit 11 (HDG/TRK preset available), word normal: logic.afs.hdg_trk_preset_available (631-633). */
  bool hdgTrkPresetAvailable = false;

  // ---- vertical (discrete words 1, 3, 4, FM altitude constraint)
  /** Word 3 bits 22-25 (G/S, FINAL, CLB, DES armed), word normal (634-642). */
  bool verticalManagedArmed = false;
  /**
   * A managed vertical mode on word 1 (not checked for SSM here): ALT CST (word 1 bit 19 = ALT* or ALT with the FM
   * altitude constraint word normal), managed CLB/DES (bit 11 or 12 without bit 13 (spare), 14 (OP) or 24 (EXP)), SRS
   * TO/GA (15, 16), G/S (22), FINAL (23), or LAND (word 4 bit 14, raw) (643-678).
   */
  bool verticalManagedActive = false;
  /** Word 1 normal (679). */
  bool word1Normal = false;
  /** Word 1 bit 17 (V/S) or 18 (FPA) (680-683), SSM not checked. */
  bool vsOrFpaMode = false;
  /** logic.afs.lvl_ch_managed: armed managed vertical mode, or an active one with word 1 normal (847). */
  bool lvlChManaged = false;
  /** logic.afs.lvl_ch_vs_fpa: V/S or FPA with word 1 normal (848). */
  bool lvlChVsFpa = false;
  /** logic.afs.exped_active: word 1 bit 24 (EXP CLB/DES), word normal (690-692). */
  bool expedActive = false;
  /**
   * LAND armed (word 3 bit 20) or LAND active (word 4 bit 14), words 3 and 4 normal (719-724): APPR light, and it
   * turns the LOC light off.
   */
  bool landArmedOrActive = false;
  /** logic.afs.loc_only_active: LOC* or LOC (word 2 bits 13, 14) or LOC armed (word 3 bit 16) without LAND (699-725). */
  bool locOnlyActive = false;
  /** logic.afs.appr_active: LAND armed/active, or FINAL armed (word 3 bit 23) / active (word 1 bit 23) (726-737). */
  bool apprActive = false;

  // ---- TCAS (discrete word 7 of the priority FMGC, raw bits, 974-982)
  bool tcasActive = false;          // bit 13
  bool tcasRaInhibited = false;     // bit 24
  bool tcasRaCorrective = false;    // bit 27
  bool tcasVsTargetUp = false;      // bit 28 (TCAS V/S target > 0)
};

/** Decodes this frame's FMGC buses (stateless). */
FmgcView readFmgcBuses(const fcu_inputs& in);

}  // namespace fcu::readable
