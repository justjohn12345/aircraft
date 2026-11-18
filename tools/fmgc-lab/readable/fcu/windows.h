// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The four AFS display windows of the FCU: SPD/MACH, HDG/TRK, ALT and V/S-FPA. Each one is a memory (the value shown
// and sent to the FMGCs as the selected target) that the knob changes, plus a dash state.
//
// Common pattern (generated FcuComputer.cpp 738-1060), per window, every running frame:
//   1. first frame after a (re)start: the memory takes the aircraft's current value from the priority FMGC
//   2. a value set from the sim (sim_input.*, -1 = none) replaces it
//   3. switching the window's unit (SPD<->MACH, HDG-V/S<->TRK-FPA) re-derives it
//   4. while dashed it follows the aircraft's current value (SPD/MACH and V/S-FPA: as is; HDG/TRK: rounded)
//   5. window-specific overrides (MCDU preset speed; V/S level-off on V/S push; V/S after TCAS)
//   6. the knob turns (raw count of this frame) are added, then the value is limited and rounded to its resolution
// The memories are float (real32_T); the arithmetic below is the generated arithmetic in the same order, so the values
// are bit-identical (e.g. a Mach is round(m / 0.01F) * 0.01F, which is not the nearest float to 0.78).
//
// Dash timers count simulation time (s); the button pulses and TCAS monostables count dt.

#pragma once

#include "FcuComputer_types.h"
#include "afs_controls.h"
#include "fcu_blocks.h"
#include "fmgc_interface.h"

namespace fcu::readable {

/** "No value set from the sim" in sim_input (CompareToConstant1_const*). */
constexpr float kNoSimInput = -1.0F;

// ---------------------------------------------------------------------------------------------------------------------
// SPD/MACH (738-829)
// ---------------------------------------------------------------------------------------------------------------------

/** Managed speed: the window dashes this long after the last speed selection/knob turn (s). */
constexpr double kSpdMachDashDelaySeconds = 10.0;
constexpr float kMinSpeedKts = 100.0F, kMaxSpeedKts = 399.0F;
constexpr float kMinMach = 0.1F, kMaxMach = 0.99F;
constexpr float kMachResolution = 0.01F;

class SpeedMachWindow {
 public:
  struct Output {
    float value = 0;      // logic.afs.spd_mach_display_value: kt, or Mach when the FMGC is in Mach (FmgcView::machActive)
    bool dashes = false;  // logic.afs.spd_mach_dashes
  };

  void reset();

  /**
   * Dashes: at once when the FMGC leaves selected speed (falling edge of selected_speed_control), and 10 s after the
   * last "selection" in managed speed: selected speed (or dw5 failed), a SPD knob turn or a value set from the sim.
   * Memory (pValue_p): steps 1-6 of the header with: unit switch = FMGC Mach mode change (to the current CAS or Mach);
   * while dashed = the current CAS/Mach, unrounded until step 6 (so a dashed window's memory moves with the actual
   * speed: the source of the C13 value-change pulses); step 5 = the MCDU preset speed/Mach when above 0; step 6 =
   * +1 kt or +0.01 per click, limited to 100-399 kt / M0.10-0.99, rounded to 1 kt / 0.01.
   */
  Output update(const fcu_inputs& in, const FmgcView& fmgc);

  float memory() const { return memory_; }
  double dashEventTime() const { return dashTimer_.eventTime; }

 private:
  blocks::EdgeDetector selectedSpeedLost_;  // PulseNode, falling edge (sf_MATLABFunction_kl)
  blocks::DashTimer dashTimer_;             // inline at 740-753 (eventTime)
  float memory_ = 0;                        // pValue_p
  bool memoryPrimed_ = false;
  bool previousMach_ = false;  // prevMachActive
  bool previousMachPrimed_ = false;
};

// ---------------------------------------------------------------------------------------------------------------------
// HDG/TRK (861-933)
// ---------------------------------------------------------------------------------------------------------------------

/** Managed lateral mode: the window dashes this long after the last selection/knob turn (s). */
constexpr double kHdgTrkDashDelaySeconds = 45.0;
/** On HDG<->TRK switching, a target within this of the old reference becomes the new reference (deg). */
constexpr float kHdgTrkSyncWindowDeg = 5.0F;

class HdgTrkWindow {
 public:
  struct Output {
    float value = 0;      // logic.afs.hdg_trk_display_value (deg, 0-359)
    bool dashes = false;  // logic.afs.hdg_trk_dashes
  };

  void reset();

  /**
   * Dashes: at once when the FMGC has no HDG/TRK mode with an AP/FD engaged (rising edge; also on the same condition
   * without a preset), and 45 s after the last "selection": HDG/TRK mode, no AP/FD, a knob turn, a value set from the
   * sim, or a preset available (the preset value stays shown). Memory (pValue_m): step 3 = on HDG<->TRK switching, if
   * the target is within 5 deg of the aircraft's old reference (heading when going to TRK, track when going to HDG) or
   * no AP/FD is engaged, it becomes the new reference (current track / heading), else the number is kept; step 4 = the
   * current heading/track rounded to 1 deg; step 6 = +1 deg per click, rounded, wrapped to 0-359.
   */
  Output update(const fcu_inputs& in, const FmgcView& fmgc, bool anyApFdEngaged, bool trkFpaActive);

  float memory() const { return memory_; }
  double dashEventTime() const { return dashTimer_.eventTime; }

 private:
  blocks::EdgeDetector managedWithApFd_;          // sf_MATLABFunction_du
  blocks::EdgeDetector managedNoPresetWithApFd_;  // sf_MATLABFunction_dc
  blocks::DashTimer dashTimer_;                   // sf_MATLABFunction1
  float memory_ = 0;                              // pValue_m
  bool memoryPrimed_ = false;
  bool previousTrkFpa_ = false;  // prevTrkFpaActive_m
  bool previousTrkFpaPrimed_ = false;
};

// ---------------------------------------------------------------------------------------------------------------------
// ALT (934-954)
// ---------------------------------------------------------------------------------------------------------------------

constexpr float kMinAltFt = 100.0F, kMaxAltFt = 49000.0F;

class AltWindow {
 public:
  void reset();

  /**
   * Memory (pValue_e), in ft (the metric display is the DMC's business). Step 1: the current altitude rounded to the
   * knob increment (100 or 1000 ft, alt_increment_1000). Step 2: a value set from the sim. Step 6, when the sim set a
   * value or the knob turned: value' = round((value + turns * (increment/2 + 1)) / increment) * increment, i.e. one
   * click moves to the next multiple of the increment (so a 1000 ft click from 5300 gives 6000 / 5000), then limited
   * to 100-49000 ft. Never dashed; no unit switch.
   */
  float update(const fcu_inputs& in, const FmgcView& fmgc);

  float memory() const { return memory_; }

 private:
  float memory_ = 0;  // pValue_e
  bool memoryPrimed_ = false;
};

// ---------------------------------------------------------------------------------------------------------------------
// V/S-FPA (955-1060)
// ---------------------------------------------------------------------------------------------------------------------

/** Not in V/S-FPA: the window dashes this long after the last knob turn (s). */
constexpr double kVsFpaDashDelaySeconds = 45.0;
/** V/S knob push to level off: V/S-FPA mode must engage within this of the push (s, MTrigNode_triggerDuration_k). */
constexpr double kLevelOffPushWindowSeconds = 1.0;
/** End of TCAS mode: V/S-FPA must engage within this of it for the post-TCAS target (s, MTrigNode1_triggerDuration_g). */
constexpr double kTcasEndWindowSeconds = 0.5;
constexpr float kMaxVsFtMin = 6000.0F, kVsResolutionFtMin = 100.0F;
constexpr float kMaxFpaDeg = 9.9F, kFpaResolutionDeg = 0.1F;
/** After a corrective TCAS RA: V/S towards the FCU altitude, 1000 ft/min, or 500 ft/min climbing above FL300. */
constexpr float kPostTcasVsFtMin = 1000.0F, kPostTcasClimbVsHighFtMin = 500.0F, kPostTcasHighAltFt = 30000.0F;

class VsFpaWindow {
 public:
  struct Output {
    float value = 0;      // logic.afs.vs_fpa_display_value: ft/min, or deg in TRK-FPA
    bool dashes = false;  // logic.afs.vs_fpa_dashes
  };

  void reset();

  /**
   * Dashes: at once when the FMGC leaves V/S-FPA with an AP/FD engaged (rising edge of "not V/S-FPA and AP/FD"), and
   * 45 s after the last "selection": V/S-FPA mode, no AP/FD, a knob turn or a value set from the sim.
   * Memory (pValue): step 3 = on HDG-V/S<->TRK-FPA switching, the current V/S or FPA; step 2 = the sim value (/10 in
   * FPA); step 4 = also on an AP engagement (rising edge of AP1 or AP2): the current V/S or FPA, unrounded until step
   * 6; step 5 = 0 on a level-off push (V/S-FPA engages within 1 s of a V/S knob push), else after TCAS (V/S-FPA
   * engages within 0.5 s of the end of TCAS mode): with "RA inhibited" (word 7 bit 24) the current V/S if the TCAS
   * target was up (bit 28), else 0; with "RA corrective" (bit 27) 1000 ft/min towards the FCU altitude, 500 ft/min
   * when climbing above 30000 ft; step 6 = +100 ft/min or +0.1 deg per click, limited to +-6000 ft/min / +-9.9 deg,
   * rounded to 100 ft/min / 0.1 deg.
   */
  Output update(const fcu_inputs& in, const FmgcView& fmgc, bool anyApFdEngaged, bool trkFpaActive,
                bool vsKnobPushPulse, float fcuAltFt);

  float memory() const { return memory_; }
  double dashEventTime() const { return dashTimer_.eventTime; }

 private:
  blocks::EdgeDetector leftVsFpaWithApFd_;    // sf_MATLABFunction_kw
  blocks::DashTimer dashTimer_;               // sf_MATLABFunction1_o
  blocks::Monostable recentVsPush_;           // sf_MATLABFunction_hh
  blocks::EdgeDetector levelOffPush_;         // sf_MATLABFunction_ar
  blocks::Monostable recentTcasEnd_;          // sf_MATLABFunction_ey (falling edge)
  blocks::EdgeDetector vsFpaAfterTcas_;       // sf_MATLABFunction_df
  blocks::EdgeDetector apEngagement_;         // sf_MATLABFunction_ch
  float memory_ = 0;                          // pValue
  bool memoryPrimed_ = false;
  bool previousTrkFpa_ = false;  // prevTrkFpaActive
  bool previousTrkFpaPrimed_ = false;
};

}  // namespace fcu::readable
