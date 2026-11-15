// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "fmgc.h"

#include "law_selection.h"
#include "output_packing.h"

namespace fmgc::readable {

namespace {

constexpr int idx(Signal s) {
  return static_cast<int>(s);
}

/** The mode signals as ap_fd_logic fields (generated bus assignment 3972–4054, latches in docs/latches.tsv). */
void copyModes(const Signals& s, base_fmgc_ap_fd_logic_outputs& a) {
  auto& lat = a.lateral_modes;
  lat.rwy_active = s[Signal::RwyActive];
  lat.nav_active = s[Signal::Nav];
  lat.loc_cpt_active = s[Signal::LocCapture];
  lat.loc_trk_active = s[Signal::LocTrack];
  lat.roll_goaround_active = s[Signal::GaTrk];
  lat.hdg_active = s[Signal::HdgActive];
  lat.trk_active = s[Signal::TrkActive];
  lat.rwy_loc_submode_active = s[Signal::RwyLoc];
  lat.rwy_trk_submode_active = s[Signal::RwyTrk];
  lat.land_active = s[Signal::Land];
  lat.align_submode_active = s[Signal::Align];
  lat.rollout_submode_active = s[Signal::RollOut];

  auto& lon = a.longitudinal_modes;
  lon.clb_active = s[Signal::Clb];
  lon.des_active = s[Signal::Des];
  lon.op_clb_active = s[Signal::OpClb];
  lon.op_des_active = s[Signal::OpDes];
  lon.exp_clb_active = s[Signal::ExpClb];
  lon.exp_des_active = s[Signal::ExpDes];
  lon.pitch_takeoff_active = s[Signal::SrsTakeoff];
  lon.pitch_goaround_active = s[Signal::SrsGoAround];
  lon.vs_active = s[Signal::VsActive];
  lon.fpa_active = s[Signal::FpaActive];
  lon.alt_acq_active = s[Signal::AltCapture];
  lon.alt_hold_active = s[Signal::AltHold];
  lon.fma_dash_display = s[Signal::AltCruise];
  lon.gs_capt_active = s[Signal::GsCapture];
  lon.gs_trk_active = s[Signal::GsTrack];
  lon.final_des_active = s[Signal::Final];
  lon.flare_active = s[Signal::Flare];
  lon.cruise_active = false;  // never set (constant of the output structure)
  lon.tcas_active = s[Signal::Tcas];

  auto& arm = a.armed_modes;
  arm.alt_acq_armed = s[Signal::AltArmed];
  arm.alt_acq_arm_possible = s[Signal::AltArmPossible];
  arm.nav_armed = s[Signal::NavArmed];
  arm.loc_armed = s[Signal::LocArmed];
  arm.land_armed = s[Signal::LandArmed];
  arm.glide_armed = s[Signal::GlideArmed];
  arm.final_des_armed = s[Signal::FinalArmed];
  arm.clb_armed = s[Signal::ClbArmed];
  arm.des_armed = s[Signal::DesArmed];
  arm.tcas_armed = s[Signal::TcasArmed];

  a.alt_cstr_applicable = s[Signal::AltCstrApplicable];
  a.alt_sel_or_cstr = s.altSelOrCstr;
  a.lateral_mode_reversion = s[Signal::LateralReversion];
  a.longitudinal_mode_reversion_vs = s[Signal::ReversionVs];
  a.longitudinal_mode_reversion_op_clb = s[Signal::ReversionOpClb];
  a.active_tcas_submode = tcas::activeSubmode(s[Signal::TcasAltAcq], s[Signal::TcasAltHold]);
}

}  // namespace

void FmgcComputer::initialize() {
  conditioning_.initialize();
  engagement_.reset();
  tcas_.reset();
  modes_ = VerticalModeLogic{};
  modes_.reset(signals_);
  targets_.initialize(targetValues_);
  lateralLaws_ = laws::LateralLaws{};
  lateralLaws_.reset();
  verticalLaws_ = laws::VerticalLaws{};
  verticalLaws_.reset();
  outerLoopInputs_ = OuterLoopInputState{};
  athr_.reset();
  lastApFd_ = {};
  lastAthr_ = {};
  lastOuterLoops_ = {};
  running_ = false;
  firstFrame_ = true;
  bus_ = Buses{};
  out_ = fmgc_outputs{};
}

void FmgcComputer::restart() {
  conditioning_.restart();
  engagement_.reset();
  tcas_.reset();
  modes_.reset(signals_);
  targets_.reset(targetValues_);
  lateralLaws_.reset();
  verticalLaws_.reset();
  athr_.reset();
  lastApFd_ = {};
  lastAthr_ = {};
  lastOuterLoops_ = {};
  firstFrame_ = true;
}

const fmgc_outputs& FmgcComputer::step(const fmgc_inputs& in) {
  if (!in.sim_data.computer_running) {
    running_ = false;  // outputs held; the next running frame restarts the computer
    return out_;
  }
  if (!running_) {
    restart();
    running_ = true;
  }
  const double dt = in.time.dt;
  base_fmgc_logic_outputs& logic = bus_.logic;
  base_fmgc_ap_fd_logic_outputs& apFd = bus_.apFd;
  apFd = {};

  // ---- 1. Input conditioning (695–1379): the logic bus except the engagement fields.
  conditioning_.update({in, lastApFd_}, logic);

  // ---- 2. Engagement stage 1 (1380–1559): AP / FD / A/THR engaged, opposite-FMGC sync, mode reset.
  engagement::Outputs eng;
  engagement_.updateEngagement({dt, in, logic, lastApFd_}, eng);
  logic.ap_fd_athr_common_condition = eng[engagement::ApFdAthrCommonCondition];
  logic.ap_fd_common_condition = eng[engagement::ApFdCommonCondition];
  logic.fd_own_engaged = eng[engagement::FdOwnEngaged];
  logic.ap_inop = eng[engagement::ApInop];
  logic.ap_own_engaged = eng[engagement::ApOwnEngaged];
  logic.athr_inop = eng[engagement::AthrInop];
  logic.athr_own_engaged = eng[engagement::AthrOwnEngaged];
  logic.fmgc_opp_priority = eng[engagement::FmgcOppPriority];
  apFd.fmgc_opp_mode_sync = eng[engagement::FmgcOppModeSync];
  apFd.any_ap_fd_engaged = eng[engagement::AnyApFdEngaged];
  apFd.lateral_mode_reset = eng[engagement::LateralModeReset];
  apFd.longitudinal_mode_reset = eng[engagement::LongitudinalModeReset];

  // ---- 3. TCAS RA decoding and TCAS flags (1560–1668).
  tcas::Outputs tcasOut;
  tcas_.update({dt, in, logic, lastApFd_}, tcasOut);
  apFd.tcas_ra_corrective = tcasOut.raCorrective;
  apFd.tcas_vs_target = tcasOut.vsTargetFpm;
  apFd.tcas_ra_inhibited = tcasOut.raInhibited;
  apFd.trk_fpa_deselected = tcasOut.trkFpaDeselected;
  apFd.longi_large_box_tcas = tcasOut.longiLargeBox;

  // ---- 4. Altitude target, vertical and lateral modes (1636–3330), in Signal order.
  External ext;
  ext.logic = &logic;
  ext.oppModeSync = apFd.fmgc_opp_mode_sync;
  ext.anyApFdEngaged = apFd.any_ap_fd_engaged;
  ext.modeReset = apFd.longitudinal_mode_reset;
  ext.tcasVsTarget = tcasOut.vsTargetFpm;
  ext.tcasRaCorrective = tcasOut.raCorrective;
  ext.tcasModeAvailable = logic.tcas_mode_available;
  ext.approach.lastFlareCondition = lastOuterLoops_.flare_law.condition_Flare;
  ext.climbDescent.inFlightFor5s = conditioning_.state().inFlight5s.output;
  ext.lateral.lastPhiLocC = lastOuterLoops_.Phi_loc_c;
  // (ext.vsTcas.speedTargetKt / machMode feed the vertical copy of the ALT soft mode, which is not used: the targets
  // block computes it below.)
  const Frame f{dt, in, lastApFd_, ext};
  for (int i = idx(Signal::AltCstrApplicable); i <= idx(Signal::TcasAltHold); i++) {
    const auto id = static_cast<Signal>(i);
    if (id == Signal::TcasAltAcq) {
      // the TCAS submodes' set terms are the ALT* / ALT conditions computed earlier this frame
      ext.vsTcas.altCaptureCond = modes_.aux.altCaptureCond;
      ext.vsTcas.altHoldCond = modes_.aux.altHoldBand;
    }
    modes_.update(id, f, signals_);
  }
  // V/S, FPA (the V/S-FPA latch split by the HDG-V/S selection) and FLARE are combinational: their place in the
  // generated order (3993–4001) does not matter, and the stages below read them.
  modes_.update(Signal::VsActive, f, signals_);
  modes_.update(Signal::FpaActive, f, signals_);
  modes_.update(Signal::Flare, f, signals_);
  copyModes(signals_, apFd);
  apFd.tcas_alt_acq_cond = modes_.aux.altCaptureCond;
  apFd.tcas_alt_hold_cond = modes_.aux.altHoldBand;
  apFd.tla_to_ga_set = modes_.aux.takeoffThrust;
  apFd.any_longitudinal_mode_engaged =
      apFd.longitudinal_modes.vs_active || apFd.longitudinal_modes.fpa_active || signals_[Signal::AltHold] ||
      signals_[Signal::AltCapture] || signals_[Signal::Clb] || signals_[Signal::Des] || signals_[Signal::ExpClb] ||
      signals_[Signal::ExpDes] || signals_[Signal::Final] || signals_[Signal::GsCapture] ||
      signals_[Signal::GsTrack] || signals_[Signal::OpClb] || signals_[Signal::OpDes] ||
      signals_[Signal::SrsGoAround] || signals_[Signal::SrsTakeoff] || signals_[Signal::Tcas] ||
      signals_[Signal::Land];

  // ---- 5. Engagement stage 2 (3335–3403): FD auto disengage, V/S target not held, reversion pulse, pitch FD bars.
  engagement_.updateFdFlags({dt, in, logic, apFd}, eng);
  apFd.fd_auto_disengage = eng[engagement::FdAutoDisengage];
  apFd.vs_target_not_held = eng[engagement::VsTargetNotHeld];
  apFd.ap_fd_mode_reversion = eng[engagement::ApFdModeReversion];
  apFd.pitch_fd_bars_flashing = eng[engagement::PitchFdBarsFlashing];
  modes_.update(Signal::RollFdBarsFlashing, f, signals_);
  apFd.roll_fd_bars_flashing = signals_[Signal::RollFdBarsFlashing];

  // ---- 6. Targets (3405–3735): HDG/TRK preset, speed modes, speed targets, ALT soft mode.
  {
    targets::Trace trace;
    const targets::Inputs ti{dt, in, logic, apFd, lastApFd_, firstFrame_};
    for (int k = 0; k < targets::kFieldCount; k++) {
      targets_.update(static_cast<targets::Field>(k), ti, targetValues_, trace);
    }
  }
  const targets::Values& tv = targetValues_;
  apFd.hdg_trk_preset_available = tv.hdgTrkPreset;
  apFd.manual_spd_control_active = tv.manualSpeed;
  apFd.auto_spd_control_active = tv.autoSpeed;
  apFd.mach_control_active = tv.machMode;
  apFd.spd_target_kts = tv.spdTargetKts;
  apFd.pfd_spd_target_kts = tv.pfdSpdTargetKts;
  apFd.alt_soft_mode_active = tv.altSoft;
  // The mode logic's own copies of these two signals are not evaluated; its latches take the targets block's values.
  signals_[Signal::HdgTrkPreset] = tv.hdgTrkPreset;
  signals_[Signal::AltSoft] = tv.altSoft;

  // ---- 7. Law selection (3736–3792) and LAND capability (3796–3863).
  modes_.update(Signal::AnyLateralMode, f, signals_);
  modes_.update(Signal::ActiveLateralLaw, f, signals_);
  modes_.update(Signal::LocBcSelection, f, signals_);
  apFd.any_lateral_mode_engaged = signals_[Signal::AnyLateralMode];
  apFd.active_lateral_law = signals_.activeLateralLaw;
  apFd.loc_bc_selection = signals_[Signal::LocBcSelection];
  apFd.active_longitudinal_law = selectLongitudinalLaw(apFd, in.fms_inputs.requested_des_submode);

  engagement_.updateLandCapability({dt, in, logic, apFd}, eng);
  apFd.land_2_capability = eng[engagement::Land2Capability];
  apFd.land_3_fail_passive_capability = eng[engagement::Land3FailPassiveCapability];
  apFd.land_3_fail_op_capability = eng[engagement::Land3FailOpCapability];
  apFd.land_2_inop = eng[engagement::Land2Inop];
  apFd.land_3_fail_passive_inop = eng[engagement::Land3FailPassiveInop];
  apFd.land_3_fail_op_inop = eng[engagement::Land3FailOpInop];
  apFd.land_2_capacity = eng[engagement::Land2Capacity];
  apFd.land_3_fail_passive_capacity = eng[engagement::Land3FailPassiveCapacity];
  apFd.land_3_fail_op_capacity = eng[engagement::Land3FailOpCapacity];

  // ---- 8. Outer loops (FmgcOuterLoops, 4055–4170): lateral laws first (the flare law reads the AP sideslip command).
  ap_laws_output& io = bus_.outerLoops;
  assembleOuterLoopInputs(outerLoopInputs_, in, logic, apFd, io);
  {
    laws::LateralResults lat;
    lateralLaws_.computeLaws(io, lat);
    laws::LateralOutputs lo{};
    lateralLaws_.computeOutputs(io, lat.cmd, lo);
    io.output.Phi_loc_c = lat.phiLocDeg;
    io.output.Nosewheel_c = lo.nosewheel;
    io.output.flight_director.Phi_c_deg = lo.fdPhiDeg;
    io.output.flight_director.Beta_c_deg = lo.fdBetaDeg;
    io.output.autopilot.Phi_c_deg = lo.apPhiDeg;
    io.output.autopilot.Beta_c_deg = lo.apBetaDeg;

    laws::LawResults vert;
    verticalLaws_.computeLaws(io, vert);
    laws::PitchOutputs po{};
    verticalLaws_.computeOutputs(io, vert.cmd, po);
    io.output.flight_director.Theta_c_deg = po.fdThetaDeg;
    io.output.autopilot.Theta_c_deg = po.apThetaDeg;
    io.output.flare_law = vert.flare;
  }

  // ---- 9. Autothrust (4180–4641).
  {
    athr::Result r;
    const athr::Inputs ai{dt, in, logic, apFd, lastAthr_};
    athr_.updateModes(ai, r);
    athr_.updateN1(ai, r);
    bus_.athr = r.out;
  }

  // ---- 10. Output packing (4642–5106).
  out_ = output::assemble({in, logic, apFd, io.output, bus_.athr});

  // ---- 11. End-of-step delays (5107–5128). The other delays belong to their stages.
  lastApFd_ = apFd;
  lastAthr_ = bus_.athr;
  lastOuterLoops_ = io.output;
  firstFrame_ = false;
  return out_;
}

}  // namespace fmgc::readable
