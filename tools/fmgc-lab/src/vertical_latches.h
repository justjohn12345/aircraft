// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The generated counterpart of each mode signal (readable/vertical.h): the SR flip-flop memory of a latch signal and
// the ap_fd_logic field of an output signal. Shared by check_vertical.cpp and check_fmgc.cpp, which include
// FmgcComputer.h with private members made public.

#pragma once

#include "vertical.h"

namespace {

using fmgc::readable::Signal;

/** Generated latch memory for a latch signal, or nullptr for combinational signals. */
const boolean_T* latchOf(const FmgcComputer::D_Work_FmgcComputer_T& d, Signal s) {
  switch (s) {
    case Signal::SrsTakeoff: return &d.Memory_PreviousInput_k;
    case Signal::RwyLoc: return &d.Memory_PreviousInput_c;
    case Signal::RwyTrk: return &d.Memory_PreviousInput_b;
    case Signal::RollOut: return &d.Memory_PreviousInput_m;
    case Signal::GaTrk: return &d.Memory_PreviousInput_el;
    case Signal::HdgTrk: return &d.Memory_PreviousInput_dy;
    case Signal::NavArmed: return &d.Memory_PreviousInput_j;
    case Signal::Nav: return &d.Memory_PreviousInput_ly;
    case Signal::LocArmed: return &d.Memory_PreviousInput_a;
    case Signal::LocCapture: return &d.Memory_PreviousInput_cv;
    case Signal::LocTrack: return &d.Memory_PreviousInput_lq;
    case Signal::HdgTrkPreset: return &d.Memory_PreviousInput_bw;
    case Signal::LandArmed: return &d.Memory_PreviousInput_l;
    case Signal::Land: return &d.Memory_PreviousInput_d;
    case Signal::FlareMemory: return &d.Memory_PreviousInput_bc;
    case Signal::FinalArmed: return &d.Memory_PreviousInput_dv;
    case Signal::Final: return &d.Memory_PreviousInput_f;
    case Signal::SrsGoAround: return &d.Memory_PreviousInput_i;
    case Signal::AltArmPossible: return &d.Memory_PreviousInput_n;
    case Signal::AltCapture: return &d.Memory_PreviousInput_ne;
    case Signal::AltHold: return &d.Memory_PreviousInput_cb;
    case Signal::AltCruiseMemory: return &d.Memory_PreviousInput_no;
    case Signal::VsFpa: return &d.Memory_PreviousInput_fg;
    case Signal::ClbArmed: return &d.Memory_PreviousInput_ma;
    case Signal::Clb: return &d.Memory_PreviousInput_ec;
    case Signal::DesArmed: return &d.Memory_PreviousInput_nt;
    case Signal::Des: return &d.Memory_PreviousInput_b3;
    case Signal::OpClb: return &d.Memory_PreviousInput_ae;
    case Signal::OpDes: return &d.Memory_PreviousInput_ev;
    case Signal::ExpClb: return &d.Memory_PreviousInput_mx;
    case Signal::ExpDes: return &d.Memory_PreviousInput_o;
    case Signal::GlideArmed: return &d.Memory_PreviousInput_fm;
    case Signal::GsCapture: return &d.Memory_PreviousInput_nu;
    case Signal::GsTrack: return &d.Memory_PreviousInput_as;
    case Signal::TcasArmed: return &d.Memory_PreviousInput_n0;
    case Signal::Tcas: return &d.Memory_PreviousInput_i5;
    case Signal::TcasAltAcq: return &d.Memory_PreviousInput_h;
    case Signal::TcasAltHold: return &d.Memory_PreviousInput_cp;
    case Signal::AltSoft: return &d.Memory_PreviousInput_ak;
    default: return nullptr;
  }
}

/** The ap_fd_logic output field of a signal, or nullptr for internal latches. */
boolean_T* outputOf(base_fmgc_ap_fd_logic_outputs& o, Signal s) {
  auto& m = o.longitudinal_modes;
  auto& a = o.armed_modes;
  auto& l = o.lateral_modes;
  switch (s) {
    case Signal::AltCstrApplicable: return &o.alt_cstr_applicable;
    case Signal::SrsTakeoff: return &m.pitch_takeoff_active;
    case Signal::RwyLoc: return &l.rwy_loc_submode_active;
    case Signal::RwyTrk: return &l.rwy_trk_submode_active;
    case Signal::RwyActive: return &l.rwy_active;
    case Signal::RollOut: return &l.rollout_submode_active;
    case Signal::Align: return &l.align_submode_active;
    case Signal::GaTrk: return &l.roll_goaround_active;
    case Signal::LateralReversion: return &o.lateral_mode_reversion;
    case Signal::HdgActive: return &l.hdg_active;
    case Signal::TrkActive: return &l.trk_active;
    case Signal::NavArmed: return &a.nav_armed;
    case Signal::Nav: return &l.nav_active;
    case Signal::LocArmed: return &a.loc_armed;
    case Signal::LocCapture: return &l.loc_cpt_active;
    case Signal::LocTrack: return &l.loc_trk_active;
    case Signal::RollFdBarsFlashing: return &o.roll_fd_bars_flashing;
    case Signal::HdgTrkPreset: return &o.hdg_trk_preset_available;
    case Signal::AnyLateralMode: return &o.any_lateral_mode_engaged;
    case Signal::LocBcSelection: return &o.loc_bc_selection;
    case Signal::LandArmed: return &a.land_armed;
    case Signal::Land: return &l.land_active;
    case Signal::FinalArmed: return &a.final_des_armed;
    case Signal::Final: return &m.final_des_active;
    case Signal::SrsGoAround: return &m.pitch_goaround_active;
    case Signal::AltArmPossible: return &a.alt_acq_arm_possible;
    case Signal::AltArmed: return &a.alt_acq_armed;
    case Signal::AltCapture: return &m.alt_acq_active;
    case Signal::AltHold: return &m.alt_hold_active;
    case Signal::AltCruise: return &m.fma_dash_display;
    case Signal::ReversionVs: return &o.longitudinal_mode_reversion_vs;
    case Signal::ClbArmed: return &a.clb_armed;
    case Signal::Clb: return &m.clb_active;
    case Signal::DesArmed: return &a.des_armed;
    case Signal::Des: return &m.des_active;
    case Signal::OpClb: return &m.op_clb_active;
    case Signal::ReversionOpClb: return &o.longitudinal_mode_reversion_op_clb;
    case Signal::OpDes: return &m.op_des_active;
    case Signal::ExpClb: return &m.exp_clb_active;
    case Signal::ExpDes: return &m.exp_des_active;
    case Signal::GlideArmed: return &a.glide_armed;
    case Signal::GsCapture: return &m.gs_capt_active;
    case Signal::GsTrack: return &m.gs_trk_active;
    case Signal::TcasArmed: return &a.tcas_armed;
    case Signal::Tcas: return &m.tcas_active;
    case Signal::AltSoft: return &o.alt_soft_mode_active;
    case Signal::VsActive: return &m.vs_active;
    case Signal::FpaActive: return &m.fpa_active;
    case Signal::Flare: return &m.flare_active;
    default: return nullptr;
  }
}

}  // namespace
