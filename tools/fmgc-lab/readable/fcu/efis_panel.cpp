// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// EFIS control panel (generated FcuComputer.cpp 1095-1183 per side; blocks 118-242, 275-310). See efis_panel.h.

#include "efis_panel.h"

#include <cmath>

namespace fcu::readable {

namespace {
constexpr bool kRising = true;
constexpr float kNoSimInput = -1.0F;
}  // namespace

void EfisControlPanel::reset() {
  cstrPb_.reset();
  wptPb_.reset();
  vordPb_.reset();
  ndbPb_.reset();
  arptPb_.reset();
  lsPb_.reset();
  baroPush_.reset();
  baroPull_.reset();
  filter_ = efis_filter_selection::NONE;
  fdOn_ = true;
  ls_.reset();
  std_ = false;
  qnh_ = true;
  qfe_ = false;
  hpa_ = kStandardHpa;
  inHg_ = kStandardInHg;
}

base_fcu_efis_logic_outputs EfisControlPanel::update(const base_fcu_efis_panel_inputs& panel, bool fdAutoActivate,
                                                     bool fdAutoDeactivate, bool qfeAvailable, float simBaroHpa) {
  // ND filters
  const bool cstr = cstrPb_.update(panel.cstr_button_pushed, kRising);
  const bool wpt = wptPb_.update(panel.wpt_button_pushed, kRising);
  const bool vord = vordPb_.update(panel.vord_button_pushed, kRising);
  const bool ndb = ndbPb_.update(panel.ndb_button_pushed, kRising);
  const bool arpt = arptPb_.update(panel.arpt_button_pushed, kRising);
  using F = efis_filter_selection;
  const bool litFilterPushed = (filter_ == F::CSTR && cstr) || (filter_ == F::WPT && wpt) ||
                               (filter_ == F::VORD && vord) || (filter_ == F::NDB && ndb) ||
                               (filter_ == F::ARPT && arpt);
  if (litFilterPushed) {
    filter_ = F::NONE;
  } else if (cstr) {
    filter_ = F::CSTR;
  } else if (wpt) {
    filter_ = F::WPT;
  } else if (vord) {
    filter_ = F::VORD;
  } else if (ndb) {
    filter_ = F::NDB;
  } else if (arpt) {
    filter_ = F::ARPT;
  }

  // FD (button level, not edge)
  if (fdAutoActivate) {
    fdOn_ = true;
  } else if (fdAutoDeactivate) {
    fdOn_ = false;
  } else if (panel.fd_button_pushed) {
    fdOn_ = !fdOn_;
  }

  // LS
  const bool lsOn = ls_.update(lsPb_.update(panel.ls_button_pushed, kRising), /*init*/ false);

  // baro reference
  const bool push = baroPush_.update(panel.baro_knob.pushed, kRising);
  const bool pull = baroPull_.update(panel.baro_knob.pulled, kRising);
  if (push) {
    if (std_) {
      std_ = false;
    } else {
      qnh_ = !qnh_;
      qfe_ = !qfe_;
    }
  } else if (pull) {
    std_ = true;
  }
  if (!qfeAvailable) {
    qnh_ = true;
    qfe_ = false;
  }

  // baro setting
  if (simBaroHpa != kNoSimInput) {
    hpa_ = simBaroHpa;
    inHg_ = simBaroHpa * kInHgPerHpa;
  }
  const float clicks = static_cast<float>(panel.baro_knob.turns);
  if (!std_ && !panel.baro_is_inhg) {
    hpa_ = std::fmax(std::fmin(hpa_ + clicks, kMaxBaroHpa), kMinBaroHpa);
    inHg_ = std::round(hpa_ * kInHgPerHpa * 100.0F) / 100.0F;
  } else if (!std_ && panel.baro_is_inhg) {
    inHg_ = std::fmax(std::fmin(clicks * 0.01F + inHg_, kMaxBaroInHg), kMinBaroInHg);
    hpa_ = inHg_ * kHpaPerInHg;
    hpa_ = std::round(hpa_);
  }

  base_fcu_efis_logic_outputs out{};
  out.fd_on = fdOn_;
  out.fd_auto_activate = fdAutoActivate;
  out.fd_auto_deactivate = fdAutoDeactivate;
  out.ls_on = lsOn;
  out.efis_filter = filter_;
  out.baro_std = std_;
  out.baro_qnh = qnh_ && !std_;
  out.baro_qfe = qfe_ && !std_;
  out.baro_value_hpa = hpa_;
  out.baro_value_inhg = inHg_;
  return out;
}

base_fcu_efis_panel_outputs EfisControlPanel::displayOutputs(const base_fcu_efis_logic_outputs& logic,
                                                              const base_fcu_efis_panel_inputs& panel) {
  base_fcu_efis_panel_outputs o{};
  using F = efis_filter_selection;
  o.fd_light_on = logic.fd_on;
  o.ls_light_on = logic.ls_on;
  o.cstr_light_on = logic.efis_filter == F::CSTR;
  o.wpt_light_on = logic.efis_filter == F::WPT;
  o.vord_light_on = logic.efis_filter == F::VORD;
  o.ndb_light_on = logic.efis_filter == F::NDB;
  o.arpt_light_on = logic.efis_filter == F::ARPT;
  // PFD baro window: 0 = "Std", 1 = hPa value, 2 = inHg value (by the unit selector, whatever QNH/QFE)
  o.baro_value_mode = static_cast<int8_T>(logic.baro_std ? 0 : (!panel.baro_is_inhg ? 1 : 2));
  switch (o.baro_value_mode) {
    case 1:
      o.baro_value = logic.baro_value_hpa;
      break;
    case 2:
      o.baro_value = logic.baro_value_inhg;
      break;
    default:
      o.baro_value = 0.0F;
      break;
  }
  // 0 = STD, 1 = QNH, 2 = QFE
  o.baro_mode = static_cast<int8_T>(logic.baro_std ? 0 : (logic.baro_qnh ? 1 : 2));
  return o;
}

}  // namespace fcu::readable
