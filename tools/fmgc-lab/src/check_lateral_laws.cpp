// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Law-level differential check of the readable lateral laws (readable/efcs/lateral_*_law) against the generated
// LateralNormalLaw and LateralDirectLaw: both stepped with the same synthetic, seeded input trajectories
// (src/law_trajectory.h), every output and hidden memory compared bit-exact every step (two NaNs match). reset() at
// random times as an ELAC restart does. The ELAC checker (src/check_elac.cpp) checks them again in situ.
//
// Usage: efcs-lateral-law-check [--seed S] [--steps N] [--special P]
// Exit status 0 when everything matched, 1 on any mismatch.

#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>

#include "lateral_law_state.h"
#include "law_trajectory.h"

namespace R = efcs::readable;
using namespace law_trajectory;

namespace {

struct Mismatch {
  long count = 0, first = -1;
  double g = 0, r = 0;
};

struct Report {
  std::map<std::string, Mismatch> fields;
  void check(const std::string& name, long step, double g, double r) {
    Mismatch& m = fields[name];
    if (same(g, r)) return;
    if (m.count++ == 0) {
      m.first = step;
      m.g = g;
      m.r = r;
    }
  }
  long failing() const {
    long n = 0;
    for (const auto& [name, m] : fields) n += m.count > 0;
    return n;
  }
  void print() const {
    for (const auto& [name, m] : fields) {
      if (m.count) {
        std::printf("  %-56s %8ld mismatches, first at step %ld: generated %.17g readable %.17g\n", name.c_str(),
                    m.count, m.first, m.g, m.r);
      }
    }
  }
};

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 1;
  long steps = 1000000;
  double specialRate = 0.0005;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--seed" && i + 1 < argc) {
      seed = std::strtoull(argv[++i], nullptr, 10);
    } else if (a == "--steps" && i + 1 < argc) {
      steps = std::atol(argv[++i]);
    } else if (a == "--special" && i + 1 < argc) {
      specialRate = std::atof(argv[++i]);
    } else {
      std::fprintf(stderr, "usage: efcs-lateral-law-check [--seed S] [--steps N] [--special P]\n");
      return 2;
    }
  }

  Rng rng(seed);
  auto gNormal = std::make_unique<LateralNormalLaw>();
  auto gDirect = std::make_unique<LateralDirectLaw>();
  R::LateralNormalLaw rNormal;
  R::LateralDirectLaw rDirect;
  gNormal->init();
  rNormal.init();

  // trajectories
  Walk theta(-15, 30, 0.3, {8.0}, specialRate), phi(-90, 90, 1.0, {-71, -67, -66, -45, -33, 0, 33, 45, 66, 67, 71},
                                                  specialRate);
  Walk r(-20, 20, 0.5, {0.0}, specialRate), p(-30, 30, 1.0, {0.0}, specialRate);
  Walk vias(-5, 450, 2.0, {-1, 0, 40, 60, 80, 100, 120, 150, 180, 320, 380, 400}, specialRate);
  Walk vtas(-5, 1100, 3.0, {100, 1000}, specialRate);
  Walk hra(-5, 2500, 10.0, {400}, specialRate);
  Walk xi(-1.2, 1.2, 0.05, {-1, 0, 1}, specialRate), zeta(-1.2, 1.2, 0.05, {-1, 0, 1}, specialRate);
  Walk apPhi(-40, 40, 1.0, {}, specialRate), apBeta(-10, 10, 0.3, {}, specialRate);
  Toggle onGround(400), tracking(3000), aoaProt(800), speedProt(800), ap(1500);

  Report rep;
  long resets = 0;
  for (long s = 1; s <= steps; s++) {
    if (rng.chance(0.0005)) {
      gNormal->reset();
      rNormal.reset();
      gDirect->reset();
      rDirect.reset();
      resets++;
    }
    R::LateralNormalLawInputs in;
    in.time_dt = frameTime(rng, specialRate);
    in.Theta_deg = theta.next(rng);
    in.Phi_deg = phi.next(rng);
    in.r_deg_s = r.next(rng);
    in.pk_deg_s = p.next(rng);
    in.V_ias_kn = vias.next(rng);
    in.V_tas_kn = vtas.next(rng);
    in.H_radio_ft = hra.next(rng);
    in.delta_xi_pos = xi.next(rng);
    in.delta_zeta_pos = zeta.next(rng);
    in.on_ground = onGround.next(rng);
    in.tracking_mode_on = tracking.next(rng);
    in.high_aoa_prot_active = aoaProt.next(rng);
    in.high_speed_prot_active = speedProt.next(rng);
    in.ap_phi_c_deg = apPhi.next(rng);
    in.ap_beta_c_deg = apBeta.next(rng);
    in.any_ap_engaged = ap.next(rng);

    double gXi = 0, gZeta = 0;
    const boolean_T onGr = in.on_ground, track = in.tracking_mode_on, aoa = in.high_aoa_prot_active,
                    spd = in.high_speed_prot_active, anyAp = in.any_ap_engaged;
    gNormal->step(&in.time_dt, &in.Theta_deg, &in.Phi_deg, &in.r_deg_s, &in.pk_deg_s, &in.V_ias_kn, &in.V_tas_kn,
                  &in.H_radio_ft, &in.delta_xi_pos, &in.delta_zeta_pos, &onGr, &track, &aoa, &spd, &in.ap_phi_c_deg,
                  &in.ap_beta_c_deg, &anyAp, &gXi, &gZeta);
    const R::LateralLawOutputs ro = rNormal.step(in);
    rep.check("normal.xi_deg", s, gXi, ro.xi_deg);
    rep.check("normal.zeta_deg", s, gZeta, ro.zeta_deg);
    lateralNormalLawPairs(*gNormal, rNormal,
                          [&](const char* name, double g, double rv) { rep.check(std::string("normal.") + name, s, g, rv); });

    R::LateralDirectLawInputs din{in.time_dt, in.delta_xi_pos};
    double gdXi = 0, gdZeta = 0;
    gDirect->step(&din.time_dt, &din.delta_xi_pos, &gdXi, &gdZeta);
    const R::LateralLawOutputs rdo = rDirect.step(din);
    rep.check("direct.xi_deg", s, gdXi, rdo.xi_deg);
    rep.check("direct.zeta_deg", s, gdZeta, rdo.zeta_deg);
    lateralDirectLawPairs(*gDirect, rDirect,
                          [&](const char* name, double g, double rv) { rep.check(std::string("direct.") + name, s, g, rv); });
  }

  std::printf("seed %llu, %ld steps, %ld resets, special rate %g: %zu fields, %ld with mismatches\n",
              static_cast<unsigned long long>(seed), steps, resets, specialRate, rep.fields.size(), rep.failing());
  rep.print();
  return rep.failing() == 0 ? 0 : 1;
}
