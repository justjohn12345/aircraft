// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential test: every readable block in blocks.h against the generated library function it replaces, on random
// input sequences. Outputs must be bit-identical (same arithmetic, same order). Also checks that every SRFlipFlop truth
// table in both models is the default one that SrLatch implements.

#include <cstdio>
#include <cstring>
#include <random>
#include <string>

#define private public
#include "FmgcComputer.h"
#include "FmgcOuterLoops.h"
#undef private

#include "blocks.h"

using namespace fmgc::blocks;

namespace {

std::mt19937_64 rng(12345);
long failures = 0;
long checks = 0;

double uni(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(rng);
}
bool coin(double p = 0.5) {
  return uni(0, 1) < p;
}
int pick(int n) {
  return static_cast<int>(rng() % n);
}

/** A time step: mostly sim-like frame times, sometimes 0 or long. */
double randomDt() {
  switch (pick(8)) {
    case 0:
      return 0.0;
    case 1:
      return uni(0.1, 2.0);
    default:
      return uni(0.005, 0.07);
  }
}

/** A value with occasional exact boundary values and repeats, so equality branches get exercised. */
double randomValue(double scale, double previous) {
  switch (pick(10)) {
    case 0:
      return previous;
    case 1:
      return 0.0;
    case 2:
      return static_cast<double>(pick(41) - 20);
    default:
      return uni(-scale, scale);
  }
}

template <typename T>
bool same(T a, T b) {
  return std::memcmp(&a, &b, sizeof(T)) == 0;
}

template <typename T>
void expectSame(const char* block, long seq, int step, T generated, T readable) {
  checks++;
  if (!same(generated, readable)) {
    if (failures++ < 20) {
      std::printf("FAIL %-22s seq %ld step %d: generated %.17g readable %.17g\n", block, seq, step,
                  static_cast<double>(generated), static_cast<double>(readable));
    }
  }
}

constexpr long SEQUENCES = 20000;
constexpr int STEPS = 200;

base_arinc_429 randomWord() {
  base_arinc_429 w{};
  w.SSM = static_cast<uint32_t>(pick(4));
  switch (pick(4)) {
    case 0:
      w.Data = static_cast<float>(rng() & 0x1fffffff);
      break;
    case 1:
      w.Data = static_cast<float>(uni(-5e9, 5e9));
      break;
    default:
      w.Data = static_cast<float>(uni(-1000, 1000));
  }
  return w;
}

void testA429() {
  for (long i = 0; i < SEQUENCES * 20; i++) {
    base_arinc_429 w = randomWord();
    float def = static_cast<float>(uni(-10, 10));
    float y;
    FmgcComputer::FmgcComputer_MATLABFunction(&w, def, &y);
    expectSame("a429::valueOr", i, 0, y, a429::valueOr(w, def));
    boolean_T b;
    FmgcComputer::FmgcComputer_MATLABFunction_ie(&w, &b);
    expectSame("a429::isNormal", i, 0, static_cast<bool>(b), a429::isNormal(w));
    FmgcComputer::FmgcComputer_MATLABFunction_m(&w, &b);
    expectSame("a429::isNotFailed", i, 0, static_cast<bool>(b), a429::isNotFailed(w));
    FmgcComputer::FmgcComputer_MATLABFunction_c(&w, &y);
    expectSame("a429::rawValue", i, 0, y, a429::rawValue(w));
    double n = static_cast<double>(pick(40) - 4);
    uint32_T bit;
    FmgcComputer::FmgcComputer_MATLABFunction_i(&w, n, &bit);
    expectSame("a429::bit", i, 0, static_cast<uint32_t>(bit), a429::bit(w, n));
  }
}

void testDiscrete() {
  for (long s = 0; s < SEQUENCES; s++) {
    bool rising = coin();
    bool retrig = coin();
    double delay = coin(0.2) ? 0.0 : uni(0.0, 3.0);
    FmgcComputer::rtDW_MATLABFunction_FmgcComputer_k_T gConfirm{};
    FmgcComputer::rtDW_MATLABFunction_FmgcComputer_c_T gEdge{};
    FmgcComputer::rtDW_MATLABFunction_FmgcComputer_f_T gMono{};
    FmgcComputer::FmgcComputer_MATLABFunction_j_Reset(&gConfirm);
    FmgcComputer::FmgcComputer_MATLABFunction_p_Reset(&gEdge);
    FmgcComputer::FmgcComputer_MATLABFunction_d_Reset(&gMono);
    ConfirmNode confirm;
    EdgeDetector edge;
    Monostable mono;
    SrLatch latch;
    bool gLatch = false;
    static const bool defaultTable[16]{false, true, false, false, true, true, false, false,
                                       true, false, true, true, false, false, false, false};
    for (int k = 0; k < STEPS; k++) {
      bool u = coin(k % 20 < 10 ? 0.8 : 0.2);
      double dt = randomDt();
      boolean_T y;
      FmgcComputer::FmgcComputer_MATLABFunction_a(u, dt, rising, delay, &y, &gConfirm);
      expectSame("ConfirmNode", s, k, static_cast<bool>(y), confirm.update(u, dt, rising, delay));
      FmgcComputer::FmgcComputer_MATLABFunction_g(u, rising, &y, &gEdge);
      expectSame("EdgeDetector", s, k, static_cast<bool>(y), edge.update(u, rising));
      FmgcComputer::FmgcComputer_MATLABFunction_m3(u, dt, &y, rising, retrig, delay, &gMono);
      expectSame("Monostable", s, k, static_cast<bool>(y), mono.update(u, dt, rising, retrig, delay));
      bool set = coin(0.3), reset = coin(0.3);
      gLatch = SrLatch::fromTable(defaultTable, set, reset, gLatch);
      expectSame("SrLatch", s, k, gLatch, latch.update(set, reset));
    }
  }
  for (long i = 0; i < SEQUENCES; i++) {
    boolean_T u[19];
    bool v[19];
    for (int j = 0; j < 19; j++) {
      u[j] = v[j] = coin();
    }
    real32_T y;
    FmgcComputer::FmgcComputer_MATLABFunction_gy(u, &y);
    expectSame("packBits", i, 0, y, packBits(v));
  }
}

void testFilters() {
  for (long s = 0; s < SEQUENCES; s++) {
    double c1 = uni(0.01, 10), c2 = uni(0.01, 10), c3 = uni(0.01, 10), c4 = uni(0.01, 10);
    double up = uni(-5, 5), lo = uni(-5, 5), init = coin() ? 0.0 : 1.0;
    FmgcComputer::rtDW_LagFilter_FmgcComputer_T gLagF{};
    FmgcComputer::rtDW_LeadLagFilter_FmgcComputer_T gLeadLagC{};
    FmgcOuterLoops::rtDW_LagFilter_FmgcOuterLoops_T gLag{};
    FmgcOuterLoops::rtDW_LagFilter_FmgcOuterLoops_j_T gLagI{};
    FmgcOuterLoops::rtDW_WashoutFilter_FmgcOuterLoops_T gWash{};
    FmgcOuterLoops::rtDW_WashoutFilter_FmgcOuterLoops_o_T gWashB{};
    FmgcOuterLoops::rtDW_LeadLagFilter_FmgcOuterLoops_T gLeadLag{};
    FmgcOuterLoops::rtDW_RateLimiter_FmgcOuterLoops_T gRate{};
    FmgcOuterLoops::rtDW_RateLimiter_FmgcOuterLoops_h_T gRateG{};
    FmgcComputer::FmgcComputer_LagFilter_Reset(&gLagF);
    FmgcComputer::FmgcComputer_LeadLagFilter_Reset(&gLeadLagC);
    FmgcOuterLoops::FmgcOuterLoops_LagFilter_Reset(&gLag);
    FmgcOuterLoops::FmgcOuterLoops_LagFilter_e_Reset(&gLagI);
    FmgcOuterLoops::FmgcOuterLoops_WashoutFilter_Reset(&gWash);
    FmgcOuterLoops::FmgcOuterLoops_WashoutFilter_l_Reset(&gWashB);
    FmgcOuterLoops::FmgcOuterLoops_LeadLagFilter_Reset(&gLeadLag);
    FmgcOuterLoops::FmgcOuterLoops_RateLimiter_Reset(&gRate);
    FmgcOuterLoops::FmgcOuterLoops_RateLimiter_f_Reset(&gRateG);
    LagFilter<float> lagF;
    LagFilter<double> lag, lagI;
    WashoutFilter wash, washB;
    LeadLagFilter leadLag, leadLagC;
    SwitchBlend rate, rateG;
    double u = 0;
    for (int k = 0; k < STEPS; k++) {
      u = randomValue(100, u);
      double dt = randomDt();
      float uf = static_cast<float>(u);
      real32_T yf;
      FmgcComputer::FmgcComputer_LagFilter(uf, c1, dt, &yf, &gLagF);
      expectSame("LagFilter<float>", s, k, yf, lagF.update(uf, c1, dt));
      double y;
      FmgcComputer::FmgcComputer_LeadLagFilter(u, c1, c2, c3, c4, dt, &y, &gLeadLagC);
      expectSame("LeadLagFilter (FC)", s, k, y, leadLagC.update(u, c1, c2, c3, c4, dt));
      FmgcOuterLoops::FmgcOuterLoops_LagFilter(u, c1, &dt, &y, &gLag);
      expectSame("LagFilter<double>", s, k, y, lag.update(u, c1, dt));
      FmgcOuterLoops::FmgcOuterLoops_LagFilter_i(&u, c1, &dt, &y, &gLagI);
      expectSame("LagFilter (ptr)", s, k, y, lagI.update(u, c1, dt));
      FmgcOuterLoops::FmgcOuterLoops_WashoutFilter(u, c1, &dt, &y, &gWash);
      expectSame("WashoutFilter", s, k, y, wash.update(u, c1, dt));
      FmgcOuterLoops::FmgcOuterLoops_WashoutFilter_b(&u, c1, &dt, &y, &gWashB);
      expectSame("WashoutFilter (ptr)", s, k, y, washB.update(u, c1, dt));
      FmgcOuterLoops::FmgcOuterLoops_LeadLagFilter(u, c1, c2, c3, c4, &dt, &y, &gLeadLag);
      expectSame("LeadLagFilter (OL)", s, k, y, leadLag.update(u, c1, c2, c3, c4, dt));
      bool b = coin(k % 30 < 15 ? 0.9 : 0.1);
      boolean_T bb = b;
      FmgcOuterLoops::FmgcOuterLoops_RateLimiter(b, up, lo, &dt, init, &y, &gRate);
      expectSame("SwitchBlend", s, k, y, rate.update(b, up, lo, dt, init));
      FmgcOuterLoops::FmgcOuterLoops_RateLimiter_g(&bb, up, lo, &dt, init, &y, &gRateG);
      expectSame("SwitchBlend (ptr)", s, k, y, rateG.update(b, up, lo, dt, init));
    }
  }
}

void testLateralVertical() {
  for (long s = 0; s < SEQUENCES; s++) {
    FmgcOuterLoops::rtDW_Chart_FmgcOuterLoops_T gChart{};
    FmgcOuterLoops::rtDW_Chart_FmgcOuterLoops_k_T gChartK{};
    FmgcOuterLoops::rtDW_MATLABFunction_FmgcOuterLoops_j_T gBank{};
    double out;
    FmgcOuterLoops::FmgcOuterLoops_Chart_Reset(&out, &gChart);
    FmgcOuterLoops::FmgcOuterLoops_Chart_c_Reset(&out, &gChartK);
    FmgcOuterLoops::FmgcOuterLoops_MATLABFunction_d_Reset(&gBank);
    TurnDirectionChart chart, chartK;
    HeadingBankLimit bank;
    double psi = uni(0, 360), dpsi = 0, phi = 0;
    for (int k = 0; k < STEPS; k++) {
      double right = uni(0, 40) * (coin(0.1) ? 9 : 1);
      double left = right - 360.0 + uni(-40, 40);
      if (coin(0.5)) {
        std::swap(right, left);
      }
      bool shortPath = coin(0.2);
      FmgcOuterLoops::FmgcOuterLoops_Chart(right, left, shortPath, &out, &gChart);
      expectSame("TurnDirectionChart", s, k, out, chart.update(right, left, shortPath));
      FmgcOuterLoops::FmgcOuterLoops_Chart_k(right, left, shortPath ? 1.0 : 0.0, &out, &gChartK);
      expectSame("TurnDirectionChart k", s, k, out, chartK.update(right, left, shortPath));
      if (coin(0.05)) {
        psi = uni(0, 360);
      }
      dpsi = coin(0.2) ? static_cast<double>(pick(7) * 5) * (coin() ? 1 : -1) : randomValue(45, dpsi);
      phi = randomValue(35, phi);
      double up, lo;
      FmgcOuterLoops::FmgcOuterLoops_MATLABFunction_g(&psi, dpsi, phi, &up, &lo, &gBank);
      double lim = bank.update(psi, dpsi, phi);
      expectSame("HeadingBankLimit up", s, k, up, lim);
      expectSame("HeadingBankLimit lo", s, k, lo, -lim);
    }
  }
  for (long i = 0; i < SEQUENCES * 10; i++) {
    double tau = uni(0.1, 30), zeta = uni(0.1, 2);
    double k1, k2;
    FmgcOuterLoops::FmgcOuterLoops_MATLABFunction(tau, zeta, &k2, &k1);
    NavGains g = navGains(tau, zeta);
    expectSame("navGains k1", i, 0, k1, g.k1);
    expectSame("navGains k2", i, 0, k2, g.k2);
    double a = randomValue(100, 0), b = randomValue(100, a), c = randomValue(100, b);
    double y;
    FmgcOuterLoops::FmgcOuterLoops_Voter1(a, b, c, &y);
    expectSame("median3", i, 0, y, median3(a, b, c));
    ap_laws_output in{};
    in.data.V_tas_kn = uni(50, 500);
    in.data.V_ias_kn = uni(100, 350);
    in.data.VLS_kn = uni(120, 160) + (coin(0.2) ? 0 : 0.5);
    in.data.VMAX_kn = uni(200, 350);
    in.input.V_c_kn = coin(0.2) ? in.data.VLS_kn : uni(120, 340);
    double u = uni(-10, 10);
    FmgcOuterLoops::FmgcOuterLoops_VSLimiter(u, &in, &y);
    expectSame("vsLimit", i, 0, y, vsLimit(u, in.data.V_tas_kn));
    FmgcOuterLoops::FmgcOuterLoops_V_LSSpeedSelection1(&in.input.V_c_kn, &in.data.VLS_kn, &y);
    expectSame("vlsWithMargin", i, 0, y, vlsWithMargin(in.input.V_c_kn, in.data.VLS_kn));
    double vs[6];
    for (double& v : vs) {
      v = uni(-6000, 6000);
    }
    double margin = uni(0, 10);
    double fd, ap;
    FmgcOuterLoops::FmgcOuterLoops_SpeedProtectionSignalSelection(&in, vs[0], vs[1], vs[2], vs[3], vs[4], vs[5], margin,
                                                                  &fd, &ap);
    ProtectedCommand pc = speedProtection(in.input.V_c_kn, in.data.VLS_kn, in.data.V_ias_kn, in.data.VMAX_kn, vs[0],
                                          vs[1], vs[2], vs[3], vs[4], vs[5], margin);
    expectSame("speedProtection fd", i, 0, fd, pc.fd);
    expectSame("speedProtection ap", i, 0, ap, pc.ap);
  }
}

}  // namespace

#include "params.gen.h"

namespace {

/** Every Logic_table* parameter must be the default SR truth table that SrLatch implements (Q and !Q columns). */
int checkLogicTables() {
  static const bool expected[16]{false, true, false, false, true, true, false, false,
                                 true, false, true, true, false, false, false, false};
  int count = 0;
  auto check = [&](const char* name, const boolean_T* table) {
    count++;
    for (int i = 0; i < 16; i++) {
      checks++;
      if (static_cast<bool>(table[i]) != expected[i]) {
        failures++;
        std::printf("FAIL SR table %s differs from the default at %d\n", name, i);
        return;
      }
    }
  };
  forEachLogicTable_FmgcComputer(FmgcComputer::FmgcComputer_P, check);
  forEachLogicTable_FmgcOuterLoops(FmgcOuterLoops::FmgcOuterLoops_rtP, check);
  return count;
}

}  // namespace

int main() {
  testA429();
  testDiscrete();
  testFilters();
  testLateralVertical();
  int tables = checkLogicTables();
  std::printf("%ld checks, %ld failures; %d SR flip-flop tables checked\n", checks, failures, tables);
  return failures == 0 ? 0 : 1;
}
