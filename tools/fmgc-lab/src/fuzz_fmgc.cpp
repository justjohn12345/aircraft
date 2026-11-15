// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// libFuzzer harness for the generated A32NX FMGC (Phase 4), and the same code as a standalone tool.
//
// Each input (src/fuzz_case.h) restores a bank snapshot (src/fuzz_bank.h) and runs the recorded inputs that followed
// it, with timed events from the scenario-injector vocabulary on top. Every step goes through the oracles
// (src/fuzz_oracle.h): behavioural properties of the stock model, and the differential checks against the readable
// stages. Coverage feedback comes from the generated model only (compiled with -fsanitize=fuzzer-no-link; the harness
// and the readable code are not instrumented), plus "state" counters in libFuzzer's extra-counters section derived from
// the model's outputs: vertical/lateral mode transitions and pairs, armed-mode sets, law pairs, A/THR FMA transitions
// and property bad states (FMGC_FUZZ_STATE_FEEDBACK=0 turns them off).
//
// Environment:
//   FMGC_FUZZ_BANK      bank file (default build/fuzz/bank.bin next to the binary's tools/fmgc-lab)
//   FMGC_FUZZ_FINDINGS  directory for finding reproducers (<check>-<hash>.bin + .txt; first 3 per check per process)
//   FMGC_FUZZ_ABORT     comma list of checks that abort (so libFuzzer saves/minimises the input): property ids,
//                       oracle names, "diff" (all differential checks, the default), "all", "none"
//
// Standalone build (FMGC_FUZZ_STANDALONE): fmgc-fuzz-run
//   run [--trace] [--summary] FILE|DIR...       run inputs, print findings (and the property table with --summary)
//   minimize CHECK IN OUT                       shrink an input that triggers CHECK (events, length, durations)
//   seeds DIR [N_EXTRA_PER_RECORDING]           write the seed corpus (one plain input per snapshot + scenario seeds)
//   info                                        print the bank contents

#include "fuzz_runner.h"

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace fmgc::fuzz;

namespace {

struct Harness {
  Bank bank;
  std::unique_ptr<OracleSet> oracles;
  std::unique_ptr<FuzzRunner> runner;
  std::string findingsDir;
  std::set<std::string> abortChecks;
  bool abortDiff = true, abortAll = false;
  bool stateFeedback = true;
  std::map<std::string, int> saved;  // reproducers written per check (this process)
  std::vector<int> savedProp = std::vector<int>(fmgc::props::kCount, 0);
  std::vector<bool> abortProp = std::vector<bool>(fmgc::props::kCount, false);
};

Harness* gH = nullptr;

std::string defaultBank(const char* argv0) {
  std::string exe = argv0 ? argv0 : "";
  std::string dir = exe.substr(0, exe.find_last_of('/') + 1);
  return dir + "fuzz/bank.bin";  // binaries live in tools/fmgc-lab/build/
}

void setup(const char* argv0, bool withOracles = true) {
  gH = new Harness();
  const char* bankPath = std::getenv("FMGC_FUZZ_BANK");
  std::string path = bankPath ? bankPath : defaultBank(argv0);
  std::string err;
  if (!gH->bank.open(path, err)) {
    std::fprintf(stderr, "fmgc-fuzz: %s\n", err.c_str());
    std::exit(2);
  }
  OracleSet::Options opt;
  if (!withOracles) {
    opt = {false, false, false, false};
  }
  gH->oracles = std::make_unique<OracleSet>(opt);
  if (gH->oracles->needsBankState() && gH->bank.header().oracleSize == 0) {
    std::fprintf(stderr, "fmgc-fuzz: the whole-computer oracle needs a bank built with it (rebuild the bank)\n");
    std::exit(2);
  }
  gH->runner = std::make_unique<FuzzRunner>(gH->bank, *gH->oracles);
  if (const char* d = std::getenv("FMGC_FUZZ_FINDINGS")) {
    gH->findingsDir = d;
  }
  if (const char* a = std::getenv("FMGC_FUZZ_ABORT")) {
    gH->abortDiff = false;
    std::string s = a;
    size_t p = 0;
    while (p <= s.size()) {
      size_t q = s.find(',', p);
      std::string tok = s.substr(p, q == std::string::npos ? std::string::npos : q - p);
      if (tok == "diff") {
        gH->abortDiff = true;
      } else if (tok == "all") {
        gH->abortAll = true;
      } else if (!tok.empty() && tok != "none") {
        gH->abortChecks.insert(tok);
      }
      if (q == std::string::npos) {
        break;
      }
      p = q + 1;
    }
  }
  for (int i = 0; i < fmgc::props::kCount; i++) {
    gH->abortProp[static_cast<size_t>(i)] = gH->abortChecks.count(fmgc::props::list()[i].id) || gH->abortChecks.count("property");
  }
  if (const char* f = std::getenv("FMGC_FUZZ_STATE_FEEDBACK")) {
    gH->stateFeedback = std::atoi(f) != 0;
  }
}

uint64_t fnv(const uint8_t* d, size_t n) {
  uint64_t h = 1469598103934665603ULL;
  for (size_t i = 0; i < n; i++) {
    h = (h ^ d[i]) * 1099511628211ULL;
  }
  return h;
}

// Property findings are looked up by id (no string compares on the hot path: libFuzzer intercepts memcmp/strcmp and
// would fill its dictionary with property names).
bool shouldAbort(const Finding& f) {
  if (gH->abortAll || (f.differential && gH->abortDiff)) {
    return true;
  }
  if (f.propId >= 0) {
    return gH->abortProp[static_cast<size_t>(f.propId)];
  }
  return !gH->abortChecks.empty() && (gH->abortChecks.count(f.check) || gH->abortChecks.count(f.oracle));
}

void saveFinding(const Finding& f, const uint8_t* data, size_t size) {
  if (gH->findingsDir.empty()) {
    return;
  }
  int& n = f.propId >= 0 ? gH->savedProp[static_cast<size_t>(f.propId)] : gH->saved[f.check];
  if (n >= 3) {
    return;
  }
  if (n == 0) {
    // at most 30 reproducers per check in the directory (libFuzzer fork mode runs many short-lived processes)
    int existing = 0;
    if (DIR* d = opendir(gH->findingsDir.c_str())) {
      std::string prefix = f.check + "-";
      while (dirent* e = readdir(d)) {
        existing += std::strncmp(e->d_name, prefix.c_str(), prefix.size()) == 0 &&
                    std::strstr(e->d_name, ".bin") != nullptr;
      }
      closedir(d);
    }
    if (existing >= 30) {
      n = 3;
      return;
    }
  }
  n++;
  char name[64];
  std::snprintf(name, sizeof name, "%016llx", static_cast<unsigned long long>(fnv(data, size)));
  std::string base = gH->findingsDir + "/" + f.check + "-" + name;
  if (FILE* o = std::fopen((base + ".bin").c_str(), "wb")) {
    std::fwrite(data, 1, size, o);
    std::fclose(o);
  }
  if (FILE* o = std::fopen((base + ".txt").c_str(), "w")) {
    std::fprintf(o, "%s %s step %ld t %.3f %s\n", f.oracle.c_str(), f.check.c_str(), f.step, f.t, f.detail.c_str());
    std::fclose(o);
  }
}

}  // namespace

#ifndef FMGC_FUZZ_STANDALONE

// ---- libFuzzer ------------------------------------------------------------------------------------------------------

namespace {
constexpr size_t kVertN = 18, kLatN = 10;
constexpr size_t kOffVertTrans = 0;                                   // 18 x 18
constexpr size_t kOffLatTrans = kOffVertTrans + kVertN * kVertN;      // 10 x 10
constexpr size_t kOffPair = kOffLatTrans + kLatN * kLatN;             // 18 x 10
constexpr size_t kOffArmed = kOffPair + kVertN * kLatN;               // 1024
constexpr size_t kOffFma = kOffArmed + 1024;                          // 15 x 15
constexpr size_t kOffLaws = kOffFma + 15 * 15;                        // 10 x 7
constexpr size_t kOffProps = kOffLaws + 10 * 7;                       // properties: bad states
constexpr size_t kExtra = kOffProps + fmgc::props::kCount;

__attribute__((section("__libfuzzer_extra_counters"))) uint8_t gExtra[kExtra];

inline void bump(size_t i) {
  if (gExtra[i] < 255) {
    gExtra[i]++;
  }
}
inline size_t lowestBit(uint32_t m) { return m ? static_cast<size_t>(__builtin_ctz(m)) + 1 : 0; }
}  // namespace

extern "C" int LLVMFuzzerInitialize(int* argc, char*** argv) {
  setup(*argc > 0 ? (*argv)[0] : nullptr);
  if (gH->stateFeedback) {
    auto* props = gH->oracles->properties();
    gH->runner->observer = [props, prevV = size_t{0}, prevL = size_t{0}, prevFma = 0](
                               uint32_t step, const fmgc_inputs&, const fmgc_outputs& out, bool running) mutable {
      if (!running) {
        return;
      }
      size_t v = props ? lowestBit(props->properties().vertMask()) : 0;
      size_t l = props ? lowestBit(props->properties().latMask()) : 0;
      int fma = static_cast<int>(out.athr.fma_mode);
      if (step > 0) {
        bump(kOffVertTrans + prevV * kVertN + v);
        bump(kOffLatTrans + prevL * kLatN + l);
        bump(kOffFma + static_cast<size_t>(prevFma % 15) * 15 + static_cast<size_t>(fma % 15));
      }
      bump(kOffPair + v * kLatN + l);
      const auto& a = out.ap_fd_logic.armed_modes;
      uint32_t armed = a.alt_acq_armed | a.alt_acq_arm_possible << 1 | a.nav_armed << 2 | a.loc_armed << 3 |
                       a.land_armed << 4 | a.glide_armed << 5 | a.final_des_armed << 6 | a.clb_armed << 7 |
                       a.des_armed << 8 | a.tcas_armed << 9;
      bump(kOffArmed + armed);
      bump(kOffLaws + static_cast<size_t>(out.ap_fd_logic.active_longitudinal_law) % 10 * 7 +
           static_cast<size_t>(out.ap_fd_logic.active_lateral_law) % 7);
      prevV = v;
      prevL = l;
      prevFma = fma;
    };
  }
  return 0;
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  if (size < kHeaderSize) {
    return -1;  // reject: libFuzzer will not add it to the corpus
  }
  FuzzCase c = decode(data, size, gH->bank.snapshots());
  gH->runner->run(c, [&](const Finding& f) {
    if (gH->stateFeedback && f.propId >= 0) {
      bump(kOffProps + static_cast<size_t>(f.propId));
    }
    saveFinding(f, data, size);
    if (shouldAbort(f)) {
      std::fprintf(stderr, "==fmgc-fuzz== %s %s step %ld t %.3f %s\n", f.oracle.c_str(), f.check.c_str(), f.step, f.t,
                   f.detail.c_str());
      std::abort();
    }
  });
  return 0;
}

#else

// ---- standalone -----------------------------------------------------------------------------------------------------

namespace {

std::vector<uint8_t> readFile(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

void listInputs(const std::string& p, std::vector<std::string>& out) {
  struct stat st {};
  if (stat(p.c_str(), &st) != 0) {
    return;
  }
  if (S_ISDIR(st.st_mode)) {
    DIR* d = opendir(p.c_str());
    std::vector<std::string> names;
    while (dirent* e = readdir(d)) {
      if (e->d_name[0] != '.') {
        names.push_back(p + "/" + e->d_name);
      }
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    for (auto& n : names) {
      listInputs(n, out);
    }
  } else if (p.size() < 4 || p.compare(p.size() - 4, 4, ".txt") != 0) {
    out.push_back(p);
  }
}

std::string describe(const FuzzCase& c) {
  const BankSnapshot& s = gH->bank.snapshot(c.snapshot);
  char buf[256];
  std::snprintf(buf, sizeof buf, "snapshot %u (%s t=%.1f, %u frames) start %u steps %u scale %u seed %u, %zu events",
                c.snapshot, gH->bank.recordingName(s.recording), s.simTime, s.frames, c.start, c.steps, c.timeScale,
                c.seed, c.events.size());
  std::string out = buf;
  for (const Event& e : c.events) {
    std::snprintf(buf, sizeof buf, "\n  step %5u +%4u %-15s a=%3u b=%5u c=%5u", e.start, e.duration, kindName(e.kind),
                  e.a, e.b, e.c);
    out += buf;
  }
  return out;
}

/** First step at which `check` fires, or -1. */
long firstHit(const FuzzCase& c, const std::string& check) {
  long hit = -1;
  gH->runner->run(c, [&](const Finding& f) {
    if (hit < 0 && (f.check == check || f.oracle == check)) {
      hit = f.step;
    }
  });
  return hit;
}

int cmdRun(const std::vector<std::string>& paths, bool trace, bool summary) {
  std::vector<std::string> files;
  for (auto& p : paths) {
    listInputs(p, files);
  }
  std::map<std::string, long> perCheck;
  long steps = 0;
  for (const std::string& file : files) {
    std::vector<uint8_t> data = readFile(file);
    if (data.size() < kHeaderSize) {
      continue;
    }
    FuzzCase c = decode(data.data(), data.size(), gH->bank.snapshots());
    if (!summary || trace) {
      std::printf("== %s\n%s\n", file.c_str(), describe(c).c_str());
    }
    if (trace) {
      auto* props = gH->oracles->properties();
      gH->runner->observer = [props, prev = std::string()](uint32_t step, const fmgc_inputs& in,
                                                           const fmgc_outputs& out, bool running) mutable {
        const auto& ap = out.ap_fd_logic;
        char buf[512];
        std::snprintf(buf, sizeof buf, "%s | %s | law %d/%d | athr %d%s | AP %d FD %d ATHR %d | H %.0f FCU %.0f tgt %.0f V_c %.1f",
                      running ? fmgc::props::Properties::vertNames(props->properties().vertMask()).c_str() : "(off)",
                      running ? fmgc::props::Properties::latNames(props->properties().latMask()).c_str() : "",
                      static_cast<int>(ap.active_longitudinal_law), static_cast<int>(ap.active_lateral_law),
                      static_cast<int>(out.athr.fma_mode), out.athr.athr_active ? "*" : "", out.logic.ap_own_engaged,
                      out.logic.fd_own_engaged, out.logic.athr_own_engaged, out.logic.altitude_indicated_ft.Data,
                      in.bus_inputs.fcu_bus.selected_alt_ft.Data, ap.alt_sel_or_cstr, ap.spd_target_kts);
        std::string modes = buf;
        if (modes.substr(0, modes.find(" | H ")) != prev.substr(0, prev.find(" | H ")) || step == 0) {
          std::printf("  %5u t=%9.3f dt=%.3f  %s\n", step, in.time.simulation_time, in.time.dt, modes.c_str());
        }
        prev = modes;
      };
    }
    std::set<std::string> seen;
    steps += gH->runner->run(c, [&](const Finding& f) {
      perCheck[f.check]++;
      if (!summary && seen.insert(f.check).second) {
        std::printf("  FINDING %s %s step %ld t %.3f %s\n", f.oracle.c_str(), f.check.c_str(), f.step, f.t,
                    f.detail.c_str());
      }
    });
    gH->runner->observer = nullptr;
  }
  std::printf("\n%zu inputs, %ld steps\n", files.size(), steps);
  for (auto& [k, n] : perCheck) {
    std::printf("finding %-40s %ld\n", k.c_str(), n);
  }
  if (summary && gH->oracles->properties()) {
    std::printf("\n");
    gH->oracles->properties()->properties().print(stdout);
  }
  return 0;
}

int cmdMinimize(const std::string& check, const std::string& in, const std::string& outPath) {
  std::vector<uint8_t> data = readFile(in);
  FuzzCase c = decode(data.data(), data.size(), gH->bank.snapshots());
  long hit = firstHit(c, check);
  if (hit < 0) {
    std::fprintf(stderr, "%s does not trigger %s\n", in.c_str(), check.c_str());
    return 1;
  }
  std::printf("before: %zu bytes, %zu events, %u steps, hit at step %ld\n", data.size(), c.events.size(), c.steps, hit);
  bool changed = true;
  while (changed) {
    changed = false;
    // cut the run just after the hit
    if (c.steps > static_cast<uint32_t>(hit) + 1) {
      FuzzCase t = c;
      t.steps = std::max<uint32_t>(32, static_cast<uint32_t>(hit) + 1);
      t = decode(encode(t).data(), encode(t).size(), gH->bank.snapshots());
      long h = firstHit(t, check);
      if (h >= 0 && t.steps < c.steps) {
        c = t, hit = h, changed = true;
      }
    }
    // drop events, last first
    for (size_t i = c.events.size(); i-- > 0;) {
      FuzzCase t = c;
      t.events.erase(t.events.begin() + static_cast<long>(i));
      auto bytes = encode(t);
      t = decode(bytes.data(), bytes.size(), gH->bank.snapshots());
      long h = firstHit(t, check);
      if (h >= 0) {
        c = t, hit = h, changed = true;
      }
    }
    // shorten durations
    for (size_t i = 0; i < c.events.size(); i++) {
      if (c.events[i].duration <= 1) {
        continue;
      }
      FuzzCase t = c;
      t.events[i].duration = std::max<uint32_t>(1, t.events[i].duration / 2);
      auto bytes = encode(t);
      t = decode(bytes.data(), bytes.size(), gH->bank.snapshots());
      long h = firstHit(t, check);
      if (h >= 0 && t.events[i].duration < c.events[i].duration) {
        c = t, hit = h, changed = true;
      }
    }
    // simpler base: time scale 1
    if (c.timeScale > 1) {
      FuzzCase t = c;
      t.timeScale = 1;
      long h = firstHit(t, check);
      if (h >= 0) {
        c = t, hit = h, changed = true;
      }
    }
  }
  auto bytes = encode(c);
  if (FILE* o = std::fopen(outPath.c_str(), "wb")) {
    std::fwrite(bytes.data(), 1, bytes.size(), o);
    std::fclose(o);
  }
  std::printf("after: %zu bytes, %zu events, %u steps, hit at step %ld\n%s\n", bytes.size(), c.events.size(), c.steps,
              hit, describe(c).c_str());
  return 0;
}

int cmdSeeds(const std::string& dir, int extra) {
  mkdir(dir.c_str(), 0755);
  std::mt19937_64 rng(12345);
  auto write = [&](const FuzzCase& c, const std::string& name) {
    auto bytes = encode(c);
    if (FILE* o = std::fopen((dir + "/" + name).c_str(), "wb")) {
      std::fwrite(bytes.data(), 1, bytes.size(), o);
      std::fclose(o);
    }
  };
  std::map<uint32_t, std::vector<uint32_t>> byRecording;
  long n = 0;
  for (uint32_t i = 0; i < gH->bank.snapshots(); i++) {
    const BankSnapshot& s = gH->bank.snapshot(i);
    byRecording[s.recording].push_back(i);
    FuzzCase c;
    c.snapshot = i;
    c.steps = std::min<uint32_t>(2072, std::max<uint32_t>(32, s.frames));
    char name[64];
    std::snprintf(name, sizeof name, "plain-%04u", i);
    write(c, name);
    n++;
  }
  // event seeds: per recording, each scenario injector once, and a few FCU / button sequences
  for (auto& [rec, snaps] : byRecording) {
    for (int k = 0; k < 9 + extra; k++) {
      FuzzCase c;
      c.snapshot = snaps[rng() % snaps.size()];
      c.steps = 1000;
      c.seed = static_cast<uint32_t>(rng() & 0xffffff);
      Event e{0, 2160, Kind::Scenario, static_cast<uint8_t>(k % 9), static_cast<uint16_t>(rng()),
              static_cast<uint16_t>(rng())};
      c.events.push_back(e);
      if (k >= 9) {  // extra: add pilot actions on top
        uint32_t at = 0;
        for (int j = 0; j < 4; j++) {
          at += 50 + rng() % 200;
          static const Kind kinds[] = {Kind::FcuPush, Kind::FcuAltitude, Kind::Buttons, Kind::FmsConstraint,
                                       Kind::FcuVs, Kind::FcuHdgVsTrkFpa, Kind::FmsPhase, Kind::Levers};
          Event p{at, static_cast<uint32_t>(5 + rng() % 20), kinds[rng() % 8], static_cast<uint8_t>(rng()),
                  static_cast<uint16_t>(rng()), static_cast<uint16_t>(rng())};
          if (p.kind == Kind::FcuAltitude || p.kind == Kind::FmsConstraint) {
            p.duration = 2000;
            p.a |= 1;
            p.b = static_cast<uint16_t>(static_cast<int16_t>(static_cast<int>(rng() % 6000) - 3000));
          }
          c.events.push_back(p);
        }
      }
      char name[64];
      std::snprintf(name, sizeof name, "scen-%02u-%02d", rec, k);
      write(c, name);
      n++;
    }
  }
  std::printf("%ld seeds in %s\n", n, dir.c_str());
  return 0;
}

int cmdInfo() {
  const BankHeader& h = gH->bank.header();
  std::printf("bank: %u snapshots from %u recordings, interval %.0f s, window %.0f s, decimation %u, oracle state %u\n",
              h.snapshots, h.recordings, h.intervalS, h.windowS, h.decimation, h.oracleSize);
  for (uint32_t i = 0; i < h.snapshots; i++) {
    const BankSnapshot& s = gH->bank.snapshot(i);
    std::printf("%4u %-28s step %9lld t %9.1f frames %u\n", i, gH->bank.recordingName(s.recording),
                static_cast<long long>(s.replayStep), s.simTime, s.frames);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: fmgc-fuzz-run run [--trace] [--summary] FILE|DIR... | minimize CHECK IN OUT | seeds DIR [N] | "
                 "info\n");
    return 2;
  }
  std::string cmd = argv[1];
  setup(argv[0]);
  if (cmd == "run") {
    bool trace = false, summary = false;
    std::vector<std::string> paths;
    for (int i = 2; i < argc; i++) {
      std::string a = argv[i];
      if (a == "--trace") {
        trace = true;
      } else if (a == "--summary") {
        summary = true;
      } else {
        paths.push_back(a);
      }
    }
    return cmdRun(paths, trace, summary);
  }
  if (cmd == "minimize" && argc == 5) {
    return cmdMinimize(argv[2], argv[3], argv[4]);
  }
  if (cmd == "seeds" && argc >= 3) {
    return cmdSeeds(argv[2], argc >= 4 ? std::atoi(argv[3]) : 3);
  }
  if (cmd == "info") {
    return cmdInfo();
  }
  std::fprintf(stderr, "unknown command\n");
  return 2;
}

#endif
