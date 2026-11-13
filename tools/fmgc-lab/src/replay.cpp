// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Replays a flight data recording through the generated FmgcComputer and compares every output field with what the
// sim recorded. Inputs are rebuilt from the recording exactly as FlyByWireInterface::updateFmgc() sets them.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "FmgcComputer.h"
#include "fdr.h"
#include "fields.gen.h"

namespace {

struct Options {
  std::string path;
  long from = 0;
  long to = -1;
  double absTol = 1e-6;
  double relTol = 1e-6;
  int maxFields = 60;
  bool assumeRunning = false;
  bool fillGaps = true;
  std::string filter;
};

struct FieldStats {
  long mismatchFrames = 0;
  long firstFrame = -1;
  double firstTime = 0;
  double firstExpected = 0;
  double firstActual = 0;
  double maxAbsDiff = 0;
  long lastFrame = -1;
};

/** Compares two copies of a struct field by field (a = recorded, b = replayed). */
struct Comparator {
  const Options& opt;
  std::map<std::string, FieldStats>& stats;
  long frame;
  double time;
  bool anyMismatch = false;

  template <typename T>
  void field(const char* prefix, const char* name, T& expected, T& actual, FieldKind kind) {
    double e = toDouble(expected);
    double a = toDouble(actual);
    bool equal;
    if (kind == FieldKind::Float) {
      if (std::isnan(e) || std::isnan(a)) {
        equal = std::isnan(e) && std::isnan(a);
      } else {
        double scale = std::fmax(1.0, std::fmax(std::fabs(e), std::fabs(a)));
        equal = std::fabs(e - a) <= opt.absTol || std::fabs(e - a) <= opt.relTol * scale;
      }
    } else {
      equal = e == a;
    }
    if (equal) {
      return;
    }
    std::string key = std::string(prefix) + name;
    if (!opt.filter.empty() && key.find(opt.filter) == std::string::npos) {
      return;
    }
    anyMismatch = true;
    FieldStats& s = stats[key];
    if (s.firstFrame < 0) {
      s.firstFrame = frame;
      s.firstTime = time;
      s.firstExpected = e;
      s.firstActual = a;
    }
    s.mismatchFrames++;
    s.lastFrame = frame;
    if (!std::isnan(e) && !std::isnan(a)) {
      s.maxAbsDiff = std::fmax(s.maxAbsDiff, std::fabs(e - a));
    }
  }

  template <typename T>
  static double toDouble(const T& v) {
    if constexpr (std::is_enum_v<T>) {
      return static_cast<double>(static_cast<int>(v));
    } else {
      return static_cast<double>(v);
    }
  }
};

void usage() {
  std::fprintf(stderr,
               "usage: fmgc-replay [options] <file.fdr>\n"
               "  --from N / --to N      frame range to compare (all frames are still stepped)\n"
               "  --abs-tol X            absolute tolerance for floats (default 1e-6)\n"
               "  --rel-tol X            relative tolerance for floats (default 1e-6)\n"
               "  --max-fields N         fields to list in the report (default 60)\n"
               "  --filter S             only compare fields whose name contains S\n"
               "  --assume-running       treat the FMGC as running on every frame\n"
               "  --no-fill-gaps         do not step through unrecorded frames (sim time jumps)\n");
}

bool parseArgs(int argc, char** argv, Options& opt) {
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    const char* v = nullptr;
    if (a == "--from" && (v = next())) {
      opt.from = std::atol(v);
    } else if (a == "--to" && (v = next())) {
      opt.to = std::atol(v);
    } else if (a == "--abs-tol" && (v = next())) {
      opt.absTol = std::atof(v);
    } else if (a == "--rel-tol" && (v = next())) {
      opt.relTol = std::atof(v);
    } else if (a == "--max-fields" && (v = next())) {
      opt.maxFields = std::atoi(v);
    } else if (a == "--filter" && (v = next())) {
      opt.filter = v;
    } else if (a == "--no-fill-gaps") {
      opt.fillGaps = false;
    } else if (a == "--assume-running") {
      opt.assumeRunning = true;
    } else if (!a.empty() && a[0] != '-' && opt.path.empty()) {
      opt.path = a;
    } else {
      return false;
    }
  }
  return !opt.path.empty();
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, opt)) {
    usage();
    return 2;
  }

  FdrReader reader(opt.path);
  if (!reader.ok()) {
    std::fprintf(stderr, "error: %s\n", reader.error().c_str());
    return 2;
  }

  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();

  auto frame = std::make_unique<FdrFrame>();
  auto previous = std::make_unique<FmgcRecord>();
  std::memset(previous.get(), 0, sizeof(FmgcRecord));
  auto inputs = std::make_unique<FmgcComputer::ExternalInputs_FmgcComputer_T>();
  std::memset(inputs.get(), 0, sizeof(*inputs));

  std::map<std::string, FieldStats> stats;
  long frames = 0, compared = 0, mismatchFrames = 0, notRunningFrames = 0;
  long firstMismatchFrame = -1;
  long gaps = 0, gapSteps = 0;
  std::vector<std::pair<long, double>> gapList;
  double monotonicTime = 0;
  auto start = std::chrono::steady_clock::now();
  double stepSeconds = 0;

  for (; reader.next(*frame); frames++) {
    const BaseData& base = frame->base;
    FmgcRecord& rec = frame->fmgc1;

    // When the computer is not running, step() leaves its outputs untouched, so the recorded FMGC block (including
    // the copy of its inputs) is frozen. Any change means it ran this frame.
    bool running = opt.assumeRunning || frames == 0 || std::memcmp(&rec, previous.get(), sizeof(FmgcRecord)) != 0;
    if (!running) {
      notRunningFrames++;
    }

    auto& in = inputs->in;

    // Unrecorded frames: the recorder is skipped during active pause while the computers keep running, which shows
    // as a jump in simulation time larger than this frame's dt. Fill the gap by stepping with the last inputs held
    // (the aircraft is frozen in active pause), at the last frame's dt.
    if (frames > 0 && opt.fillGaps) {
      double gap = base.simulation_time_s - in.time.simulation_time - base.simulation_delta_time_s;
      double fillDt = in.time.dt > 0 ? in.time.dt : base.simulation_delta_time_s;
      if (gap > 0.1 && fillDt > 0) {
        long steps = std::lround(gap / fillDt);
        fillDt = gap / steps;
        for (long i = 0; i < steps; i++) {
          monotonicTime += fillDt;
          in.time.dt = fillDt;
          in.time.simulation_time += fillDt;
          in.time.monotonic_time = monotonicTime;
          computer->setExternalInputs(inputs.get());
          computer->step();
        }
        gaps++;
        gapSteps += steps;
        gapList.push_back({frames, gap});
      }
    }

    monotonicTime += base.simulation_delta_time_s;
    in.time.dt = base.simulation_delta_time_s;
    in.time.simulation_time = base.simulation_time_s;
    in.time.monotonic_time = monotonicTime;
    // The laws are not run (and nothing is recorded) during slew and pause, so these are false on recorded frames.
    // FIXME slew_on is true on the first frame after a slew; not recoverable from the recording.
    in.sim_data.slew_on = false;
    in.sim_data.pause_on = false;
    in.sim_data.tracking_mode_on_override = false;
    in.sim_data.tailstrike_protection_on = false;
    in.sim_data.computer_running = running;
    in.discrete_inputs = rec.discrete_inputs;
    in.fms_inputs = rec.fms_inputs;
    in.bus_inputs = rec.bus_inputs;

    auto t0 = std::chrono::steady_clock::now();
    computer->setExternalInputs(inputs.get());
    computer->step();
    stepSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (frames >= opt.from && (opt.to < 0 || frames <= opt.to) && running) {
      fmgc_outputs out = computer->getExternalOutputs().out;
      Comparator cmp{opt, stats, frames, base.simulation_time_s};
      visit(cmp, "logic.", rec.logic, out.logic);
      visit(cmp, "ap_fd_logic.", rec.ap_fd_logic, out.ap_fd_logic);
      visit(cmp, "ap_fd_outer_loops.", rec.ap_fd_outer_loops, out.ap_fd_outer_loops);
      visit(cmp, "athr.", rec.athr, out.athr);
      visit(cmp, "discrete_outputs.", rec.discrete_outputs, out.discrete_outputs);
      visit(cmp, "bus_outputs.", rec.bus_outputs, out.bus_outputs);
      compared++;
      if (cmp.anyMismatch) {
        mismatchFrames++;
        if (firstMismatchFrame < 0) {
          firstMismatchFrame = frames;
        }
      }
    }

    *previous = rec;
  }

  double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  std::printf("file             %s\n", opt.path.c_str());
  std::printf("frames           %ld (%ld not running), compared %ld\n", frames, notRunningFrames, compared);
  std::printf("gaps filled      %ld (%ld steps with held inputs)", gaps, gapSteps);
  for (auto& [f, g] : gapList) {
    std::printf(" [frame %ld: %.2f s]", f, g);
  }
  std::printf("\n");
  std::printf("speed            %.0f steps/s (step only), %.0f frames/s (incl. decompress + compare)\n",
              frames / std::fmax(stepSeconds, 1e-9), frames / std::fmax(wall, 1e-9));
  std::printf("mismatch frames  %ld (%.3f%%), first at frame %ld\n", mismatchFrames,
              compared ? 100.0 * mismatchFrames / compared : 0.0, firstMismatchFrame);
  std::printf("fields differing %zu\n", stats.size());

  std::vector<std::pair<std::string, FieldStats>> sorted(stats.begin(), stats.end());
  std::sort(sorted.begin(), sorted.end(), [](auto& a, auto& b) { return a.second.firstFrame < b.second.firstFrame; });
  int shown = 0;
  if (!sorted.empty()) {
    std::printf("\n%-62s %9s %9s %10s %14s %14s %12s\n", "field (by first mismatch)", "frames", "first", "t_first",
                "recorded", "replayed", "max |diff|");
  }
  for (auto& [name, s] : sorted) {
    if (shown++ >= opt.maxFields) {
      std::printf("... %zu more\n", sorted.size() - opt.maxFields);
      break;
    }
    std::printf("%-62s %9ld %9ld %9ld %10.2f %14.6g %14.6g %12.4g\n", name.c_str(), s.mismatchFrames, s.firstFrame, s.lastFrame,
                s.firstTime, s.firstExpected, s.firstActual, s.maxAbsDiff);
  }

  return mismatchFrames == 0 ? 0 : 1;
}
