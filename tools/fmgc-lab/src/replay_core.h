// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Drives a generated FmgcComputer from a flight data recording: rebuilds fmgc_inputs per frame the way
// FlyByWireInterface::updateFmgc() sets them, detects frames where the computer was not running, and fills unrecorded
// gaps (active pause) by stepping with the last inputs held. Shared by the replay tool and probes.

#pragma once

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "FmgcComputer.h"
#include "fdr.h"

class Replayer {
 public:
  struct Options {
    bool assumeRunning = false;
    bool fillGaps = true;
  };

  /** Called after each recorded frame is stepped: frame index, the frame, whether the computer was running. */
  struct Frame {
    long index;
    const FdrFrame* fdr;
    bool running;
  };

  Replayer(const std::string& path, Options opt, FmgcComputer& computer)
      : reader_(path), opt_(opt), computer_(computer) {
    frame_ = std::make_unique<FdrFrame>();
    previous_ = std::make_unique<FmgcRecord>();
    std::memset(previous_.get(), 0, sizeof(FmgcRecord));
    inputs_ = std::make_unique<FmgcComputer::ExternalInputs_FmgcComputer_T>();
    std::memset(inputs_.get(), 0, sizeof(*inputs_));
  }

  bool ok() const { return reader_.ok(); }
  const std::string& error() const { return reader_.error(); }

  long frames() const { return frames_; }
  long notRunningFrames() const { return notRunning_; }
  long gapSteps() const { return gapSteps_; }
  const std::vector<std::pair<long, double>>& gaps() const { return gapList_; }
  double stepSeconds() const { return stepSeconds_; }

  /** Steps the next recorded frame; false at the end of the file. */
  bool next(Frame& out) {
    if (!reader_.next(*frame_)) {
      return false;
    }
    const BaseData& base = frame_->base;
    FmgcRecord& rec = frame_->fmgc1;

    // When the computer is not running, step() leaves its outputs untouched, so the recorded FMGC block (including
    // the copy of its inputs) is frozen. Any change means it ran this frame.
    bool running = opt_.assumeRunning || frames_ == 0 || std::memcmp(&rec, previous_.get(), sizeof(FmgcRecord)) != 0;
    if (!running) {
      notRunning_++;
    }

    auto& in = inputs_->in;

    // Unrecorded frames: the recorder is skipped during active pause while the computers keep running, which shows
    // as a jump in simulation time larger than this frame's dt. Fill the gap by stepping with the last inputs held
    // (the aircraft is frozen in active pause), at the last frame's dt.
    if (frames_ > 0 && opt_.fillGaps) {
      double gap = base.simulation_time_s - in.time.simulation_time - base.simulation_delta_time_s;
      double fillDt = in.time.dt > 0 ? in.time.dt : base.simulation_delta_time_s;
      if (gap > 0.1 && fillDt > 0) {
        long steps = std::lround(gap / fillDt);
        fillDt = gap / steps;
        for (long i = 0; i < steps; i++) {
          monotonicTime_ += fillDt;
          in.time.dt = fillDt;
          in.time.simulation_time += fillDt;
          in.time.monotonic_time = monotonicTime_;
          computer_.setExternalInputs(inputs_.get());
          computer_.step();
        }
        gapSteps_ += steps;
        gapList_.push_back({frames_, gap});
      }
    }

    monotonicTime_ += base.simulation_delta_time_s;
    in.time.dt = base.simulation_delta_time_s;
    in.time.simulation_time = base.simulation_time_s;
    in.time.monotonic_time = monotonicTime_;
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
    computer_.setExternalInputs(inputs_.get());
    computer_.step();
    stepSeconds_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    out = {frames_, frame_.get(), running};
    *previous_ = rec;
    frames_++;
    return true;
  }

 private:
  FdrReader reader_;
  Options opt_;
  FmgcComputer& computer_;
  std::unique_ptr<FdrFrame> frame_;
  std::unique_ptr<FmgcRecord> previous_;
  std::unique_ptr<FmgcComputer::ExternalInputs_FmgcComputer_T> inputs_;
  long frames_ = 0, notRunning_ = 0, gapSteps_ = 0;
  std::vector<std::pair<long, double>> gapList_;
  double monotonicTime_ = 0;
  double stepSeconds_ = 0;
};
