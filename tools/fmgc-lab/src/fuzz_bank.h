// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Snapshot bank for the fuzzer: generated-model states taken while replaying the recordings, each followed by a window
// of the inputs the replay gave the model next. A fuzz input picks a snapshot, restores the model state (a 10 KB copy)
// and runs from there, so no run re-replays a recording from frame 0. The file is mmap'ed read-only and shared, so
// several fuzz workers share one copy in the page cache.
//
// Layout (native endianness, one file, built by fmgc-fuzz-bank from the corpus):
//   BankHeader
//   per snapshot, at the offsets in its BankSnapshot:
//     FmgcComputer state (sizeof(FmgcComputer) bytes: the whole object, copyable by memcpy: no pointers)
//     oracle state (oracleSize bytes; 0 unless a stateful oracle such as the whole-computer differential is built in)
//     fmgc_inputs window[frames]                exactly what the replay passed to step(), decimated (see below)
//   BankSnapshot[snapshots]                     at tableOffset
//   char names[recordings][128]                 recording base names, at namesOffset
//
// Windows are decimated by `decimation`: each stored frame stands for `decimation` consecutive replay steps (inputs of
// the last one, dt summed). Any input is a valid test input for the generated model (it is the oracle), so this only
// trades time resolution for memory; the snapshot state itself is exact.

#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "FmgcComputer.h"

namespace fmgc::fuzz {

constexpr char kBankMagic[8] = {'F', 'M', 'G', 'C', 'B', 'N', 'K', '1'};

struct BankHeader {
  char magic[8];
  uint32_t inputsSize;    // sizeof(fmgc_inputs)
  uint32_t computerSize;  // sizeof(FmgcComputer)
  uint32_t snapshots;
  uint32_t recordings;
  uint32_t oracleSize;    // bytes of oracle state per snapshot
  uint32_t decimation;
  double intervalS;       // snapshot spacing in recording time
  double windowS;
  uint64_t tableOffset;
  uint64_t namesOffset;
};

struct BankSnapshot {
  uint32_t recording;
  uint32_t frames;        // stored (decimated) input frames
  int64_t replayStep;     // replay step index of the snapshot (steps before it, incl. gap fills)
  double simTime;         // simulation time of the first window frame
  uint64_t stateOffset;   // FmgcComputer bytes
  uint64_t oracleOffset;
  uint64_t framesOffset;  // fmgc_inputs[frames]
};

class Bank {
 public:
  Bank() = default;
  Bank(const Bank&) = delete;
  Bank& operator=(const Bank&) = delete;
  ~Bank() {
    if (base_) {
      munmap(const_cast<uint8_t*>(base_), size_);
    }
  }

  bool open(const std::string& path, std::string& error) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
      error = "cannot open bank " + path;
      return false;
    }
    struct stat st {};
    fstat(fd, &st);
    size_ = static_cast<size_t>(st.st_size);
    void* p = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED || size_ < sizeof(BankHeader)) {
      error = "cannot map bank " + path;
      base_ = nullptr;
      return false;
    }
    base_ = static_cast<const uint8_t*>(p);
    header_ = reinterpret_cast<const BankHeader*>(base_);
    if (std::memcmp(header_->magic, kBankMagic, 8) != 0 || header_->inputsSize != sizeof(fmgc_inputs) ||
        header_->computerSize != sizeof(FmgcComputer)) {
      error = "bank " + path + " does not match this build (rebuild it)";
      return false;
    }
    snaps_ = reinterpret_cast<const BankSnapshot*>(base_ + header_->tableOffset);
    names_ = reinterpret_cast<const char*>(base_ + header_->namesOffset);
    return true;
  }

  const BankHeader& header() const { return *header_; }
  uint32_t snapshots() const { return header_->snapshots; }
  const BankSnapshot& snapshot(uint32_t i) const { return snaps_[i]; }
  const char* recordingName(uint32_t r) const { return names_ + 128 * r; }
  const uint8_t* state(uint32_t i) const { return base_ + snaps_[i].stateOffset; }
  const uint8_t* oracleState(uint32_t i) const { return base_ + snaps_[i].oracleOffset; }
  const fmgc_inputs* frames(uint32_t i) const {
    return reinterpret_cast<const fmgc_inputs*>(base_ + snaps_[i].framesOffset);
  }

  /** Restores the generated model to snapshot i. */
  void restore(uint32_t i, FmgcComputer& computer) const {
    std::memcpy(static_cast<void*>(&computer), state(i), sizeof(FmgcComputer));
  }

 private:
  const uint8_t* base_ = nullptr;
  size_t size_ = 0;
  const BankHeader* header_ = nullptr;
  const BankSnapshot* snaps_ = nullptr;
  const char* names_ = nullptr;
};

}  // namespace fmgc::fuzz
