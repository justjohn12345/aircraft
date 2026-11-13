// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Reader for A32NX flight data recorder files (.fdr): a gzip stream holding the interface version (uint64) followed
// by one record per frame, written by FlightDataRecorder::update(). The layout must match the recorder of the
// interface version below.

#pragma once

#include <zlib.h>
#include <cstdint>
#include <string>

#include "ElacComputer_types.h"
#include "FacComputer_types.h"
#include "FadecComputer_types.h"
#include "FmgcComputer_types.h"
#include "SecComputer_types.h"
#include "recording/RecordingDataTypes.h"

constexpr uint64_t FDR_INTERFACE_VERSION = 3200006;

struct ElacRecord {
  base_elac_out_bus bus_outputs;
  base_elac_discrete_outputs discrete_outputs;
  base_elac_analog_outputs analog_outputs;
};

struct SecRecord {
  base_sec_out_bus bus_outputs;
  base_sec_discrete_outputs discrete_outputs;
  base_sec_analog_outputs analog_outputs;
};

struct FacRecord {
  base_fac_bus bus_outputs;
  base_fac_discrete_outputs discrete_outputs;
  base_fac_analog_outputs analog_outputs;
};

/** The FMGC 1 part of a record, in file order. */
struct FmgcRecord {
  base_fmgc_logic_outputs logic;
  base_fmgc_ap_fd_logic_outputs ap_fd_logic;
  ap_raw_output ap_fd_outer_loops;
  base_fmgc_athr_outputs athr;
  base_fmgc_discrete_outputs discrete_outputs;
  base_fmgc_bus_outputs bus_outputs;
  base_fmgc_bus_inputs bus_inputs;
  base_fmgc_discrete_inputs discrete_inputs;
  base_fms_inputs fms_inputs;
};

struct FadecRecord {
  base_ecu_bus bus_outputs;
  athr_output outputs;
};

struct FdrFrame {
  BaseData base;
  AircraftSpecificData specific;
  ElacRecord elac[2];
  SecRecord sec[3];
  FacRecord fac[2];
  FmgcRecord fmgc1;
  FadecRecord fadec[1];
};

class FdrReader {
 public:
  explicit FdrReader(const std::string& path) {
    file_ = gzopen(path.c_str(), "rb");
    if (!file_) {
      error_ = "cannot open " + path;
      return;
    }
    gzbuffer(file_, 1 << 20);
    if (!readRaw(&version_, sizeof(version_))) {
      error_ = "cannot read interface version";
    } else if (version_ != FDR_INTERFACE_VERSION) {
      error_ = "unsupported interface version " + std::to_string(version_);
    }
  }

  ~FdrReader() {
    if (file_) {
      gzclose(file_);
    }
  }

  FdrReader(const FdrReader&) = delete;
  FdrReader& operator=(const FdrReader&) = delete;

  bool ok() const { return error_.empty(); }
  const std::string& error() const { return error_; }
  uint64_t version() const { return version_; }

  /** Reads the next frame; false at the end of the file (a truncated last frame is ignored). */
  bool next(FdrFrame& frame) {
    return readField(frame.base) && readField(frame.specific) && readElac(frame.elac[0]) && readElac(frame.elac[1]) &&
           readSec(frame.sec[0]) && readSec(frame.sec[1]) && readSec(frame.sec[2]) && readFac(frame.fac[0]) &&
           readFac(frame.fac[1]) && readFmgc(frame.fmgc1) && readField(frame.fadec[0].bus_outputs) &&
           readField(frame.fadec[0].outputs);
  }

 private:
  gzFile file_ = nullptr;
  uint64_t version_ = 0;
  std::string error_;

  bool readRaw(void* dst, size_t size) { return gzread(file_, dst, static_cast<unsigned>(size)) == static_cast<int>(size); }

  // Fields are written one by one (no padding between them), so read them one by one too.
  template <typename T>
  bool readField(T& value) {
    return readRaw(&value, sizeof(T));
  }

  bool readElac(ElacRecord& r) { return readField(r.bus_outputs) && readField(r.discrete_outputs) && readField(r.analog_outputs); }
  bool readSec(SecRecord& r) { return readField(r.bus_outputs) && readField(r.discrete_outputs) && readField(r.analog_outputs); }
  bool readFac(FacRecord& r) { return readField(r.bus_outputs) && readField(r.discrete_outputs) && readField(r.analog_outputs); }

  bool readFmgc(FmgcRecord& r) {
    return readField(r.logic) && readField(r.ap_fd_logic) && readField(r.ap_fd_outer_loops) && readField(r.athr) &&
           readField(r.discrete_outputs) && readField(r.bus_outputs) && readField(r.bus_inputs) && readField(r.discrete_inputs) &&
           readField(r.fms_inputs);
  }
};
