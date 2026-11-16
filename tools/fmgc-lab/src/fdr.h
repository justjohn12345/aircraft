// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Reader for A32NX flight data recorder files (.fdr): a gzip stream holding the interface version (uint64) followed
// by one record per frame, written by FlightDataRecorder::update(). The layout must match the recorder of one of the
// interface versions below: 3200006 (stock), 3209001 (personal build: 3200006 plus the FCU at the end of the record) or
// 3209002 (personal build: 3209001 plus the FMS VNAV state).

#pragma once

#include <zlib.h>
#include <cstdint>
#include <string>

#include "ElacComputer_types.h"
#include "FacComputer_types.h"
#include "FadecComputer_types.h"
#include "FcuComputer_types.h"
#include "FmgcComputer_types.h"
#include "SecComputer_types.h"
#include "recording/RecordingDataTypes.h"

constexpr uint64_t FDR_INTERFACE_VERSION = 3200006;
constexpr uint64_t FDR_INTERFACE_VERSION_FCU = 3209001;
constexpr uint64_t FDR_INTERFACE_VERSION_FCU_VNAV = 3209002;
// 3209002 plus the descent/MCDU profile diagnostics (FmsVnavDiagData, 20 doubles; personal/fdr-vnav-diag)
constexpr uint64_t FDR_INTERFACE_VERSION_FCU_VNAV_DIAG = 3209003;
constexpr size_t FDR_FMS_VNAV_DIAG_COUNT = 20;

/**
 * The FMS VNAV state of a 3209002 record: a copy of FmsVnavData (RecordingDataTypes.h of the personal/fdr-vnav branch),
 * read from the L:A32NX_FDR_VNAV_* variables. Distances in NM, altitudes in ft, -1 when not available.
 */
struct FmsVnavRecord {
  double distance_to_end_nm;
  double total_distance_nm;
  double along_path_nm;
  double active_leg_index;
  double plan_version;
  double descent_constraint_count;
  double next_descent_constraint_distance_nm;
  double next_descent_constraint_alt_ft;
  double profile_valid;
  double profile_total_nm;
  double profile_position_nm;
  double profile_tod_nm;
  double profile_end_nm;
  double profile_target_alt_ft;
  double linear_deviation_ft;
  double descent_profile_compute_count;
  double cost_index;
  double zero_fuel_weight_t;
  double cruise_level;
  double managed_descent_speed_kts;
  double approach_selected;
  double stock_constraint_alt_ft;
};
static_assert(sizeof(FmsVnavRecord) == 22 * sizeof(double));

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

/** The FCU part of a 3209001 record: the model inputs of its last step, and its outputs except `data`. */
struct FcuRecord {
  fcu_inputs inputs;
  base_fcu_logic_outputs logic;
  base_fcu_discrete_outputs discrete_outputs;
  base_fcu_bus bus_outputs;
};

struct FdrFrame {
  BaseData base;
  AircraftSpecificData specific;
  ElacRecord elac[2];
  SecRecord sec[3];
  FacRecord fac[2];
  FmgcRecord fmgc1;
  FadecRecord fadec[1];
  bool hasFcu;  // 3209001 and 3209002
  FcuRecord fcu;
  bool hasFmsVnav;  // 3209002 and 3209003
  FmsVnavRecord fmsVnav;
  bool hasFmsVnavDiag;  // 3209003 only, the fields of FmsVnavDiagData in order
  double fmsVnavDiag[FDR_FMS_VNAV_DIAG_COUNT];
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
    } else if (version_ != FDR_INTERFACE_VERSION && version_ != FDR_INTERFACE_VERSION_FCU &&
               version_ != FDR_INTERFACE_VERSION_FCU_VNAV && version_ != FDR_INTERFACE_VERSION_FCU_VNAV_DIAG) {
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
  bool hasFcu() const { return version_ >= FDR_INTERFACE_VERSION_FCU && version_ <= FDR_INTERFACE_VERSION_FCU_VNAV_DIAG; }
  bool hasFmsVnav() const { return version_ == FDR_INTERFACE_VERSION_FCU_VNAV || hasFmsVnavDiag(); }
  bool hasFmsVnavDiag() const { return version_ == FDR_INTERFACE_VERSION_FCU_VNAV_DIAG; }

  /** Reads the next frame; false at the end of the file (a truncated last frame is ignored). */
  bool next(FdrFrame& frame) {
    return readField(frame.base) && readField(frame.specific) && readElac(frame.elac[0]) && readElac(frame.elac[1]) &&
           readSec(frame.sec[0]) && readSec(frame.sec[1]) && readSec(frame.sec[2]) && readFac(frame.fac[0]) &&
           readFac(frame.fac[1]) && readFmgc(frame.fmgc1) && readField(frame.fadec[0].bus_outputs) &&
           readField(frame.fadec[0].outputs) && readFcu(frame) && readFmsVnav(frame);
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

  bool readFcu(FdrFrame& frame) {
    frame.hasFcu = hasFcu();
    if (!frame.hasFcu) {
      return true;
    }
    FcuRecord& r = frame.fcu;
    return readField(r.inputs) && readField(r.logic) && readField(r.discrete_outputs) && readField(r.bus_outputs);
  }

  bool readFmsVnav(FdrFrame& frame) {
    frame.hasFmsVnav = hasFmsVnav();
    frame.hasFmsVnavDiag = hasFmsVnavDiag();
    return (!frame.hasFmsVnav || readField(frame.fmsVnav)) && (!frame.hasFmsVnavDiag || readField(frame.fmsVnavDiag));
  }

  bool readFmgc(FmgcRecord& r) {
    return readField(r.logic) && readField(r.ap_fd_logic) && readField(r.ap_fd_outer_loops) && readField(r.athr) &&
           readField(r.discrete_outputs) && readField(r.bus_outputs) && readField(r.bus_inputs) && readField(r.discrete_inputs) &&
           readField(r.fms_inputs);
  }
};
