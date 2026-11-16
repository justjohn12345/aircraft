#pragma once

#include <fstream>

#include "../elac/Elac.h"
#include "../fac/Fac.h"
#include "../fcu/Fcu.h"
#include "../model/FadecComputer.h"
#include "../model/FmgcComputer_types.h"
#include "../sec/Sec.h"
#include "LocalVariable.h"
#include "RecordingDataTypes.h"
#include "zfstream.h"

class FlightDataRecorder {
 public:
  // IMPORTANT: this constant needs to increased with every interface change
  // 3209001: personal build, 3200006 plus the FCU model inputs and outputs at the end of each record
  // 3209002: personal build, 3209001 plus the FMS VNAV state (FmsVnavData) at the end of each record
  const uint64_t INTERFACE_VERSION = 3209002;

  const uint32_t NUMBER_OF_ELAC_TO_WRITE = 2;
  const uint32_t NUMBER_OF_SEC_TO_WRITE = 3;
  const uint32_t NUMBER_OF_FAC_TO_WRITE = 2;
  const uint32_t NUMBER_OF_FADEC_TO_WRITE = 1;

  void initialize();

  void update(const BaseData& baseData,
              const AircraftSpecificData& aircraftSpecificData,
              Elac (&elacs)[2],
              Sec (&secs)[3],
              Fac (&facs)[2],
              const fmgc_outputs& fmgc1,
              const fmgc_outputs& fmgc2,
              FadecComputer (&fadecs)[2],
              const Fcu& fcu);

  void terminate();

  // The file being written ("" before the first record) and the number of records in it; the total number of records
  // written this session (the FMGC shadow log uses them to point at the frame of a mismatch).
  const std::string& getCurrentFilename() const { return currentFilename; }
  long getRecordsInCurrentFile() const { return recordsInCurrentFile; }
  unsigned long long getTotalRecords() const { return totalRecords; }

 private:
  const std::string CONFIGURATION_FILEPATH = "\\work\\FlightDataRecorder.ini";

  std::unique_ptr<LocalVariable> idIsEnabled;
  int sampleCounter = 0;
  int maximumSampleCounter = 864000;
  int maximumFileCount = 15;
  std::shared_ptr<gzofstream> fileStream;
  std::string currentFilename;
  long recordsInCurrentFile = 0;
  unsigned long long totalRecords = 0;

  void manageFlightDataRecorderFiles();

  std::string getFlightDataRecorderFilename();

  void cleanUpFlightDataRecorderFiles();

  void loadConfiguration();

  void writeElac(Elac& elac);

  void writeSec(Sec& sec);

  void writeFac(Fac& fac);

  void writeFmgc(const fmgc_outputs& fmgc);

  void writeFadec(FadecComputer& fadec);

  void writeFcu(const Fcu& fcu);

  void writeFmsVnav();

  // The L:vars of FmsVnavData, in the order of its fields (published by the FMS VnavDriver and FMCMainDisplay)
  static constexpr size_t FMS_VNAV_VARIABLE_COUNT = sizeof(FmsVnavData) / sizeof(double);
  static constexpr const char* FMS_VNAV_VARIABLES[] = {
      "A32NX_FDR_VNAV_DISTANCE_TO_END",
      "A32NX_FDR_VNAV_TOTAL_DISTANCE",
      "A32NX_FDR_VNAV_ALONG_PATH",
      "A32NX_FDR_VNAV_ACTIVE_LEG_INDEX",
      "A32NX_FDR_VNAV_PLAN_VERSION",
      "A32NX_FDR_VNAV_DESCENT_CONSTRAINT_COUNT",
      "A32NX_FDR_VNAV_NEXT_DESCENT_CONSTRAINT_DISTANCE",
      "A32NX_FDR_VNAV_NEXT_DESCENT_CONSTRAINT_ALT",
      "A32NX_FDR_VNAV_PROFILE_VALID",
      "A32NX_FDR_VNAV_PROFILE_TOTAL",
      "A32NX_FDR_VNAV_PROFILE_POSITION",
      "A32NX_FDR_VNAV_PROFILE_TOD",
      "A32NX_FDR_VNAV_PROFILE_END",
      "A32NX_FDR_VNAV_PROFILE_TARGET_ALT",
      "A32NX_FDR_VNAV_LINEAR_DEVIATION",
      "A32NX_FDR_VNAV_DESCENT_PROFILE_COMPUTE_COUNT",
      "A32NX_FDR_VNAV_COST_INDEX",
      "A32NX_FDR_VNAV_ZERO_FUEL_WEIGHT",
      "A32NX_FDR_VNAV_CRUISE_LEVEL",
      "A32NX_FDR_VNAV_MANAGED_DESCENT_SPEED",
      "A32NX_FDR_VNAV_APPROACH_SELECTED",
      "A32NX_FDR_VNAV_STOCK_CONSTRAINT_ALT",
  };
  static_assert(sizeof(FMS_VNAV_VARIABLES) / sizeof(FMS_VNAV_VARIABLES[0]) == FMS_VNAV_VARIABLE_COUNT);
  std::unique_ptr<LocalVariable> fmsVnavVariables[FMS_VNAV_VARIABLE_COUNT];
};
