#pragma once
#include <memory>

#include "../model/FmgcComputer.h"

class FmgcShadow;

class Fmgc {
 public:
  Fmgc(bool isUnit1);

  Fmgc(const Fmgc&);

  ~Fmgc();

  void update(double deltaTime, double simulationTime, bool faultActive, bool isPowered);

  base_fmgc_bus_outputs getBusOutputs();

  base_fmgc_discrete_outputs getDiscreteOutputs();

  const fmgc_outputs& getDebugOutputs() const;

  // Shadow mode (ini [FMGC] SHADOW_ENABLED, see tools/fmgc-lab/docs/shadow.md): the readable FMGC runs alongside the
  // generated one on the same inputs and its outputs are compared; the aircraft only ever uses the generated outputs.
  // Call before the first update(), so both computers start from power-up.
  void enableShadow();

  FmgcShadow* getShadow() const { return shadow.get(); }

  FmgcComputer::ExternalInputs_FmgcComputer_T modelInputs = {};

 private:
  void initSelfTests();

  void clearMemory();

  void monitorPowerSupply(double deltaTime, bool isPowered);

  void monitorSelf(bool faultActive);

  void updateSelfTest(double deltaTime);

  // Model
  FmgcComputer fmgcComputer;
  fmgc_outputs modelOutputs;

  // Shadow checker, null unless enabled
  std::unique_ptr<FmgcShadow> shadow;

  // Computer Self-monitoring vars
  bool monitoringHealthy;

  bool selfTestApEngagedDiscreteOn;

  bool selfTestAthrEngagedDiscreteOn;

  bool selfTestDigitalOutValid;

  bool cpuStopped;

  // Power Supply monitoring
  double powerSupplyOutageTime;

  bool powerSupplyFault;

  // Selftest vars
  double selfTestTimer;

  bool selfTestComplete;

  // Constants
  const bool isUnit1;

  const double minimumPowerOutageTimeForFailure = 0.02;
  const double selfTestDuration = 30;
};
