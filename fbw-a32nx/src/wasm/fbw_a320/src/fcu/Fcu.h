#pragma once
#include <memory>

#include "../model/FcuComputer.h"

class FcuShadow;

class Fcu {
 public:
  Fcu();

  ~Fcu();

  void update(double deltaTime, double simulationTime, bool fcu1FaultActive, bool fcu2FaultActive, bool fcu1IsPowered, bool fcu2IsPowered);

  base_fcu_bus getBusOutputs();

  base_fcu_discrete_outputs getDiscreteOutputs();

  // The FCU model's own outputs of the last step, before the wrapper adds its monitoring (flight data recorder)
  const fcu_outputs& getModelOutputs() const { return fcuComputer.getExternalOutputs().out; }

  // Shadow mode (ini [FCU] SHADOW_ENABLED, see tools/fmgc-lab/docs/shadow.md): the readable FCU runs alongside the
  // generated one on the same inputs and its outputs are compared; the aircraft only ever uses the generated outputs.
  // Call before the first update(), so both computers start from power-up.
  void enableShadow();

  FcuShadow* getShadow() const { return shadow.get(); }

  FcuComputer::ExternalInputs_FcuComputer_T modelInputs = {};

 private:
  void initSelfTests(int index);

  void monitorPowerSupply(double deltaTime, bool isPowered, int index);

  void monitorSelf(bool faultActive, int index);

  void updateSelfTest(double deltaTime, int index);

  void maskSpdMachValueChange(double deltaTime);

  // Model
  FcuComputer fcuComputer;
  fcu_outputs modelOutputs;

  // Shadow checker, null unless enabled
  std::unique_ptr<FcuShadow> shadow;

  // Computer Self-monitoring vars
  bool fcuHealthy;

  bool monitoringHealthy[2];

  bool cpuStopped[2];

  // Power Supply monitoring
  double powerSupplyOutageTime[2];

  bool powerSupplyFault[2];

  // Selftest vars
  double selfTestTimer[2];

  bool selfTestComplete[2];

  // SPD/MACH value change masking in managed speed
  double timeSinceSpdMachPilotInput = 0;

  // Constants
  const double minimumPowerOutageTimeForFailure = 0.02;
  const double selfTestDuration = 4;
  // The model's value change pulse lasts 0.5 s; pass the bit for 1 s after a pilot input
  const double spdMachValueChangePulseDuration = 1;
};
