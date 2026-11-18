#pragma once

#include <memory>

#include "../Arinc429.h"
#include "../model/FacComputer.h"
#include "../utils/PulseNode.h"
#include "../utils/SRFlipFlop.h"
#include "FacIO.h"

class FacShadow;

class Fac {
 public:
  Fac(bool isUnit1);

  Fac(const Fac&);

  ~Fac();

  void update(double deltaTime, double simulationTime, bool faultActive, bool isPowered);

  base_fac_bus getBusOutputs();

  base_fac_discrete_outputs getDiscreteOutputs();

  base_fac_analog_outputs getAnalogOutputs();

  // Shadow mode (ini [FAC] SHADOW_ENABLED, see tools/fmgc-lab/docs/shadow.md): the readable FAC runs alongside the
  // generated one on the same inputs and its outputs are compared; the aircraft only ever uses the generated outputs.
  // Call before the first update(), so both computers start from power-up.
  void enableShadow();

  FacShadow* getShadow() const { return shadow.get(); }

  FacComputer::ExternalInputs_FacComputer_T modelInputs = {};

  // The model inputs of the last model step, exactly as the model saw them (recorded by the flight data recorder). A
  // power outage over 10 ms stops the model: they then stay those of the step before the outage.
  const FacComputer::ExternalInputs_FacComputer_T& getLastStepInputs() const { return lastStepInputs; }

 private:
  void initSelfTests();

  void clearMemory();

  void monitorPowerSupply(double deltaTime, bool isPowered);

  void monitorSelf(bool faultActive);

  void updateSelfTest(double deltaTime);

  // Model
  FacComputer facComputer;
  fac_outputs modelOutputs;
  FacComputer::ExternalInputs_FacComputer_T lastStepInputs = {};

  // Shadow checker, null unless enabled
  std::unique_ptr<FacShadow> shadow;

  // Computer Self-monitoring vars
  bool facHealthy;

  bool selfTestFaultLightVisible;

  SRFlipFlop facHealthyFlipFlop = SRFlipFlop(false);

  PulseNode pushbuttonPulse = PulseNode(true);

  // Power Supply monitoring
  double powerSupplyOutageTime;

  bool longPowerFailure;

  bool shortPowerFailure;

  // Selftest vars
  double selfTestTimer;

  bool selfTestComplete;

  // Constants
  const bool isUnit1;

  const double longPowerFailureTime = 0.2;
  const double shortPowerFailureTime = 0.01;
  const double selfTestDuration = 20;
};
