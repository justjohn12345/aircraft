#include "Fcu.h"
#include "../Arinc429Utils.h"
#include "../shadow/FcuShadow.h"

Fcu::Fcu() {
  fcuComputer.initialize();
}

Fcu::~Fcu() = default;

void Fcu::enableShadow() {
  if (!shadow) {
    shadow = std::make_unique<FcuShadow>();
  }
}

void Fcu::initSelfTests(int index) {
  if (powerSupplyFault[index])
    return;

  selfTestTimer[index] = selfTestDuration;
}

void Fcu::update(double deltaTime,
                 double simulationTime,
                 bool fcu1FaultActive,
                 bool fcu2FaultActive,
                 bool fcu1IsPowered,
                 bool fcu2IsPowered) {
  for (int i = 0; i < 2; i++) {
    monitorPowerSupply(deltaTime, i == 0 ? fcu1IsPowered : fcu2IsPowered, i);

    updateSelfTest(deltaTime, i);
    monitorSelf(i == 0 ? fcu1FaultActive : fcu2FaultActive, i);
  }

  fcuHealthy = monitoringHealthy[0] || monitoringHealthy[1];
  modelInputs.in.sim_data.computer_running = fcuHealthy;

  // In managed speed the FMGC commands the SPD/MACH switchover (A320 FCOM DSC-22_30-40 SPEED/MACH SWITCHING), so the
  // SPD/MACH pb is ignored: it only switches between speed and Mach in selected speed. Uses the managed speed indication
  // of the previous step.
  if (modelOutputs.discrete_outputs.afs_outputs.spd_mach_managed) {
    modelInputs.in.discrete_inputs.afs_inputs.spd_mach_button_pressed = false;
  }

  fcuComputer.setExternalInputs(&modelInputs);
  fcuComputer.step();
  modelOutputs = fcuComputer.getExternalOutputs().out;

  if (shadow) {
    // Same inputs as the generated step (computer_running included); compare only, the outputs above are unchanged.
    shadow->step(modelInputs.in, fcuComputer.getExternalOutputs().out);
  }

  maskSpdMachValueChange(deltaTime);
}

void Fcu::maskSpdMachValueChange(double deltaTime) {
  // While the SPD/MACH window is dashed, the model's SPD/MACH memory follows the actual CAS/Mach from the FMGC bus (rounded
  // to 1 kt / 0.01), and the "SPD/MACH value changed" bit (word 1 bit 15) pulses whenever that rounded value moves, e.g.
  // when the cruise Mach drifts across 0.xx5. The FMGC takes the bit as a pilot speed change and drops the ALT soft mode (FCOM
  // DSC-22_30-70-70), so the A/THR goes back to full gain. Only pass the bit in managed speed when the pilot used the
  // SPD/MACH controls (or set a value from the sim) during the pulse.
  const auto& afs = modelInputs.in.discrete_inputs.afs_inputs;
  const bool pilotInput = afs.spd_knob.turns != 0 || afs.spd_knob.pushed || afs.spd_knob.pulled || afs.spd_mach_button_pressed ||
                          modelInputs.in.sim_input.spd_mach != -1.0F;
  timeSinceSpdMachPilotInput = pilotInput ? 0 : timeSinceSpdMachPilotInput + deltaTime;

  if (modelOutputs.discrete_outputs.afs_outputs.spd_mach_managed && timeSinceSpdMachPilotInput > spdMachValueChangePulseDuration) {
    Arinc429Utils::setBit(modelOutputs.bus_outputs.fcu_discrete_word_1, 15, false);
  }
}

void Fcu::monitorSelf(bool faultActive, int index) {
  if (faultActive || powerSupplyFault[index] || !selfTestComplete[index]) {
    monitoringHealthy[index] = false;
  } else {
    monitoringHealthy[index] = true;
  }
}

void Fcu::monitorPowerSupply(double deltaTime, bool isPowered, int index) {
  if (!isPowered) {
    powerSupplyOutageTime[index] += deltaTime;
  }
  if (powerSupplyOutageTime[index] > minimumPowerOutageTimeForFailure) {
    powerSupplyFault[index] = true;
  }
  if (isPowered && powerSupplyFault[index]) {
    powerSupplyFault[index] = false;
    initSelfTests(index);
    powerSupplyOutageTime[index] = 0;
  }
}

void Fcu::updateSelfTest(double deltaTime, int index) {
  if (selfTestTimer[index] > 0) {
    selfTestTimer[index] -= deltaTime;
  }
  if (selfTestTimer[index] <= 0) {
    selfTestComplete[index] = true;
  } else {
    selfTestComplete[index] = false;
  }
}

base_fcu_bus Fcu::getBusOutputs() {
  if (!fcuHealthy) {
    return {};
  }

  Arinc429Utils::setBit(modelOutputs.bus_outputs.fcu_discrete_word_2, 24, monitoringHealthy[0]);
  Arinc429Utils::setBit(modelOutputs.bus_outputs.fcu_discrete_word_2, 25, monitoringHealthy[1]);

  return modelOutputs.bus_outputs;
}

base_fcu_discrete_outputs Fcu::getDiscreteOutputs() {
  base_fcu_discrete_outputs output = {};

  output.fcu_healthy = fcuHealthy;
  if (!fcuHealthy) {
    return output;
  } else {
    return modelOutputs.discrete_outputs;
  }
}
