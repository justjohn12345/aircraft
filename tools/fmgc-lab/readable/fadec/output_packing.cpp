// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The ECU bus (generated FadecComputer.cpp 325-414). See output_packing.h.

#include "output_packing.h"

#include <cstdint>

#include "fadec_blocks.h"

namespace fadec::readable {

namespace {

constexpr uint32_t kNormal = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
constexpr uint32_t kNoComputedData = static_cast<uint32_t>(SignStatusMatrix::NoComputedData);

/** Discrete word flags, ARINC bits 11-29 (generated MATLABFunction_l on a 19-element vector). */
class DiscreteBits {
 public:
  DiscreteBits& bit(int arincBit, bool value) {
    bits_[arincBit - 11] = value;
    return *this;
  }
  base_arinc_429 word() const { return {kNormal, blocks::packBits(bits_)}; }

 private:
  bool bits_[19]{};
};

base_arinc_429 normal(double value) {
  return {kNormal, static_cast<real32_T>(value)};
}

}  // namespace

base_ecu_bus packEcuBus(const athr_in& in, const EcuBusInputs& v) {
  using T = athr_thrust_limit_type;
  base_ecu_bus b{};
  b.n1_ref_percent = normal(v.n1ForLever);
  b.selected_tla_deg = normal(in.input.TLA_deg);
  b.selected_flex_temp_deg.SSM = kNoComputedData;
  if (v.flexActive) {
    b.selected_flex_temp_deg.SSM = kNormal;
  }
  b.selected_flex_temp_deg.Data = blocks::a429::valueOr(in.fcu_input.fcu_flex_to_temp_deg_c, 0.0F);
  b.ecu_status_word_1 = {kNormal, 0.0F};
  b.ecu_status_word_2 = {kNormal, 0.0F};
  b.ecu_status_word_3 = DiscreteBits()
                            .bit(15, v.limit.type == T::REVERSE)
                            .bit(19, v.controlActive)
                            .bit(21, v.alphaFloor)
                            .bit(22, v.limit.type == T::TOGA)
                            .bit(23, v.limit.type == T::FLEX)
                            .bit(24, v.limit.type == T::MCT)
                            .bit(25, v.limit.type == T::CLB)
                            .word();
  b.n1_limit_percent = normal(v.limit.percent);
  b.n1_maximum_percent = normal(in.input.thrust_limit_TOGA_percent);
  b.n1_command_percent = normal(v.n1Command);
  b.selected_n2_actual_percent = normal(in.data.engine_N2_percent);
  b.selected_n1_actual_percent = normal(in.data.engine_N1_percent);
  // 389-414: the generated code first writes N1 actual into the maintenance word's data, then the packed bits
  b.ecu_maintenance_word_6 = DiscreteBits().bit(12, v.thrustLock).word();
  return b;
}

}  // namespace fadec::readable
