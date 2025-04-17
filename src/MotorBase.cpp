// Copyright 2025 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "foc/motor/MotorBase.hpp"

#include <utility>

namespace foc {

auto const DEF_POWER_SUPPLY = 12;
auto const DEF_CURRENT_LIM = 2;
auto const DEF_INDEX_SEARCH_TARGET_VELOCITY = 2;
auto const DEF_VOLTAGE_SENSOR_ALIGN = 1;

MotorBase::MotorBase(MotorBase::PolePairs pp, MotorBase::OptionalResistance r, MotorBase::OptionalKVRating kv, MotorBase::OptionalInductance l)
    : m_polePairs(pp), m_phaseResistance(r), m_KVRating(kv), m_phaseInductance(l) {
  voltage_limit = DEF_POWER_SUPPLY;
  current_limit = DEF_CURRENT_LIM;

  m_velocityIndexSearch = DEF_INDEX_SEARCH_TARGET_VELOCITY;
  m_voltageSensorAlign = DEF_VOLTAGE_SENSOR_ALIGN;

  m_focModulation = FOCModulationType::SINE_PWM;

  m_target = 0;
  m_voltage.d = 0;
  m_voltage.q = 0;

  m_currentSP = 0;
  m_current.q = 0;
  m_current.d = 0;

  m_voltageBemf = 0;

  m_uAlpha = 0;
  m_uBeta = 0;

  m_encoderOffset = 0.0f;
}

void MotorBase::linkDriver(MotorBase::Driver driver) { m_driver = std::move(driver); }

void MotorBase::linkEncoder(Encoder encoder) { m_encoder = std::move(encoder); }

void MotorBase::linkCurrentSense(CurrentSensor currentSensor) { m_currentSensor = std::move(currentSensor); }

auto MotorBase::shaftAngle() -> float {
  if (not m_encoder) {
    return m_shaftAngle;
  }

  return m_encoderDirection * m_LPFAngle(m_encoder->getAngle()) - m_encoderOffset;
  return 0;
}

auto MotorBase::shaftVelocity() -> float {
  if (not m_encoder) {
    return m_shaftVelocity;
  }

  return m_encoderDirection * m_LPFVelocity(m_encoder->getVelocity());
  return 0;
}

auto MotorBase::electricalAngle() -> float {
  if (not m_encoder) {
    return m_electricalAngle;
  }

//  return _normalizeAngle((float)(m_encoderDirection * m_polePairs) * m_encoder->getMechanicalAngle() - m_zeroElectricAngle);
//  return _normalizeAngle((float)(m_encoderDirection * m_polePairs) * m_encoder->getAngle() - m_zeroElectricAngle);
  return 0;
}

} // namespace foc
