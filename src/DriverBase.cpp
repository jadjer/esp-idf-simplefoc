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

#include "foc/driver/DriverBase.hpp"

namespace foc {
//
//// enable motor driver
//void DriverBase::enable() {
//  // enable_pin the driver - if enable_pin pin available
//  if (_isset(enable_pin))
//    digitalWrite(enable_pin, enable_active_high);
//  // set phase state enabled
//  setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
//  // set zero to PWM
//  setPwm(0, 0, 0);
//}
//
//// disable motor driver
//void DriverBase::disable() {
//  // set phase state to disabled
//  setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
//  // set zero to PWM
//  setPwm(0, 0, 0);
//  // disable the driver - if enable_pin pin available
//  if (_isset(enable_pin))
//    digitalWrite(enable_pin, !enable_active_high);
//}
//
//// init hardware pins
//int DriverBase::init() {
//
//  // PWM pins
//  pinMode(pwmA_h, OUTPUT);
//  pinMode(pwmB_h, OUTPUT);
//  pinMode(pwmC_h, OUTPUT);
//  pinMode(pwmA_l, OUTPUT);
//  pinMode(pwmB_l, OUTPUT);
//  pinMode(pwmC_l, OUTPUT);
//  if (_isset(enable_pin))
//    pinMode(enable_pin, OUTPUT);
//
//  // sanity check for the voltage limit configuration
//  if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
//    voltage_limit = voltage_power_supply;
//
//  // set phase state to disabled
//  phase_state[0] = PhaseState::PHASE_OFF;
//  phase_state[1] = PhaseState::PHASE_OFF;
//  phase_state[2] = PhaseState::PHASE_OFF;
//
//  // set zero to PWM
//  dc_a = dc_b = dc_c = 0;
//
//  // configure 6pwm
//  // hardware specific function - depending on driver and mcu
//  params = _configure6PWM(pwm_frequency, dead_zone, pwmA_h, pwmA_l, pwmB_h, pwmB_l, pwmC_h, pwmC_l);
//  initialized = (params != SIMPLEFOC_DRIVER_INIT_FAILED);
//  return params != SIMPLEFOC_DRIVER_INIT_FAILED;
//}
//
//// Set voltage to the pwm pin
//void DriverBase::setPwm(float Ua, float Ub, float Uc) {
//  // limit the voltage in driver
//  Ua = _constrain(Ua, 0, voltage_limit);
//  Ub = _constrain(Ub, 0, voltage_limit);
//  Uc = _constrain(Uc, 0, voltage_limit);
//  // calculate duty cycle
//  // limited in [0,1]
//  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
//  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
//  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
//  // hardware specific writing
//  // hardware specific function - depending on driver and mcu
//  _writeDutyCycle6PWM(dc_a, dc_b, dc_c, phase_state, params);
//}
//
//// Set the phase state
//// actually changing the state is only done on the next call to setPwm, and
//// depends on the hardware capabilities of the driver and MCU.
//void DriverBase::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
//  phase_state[0] = sa;
//  phase_state[1] = sb;
//  phase_state[2] = sc;
//}

} // namespace foc
