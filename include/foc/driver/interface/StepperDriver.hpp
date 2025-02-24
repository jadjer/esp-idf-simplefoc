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

#pragma once

#include <foc/driver/interface/Driver.hpp>

namespace foc::interface {

class StepperDriver : public Driver {
public:
  ~StepperDriver() override = default;

public:
  /**
         * Set phase voltages to the hardware 
         * 
         * @param Ua phase A voltage
         * @param Ub phase B voltage
        */
  virtual void setPwm(float Ua, float Ub) = 0;

  /**
         * Set phase state, enable/disable
         *
         * @param sc - phase A state : active / disabled ( high impedance )
         * @param sb - phase B state : active / disabled ( high impedance )
        */
  virtual void setPhaseState(PhaseState sa, PhaseState sb) = 0;

  /** driver type getter function */
  DriverType type() override { return DriverType::Stepper; };
};

}// namespace foc::driver::interface
