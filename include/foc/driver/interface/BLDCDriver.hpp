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

class BLDCDriver : public Driver {
public:
  ~BLDCDriver() override = default;

public:
  float dc_a;//!< currently set duty cycle on phaseA
  float dc_b;//!< currently set duty cycle on phaseB
  float dc_c;//!< currently set duty cycle on phaseC

  /**
         * Set phase voltages to the hardware
         *
         * @param Ua - phase A voltage
         * @param Ub - phase B voltage
         * @param Uc - phase C voltage
        */
  virtual void setPwm(float Ua, float Ub, float Uc) = 0;

  /**
         * Set phase state, enable/disable
         *
         * @param sc - phase A state : active / disabled ( high impedance )
         * @param sb - phase B state : active / disabled ( high impedance )
         * @param sa - phase C state : active / disabled ( high impedance )
        */
  virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;

  /** driver type getter function */
  DriverType type() override { return DriverType::BLDC; };
};

}
