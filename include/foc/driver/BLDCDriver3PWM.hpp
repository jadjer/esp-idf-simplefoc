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

#include <foc/driver/interface/BLDCDriver.hpp>

#include "../common/defaults.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

namespace foc {

/**
 3 pwm bldc driver class
*/
class BLDCDriver3PWM : public interface::BLDCDriver {
public:
  /**
      BLDCDriver class constructor
      @param phA A phase pwm pin
      @param phB B phase pwm pin
      @param phC C phase pwm pin
      @param en1 enable pin (optional input)
      @param en2 enable pin (optional input)
      @param en3 enable pin (optional input)
    */
  BLDCDriver3PWM(int phA, int phB, int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET);

  /**  Motor hardware init function */
  int init() override;
  /** Motor disable function */
  void disable() override;
  /** Motor enable function */
  void enable() override;

  // hardware variables
  int pwmA;       //!< phase A pwm pin number
  int pwmB;       //!< phase B pwm pin number
  int pwmC;       //!< phase C pwm pin number
  int enableA_pin;//!< enable pin number
  int enableB_pin;//!< enable pin number
  int enableC_pin;//!< enable pin number

  /**
     * Set phase voltages to the hardware 
     * 
     * @param Ua - phase A voltage
     * @param Ub - phase B voltage
     * @param Uc - phase C voltage
    */
  void setPwm(float Ua, float Ub, float Uc) override;

  /**
     * Set phase voltages to the hardware
     * > Only possible is the driver has separate enable pins for all phases!  
     * 
     * @param sc - phase A state : active / disabled ( high impedance )
     * @param sb - phase B state : active / disabled ( high impedance )
     * @param sa - phase C state : active / disabled ( high impedance )
    */
  virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;

private:
};

}// namespace foc::driver
