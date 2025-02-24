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

#include <foc/current_sensor/BaseCurrentSensor.hpp>

#include "../common/base_classes/BLDCDriver.h"
#include "../common/base_classes/StepperDriver.h"
#include "../common/defaults.h"
#include "../common/foc_utils.h"
#include "../common/lowpass_filter.h"
#include "../common/time_utils.h"

namespace foc {

class InlineCurrentSense : public BaseCurrentSensor {
public:
  /**
      InlineCurrentSense class constructor
      @param shunt_resistor shunt resistor value
      @param gain current-sense op-amp gain
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
  InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);
  /**
      InlineCurrentSense class constructor
      @param mVpA mV per Amp ratio
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
  InlineCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET);

  // CurrentSense interface implementing functions
  int init() override;
  PhaseCurrent_s getPhaseCurrents() override;

private:
  // gain variables
  float shunt_resistor;     //!< Shunt resistor value
  float amp_gain;           //!< amp gain value
  float volts_to_amps_ratio;//!< Volts to amps ratio

  /**
     *  Function finding zero offsets of the ADC
     */
  void calibrateOffsets();
};

}// namespace foc
