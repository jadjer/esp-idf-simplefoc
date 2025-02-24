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
#include "../common/base_classes/FOCMotor.h"
#include "../common/base_classes/StepperDriver.h"
#include "../common/defaults.h"
#include "../common/foc_utils.h"
#include "../common/lowpass_filter.h"

namespace foc {

class LowSideCurrentSensor : public BaseCurrentSensor {
public:
  /**
      LowSideCurrentSense class constructor
      @param shunt_resistor shunt resistor value
      @param gain current-sense op-amp gain
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
  LowSideCurrentSensor(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
  /**
      LowSideCurrentSense class constructor
      @param mVpA mV per Amp ratio
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
  LowSideCurrentSensor(float mVpA, int pinA, int pinB, int pinC = _NC);

  // CurrentSense interface implementing functions
  int init() override;
  PhaseCurrent_s getPhaseCurrents();

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
