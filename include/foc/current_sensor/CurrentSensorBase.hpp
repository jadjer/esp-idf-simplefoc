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

#include <foc/current_sensor/interface/CurrentSensor.hpp>

namespace foc {

/**
 *  Current sensing abstract class defintion
 * Each current sensing implementation needs to extend this interface
 */
class CurrentSensorBase : public interface::CurrentSensor {
public:
  /**
   * enable the current sense. default implementation does nothing, but you can
   * override it to do something useful.
   */
  auto enable() -> void override;

  /**
   * disable the current sense. default implementation does nothing, but you can
   * override it to do something useful.
   */
  auto disable() -> void override;

public:
  /**
   * Function reading the magnitude of the current set to the motor
   *  It returns the absolute or signed magnitude if possible
   *  It can receive the motor electrical angle to help with calculation
   *  This function is used with the current control  (not foc)
   *
   * @param angle_el - electrical angle of the motor (optional)
   */
  auto getDCCurrent(ElectricalAngle electricalAngle) -> Current override;

  /**
   * Function used for FOC control, it reads the DQ currents of the motor
   *   It uses the function getPhaseCurrents internally
   *
   * @param angle_el - motor electrical angle
   */
  auto getFOCCurrents(ElectricalAngle electricalAngle) -> DQCurrent override;

  /**
   * Function used for Clarke transform in FOC control
   *   It reads the phase currents of the motor
   *   It returns the alpha and beta currents
   *
   * @param current - phase current
   */
  auto getABCurrents(PhaseCurrent current) -> ABCurrent override;

  /**
   * Function used for Park transform in FOC control
   *   It reads the Alpha Beta currents and electrical angle of the motor
   *   It returns the D and Q currents
   *
   * @param current - phase current
   */
  auto getDQCurrents(ABCurrent current, ElectricalAngle electricalAngle) -> DQCurrent override;

  /**
   * Function used to read the average current values over N samples
   */
  auto readAverageCurrents(int N = 100) -> PhaseCurrent;

private:
  // variables
  bool skip_align = false;  //!< variable signaling that the phase current
                            //!< direction should be verified during initFOC()
  bool initialized = false; // true if current sense was successfully initialized

  // ADC measurement gain for each phase
  // support for different gains for different phases of more commonly -
  // inverted phase currents this should be automated later
  float gain_a; //!< phase A gain
  float gain_b; //!< phase B gain
  float gain_c; //!< phase C gain

  float offset_ia; //!< zero current A voltage value (center of the adc reading)
  float offset_ib; //!< zero current B voltage value (center of the adc reading)
  float offset_ic; //!< zero current C voltage value (center of the adc reading)

  // hardware variables
  int pinA; //!< pin A analog pin for current measurement
  int pinB; //!< pin B analog pin for current measurement
  int pinC; //!< pin C analog pin for current measurement
};

} // namespace foc
