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

#include <foc/motor/BaseMotor.hpp>
#include <foc/driver/interface/StepperDriver.hpp>
#include <foc/encoder/BaseEncoder.hpp>

#include "common/defaults.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

namespace foc {

/**
 Stepper Motor class
*/
class StepperMotor : public BaseMotor {
public:
  /**
      StepperMotor class constructor
      @param pp  pole pair number 
     @param R  motor phase resistance - [Ohm]
     @param KV  motor KV rating (1/K_bemf) - rpm/V
     @param L  motor phase inductance - [H]
    */
  StepperMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);
  ~StepperMotor() override = default;

public:
  /**
     * Function linking a motor and a foc driver 
     * 
     * @param driver StepperDriver class implementing all the hardware specific functions necessary PWM setting
     */
  void linkDriver(StepperDriver *driver);

  /**
      * StepperDriver link:
      * - 4PWM  - L298N for example
    */
  StepperDriver *driver;

  /**  Motor hardware init function */
  int init() override;
  /** Motor disable function */
  void disable() override;
  /** Motor enable function */
  void enable() override;

  /**
     * Function initializing FOC algorithm
     * and aligning sensor's and motors' zero position 
     * 
     * - If zero_electric_offset parameter is set the alignment procedure is skipped
     */
  int initFOC() override;
  /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */
  void loopFOC() override;
  /**
     * Function executing the control loops set by the controller parameter of the StepperMotor.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
  void move(float target = NOT_SET) override;

  /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param Ud Current voltage in d axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
  void setPhaseVoltage(float Uq, float Ud, float angle_el) override;

private:
  /** Sensor alignment to electrical 0 angle of the motor */
  int alignSensor();
  /** Motor and sensor alignment to the sensors absolute 0 angle  */
  int absoluteZeroSearch();
  /** Current sense and motor phase alignment */
  int alignCurrentSense();

  // Open loop motion control
  /**
     * Function (iterative) generating open loop movement for target velocity
     * it uses voltage_limit variable
     * 
     * @param target_velocity - rad/s
     */
  float velocityOpenloop(float target_velocity);
  /**
     * Function (iterative) generating open loop movement towards the target angle
     * it uses voltage_limit and velocity_limit variables
     * 
     * @param target_angle - rad
     */
  float angleOpenloop(float target_angle);
  // open loop variables
  long open_loop_timestamp;
};

}// namespace foc
