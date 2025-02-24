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

#include <cstdint>
#include <memory>

#include <esp_err.h>

#include <foc/current_sensor/BaseCurrentSensor.hpp>
#include <foc/driver/interface/Driver.hpp>
#include <foc/encoder/BaseEncoder.hpp>

#include "../defaults.h"
#include "../foc_utils.h"
#include "../lowpass_filter.h"
#include "../pid.h"
#include "../time_utils.h"
#include "common/foc_utils.h"

/**
 * @namespace foc
 */
namespace foc {

/**
 *  Motiron control type
 */
enum MotionControlType : std::uint8_t {
  torque = 0x00,  //!< Torque control
  velocity = 0x01,//!< Velocity motion control
  angle = 0x02,   //!< Position/angle motion control
  velocity_openloop = 0x03,
  angle_openloop = 0x04
};

/**
 *  Motiron control type
 */
enum TorqueControlType : std::uint8_t {
  voltage = 0x00,    //!< Torque control using voltage
  dc_current = 0x01, //!< Torque control using DC current (one current magnitude)
  foc_current = 0x02,//!< torque control using dq currents
};

/**
 *  FOC modulation type
 */
enum FOCModulationType : std::uint8_t {
  SinePWM = 0x00,       //!< Sinusoidal PWM modulation
  SpaceVectorPWM = 0x01,//!< Space vector modulation method
  Trapezoid_120 = 0x02,
  Trapezoid_150 = 0x03,
};

enum FOCMotorStatus : std::uint8_t {
  motor_uninitialized = 0x00,//!< Motor is not yet initialized
  motor_initializing = 0x01, //!< Motor intiialization is in progress
  motor_uncalibrated = 0x02, //!< Motor is initialized, but not calibrated (open loop possible)
  motor_calibrating = 0x03,  //!< Motor calibration in progress
  motor_ready = 0x04,        //!< Motor is initialized and calibrated (closed loop possible)
  motor_error = 0x08,        //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
  motor_calib_failed = 0x0E, //!< Motor calibration failed (possibly recoverable)
  motor_init_failed = 0x0F,  //!< Motor initialization failed (not recoverable)
};

/**
 * @class BaseMotor
 * @brief Generic motor class
 */
class BaseMotor {
public:
  using Driver = std::unique_ptr<interface::Driver>;
  using Encoder = std::unique_ptr<BaseEncoder>;
  using CurrentSensor = std::unique_ptr<BaseCurrentSensor>;

public:
  explicit BaseMotor(std::uint8_t pp, float R = -1, float KV = -1, float L = -1);
  virtual ~BaseMotor() = default;

public:
  /**
   * Function linking a motor and a sensor
   * @param sensor Sensor class  wrapper for the FOC algorihtm to read the motor angle and velocity
   */
  void linkEncoder(Encoder sensor);
  /**
   * Function linking a motor and current sensing
   * @param currentSensor CurrentSense class wrapper for the FOC algorihtm to read the motor current measurements
   */
  void linkCurrentSense(CurrentSensor currentSensor);

public:
  /**
   * Motor hardware init function
   * @return
   */
  virtual esp_err_t init() = 0;
  /**
   * Motor disable function
   */
  virtual void disable() = 0;
  /**
   * Motor enable function
   */
  virtual void enable() = 0;

  /**
     * Function initializing FOC algorithm
     * and aligning sensor's and motors' zero position 
     * 
     * - If zero_electric_offset parameter is set the alignment procedure is skipped
     */
  virtual int initFOC() = 0;
  /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */
  virtual void loopFOC() = 0;
  /**
     * Function executing the control loops set by the controller parameter of the BLDCMotor.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
  virtual void move(float target) = 0;

  /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param Ud Current voltage in d axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
  virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) = 0;

public:
  /** Shaft angle calculation in radians [rad] */
  float shaftAngle();
  /**
     * Shaft angle calculation function in radian per second [rad/s]
     * It implements low pass filtering
     */
  float shaftVelocity();

  /**
     * Electrical angle calculation  
     */
  float electricalAngle();

public:
  std::uint8_t const m_polePairs;    //!< motor pole pairs number
  float const m_phaseResistance = -1;//!< motor phase resistance
  float const m_KVRating = -1;       //!< motor KV rating
  float const m_phaseInductance = -1;//!< motor phase inductance

protected:
  Encoder m_encoder = nullptr;
  CurrentSensor m_currentSensor = nullptr;

protected:
  // state variables
  float m_target;                    //!< current target value - depends of the controller
  float m_feedForwardVelocity = 0.0f;//!< current feed forward velocity
  float m_shaftAngle;                //!< current motor angle
  float m_electricalAngle;           //!< current electrical angle
  float m_shaftVelocity;             //!< current motor velocity
  float m_currentSP;                 //!< target current ( q current )
  float m_shaftVelocitySP;           //!< current target velocity
  float m_shaftAngleSP;              //!< current target angle
  DQVoltage_s m_voltage;             //!< current d and q voltage set to the motor
  DQCurrent_s m_current;             //!< current d and q current measured
  float m_voltageBemf;               //!< estimated backemf voltage (if provided KV constant)
  float m_uAlpha, m_uBeta;           //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

  // motor configuration parameters
  float m_voltageSensorAlign; //!< sensor and motor align voltage parameter
  float m_velocityIndexSearch;//!< target velocity for index search

  // limiting variables
  float voltage_limit; //!< Voltage limiting variable - global limit
  float current_limit; //!< Current limiting variable - global limit
  float velocity_limit;//!< Velocity limiting variable - global limit

  // motor status vairables
  bool m_enabled = false;                                            //!< enabled or disabled motor flag
  FOCMotorStatus m_motorStatus = FOCMotorStatus::motor_uninitialized;//!< motor status

  // pwm modulation related variables
  FOCModulationType m_focModulation;   //!<  parameter determining modulation algorithm
  std::int8_t m_modulationCentered = 1;//!< flag (1) centered modulation around driver limit /2  or  (0) pulled to 0

  // configuration structures
  TorqueControlType m_torqueController;//!< parameter determining the torque control type
  MotionControlType m_controller;      //!< parameter determining the control loop to be used

  // controllers and low pass filters
  PIDController m_PIDCurrentQ{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};//!< parameter determining the q current PID config
  PIDController m_PIDCurrentD{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};//!< parameter determining the d current PID config
  LowPassFilter m_LPFCurrentQ{DEF_CURR_FILTER_Tf};                                                                 //!<  parameter determining the current Low pass filter configuration
  LowPassFilter m_LPFCurrentD{DEF_CURR_FILTER_Tf};                                                                 //!<  parameter determining the current Low pass filter configuration
  PIDController m_PIDVelocity{DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT};   //!< parameter determining the velocity PID configuration
  PIDController m_PIDAngle{DEF_P_ANGLE_P, 0, 0, 0, DEF_VEL_LIM};                                                   //!< parameter determining the position PID configuration
  LowPassFilter m_LPFVelocity{DEF_VEL_FILTER_Tf};                                                                  //!<  parameter determining the velocity Low pass filter configuration
  LowPassFilter m_LPFAngle{0.0};                                                                                   //!<  parameter determining the angle low pass filter configuration
  unsigned int m_motionDownsample = DEF_MOTION_DOWNSMAPLE;                                                         //!< parameter defining the ratio of downsampling for move commad
  unsigned int m_motionCNT = 0;                                                                                    //!< counting variable for downsampling for move commad

  // sensor related variabels
  float m_encoderOffset;                            //!< user defined sensor zero offset
  float m_zeroElectricAngle = NOT_SET;              //!< absolute zero electric angle - if available
  Direction m_encoderDirection = Direction::UNKNOWN;//!< default is CW. if sensor_direction == Direction::CCW then direction will be flipped compared to CW. Set to UNKNOWN to set by calibration
  bool m_ppCheckResult = false;                     //!< the result of the PP check, if run during loopFOC
};

}// namespace foc
