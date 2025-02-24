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

#include "foc/motor/BLDCMotor.hpp"

#include <esp_log.h>

namespace foc {

auto const TAG = "BLDCMotor";

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_120_map[6][3] = {
    {0, 1, -1},
    {-1, 1, 0},
    {-1, 0, 1},
    {0, -1, 1},
    {1, -1, 0},
    {1, 0, -1},
};

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_150_map[12][3] = {
    {0, 1, -1},
    {-1, 1, -1},
    {-1, 1, 0},
    {-1, 1, 1},
    {-1, 0, 1},
    {-1, -1, 1},
    {0, -1, 1},
    {1, -1, 1},
    {1, -1, 0},
    {1, -1, -1},
    {1, 0, -1},
    {1, 1, -1},
};

BLDCMotor::BLDCMotor(std::uint8_t pp, float R, float KV, float L) : BaseMotor(pp, R, KV, L) {
  m_torqueController = TorqueControlType::voltage;
}

/**
	Link the driver which controls the motor
*/
void BLDCMotor::linkDriver(Driver driver) {
  m_driver = std::move(driver);
}

// init hardware pins
esp_err_t BLDCMotor::init() {
  if (not m_driver) {
    m_motorStatus = FOCMotorStatus::motor_init_failed;
    ESP_LOGE(TAG, "Init not possible, driver is not set");
    return ESP_FAIL;
  }

  if (not m_driver->initialized) {
    m_motorStatus = FOCMotorStatus::motor_init_failed;
    ESP_LOGE(TAG, "Init not possible, driver not initialized");
    return ESP_FAIL;
  }

  m_motorStatus = FOCMotorStatus::motor_initializing;
  ESP_LOGI(TAG, "Init");

  if (voltage_limit > m_driver->voltage_limit) {
    voltage_limit = m_driver->voltage_limit;
  }

  if (m_voltageSensorAlign > voltage_limit) {
    m_voltageSensorAlign = voltage_limit;
  }

  if (m_currentSensor) {
    m_PIDCurrentQ.limit = voltage_limit;
    m_PIDCurrentD.limit = voltage_limit;
  }

  if ((m_phaseResistance > 0) or (m_torqueController != TorqueControlType::voltage)) {
    // velocity control loop controls current
    m_PIDVelocity.limit = current_limit;
  } else {
    // velocity control loop controls the voltage
    m_PIDVelocity.limit = voltage_limit;
  }

  m_PIDAngle.limit = velocity_limit;

  // if using open loop control, set a CW as the default direction if not already set
  if ((m_controller == MotionControlType::angle_openloop or m_controller == MotionControlType::velocity_openloop) and (m_encoderDirection == Direction::UNKNOWN)) {
    m_encoderDirection = Direction::CW;
  }

  _delay(500);

  // enable motor
  SIMPLEFOC_DEBUG("MOT: Enable driver.");

  enable();

  _delay(500);

  m_motorStatus = FOCMotorStatus::motor_uncalibrated;

  return 1;
}

void BLDCMotor::disable() {
  if (m_currentSensor) {
    m_currentSensor->disable();
  }

  m_driver->setPwm(0, 0, 0);
  m_driver->disable();
  m_enabled = false;
}

void BLDCMotor::enable() {
  m_driver->enable();
  m_driver->setPwm(0, 0, 0);

  if (m_currentSensor) {
    m_currentSensor->enable();
  }

  m_PIDVelocity.reset();
  m_PIDAngle.reset();
  m_PIDCurrentQ.reset();
  m_PIDCurrentD.reset();

  m_enabled = true;
}

int BLDCMotor::initFOC() {
  int exit_flag = 1;

  m_motorStatus = FOCMotorStatus::motor_calibrating;

  // align motor if necessary
  // alignment necessary for encoders!
  // sensor and motor alignment - can be skipped
  // by setting motor.sensor_direction and motor.zero_electric_angle
  if (m_encoder) {
    exit_flag *= alignSensor();
    // added the shaft_angle update
    m_encoder->update();
    m_shaftAngle = shaftAngle();

    // aligning the current sensor - can be skipped
    // checks if driver phases are the same as current sense phases
    // and checks the direction of measuremnt.
    if (exit_flag) {
      if (m_currentSensor) {
        if (!m_currentSensor->initialized) {
          m_motorStatus = FOCMotorStatus::motor_calib_failed;
          SIMPLEFOC_DEBUG("MOT: Init FOC error, current sense not initialized");
          exit_flag = 0;
        } else {
          exit_flag *= alignCurrentSense();
        }
      } else {
        SIMPLEFOC_DEBUG("MOT: No current sense.");
      }
    }

  } else {
    SIMPLEFOC_DEBUG("MOT: No sensor.");
    if ((m_controller == MotionControlType::angle_openloop or m_controller == MotionControlType::velocity_openloop)) {
      exit_flag = 1;
      SIMPLEFOC_DEBUG("MOT: Openloop only!");
    } else {
      exit_flag = 0;// no FOC without sensor
    }
  }

  if (exit_flag) {
    SIMPLEFOC_DEBUG("MOT: Ready.");
    m_motorStatus = FOCMotorStatus::motor_ready;

  } else {
    SIMPLEFOC_DEBUG("MOT: Init FOC failed.");
    m_motorStatus = FOCMotorStatus::motor_calib_failed;

    disable();
  }

  return exit_flag;
}

// Calibarthe the motor and current sense phases
int BLDCMotor::alignCurrentSense() {
  int exit_flag = 1;// success

  SIMPLEFOC_DEBUG("MOT: Align current sense.");

  // align current sense and the driver
  exit_flag = m_currentSensor->driverAlign(m_voltageSensorAlign, m_modulationCentered);
  if (!exit_flag) {
    // error in current sense - phase either not measured or bad connection
    SIMPLEFOC_DEBUG("MOT: Align error!");
    exit_flag = 0;
  } else {
    // output the alignment status flag
    SIMPLEFOC_DEBUG("MOT: Success: ", exit_flag);
  }

  return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int BLDCMotor::alignSensor() {
  int exit_flag = 1;//success
  SIMPLEFOC_DEBUG("MOT: Align sensor.");

  // check if sensor needs zero search
  if (m_encoder->needsSearch())
    exit_flag = absoluteZeroSearch();
  // stop init if not found index
  if (!exit_flag)
    return exit_flag;

  // v2.3.3 fix for R_AVR_7_PCREL against symbol" bug for AVR boards
  // TODO figure out why this works
  float voltage_align = m_voltageSensorAlign;

  // if unknown natural direction
  if (m_encoderDirection == Direction::UNKNOWN) {

    // find natural direction
    // move one electrical revolution forward
    for (int i = 0; i <= 500; i++) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0, angle);
      m_encoder->update();
      _delay(2);
    }
    // take and angle in the middle
    m_encoder->update();
    float mid_angle = m_encoder->getAngle();
    // move one electrical revolution backwards
    for (int i = 500; i >= 0; i--) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0, angle);
      m_encoder->update();
      _delay(2);
    }
    m_encoder->update();
    float end_angle = m_encoder->getAngle();
    // setPhaseVoltage(0, 0, 0);
    _delay(200);
    // determine the direction the sensor moved
    float moved = fabs(mid_angle - end_angle);
    if (moved < MIN_ANGLE_DETECT_MOVEMENT) {// minimum angle to detect movement
      SIMPLEFOC_DEBUG("MOT: Failed to notice movement");
      return 0;// failed calibration
    } else if (mid_angle < end_angle) {
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW");
      m_encoderDirection = Direction::CCW;
    } else {
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CW");
      m_encoderDirection = Direction::CW;
    }
    // check pole pair number
    m_ppCheckResult = !(fabs(moved * m_polePairs - _2PI) > 0.5f);// 0.5f is arbitrary number it can be lower or higher!
    if (m_ppCheckResult == false) {
      SIMPLEFOC_DEBUG("MOT: PP check: fail - estimated pp: ", _2PI / moved);
    } else {
      SIMPLEFOC_DEBUG("MOT: PP check: OK!");
    }

  } else {
    SIMPLEFOC_DEBUG("MOT: Skip dir calib.");
  }

  // zero electric angle not known
  if (!_isset(m_zeroElectricAngle)) {
    // align the electrical phases of the motor and sensor
    // set angle -90(270 = 3PI/2) degrees
    setPhaseVoltage(voltage_align, 0, _3PI_2);
    _delay(700);
    // read the sensor
    m_encoder->update();
    // get the current zero electric angle
    m_zeroElectricAngle = 0;
    m_zeroElectricAngle = electricalAngle();
    //zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    _delay(20);
    // stop everything
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  } else {
    SIMPLEFOC_DEBUG("MOT: Skip offset calib.");
  }
  return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int BLDCMotor::absoluteZeroSearch() {
  // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
  //                    of float is sufficient.
  SIMPLEFOC_DEBUG("MOT: Index search...");
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while (sensor->needsSearch() && shaft_angle < _2PI) {
    angleOpenloop(1.5f * _2PI);
    // call important for some sensors not to loose count
    // not needed for the search
    sensor->update();
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found

  return !sensor->needsSearch();
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void BLDCMotor::loopFOC() {
  // update sensor - do this even in open-loop mode, as user may be switching between modes and we could lose track
  //                 of full rotations otherwise.
  if (sensor)
    sensor->update();

  // if open-loop do nothing
  if (controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop)
    return;

  // if disabled do nothing
  if (!enabled)
    return;

  // Needs the update() to be called first
  // This function will not have numerical issues because it uses Sensor::getMechanicalAngle()
  // which is in range 0-2PI
  electrical_angle = electricalAngle();
  switch (torque_controller) {
  case TorqueControlType::voltage:
    // no need to do anything really
    break;
  case TorqueControlType::dc_current:
    if (!current_sense)
      return;
    // read overall current magnitude
    current.q = current_sense->getDCCurrent(electrical_angle);
    // filter the value values
    current.q = LPF_current_q(current.q);
    // calculate the phase voltage
    voltage.q = PID_current_q(current_sp - current.q);
    // d voltage  - lag compensation
    if (_isset(phase_inductance))
      voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
    else
      voltage.d = 0;
    break;
  case TorqueControlType::foc_current:
    if (!current_sense)
      return;
    // read dq currents
    current = current_sense->getFOCCurrents(electrical_angle);
    // filter values
    current.q = LPF_current_q(current.q);
    current.d = LPF_current_d(current.d);
    // calculate the phase voltages
    voltage.q = PID_current_q(current_sp - current.q);
    voltage.d = PID_current_d(-current.d);
    // d voltage - lag compensation - TODO verify
    // if(_isset(phase_inductance)) voltage.d = _constrain( voltage.d - current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
    break;
  default:
    // no torque control selected
    SIMPLEFOC_DEBUG("MOT: no torque control selected!");
    break;
  }

  // set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void BLDCMotor::move(float new_target) {

  // set internal target variable
  if (_isset(new_target))
    target = new_target;

  // downsampling (optional)
  if (motion_cnt++ < motion_downsample)
    return;
  motion_cnt = 0;

  // shaft angle/velocity need the update() to be called first
  // get shaft angle
  // TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a float
  //                        For this reason it is NOT precise when the angles become large.
  //                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a problem
  //                        when switching to a 2-component representation.
  if (controller != MotionControlType::angle_openloop && controller != MotionControlType::velocity_openloop)
    shaft_angle = shaftAngle();// read value even if motor is disabled to keep the monitoring updated but not in openloop mode
  // get angular velocity  TODO the velocity reading probably also shouldn't happen in open loop modes?
  shaft_velocity = shaftVelocity();// read value even if motor is disabled to keep the monitoring updated

  // if disabled do nothing
  if (!enabled)
    return;

  // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
  if (_isset(KV_rating))
    voltage_bemf = shaft_velocity / (KV_rating * _SQRT3) / _RPM_TO_RADS;
  // estimate the motor current if phase reistance available and current_sense not available
  if (!current_sense && _isset(phase_resistance))
    current.q = (voltage.q - voltage_bemf) / phase_resistance;

  // upgrade the current based voltage limit
  switch (controller) {
  case MotionControlType::torque:
    if (torque_controller == TorqueControlType::voltage) {// if voltage torque control
      if (!_isset(phase_resistance))
        voltage.q = target;
      else
        voltage.q = target * phase_resistance + voltage_bemf;
      voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
      // set d-component (lag compensation if known inductance)
      if (!_isset(phase_inductance))
        voltage.d = 0;
      else
        voltage.d = _constrain(-target * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
    } else {
      current_sp = target;// if current/foc_current torque control
    }
    break;
  case MotionControlType::angle:
    // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
    //                        the angles are large. This results in not being able to command small changes at high position values.
    //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
    // angle set point
    shaft_angle_sp = target;
    // calculate velocity set point
    shaft_velocity_sp = feed_forward_velocity + P_angle(shaft_angle_sp - shaft_angle);
    shaft_velocity_sp = _constrain(shaft_velocity_sp, -velocity_limit, velocity_limit);
    // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
    current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);// if voltage torque control
    // if torque controlled through voltage
    if (torque_controller == TorqueControlType::voltage) {
      // use voltage if phase-resistance not provided
      if (!_isset(phase_resistance))
        voltage.q = current_sp;
      else
        voltage.q = _constrain(current_sp * phase_resistance + voltage_bemf, -voltage_limit, voltage_limit);
      // set d-component (lag compensation if known inductance)
      if (!_isset(phase_inductance))
        voltage.d = 0;
      else
        voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
    }
    break;
  case MotionControlType::velocity:
    // velocity set point - sensor precision: this calculation is numerically precise.
    shaft_velocity_sp = target;
    // calculate the torque command
    current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);// if current/foc_current torque control
    // if torque controlled through voltage control
    if (torque_controller == TorqueControlType::voltage) {
      // use voltage if phase-resistance not provided
      if (!_isset(phase_resistance))
        voltage.q = current_sp;
      else
        voltage.q = _constrain(current_sp * phase_resistance + voltage_bemf, -voltage_limit, voltage_limit);
      // set d-component (lag compensation if known inductance)
      if (!_isset(phase_inductance))
        voltage.d = 0;
      else
        voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
    }
    break;
  case MotionControlType::velocity_openloop:
    // velocity control in open loop - sensor precision: this calculation is numerically precise.
    shaft_velocity_sp = target;
    voltage.q = velocityOpenloop(shaft_velocity_sp);// returns the voltage that is set to the motor
    voltage.d = 0;
    break;
  case MotionControlType::angle_openloop:
    // angle control in open loop -
    // TODO sensor precision: this calculation NOT numerically precise, and subject
    //                        to the same problems in small set-point changes at high angles
    //                        as the closed loop version.
    shaft_angle_sp = target;
    voltage.q = angleOpenloop(shaft_angle_sp);// returns the voltage that is set to the motor
    voltage.d = 0;
    break;
  }
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca, _sa;

  switch (foc_modulation) {
  case FOCModulationType::Trapezoid_120:
    // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
    // determine the sector
    sector = 6 * (_normalizeAngle(angle_el + _PI_6) / _2PI);// adding PI/6 to align with other modes
    // centering the voltages around either
    // modulation_centered == true > driver.voltage_limit/2
    // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
    center = modulation_centered ? (driver->voltage_limit) / 2 : Uq;

    if (trap_120_map[sector][0] == _HIGH_IMPEDANCE) {
      Ua = center;
      Ub = trap_120_map[sector][1] * Uq + center;
      Uc = trap_120_map[sector][2] * Uq + center;
      driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON);// disable phase if possible
    } else if (trap_120_map[sector][1] == _HIGH_IMPEDANCE) {
      Ua = trap_120_map[sector][0] * Uq + center;
      Ub = center;
      Uc = trap_120_map[sector][2] * Uq + center;
      driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);// disable phase if possible
    } else {
      Ua = trap_120_map[sector][0] * Uq + center;
      Ub = trap_120_map[sector][1] * Uq + center;
      Uc = center;
      driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);// disable phase if possible
    }

    break;

  case FOCModulationType::Trapezoid_150:
    // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
    // determine the sector
    sector = 12 * (_normalizeAngle(angle_el + _PI_6) / _2PI);// adding PI/6 to align with other modes
    // centering the voltages around either
    // modulation_centered == true > driver.voltage_limit/2
    // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
    center = modulation_centered ? (driver->voltage_limit) / 2 : Uq;

    if (trap_150_map[sector][0] == _HIGH_IMPEDANCE) {
      Ua = center;
      Ub = trap_150_map[sector][1] * Uq + center;
      Uc = trap_150_map[sector][2] * Uq + center;
      driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON);// disable phase if possible
    } else if (trap_150_map[sector][1] == _HIGH_IMPEDANCE) {
      Ua = trap_150_map[sector][0] * Uq + center;
      Ub = center;
      Uc = trap_150_map[sector][2] * Uq + center;
      driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);// disable phase if possible
    } else if (trap_150_map[sector][2] == _HIGH_IMPEDANCE) {
      Ua = trap_150_map[sector][0] * Uq + center;
      Ub = trap_150_map[sector][1] * Uq + center;
      Uc = center;
      driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);// disable phase if possible
    } else {
      Ua = trap_150_map[sector][0] * Uq + center;
      Ub = trap_150_map[sector][1] * Uq + center;
      Uc = trap_150_map[sector][2] * Uq + center;
      driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);// enable all phases
    }

    break;

  case FOCModulationType::SinePWM:
  case FOCModulationType::SpaceVectorPWM:
    // Sinusoidal PWM modulation
    // Inverse Park + Clarke transformation
    _sincos(angle_el, &_sa, &_ca);

    // Inverse park transform
    Ualpha = _ca * Ud - _sa * Uq;// -sin(angle) * Uq;
    Ubeta = _sa * Ud + _ca * Uq; //  cos(angle) * Uq;

    // Clarke transform
    Ua = Ualpha;
    Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
    Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

    center = m_driver->voltage_limit / 2;
    if (foc_modulation == FOCModulationType::SpaceVectorPWM) {
      // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
      // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
      // Midpoint Clamp
      float Umin = min(Ua, min(Ub, Uc));
      float Umax = max(Ua, max(Ub, Uc));
      center -= (Umax + Umin) / 2;
    }

    if (!modulation_centered) {
      float Umin = min(Ua, min(Ub, Uc));
      Ua -= Umin;
      Ub -= Umin;
      Uc -= Umin;
    } else {
      Ua += center;
      Ub += center;
      Uc += center;
    }

    break;
  }

  // set the voltages in driver
  m_driver->setPwm(Ua, Ub, Uc);
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float BLDCMotor::velocityOpenloop(float target_velocity) {
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
  // for display purposes
  shaft_velocity = target_velocity;

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if (_isset(phase_resistance)) {
    Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    // recalculate the current
    current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float BLDCMotor::angleOpenloop(float target_angle) {
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
  //                        where small position changes are no longer captured by the precision of floats
  //                        when the total position is large.
  if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts)) {
    shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit) * Ts;
    shaft_velocity = velocity_limit;
  } else {
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if (_isset(phase_resistance)) {
    Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    // recalculate the current
    current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  // sensor precision: this calculation is OK due to the normalisation
  setPhaseVoltage(Uq, 0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

}// namespace foc
