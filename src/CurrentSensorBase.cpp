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

#include "foc/current_sensor/CurrentSensorBase.hpp"

namespace foc {

// get current magnitude
//   - absolute  - if no electrical_angle provided
//   - signed    - if angle provided
float CurrentSensorBase::getDCCurrent(CurrentSensorBase::ElectricalAngle electricalAngle) {
  // read current phase currents
  PhaseCurrent current = getPhaseCurrents();

  // calculate clarke transform
  ABCurrent ABcurrent = getABCurrents(current);

  // current sign - if motor angle not provided the magnitude is always positive
  float sign = 1;

  // if motor angle provided function returns signed value of the current
  // determine the sign of the current
  // sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1
  if (electricalAngle) {
    float ct;
    float st;
    _sincos(electricalAngle, &st, &ct);
    sign = (ABcurrent.beta * ct - ABcurrent.alpha * st) > 0 ? 1 : -1;
  }
  // return current magnitude
  return sign * _sqrt(ABcurrent.alpha * ABcurrent.alpha + ABcurrent.beta * ABcurrent.beta);
}

// function used with the foc algorithm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents
//   - using getPhaseCurrents and getABCurrents internally
DQCurrent CurrentSensorBase::getFOCCurrents(CurrentSensorBase::ElectricalAngle electricalAngle) {
  // read current phase currents
  PhaseCurrent current = getPhaseCurrents();

  // calculate clarke transform
  ABCurrent ABcurrent = getABCurrents(current);

  // calculate park transform
  DQCurrent return_current = getDQCurrents(ABcurrent, electricalAngle);

  return return_current;
}

// function used with the foc algorithm
//   calculating Alpha Beta currents from phase currents
//   - function calculating Clarke transform of the phase currents
ABCurrent CurrentSensorBase::getABCurrents(PhaseCurrent current) {

  // check if driver is an instance of StepperDriver
  // if so there is no need to Clarke transform
  if (driver_type == DriverType::Stepper) {
    ABCurrent return_ABcurrent;
    return_ABcurrent.alpha = current.a;
    return_ABcurrent.beta = current.b;
    return return_ABcurrent;
  }

  // otherwise it's a BLDC motor and
  // calculate clarke transform
  float i_alpha, i_beta;
  if (!current.c) {
    // if only two measured currents
    i_alpha = current.a;
    i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
  } else if (!current.a) {
    // if only two measured currents
    float a = -current.c - current.b;
    i_alpha = a;
    i_beta = _1_SQRT3 * a + _2_SQRT3 * current.b;
  } else if (!current.b) {
    // if only two measured currents
    float b = -current.a - current.c;
    i_alpha = current.a;
    i_beta = _1_SQRT3 * current.a + _2_SQRT3 * b;
  } else {
    // signal filtering using identity a + b + c = 0. Assumes measurement error
    // is normally distributed.
    float mid = (1.f / 3) * (current.a + current.b + current.c);
    float a = current.a - mid;
    float b = current.b - mid;
    i_alpha = a;
    i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
  }

  ABCurrent return_ABcurrent;
  return_ABcurrent.alpha = i_alpha;
  return_ABcurrent.beta = i_beta;
  return return_ABcurrent;
}

// function used with the foc algorithm
//   calculating D and Q currents from Alpha Beta currents and electrical angle
//   - function calculating Clarke transform of the phase currents
DQCurrent CurrentSensorBase::getDQCurrents(ABCurrent current, CurrentSensorBase::ElectricalAngle electricalAngle) {
  // calculate park transform
  float ct;
  float st;
  _sincos(electricalAngle, &st, &ct);
  DQCurrent return_current;
  return_current.d = current.alpha * ct + current.beta * st;
  return_current.q = current.beta * ct - current.alpha * st;
  return return_current;
}

void CurrentSensorBase::enable() {
  // nothing is done here, but you can override this function
};

void CurrentSensorBase::disable() {
  // nothing is done here, but you can override this function
};

// Helper function to read and average phase currents
PhaseCurrent CurrentSensorBase::readAverageCurrents(int N) {
  PhaseCurrent c = getPhaseCurrents();
  for (int i = 0; i < N; i++) {
    PhaseCurrent c1 = getPhaseCurrents();
    c.a = c.a * 0.6f + 0.4f * c1.a;
    c.b = c.b * 0.6f + 0.4f * c1.b;
    c.c = c.c * 0.6f + 0.4f * c1.c;
    _delay(3);
  }
  return c;
};

} // namespace foc
