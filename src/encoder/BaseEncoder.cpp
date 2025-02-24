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

#include "foc/encoder/BaseEncoder.hpp"

#include "../foc_utils.h"
#include "../time_utils.h"


namespace foc {

void BaseEncoder::update() {
  float val = getSensorAngle();
  if (val < 0)// sensor angles are strictly non-negative. Negative values are used to signal errors.
    return;   // TODO signal error, e.g. via a flag and counter
  angle_prev_ts = _micros();
  float d_angle = val - angle_prev;
  // if overflow happened track it as full rotation
  if (abs(d_angle) > (0.8f * _2PI))
    full_rotations += (d_angle > 0) ? -1 : 1;
  angle_prev = val;
}

/** get current angular velocity (rad/s) */
float BaseEncoder::getVelocity() {
  // calculate sample time
  float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6f;
  if (Ts < 0.0f) {// handle micros() overflow - we need to reset vel_angle_prev_ts
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return velocity;
  }
  if (Ts < min_elapsed_time)
    return velocity;// don't update velocity if deltaT is too small

  velocity = ((float) (full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;
  vel_angle_prev = angle_prev;
  vel_full_rotations = full_rotations;
  vel_angle_prev_ts = angle_prev_ts;
  return velocity;
}

void BaseEncoder::init() {
  // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
  getSensorAngle();// call once
  delayMicroseconds(1);
  vel_angle_prev = getSensorAngle();// call again
  vel_angle_prev_ts = _micros();
  delay(1);
  getSensorAngle();// call once
  delayMicroseconds(1);
  angle_prev = getSensorAngle();// call again
  angle_prev_ts = _micros();
}

float BaseEncoder::getMechanicalAngle() {
  return angle_prev;
}

float BaseEncoder::getAngle() {
  return (float) full_rotations * _2PI + angle_prev;
}

double BaseEncoder::getPreciseAngle() {
  return (double) full_rotations * (double) _2PI + (double) angle_prev;
}

int32_t BaseEncoder::getFullRotations() {
  return full_rotations;
}

int BaseEncoder::needsSearch() {
  return 0;// default false
}

}
