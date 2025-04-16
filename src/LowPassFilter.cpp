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

#include "foc/utils/LowPassFilter.hpp"

#include <esp_timer.h>

namespace foc {

LowPassFilter::LowPassFilter(float time_constant) : Tf(time_constant), y_prev(0.0f) { timestamp_prev = esp_timer_get_time(); }

float LowPassFilter::operator()(float x) {
  unsigned long timestamp = esp_timer_get_time();
  float dt = (timestamp - timestamp_prev) * 1e-6f;

  if (dt < 0.0f)
    dt = 1e-3f;
  else if (dt > 0.3f) {
    y_prev = x;
    timestamp_prev = timestamp;
    return x;
  }

  float alpha = Tf / (Tf + dt);
  float y = alpha * y_prev + (1.0f - alpha) * x;
  y_prev = y;
  timestamp_prev = timestamp;
  return y;
}

} // namespace foc
