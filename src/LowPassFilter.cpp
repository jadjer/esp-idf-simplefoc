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

LowPassFilter::LowPassFilter(LowPassFilter::Value timeConstant) : m_filterTime(timeConstant), m_yPrev(0.0f) { m_timestampPrev = esp_timer_get_time(); }

auto LowPassFilter::operator()(LowPassFilter::Value x) -> LowPassFilter::Value {
  LowPassFilter::Time const currentTime = esp_timer_get_time();
  auto const timeDifferent = (currentTime - m_timestampPrev) * 1e-6f;

  if (timeDifferent > 0.3f) {
    m_yPrev = x;
    m_timestampPrev = currentTime;
    return x;
  }

  LowPassFilter::Value const alpha = m_filterTime / (m_filterTime + timeDifferent);
  LowPassFilter::Value const y = alpha * m_yPrev + (1.0f - alpha) * x;

  m_yPrev = y;
  m_timestampPrev = currentTime;

  return y;
}

} // namespace foc
