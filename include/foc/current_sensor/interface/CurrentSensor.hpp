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

#include <esp_err.h>

namespace foc {

struct DQCurrent {
  float d;
  float q;
};

struct PhaseCurrent {
  float a;
  float b;
  float c;
};

struct ABCurrent {
  float alpha;
  float beta;
};

namespace interface {

class CurrentSensor {
public:
  using Error = esp_err_t;
  using Current = float;
  using ElectricalAngle = float;

public:
  virtual ~CurrentSensor() = default;

public:
  virtual auto init() -> Error = 0;
  [[maybe_unused]] virtual auto enable() -> void = 0;
  [[maybe_unused]] virtual auto disable() -> void = 0;

public:
  [[nodiscard]] [[maybe_unused]] virtual auto getPhaseCurrents() -> PhaseCurrent = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getDCCurrent(ElectricalAngle electricalAngle) -> Current = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getFOCCurrents(ElectricalAngle electricalAngle) -> DQCurrent = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getABCurrents(PhaseCurrent current) -> ABCurrent = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getDQCurrents(ABCurrent current, ElectricalAngle electricalAngle) -> DQCurrent = 0;
};

} // namespace interface
} // namespace foc
