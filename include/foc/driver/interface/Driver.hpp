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

namespace foc {

enum PhaseState : std::uint8_t {
  PHASE_OFF [[maybe_unused]] = 0,
  PHASE_ON [[maybe_unused]] = 1,
  PHASE_HI [[maybe_unused]] = 2,
  PHASE_LO [[maybe_unused]] = 3,
};

namespace interface {

class Driver {
public:
  virtual ~Driver() = default;

public:
  virtual auto init() -> void = 0;
  [[maybe_unused]] virtual auto enable() -> void = 0;
  [[maybe_unused]] virtual auto disable() -> void = 0;

public:
  [[maybe_unused]] virtual auto setPwm(float Ua, float Ub, float Uc) -> void = 0;
  [[maybe_unused]] virtual auto setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) -> void = 0;
};

} // namespace interface
} // namespace foc
