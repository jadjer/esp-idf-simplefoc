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

/**
 * @namespace foc::interface
 */
namespace foc {

/**
 *  Direction structure
 */
enum Direction : std::int8_t {
  CW [[maybe_unused]] = 1,     // clockwise
  CCW [[maybe_unused]] = -1,   // counter clockwise
  UNKNOWN [[maybe_unused]] = 0 // not yet known or invalid state
};

/**
 *  Pullup configuration structure
 */
enum Pullup : std::uint8_t {
  USE_INTERN [[maybe_unused]] = 0x00, //!< Use internal pullups
  USE_EXTERN [[maybe_unused]] = 0x01  //!< Use external pullups
};

namespace interface {

class Encoder {
public:
  using Angle = float;
  using Velocity = float;
  using Rotations = std::int64_t;

public:
  virtual ~Encoder() = default;

public:
  [[nodiscard]] [[maybe_unused]] virtual auto getSensorAngle() -> Angle = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getAngle() -> Angle = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getVelocity() -> Velocity = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getRotations() -> Rotations = 0;
};

} // namespace interface
} // namespace foc
