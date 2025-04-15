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
#include <esp_err.h>

namespace foc::interface {

class Motor {
public:
  using Error = esp_err_t;
  using Target = float;

public:
  virtual ~Motor() = default;

public:
  virtual Error initFOC() = 0;

public:
  virtual void disable() = 0;

  virtual void enable() = 0;

public:
  virtual void loopFOC() = 0;

public:
  virtual void move(Target target) = 0;
};

} // namespace foc::interface
