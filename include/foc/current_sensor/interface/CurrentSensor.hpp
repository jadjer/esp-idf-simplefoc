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
  using Current = float;
  using ElectricalAngle = float;

public:
  virtual ~CurrentSensor() = default;

public:
  virtual int init() = 0;

public:
  virtual void enable() = 0;

  virtual void disable() = 0;

public:
  virtual PhaseCurrent getPhaseCurrents() = 0;

  virtual Current getDCCurrent(ElectricalAngle electricalAngle) = 0;

  virtual DQCurrent getFOCCurrents(ElectricalAngle electricalAngle) = 0;

  virtual ABCurrent getABCurrents(PhaseCurrent current) = 0;

  virtual DQCurrent getDQCurrents(ABCurrent current, ElectricalAngle electricalAngle) = 0;
};

} // namespace interface
} // namespace foc
