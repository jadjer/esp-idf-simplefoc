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

#include <foc/driver/interface/Driver.hpp>

namespace foc {

class DriverBase : public interface::Driver {
    public:
        using Frequency = float;
        using Voltage = float;
        using DutyCycle = float;

    public:
        ~DriverBase() override = default;

    public:
        /**
               * Set phase voltages to the hardware
               *
               * @param Ua - phase A voltage
               * @param Ub - phase B voltage
               * @param Uc - phase C voltage
              */
        virtual void setPwm(float Ua, float Ub, float Uc) = 0;

        /**
               * Set phase state, enable/disable
               *
               * @param sc - phase A state : active / disabled ( high impedance )
               * @param sb - phase B state : active / disabled ( high impedance )
               * @param sa - phase C state : active / disabled ( high impedance )
              */
        virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;

    protected:
        Frequency const m_pwmFrequency = 0;    //!< pwm frequency value in hertz
        Voltage const m_voltagePowerSupply = 0;//!< power supply voltage
        Voltage const m_voltageLimit = 0;      //!< limiting voltage set to the motor

    protected:
        bool m_initialized = false;    //!< true if driver was successfully initialized
        bool m_enableActiveHigh = true;//!< enable pin should be set to high to enable the driver (default is HIGH)

    protected:
        DutyCycle m_dutyCycleA;
        DutyCycle m_dutyCycleB;
        DutyCycle m_dutyCycleC;
    };

}// namespace foc::interface
