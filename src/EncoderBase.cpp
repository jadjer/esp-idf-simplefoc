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

#include "foc/encoder/EncoderBase.hpp"

#include <cmath>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

namespace foc {

    void EncoderBase::init() {
        // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
        getSensorAngle();

        vTaskDelay(pdMS_TO_TICKS(1));

        m_velocityAnglePrev = getSensorAngle();// call again
        m_velocityAnglePrevTimestamp = esp_timer_get_time();

        vTaskDelay(pdMS_TO_TICKS(1));

        getSensorAngle();

        vTaskDelay(pdMS_TO_TICKS(1));

        m_anglePrev = getSensorAngle();
        m_anglePrevTimestamp = esp_timer_get_time();
    }

    void EncoderBase::update() {
        float val = getSensorAngle();
        if (val < 0) {// sensor angles are strictly non-negative. Negative values are used to signal errors.
            return;   // TODO signal error, e.g. via a flag and counter
        }

        m_anglePrevTimestamp = esp_timer_get_time();

        float d_angle = val - m_anglePrev;
        // if overflow happened track it as full rotation
        if (abs(d_angle) > (0.8f * M_PI * 2)) {
            m_fullRotations += (d_angle > 0) ? -1 : 1;
        }

        m_anglePrev = val;
    }

/** get current angular velocity (rad/s) */
    float EncoderBase::getVelocity() {
        // calculate sample time
        float Ts = (m_anglePrevTimestamp - m_velocityAnglePrevTimestamp) * 1e-6f;
        if (Ts < 0.0f) {// handle micros() overflow - we need to reset vel_angle_prev_ts
            m_velocityAnglePrev = m_anglePrev;
            m_velocityFullRotations = m_fullRotations;
            m_velocityAnglePrevTimestamp = m_anglePrevTimestamp;
            return m_velocity;
        }
        if (Ts < m_minElapsedTime) {
            return m_velocity;// don't update velocity if deltaT is too small
        }

        m_velocity = ((float) (m_fullRotations - m_velocityFullRotations) * M_PI * 2 + (m_anglePrev - m_velocityAnglePrev)) / Ts;
        m_velocityAnglePrev = m_anglePrev;
        m_velocityFullRotations = m_fullRotations;
        m_velocityAnglePrevTimestamp = m_anglePrevTimestamp;

        return m_velocity;
    }


    EncoderBase::Angle EncoderBase::getMechanicalAngle() {
        return m_anglePrev;
    }

    EncoderBase::Angle EncoderBase::getAngle() {
        return (float) m_fullRotations * M_PI * 2 + m_anglePrev;
    }

    EncoderBase::PreciseAngle EncoderBase::getPreciseAngle() {
        return (double) m_fullRotations * (double) M_PI * 2 + (double) m_anglePrev;
    }

    EncoderBase::Rotations EncoderBase::getFullRotations() {
        return m_fullRotations;
    }

    int EncoderBase::needsSearch() {
        return 0;// default false
    }

}
