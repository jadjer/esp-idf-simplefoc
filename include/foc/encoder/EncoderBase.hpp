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

#include <foc/encoder/interface/Encoder.hpp>

/**
 * @namespace foc
 */
namespace foc {

/**
 * @class BaseEncoder
 *
 * Sensor abstract class defintion
 *
 * This class is purposefully kept simple, as a base for all kinds of sensors.
 * Currently we have Encoders, Magnetic Encoders and Hall Sensor
 * implementations. This base class extracts the most basic common features so
 * that a FOC driver can obtain the data it needs for operation.
 *
 * To implement your own sensors, create a sub-class of this class, and
 * implement the getSensorAngle() method. getSensorAngle() returns a float
 * value, in radians, representing the current shaft angle in the range 0 to
 * 2*PI (one full turn).
 *
 * To function correctly, the sensor class update() method has to be called
 * sufficiently quickly. Normally, the BLDCMotor's loopFOC() function calls it
 * once per iteration, so you must ensure to call loopFOC() quickly enough, both
 * for correct motor and sensor operation.
 *
 * The Sensor base class provides an implementation of getVelocity(), and takes
 * care of counting full revolutions in a precise way, but if you wish you can
 * additionally override these methods to provide more optimal implementations
 * for your hardware.
 *
 */
class EncoderBase : public interface::Encoder {
public:
  using Time = std::int64_t;
  using PreciseAngle = double;

public:
  ~EncoderBase() override = default;

public:
  /**
   * Get mechanical shaft angle in the range 0 to 2PI. This value will be as
   * precise as possible with the hardware. Base implementation uses the values
   * returned by update() so that the same values are returned until update() is
   * called again.
   */
  virtual auto getMechanicalAngle() -> Angle;

  /**
   * Get current position (in rad) including full rotations and shaft angle.
   * Base implementation uses the values returned by update() so that the same
   * values are returned until update() is called again.
   * Note that this value has limited precision as the number of rotations
   * increases, because the limited precision of float can't capture the large
   * angle of the full rotations and the small angle of the shaft angle at the
   * same time.
   */
  auto getAngle() -> Angle override;

  /**
   * On architectures supporting it, this will return a double precision
   * position value, which should have improved precision for large position
   * values. Base implementation uses the values returned by update() so that
   * the same values are returned until update() is called again.
   */
  virtual auto getPreciseAngle() -> PreciseAngle;

  /**
   * Get current angular velocity (rad/s)
   * Can be overridden in subclasses. Base implementation uses the values
   * returned by update() so that it only makes sense to call this if update()
   * has been called in the meantime.
   */
  auto getVelocity() -> Velocity override;

  /**
   * Get the number of full rotations
   * Base implementation uses the values returned by update() so that the same
   * values are returned until update() is called again.
   */
  virtual auto getFullRotations() -> Rotations;

public:
  /**
   * Updates the sensor values by reading the hardware sensor.
   * Some implementations may work with interrupts, and not need this.
   * The base implementation calls getSensorAngle(), and updates internal
   * fields for angle, timestamp and full rotations.
   * This method must be called frequently enough to guarantee that full
   * rotations are not "missed" due to infrequent polling.
   * Override in subclasses if alternative behaviours are required for your
   * sensor hardware.
   */
  virtual void update();

public:
  /**
   * returns 0 if it does need search for absolute zero
   * 0 - magnetic sensor (& encoder with index which is found)
   * 1 - ecoder with index (with index not found yet)
   */
  virtual auto needsSearch() -> int;

protected:
  /**
   * Get current shaft angle from the sensor hardware, and
   * return it as a float in radians, in the range 0 to 2PI.
   *
   * This method is pure virtual and must be implemented in subclasses.
   * Calling this method directly does not update the base-class internal
   * fields. Use update() when calling from outside code.
   */
  //        virtual float getSensorAngle() = 0;

  /**
   * Call Sensor::init() from your sensor subclass's init method if you want
   * smoother startup The base class init() method calls getSensorAngle()
   * several times to initialize the internal fields to current values, ensuring
   * there is no discontinuity ("jump from zero") during the first calls to
   * sensor.getAngle() and sensor.getVelocity()
   */
  virtual void init();

protected:
  float const m_minElapsedTime = 0.000100; // default is 100 microseconds, or 10kHz

protected:
  Velocity m_velocity = 0.0f;
  Angle m_anglePrev = 0.0f;              // result of last call to getSensorAngle(), used for
                                         // full rotations and velocity
  Time m_anglePrevTimestamp = 0;         // timestamp of last call to getAngle, used for velocity
  Velocity m_velocityAnglePrev = 0.0f;   // angle at last call to getVelocity, used for velocity
  Time m_velocityAnglePrevTimestamp = 0; // last velocity calculation timestamp
  Rotations m_fullRotations = 0;         // full rotation tracking
  Velocity m_velocityFullRotations = 0;  // previous full rotation value for velocity calculation
};

} // namespace foc
