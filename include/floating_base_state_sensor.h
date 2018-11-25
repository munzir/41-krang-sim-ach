/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file floating_base_state_sensor.h
 * @author Munzir Zafar
 * @date Nov 24, 2018
 * @brief Sensor that simulates the behavior of inertial measurement unit (IMU)
 * fixed on krang's base link
 */

#ifndef KRANG_SIMULATION_FLOATING_BASE_STATE_SENSOR_H_
#define KRANG_SIMULATION_FLOATING_BASE_STATE_SENSOR_H_

#include <dart/dart.hpp> // dart::dynamics::

#include "sensor.h"

class FloatingBaseStateSensor : public SensorBase {
 public:
  FloatingBaseStateSensor(dart::dynamics::SkeletonPtr robot) : robot_(robot) {}
  ~FloatingBaseStateSensor();

  void Update();
  void Destroy() {}

  // Krang robot pointer
  dart::dynamics::SkeletonPtr robot_;

  // A free vector represented in our imu frame of reference whose axes are
  // chosen differently from physical IMU axes because that is the data coming
  // out of it
  struct Vector {
    double x_;  // component along x-axis of the imu frame of reference which is
                // -Y axis on the physical IMU
    double y_;  // component along y-axis of the imu frame of reference which is
                // +Z axis on the physical IMU
    double z_;  // component along z-axis of the imu frame of reference which is
                // -X axis on the physical IMU
  };
  // gravity vector direction
  Vector gravity_direction_;
  // base link's angular velocity
  Vector angular_velocity_;
}

#endif  // KRANG_SIMULATION_FLOATING_BASE_STATE_SENSOR_H_
