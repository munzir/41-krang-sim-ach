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
 * @file floating_base_state_sensor.cpp
 * @author Munzir Zafar
 * @date Nov 24, 2018
 * @brief Sensor that simulates the behavior of the inertial measurement unit
 * (IMU) fixed on krang's base link
 */

#include "floating_base_state_sensor.h"

FloatingBaseStateSensor::Update() {
  // We define a frame of reference "imu". Origin on the center of the physical
  // imu, x, y and z axes along the -y, +z and -x axes of the physical imu
  // (labeled). Henceforth x_imu, y_imu and z_imu refer to the axes of frame
  // "imu"

  // The angle that x_imu makes wrt the -z-axis of the base frame
  static const double mount_angle = -.7853981634;

  // Axes of "imu" frame represented in the base frame
  Eigen::Vector3d x_imu_wrt_base, y_imu_wrt_base, z_imu_wrt_base;
  x_imu_wrt_base << sin(mount_angle), 0, -cos(mount_angle);
  y_imu_wrt_base << cos(mount_angle), 0, sin(mount_angle);
  z_imu_wrt_base << 0, -1, 0;

  // Rotation matrix of "imu" frame wrt base frame
  Eigen::Matrix3d rot_imu_wrt_base;
  rot_imu_wrt_base << x_imu_wrt_base, y_imu_wrt_base, z_imu_wrt_base;

  // Rotation of base frame wrt to the world
  Eigen::Matrix3d rot_base_wrt_world;
  rot_base_wrt_world = robot_->getBodyNode("Base")->getTransform().rotation();

  // Gravity vector (-z-axis of the world frame) represented in base frame
  Eigen::Vector3d gravity_direction_wrt_base;
  gravity_direction_wrt_base = -rot_base_wrt_world.transpose().col(2);

  // Gravity vector represented in the "imu" frame
  Eigen::Vector3d gravity_direction_wrt_imu;
  gravity_direction_wrt_imu =
      rot_imu_wrt_base.transpose() * gravity_direction_wrt_base;

  // Angular velocity of the base frame represented in the world
  Eigen::Vector3d base_ang_vel_wrt_world;
  base_ang_vel_wrt_world = robot_->getBodyNode("Base")->getAngularVelocity();

  // Angular velocity of the base frame in the "imu" frame
  Eigen::Vector3d base_ang_vel_wrt_imu;
  base_ang_vel_wrt_imu = rot_imu_wrt_base.transpose() *
                         rot_base_wrt_world.transpose() *
                         base_ang_vel_wrt_world;

  // Update the members of our object
  gravity_direction_.x_ = gravity_direction_wrt_imu(0);
  gravity_direction_.y_ = gravity_direction_wrt_imu(1);
  gravity_direction_.z_ = gravity_direction_wrt_imu(2);
  angular_velocity_.x_ = angular_velocity_wrt_imu(0);
  angular_velocity_.y_ = angular_velocity_wrt_imu(1);
  angular_velocity_.z_ = angular_velocity_wrt_imu(2);
}
