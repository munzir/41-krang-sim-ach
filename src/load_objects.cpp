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
 * @file load_objects.cpp
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief loads objects into krang simulation
 */

#include "load_objects.h"

#include <Eigen/Eigen>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <memory>

#include "sim_config.h"

//====================================================================
dart::dynamics::SkeletonPtr CreateFloor() {
  using namespace dart::dynamics;

  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  //  body->setFrictionCoeff(1e16);

  // Give the body a shape
  double floor_width = 50;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode =
      body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//====================================================================
// Calculating the axis angle representation of orientation from heading_init
// and q_base_init: RotX(pi/2)*RotY(-pi/2+heading_init)*RotX(-q_base_init)
Eigen::AngleAxisd GetKrangBaseAngleAxis(const double& heading_init,
                                        const double& q_base_init) {
  Eigen::Transform<double, 3, Eigen::Affine> baseTf;
  baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  baseTf.prerotate(Eigen::AngleAxisd(-q_base_init, Eigen::Vector3d::UnitX()))
      .prerotate(
          Eigen::AngleAxisd(-M_PI / 2 + heading_init, Eigen::Vector3d::UnitY()))
      .prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  auto aa = Eigen::AngleAxisd(baseTf.rotation());
  return aa;
}

//====================================================================
dart::dynamics::SkeletonPtr CreateKrang(SimConfig& params) {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang;
  krang = loader.parseSkeleton(params.krang_urdf_path);
  krang->setName("krang");

  // Set the positions
  SetKrangInitPos<SimConfig>(params, krang);

  // If balanced init pose is required
  if (params.init_with_balance_pose) {
    Eigen::Vector3d COM;
    COM = krang->getCOM() - params.xyz_init;
    double th = atan2(COM(0), COM(2));

    // Adjust q_base_init to bring COM on top of wheels and set the positions
    // again
    params.q_base_init -= th;
    SetKrangInitPos(params, krang);
  }

  // Set wheel frictions
  krang->getJoint(0)->setDampingCoefficient(0, 0.5);
  krang->getJoint(1)->setDampingCoefficient(0, 0.5);

  return krang;
}
//====================================================================
Eigen::Matrix<double, 6, 1> GetBaseImuData(dart::dynamics::SkeletonPtr krang) {
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
  rot_base_wrt_world = krang->getBodyNode("Base")->getTransform().rotation();

  // Gravity vector (-z-axis of the world frame) represented in base frame
  Eigen::Vector3d gravity_direction_wrt_base;
  gravity_direction_wrt_base = -rot_base_wrt_world.transpose().col(2);


  // Gravity vector represented in the "imu" frame
  Eigen::Vector3d gravity_direction_wrt_imu;
  gravity_direction_wrt_imu =
      rot_imu_wrt_base.transpose() * gravity_direction_wrt_base;

  // Angular velocity of the base frame represented in the world
  Eigen::Vector3d base_ang_vel_wrt_world;
  base_ang_vel_wrt_world = krang->getBodyNode("Base")->getAngularVelocity();

  // Angular velocity of the base frame in the "imu" frame
  Eigen::Vector3d base_ang_vel_wrt_imu;
  base_ang_vel_wrt_imu = rot_imu_wrt_base.transpose() *
                         rot_base_wrt_world.transpose() *
                         base_ang_vel_wrt_world;

  // Return vector
  Eigen::Matrix<double, 6, 1> imu_data;
  imu_data << gravity_direction_wrt_imu, base_ang_vel_wrt_imu;
  return imu_data;
}
