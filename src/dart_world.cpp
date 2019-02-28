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

#include "dart_world.h"

#include <assert.h>                    // assert()
#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <stdio.h>                     // std::cout
#include <string.h>                    // strcpy()
#include <Eigen/Eigen>                 // Eigen::
#include <cstring>                     // strlen
#include <dart/dart.hpp>               // dart::dynamics, dart::simulation
#include <dart/utils/urdf/urdf.hpp>    // dart::utils::DartLoader
#include <iostream>                    // std::istringstream
#include <memory>                      // std::make_shared
#include <string>                      // std::string
#include <vector>                      // std::vector

#include "file_ops.hpp"  // readInputFileAsMatrix()

namespace krang_sim_ach {

namespace dart_world {

//==============================================================================
//// Create the world with objects loaded in desired initial configuration
dart::simulation::WorldPtr CreateWorld(const char* path_to_dart_params,
                                       bool* render) {
  // Read the parameters
  DartParams params;
  ReadDartParams(path_to_dart_params, &params);

  // Create world
  dart::simulation::WorldPtr world =
      std::make_shared<dart::simulation::World>();

  // Load floor
  dart::dynamics::SkeletonPtr floor = CreateFloor();
  world->addSkeleton(floor);

  // Load robot
  dart::dynamics::SkeletonPtr robot = CreateKrang(params);
  world->addSkeleton(robot);

  // Time step
  world->setTimeStep(params.time_step);

  // Render?
  *render = params.render;

  return world;
}

//==============================================================================
//// Read init pose params
void ReadInitPoseParams(const char* config_file, KrangInitPoseParams* params) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << "Reading init pose parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    // Initial heading
    params->heading_init = cfg->lookupFloat(scope, "heading_init");
    std::cout << "heading_init: " << params->heading_init << std::endl;

    // Initial base pitch angle
    params->q_base_init = cfg->lookupFloat(scope, "q_base_init");
    std::cout << "q_base_init: " << params->q_base_init << std::endl;

    // Init base xyz location in the world frame
    str = cfg->lookupString(scope, "xyz_init");
    stream.str(str);
    for (int i = 0; i < 3; i++) stream >> params->xyz_init(i);
    stream.clear();
    std::cout << "xyz_init: " << params->xyz_init.transpose() << std::endl;

    // Initial left wheel angle
    params->q_lwheel_init = cfg->lookupFloat(scope, "q_lwheel_init");
    std::cout << "q_lwheel_init: " << params->q_lwheel_init << std::endl;

    // Initial right wheel angle
    params->q_rwheel_init = cfg->lookupFloat(scope, "q_rwheel_init");
    std::cout << "q_rwheel_init: " << params->q_rwheel_init << std::endl;

    // Initial waist angle
    params->q_waist_init = cfg->lookupFloat(scope, "q_waist_init");
    std::cout << "q_waist_init: " << params->q_waist_init << std::endl;

    // Initial torso angle
    params->q_torso_init = cfg->lookupFloat(scope, "q_torso_init");
    std::cout << "q_torso_init: " << params->q_torso_init << std::endl;

    // Initial kinect angle
    params->q_kinect_init = cfg->lookupFloat(scope, "q_kinect_init");
    std::cout << "q_kinect_init: " << params->q_kinect_init << std::endl;

    // Initial configuration of the left arm
    str = cfg->lookupString(scope, "q_left_arm_init");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->q_left_arm_init(i);
    stream.clear();
    std::cout << "q_left_arm_init: " << params->q_left_arm_init.transpose()
              << std::endl;

    // Initial configuration of the right arm
    str = cfg->lookupString(scope, "q_right_arm_init");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->q_right_arm_init(i);
    stream.clear();
    std::cout << "q_right_arm_init: " << params->q_right_arm_init.transpose()
              << std::endl;

    // To have initial pose as a balanced pose or not
    params->init_with_balance_pose =
        cfg->lookupBoolean(scope, "init_with_balance_pose");
    std::cout << "init_with_balance_pose: "
              << (params->init_with_balance_pose ? "true" : "false")
              << std::endl;
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading init pose config parameters");
  }
  std::cout << std::endl;
}

//==============================================================================
//// Read position limit parameters from a configuration file
void ReadPositionLimitParams(const char* config_file,
                             KrangPositionLimitParams* params) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << "Reading position limit configuration parameters ..."
            << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    // Position limits on rotating joints of the arm
    params->rotating_joint_min = cfg->lookupFloat(scope, "rotating_joint_min");
    std::cout << "rotating_joint_min: " << params->rotating_joint_min
              << std::endl;
    params->rotating_joint_max = cfg->lookupFloat(scope, "rotating_joint_max");
    std::cout << "rotating_joint_max: " << params->rotating_joint_max
              << std::endl;

    // Position limits on bending joints of the arm
    params->bending_joint_min = cfg->lookupFloat(scope, "bending_joint_min");
    std::cout << "bending_joint_min: " << params->bending_joint_min
              << std::endl;
    params->bending_joint_max = cfg->lookupFloat(scope, "bending_joint_max");
    std::cout << "bending_joint_max: " << params->bending_joint_max
              << std::endl;

    // Position limits on torso
    params->torso_min = cfg->lookupFloat(scope, "torso_min");
    std::cout << "torso_min: " << params->torso_min << std::endl;
    params->torso_max = cfg->lookupFloat(scope, "torso_max");
    std::cout << "torso_max: " << params->torso_max << std::endl;

    // Position limits on waist
    params->waist_min = cfg->lookupFloat(scope, "waist_min");
    std::cout << "waist_min: " << params->waist_min << std::endl;
    params->waist_max = cfg->lookupFloat(scope, "waist_max");
    std::cout << "waist_max: " << params->waist_max << std::endl;
  }

  catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading position limit config parameters");
  }
  std::cout << std::endl;
}

//==============================================================================
//// Read parameters from the config file
void ReadDartParams(const char* config_file, DartParams* params) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << "Reading dart world configuration parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    // Read time step
    params->time_step = cfg->lookupFloat(scope, "time_step");
    std::cout << "time_step: " << params->time_step << std::endl;

    // Read the path to Krang urdf file
    strcpy(params->krang_urdf_path,
           cfg->lookupString(scope, "krang_urdf_path"));
    std::cout << "krang_urdf_path: " << params->krang_urdf_path << std::endl;

    // Read the path to Krang urdf file
    strcpy(params->com_params_path,
           cfg->lookupString(scope, "com_params_path"));
    std::cout << "com_params_path: " << params->com_params_path << std::endl;

    // Read init pose params
    ReadInitPoseParams(config_file, &params->init_pose_params);

    // Position limits on rotating joints of the arm
    ReadPositionLimitParams(config_file, &params->position_limit_params);

    // Render?
    params->render = cfg->lookupBoolean(scope, "render");
    std::cout << "render: " << (params->render ? "true" : "false") << std::endl;
  }

  catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading dart world config parameters");
  }
  std::cout << std::endl;
}

//==============================================================================
//// Create a floor
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

//==============================================================================
//// Create Krang with desired init pose, CoM parameters and joint limits
dart::dynamics::SkeletonPtr CreateKrang(DartParams& params) {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang;
  krang = loader.parseSkeleton(params.krang_urdf_path);
  krang->setName("krang");

  // Set CoM parameters
  if (strlen(params.com_params_path) != 0)
    SetKrangComParams(params.com_params_path, krang);
  else
    std::cout << "Using default com parameters" << std::endl;

  // Set the positions
  SetKrangInitPose(params.init_pose_params, krang);

  // Set position limits
  std::cout << " setting krang joint position limits ... " << std::endl;
  SetKrangJointPositionLimits(params.position_limit_params, krang);
  std::cout << " done setting krang joint position limits ... " << std::endl;

  // Set wheel frictions
  // TODO: Is this correct?
  krang->getJoint(0)->setDampingCoefficient(0, 0.5);
  krang->getJoint(1)->setDampingCoefficient(0, 0.5);

  return krang;
}

//==============================================================================
//// Set CoM parameters on the robot. These params were learnt through
//// data collection in various poses of the robot
// TODO: This CoM params file should list the parameters with names of the links
// Those names should then be used to set these parameters
void SetKrangComParams(const char* com_params_path,
                       dart::dynamics::SkeletonPtr robot) {
  // Load the parameters from the file
  Eigen::MatrixXd beta;
  try {
    beta = readInputFileAsMatrix(std::string(com_params_path));
  } catch (exception& e) {
    std::cout << e.what() << std::endl;
    assert(false && "Problem loading CoM parameters ... ");
  }

  // Set the parameters in the robot
  Eigen::Vector3d bodyMCOM;
  int num_body_params = 4;
  double mi;
  int numBodies = beta.cols() / num_body_params;
  for (int i = 0; i < numBodies; i++) {
    mi = beta(0, i * num_body_params);
    bodyMCOM(0) = beta(0, i * num_body_params + 1);
    bodyMCOM(1) = beta(0, i * num_body_params + 2);
    bodyMCOM(2) = beta(0, i * num_body_params + 3);

    robot->getBodyNode(i)->setMass(mi);
    robot->getBodyNode(i)->setLocalCOM(bodyMCOM / mi);
  }
}

//==============================================================================
// Given all the initial pose parameters, sets the positions of krang
void SetKrangInitPoseRaw(const KrangInitPoseParams& params,
                         dart::dynamics::SkeletonPtr krang) {
  Eigen::AngleAxisd aa;
  aa = GetKrangBaseAngleAxis(params.heading_init, params.q_base_init);
  Eigen::Matrix<double, 6, 1> q_base;
  q_base << aa.angle() * aa.axis(), params.xyz_init;
  krang->setPositions({0, 1, 2, 3, 4, 5}, q_base);
  krang->getJoint("JLWheel")->setPosition(0, params.q_lwheel_init);
  krang->getJoint("JRWheel")->setPosition(0, params.q_rwheel_init);
  krang->getJoint("JWaist")->setPosition(0, params.q_waist_init);
  krang->getJoint("JTorso")->setPosition(0, params.q_torso_init);
  std::vector<std::string> left_arm_joint_names = {"LJ1", "LJ2", "LJ3", "LJ4",
                                                   "LJ5", "LJ6", "LJFT"};
  std::vector<std::string> right_arm_joint_names = {"RJ1", "RJ2", "RJ3", "RJ4",
                                                    "RJ5", "RJ6", "RJFT"};
  for (int i = 0; i < 7; i++) {
    krang->getJoint(left_arm_joint_names[i])
        ->setPosition(0, params.q_left_arm_init(i));
    krang->getJoint(right_arm_joint_names[i])
        ->setPosition(0, params.q_right_arm_init(i));
  }
}

//==============================================================================
// Given all the initial pose parameters, sets the positions of krang
// If init_with_balance_pose was requested, set the pose to balanced one
void SetKrangInitPose(KrangInitPoseParams& params,
                      dart::dynamics::SkeletonPtr krang) {
  // Set the positions
  SetKrangInitPoseRaw(params, krang);

  // If balanced init pose is required
  if (params.init_with_balance_pose) {
    Eigen::Vector3d COM;
    COM = krang->getCOM() - params.xyz_init;
    double th = atan2(COM(0), COM(2));

    // Adjust q_base_init to bring COM on top of wheels and set the positions
    // again
    params.q_base_init -= th;
    SetKrangInitPoseRaw(params, krang);
  }
}

//==============================================================================
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

//==============================================================================
// Set Krang Joint position limits
void SetKrangJointPositionLimits(const KrangPositionLimitParams& params,
                                 dart::dynamics::SkeletonPtr krang) {
  krang->getJoint("JWaist")->setPositionLowerLimit(0, params.waist_min);
  krang->getJoint("JWaist")->setPositionUpperLimit(0, params.waist_max);
  krang->getJoint("JTorso")->setPositionLowerLimit(0, params.torso_min);
  krang->getJoint("JTorso")->setPositionUpperLimit(0, params.torso_max);
  std::vector<std::string> rotating_joint_names = {"LJ1", "LJ3", "LJ5", "LJFT",
                                                   "RJ1", "RJ3", "RJ5", "RJFT"};
  for (int i = 0; i < rotating_joint_names.size(); i++) {
    krang->getJoint(rotating_joint_names[i])
        ->setPositionLowerLimit(0, params.rotating_joint_min);
    krang->getJoint(rotating_joint_names[i])
        ->setPositionUpperLimit(0, params.rotating_joint_max);
  }
  std::vector<std::string> bending_joint_names = {"LJ2", "LJ4", "LJ6",
                                                  "RJ2", "RJ4", "RJ6"};
  for (int i = 0; i < bending_joint_names.size(); i++) {
    krang->getJoint(bending_joint_names[i])
        ->setPositionLowerLimit(0, params.bending_joint_min);
    krang->getJoint(bending_joint_names[i])
        ->setPositionUpperLimit(0, params.bending_joint_max);
  }
}

//==============================================================================
Eigen::Vector3d GetBodyCom(dart::dynamics::SkeletonPtr robot) {
  dart::dynamics::BodyNodePtr lwheel = robot->getBodyNode("LWheel");
  dart::dynamics::BodyNodePtr rwheel = robot->getBodyNode("RWheel");
  double wheel_mass = lwheel->getMass();
  double full_mass = robot->getMass();
  return (full_mass * robot->getCOM() - wheel_mass * lwheel->getCOM() -
          wheel_mass * rwheel->getCOM()) /
         (full_mass - 2 * wheel_mass);
}

//==============================================================================
Eigen::Matrix<double, 4, 1> GetKrangCom(dart::dynamics::SkeletonPtr robot) {
  Eigen::Matrix3d base_rot =
      robot->getBodyNode("Base")->getTransform().rotation();
  double psi = atan2(base_rot(0, 0), -base_rot(1, 0));
  Eigen::Matrix3d rot0;
  rot0 << cos(psi), sin(psi), 0,  //
      -sin(psi), cos(psi), 0,     //
      0, 0, 1;
  // return rot0 * (robot->getCOM() - robot->getPositions().segment(3, 3));
  Eigen::Matrix<double, 4, 1> output;
  output << (GetBodyCom(robot) - robot->getPositions().segment(3, 3)), psi;
  return output;
}

}  // namespace dart_world

}  // namespace krang_sim_ach
