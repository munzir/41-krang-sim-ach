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
 * @file dart_world.h
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief Header for dart_world.cpp that creates the world with all
 * objects loaded in desired intial configuration
 */

#ifndef KRANG_SIMULATION_LOAD_OBJECTS_H_
#define KRANG_SIMULATION_LOAD_OBJECTS_H_

#include <Eigen/Eigen>    // Eigen::
#include <dart/dart.hpp>  // dart::simulation, dart::dynamics

namespace krang_sim_ach {

namespace dart_world {

struct KrangInitPoseParams {
  // Initial pose parameters
  double heading_init;
  double q_base_init;
  Eigen::Vector3d xyz_init;
  double q_lwheel_init;
  double q_rwheel_init;
  double q_waist_init;
  double q_torso_init;
  double q_kinect_init;
  Eigen::Matrix<double, 7, 1> q_left_arm_init;
  Eigen::Matrix<double, 7, 1> q_right_arm_init;

  // To have initial pose as a balanced pose or not
  bool init_with_balance_pose;
};

struct KrangPositionLimitParams {
  double rotating_joint_min;
  double rotating_joint_max;
  double bending_joint_min;
  double bending_joint_max;
  double torso_min;
  double torso_max;
  double waist_min;
  double waist_max;
};

struct DartParams {
  // Time step
  double time_step;

  // Path to URDF file
  char krang_urdf_path[1024];

  // Path to URDF file
  char com_params_path[1024];

  // Initial pose parameters
  KrangInitPoseParams init_pose_params;

  // Position limit parameters
  KrangPositionLimitParams position_limit_params;

  // render?
  bool render;

  // marks on ground?
  bool marks_on_ground;
};

//==============================================================================
//// Create the world with objects loaded in desired initial configuration
dart::simulation::WorldPtr CreateWorld(const char* path_to_dart_params,
                                       bool* render, bool* marks_on_ground);

//==============================================================================
//// Read init pose parameters from the config file
void ReadInitPoseParams(const char* config_file, KrangInitPoseParams* params);

//==============================================================================
//// Read position limit parameters from the config file
void ReadPositionLimitParams(const char* config_file,
                             KrangPositionLimitParams* params);

//==============================================================================
//// Read parameters from the config file
void ReadDartParams(const char* config_file, DartParams* params);

//==============================================================================
//// Create a floor
dart::dynamics::SkeletonPtr CreateFloor();

//==============================================================================
//// Create Krang with desired init pose, CoM parameters and joint limits
dart::dynamics::SkeletonPtr CreateKrang(DartParams& params);

//==============================================================================
//// Set CoM parameters on the robot. These params were learnt through
//// data collection in various poses of the robot
// TODO: This CoM params file should list the parameters with names of the links
// Those names should then be used to set these parameters
void SetKrangComParams(const char* com_params_path,
                       dart::dynamics::SkeletonPtr robot);

//==============================================================================
// Given all the initial pose parameters, sets the positions of krang
void SetKrangInitPoseRaw(const KrangInitPoseParams& params,
                         dart::dynamics::SkeletonPtr krang);

//==============================================================================
// Given all the initial pose parameters, sets the positions of krang
// If init_with_balance_pose was requested, set the pose to balanced one
void SetKrangInitPose(KrangInitPoseParams& params,
                      dart::dynamics::SkeletonPtr krang);

//==============================================================================
// Calculating the axis angle representation of orientation from heading_init
// and q_base_init: RotX(pi/2)*RotY(-pi/2+heading_init)*RotX(-q_base_init)
Eigen::AngleAxisd GetKrangBaseAngleAxis(const double& heading_init,
                                        const double& q_base_init);

//==============================================================================
// Set Krang Joint position limits
void SetKrangJointPositionLimits(const KrangPositionLimitParams& params,
                                 dart::dynamics::SkeletonPtr krang);

//==============================================================================
// Get com of only the body excluding the wheels in world frame
Eigen::Vector3d GetBodyCom(dart::dynamics::SkeletonPtr robot);

//==============================================================================
// Get Krang Com in Frame 0
Eigen::Matrix<double, 4, 1> GetKrangCom(dart::dynamics::SkeletonPtr robot);

}  // namespace dart_world

}  // namespace krang_sim_ach
#endif  // KRANG_SIMULATION_LOAD_OBJECTS_H_
