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
 * @file load_objects.h
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief Header for load_objects.cpp that loads objects into krang
 * simulation
 */

#ifndef KRANG_SIMULATION_LOAD_OBJECTS_H_
#define KRANG_SIMULATION_LOAD_OBJECTS_H_

#include <Eigen/Eigen>
#include <dart/dart.hpp>

#include "sim_config.h"

// Given heading and base angle of krang, this function returns the
// angle axis representation for base frame orientation accepted by
// DART
Eigen::AngleAxisd GetKrangBaseAngleAxis(const double& heading_init,
                                        const double& q_base_init);

// Given all the initial pose parameters, sets the positions of krang
template <typename T>
void SetKrangInitPos(const T& params, dart::dynamics::SkeletonPtr krang) {
  Eigen::AngleAxisd aa;
  aa = GetKrangBaseAngleAxis(params.heading_init, params.q_base_init);
  Eigen::Matrix<double, 25, 1> q;
  q << aa.angle() * aa.axis(), params.xyz_init, params.q_lwheel_init,
      params.q_rwheel_init, params.q_waist_init, params.q_torso_init,
      params.q_kinect_init, params.q_left_arm_init, params.q_right_arm_init;
  krang->setPositions(q);
}
// Create a floor
dart::dynamics::SkeletonPtr CreateFloor();

// Create Krang with the initial pose that is specified by params
// Path to krang's urdf is also to be specified in params
dart::dynamics::SkeletonPtr CreateKrang(SimConfig& params);

#endif  // KRANG_SIMULATION_LOAD_OBJECTS_H_
