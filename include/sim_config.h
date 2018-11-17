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
 * @file sim_config.h
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief Header for sim_config.cpp that reads configuration parameters
 * from a cfg file
 */

#ifndef KRANG_SIMULATION_CONFIG_H_
#define KRANG_SIMULATION_CONFIG_H_

#include <Eigen/Eigen>

// Structure in which all configurable parameters are read at the beginning of
// the program
struct SimConfig {
  // Path to URDF file
  char krangUrdfPath[1024];

  // Initial pose parameters
  double headingInit;
  double qBaseInit;
  Eigen::Vector3d xyzInit;
  double qLWheelInit;
  double qRWheelInit;
  double qWaistInit;
  double qTorsoInit;
  double qKinectInit;
  Eigen::Matrix<double, 7, 1> qLeftArmInit;
  Eigen::Matrix<double, 7, 1> qRightArmInit;

  // To have initial pose as a balanced pose or not
  bool initWithBalancePose;

  // Somatic daemon identifier
  char daemonIdentifier[128];

  // Daemonize?
  bool daemonize;

  // Channels
  char wheelsCmdChanv[64];
  char wheelsStateChan[64];
  char waistCmdChan[64];
  char waistStateChan[64];
  char torsoCmdChan[64];
  char torsoStateChan[64];
  char leftArmCmdChan[64];
  char leftArmStateChan[64];
  char rightArmCmdChan[64];
  char rightArmStateChan[64];
};

// Function for reading configuration parameters. First argument is the location
// of cfg file from the parameters are to be read. Second argument is the output
// where the parameters are stored
void ReadConfigParams(const char* config_file, SimConfig* params);

#endif  // KRANG_SIMULATION_CONFIG_H_
