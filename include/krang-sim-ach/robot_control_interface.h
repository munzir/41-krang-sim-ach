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
 * @file robot_control_interface.h
 * @author Munzir Zafar
 * @date Nov 23, 2018
 * @brief Header for RobotControlInterface class that is a composite of
 * MotorGroup and SensorGroup objects to allow external interface to access
 * robot states and control robot actuators
 */

#ifndef KRANG_SIMULATION_ROBOT_CONTROL_INTERFACE_H_
#define KRANG_SIMULATION_ROBOT_CONTROL_INTERFACE_H_

#include <dart/dart.hpp>  // dart::dynamics
#include <string>         // std::string
#include <vector>         // std::vector

#include "ach_interface.h"  // InterfaceContext
#include "motor_group.h"    // MotorGroupBase
#include "sensor_group.h"   // SensorGroupBase

class RobotControlInterface {
 public:
  RobotControlInterface(dart::dynamics::SkeletonPtr robot,
                        const char* motor_config_file,
                        const char* interface_config_file);
  void Destroy();
  ~RobotControlInterface() { Destroy(); }

  void Run();

  void MutexLock();

  void MutexUnlock();

 public:
  bool external_timestepping_;
  WorldInterface* world_interface_;
  std::vector<MotorGroup*> motor_groups_;
  std::vector<SensorGroup*> sensor_groups_;

 private:
  struct RobotControlInterfaceParams {
    bool external_timestepping_;
    std::string sim_control_channel_;
    std::string sim_state_channel_;
    int num_motor_groups_;
    std::vector<std::string> motor_group_names_;
    std::vector<std::vector<std::string>> motor_group_joints_;
    std::vector<std::string> motor_group_command_channel_names_;
    std::vector<std::string> motor_group_state_channel_names_;
    int num_sensor_groups_;
    std::vector<std::string> sensor_group_names_;
    std::vector<std::string> sensor_group_state_channel_names_;
  };

  void ReadParams(const char* interface_config_file,
                  RobotControlInterfaceParams* params);
  InterfaceContext interface_context_;
};

#endif  // KRANG_SIMULATION_ROBOT_CONTROL_INTERFACE_H_

